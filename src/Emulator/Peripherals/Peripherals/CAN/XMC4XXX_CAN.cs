//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.CAN
{
    // MultiCAN module of the Infineon XMC4xxx family (XMC4500: 3 nodes, 64 message objects, 8 service request lines),
    // register layout from the XMC4500 Reference Manual v1.6, chapter 18 "Controller Area Network Controller (MultiCAN)".
    //
    // Modelled: module control, list structure with the panel commands, message pending registers, the nodes
    // (INIT/CCE, bit timing used for the frame duration, loop-back mode with the internal CAN bus), message objects
    // with transmit/receive acceptance filtering, remote frames, single data transfer and the interrupt routing to
    // the service request lines SR0-SR7. Not modelled: FIFOs, gateway, error handling (error counters, bus-off),
    // frame counter and analysis modes.
    // Nodes that are not in loop-back mode share the single external bus of this peripheral, which is connected to a
    // CANHub (ICAN). The service request lines are pulsed (an interrupt request is an event, not a level).
    public class XMC4XXX_CAN : IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize, ICAN, INumberedGPIOOutput
    {
        public XMC4XXX_CAN(IMachine machine, uint peripheralClockFrequency = 72000000)
        {
            this.machine = machine;
            this.peripheralClockFrequency = peripheralClockFrequency;
            locker = new object();

            var connections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < ServiceRequestLines; i++)
            {
                connections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(connections);

            lists = new List<int>[ListCount];
            for(var i = 0; i < ListCount; i++)
            {
                lists[i] = new List<int>();
            }
            messagePending = new uint[MessagePendingRegisters];

            nodes = new Node[NodeCount];
            for(var i = 0; i < NodeCount; i++)
            {
                nodes[i] = new Node(i);
            }
            messageObjects = new MessageObject[MessageObjectCount];
            for(var i = 0; i < MessageObjectCount; i++)
            {
                messageObjects[i] = new MessageObject(this, i);
            }

            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
            Reset();
        }

        public void Reset()
        {
            lock(locker)
            {
                RegistersCollection.Reset();
                InitializeLists();
                foreach(var node in nodes)
                {
                    node.Reset();
                }
                foreach(var mo in messageObjects)
                {
                    mo.Reset();
                }
                Array.Clear(messagePending, 0, messagePending.Length);
                messageIndexMask = 0;
                moduleControl = 0;
                panelArgument1 = 0;
                panelArgument2 = 0;
                for(var i = 0; i < ServiceRequestLines; i++)
                {
                    Connections[i].Unset();
                }
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            lock(locker)
            {
                if(offset >= MessageObjectsOffset)
                {
                    var index = (int)((offset - MessageObjectsOffset) / MessageObjectStride);
                    if(index >= MessageObjectCount)
                    {
                        this.Log(LogLevel.Warning, "Read from non-existing message object at offset 0x{0:X}", offset);
                        return 0;
                    }
                    return messageObjects[index].RegistersCollection.Read(offset % MessageObjectStride);
                }
                return RegistersCollection.Read(offset);
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            lock(locker)
            {
                if(offset >= MessageObjectsOffset)
                {
                    var index = (int)((offset - MessageObjectsOffset) / MessageObjectStride);
                    if(index >= MessageObjectCount)
                    {
                        this.Log(LogLevel.Warning, "Write to non-existing message object at offset 0x{0:X}", offset);
                        return;
                    }
                    messageObjects[index].RegistersCollection.Write(offset % MessageObjectStride, value);
                    return;
                }
                RegistersCollection.Write(offset, value);
            }
        }

        // Frame arriving from the external CAN bus (CANHub): delivered to all active nodes that are not in loop-back mode.
        public void OnFrameReceived(CANMessageFrame message)
        {
            lock(locker)
            {
                this.Log(LogLevel.Debug, "Frame received from the bus: {0}", message);
                foreach(var node in nodes.Where(n => n.Active && !n.LoopBack))
                {
                    ReceiveFrame(node, message);
                }
                StartTransmissions();
            }
        }

        public long Size => 0x4000;

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public event Action<CANMessageFrame> FrameSent;

        private void DefineRegisters()
        {
            Registers.ClockControl.Define(this, 0x3)
                .WithFlag(0, out moduleDisable, name: "DISR")
                .WithFlag(1, FieldMode.Read, valueProviderCallback: _ => moduleDisable.Value, name: "DISS")
                .WithReservedBits(2, 1)
                .WithFlag(3, name: "EDIS")
                .WithFlag(4, FieldMode.Write, name: "SBWE")
                .WithReservedBits(5, 27);

            Registers.ModuleIdentification.Define(this, 0x002BC001)
                .WithValueField(0, 32, FieldMode.Read, name: "ID");

            Registers.FractionalDivider.Define(this)
                .WithValueField(0, 10, out fractionalDividerStep, name: "STEP")
                .WithReservedBits(10, 1)
                .WithTaggedFlag("SM", 11)
                .WithTag("SC", 12, 2)
                .WithValueField(14, 2, out fractionalDividerMode, name: "DM")
                .WithTag("RESULT", 16, 10)
                .WithReservedBits(26, 2)
                .WithTaggedFlag("SUSACK", 28)
                .WithTaggedFlag("SUSREQ", 29)
                .WithTaggedFlag("ENHW", 30)
                .WithTaggedFlag("DISCLK", 31);

            for(var i = 0; i < ListCount; i++)
            {
                var list = i;
                (Registers.List0 + 4 * i).Define(this)
                    .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => GetListRegister(list), name: $"LIST{i}");
            }

            for(var i = 0; i < MessagePendingRegisters; i++)
            {
                var k = i;
                (Registers.MessagePending0 + 4 * i).Define(this)
                    .WithValueField(0, 32, valueProviderCallback: _ => messagePending[k], writeCallback: (_, value) =>
                    {
                        // bits written with 0 are cleared, bits written with 1 are left unchanged
                        messagePending[k] &= (uint)value;
                    }, name: $"MSPND{i}");

                (Registers.MessageIndex0 + 4 * i).Define(this, 0x20)
                    .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => GetMessageIndex(k), name: $"MSID{i}");
            }

            Registers.MessageIndexMask.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => messageIndexMask, writeCallback: (_, value) => messageIndexMask = (uint)value, name: "MSIMASK");

            Registers.PanelControl.Define(this)
                .WithValueField(0, 8, valueProviderCallback: _ => 0, name: "PANCMD")
                .WithFlag(8, FieldMode.Read, valueProviderCallback: _ => false, name: "BUSY")
                .WithFlag(9, FieldMode.Read, valueProviderCallback: _ => false, name: "RBUSY")
                .WithReservedBits(10, 6)
                .WithValueField(16, 8, valueProviderCallback: _ => panelArgument1, name: "PANAR1")
                .WithValueField(24, 8, valueProviderCallback: _ => panelArgument2, name: "PANAR2")
                .WithWriteCallback((_, value) => ExecutePanelCommand(value));

            Registers.ModuleControl.Define(this)
                .WithReservedBits(0, 12)
                .WithValueField(12, 4, valueProviderCallback: _ => (moduleControl >> 12) & 0xF,
                    writeCallback: (_, value) => moduleControl = (uint)(value << 12), name: "MPSEL")
                .WithReservedBits(16, 16);

            Registers.ModuleInterruptTrigger.Define(this)
                .WithValueField(0, 8, FieldMode.Write, writeCallback: (_, value) =>
                {
                    for(var i = 0; i < ServiceRequestLines; i++)
                    {
                        if((value & (1UL << i)) != 0)
                        {
                            PulseServiceRequest((uint)i);
                        }
                    }
                }, name: "IT")
                .WithReservedBits(8, 24);

            foreach(var node in nodes)
            {
                DefineNodeRegisters(node);
            }
        }

        private void DefineNodeRegisters(Node node)
        {
            var offset = node.Number * NodeStride;

            (Registers.NodeControl + offset).Define(this, 0x1)
                .WithFlag(0, out node.Init, name: "INIT", changeCallback: (_, value) => OnNodeInitChanged(node, value))
                .WithFlag(1, out node.TransferInterruptEnable, name: "TRIE")
                .WithFlag(2, out node.LastErrorInterruptEnable, name: "LECIE")
                .WithFlag(3, out node.AlertInterruptEnable, name: "ALIE")
                .WithFlag(4, out node.Disable, name: "CANDIS", changeCallback: (_, value) =>
                {
                    // the node leaves the bus (immediately in this model) and INIT is set by hardware
                    if(value)
                    {
                        node.Init.Value = true;
                        OnNodeInitChanged(node, true);
                    }
                })
                .WithReservedBits(5, 1)
                .WithFlag(6, out node.ConfigurationChangeEnable, name: "CCE")
                .WithFlag(7, name: "CALM",
                    valueProviderCallback: _ => node.AnalyzerMode,
                    writeCallback: (_, value) =>
                    {
                        if(node.Init.Value)
                        {
                            node.AnalyzerMode = value;
                        }
                    })
                .WithFlag(8, name: "SUSEN")
                .WithReservedBits(9, 23);

            (Registers.NodeStatus + offset).Define(this)
                .WithValueField(0, 3, name: "LEC",
                    valueProviderCallback: _ => node.LastErrorCode,
                    writeCallback: (_, value) => { if(value == 7) { node.LastErrorCode = 7; } else if(node.LastErrorCode == 7) { node.LastErrorCode = 0; } })
                .WithFlag(3, out node.TransmitOk, FieldMode.Read | FieldMode.WriteZeroToClear, name: "TXOK")
                .WithFlag(4, out node.ReceiveOk, FieldMode.Read | FieldMode.WriteZeroToClear, name: "RXOK")
                .WithFlag(5, out node.Alert, FieldMode.Read | FieldMode.WriteZeroToClear, name: "ALERT")
                .WithFlag(6, FieldMode.Read, valueProviderCallback: _ => false, name: "EWRN")
                .WithFlag(7, FieldMode.Read, valueProviderCallback: _ => false, name: "BOFF")
                .WithFlag(8, out node.ListLengthError, FieldMode.Read | FieldMode.WriteZeroToClear, name: "LLE")
                .WithFlag(9, out node.ListObjectError, FieldMode.Read | FieldMode.WriteZeroToClear, name: "LOE")
                .WithFlag(10, FieldMode.Read, valueProviderCallback: _ => false, name: "SUSACK")
                .WithReservedBits(11, 21);

            (Registers.NodeInterruptPointer + offset).Define(this)
                .WithValueField(0, 4, out node.AlertPointer, name: "ALINP")
                .WithValueField(4, 4, out node.LastErrorPointer, name: "LECINP")
                .WithValueField(8, 4, out node.TransferPointer, name: "TRINP")
                .WithValueField(12, 4, out node.FrameCounterPointer, name: "CFCINP")
                .WithReservedBits(16, 16);

            // NPCR, NBTR and NECNT can only be written while CCE is set
            (Registers.NodePortControl + offset).Define(this)
                .WithValueField(0, 32, name: "NPCR",
                    valueProviderCallback: _ => node.PortControl,
                    writeCallback: (_, value) => { if(node.ConfigurationChangeEnable.Value) { node.PortControl = (uint)value & NodePortControlMask; } });

            (Registers.NodeBitTiming + offset).Define(this)
                .WithValueField(0, 32, name: "NBTR",
                    valueProviderCallback: _ => node.BitTiming,
                    writeCallback: (_, value) => { if(node.ConfigurationChangeEnable.Value) { node.BitTiming = (uint)value & NodeBitTimingMask; } });

            (Registers.NodeErrorCounter + offset).Define(this)
                .WithValueField(0, 32, name: "NECNT",
                    valueProviderCallback: _ => node.ErrorCounter,
                    writeCallback: (_, value) => { if(node.ConfigurationChangeEnable.Value) { node.ErrorCounter = (uint)value; } });

            (Registers.NodeFrameCounter + offset).Define(this)
                .WithValueField(0, 32, name: "NFCR",
                    valueProviderCallback: _ => node.FrameCounter,
                    writeCallback: (_, value) => node.FrameCounter = (uint)value);
        }

        private void OnNodeInitChanged(Node node, bool init)
        {
            if(init)
            {
                // an ongoing transfer is cancelled
                node.TransmitToken++;
                node.TransmitBusy = false;
            }
            else
            {
                StartTransmissions();
            }
        }

        private void InitializeLists()
        {
            foreach(var list in lists)
            {
                list.Clear();
            }
            for(var i = 0; i < MessageObjectCount; i++)
            {
                lists[0].Add(i);
                messageObjects[i].List = 0;
            }
        }

        private uint GetListRegister(int list)
        {
            var objects = lists[list];
            if(objects.Count == 0)
            {
                return 1u << 24;
            }
            return (uint)(objects[0] | (objects[objects.Count - 1] << 8) | ((objects.Count - 1) << 16));
        }

        private uint GetMessageIndex(int register)
        {
            var pending = messagePending[register] & messageIndexMask;
            if(pending == 0)
            {
                return 0x20;
            }
            for(var i = 0; i < 32; i++)
            {
                if((pending & (1u << i)) != 0)
                {
                    return (uint)i;
                }
            }
            return 0x20;
        }

        private void GetNeighbours(MessageObject mo, out int previous, out int next)
        {
            var list = lists[mo.List];
            var index = list.IndexOf(mo.Number);
            previous = index > 0 ? list[index - 1] : mo.Number;
            next = index >= 0 && index < list.Count - 1 ? list[index + 1] : mo.Number;
        }

        private void ExecutePanelCommand(uint value)
        {
            var command = value & 0xFF;
            var argument1 = (value >> 16) & 0xFF;
            var argument2 = (value >> 24) & 0xFF;
            panelArgument1 = argument1;
            panelArgument2 = argument2;
            const uint error = 0x80;

            switch(command)
            {
            case PanelCommandNoOperation:
                break;
            case PanelCommandInitializeLists:
                if(nodes.All(n => n.Init.Value && n.ConfigurationChangeEnable.Value))
                {
                    InitializeLists();
                    panelArgument2 = 0;
                }
                else
                {
                    panelArgument2 = error;
                }
                break;
            case PanelCommandStaticAllocate:
                if(argument1 < MessageObjectCount && argument2 < ListCount)
                {
                    MoveToList(messageObjects[argument1], (int)argument2);
                }
                break;
            case PanelCommandDynamicAllocate:
                if(argument2 < ListCount && lists[0].Count > 0)
                {
                    var mo = messageObjects[lists[0][0]];
                    MoveToList(mo, (int)argument2);
                    panelArgument1 = (uint)mo.Number;
                    panelArgument2 = 0;
                }
                else
                {
                    panelArgument2 = error;
                }
                break;
            case PanelCommandStaticInsertBefore:
            case PanelCommandStaticInsertBehind:
                if(argument1 < MessageObjectCount && argument2 < MessageObjectCount)
                {
                    InsertRelative(messageObjects[argument1], messageObjects[argument2], command == PanelCommandStaticInsertBehind);
                }
                break;
            case PanelCommandDynamicInsertBefore:
            case PanelCommandDynamicInsertBehind:
                if(argument2 < MessageObjectCount && lists[0].Count > 0)
                {
                    var mo = messageObjects[lists[0][0]];
                    InsertRelative(mo, messageObjects[argument2], command == PanelCommandDynamicInsertBehind);
                    panelArgument1 = (uint)mo.Number;
                    panelArgument2 = 0;
                }
                else
                {
                    panelArgument2 = error;
                }
                break;
            default:
                this.Log(LogLevel.Warning, "Unknown panel command 0x{0:X}", command);
                break;
            }
        }

        private void MoveToList(MessageObject mo, int list)
        {
            lists[mo.List].Remove(mo.Number);
            lists[list].Add(mo.Number);
            mo.List = list;
        }

        private void InsertRelative(MessageObject source, MessageObject destination, bool behind)
        {
            if(source == destination)
            {
                return;
            }
            lists[source.List].Remove(source.Number);
            var list = lists[destination.List];
            var index = list.IndexOf(destination.Number);
            list.Insert(behind ? index + 1 : index, source.Number);
            source.List = destination.List;
        }

        private void PulseServiceRequest(uint line)
        {
            if(line >= ServiceRequestLines)
            {
                this.Log(LogLevel.Debug, "Interrupt pointer {0} does not select a service request line", line);
                return;
            }
            this.Log(LogLevel.Noisy, "Service request {0}", line);
            Connections[(int)line].Set(true);
            Connections[(int)line].Set(false);
        }

        private void SetMessagePending(MessageObject mo, bool receive)
        {
            uint register;
            int bit;
            if(((moduleControl >> 12) & 0xF) == 0xF)
            {
                var pointer = receive ? mo.ReceivePointer : mo.TransmitPointer;
                register = (pointer >> 1) & 0x7;
                bit = (int)(((pointer & 1) << 4) | (mo.PendingNumber & 0xF));
            }
            else
            {
                register = (mo.PendingNumber >> 5) & 0x7;
                bit = (int)(mo.PendingNumber & 0x1F);
            }
            messagePending[register] |= 1u << bit;
        }

        private void ApplyMessageObjectControl(MessageObject mo, uint value)
        {
            for(var bit = 0; bit < MessageObjectControlBits; bit++)
            {
                var reset = (value & (1u << bit)) != 0;
                var set = (value & (1u << (bit + 16))) != 0;
                if(reset && set)
                {
                    mo.Status ^= 1u << bit;
                }
                else if(reset)
                {
                    mo.Status &= ~(1u << bit);
                }
                else if(set)
                {
                    mo.Status |= 1u << bit;
                }
            }
            StartTransmissions();
        }

        private void StartTransmissions()
        {
            foreach(var node in nodes)
            {
                TryStartTransmission(node);
            }
        }

        private void TryStartTransmission(Node node)
        {
            if(node.TransmitBusy || !node.Active || node.AnalyzerMode)
            {
                return;
            }

            MessageObject selected = null;
            foreach(var number in lists[node.Number + 1])
            {
                var mo = messageObjects[number];
                if(!mo.HasAll(MessageObject.MessageValid | MessageObject.TransmitRequest | MessageObject.TransmitEnable0 | MessageObject.TransmitEnable1))
                {
                    continue;
                }
                if(selected == null || IsHigherTransmitPriority(mo, selected))
                {
                    selected = mo;
                }
            }
            if(selected == null)
            {
                return;
            }

            var frame = CreateFrame(selected);
            // NEWDAT is cleared when the transmission starts; if software sets it again, TXRQ stays set after the transfer
            selected.Status &= ~MessageObject.NewData;
            node.TransmitBusy = true;
            var token = ++node.TransmitToken;
            var duration = GetFrameDuration(node, frame);
            this.Log(LogLevel.Debug, "Node {0}: transmitting message object {1}: {2}", node.Number, selected.Number, frame);
            machine.ScheduleAction(duration, _ =>
            {
                lock(locker)
                {
                    if(token == node.TransmitToken)
                    {
                        FinishTransmission(node, selected, frame);
                    }
                }
            });
        }

        private static bool IsHigherTransmitPriority(MessageObject candidate, MessageObject current)
        {
            // the objects are visited in list order, so equal priority classes keep the earlier object
            if(candidate.Priority != current.Priority)
            {
                return candidate.Priority < current.Priority;
            }
            if(candidate.Priority == 2)
            {
                return candidate.ArbitrationValue < current.ArbitrationValue;
            }
            return false;
        }

        private CANMessageFrame CreateFrame(MessageObject mo)
        {
            var remote = !mo.Has(MessageObject.Direction);
            var length = (int)Math.Min(mo.DataLengthCode, 8u);
            var data = new byte[length];
            var raw = BitConverter.GetBytes(((ulong)mo.DataHigh << 32) | mo.DataLow);
            if(!remote)
            {
                Array.Copy(raw, data, length);
            }
            return mo.ExtendedIdentifier
                ? new CANMessageFrame(mo.Identifier, data, extendedFormat: true, remoteFrame: remote)
                : new CANMessageFrame(mo.Identifier >> StandardIdentifierShift, data, extendedFormat: false, remoteFrame: remote);
        }

        private TimeInterval GetFrameDuration(Node node, CANMessageFrame frame)
        {
            // fCAN follows the fractional divider; the bit time follows the node bit timing register
            double canFrequency = peripheralClockFrequency;
            var step = fractionalDividerStep.Value;
            if(fractionalDividerMode.Value == 1)
            {
                canFrequency = peripheralClockFrequency / (double)(1024 - step);
            }
            else if(fractionalDividerMode.Value == 2 && step != 0)
            {
                canFrequency = peripheralClockFrequency * step / 1024.0;
            }
            var timing = node.BitTiming;
            var prescaler = (timing & 0x3F) + 1;
            if((timing & (1u << 15)) != 0)
            {
                prescaler *= 8;
            }
            var quanta = 1 + (((timing >> 8) & 0xF) + 1) + (((timing >> 12) & 0x7) + 1);
            var bitTime = prescaler * quanta / canFrequency;
            var bits = (frame.ExtendedFormat ? 67 : 47) + (frame.RemoteFrame ? 0 : 8 * frame.Data.Length);
            var microseconds = Math.Max(1.0, bits * bitTime * 1e6);
            return TimeInterval.FromMicroseconds((ulong)microseconds);
        }

        private void FinishTransmission(Node node, MessageObject mo, CANMessageFrame frame)
        {
            node.TransmitBusy = false;
            this.Log(LogLevel.Debug, "Node {0}: message object {1} transmitted", node.Number, mo.Number);

            if(node.LoopBack)
            {
                // the internal CAN bus connects all nodes in loop-back mode
                foreach(var other in nodes.Where(n => n != node && n.Active && n.LoopBack))
                {
                    ReceiveFrame(other, frame);
                }
            }
            else
            {
                FrameSent?.Invoke(frame);
            }

            mo.Status |= MessageObject.TransmitPending;
            SetMessagePending(mo, false);
            if(!mo.Has(MessageObject.NewData))
            {
                mo.Status &= ~MessageObject.TransmitRequest;
            }
            if(mo.SingleDataTransfer && !frame.RemoteFrame)
            {
                mo.Status &= ~MessageObject.MessageValid;
            }
            if(mo.TransmitInterruptEnable)
            {
                PulseServiceRequest(mo.TransmitPointer);
            }

            node.TransmitOk.Value = true;
            if(node.TransferInterruptEnable.Value)
            {
                PulseServiceRequest((uint)node.TransferPointer.Value);
            }
            StartTransmissions();
        }

        private void ReceiveFrame(Node node, CANMessageFrame frame)
        {
            var mo = FindReceiveObject(node, frame);
            node.ReceiveOk.Value = true;
            if(mo == null)
            {
                this.Log(LogLevel.Debug, "Node {0}: no message object accepts {1}", node.Number, frame);
            }
            else if(frame.RemoteFrame)
            {
                // a remote frame requests the data frame of a matching transmit object
                this.Log(LogLevel.Debug, "Node {0}: remote request for message object {1}", node.Number, mo.Number);
                mo.Status |= MessageObject.TransmitRequest;
            }
            else
            {
                StoreFrame(mo, frame);
            }
            if(node.TransferInterruptEnable.Value)
            {
                PulseServiceRequest((uint)node.TransferPointer.Value);
            }
        }

        private MessageObject FindReceiveObject(Node node, CANMessageFrame frame)
        {
            var identifier = frame.ExtendedFormat ? frame.Id : frame.Id << StandardIdentifierShift;
            MessageObject best = null;
            foreach(var number in lists[node.Number + 1])
            {
                var mo = messageObjects[number];
                if(!mo.HasAll(MessageObject.MessageValid | MessageObject.ReceiveEnable))
                {
                    continue;
                }
                // DIR = 0 accepts data frames only, DIR = 1 remote frames only
                if(mo.Has(MessageObject.Direction) != frame.RemoteFrame)
                {
                    continue;
                }
                if(mo.MatchIdentifierExtension && mo.ExtendedIdentifier != frame.ExtendedFormat)
                {
                    continue;
                }
                if(((identifier ^ mo.Identifier) & mo.AcceptanceMask) != 0)
                {
                    continue;
                }
                if(best == null || mo.Priority < best.Priority)
                {
                    best = mo;
                }
            }
            return best;
        }

        private void StoreFrame(MessageObject mo, CANMessageFrame frame)
        {
            this.Log(LogLevel.Debug, "Message object {0} receives {1}", mo.Number, frame);
            mo.StoreFrame(frame, frame.ExtendedFormat ? frame.Id : frame.Id << StandardIdentifierShift);
            if(mo.Has(MessageObject.NewData))
            {
                mo.Status |= MessageObject.MessageLost;
            }
            mo.Status |= MessageObject.NewData | MessageObject.ReceivePending;
            if(mo.SingleDataTransfer)
            {
                mo.Status &= ~MessageObject.MessageValid;
            }
            SetMessagePending(mo, true);
            if(mo.ReceiveInterruptEnable)
            {
                PulseServiceRequest(mo.ReceivePointer);
            }
        }

        private IFlagRegisterField moduleDisable;
        private IValueRegisterField fractionalDividerStep;
        private IValueRegisterField fractionalDividerMode;
        private uint moduleControl;
        private uint messageIndexMask;
        private uint panelArgument1;
        private uint panelArgument2;

        private readonly IMachine machine;
        private readonly uint peripheralClockFrequency;
        private readonly object locker;
        private readonly List<int>[] lists;
        private readonly uint[] messagePending;
        private readonly Node[] nodes;
        private readonly MessageObject[] messageObjects;

        private const int ServiceRequestLines = 8;
        private const int NodeCount = 3;
        private const int MessageObjectCount = 64;
        private const int ListCount = 8;
        private const int MessagePendingRegisters = 8;
        private const int NodeStride = 0x100;
        private const long MessageObjectsOffset = 0x1000;
        private const long MessageObjectStride = 0x20;
        private const int MessageObjectControlBits = 12;
        private const int StandardIdentifierShift = 18;
        private const uint NodePortControlMask = 0x107;
        private const uint NodeBitTimingMask = 0xFFFF;

        private const uint PanelCommandNoOperation = 0x0;
        private const uint PanelCommandInitializeLists = 0x1;
        private const uint PanelCommandStaticAllocate = 0x2;
        private const uint PanelCommandDynamicAllocate = 0x3;
        private const uint PanelCommandStaticInsertBefore = 0x4;
        private const uint PanelCommandDynamicInsertBefore = 0x5;
        private const uint PanelCommandStaticInsertBehind = 0x6;
        private const uint PanelCommandDynamicInsertBehind = 0x7;

        private class Node
        {
            public Node(int number)
            {
                Number = number;
            }

            public void Reset()
            {
                PortControl = 0;
                BitTiming = 0;
                ErrorCounter = 0x00600000;
                FrameCounter = 0;
                LastErrorCode = 0;
                AnalyzerMode = false;
                TransmitBusy = false;
                TransmitToken++;
            }

            public int Number { get; }

            public bool Active => !Init.Value;

            public bool LoopBack => (PortControl & (1u << 8)) != 0;

            public uint PortControl;
            public uint BitTiming;
            public uint ErrorCounter;
            public uint FrameCounter;
            public uint LastErrorCode;
            public bool AnalyzerMode;
            public bool TransmitBusy;
            public int TransmitToken;

            public IFlagRegisterField Init;
            public IFlagRegisterField TransferInterruptEnable;
            public IFlagRegisterField LastErrorInterruptEnable;
            public IFlagRegisterField AlertInterruptEnable;
            public IFlagRegisterField Disable;
            public IFlagRegisterField ConfigurationChangeEnable;
            public IFlagRegisterField TransmitOk;
            public IFlagRegisterField ReceiveOk;
            public IFlagRegisterField Alert;
            public IFlagRegisterField ListLengthError;
            public IFlagRegisterField ListObjectError;
            public IValueRegisterField AlertPointer;
            public IValueRegisterField LastErrorPointer;
            public IValueRegisterField TransferPointer;
            public IValueRegisterField FrameCounterPointer;
        }

        private class MessageObject : IProvidesRegisterCollection<DoubleWordRegisterCollection>
        {
            public MessageObject(XMC4XXX_CAN parent, int number)
            {
                this.parent = parent;
                Number = number;
                RegistersCollection = new DoubleWordRegisterCollection(parent);
                DefineRegisters();
            }

            public void Reset()
            {
                RegistersCollection.Reset();
                FunctionControl = 0;
                FifoGatewayPointer = 0;
                InterruptPointer = 0;
                AcceptanceMaskRegister = 0x3FFFFFFF;
                DataLow = 0;
                DataHigh = 0;
                Arbitration = 0;
                Status = 0;
            }

            public void StoreFrame(CANMessageFrame frame, uint identifier)
            {
                Arbitration = (Arbitration & ~IdentifierMask) | (identifier & IdentifierMask);
                if(!MatchIdentifierExtension)
                {
                    Arbitration = frame.ExtendedFormat ? Arbitration | IdentifierExtensionBit : Arbitration & ~IdentifierExtensionBit;
                }
                var data = new byte[8];
                Array.Copy(frame.Data, data, Math.Min(frame.Data.Length, 8));
                DataLow = BitConverter.ToUInt32(data, 0);
                DataHigh = BitConverter.ToUInt32(data, 4);
                FunctionControl = (FunctionControl & ~(0xFu << 24)) | ((uint)frame.Data.Length << 24);
            }

            public bool Has(uint flag)
            {
                return (Status & flag) != 0;
            }

            public bool HasAll(uint flags)
            {
                return (Status & flags) == flags;
            }

            public int Number { get; }

            public int List { get; set; }

            public DoubleWordRegisterCollection RegistersCollection { get; }

            public uint Status;
            public uint FunctionControl;
            public uint FifoGatewayPointer;
            public uint InterruptPointer;
            public uint AcceptanceMaskRegister;
            public uint DataLow;
            public uint DataHigh;
            public uint Arbitration;

            public bool ReceiveInterruptEnable => (FunctionControl & (1u << 16)) != 0;

            public bool TransmitInterruptEnable => (FunctionControl & (1u << 17)) != 0;

            public bool SingleDataTransfer => (FunctionControl & (1u << 22)) != 0;

            public uint DataLengthCode => (FunctionControl >> 24) & 0xF;

            public uint ReceivePointer => InterruptPointer & 0xF;

            public uint TransmitPointer => (InterruptPointer >> 4) & 0xF;

            public uint PendingNumber => (InterruptPointer >> 8) & 0xFF;

            public uint Identifier => Arbitration & IdentifierMask;

            public bool ExtendedIdentifier => (Arbitration & IdentifierExtensionBit) != 0;

            public uint Priority => Arbitration >> 30;

            // the arbitration value used for PRI = 10b: identifier compared as 29 bit value, standard before extended
            public uint ArbitrationValue => Identifier;

            public uint AcceptanceMask => AcceptanceMaskRegister & IdentifierMask;

            public bool MatchIdentifierExtension => (AcceptanceMaskRegister & IdentifierExtensionBit) != 0;

            public const uint ReceivePending = 1u << 0;
            public const uint TransmitPending = 1u << 1;
            public const uint ReceiveUpdating = 1u << 2;
            public const uint NewData = 1u << 3;
            public const uint MessageLost = 1u << 4;
            public const uint MessageValid = 1u << 5;
            public const uint ReceiveTransmitSelected = 1u << 6;
            public const uint ReceiveEnable = 1u << 7;
            public const uint TransmitRequest = 1u << 8;
            public const uint TransmitEnable0 = 1u << 9;
            public const uint TransmitEnable1 = 1u << 10;
            public const uint Direction = 1u << 11;

            private void DefineRegisters()
            {
                MessageObjectRegisters.FunctionControl.Define(this)
                    .WithValueField(0, 32, name: "MOFCR",
                        valueProviderCallback: _ => FunctionControl,
                        writeCallback: (_, value) => FunctionControl = (uint)value);

                MessageObjectRegisters.FifoGatewayPointer.Define(this)
                    .WithValueField(0, 32, name: "MOFGPR",
                        valueProviderCallback: _ => FifoGatewayPointer,
                        writeCallback: (_, value) => FifoGatewayPointer = (uint)value);

                MessageObjectRegisters.InterruptPointer.Define(this)
                    .WithValueField(0, 32, name: "MOIPR",
                        valueProviderCallback: _ => InterruptPointer,
                        writeCallback: (_, value) => InterruptPointer = (uint)value);

                MessageObjectRegisters.AcceptanceMask.Define(this, 0x3FFFFFFF)
                    .WithValueField(0, 32, name: "MOAMR",
                        valueProviderCallback: _ => AcceptanceMaskRegister,
                        writeCallback: (_, value) => AcceptanceMaskRegister = (uint)value & 0x3FFFFFFF);

                MessageObjectRegisters.DataLow.Define(this)
                    .WithValueField(0, 32, name: "MODATAL",
                        valueProviderCallback: _ => DataLow,
                        writeCallback: (_, value) => DataLow = (uint)value);

                MessageObjectRegisters.DataHigh.Define(this)
                    .WithValueField(0, 32, name: "MODATAH",
                        valueProviderCallback: _ => DataHigh,
                        writeCallback: (_, value) => DataHigh = (uint)value);

                MessageObjectRegisters.Arbitration.Define(this)
                    .WithValueField(0, 32, name: "MOAR",
                        valueProviderCallback: _ => Arbitration,
                        writeCallback: (_, value) => Arbitration = (uint)value);

                // MOSTAT when read, MOCTR (set/reset bits) when written
                MessageObjectRegisters.StatusControl.Define(this)
                    .WithValueField(0, 32, name: "MOSTAT_MOCTR",
                        valueProviderCallback: _ => BuildStatus(),
                        writeCallback: (_, value) => parent.ApplyMessageObjectControl(this, (uint)value));
            }

            private uint BuildStatus()
            {
                parent.GetNeighbours(this, out var previous, out var next);
                return (Status & 0xFFF) | ((uint)List << 12) | ((uint)previous << 16) | ((uint)next << 24);
            }

            private readonly XMC4XXX_CAN parent;

            private const uint IdentifierMask = 0x1FFFFFFF;
            private const uint IdentifierExtensionBit = 1u << 29;

            private enum MessageObjectRegisters : long
            {
                FunctionControl = 0x00,
                FifoGatewayPointer = 0x04,
                InterruptPointer = 0x08,
                AcceptanceMask = 0x0C,
                DataLow = 0x10,
                DataHigh = 0x14,
                Arbitration = 0x18,
                StatusControl = 0x1C,
            }
        }

        private enum Registers : long
        {
            ClockControl = 0x000,
            ModuleIdentification = 0x008,
            FractionalDivider = 0x00C,
            List0 = 0x100,
            MessagePending0 = 0x140,
            MessageIndex0 = 0x180,
            MessageIndexMask = 0x1C0,
            PanelControl = 0x1C4,
            ModuleControl = 0x1C8,
            ModuleInterruptTrigger = 0x1CC,
            // node registers of node 0, node n is at +n*0x100
            NodeControl = 0x200,
            NodeStatus = 0x204,
            NodeInterruptPointer = 0x208,
            NodePortControl = 0x20C,
            NodeBitTiming = 0x210,
            NodeErrorCounter = 0x214,
            NodeFrameCounter = 0x218,
        }
    }
}
