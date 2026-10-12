//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.DMA
{
    // General Purpose DMA unit (GPDMA0 with 8 channels at 0x50014000, GPDMA1 with 4 channels at 0x50018000) of the Infineon XMC4500,
    // Reference Manual v1.6, chapter 5.
    //
    // Modeled: all transfer types (memory to memory, memory to peripheral, peripheral to memory, peripheral to peripheral) with the GPDMA as
    // flow controller; transfer widths 8/16/32 bit with a channel FIFO (32 bytes for channels 0 and 1 of GPDMA0, 8 bytes otherwise);
    // address increment/decrement/no change, source gather and destination scatter (channels 0 and 1 of GPDMA0); multi-block transfers by
    // block chaining with linked lists (including the write-back of CTLH), auto-reload and contiguous addresses (channels 0 and 1 of GPDMA0,
    // Table 5-5), the stall between blocks of an auto-reload transfer while the block interrupt is unmasked; hardware handshaking through the
    // DMA line router (one request is one transaction of a burst length of source/destination MSIZE items, shortened at the end of the block)
    // and software handshaking through REQSRCREG/REQDSTREG/SGLREQSRCREG/SGLREQDSTREG; the five interrupt types (transfer, block, source
    // transaction, destination transaction, error) with raw status, mask (with write enables), clear and the combined status; priorities.
    // The error interrupt is raised when a transfer addresses nothing on the bus; the channel is then disabled.
    // Not modeled: peripheral flow control (a warning is logged, the GPDMA acts as flow controller), the last transaction registers, status
    // fetch (SSTAT/DSTAT write-back), FCMODE/FIFO_MODE/MAX_ABRST/locking/handshaking polarity (stored only), disabling a channel in the middle of
    // a transfer without data loss, bus timing (see transferFrequency).
    //
    // Timing: the engine performs one bus transfer (a read from the source or a write to the destination) per tick of a timer with
    // transferFrequency; the channel with the highest CH_PRIOR (lowest number on a tie) that can proceed is served. The timer only runs while
    // a channel can proceed, so a channel waiting for a request costs nothing.
    //
    // The service request line (IRQ) is the combined interrupt of the unit (NVIC 105 for GPDMA0, 110 for GPDMA1).
    // Requests: the unit is attached to the DLR (dlr, firstDlrLine = 0 for GPDMA0, 8 for GPDMA1); a request that is serviced by all channels
    // waiting for the line is acknowledged to the DLR when the last of them starts its transaction.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class XMC4XXX_GPDMA : IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize
    {
        public XMC4XXX_GPDMA(IMachine machine, XMC4XXX_DLR dlr = null, int firstDlrLine = 0, int numberOfChannels = 8, long transferFrequency = 10000000)
        {
            this.machine = machine;
            this.dlr = dlr;
            this.firstDlrLine = firstDlrLine;
            this.numberOfChannels = numberOfChannels;
            this.hasExtendedChannels = firstDlrLine == 0;
            bus = machine.GetSystemBus(this);

            IRQ = new GPIO();
            RegistersCollection = new DoubleWordRegisterCollection(this);

            channels = new Channel[numberOfChannels];
            for(var i = 0; i < numberOfChannels; i++)
            {
                channels[i] = new Channel(this, i);
            }
            raw = new uint[EventCount];
            mask = new uint[EventCount];
            servedMask = new uint[numberOfChannels];
            softRequestSource = new uint[3];
            softRequestDestination = new uint[3];

            engineTimer = new LimitTimer(machine.ClockSource, (ulong)transferFrequency, this, "engine", limit: 1, workMode: WorkMode.Periodic, eventEnabled: true);
            engineTimer.LimitReached += Step;

            DefineRegisters();
            dlr?.AttachDma(firstDlrLine, numberOfChannels, OnLineRequest, OnLineReset);
        }

        public void Reset()
        {
            engineTimer.Enabled = false;
            RegistersCollection.Reset();
            Array.Clear(raw, 0, raw.Length);
            Array.Clear(mask, 0, mask.Length);
            Array.Clear(servedMask, 0, servedMask.Length);
            Array.Clear(softRequestSource, 0, softRequestSource.Length);
            Array.Clear(softRequestDestination, 0, softRequestDestination.Length);
            dmaEnabled = false;
            foreach(var channel in channels)
            {
                channel.Reset();
            }
            UpdateInterrupts();
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            RegistersCollection.Write(offset, value);
        }

        public long Size => 0x4000;

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public GPIO IRQ { get; }

        private void DefineRegisters()
        {
            for(var i = 0; i < numberOfChannels; i++)
            {
                channels[i].DefineRegisters(RegistersCollection);
            }

            for(var kind = 0; kind < EventCount; kind++)
            {
                var type = kind;
                var name = EventNames[kind];
                // RAWxxx (the registers can be written for tests), STATUSxxx, MASKxxx (write enables in the upper byte), CLEARxxx
                RegistersCollection.DefineRegister((long)Registers.RawTransfer + 8 * type)
                    .WithValueField(0, 32, name: $"RAW{name}",
                        valueProviderCallback: _ => raw[type],
                        writeCallback: (_, value) =>
                        {
                            raw[type] = (uint)value & ChannelMask;
                            UpdateInterrupts();
                        });
                RegistersCollection.DefineRegister((long)Registers.StatusTransfer + 8 * type)
                    .WithValueField(0, 32, FieldMode.Read, name: $"STATUS{name}", valueProviderCallback: _ => Status(type));
                RegistersCollection.DefineRegister((long)Registers.MaskTransfer + 8 * type)
                    .WithValueField(0, 32, name: $"MASK{name}",
                        valueProviderCallback: _ => mask[type],
                        writeCallback: (_, value) =>
                        {
                            for(var ch = 0; ch < numberOfChannels; ch++)
                            {
                                if((value & (1UL << (8 + ch))) != 0)
                                {
                                    mask[type] = (mask[type] & ~(1u << ch)) | (((uint)value >> ch) & 1u) << ch;
                                }
                            }
                            UpdateInterrupts();
                        });
                RegistersCollection.DefineRegister((long)Registers.ClearTransfer + 8 * type)
                    .WithValueField(0, 32, FieldMode.Write, name: $"CLEAR{name}", writeCallback: (_, value) =>
                    {
                        raw[type] &= ~(uint)value;
                        UpdateInterrupts();
                        if(type == BlockEvent)
                        {
                            for(var ch = 0; ch < numberOfChannels; ch++)
                            {
                                if((value & (1UL << ch)) != 0)
                                {
                                    channels[ch].BlockInterruptCleared();
                                }
                            }
                        }
                    });
            }

            Registers.StatusInterrupt.Define(this)
                .WithFlags(0, EventCount, FieldMode.Read, name: "STATUSINT", valueProviderCallback: (type, _) => Status(type) != 0)
                .WithReservedBits(EventCount, 32 - EventCount);

            DefineSoftwareRequest(Registers.RequestSource, softRequestSource, 0, "REQSRCREG");
            DefineSoftwareRequest(Registers.RequestDestination, softRequestDestination, 0, "REQDSTREG");
            DefineSoftwareRequest(Registers.SingleRequestSource, softRequestSource, 1, "SGLREQSRCREG");
            DefineSoftwareRequest(Registers.SingleRequestDestination, softRequestDestination, 1, "SGLREQDSTREG");
            DefineSoftwareRequest(Registers.LastSource, softRequestSource, 2, "LSTSRCREG");
            DefineSoftwareRequest(Registers.LastDestination, softRequestDestination, 2, "LSTDSTREG");

            Registers.Configuration.Define(this)
                .WithFlag(0, name: "DMA_EN",
                    valueProviderCallback: _ => dmaEnabled,
                    writeCallback: (_, value) =>
                    {
                        dmaEnabled = value;
                        if(!value)
                        {
                            foreach(var channel in channels)
                            {
                                channel.Disable();
                            }
                        }
                        Kick();
                    })
                .WithReservedBits(1, 31);

            Registers.ChannelEnable.Define(this)
                .WithValueField(0, 32, name: "CHENREG",
                    valueProviderCallback: _ =>
                    {
                        var value = 0u;
                        if(dmaEnabled)
                        {
                            for(var ch = 0; ch < numberOfChannels; ch++)
                            {
                                value |= channels[ch].Enabled ? 1u << ch : 0;
                            }
                        }
                        return value;
                    },
                    writeCallback: (_, value) =>
                    {
                        if(!dmaEnabled)
                        {
                            this.Log(LogLevel.Warning, "CHENREG written while the GPDMA is disabled, ignored");
                            return;
                        }
                        for(var ch = 0; ch < numberOfChannels; ch++)
                        {
                            if((value & (1UL << (8 + ch))) == 0)
                            {
                                continue;
                            }
                            if((value & (1UL << ch)) != 0)
                            {
                                channels[ch].Enable();
                            }
                            else
                            {
                                channels[ch].Disable();
                            }
                        }
                        Kick();
                    });

            Registers.Id.Define(this, firstDlrLine == 0 ? 0x00AFC001u : 0x00B0C001u)
                .WithValueField(0, 32, FieldMode.Read, name: "ID");
            Registers.Type.Define(this, 0x44571110)
                .WithValueField(0, 32, FieldMode.Read, name: "TYPE");
            Registers.Version.Define(this, 0x3231342A)
                .WithValueField(0, 32, FieldMode.Read, name: "VERSION");
        }

        // The software handshaking registers: bit x is the value of channel x, bit 8 + x its write enable. rows: 0 = request, 1 = single, 2 = last
        private void DefineSoftwareRequest(Registers register, uint[] storage, int row, string name)
        {
            register.Define(this)
                .WithValueField(0, 32, name: name,
                    valueProviderCallback: _ => storage[row],
                    writeCallback: (_, value) =>
                    {
                        for(var ch = 0; ch < numberOfChannels; ch++)
                        {
                            if((value & (1UL << (8 + ch))) != 0)
                            {
                                storage[row] = (storage[row] & ~(1u << ch)) | (((uint)value >> ch) & 1u) << ch;
                            }
                        }
                        Kick();
                    });
        }

        private uint Status(int type)
        {
            var interruptEnabled = 0u;
            for(var ch = 0; ch < numberOfChannels; ch++)
            {
                interruptEnabled |= channels[ch].InterruptEnable ? 1u << ch : 0;
            }
            return raw[type] & mask[type] & interruptEnabled;
        }

        private void UpdateInterrupts()
        {
            var active = false;
            for(var type = 0; type < EventCount; type++)
            {
                active |= Status(type) != 0;
            }
            IRQ.Set(active);
        }

        private void RaiseEvent(int type, int channel)
        {
            raw[type] |= 1u << channel;
            UpdateInterrupts();
        }

        // Wakes the engine up: a channel might be able to proceed
        private void Kick()
        {
            if(!engineTimer.Enabled)
            {
                engineTimer.Enabled = true;
            }
        }

        private void Step()
        {
            Channel best = null;
            foreach(var channel in channels)
            {
                if(!channel.CanProceed())
                {
                    continue;
                }
                if(best == null || channel.Priority > best.Priority)
                {
                    best = channel;
                }
            }
            if(best == null)
            {
                engineTimer.Enabled = false;
                return;
            }
            best.Proceed();
        }

        private void OnLineRequest(int line)
        {
            Kick();
        }

        private void OnLineReset(int line)
        {
            for(var ch = 0; ch < numberOfChannels; ch++)
            {
                servedMask[ch] &= ~(1u << line);
            }
        }

        // The request of a DLR line (unit relative number) can be taken by the channel when it is pending and the channel has not served it yet
        private bool IsRequestAvailable(int line, int channel)
        {
            return dlr != null && dlr.IsRequestPending(firstDlrLine + line) && (servedMask[channel] & (1u << line)) == 0;
        }

        private void TakeRequest(int line, int channel)
        {
            servedMask[channel] |= 1u << line;
            AcknowledgeIfServed(line);
        }

        // The request is acknowledged to the DLR when every running channel that waits for the line has served it
        private void AcknowledgeIfServed(int line)
        {
            var waiting = 0u;
            var served = 0u;
            for(var ch = 0; ch < numberOfChannels; ch++)
            {
                if(channels[ch].UsesLine(line))
                {
                    waiting |= 1u << ch;
                }
                if((servedMask[ch] & (1u << line)) != 0)
                {
                    served |= 1u << ch;
                }
            }
            if(waiting == 0 || (waiting & ~served) != 0)
            {
                return;
            }
            dlr.Acknowledge(firstDlrLine + line);
            for(var ch = 0; ch < numberOfChannels; ch++)
            {
                servedMask[ch] &= ~(1u << line);
            }
        }

        private readonly IMachine machine;
        private readonly IBusController bus;
        private readonly XMC4XXX_DLR dlr;
        private readonly int firstDlrLine;
        private readonly int numberOfChannels;
        private readonly bool hasExtendedChannels;
        private readonly Channel[] channels;
        private readonly LimitTimer engineTimer;
        // raw status and mask of the five event types (a bit per channel)
        private readonly uint[] raw;
        private readonly uint[] mask;
        // per channel: the lines whose current request it has serviced
        private readonly uint[] servedMask;
        // software handshaking registers [request, single, last][channel bit]
        private readonly uint[] softRequestSource;
        private readonly uint[] softRequestDestination;
        private bool dmaEnabled;

        private const int EventCount = 5;
        private const int TransferEvent = 0;
        private const int BlockEvent = 1;
        private const int SourceTransactionEvent = 2;
        private const int DestinationTransactionEvent = 3;
        private const int ErrorEvent = 4;
        private const uint ChannelMask = 0xFF;
        private static readonly string[] EventNames = { "TFR", "BLOCK", "SRCTRAN", "DSTTRAN", "ERR" };

        private class Channel
        {
            public Channel(XMC4XXX_GPDMA unit, int index)
            {
                this.unit = unit;
                this.index = index;
                // channels 0 and 1 of GPDMA0 have a 32 byte FIFO, a linked list pointer, gather/scatter and status registers
                extended = unit.hasExtendedChannels && index < 2;
                fifoCapacity = extended ? 32 : 8;
                fifo = new Queue<byte>();
                Reset();
            }

            public void Reset()
            {
                Enabled = false;
                blockActive = false;
                stalled = false;
                fifo.Clear();
                sar = 0;
                dar = 0;
                llp = 0;
                controlLow = 0x00304801;
                controlHigh = 0x2;
                sourceStatus = 0;
                destinationStatus = 0;
                sourceStatusAddress = 0;
                destinationStatusAddress = 0;
                configLow = 0x0E00u | (uint)(index << 5);
                configHigh = 0x4;
                sourceGather = 0;
                destinationScatter = 0;
                sourceTransactionLeft = 0;
                destinationTransactionLeft = 0;
                currentLinkedListItem = 0;
            }

            public void DefineRegisters(DoubleWordRegisterCollection registers)
            {
                var b = index * ChannelStride;
                DefineStorage(registers, b + 0x00, "SAR", () => sar, v => sar = v, lockWhenEnabled: true);
                DefineStorage(registers, b + 0x08, "DAR", () => dar, v => dar = v, lockWhenEnabled: true);
                if(extended)
                {
                    DefineStorage(registers, b + 0x10, "LLP", () => llp, v => llp = v & ~3u, lockWhenEnabled: true);
                    DefineStorage(registers, b + 0x20, "SSTAT", () => sourceStatus, v => sourceStatus = v, lockWhenEnabled: true);
                    DefineStorage(registers, b + 0x28, "DSTAT", () => destinationStatus, v => destinationStatus = v, lockWhenEnabled: true);
                    DefineStorage(registers, b + 0x30, "SSTATAR", () => sourceStatusAddress, v => sourceStatusAddress = v, lockWhenEnabled: true);
                    DefineStorage(registers, b + 0x38, "DSTATAR", () => destinationStatusAddress, v => destinationStatusAddress = v, lockWhenEnabled: true);
                    DefineStorage(registers, b + 0x48, "SGR", () => sourceGather, v => sourceGather = v, lockWhenEnabled: true);
                    DefineStorage(registers, b + 0x50, "DSR", () => destinationScatter, v => destinationScatter = v, lockWhenEnabled: true);
                }
                else
                {
                    // the XMC library writes these registers of all channels; they do not exist for channels 2-7 and GPDMA1
                    foreach(var offset in new[] { 0x10, 0x20, 0x28, 0x30, 0x38, 0x48, 0x50 })
                    {
                        registers.DefineRegister(b + offset).WithIgnoredBits(0, 32);
                    }
                }

                // CTLL may be rewritten while the channel runs: the library clears LLP_SRC_EN/LLP_DST_EN to end a multi-block transfer
                DefineStorage(registers, b + 0x18, "CTLL", () => controlLow, v => controlLow = v, lockWhenEnabled: false);
                registers.DefineRegister(b + 0x1C, 0x2)
                    .WithValueField(0, 12, name: "BLOCK_TS",
                        valueProviderCallback: _ => blockActive || Enabled ? readbackItems : controlHigh & BlockSizeMask,
                        writeCallback: (_, value) =>
                        {
                            if(Enabled)
                            {
                                unit.Log(LogLevel.Warning, "Channel {0}: write to CTLH.BLOCK_TS while the channel is enabled, ignored", index);
                                return;
                            }
                            controlHigh = (controlHigh & ~BlockSizeMask) | (uint)value;
                            readbackItems = (uint)value;
                        })
                    .WithFlag(12, name: "DONE",
                        valueProviderCallback: _ => (controlHigh & DoneBit) != 0,
                        writeCallback: (_, value) => controlHigh = value ? controlHigh | DoneBit : controlHigh & ~DoneBit)
                    .WithReservedBits(13, 19);

                registers.DefineRegister(b + 0x40, 0x0E00u | (uint)(index << 5))
                    .WithValueField(0, 32, name: "CFGL",
                        valueProviderCallback: _ => (configLow & ~FifoEmptyBit) | (fifo.Count == 0 ? FifoEmptyBit : 0),
                        writeCallback: (_, value) =>
                        {
                            var wasSuspended = Suspended;
                            configLow = (uint)value;
                            if(wasSuspended && !Suspended)
                            {
                                unit.Kick();
                            }
                        });
                DefineStorage(registers, b + 0x44, "CFGH", () => configHigh, v => configHigh = v, lockWhenEnabled: false, reset: 0x4);
            }

            public bool Enabled { get; private set; }

            public bool InterruptEnable => (controlLow & 1) != 0;

            public int Priority => (int)((configLow >> 5) & 7);

            public void Enable()
            {
                if(Enabled)
                {
                    return;
                }
                Enabled = true;
                fifo.Clear();
                stalled = false;
                currentLinkedListItem = 0;
                initialSar = sar;
                initialDar = dar;
                initialControlLow = controlLow;
                initialControlHigh = controlHigh;
                initialLlp = llp;

                if(((controlLow >> 20) & 7) >= 4)
                {
                    unit.Log(LogLevel.Warning, "Channel {0}: peripheral flow control is not supported (TT_FC={1}), the GPDMA is the flow controller", index, (controlLow >> 20) & 7);
                }
                if(LinkedListUpdate && !FetchLinkedListItem())
                {
                    return;
                }
                StartBlock();
            }

            public void Disable()
            {
                if(!Enabled)
                {
                    return;
                }
                Enabled = false;
                blockActive = false;
                stalled = false;
                fifo.Clear();
                ReleaseLines();
            }

            public bool UsesLine(int line)
            {
                return Enabled && blockActive && ((HardwareSource && SourcePeripheralLine == line) || (HardwareDestination && DestinationPeripheralLine == line));
            }

            public void BlockInterruptCleared()
            {
                if(stalled)
                {
                    stalled = false;
                    StartBlock();
                    unit.Kick();
                }
            }

            public bool CanProceed()
            {
                return Enabled && blockActive && (WriteSize() > 0 && DestinationReady() || CanRead());
            }

            // One bus transfer: the destination write is preferred over the next source read
            public void Proceed()
            {
                var size = WriteSize();
                if(size > 0 && DestinationReady())
                {
                    Write(size);
                    return;
                }
                if(CanRead())
                {
                    Read();
                }
            }

            private bool Suspended => (configLow & SuspendBit) != 0;

            private bool LinkedListUpdate => (llp & ~3u) != 0 && (controlLow & (LinkedListSourceBit | LinkedListDestinationBit)) != 0;

            private int TransferType => (int)((controlLow >> 20) & 7);

            private bool SourceIsPeripheral => TransferType == 2 || TransferType == 3 || TransferType == 4 || TransferType == 5 || TransferType == 7;

            private bool DestinationIsPeripheral => TransferType == 1 || TransferType == 3 || TransferType == 6 || TransferType == 7;

            private bool HardwareSource => SourceIsPeripheral && (configLow & HandshakeSourceSoftwareBit) == 0;

            private bool HardwareDestination => DestinationIsPeripheral && (configLow & HandshakeDestinationSoftwareBit) == 0;

            private int SourcePeripheralLine => (int)((configHigh >> 7) & 0xF) & (unit.numberOfChannels > 4 ? 7 : 3);

            private int DestinationPeripheralLine => (int)((configHigh >> 11) & 0xF) & (unit.numberOfChannels > 4 ? 7 : 3);

            private int SourceWidth => 1 << (int)Math.Min((controlLow >> 4) & 7, 2);

            private int DestinationWidth => 1 << (int)Math.Min((controlLow >> 1) & 7, 2);

            private static int BurstLength(uint field)
            {
                return field == 1 ? 4 : field == 2 ? 8 : 1;
            }

            private void DefineStorage(DoubleWordRegisterCollection registers, long offset, string name, Func<uint> getter, Action<uint> setter, bool lockWhenEnabled, uint reset = 0)
            {
                registers.DefineRegister(offset, reset)
                    .WithValueField(0, 32, name: name,
                        valueProviderCallback: _ => getter(),
                        writeCallback: (_, value) =>
                        {
                            if(lockWhenEnabled && Enabled)
                            {
                                unit.Log(LogLevel.Warning, "Channel {0}: write to {1} while the channel is enabled, ignored", index, name);
                                return;
                            }
                            setter((uint)value);
                        });
            }

            // Loads SAR/DAR (for the linked sides), LLP, CTL from the linked list item that LLP points to
            private bool FetchLinkedListItem()
            {
                var address = llp & ~3u;
                if(!unit.IsMapped(address) || !unit.IsMapped(address + 0x10))
                {
                    unit.Log(LogLevel.Warning, "Channel {0}: linked list item at 0x{1:X8} is not accessible", index, address);
                    RaiseError();
                    return false;
                }
                var itemSar = unit.bus.ReadDoubleWord(address);
                var itemDar = unit.bus.ReadDoubleWord(address + 4);
                var itemLlp = unit.bus.ReadDoubleWord(address + 8);
                var itemControlLow = unit.bus.ReadDoubleWord(address + 0xC);
                var itemControlHigh = unit.bus.ReadDoubleWord(address + 0x10);
                unit.Log(LogLevel.Noisy, "Channel {0}: linked list item at 0x{1:X8}: SAR=0x{2:X8} DAR=0x{3:X8} LLP=0x{4:X8} CTL=0x{5:X8}/0x{6:X8}", index, address, itemSar, itemDar, itemLlp, itemControlHigh, itemControlLow);

                var sourceLinked = (controlLow & LinkedListSourceBit) != 0;
                var destinationLinked = (controlLow & LinkedListDestinationBit) != 0;
                if(sourceLinked)
                {
                    sar = itemSar;
                }
                else if((configLow & ReloadSourceBit) != 0)
                {
                    sar = initialSar;
                }
                if(destinationLinked)
                {
                    dar = itemDar;
                }
                else if((configLow & ReloadDestinationBit) != 0)
                {
                    dar = initialDar;
                }
                llp = itemLlp & ~3u;
                controlLow = itemControlLow;
                controlHigh = itemControlHigh;
                currentLinkedListItem = address;
                return true;
            }

            private void StartBlock()
            {
                blockActive = true;
                var items = controlHigh & BlockSizeMask;
                sourceBytesLeft = (int)items * SourceWidth;
                destinationBytesLeft = sourceBytesLeft;
                readbackItems = 0;
                sourceTransactionLeft = 0;
                destinationTransactionLeft = 0;
                gatherLeft = (int)(sourceGather >> 20);
                scatterLeft = (int)(destinationScatter >> 20);
                if(items == 0)
                {
                    BlockComplete();
                }
            }

            // The source is read as long as the FIFO has room; a peripheral source needs a request for every transaction
            private bool CanRead()
            {
                if(!blockActive || sourceBytesLeft <= 0 || Suspended || fifo.Count + SourceWidth > fifoCapacity)
                {
                    return false;
                }
                return !SourceIsPeripheral || sourceTransactionLeft > 0 || SourceRequestAvailable();
            }

            private bool DestinationReady()
            {
                return !DestinationIsPeripheral || destinationTransactionLeft > 0 || DestinationRequestAvailable();
            }

            // The size of the next write to the destination: a full transfer, or the rest at the end of the block, 0 when the FIFO has no data for it
            private int WriteSize()
            {
                if(!blockActive || destinationBytesLeft <= 0 || fifo.Count == 0)
                {
                    return 0;
                }
                var size = DestinationWidth;
                if(fifo.Count < size)
                {
                    if(sourceBytesLeft > 0)
                    {
                        return 0;
                    }
                    size = LargestPowerOfTwoUpTo(fifo.Count);
                }
                if(size > destinationBytesLeft)
                {
                    size = LargestPowerOfTwoUpTo(destinationBytesLeft);
                }
                return size;
            }

            private static int LargestPowerOfTwoUpTo(int value)
            {
                return value >= 4 ? 4 : value >= 2 ? 2 : 1;
            }

            private bool SourceRequestAvailable()
            {
                if(HardwareSource)
                {
                    return unit.IsRequestAvailable(SourcePeripheralLine, index);
                }
                return SoftwareRequestAvailable(unit.softRequestSource);
            }

            private bool DestinationRequestAvailable()
            {
                if(HardwareDestination)
                {
                    return unit.IsRequestAvailable(DestinationPeripheralLine, index);
                }
                return SoftwareRequestAvailable(unit.softRequestDestination);
            }

            // Software handshaking: the single request bit starts a transaction, it is a burst when the request bit is set as well
            private bool SoftwareRequestAvailable(uint[] registers)
            {
                return (registers[1] & (1u << index)) != 0;
            }

            private int TakeSoftwareRequest(uint[] registers, int itemsLeft, int burst)
            {
                var bit = 1u << index;
                var isBurst = (registers[0] & bit) != 0;
                registers[0] &= ~bit;
                registers[1] &= ~bit;
                return isBurst ? Math.Min(burst, itemsLeft) : 1;
            }

            private void Read()
            {
                var width = SourceWidth;
                if(SourceIsPeripheral && sourceTransactionLeft == 0)
                {
                    var items = sourceBytesLeft / width;
                    var burst = BurstLength((controlLow >> 14) & 7);
                    if(HardwareSource)
                    {
                        unit.TakeRequest(SourcePeripheralLine, index);
                        sourceTransactionLeft = Math.Min(burst, items);
                    }
                    else
                    {
                        sourceTransactionLeft = TakeSoftwareRequest(unit.softRequestSource, items, burst);
                    }
                }

                if(!unit.IsMapped(sar))
                {
                    unit.Log(LogLevel.Warning, "Channel {0}: read from 0x{1:X8} is not mapped, the transfer is aborted", index, sar);
                    RaiseError();
                    return;
                }
                uint value;
                switch(width)
                {
                    case 1:
                        value = unit.bus.ReadByte(sar);
                        break;
                    case 2:
                        value = unit.bus.ReadWord(sar);
                        break;
                    default:
                        value = unit.bus.ReadDoubleWord(sar);
                        break;
                }
                for(var i = 0; i < width; i++)
                {
                    fifo.Enqueue((byte)(value >> (8 * i)));
                }
                sourceBytesLeft -= width;
                readbackItems++;
                sar = NextAddress(sar, width, (controlLow >> 9) & 3, (controlLow & GatherEnableBit) != 0, sourceGather, ref gatherLeft);

                if(SourceIsPeripheral && --sourceTransactionLeft == 0)
                {
                    unit.RaiseEvent(SourceTransactionEvent, index);
                }
            }

            private void Write(int size)
            {
                if(DestinationIsPeripheral && destinationTransactionLeft == 0)
                {
                    var items = (destinationBytesLeft + DestinationWidth - 1) / DestinationWidth;
                    var burst = BurstLength((controlLow >> 11) & 7);
                    if(HardwareDestination)
                    {
                        unit.TakeRequest(DestinationPeripheralLine, index);
                        destinationTransactionLeft = Math.Min(burst, items);
                    }
                    else
                    {
                        destinationTransactionLeft = TakeSoftwareRequest(unit.softRequestDestination, items, burst);
                    }
                }

                if(!unit.IsMapped(dar))
                {
                    unit.Log(LogLevel.Warning, "Channel {0}: write to 0x{1:X8} is not mapped, the transfer is aborted", index, dar);
                    RaiseError();
                    return;
                }
                var value = 0u;
                for(var i = 0; i < size; i++)
                {
                    value |= (uint)fifo.Dequeue() << (8 * i);
                }
                switch(size)
                {
                    case 1:
                        unit.bus.WriteByte(dar, (byte)value);
                        break;
                    case 2:
                        unit.bus.WriteWord(dar, (ushort)value);
                        break;
                    default:
                        unit.bus.WriteDoubleWord(dar, value);
                        break;
                }
                destinationBytesLeft -= size;
                dar = NextAddress(dar, size, (controlLow >> 7) & 3, (controlLow & ScatterEnableBit) != 0, destinationScatter, ref scatterLeft);

                if(DestinationIsPeripheral && --destinationTransactionLeft == 0)
                {
                    unit.RaiseEvent(DestinationTransactionEvent, index);
                }
                if(destinationBytesLeft <= 0)
                {
                    BlockComplete();
                }
            }

            // Address after a transfer: increment (00), decrement (01), no change (1x); gather/scatter skips an interval after every count transfers
            private static uint NextAddress(uint address, int width, uint mode, bool gatherScatter, uint gatherScatterRegister, ref int left)
            {
                if((mode & 2) != 0)
                {
                    return address;
                }
                var step = (int)width;
                var next = mode == 0 ? address + (uint)step : address - (uint)step;
                if(gatherScatter && gatherScatterRegister >> 20 != 0 && --left <= 0)
                {
                    var skip = (int)((gatherScatterRegister & 0xFFFFF) * (uint)width);
                    next = mode == 0 ? next + (uint)skip : next - (uint)skip;
                    left = (int)(gatherScatterRegister >> 20);
                }
                return next;
            }

            private void BlockComplete()
            {
                blockActive = false;
                controlHigh = (controlHigh & ~BlockSizeMask) | (readbackItems & BlockSizeMask);
                if(currentLinkedListItem != 0)
                {
                    // write-back of CTLH with the DONE bit into the linked list item the block was loaded from
                    if(unit.IsMapped(currentLinkedListItem + 0x10))
                    {
                        unit.bus.WriteDoubleWord(currentLinkedListItem + 0x10, controlHigh | DoneBit);
                    }
                }
                unit.RaiseEvent(BlockEvent, index);

                var linked = LinkedListUpdate;
                var reload = (configLow & (ReloadSourceBit | ReloadDestinationBit)) != 0;
                if(!linked && !reload)
                {
                    TransferComplete();
                    return;
                }

                if(linked)
                {
                    if(!FetchLinkedListItem())
                    {
                        return;
                    }
                }
                else
                {
                    if((configLow & ReloadSourceBit) != 0)
                    {
                        sar = initialSar;
                    }
                    if((configLow & ReloadDestinationBit) != 0)
                    {
                        dar = initialDar;
                    }
                    controlLow = initialControlLow;
                    controlHigh = initialControlHigh;
                    llp = initialLlp;
                }

                // with auto-reload of an address the channel waits for the software to clear the block interrupt while that is unmasked
                if(reload && InterruptEnable && (unit.mask[BlockEvent] & (1u << index)) != 0)
                {
                    stalled = true;
                    return;
                }
                StartBlock();
            }

            private void TransferComplete()
            {
                Enabled = false;
                ReleaseLines();
                unit.RaiseEvent(TransferEvent, index);
            }

            private void RaiseError()
            {
                Enabled = false;
                blockActive = false;
                fifo.Clear();
                ReleaseLines();
                unit.RaiseEvent(ErrorEvent, index);
            }

            // The channel no longer waits for requests: the ones that only it was missing to be acknowledged are done now
            private void ReleaseLines()
            {
                for(var line = 0; line < unit.numberOfChannels; line++)
                {
                    unit.servedMask[index] &= ~(1u << line);
                    unit.AcknowledgeIfServedSafe(line);
                }
            }

            private readonly XMC4XXX_GPDMA unit;
            private readonly int index;
            private readonly bool extended;
            private readonly int fifoCapacity;
            private readonly Queue<byte> fifo;

            // registers
            private uint sar;
            private uint dar;
            private uint llp;
            private uint controlLow;
            private uint controlHigh;
            private uint sourceStatus;
            private uint destinationStatus;
            private uint sourceStatusAddress;
            private uint destinationStatusAddress;
            private uint configLow;
            private uint configHigh;
            private uint sourceGather;
            private uint destinationScatter;

            // state of the transfer
            private bool blockActive;
            private bool stalled;
            private int sourceBytesLeft;
            private int destinationBytesLeft;
            private int sourceTransactionLeft;
            private int destinationTransactionLeft;
            private int gatherLeft;
            private int scatterLeft;
            private uint readbackItems;
            private uint currentLinkedListItem;
            private uint initialSar;
            private uint initialDar;
            private uint initialControlLow;
            private uint initialControlHigh;
            private uint initialLlp;

            private const long ChannelStride = 0x58;
            private const uint BlockSizeMask = 0xFFF;
            private const uint DoneBit = 1u << 12;
            private const uint GatherEnableBit = 1u << 17;
            private const uint ScatterEnableBit = 1u << 18;
            private const uint LinkedListDestinationBit = 1u << 27;
            private const uint LinkedListSourceBit = 1u << 28;
            private const uint SuspendBit = 1u << 8;
            private const uint FifoEmptyBit = 1u << 9;
            private const uint HandshakeDestinationSoftwareBit = 1u << 10;
            private const uint HandshakeSourceSoftwareBit = 1u << 11;
            private const uint ReloadSourceBit = 1u << 30;
            private const uint ReloadDestinationBit = 1u << 31;
        }

        private bool IsMapped(uint address)
        {
            return bus.WhatPeripheralIsAt(address) != null;
        }

        private void AcknowledgeIfServedSafe(int line)
        {
            if(dlr != null && dlr.IsRequestPending(firstDlrLine + line))
            {
                AcknowledgeIfServed(line);
            }
        }

        private enum Registers : long
        {
            RawTransfer = 0x2C0,
            StatusTransfer = 0x2E8,
            MaskTransfer = 0x310,
            ClearTransfer = 0x338,
            StatusInterrupt = 0x360,
            RequestSource = 0x368,
            RequestDestination = 0x370,
            SingleRequestSource = 0x378,
            SingleRequestDestination = 0x380,
            LastSource = 0x388,
            LastDestination = 0x390,
            Configuration = 0x398,
            ChannelEnable = 0x3A0,
            Id = 0x3A8,
            Type = 0x3F8,
            Version = 0x3FC,
        }
    }
}
