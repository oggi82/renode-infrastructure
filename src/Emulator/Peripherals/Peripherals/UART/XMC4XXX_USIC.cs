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
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    // One USIC channel of the Infineon XMC4xxx family (Reference Manual v1.6, chapter 17).
    // The protocol is selected at run time by CCR.MODE: ASC (UART), SSC (SPI) or IIC (I2C).
    // Common to all protocols: TBUF/IN, RBUF/OUTR, the optional transmit and receive FIFOs
    // (TBCTR/RBCTR/TRBPTR/TRBSR), the indication flags in PSR and the six service request lines (INPR).
    // Every event produces a pulse on the service request line chosen by the node pointer.
    //
    // ASC is timed: a frame needs start + data + parity + stop bits at the baud rate derived from
    // FDR, BRG and the peripheral clock of the SCU. SSC and IIC transfers are performed at once.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public partial class XMC4XXX_USIC : UARTBase, IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize, INumberedGPIOOutput, IGPIOReceiver,
        IPeripheralContainer<ISPIPeripheral, NumberRegistrationPoint<int>>, IPeripheralContainer<II2CPeripheral, NumberRegistrationPoint<int>>
    {
        public XMC4XXX_USIC(IMachine machine, XMC4XXX_SCU scu = null, uint peripheralClockFrequency = 72000000) : base(machine)
        {
            this.machine = machine;
            this.scu = scu;
            this.peripheralClockFrequency = peripheralClockFrequency;
            spiContainer = new SimpleContainerHelper<ISPIPeripheral>(machine, this);
            i2cContainer = new SimpleContainerHelper<II2CPeripheral>(machine, this);

            var connections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < ServiceRequestLines; i++)
            {
                connections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(connections);

            txFifo = new Queue<TransmitEntry>();
            rxFifo = new Queue<ReceivedWord>();
            receiveBuffer = new Queue<ReceivedWord>();

            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
            ResetState();
        }

        public override void Reset()
        {
            base.Reset();
            RegistersCollection.Reset();
            ResetState();
            UpdateInterrupts();
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint val)
        {
            RegistersCollection.Write(offset, val);
        }

        // Input pins of the input stages: number n is the level seen at DXn (before the polarity bit)
        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= InputStageCount)
            {
                return;
            }
            var old = inputLevel[number];
            inputLevel[number] = value;
            if(number == 2 && old != value)
            {
                OnSelectInputEdge(value);
            }
        }

        public void Register(ISPIPeripheral peripheral, NumberRegistrationPoint<int> registrationPoint) => spiContainer.Register(peripheral, registrationPoint);

        public void Unregister(ISPIPeripheral peripheral) => spiContainer.Unregister(peripheral);

        public IEnumerable<NumberRegistrationPoint<int>> GetRegistrationPoints(ISPIPeripheral peripheral) => spiContainer.GetRegistrationPoints(peripheral);

        public void Register(II2CPeripheral peripheral, NumberRegistrationPoint<int> registrationPoint) => i2cContainer.Register(peripheral, registrationPoint);

        public void Unregister(II2CPeripheral peripheral) => i2cContainer.Unregister(peripheral);

        public IEnumerable<NumberRegistrationPoint<int>> GetRegistrationPoints(II2CPeripheral peripheral) => i2cContainer.GetRegistrationPoints(peripheral);

        IEnumerable<IRegistered<ISPIPeripheral, NumberRegistrationPoint<int>>> IPeripheralContainer<ISPIPeripheral, NumberRegistrationPoint<int>>.Children => spiContainer.Children;

        IEnumerable<IRegistered<II2CPeripheral, NumberRegistrationPoint<int>>> IPeripheralContainer<II2CPeripheral, NumberRegistrationPoint<int>>.Children => i2cContainer.Children;

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        // Emulates an external wire between the TX and the RX pin of this channel (ASC)
        public bool Loopback { get; set; }

        // The channel whose pins are wired to this channel: ASC data goes to the peer's receiver, an SSC master
        // shifts words through the peer (SSC slave), an IIC master addresses the peer as slave.
        public XMC4XXX_USIC Peer { get; private set; }

        public void ConnectPeer(XMC4XXX_USIC peer)
        {
            Peer = peer;
        }

        public override Bits StopBits => (pcr.Value & 2) != 0 ? Bits.Two : Bits.One;

        public override Parity ParityBit
        {
            get
            {
                switch(ccr.Value >> 8 & 3)
                {
                    case 2:
                        return Parity.Even;
                    case 3:
                        return Parity.Odd;
                    default:
                        return Parity.None;
                }
            }
        }

        // ASC baud rate = fPDIV / ((PCTQ + 1) * (DCTQ + 1)) with fPDIV = fFD / (PDIV + 1) and fFD from the fractional divider
        public override uint BaudRate
        {
            get
            {
                var baud = AscBaudRate();
                return baud < 1 ? DefaultBaudRate : (uint)baud;
            }
        }

        public long Size => 0x200;

        private int CurrentMode => (int)(ccr.Value & 0xF);

        protected override bool IsReceiveEnabled => CurrentMode == ModeAsc;

        protected override void CharWritten()
        {
            ScheduleAscReceive();
        }

        protected override void QueueEmptied()
        {
            // intentionally left blank
        }

        private void ResetState()
        {
            generation++;
            tdv = false;
            tbufData = 0;
            tbufWritten = false;
            triggerEvent = false;
            ascTxBusy = false;
            ascRxScheduled = false;
            txFifo.Clear();
            rxFifo.Clear();
            receiveBuffer.Clear();
            txIn = txOut = rxIn = rxOut = 0;
            psr = 0;
            trbsr = 0;
            transmitBufferTrigger = false;
            receiveBufferTrigger = false;
            ResetProtocolState();
        }

        private void DefineRegisters()
        {
            Registers.ChannelConfiguration.Define(this, 0xCF)
                .WithValueField(0, 32, FieldMode.Read, name: "CCFG");

            Registers.KernelStateConfiguration.Define(this)
                .WithValueField(0, 32, name: "KSCFG");

            Registers.FractionalDivider.Define(this)
                .WithValueField(0, 10, out fdrStep, name: "STEP")
                .WithReservedBits(10, 4)
                .WithValueField(14, 2, out fdrMode, name: "DM")
                .WithValueField(16, 10, FieldMode.Read, name: "RESULT")
                .WithReservedBits(26, 6);

            Registers.BaudRateGenerator.Define(this)
                .WithValueField(0, 2, out brgClockSelect, name: "CLKSEL")
                .WithReservedBits(2, 1)
                .WithFlag(3, name: "TMEN")
                .WithFlag(4, out brgPpp, name: "PPPEN")
                .WithValueField(6, 2, out brgCtqSel, name: "CTQSEL")
                .WithValueField(8, 2, out brgPctq, name: "PCTQ")
                .WithValueField(10, 5, out brgDctq, name: "DCTQ")
                .WithReservedBits(15, 1)
                .WithValueField(16, 10, out brgPdiv, name: "PDIV")
                .WithReservedBits(26, 2)
                .WithFlag(28, name: "SCLKOSEL")
                .WithFlag(29, name: "MCLKCFG")
                .WithValueField(30, 2, name: "SCLKCFG");

            Registers.InterruptNodePointer.Define(this)
                .WithValueField(0, 3, out inprTsinp, name: "TSINP")
                .WithReservedBits(3, 1)
                .WithValueField(4, 3, out inprTbinp, name: "TBINP")
                .WithReservedBits(7, 1)
                .WithValueField(8, 3, out inprRinp, name: "RINP")
                .WithReservedBits(11, 1)
                .WithValueField(12, 3, out inprAinp, name: "AINP")
                .WithReservedBits(15, 1)
                .WithValueField(16, 3, out inprPinp, name: "PINP")
                .WithReservedBits(19, 13);

            for(var i = 0; i < InputStageCount; i++)
            {
                var index = i;
                (Registers.Input0Control + 4 * i).Define(this)
                    .WithValueField(0, 3, out dxSelect[i], name: $"DX{i}CR.DSEL")
                    .WithFlag(3, name: $"DX{i}CR.DCEN")
                    .WithFlag(4, out dxInputSwitch[i], name: $"DX{i}CR.INSW")
                    .WithFlag(5, name: $"DX{i}CR.DFEN")
                    .WithFlag(6, name: $"DX{i}CR.DSEN")
                    .WithReservedBits(7, 1)
                    .WithFlag(8, out dxPolarity[i], name: $"DX{i}CR.DPOL")
                    .WithFlag(9, name: $"DX{i}CR.SFSEL")
                    .WithValueField(10, 2, out dxCombination[i], name: $"DX{i}CR.CM")
                    .WithReservedBits(12, 3)
                    .WithFlag(15, FieldMode.Read, name: $"DX{i}CR.DXS", valueProviderCallback: _ => InputStageState(index))
                    .WithReservedBits(16, 16);
            }

            Registers.ShiftControl.Define(this)
                .WithFlag(0, out shiftMsbFirst, name: "SDIR")
                .WithFlag(1, out passiveDataLevel, name: "PDL")
                .WithValueField(2, 2, out dataShiftMode, name: "DSM")
                .WithValueField(4, 1, out hardwarePortDirection, name: "HPCDIR")
                .WithReservedBits(5, 1)
                .WithValueField(6, 2, name: "DOCFG")
                .WithValueField(8, 2, out transferMode, name: "TRM")
                .WithReservedBits(10, 6)
                .WithValueField(16, 6, out frameLengthField, name: "FLE")
                .WithReservedBits(22, 2)
                .WithValueField(24, 4, out wordLengthField, name: "WLE")
                .WithReservedBits(28, 4);

            Registers.TransmitControlStatus.Define(this)
                .WithFlag(0, out tcsrWordLengthMode, name: "WLEMD")
                .WithFlag(1, out tcsrSelectMode, name: "SELMD")
                .WithFlag(2, out tcsrFrameLengthMode, name: "FLEMD")
                .WithFlag(3, name: "WAMD")
                .WithFlag(4, out tcsrPortMode, name: "HPCMD")
                .WithFlag(5, out tcsrStartOfFrame, name: "SOF")
                .WithFlag(6, out tcsrEndOfFrame, name: "EOF")
                .WithFlag(7, FieldMode.Read, name: "TDV", valueProviderCallback: _ => tdv)
                .WithFlag(8, out tcsrSingleShot, name: "TDSSM")
                .WithReservedBits(9, 1)
                .WithValueField(10, 2, out tcsrGating, name: "TDEN")
                .WithFlag(12, out tcsrTriggerEnable, name: "TDVTR")
                .WithFlag(13, name: "WA")
                .WithReservedBits(14, 10)
                .WithFlag(24, FieldMode.Read, name: "TSOF", valueProviderCallback: _ => false)
                .WithReservedBits(25, 1)
                .WithFlag(26, FieldMode.Read, name: "TV", valueProviderCallback: _ => tdv && StartAllowed())
                .WithFlag(27, FieldMode.Read, name: "TVC", valueProviderCallback: _ => false)
                .WithFlag(28, FieldMode.Read, name: "TE", valueProviderCallback: _ => triggerEvent)
                .WithReservedBits(29, 3)
                .WithWriteCallback((_, __) => Pump());

            Registers.ProtocolControl.Define(this)
                .WithValueField(0, 32, out pcr, name: "PCR", changeCallback: (oldValue, newValue) => OnProtocolControlChanged((uint)oldValue, (uint)newValue));

            Registers.ChannelControl.Define(this)
                .WithValueField(0, 32, out ccr, name: "CCR", changeCallback: (oldValue, newValue) =>
                {
                    if((oldValue & 0xF) != (newValue & 0xF))
                    {
                        OnModeChanged();
                    }
                });

            Registers.CapturedModeTimer.Define(this)
                .WithValueField(0, 32, name: "CMTR");

            Registers.ProtocolStatus.Define(this)
                .WithValueField(0, 32, name: "PSR",
                    valueProviderCallback: _ => ReadProtocolStatus(),
                    writeCallback: (_, value) => psr |= (uint)value & SettablePsrMask);

            Registers.ProtocolStatusClear.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PSCR", writeCallback: (_, value) => ClearProtocolStatus((uint)value));

            Registers.ReceiveBufferStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "RBUFSR", valueProviderCallback: _ => ReceiveBufferStatus());

            Registers.ReceiveBuffer.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "RBUF", valueProviderCallback: _ => ReadReceiveBuffer(true));
            Registers.ReceiveBufferDebug.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "RBUFD", valueProviderCallback: _ => ReadReceiveBuffer(false));
            Registers.ReceiveBuffer0.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "RBUF0", valueProviderCallback: _ => PeekReceiveBuffer(0));
            Registers.ReceiveBuffer1.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "RBUF1", valueProviderCallback: _ => PeekReceiveBuffer(1));
            Registers.ReceiveBuffer01Status.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "RBUF01SR", valueProviderCallback: _ => ReceiveBuffer01Status());

            Registers.FlagModification.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "FMR", writeCallback: (_, value) => ModifyFlags((uint)value));

            for(var i = 0; i < TransmitWindowSize; i++)
            {
                var tci = i;
                (Registers.TransmitBuffer0 + 4 * i).Define(this)
                    .WithValueField(0, 16, name: "TBUF", valueProviderCallback: _ => tbufData, writeCallback: (_, value) => WriteTransmitBuffer(tci, (uint)value))
                    .WithReservedBits(16, 16);
                (Registers.FifoInput0 + 4 * i).Define(this)
                    .WithValueField(0, 16, FieldMode.Write, name: "IN", writeCallback: (_, value) => WriteFifoInput(tci, (uint)value))
                    .WithReservedBits(16, 16);
            }

            Registers.Bypass.Define(this)
                .WithValueField(0, 32, name: "BYP");
            Registers.BypassControl.Define(this)
                .WithValueField(0, 32, name: "BYPCR");

            Registers.TransmitBufferControl.Define(this)
                .WithValueField(0, 6, out tbctrDataPointer, name: "DPTR")
                .WithReservedBits(6, 2)
                .WithValueField(8, 6, out tbctrLimit, name: "LIMIT")
                .WithFlag(14, out tbctrTriggerMode, name: "STBTM")
                .WithFlag(15, out tbctrTriggerEnable, name: "STBTEN")
                .WithValueField(16, 3, out tbctrStandardNode, name: "STBINP")
                .WithValueField(19, 3, out tbctrErrorNode, name: "ATBINP")
                .WithReservedBits(22, 2)
                .WithValueField(24, 3, out tbctrSize, name: "SIZE")
                .WithReservedBits(27, 1)
                .WithFlag(28, out tbctrLimitOccurrence, name: "LOF")
                .WithReservedBits(29, 1)
                .WithFlag(30, out tbctrStandardEnable, name: "STBIEN")
                .WithFlag(31, out tbctrErrorEnable, name: "TBERIEN")
                .WithWriteCallback((_, __) => OnTransmitBufferControlWritten());

            Registers.ReceiveBufferControl.Define(this)
                .WithValueField(0, 6, out rbctrDataPointer, name: "DPTR")
                .WithReservedBits(6, 2)
                .WithValueField(8, 6, out rbctrLimit, name: "LIMIT")
                .WithFlag(14, out rbctrTriggerMode, name: "SRBTM")
                .WithFlag(15, out rbctrTriggerEnable, name: "SRBTEN")
                .WithValueField(16, 3, out rbctrStandardNode, name: "SRBINP")
                .WithValueField(19, 3, out rbctrAlternativeNode, name: "ARBINP")
                .WithValueField(22, 2, name: "RCIM")
                .WithValueField(24, 3, out rbctrSize, name: "SIZE")
                .WithFlag(27, out rbctrReceiveControlMode, name: "RNM")
                .WithFlag(28, out rbctrLimitOccurrence, name: "LOF")
                .WithFlag(29, out rbctrAlternativeEnable, name: "ARBIEN")
                .WithFlag(30, out rbctrStandardEnable, name: "SRBIEN")
                .WithFlag(31, out rbctrErrorEnable, name: "RBERIEN")
                .WithWriteCallback((_, __) => OnReceiveBufferControlWritten());

            Registers.TransmitReceiveBufferPointer.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "TRBPTR", valueProviderCallback: _ => BufferPointers());

            Registers.TransmitReceiveBufferStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "TRBSR", valueProviderCallback: _ => BufferStatus());

            Registers.TransmitReceiveBufferStatusClear.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "TRBSCR", writeCallback: (_, value) => ClearBufferStatus((uint)value));

            Registers.ReceiveBufferOutput.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "OUTR", valueProviderCallback: _ => ReadFifoOutput(true));
            Registers.ReceiveBufferOutputDebug.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "OUTDR", valueProviderCallback: _ => ReadFifoOutput(false));
        }

        // ---------------------------------------------------------------- interrupts and flags

        private bool CcrBit(int bit) => (ccr.Value >> bit & 1) != 0;

        private void Pulse(ulong node)
        {
            if(node < ServiceRequestLines)
            {
                Connections[(int)node].Blink();
            }
        }

        private void UpdateInterrupts()
        {
            // All outputs are pulses; nothing is held between events
            for(var i = 0; i < ServiceRequestLines; i++)
            {
                Connections[i].Unset();
            }
        }

        private void RaiseTransmitShift()
        {
            psr |= PsrTsif;
            if(CcrBit(12))
            {
                Pulse(inprTsinp.Value);
            }
        }

        private void RaiseTransmitBuffer()
        {
            psr |= PsrTbif;
            if(CcrBit(13))
            {
                Pulse(inprTbinp.Value);
            }
        }

        private void RaiseReceiveStart()
        {
            psr |= PsrRsif;
            if(CcrBit(10))
            {
                Pulse(inprTbinp.Value);
            }
        }

        private void RaiseDataLost()
        {
            psr |= PsrDlif;
            if(CcrBit(11))
            {
                Pulse(inprPinp.Value);
            }
        }

        // Protocol events: the flag is a bit of the protocol part of PSR, enabled by a bit in PCR, routed to PINP
        private void RaiseProtocolEvent(uint flag, bool enabled)
        {
            psr |= flag;
            if(enabled)
            {
                Pulse(inprPinp.Value);
            }
        }

        private uint ReadProtocolStatus()
        {
            var value = psr;
            if(CurrentMode == ModeAsc)
            {
                value &= ~(uint)0x203;
                value |= (!ascTxBusy && !tdv ? 1u : 0u) | 2u | (ascTxBusy ? 0x200u : 0u);
            }
            else if(CurrentMode == ModeSsc)
            {
                value = SscStatus(value);
            }
            return value;
        }

        private void ClearProtocolStatus(uint value)
        {
            psr &= ~(value & ClearablePsrMask);
            if(CurrentMode == ModeSsc && (value & 1) != 0)
            {
                SscClearSelect();
            }
        }

        private void OnModeChanged()
        {
            ResetProtocolState();
            if(CurrentMode != ModeAsc)
            {
                ClearBuffer();
            }
        }

        private bool StartAllowed()
        {
            switch(tcsrGating.Value)
            {
                case 0:
                    return false;
                case 2:
                    return !InputStageState(2);
                case 3:
                    return InputStageState(2);
                default:
                    break;
            }
            return !tcsrTriggerEnable.Value || triggerEvent;
        }

        private bool InputStageState(int stage)
        {
            return inputLevel[stage] != dxPolarity[stage].Value;
        }

        private void OnSelectInputEdge(bool newLevel)
        {
            // DX2CR.CM: 1 rising edge, 2 falling edge, 3 both edges
            var cm = dxCombination[2].Value;
            var rising = newLevel != dxPolarity[2].Value;
            var trigger = (cm == 1 && rising) || (cm == 2 && !rising) || cm == 3;
            if(trigger)
            {
                triggerEvent = true;
                OnSelectTrigger();
                Pump();
            }
        }

        // ---------------------------------------------------------------- transmit path

        private void ApplyTransmitControlInformation(int tci)
        {
            if(tcsrWordLengthMode.Value)
            {
                wordLengthField.Value = (ulong)(tci & 0xF);
                tcsrEndOfFrame.Value = (tci & 0x10) != 0;
            }
            if(tcsrFrameLengthMode.Value)
            {
                frameLengthField.Value = (ulong)(tci & 0x1F);
            }
            if(tcsrSelectMode.Value)
            {
                pcr.Value = (pcr.Value & ~0xFF0000UL) | (ulong)(tci & 0x1F) << 16;
            }
            if(tcsrPortMode.Value)
            {
                dataShiftMode.Value = (ulong)(tci & 3);
                hardwarePortDirection.Value = (ulong)(tci >> 2 & 1);
            }
        }

        private void WriteTransmitBuffer(int tci, uint value)
        {
            tbufData = value & 0xFFFF;
            tbufWritten = true;
            tdv = true;
            ApplyTransmitControlInformation(tci);
            Pump();
        }

        private void WriteFifoInput(int tci, uint value)
        {
            var size = FifoSize(tbctrSize.Value);
            if(size == 0)
            {
                WriteTransmitBuffer(tci, value);
                return;
            }
            if(txFifo.Count >= size)
            {
                trbsr |= TrbsrTbeI;
                if(tbctrErrorEnable.Value)
                {
                    Pulse(tbctrErrorNode.Value);
                }
                return;
            }
            var old = txFifo.Count;
            txFifo.Enqueue(new TransmitEntry { Data = value & 0xFFFF, Tci = tci });
            txIn++;
            TransmitLevelChanged(old, txFifo.Count, true);
            Pump();
        }

        private void TransferFifoToTransmitBuffer()
        {
            while(!tdv && txFifo.Count > 0)
            {
                var old = txFifo.Count;
                var entry = txFifo.Dequeue();
                txOut++;
                tbufData = entry.Data;
                tbufWritten = true;
                tdv = true;
                ApplyTransmitControlInformation(entry.Tci);
                TransmitLevelChanged(old, txFifo.Count, false);
            }
        }

        private void TransmitLevelChanged(int old, int now, bool written)
        {
            var limit = (int)tbctrLimit.Value;
            var lof = tbctrLimitOccurrence.Value;
            var occurred = old == limit && (lof ? now > limit : now < limit);
            if(tbctrTriggerEnable.Value && transmitBufferTrigger && !occurred && lof == written && tbctrStandardEnable.Value)
            {
                Pulse(tbctrStandardNode.Value);
            }
            if(occurred)
            {
                trbsr |= TrbsrStbi;
                if(tbctrStandardEnable.Value)
                {
                    Pulse(tbctrStandardNode.Value);
                }
                if(tbctrTriggerEnable.Value)
                {
                    transmitBufferTrigger = true;
                }
            }
            if(transmitBufferTrigger && now == (tbctrTriggerMode.Value ? (int)FifoSize(tbctrSize.Value) : limit) && !occurred)
            {
                transmitBufferTrigger = false;
            }
        }

        // Called after TBUF has been loaded into the shift register
        private void ConsumeTransmitBuffer()
        {
            tdv = false;
            triggerEvent = false;
            RaiseTransmitBuffer();
        }

        // Moves FIFO words into TBUF and lets the protocol engine take them, until nothing is left to do
        private void Pump()
        {
            if(pumping)
            {
                return;
            }
            pumping = true;
            try
            {
                bool progress;
                do
                {
                    TransferFifoToTransmitBuffer();
                    progress = false;
                    if(!tdv || !StartAllowed())
                    {
                        break;
                    }
                    switch((int)CurrentMode)
                    {
                        case ModeAsc:
                            progress = AscStartTransmission();
                            break;
                        case ModeSsc:
                            progress = SscStartTransmission();
                            break;
                        case ModeIic:
                            progress = IicStartTransmission();
                            break;
                        default:
                            break;
                    }
                }
                while(progress);
            }
            finally
            {
                pumping = false;
            }
        }

        private void OnTransmitBufferControlWritten()
        {
            var size = FifoSize(tbctrSize.Value);
            if(size != configuredTxSize || tbctrDataPointer.Value != configuredTxPointer)
            {
                configuredTxSize = size;
                configuredTxPointer = tbctrDataPointer.Value;
                txFifo.Clear();
                txIn = txOut = 0;
            }
        }

        // ---------------------------------------------------------------- receive path

        private static uint FifoSize(ulong sizeField)
        {
            return sizeField == 0 || sizeField > 6 ? 0u : 1u << (int)sizeField;
        }

        private void OnReceiveBufferControlWritten()
        {
            var size = FifoSize(rbctrSize.Value);
            if(size != configuredRxSize || rbctrDataPointer.Value != configuredRxPointer)
            {
                configuredRxSize = size;
                configuredRxPointer = rbctrDataPointer.Value;
                rxFifo.Clear();
                rxIn = rxOut = 0;
            }
        }

        private void ReceiveWord(ReceivedWord word)
        {
            var fifoSize = FifoSize(rbctrSize.Value);
            if(fifoSize > 0)
            {
                if(rxFifo.Count >= fifoSize)
                {
                    RaiseDataLost();
                }
                else
                {
                    var old = rxFifo.Count;
                    rxFifo.Enqueue(word);
                    rxIn++;
                    ReceiveLevelChanged(old, rxFifo.Count, true, word);
                }
            }
            else
            {
                if(receiveBuffer.Count >= 2)
                {
                    receiveBuffer.Dequeue();
                    RaiseDataLost();
                }
                receiveBuffer.Enqueue(word);
            }

            if(word.ProtocolError)
            {
                psr |= PsrAif;
                if(CcrBit(15))
                {
                    Pulse(inprAinp.Value);
                }
            }
            else
            {
                psr |= PsrRif;
                if(CcrBit(14))
                {
                    Pulse(inprRinp.Value);
                }
            }
        }

        private void ReceiveLevelChanged(int old, int now, bool written, ReceivedWord word)
        {
            if(rbctrReceiveControlMode.Value)
            {
                if(!written)
                {
                    return;
                }
                if(word.ProtocolError)
                {
                    trbsr |= TrbsrArbi;
                    if(rbctrAlternativeEnable.Value)
                    {
                        Pulse(rbctrAlternativeNode.Value);
                    }
                }
                else
                {
                    trbsr |= TrbsrSrbi;
                    if(rbctrStandardEnable.Value)
                    {
                        Pulse(rbctrStandardNode.Value);
                    }
                }
                return;
            }

            var limit = (int)rbctrLimit.Value;
            var lof = rbctrLimitOccurrence.Value;
            var occurred = old == limit && (lof ? now > limit : now < limit);
            if(rbctrTriggerEnable.Value && receiveBufferTrigger && !occurred && lof == written && rbctrStandardEnable.Value)
            {
                Pulse(rbctrStandardNode.Value);
            }
            if(occurred)
            {
                trbsr |= TrbsrSrbi;
                if(rbctrStandardEnable.Value)
                {
                    Pulse(rbctrStandardNode.Value);
                }
                if(rbctrTriggerEnable.Value)
                {
                    receiveBufferTrigger = true;
                }
            }
            if(receiveBufferTrigger && now == (rbctrTriggerMode.Value ? 0 : limit) && !occurred)
            {
                receiveBufferTrigger = false;
            }
        }

        private uint ReadFifoOutput(bool remove)
        {
            if(rxFifo.Count == 0)
            {
                if(remove)
                {
                    trbsr |= TrbsrRberi;
                    if(rbctrErrorEnable.Value)
                    {
                        Pulse(rbctrAlternativeNode.Value);
                    }
                }
                return 0;
            }
            var word = rxFifo.Peek();
            if(remove)
            {
                var old = rxFifo.Count;
                rxFifo.Dequeue();
                rxOut++;
                ReceiveLevelChanged(old, rxFifo.Count, false, word);
            }
            return word.Data | ((word.ProtocolError ? 0x10u : 0u) | (word.WordLength & 0xF)) << 16;
        }

        private uint ReadReceiveBuffer(bool remove)
        {
            if(receiveBuffer.Count == 0)
            {
                return 0;
            }
            return remove ? receiveBuffer.Dequeue().Data : receiveBuffer.Peek().Data;
        }

        private uint PeekReceiveBuffer(int index)
        {
            return index < receiveBuffer.Count ? receiveBuffer.ElementAt(index).Data : 0;
        }

        private uint ReceiveBufferStatus()
        {
            if(receiveBuffer.Count == 0)
            {
                return 0;
            }
            return StatusOf(receiveBuffer.Peek(), 0) | 1u << 13;
        }

        private uint ReceiveBuffer01Status()
        {
            var value = 0u;
            if(receiveBuffer.Count > 0)
            {
                value |= (StatusOf(receiveBuffer.ElementAt(0), 0) & 0x7FFF) | 1u << 13;
            }
            if(receiveBuffer.Count > 1)
            {
                value |= (StatusOf(receiveBuffer.ElementAt(1), 0) & 0x7FFF) << 16 | 1u << 29;
            }
            return value;
        }

        private static uint StatusOf(ReceivedWord word, uint source)
        {
            return word.WordLength | (word.StartOfFrame ? 0x40u : 0u) | (word.ProtocolArgument ? 0x100u : 0u) | (word.ProtocolError ? 0x200u : 0u) | source << 15;
        }

        private uint BufferPointers()
        {
            uint Pointer(ulong dptr, uint size, uint counter) => size == 0 ? 0 : (uint)(dptr + counter % size) & 0x3F;
            var txSize = FifoSize(tbctrSize.Value);
            var rxSize = FifoSize(rbctrSize.Value);
            return Pointer(tbctrDataPointer.Value, txSize, (uint)txIn) | Pointer(tbctrDataPointer.Value, txSize, (uint)txOut) << 8
                | Pointer(rbctrDataPointer.Value, rxSize, (uint)rxIn) << 16 | Pointer(rbctrDataPointer.Value, rxSize, (uint)rxOut) << 24;
        }

        private uint BufferStatus()
        {
            var txSize = (int)FifoSize(tbctrSize.Value);
            var rxSize = (int)FifoSize(rbctrSize.Value);
            var value = trbsr;
            if(rxFifo.Count == 0)
            {
                value |= 1u << 3;
            }
            if(rxSize > 0 && rxFifo.Count >= rxSize)
            {
                value |= 1u << 4;
            }
            if(receiveBufferTrigger)
            {
                value |= 1u << 6;
            }
            if(txFifo.Count == 0)
            {
                value |= 1u << 11;
            }
            if(txSize > 0 && txFifo.Count >= txSize)
            {
                value |= 1u << 12;
            }
            if(transmitBufferTrigger)
            {
                value |= 1u << 14;
            }
            return value | (uint)rxFifo.Count << 16 | (uint)txFifo.Count << 24;
        }

        private void ClearBufferStatus(uint value)
        {
            trbsr &= ~(value & (TrbsrSrbi | TrbsrRberi | TrbsrArbi | TrbsrStbi | TrbsrTbeI));
            if((value & 1u << 14) != 0)
            {
                rxFifo.Clear();
                rxIn = rxOut = 0;
                receiveBufferTrigger = false;
            }
            if((value & 1u << 15) != 0)
            {
                txFifo.Clear();
                txIn = txOut = 0;
                transmitBufferTrigger = false;
            }
        }

        private void ModifyFlags(uint value)
        {
            switch(value & 3)
            {
                case 1:
                    tdv = true;
                    break;
                case 2:
                    tdv = false;
                    triggerEvent = false;
                    break;
                default:
                    break;
            }
            if((value & 1u << 14) != 0 || (value & 1u << 15) != 0)
            {
                if(receiveBuffer.Count > 0)
                {
                    receiveBuffer.Dequeue();
                }
            }
            for(var i = 0; i < ServiceRequestLines; i++)
            {
                if((value & 1u << (16 + i)) != 0)
                {
                    Connections[i].Blink();
                }
            }
            Pump();
        }

        // ---------------------------------------------------------------- ASC (UART)

        // fPDIV: fractional divider output divided by PDIV + 1 (and by 2 for PPPEN); 0 if the divider is off
        private double ProtocolDividerFrequency()
        {
            var peripheralClock = scu != null ? scu.PeripheralClockFrequency : peripheralClockFrequency;
            double frequency;
            switch(fdrMode.Value)
            {
                case 1:
                    frequency = (double)peripheralClock / (1024 - (int)fdrStep.Value);
                    break;
                case 2:
                    frequency = (double)peripheralClock * fdrStep.Value / 1024;
                    break;
                default:
                    return 0;
            }
            if(brgPpp.Value)
            {
                frequency /= 2;
            }
            return frequency / (brgPdiv.Value + 1);
        }

        private double AscBaudRate()
        {
            return ProtocolDividerFrequency() / ((brgPctq.Value + 1) * (brgDctq.Value + 1));
        }

        private double AscFrameSeconds()
        {
            var bits = 1 + (int)wordLengthField.Value + 1 + ((ccr.Value >> 8 & 3) >= 2 ? 1 : 0) + ((pcr.Value & 2) != 0 ? 2 : 1);
            return bits / (double)BaudRate;
        }

        private bool AscStartTransmission()
        {
            if(ascTxBusy)
            {
                return false;
            }
            var data = (byte)(tbufData & (0xFFu >> (7 - (int)Math.Min(wordLengthField.Value, 7))));
            ConsumeTransmitBuffer();
            ascTxBusy = true;
            var current = generation;
            machine.ScheduleAction(TimeInterval.FromSeconds(AscFrameSeconds()), _ => AscTransmissionDone(current, data), name: "XMC4XXX_USIC transmit");
            return true;
        }

        private void AscTransmissionDone(int expectedGeneration, byte data)
        {
            if(expectedGeneration != generation)
            {
                return;
            }
            ascTxBusy = false;
            TransmitCharacter(data);
            if(Loopback)
            {
                WriteChar(data);
            }
            Peer?.WriteChar(data);
            RaiseTransmitShift();
            Pump();
        }

        private void ScheduleAscReceive()
        {
            if(ascRxScheduled)
            {
                return;
            }
            ascRxScheduled = true;
            var current = generation;
            machine.ScheduleAction(TimeInterval.FromSeconds(AscFrameSeconds()), _ => AscReceptionDone(current), name: "XMC4XXX_USIC receive");
        }

        private void AscReceptionDone(int expectedGeneration)
        {
            if(expectedGeneration != generation)
            {
                return;
            }
            ascRxScheduled = false;
            if(!TryGetCharacter(out var character))
            {
                return;
            }
            RaiseReceiveStart();
            ReceiveWord(new ReceivedWord { Data = character, WordLength = (uint)wordLengthField.Value });
            if(Count > 0)
            {
                ScheduleAscReceive();
            }
        }

        private readonly IMachine machine;
        private readonly XMC4XXX_SCU scu;
        private readonly uint peripheralClockFrequency;
        private readonly SimpleContainerHelper<ISPIPeripheral> spiContainer;
        private readonly SimpleContainerHelper<II2CPeripheral> i2cContainer;
        private readonly Queue<TransmitEntry> txFifo;
        private readonly Queue<ReceivedWord> rxFifo;
        private readonly Queue<ReceivedWord> receiveBuffer;
        private readonly bool[] inputLevel = new bool[InputStageCount];
        private readonly IValueRegisterField[] dxSelect = new IValueRegisterField[InputStageCount];
        private readonly IFlagRegisterField[] dxInputSwitch = new IFlagRegisterField[InputStageCount];
        private readonly IFlagRegisterField[] dxPolarity = new IFlagRegisterField[InputStageCount];
        private readonly IValueRegisterField[] dxCombination = new IValueRegisterField[InputStageCount];

        private int generation;
        private bool pumping;
        private bool tdv;
        private bool tbufWritten;
        private uint tbufData;
        private bool triggerEvent;
        private bool ascTxBusy;
        private bool ascRxScheduled;
        private uint psr;
        private uint trbsr;
        private bool transmitBufferTrigger;
        private bool receiveBufferTrigger;
        private uint txIn;
        private uint txOut;
        private uint rxIn;
        private uint rxOut;
        private uint configuredTxSize;
        private uint configuredRxSize;
        private ulong configuredTxPointer;
        private ulong configuredRxPointer;

        private IValueRegisterField fdrStep;
        private IValueRegisterField fdrMode;
        private IValueRegisterField brgClockSelect;
        private IFlagRegisterField brgPpp;
        private IValueRegisterField brgCtqSel;
        private IValueRegisterField brgPctq;
        private IValueRegisterField brgDctq;
        private IValueRegisterField brgPdiv;
        private IValueRegisterField inprTsinp;
        private IValueRegisterField inprTbinp;
        private IValueRegisterField inprRinp;
        private IValueRegisterField inprAinp;
        private IValueRegisterField inprPinp;
        private IFlagRegisterField shiftMsbFirst;
        private IFlagRegisterField passiveDataLevel;
        private IValueRegisterField dataShiftMode;
        private IValueRegisterField hardwarePortDirection;
        private IValueRegisterField transferMode;
        private IValueRegisterField frameLengthField;
        private IValueRegisterField wordLengthField;
        private IFlagRegisterField tcsrWordLengthMode;
        private IFlagRegisterField tcsrSelectMode;
        private IFlagRegisterField tcsrFrameLengthMode;
        private IFlagRegisterField tcsrPortMode;
        private IFlagRegisterField tcsrStartOfFrame;
        private IFlagRegisterField tcsrEndOfFrame;
        private IFlagRegisterField tcsrSingleShot;
        private IValueRegisterField tcsrGating;
        private IFlagRegisterField tcsrTriggerEnable;
        private IValueRegisterField pcr;
        private IValueRegisterField ccr;
        private IValueRegisterField tbctrDataPointer;
        private IValueRegisterField tbctrLimit;
        private IFlagRegisterField tbctrTriggerMode;
        private IFlagRegisterField tbctrTriggerEnable;
        private IValueRegisterField tbctrStandardNode;
        private IValueRegisterField tbctrErrorNode;
        private IValueRegisterField tbctrSize;
        private IFlagRegisterField tbctrLimitOccurrence;
        private IFlagRegisterField tbctrStandardEnable;
        private IFlagRegisterField tbctrErrorEnable;
        private IValueRegisterField rbctrDataPointer;
        private IValueRegisterField rbctrLimit;
        private IFlagRegisterField rbctrTriggerMode;
        private IFlagRegisterField rbctrTriggerEnable;
        private IValueRegisterField rbctrStandardNode;
        private IValueRegisterField rbctrAlternativeNode;
        private IValueRegisterField rbctrSize;
        private IFlagRegisterField rbctrReceiveControlMode;
        private IFlagRegisterField rbctrLimitOccurrence;
        private IFlagRegisterField rbctrAlternativeEnable;
        private IFlagRegisterField rbctrStandardEnable;
        private IFlagRegisterField rbctrErrorEnable;

        private const int ServiceRequestLines = 6;
        private const int InputStageCount = 6;
        private const int TransmitWindowSize = 32;
        private const uint DefaultBaudRate = 115200;
        private const int ModeSsc = 1;
        private const int ModeAsc = 2;
        private const int ModeIic = 4;

        private const uint PsrRsif = 1u << 10;
        private const uint PsrDlif = 1u << 11;
        private const uint PsrTsif = 1u << 12;
        private const uint PsrTbif = 1u << 13;
        private const uint PsrRif = 1u << 14;
        private const uint PsrAif = 1u << 15;
        private const uint PsrBrgif = 1u << 16;
        private const uint SettablePsrMask = 0x1FFFF;
        private const uint ClearablePsrMask = 0x1FFFF;

        private const uint TrbsrSrbi = 1u << 0;
        private const uint TrbsrRberi = 1u << 1;
        private const uint TrbsrArbi = 1u << 2;
        private const uint TrbsrStbi = 1u << 8;
        private const uint TrbsrTbeI = 1u << 9;

        private class TransmitEntry
        {
            public uint Data;
            public int Tci;
        }

        private class ReceivedWord
        {
            public uint Data;
            public uint WordLength;
            public bool StartOfFrame;
            public bool ProtocolArgument;
            public bool ProtocolError;
        }

        private enum Registers : long
        {
            ChannelConfiguration = 0x04,
            KernelStateConfiguration = 0x0C,
            FractionalDivider = 0x10,
            BaudRateGenerator = 0x14,
            InterruptNodePointer = 0x18,
            Input0Control = 0x1C,
            ShiftControl = 0x34,
            TransmitControlStatus = 0x38,
            ProtocolControl = 0x3C,
            ChannelControl = 0x40,
            CapturedModeTimer = 0x44,
            ProtocolStatus = 0x48,
            ProtocolStatusClear = 0x4C,
            ReceiveBufferStatus = 0x50,
            ReceiveBuffer = 0x54,
            ReceiveBufferDebug = 0x58,
            ReceiveBuffer0 = 0x5C,
            ReceiveBuffer1 = 0x60,
            ReceiveBuffer01Status = 0x64,
            FlagModification = 0x68,
            TransmitBuffer0 = 0x80,
            Bypass = 0x100,
            BypassControl = 0x104,
            TransmitBufferControl = 0x108,
            ReceiveBufferControl = 0x10C,
            TransmitReceiveBufferPointer = 0x110,
            TransmitReceiveBufferStatus = 0x114,
            TransmitReceiveBufferStatusClear = 0x118,
            ReceiveBufferOutput = 0x11C,
            ReceiveBufferOutputDebug = 0x120,
            FifoInput0 = 0x180,
        }
    }
}
