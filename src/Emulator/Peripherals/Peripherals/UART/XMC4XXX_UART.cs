//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System.Collections.Generic;
using System.Collections.ObjectModel;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.UART
{
    // One USIC channel of the Infineon XMC4xxx family running in the asynchronous serial channel (ASC/UART) mode.
    // Data is sent and received without FIFOs and without shift timing: a word written to TBUF/IN is passed to
    // the host immediately and the transmit indication flags (TSIF, TBIF) are set at once.
    // The six service request lines of a USIC module are exposed as Connections 0-5 and selected per event with INPR.
    // The baud rate is derived from FDR and BRG assuming the given peripheral clock frequency.
    public class XMC4XXX_UART : UARTBase, IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize, INumberedGPIOOutput
    {
        public XMC4XXX_UART(IMachine machine, uint peripheralClockFrequency = 120000000) : base(machine)
        {
            this.peripheralClockFrequency = peripheralClockFrequency;

            var connections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < ServiceRequestLines; i++)
            {
                connections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(connections);

            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            RegistersCollection.Reset();
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

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public override Bits StopBits => stopBits.Value ? Bits.Two : Bits.One;

        public override Parity ParityBit
        {
            get
            {
                switch(parityMode.Value)
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

        public override uint BaudRate
        {
            get
            {
                // fPDIV = fPERIPH * STEP / 1024 (fractional divider), baud rate = fPDIV / ((PDIV + 1) * (DCTQ + 1) * (PCTQ + 1))
                if(fractionalDividerStep.Value == 0)
                {
                    return DefaultBaudRate;
                }
                var divider = (baudRatePreDivider.Value + 1) * (timeQuantaPerBit.Value + 1) * (timeQuantaPreDivider.Value + 1);
                var baudRate = (ulong)peripheralClockFrequency * fractionalDividerStep.Value / 1024 / divider;
                return baudRate == 0 ? DefaultBaudRate : (uint)baudRate;
            }
        }

        public long Size => 0x200;

        protected override void CharWritten()
        {
            receiveIndication.Value = true;
            UpdateInterrupts();
        }

        protected override void QueueEmptied()
        {
            // intentionally left blank
        }

        private void DefineRegisters()
        {
            Registers.KernelStateConfiguration.Define(this)
                .WithFlag(0, name: "MODEN")
                .WithFlag(1, name: "BPMODEN")
                .WithReservedBits(2, 2)
                .WithTag("NOMCFG", 4, 2)
                .WithReservedBits(6, 1)
                .WithTaggedFlag("BPNOM", 7)
                .WithTag("SUMCFG", 8, 2)
                .WithReservedBits(10, 1)
                .WithTaggedFlag("BPSUM", 11)
                .WithReservedBits(12, 20);

            Registers.FractionalDivider.Define(this)
                .WithValueField(0, 10, out fractionalDividerStep, name: "STEP")
                .WithReservedBits(10, 4)
                .WithValueField(14, 2, name: "DM")
                .WithReservedBits(16, 16);

            Registers.BaudRateGenerator.Define(this)
                .WithTag("CLKSEL", 0, 2)
                .WithReservedBits(2, 1)
                .WithTaggedFlag("TMEN", 3)
                .WithTaggedFlag("PPPEN", 4)
                .WithTag("CTQSEL", 6, 2)
                .WithValueField(8, 2, out timeQuantaPreDivider, name: "PCTQ")
                .WithValueField(10, 5, out timeQuantaPerBit, name: "DCTQ")
                .WithReservedBits(15, 1)
                .WithValueField(16, 10, out baudRatePreDivider, name: "PDIV")
                .WithTag("SCLKOSEL", 26, 1)
                .WithTag("MCLKCFG", 27, 2)
                .WithTag("SCLKCFG", 29, 2)
                .WithReservedBits(31, 1);

            Registers.InterruptNodePointer.Define(this)
                .WithValueField(0, 3, out transmitShiftNode, name: "TSINP")
                .WithReservedBits(3, 1)
                .WithValueField(4, 3, out transmitBufferNode, name: "TBINP")
                .WithReservedBits(7, 1)
                .WithValueField(8, 3, out receiveNode, name: "RINP")
                .WithReservedBits(11, 1)
                .WithValueField(12, 3, out alternativeReceiveNode, name: "AINP")
                .WithReservedBits(15, 1)
                .WithTag("PINP", 16, 3)
                .WithReservedBits(19, 13)
                .WithChangeCallback((_, __) => UpdateInterrupts());

            // The input stage, shift control and transmit control registers are stored only
            for(var offset = Registers.Input0Control; offset <= Registers.TransmitControlStatus; offset += 4)
            {
                offset.Define(this)
                    .WithValueField(0, 32, name: $"REG_{(long)offset:X2}");
            }

            Registers.ProtocolControl.Define(this)
                .WithFlag(0, name: "SMD")
                .WithFlag(1, out stopBits, name: "STPB")
                .WithFlag(2, name: "IDM")
                .WithFlag(3, name: "SBIEN")
                .WithFlag(4, name: "CDEN")
                .WithFlag(5, name: "RNIEN")
                .WithFlag(6, name: "FEIEN")
                .WithFlag(7, name: "FFIEN")
                .WithValueField(8, 5, name: "SP")
                .WithValueField(13, 3, name: "PL")
                .WithFlag(16, name: "RSTEN")
                .WithFlag(17, name: "TSTEN")
                .WithReservedBits(18, 13)
                .WithFlag(31, name: "MCLK");

            Registers.ChannelControl.Define(this)
                .WithValueField(0, 4, out mode, name: "MODE")
                .WithReservedBits(4, 2)
                .WithTaggedFlag("HPCEN", 6)
                .WithReservedBits(7, 1)
                .WithValueField(8, 2, out parityMode, name: "PM")
                .WithTaggedFlag("RSIEN", 10)
                .WithTaggedFlag("DLIEN", 11)
                .WithFlag(12, out transmitShiftInterruptEnable, name: "TSIEN")
                .WithFlag(13, out transmitBufferInterruptEnable, name: "TBIEN")
                .WithFlag(14, out receiveInterruptEnable, name: "RIEN")
                .WithFlag(15, out alternativeReceiveInterruptEnable, name: "AIEN")
                .WithTaggedFlag("BRGIEN", 16)
                .WithReservedBits(17, 15)
                .WithChangeCallback((_, __) => UpdateInterrupts());

            Registers.CapturedModeTimer.Define(this)
                .WithValueField(0, 32, name: "CMTR");

            Registers.ProtocolStatus.Define(this)
                .WithTag("ST0-ST9", 0, 10)
                .WithTaggedFlag("RSIF", 10)
                .WithTaggedFlag("DLIF", 11)
                .WithFlag(12, out transmitShiftIndication, FieldMode.Read, name: "TSIF")
                .WithFlag(13, out transmitBufferIndication, FieldMode.Read, name: "TBIF")
                .WithFlag(14, out receiveIndication, FieldMode.Read, name: "RIF")
                .WithFlag(15, out alternativeReceiveIndication, FieldMode.Read, name: "AIF")
                .WithTaggedFlag("BRGIF", 16)
                .WithReservedBits(17, 15);

            Registers.ProtocolStatusClear.Define(this)
                .WithValueField(0, 10, FieldMode.Write, name: "CST0-CST9")
                .WithFlag(10, FieldMode.Write, name: "CRSIF")
                .WithFlag(11, FieldMode.Write, name: "CDLIF")
                .WithFlag(12, FieldMode.Write, name: "CTSIF", writeCallback: (_, value) => ClearIndication(transmitShiftIndication, value))
                .WithFlag(13, FieldMode.Write, name: "CTBIF", writeCallback: (_, value) => ClearIndication(transmitBufferIndication, value))
                .WithFlag(14, FieldMode.Write, name: "CRIF", writeCallback: (_, value) => ClearIndication(receiveIndication, value))
                .WithFlag(15, FieldMode.Write, name: "CAIF", writeCallback: (_, value) => ClearIndication(alternativeReceiveIndication, value))
                .WithFlag(16, FieldMode.Write, name: "CBRGIF")
                .WithIgnoredBits(17, 15);

            Registers.ReceiveBufferStatus.Define(this)
                .WithReservedBits(0, 13)
                .WithFlag(13, FieldMode.Read, name: "RDV0", valueProviderCallback: _ => Count > 0)
                .WithTaggedFlag("RDV1", 14)
                .WithReservedBits(15, 17);

            // Reading any of the receive buffer registers returns the oldest received word.
            DefineReceiveBuffer(Registers.ReceiveBuffer, "RBUF");
            DefineReceiveBuffer(Registers.ReceiveBufferDebug, "RBUFD");
            DefineReceiveBuffer(Registers.ReceiveBuffer0, "RBUF0");

            Registers.FlagModification.Define(this)
                .WithTag("MTDV", 0, 2)
                .WithReservedBits(2, 2)
                .WithTaggedFlag("ATVC", 4)
                .WithReservedBits(5, 3)
                .WithTaggedFlag("CRDV0", 8)
                .WithTaggedFlag("CRDV1", 9)
                .WithReservedBits(10, 6)
                .WithTag("SIO0-SIO5", 16, 6)
                .WithReservedBits(22, 10);

            // Both the transmit buffer and the transmit FIFO input window send a word
            for(var i = 0; i < TransmitWindowSize; i++)
            {
                DefineTransmitWindow((Registers)((long)Registers.TransmitBuffer0 + 4 * i), "TBUF");
                DefineTransmitWindow((Registers)((long)Registers.FifoInput0 + 4 * i), "IN");
            }

            // Bypass data and FIFO configuration are stored only; the model never uses a FIFO
            for(var offset = Registers.Bypass; offset <= Registers.OutputDebug; offset += 4)
            {
                offset.Define(this)
                    .WithValueField(0, 32, name: $"REG_{(long)offset:X3}");
            }
        }

        private void DefineReceiveBuffer(Registers register, string name)
        {
            register.Define(this)
                .WithValueField(0, 16, FieldMode.Read, name: name, valueProviderCallback: _ =>
                {
                    if(!TryGetCharacter(out var character))
                    {
                        this.Log(LogLevel.Warning, "Read from an empty receive buffer");
                        return 0;
                    }
                    return character;
                })
                .WithReservedBits(16, 16);
        }

        private void DefineTransmitWindow(Registers register, string name)
        {
            register.Define(this)
                .WithValueField(0, 16, FieldMode.Write, name: name, writeCallback: (_, value) => Transmit((byte)value))
                .WithReservedBits(16, 16);
        }

        private void Transmit(byte value)
        {
            if(mode.Value != AsynchronousSerialChannelMode)
            {
                this.Log(LogLevel.Warning, "Ignoring transmission of 0x{0:X2}, channel is not in the ASC mode (CCR.MODE = {1})", value, mode.Value);
                return;
            }
            TransmitCharacter(value);
            transmitShiftIndication.Value = true;
            transmitBufferIndication.Value = true;
            UpdateInterrupts();
        }

        private void ClearIndication(IFlagRegisterField indication, bool clear)
        {
            if(!clear)
            {
                return;
            }
            indication.Value = false;
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            var lines = new bool[ServiceRequestLines];
            Route(lines, transmitShiftIndication.Value && transmitShiftInterruptEnable.Value, transmitShiftNode.Value);
            Route(lines, transmitBufferIndication.Value && transmitBufferInterruptEnable.Value, transmitBufferNode.Value);
            Route(lines, receiveIndication.Value && receiveInterruptEnable.Value, receiveNode.Value);
            Route(lines, alternativeReceiveIndication.Value && alternativeReceiveInterruptEnable.Value, alternativeReceiveNode.Value);

            for(var i = 0; i < ServiceRequestLines; i++)
            {
                Connections[i].Set(lines[i]);
            }
        }

        private static void Route(bool[] lines, bool active, ulong node)
        {
            if(active && node < (ulong)lines.Length)
            {
                lines[node] = true;
            }
        }

        private readonly uint peripheralClockFrequency;

        private IValueRegisterField fractionalDividerStep;
        private IValueRegisterField timeQuantaPreDivider;
        private IValueRegisterField timeQuantaPerBit;
        private IValueRegisterField baudRatePreDivider;
        private IValueRegisterField transmitShiftNode;
        private IValueRegisterField transmitBufferNode;
        private IValueRegisterField receiveNode;
        private IValueRegisterField alternativeReceiveNode;
        private IValueRegisterField mode;
        private IValueRegisterField parityMode;
        private IFlagRegisterField stopBits;
        private IFlagRegisterField transmitShiftInterruptEnable;
        private IFlagRegisterField transmitBufferInterruptEnable;
        private IFlagRegisterField receiveInterruptEnable;
        private IFlagRegisterField alternativeReceiveInterruptEnable;
        private IFlagRegisterField transmitShiftIndication;
        private IFlagRegisterField transmitBufferIndication;
        private IFlagRegisterField receiveIndication;
        private IFlagRegisterField alternativeReceiveIndication;

        private const int ServiceRequestLines = 6;
        private const int TransmitWindowSize = 32;
        private const uint DefaultBaudRate = 115200;
        private const ulong AsynchronousSerialChannelMode = 2;

        private enum Registers : long
        {
            KernelStateConfiguration = 0x0C,
            FractionalDivider = 0x10,
            BaudRateGenerator = 0x14,
            InterruptNodePointer = 0x18,
            Input0Control = 0x1C,
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
            FlagModification = 0x68,
            TransmitBuffer0 = 0x80,
            Bypass = 0x100,
            OutputDebug = 0x120,
            FifoInput0 = 0x180,
        }
    }
}
