//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.GPIOPort
{
    // A single port (Pn) of the Infineon XMC4xxx family, e.g. P1 is mapped at 0x48028100.
    // Pins configured as general purpose push-pull or open-drain outputs drive the matching Connections entry
    // from the output latch, all other pins act as inputs fed through OnGPIO.
    // Alternate output functions, hardware control (HWSEL), pad driver and pull-up/down settings are only stored.
    public class XMC4XXX_GPIOPort : BaseGPIOPort, IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize
    {
        public XMC4XXX_GPIOPort(IMachine machine, int numberOfPins = 16, uint pinDisableResetValue = 0) : base(machine, numberOfPins)
        {
            this.pinDisableResetValue = pinDisableResetValue;
            outputLatch = new bool[numberOfPins];
            pinControl = new byte[numberOfPins];
            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
            Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            RegistersCollection.Write(offset, value);
        }

        public override void Reset()
        {
            base.Reset();
            RegistersCollection.Reset();
            for(var i = 0; i < NumberOfConnections; i++)
            {
                outputLatch[i] = false;
                pinControl[i] = 0;
            }
        }

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public long Size => 0x100;

        private void DefineRegisters()
        {
            Registers.Output.Define(this)
                .WithFlags(0, 16, name: "Pn",
                    writeCallback: (i, _, value) => SetOutputLatch(i, value),
                    valueProviderCallback: (i, _) => i < NumberOfConnections && outputLatch[i])
                .WithReservedBits(16, 16);

            Registers.OutputModification.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PSn/PRn", writeCallback: (_, value) =>
                {
                    for(var i = 0; i < NumberOfConnections; i++)
                    {
                        var set = (value & (1u << i)) != 0;
                        var reset = (value & (1u << (i + 16))) != 0;
                        if(set && reset)
                        {
                            SetOutputLatch(i, !outputLatch[i]);
                        }
                        else if(set || reset)
                        {
                            SetOutputLatch(i, set);
                        }
                    }
                });

            for(var register = 0; register < 4; register++)
            {
                var iocr = (Registers)((long)Registers.InputOutputControl0 + 4 * register);
                var definition = iocr.Define(this);
                for(var field = 0; field < 4; field++)
                {
                    var pin = register * 4 + field;
                    definition
                        .WithReservedBits(field * 8, 3)
                        .WithValueField(field * 8 + 3, 5, name: $"PC{pin}",
                            writeCallback: (_, value) => SetPinControl(pin, (byte)value),
                            valueProviderCallback: _ => pin < NumberOfConnections ? (ulong)pinControl[pin] : 0);
                }
            }

            Registers.Input.Define(this)
                .WithFlags(0, 16, FieldMode.Read, name: "Pn", valueProviderCallback: (i, _) => i < NumberOfConnections && GetPinLevel(i))
                .WithReservedBits(16, 16);

            Registers.PadDriver0.Define(this)
                .WithValueField(0, 32, name: "PDR0");
            Registers.PadDriver1.Define(this)
                .WithValueField(0, 32, name: "PDR1");
            Registers.PinFunctionDecision.Define(this, pinDisableResetValue)
                .WithValueField(0, 32, name: "PDISC");
            Registers.PinPowerSave.Define(this)
                .WithValueField(0, 32, name: "PPS");
            Registers.PinHardwareSelect.Define(this)
                .WithValueField(0, 32, name: "HWSEL");
        }

        private void SetOutputLatch(int pin, bool value)
        {
            if(pin >= NumberOfConnections)
            {
                return;
            }
            outputLatch[pin] = value;
            UpdateOutput(pin);
        }

        private void SetPinControl(int pin, byte value)
        {
            if(pin >= NumberOfConnections)
            {
                return;
            }
            pinControl[pin] = value;
            UpdateOutput(pin);
        }

        // PC[4:0]: 0b00000 - 0b01111 are inputs, 0b10000 is a push-pull and 0b11000 an open-drain general purpose output, other 0b1xxxx values are alternate functions
        private bool IsGeneralPurposeOutput(int pin) => pinControl[pin] == 0x10 || pinControl[pin] == 0x18;

        private bool GetPinLevel(int pin) => IsGeneralPurposeOutput(pin) ? outputLatch[pin] : State[pin];

        private void UpdateOutput(int pin)
        {
            Connections[pin].Set(IsGeneralPurposeOutput(pin) && outputLatch[pin]);
        }

        private readonly bool[] outputLatch;
        private readonly byte[] pinControl;
        private readonly uint pinDisableResetValue;

        private enum Registers : long
        {
            Output = 0x00,
            OutputModification = 0x04,
            InputOutputControl0 = 0x10,
            InputOutputControl4 = 0x14,
            InputOutputControl8 = 0x18,
            InputOutputControl12 = 0x1C,
            Input = 0x24,
            PadDriver0 = 0x40,
            PadDriver1 = 0x44,
            PinFunctionDecision = 0x60,
            PinPowerSave = 0x70,
            PinHardwareSelect = 0x74,
        }
    }
}
