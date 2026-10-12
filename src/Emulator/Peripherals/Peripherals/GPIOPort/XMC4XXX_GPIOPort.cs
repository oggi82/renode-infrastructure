//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.GPIOPort
{
    // A single port (Pn) of the Infineon XMC4500, e.g. P1 is mapped at 0x48028100 (reference manual v1.6, chapter 25 "PORTS").
    //
    // Pin model: the pad level of a pin is
    //   - the output level, if the pin is a general purpose output (PC = 0b10000 push-pull, 0b11000 open-drain)
    //     or an alternate output (PC = 0b1x001 - 0b1x100) whose level was provided with the alternate output inputs,
    //   - otherwise the level driven from outside (OnGPIO) or, if nothing drives the pin, the level of the internal
    //     pull device (pull-up: 1, pull-down and none: 0).
    // The open-drain level 1 is a released line, it follows the external level (default 1).
    //
    // GPIO connections (outputs):
    //   0 .. n-1    the level the port drives onto the pin (LEDs and other board components are connected here).
    //   n .. 2n-1   the level the on-chip peripherals see at the pin, i.e. the pad level, inverted if PC selects an inverted input
    //               (used to route pins to ERU inputs, CCU/USIC input signals etc.). Inputs of P14/P15 read 0 while PDISC disables the pad.
    // GPIO inputs (OnGPIO):
    //   0 .. n-1    the level driven onto the pad from outside (buttons, test stimulus).
    //   alt*n + pin the level of alternate output function alt (1-4) of the pin as delivered by the on-chip peripheral that owns it
    //               (see SetAlternateOutput); it reaches the pad only while IOCR selects that alternate function.
    // Hardware control (HWSEL), pad driver and pin power save settings are only stored.
    public class XMC4XXX_GPIOPort : BaseGPIOPort, IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize
    {
        public XMC4XXX_GPIOPort(IMachine machine, int numberOfPins = 16, uint pinDisableResetValue = 0, bool analogPort = false, uint hwselResetValue = 0)
            : base(machine, 2 * numberOfPins)
        {
            this.numberOfPins = numberOfPins;
            this.pinDisableResetValue = pinDisableResetValue;
            this.analogPort = analogPort;
            this.hwselResetValue = hwselResetValue;
            outputLatch = new bool[RegisterPins];
            pinControl = new byte[RegisterPins];
            externalLevel = new bool[RegisterPins];
            externallyDriven = new bool[RegisterPins];
            alternateOutput = new bool[RegisterPins, AlternateFunctions + 1];
            pinDisable = pinDisableResetValue;
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
            for(var i = 0; i < RegisterPins; i++)
            {
                outputLatch[i] = false;
                pinControl[i] = 0;
                externalLevel[i] = false;
                externallyDriven[i] = false;
                for(var alt = 0; alt <= AlternateFunctions; alt++)
                {
                    alternateOutput[i, alt] = false;
                }
            }
            pinDisable = pinDisableResetValue;
            UpdateAllPins();
        }

        // Pad level driven from outside (buttons, stimulus) for pins 0 .. n-1, level of an alternate output function for the numbers above.
        public override void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= (AlternateFunctions + 1) * numberOfPins)
            {
                this.Log(LogLevel.Error, "This port supports gpio inputs from 0 to {0}, but {1} was called", (AlternateFunctions + 1) * numberOfPins - 1, number);
                return;
            }
            if(number < numberOfPins)
            {
                externalLevel[number] = value;
                externallyDriven[number] = true;
                State[number] = value;
                UpdatePin(number);
            }
            else
            {
                SetAlternateOutput(number % numberOfPins, number / numberOfPins, value);
            }
        }

        // Entry point for on-chip peripherals: sets the level of alternate output function 'function' (1-4) of 'pin';
        // the pad takes the level while Pn_IOCR selects that function for the pin.
        public void SetAlternateOutput(int pin, int function, bool value)
        {
            if(pin < 0 || pin >= numberOfPins || function < 1 || function > AlternateFunctions)
            {
                this.Log(LogLevel.Error, "Invalid alternate output: pin {0}, function {1}", pin, function);
                return;
            }
            alternateOutput[pin, function] = value;
            UpdatePin(pin);
        }

        // The level at the pad of the pin as read from Pn_IN.
        public bool GetPadLevel(int pin) => pin >= 0 && pin < RegisterPins && PadLevel(pin);

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public long Size => 0x100;

        private void DefineRegisters()
        {
            Registers.Output.Define(this)
                .WithFlags(0, 16, name: "Pn",
                    writeCallback: (i, _, value) => SetOutputLatch(i, value),
                    valueProviderCallback: (i, _) => outputLatch[i])
                .WithReservedBits(16, 16);

            Registers.OutputModification.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PRn/PSn", writeCallback: (_, value) =>
                {
                    for(var i = 0; i < RegisterPins; i++)
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
                            valueProviderCallback: _ => pinControl[pin]);
                }
            }

            Registers.Input.Define(this)
                .WithFlags(0, 16, FieldMode.Read, name: "Pn", valueProviderCallback: (i, _) => PadLevel(i))
                .WithReservedBits(16, 16);

            // P14 and P15 have no pad driver mode registers
            if(!analogPort)
            {
                Registers.PadDriver0.Define(this, 0x22222222)
                    .WithValueField(0, 32, name: "PDR0");
                Registers.PadDriver1.Define(this, 0x22222222)
                    .WithValueField(0, 32, name: "PDR1");
            }

            // Digital ports: read-only, reflects the pins that are not bonded out; analog ports: PDISx disables the digital pad input
            Registers.PinFunctionDecision.Define(this, pinDisableResetValue)
                .WithFlags(0, 16, name: "PDISC",
                    valueProviderCallback: (i, _) => (pinDisable & (1u << i)) != 0,
                    writeCallback: (i, _, value) =>
                    {
                        if(analogPort)
                        {
                            pinDisable = value ? pinDisable | (1u << i) : pinDisable & ~(1u << i);
                            UpdatePin(i);
                        }
                    })
                .WithReservedBits(16, 16);

            Registers.PinPowerSave.Define(this)
                .WithValueField(0, 32, name: "PPS");
            Registers.PinHardwareSelect.Define(this, hwselResetValue)
                .WithValueField(0, 32, name: "HWSEL");
        }

        private void SetOutputLatch(int pin, bool value)
        {
            outputLatch[pin] = value;
            UpdatePin(pin);
        }

        private void SetPinControl(int pin, byte value)
        {
            pinControl[pin] = value;
            UpdatePin(pin);
        }

        // PC[4:0]: 0b0xxxx are inputs (bits 1:0 select the pull device, bit 2 inverts the input, 0bx0011 samples the input into Pn_OUT),
        // 0b10000 is a push-pull and 0b11000 an open-drain general purpose output, 0b1x001 - 0b1x100 are alternate output functions
        private bool IsOutput(int pin) => (pinControl[pin] & 0x10) != 0;

        private bool IsOpenDrain(int pin) => (pinControl[pin] & 0x08) != 0;

        private int AlternateFunction(int pin) => IsOutput(pin) ? pinControl[pin] & 0x07 : 0;

        private bool IsInvertedInput(int pin) => !IsOutput(pin) && (pinControl[pin] & 0x04) != 0;

        private bool IsPullUp(int pin) => !IsOutput(pin) && (pinControl[pin] & 0x03) == 2;

        private bool ExternalLevel(int pin) => externallyDriven[pin] ? externalLevel[pin] : IsPullUp(pin);

        // The level an output driver delivers, an open-drain output only pulls low
        private bool OutputLevel(int pin)
        {
            var alternate = AlternateFunction(pin);
            if(alternate > AlternateFunctions)
            {
                return false; // reserved function
            }
            var level = alternate == 0 ? outputLatch[pin] : alternateOutput[pin, alternate];
            return IsOpenDrain(pin) ? level && (!externallyDriven[pin] || externalLevel[pin]) : level;
        }

        private bool PadLevel(int pin)
        {
            if(analogPort && (pinDisable & (1u << pin)) != 0)
            {
                return false;
            }
            return IsOutput(pin) ? OutputLevel(pin) : ExternalLevel(pin);
        }

        private void UpdateAllPins()
        {
            for(var pin = 0; pin < RegisterPins; pin++)
            {
                UpdatePin(pin);
            }
        }

        private void UpdatePin(int pin)
        {
            var pad = PadLevel(pin);
            // PC = 0bx0011 / 0bx0111: Pn_OUTx continuously samples the input value
            if(!IsOutput(pin) && (pinControl[pin] & 0x03) == 3)
            {
                outputLatch[pin] = pad;
            }
            if(pin >= numberOfPins)
            {
                return;
            }
            Connections[pin].Set(IsOutput(pin) && OutputLevel(pin));
            Connections[numberOfPins + pin].Set(pad != IsInvertedInput(pin));
        }

        private const int RegisterPins = 16;
        private const int AlternateFunctions = 4;

        private readonly bool[] outputLatch;
        private readonly byte[] pinControl;
        private readonly bool[] externalLevel;
        private readonly bool[] externallyDriven;
        private readonly bool[,] alternateOutput;
        private readonly int numberOfPins;
        private readonly uint pinDisableResetValue;
        private readonly bool analogPort;
        private readonly uint hwselResetValue;
        private uint pinDisable;

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
