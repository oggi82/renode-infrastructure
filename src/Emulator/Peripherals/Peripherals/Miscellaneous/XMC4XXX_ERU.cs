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

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // Event Request Unit (ERU0 at 0x50004800, ERU1 at 0x40044000) of the Infineon XMC4500 (reference manual v1.6, chapter 4.5 and 4.10.2).
    // Four input channels x (ERSx: event request select, ETLx: event trigger logic) feed four output gating units y (OGUy).
    //
    // GPIO inputs (the signals A0-A3 and B0-B3 of the channels, see tables 4-8 and 4-9 of the reference manual):
    //   x*8 + 0..3    ERU_xA0 - ERU_xA3
    //   x*8 + 4..7    ERU_xB0 - ERU_xB3
    //   32 + y*4 + k  peripheral trigger ERU_OGUyk (k = 1..3, selected by EXOCONy.ISS; k = 3 is tied to 1 in the device and has no input)
    // GPIO outputs (Connections):
    //   y             ERU_IOUTy  gated trigger, the service request (a pulse); ERU0: NVIC 1-4, ERU1: NVIC 5-8
    //   4 + y         ERU_PDOUTy pattern match
    //   8 + y         ERU_GOUTy  gating signal
    //   12 + y        ERU_TOUTy  trigger (a pulse)
    // With internalLoopback (ERU1) the IOUT signals are fed back to the inputs ERU1.0B3 (IOUT3), ERU1.1A3 and ERU1.1B3 (IOUT2).
    //
    // Simplifications: signals are sampled without synchronizer delay and the output pulses have zero length. Writing EXISEL or the ERS
    // fields of EXICONx (SS, NA, NB) and the pattern fields of EXOCONy never raises an event, only input changes, software writes of the
    // status flag FL do. A peripheral trigger is the rising edge of the selected ERU_OGUyk signal.
    public class XMC4XXX_ERU : IDoubleWordPeripheral, IKnownSize, IGPIOReceiver, INumberedGPIOOutput, IProvidesRegisterCollection<DoubleWordRegisterCollection>
    {
        public XMC4XXX_ERU(IMachine machine, bool internalLoopback = false)
        {
            this.internalLoopback = internalLoopback;

            var connections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < 4 * OutputsPerKind; i++)
            {
                connections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(connections);

            inputs = new bool[Channels * 8];
            ersOutput = new bool[Channels];
            statusFlag = new bool[Channels];
            patternMatch = new bool[Channels];
            peripheralTrigger = new bool[Channels, 4];

            pulseEnable = new IFlagRegisterField[Channels];
            levelDetection = new IFlagRegisterField[Channels];
            risingEdge = new IFlagRegisterField[Channels];
            fallingEdge = new IFlagRegisterField[Channels];
            outputChannel = new IValueRegisterField[Channels];
            sourceSelect = new IValueRegisterField[Channels];
            negateA = new IFlagRegisterField[Channels];
            negateB = new IFlagRegisterField[Channels];
            internalSource = new IValueRegisterField[Channels];
            gatingEvent = new IFlagRegisterField[Channels];
            gatingSelect = new IValueRegisterField[Channels];
            patternEnable = new IFlagRegisterField[Channels, Channels];

            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
        }

        public void Reset()
        {
            foreach(var connection in Connections.Values)
            {
                connection.Unset();
            }
            for(var i = 0; i < inputs.Length; i++)
            {
                inputs[i] = false;
            }
            for(var i = 0; i < Channels; i++)
            {
                statusFlag[i] = false;
                selectedA[i] = 0;
                selectedB[i] = 0;
                for(var k = 0; k < 4; k++)
                {
                    peripheralTrigger[i, k] = false;
                }
            }
            RegistersCollection.Reset();
            for(var i = 0; i < Channels; i++)
            {
                ersOutput[i] = EvaluateEventRequestSelect(i);
                patternMatch[i] = EvaluatePattern(i);
            }
            UpdatePatternOutputs();
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            RegistersCollection.Write(offset, value);
        }

        public void OnGPIO(int number, bool value)
        {
            if(number >= 0 && number < Channels * 8)
            {
                SetInput(number, value);
            }
            else if(number >= 32 && number < 32 + 4 * Channels && (number & 3) != 0)
            {
                SetPeripheralTrigger((number - 32) >> 2, number & 3, value);
            }
            else
            {
                this.Log(LogLevel.Error, "Invalid gpio input {0}, valid are 0-{1} and {2}-{3}", number, Channels * 8 - 1, 32, 32 + 4 * Channels - 1);
            }
        }

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public long Size => 0x100;

        private void DefineRegisters()
        {
            Registers.InputSelect.Define(this)
                .WithValueField(0, 16, name: "EXS", writeCallback: (_, value) =>
                {
                    for(var channel = 0; channel < Channels; channel++)
                    {
                        selectedA[channel] = (int)((value >> (4 * channel)) & 3);
                        selectedB[channel] = (int)((value >> (4 * channel + 2)) & 3);
                    }
                    ReevaluateInputsSilently();
                }, valueProviderCallback: _ =>
                {
                    var result = 0u;
                    for(var channel = 0; channel < Channels; channel++)
                    {
                        result |= (uint)selectedA[channel] << (4 * channel);
                        result |= (uint)selectedB[channel] << (4 * channel + 2);
                    }
                    return result;
                })
                .WithReservedBits(16, 16);

            for(var channel = 0; channel < Channels; channel++)
            {
                var x = channel;
                (Registers.InputControl0 + 4 * x).Define(this)
                    .WithFlag(0, out pulseEnable[x], name: "PE")
                    .WithFlag(1, out levelDetection[x], name: "LD")
                    .WithFlag(2, out risingEdge[x], name: "RE")
                    .WithFlag(3, out fallingEdge[x], name: "FE")
                    .WithValueField(4, 3, out outputChannel[x], name: "OCS")
                    .WithFlag(7, name: "FL",
                        valueProviderCallback: _ => statusFlag[x],
                        writeCallback: (_, value) =>
                        {
                            if(statusFlag[x] != value)
                            {
                                statusFlag[x] = value;
                                UpdatePatternOutputs(true);
                            }
                        })
                    .WithValueField(8, 2, out sourceSelect[x], name: "SS", writeCallback: (_, __) => ReevaluateInputsSilently())
                    .WithFlag(10, out negateA[x], name: "NA", writeCallback: (_, __) => ReevaluateInputsSilently())
                    .WithFlag(11, out negateB[x], name: "NB", writeCallback: (_, __) => ReevaluateInputsSilently())
                    .WithReservedBits(12, 20);
            }

            for(var gate = 0; gate < Channels; gate++)
            {
                var y = gate;
                (Registers.OutputControl0 + 4 * y).Define(this, 0x8)
                    .WithValueField(0, 2, out internalSource[y], name: "ISS")
                    .WithFlag(2, out gatingEvent[y], name: "GEEN")
                    .WithFlag(3, FieldMode.Read, name: "PDR", valueProviderCallback: _ => patternMatch[y])
                    .WithValueField(4, 2, out gatingSelect[y], name: "GP", writeCallback: (_, __) => UpdatePatternOutputs())
                    .WithReservedBits(6, 6)
                    .WithFlag(12, out patternEnable[y, 0], name: "IPEN0", writeCallback: (_, __) => UpdatePatternOutputs())
                    .WithFlag(13, out patternEnable[y, 1], name: "IPEN1", writeCallback: (_, __) => UpdatePatternOutputs())
                    .WithFlag(14, out patternEnable[y, 2], name: "IPEN2", writeCallback: (_, __) => UpdatePatternOutputs())
                    .WithFlag(15, out patternEnable[y, 3], name: "IPEN3", writeCallback: (_, __) => UpdatePatternOutputs())
                    .WithReservedBits(16, 16);
            }
        }

        private void SetInput(int number, bool value)
        {
            if(inputs[number] == value)
            {
                return;
            }
            inputs[number] = value;
            var channel = number >> 3;
            var newOutput = EvaluateEventRequestSelect(channel);
            if(newOutput == ersOutput[channel])
            {
                return;
            }
            ersOutput[channel] = newOutput;
            DetectEdge(channel, newOutput);
        }

        private void SetPeripheralTrigger(int gate, int source, bool value)
        {
            var rising = value && !peripheralTrigger[gate, source];
            peripheralTrigger[gate, source] = value;
            if(rising && (int)internalSource[gate].Value == source)
            {
                RaiseTrigger(gate);
            }
        }

        // ERSxO: input A (EXISEL.EXSxA) and B (EXSxB), optionally negated, combined as selected by EXICONx.SS
        private bool EvaluateEventRequestSelect(int channel)
        {
            var a = inputs[channel * 8 + selectedA[channel]] != negateA[channel].Value;
            var b = inputs[channel * 8 + 4 + selectedB[channel]] != negateB[channel].Value;
            switch(sourceSelect[channel].Value)
            {
                case 0:
                    return a;
                case 1:
                    return b;
                case 2:
                    return a || b;
                default:
                    return a && b;
            }
        }

        // A change of the ERS configuration only follows the new level, it is not an event
        private void ReevaluateInputsSilently()
        {
            for(var channel = 0; channel < Channels; channel++)
            {
                ersOutput[channel] = EvaluateEventRequestSelect(channel);
            }
        }

        private void DetectEdge(int channel, bool rising)
        {
            var isEvent = rising ? risingEdge[channel].Value : fallingEdge[channel].Value;
            var statusChanged = false;
            if(isEvent)
            {
                statusChanged = !statusFlag[channel];
                statusFlag[channel] = true;
            }
            else if(levelDetection[channel].Value)
            {
                // the opposite edge rebuilds the level of the desired event
                statusChanged = statusFlag[channel];
                statusFlag[channel] = false;
            }
            if(statusChanged)
            {
                UpdatePatternOutputs(true);
            }
            if(isEvent && pulseEnable[channel].Value && outputChannel[channel].Value < Channels)
            {
                RaiseTrigger((int)outputChannel[channel].Value);
            }
        }

        // The pattern match is true while all flags enabled in IPENx are set (also if none is enabled)
        private bool EvaluatePattern(int gate)
        {
            for(var channel = 0; channel < Channels; channel++)
            {
                if(patternEnable[gate, channel].Value && !statusFlag[channel])
                {
                    return false;
                }
            }
            return true;
        }

        private bool AnyPatternInput(int gate)
        {
            for(var channel = 0; channel < Channels; channel++)
            {
                if(patternEnable[gate, channel].Value)
                {
                    return true;
                }
            }
            return false;
        }

        // Recomputes PDR and the PDOUT/GOUT signals of all OGUs; a pattern result change is a trigger event if enabled by GEEN
        private void UpdatePatternOutputs(bool statusFlagChanged = false)
        {
            for(var gate = 0; gate < Channels; gate++)
            {
                var match = EvaluatePattern(gate);
                var changed = match != patternMatch[gate];
                patternMatch[gate] = match;
                Connections[OutputsPerKind + gate].Set(match);
                Connections[2 * OutputsPerKind + gate].Set(IsGatingOpen(gate));
                if(changed && statusFlagChanged && gatingEvent[gate].Value && AnyPatternInput(gate))
                {
                    RaiseTrigger(gate);
                }
            }
        }

        // GP: 0 - GOUT = 0, 1 - GOUT = 1, 2 - GOUT = PDOUT, 3 - GOUT = !PDOUT
        private bool IsGatingOpen(int gate)
        {
            switch(gatingSelect[gate].Value)
            {
                case 0:
                    return false;
                case 1:
                    return true;
                case 2:
                    return patternMatch[gate];
                default:
                    return !patternMatch[gate];
            }
        }

        // ERU_TOUTy pulses with every trigger, ERU_IOUTy only while the gating allows it
        private void RaiseTrigger(int gate)
        {
            var gated = IsGatingOpen(gate);
            this.Log(LogLevel.Noisy, "OGU{0} trigger, gating {1}", gate, gated ? "open" : "closed");
            Connections[3 * OutputsPerKind + gate].Set(true);
            Connections[3 * OutputsPerKind + gate].Set(false);
            if(!gated)
            {
                return;
            }
            Connections[gate].Set(true);
            if(internalLoopback)
            {
                FeedBack(gate, true);
            }
            Connections[gate].Set(false);
            if(internalLoopback)
            {
                FeedBack(gate, false);
            }
        }

        // ERU1.0B3 = IOUT3, ERU1.1A3 = IOUT2, ERU1.1B3 = IOUT2
        private void FeedBack(int gate, bool value)
        {
            switch(gate)
            {
                case 3:
                    SetInput(0 * 8 + 4 + 3, value);
                    break;
                case 2:
                    SetInput(1 * 8 + 3, value);
                    SetInput(1 * 8 + 4 + 3, value);
                    break;
            }
        }

        private const int Channels = 4;
        private const int OutputsPerKind = 4;

        private readonly bool internalLoopback;
        private readonly bool[] inputs;
        private readonly bool[] ersOutput;
        private readonly bool[] statusFlag;
        private readonly bool[] patternMatch;
        private readonly bool[,] peripheralTrigger;
        private readonly int[] selectedA = new int[Channels];
        private readonly int[] selectedB = new int[Channels];

        private readonly IFlagRegisterField[] pulseEnable;
        private readonly IFlagRegisterField[] levelDetection;
        private readonly IFlagRegisterField[] risingEdge;
        private readonly IFlagRegisterField[] fallingEdge;
        private readonly IValueRegisterField[] outputChannel;
        private readonly IValueRegisterField[] sourceSelect;
        private readonly IFlagRegisterField[] negateA;
        private readonly IFlagRegisterField[] negateB;
        private readonly IValueRegisterField[] internalSource;
        private readonly IFlagRegisterField[] gatingEvent;
        private readonly IValueRegisterField[] gatingSelect;
        private readonly IFlagRegisterField[,] patternEnable;

        private enum Registers : long
        {
            InputSelect = 0x00,
            InputControl0 = 0x10,
            InputControl1 = 0x14,
            InputControl2 = 0x18,
            InputControl3 = 0x1C,
            OutputControl0 = 0x20,
            OutputControl1 = 0x24,
            OutputControl2 = 0x28,
            OutputControl3 = 0x2C,
        }
    }
}
