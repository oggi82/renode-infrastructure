//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // Position Interface (POSIF0/POSIF1) of the Infineon XMC4xxx family. Reference Manual v1.6, chapter 24.
    //
    // Modeled: the hall sensor mode (edge detection on the three hall inputs, sampling on the delay trigger HSD, comparison with the
    // current/expected pattern, correct/wrong hall event, idle generation, external wrong hall event, shadow transfer of the patterns),
    // the multi-channel pattern (update enable by MSET or software, update on the MSYNC falling edge, MOUT outputs), the quadrature
    // decoder mode (phase decoding with swap and level selection, quadrature and period clock, direction, phase error, index) and the
    // direction count mode, the run bit with the synchronous start pulse, the interrupt flags with the two service request lines.
    // All decoding is combinatorial, the POSIF clock is not simulated.
    // Not modeled: the input low pass filters (PCONF.LPC, only stored), suspend, the exact index edge memory of the quadrature decoder
    // (the index is evaluated at the next valid phase transition).
    //
    // Connections: 0-6 = OUT0-OUT6, 7-8 = service request lines SR0 and SR1 (one-event pulses), 16-31 = MOUT0-MOUT15.
    // OUT0-OUT5 are pulses (a rising and a falling edge) except OUT1 in quadrature mode (direction level) and OUT6 (multi-channel
    // update enable level).
    // Inputs (OnGPIO): 0-3 IN0A-D (hall input 1 / phase A), 4-7 IN1A-D (hall input 2 / phase B), 8-11 IN2A-D (hall input 3 / index),
    // 12-15 HSDA-D (hall sampling delay, the ST0 of the CCU4 slice 0), 16-19 EWHEA-D, 20-27 MSETA-H, 28-31 MSYNCA-D.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class XMC4XXX_POSIF : BasicDoubleWordPeripheral, IKnownSize, INumberedGPIOOutput, IGPIOReceiver
    {
        public XMC4XXX_POSIF(IMachine machine) : base(machine)
        {
            var connections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < OutputCount + ServiceRequestCount; i++)
            {
                connections[i] = new GPIO();
            }
            for(var i = 0; i < MultiChannelOutputCount; i++)
            {
                connections[MultiChannelOutputBase + i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(connections);
            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            pconf = 0;
            suspend = 0;
            running = false;
            Array.Clear(inputs, 0, inputs.Length);
            currentHallPattern = 0;
            expectedHallPattern = 0;
            shadowHallPattern = 0;
            sampledHallPattern = 0;
            multiChannelPattern = 0;
            multiChannelShadowPattern = 0;
            updateEnabled = false;
            correctHallEvent = false;
            samplePending = false;
            quadratureControl = 0;
            interruptFlags = 0;
            interruptEnable = 0;
            ResetDecoder();
            foreach(var connection in Connections.Values)
            {
                connection.Unset();
            }
        }

        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= InputCount)
            {
                this.Log(LogLevel.Warning, "Input {0} does not exist", number);
                return;
            }
            if(inputs[number] == value)
            {
                return;
            }
            inputs[number] = value;

            switch(number / 4)
            {
            case 0:
            case 1:
            case 2:
                OnDataInputChanged(number / 4, number % 4, value);
                break;
            case 3:
                OnSampleDelayInput(number % 4, value);
                break;
            case 4:
                OnExternalWrongHallInput(number % 4, value);
                break;
            case 5:
            case 6:
                OnPatternUpdateInput(number - 20, value);
                break;
            default:
                OnSynchronizationInput(number - 28, value);
                break;
            }
        }

        public long Size => 0x4000;

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        private void DefineRegisters()
        {
            Registers.Configuration.Define(this)
                .WithValueField(0, 32, name: "PCONF", valueProviderCallback: _ => pconf, writeCallback: (_, value) =>
                {
                    pconf = (uint)value & 0x7FFF3F37;
                    ResetDecoderInputs();
                });

            Registers.Suspend.Define(this)
                .WithValueField(0, 4, name: "SUSPEND", valueProviderCallback: _ => suspend, writeCallback: (_, value) => suspend = (uint)value)
                .WithReservedBits(4, 28);

            Registers.RunSet.Define(this)
                .WithFlag(0, FieldMode.Write, name: "SRB", writeCallback: (_, value) => { if(value) Start(); })
                .WithReservedBits(1, 31);

            Registers.RunClear.Define(this)
                .WithFlag(0, FieldMode.Write, name: "CRB", writeCallback: (_, value) => { if(value) running = false; })
                .WithFlag(1, FieldMode.Write, name: "CSM", writeCallback: (_, value) => { if(value) ClearCurrentStatus(); })
                .WithReservedBits(2, 30);

            Registers.RunStatus.Define(this)
                .WithFlag(0, FieldMode.Read, name: "RB", valueProviderCallback: _ => running)
                .WithReservedBits(1, 31);

            // Module identification: module number 0xA8 (POSIF), type 0xC0, revision 0x08 (POSIF V1.8)
            Registers.ModuleIdentification.Define(this, 0x00A8C008)
                .WithValueField(0, 32, FieldMode.Read, name: "MIDR");

            Registers.HallPatterns.Define(this)
                .WithValueField(0, 3, FieldMode.Read, name: "HCP", valueProviderCallback: _ => currentHallPattern)
                .WithValueField(3, 3, FieldMode.Read, name: "HEP", valueProviderCallback: _ => expectedHallPattern)
                .WithReservedBits(6, 26);

            Registers.HallShadowPatterns.Define(this)
                .WithValueField(0, 3, name: "HCPS", valueProviderCallback: _ => shadowHallPattern & 7, writeCallback: (_, value) => shadowHallPattern = (shadowHallPattern & ~7u) | (uint)value)
                .WithValueField(3, 3, name: "HEPS", valueProviderCallback: _ => shadowHallPattern >> 3, writeCallback: (_, value) => shadowHallPattern = (shadowHallPattern & 7u) | ((uint)value << 3))
                .WithReservedBits(6, 26);

            Registers.MultiChannelPattern.Define(this)
                .WithValueField(0, 16, FieldMode.Read, name: "MCMP", valueProviderCallback: _ => multiChannelPattern)
                .WithReservedBits(16, 16);

            Registers.MultiChannelShadowPattern.Define(this)
                .WithValueField(0, 16, name: "MCMPS", valueProviderCallback: _ => multiChannelShadowPattern, writeCallback: (_, value) => multiChannelShadowPattern = (uint)value)
                .WithReservedBits(16, 16);

            Registers.MultiChannelControlSet.Define(this)
                .WithFlag(0, FieldMode.Write, name: "MNPS", writeCallback: (_, value) => { if(value) SetUpdateEnabled(true); })
                .WithFlag(1, FieldMode.Write, name: "STHR", writeCallback: (_, value) => { if(value) TransferHallPatterns(); })
                .WithFlag(2, FieldMode.Write, name: "STMR", writeCallback: (_, value) => { if(value) TransferMultiChannelPattern(); })
                .WithReservedBits(3, 29);

            Registers.MultiChannelControlClear.Define(this)
                .WithFlag(0, FieldMode.Write, name: "MNPC", writeCallback: (_, value) => { if(value) SetUpdateEnabled(false); })
                .WithFlag(1, FieldMode.Write, name: "MPC", writeCallback: (_, value) => { if(value) SetMultiChannelPattern(0); })
                .WithReservedBits(2, 30);

            Registers.MultiChannelControlFlag.Define(this)
                .WithFlag(0, FieldMode.Read, name: "MSS", valueProviderCallback: _ => updateEnabled)
                .WithReservedBits(1, 31);

            Registers.QuadratureDecoderControl.Define(this)
                .WithValueField(0, 32, name: "QDC", valueProviderCallback: _ => quadratureControl | (direction ? 1u << 8 : 0u),
                    writeCallback: (_, value) => { quadratureControl = (uint)value & 0x37; ResetDecoderInputs(); });

            Registers.InterruptFlags.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "PFLG", valueProviderCallback: _ => interruptFlags);

            Registers.InterruptEnable.Define(this)
                .WithValueField(0, 32, name: "PFLGE", valueProviderCallback: _ => interruptEnable, writeCallback: (_, value) => interruptEnable = (uint)value & FlagEnableMask);

            Registers.InterruptSet.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "SPFLG", writeCallback: (_, value) =>
                {
                    for(var bit = 0; bit < 13; bit++)
                    {
                        if((FlagMask & (1u << bit)) != 0 && (value & (1ul << bit)) != 0)
                        {
                            RaiseFlag(1u << bit);
                        }
                    }
                });

            Registers.InterruptClear.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "RPFLG", writeCallback: (_, value) => interruptFlags &= ~((uint)value & FlagMask));

            Registers.Debug.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "PDBG", valueProviderCallback: _ =>
                    (uint)phaseState | ((uint)previousPhaseState << 2) | (indexLevel ? 1u << 4 : 0u) | ((sampledHallPattern & 7) << 5));
        }

        private int Mode => (int)(pconf & 3);

        private bool HallMode => Mode == 0;

        private bool QuadratureMode => (Mode & 1) != 0;

        private bool DirectionCountMode => (pconf & (1u << 2)) != 0;

        private bool IdleDisabled => (pconf & (1u << 4)) != 0;

        private bool SoftwarePatternUpdate => (pconf & (1u << 5)) != 0;

        private int SelectedLetter(int group) => (int)((pconf >> (8 + 2 * group)) & 3);

        private bool Selected(int group) => inputs[group * 4 + SelectedLetter(group)];

        private void Start()
        {
            running = true;
            Pulse(5);
            ResetDecoderInputs();
        }

        // Resets the state of the decoder and of the hall/multi-channel logic, flags and configuration stay
        private void ClearCurrentStatus()
        {
            currentHallPattern = 0;
            expectedHallPattern = 0;
            samplePending = false;
            correctHallEvent = false;
            updateEnabled = false;
            Connections[6].Set(false);
            SetMultiChannelPattern(0);
            ResetDecoder();
        }

        private void ResetDecoder()
        {
            direction = false;
            firstIndexSeen = false;
            firstIndexDirection = false;
            indexPending = false;
            indexLevel = false;
            ResetDecoderInputs();
        }

        // Takes over the current levels of the phase inputs as decoder state
        private void ResetDecoderInputs()
        {
            phaseState = ReadPhaseState();
            previousPhaseState = phaseState;
            indexLevel = Selected(2);
            Connections[1].Set(QuadratureMode && direction);
        }

        private int ReadPhaseState()
        {
            var phaseA = Selected(0) ^ ((quadratureControl & 1) != 0);
            var phaseB = Selected(1) ^ ((quadratureControl & 2) != 0);
            if((quadratureControl & 4) != 0)
            {
                var swap = phaseA;
                phaseA = phaseB;
                phaseB = swap;
            }
            return (phaseA ? 1 : 0) | (phaseB ? 2 : 0);
        }

        private void OnDataInputChanged(int group, int letter, bool value)
        {
            if(letter != SelectedLetter(group))
            {
                return;
            }
            if(group == 2)
            {
                indexLevel = value;
            }
            if(!running)
            {
                return;
            }

            if(HallMode)
            {
                // any edge of a hall input
                SetFlagAndPulse(HallInputUpdateFlag, 0);
                samplePending = true;
            }
            else if(QuadratureMode)
            {
                OnQuadratureInput(group, value);
            }
        }

        private void OnSampleDelayInput(int letter, bool value)
        {
            var selectedLetter = (int)((pconf >> 16) & 1);
            var fallingEdge = (pconf & (1u << 17)) != 0;
            if(letter != selectedLetter || value == fallingEdge || !running || !HallMode || !samplePending)
            {
                return;
            }

            samplePending = false;
            sampledHallPattern = (Selected(0) ? 1u : 0u) | (Selected(1) ? 2u : 0u) | (Selected(2) ? 4u : 0u);
            if(sampledHallPattern == expectedHallPattern)
            {
                TransferHallPatterns();
                correctHallEvent = true;
                Pulse(1);
                RaiseFlag(HallCorrectEventFlag);
            }
            else if(sampledHallPattern == currentHallPattern)
            {
                // line glitch, only the stop signal is generated
                Pulse(3);
            }
            else
            {
                WrongHallEvent();
            }
        }

        private void OnExternalWrongHallInput(int letter, bool value)
        {
            var enabled = (pconf & (1u << 26)) != 0;
            var activeLow = (pconf & (1u << 27)) != 0;
            if(!enabled || letter != (int)((pconf >> 24) & 3) || (value == activeLow) || !running || !HallMode)
            {
                return;
            }
            WrongHallEvent();
        }

        private void WrongHallEvent()
        {
            Pulse(3);
            if(!IdleDisabled)
            {
                Pulse(2);
                running = false;
                SetMultiChannelPattern(0);
                updateEnabled = false;
                Connections[6].Set(false);
            }
            RaiseFlag(HallWrongEventFlag);
        }

        private void TransferHallPatterns()
        {
            currentHallPattern = shadowHallPattern & 7;
            expectedHallPattern = (shadowHallPattern >> 3) & 7;
        }

        private void OnPatternUpdateInput(int letter, bool value)
        {
            var fallingEdge = (pconf & (1u << 21)) != 0;
            if(letter != (int)((pconf >> 18) & 7) || value == fallingEdge || !running || SoftwarePatternUpdate)
            {
                return;
            }
            // in hall mode the update has to follow a correct hall event
            if(HallMode && !correctHallEvent)
            {
                return;
            }
            SetUpdateEnabled(true);
        }

        private void OnSynchronizationInput(int letter, bool value)
        {
            if(letter != (int)((pconf >> 22) & 3) || value || !running || !updateEnabled)
            {
                return;
            }
            TransferMultiChannelPattern();
        }

        private void SetUpdateEnabled(bool value)
        {
            updateEnabled = value;
            if(!value)
            {
                correctHallEvent = false;
            }
            Connections[6].Set(value);
        }

        private void TransferMultiChannelPattern()
        {
            SetMultiChannelPattern(multiChannelShadowPattern);
            SetUpdateEnabled(false);
            Pulse(4);
            RaiseFlag(MultiChannelTransferFlag);
        }

        private void SetMultiChannelPattern(uint pattern)
        {
            multiChannelPattern = pattern & 0xFFFF;
            for(var i = 0; i < MultiChannelOutputCount; i++)
            {
                Connections[MultiChannelOutputBase + i].Set((multiChannelPattern & (1u << i)) != 0);
            }
        }

        private void OnQuadratureInput(int group, bool value)
        {
            if(DirectionCountMode)
            {
                // IN0 is the clock, IN1 the direction
                if(group == 0 && value)
                {
                    direction = Selected(1);
                    Connections[1].Set(direction);
                    Pulse(0);
                    RaiseFlag(CountFlag);
                }
                else if(group == 1)
                {
                    var changed = direction != value;
                    direction = value;
                    Connections[1].Set(direction);
                    if(changed)
                    {
                        RaiseFlag(DirectionFlag);
                    }
                }
                return;
            }

            if(group == 2)
            {
                indexLevel = value;
                if(value)
                {
                    indexPending = true;
                }
                return;
            }

            var newState = ReadPhaseState();
            if(newState == phaseState)
            {
                return;
            }
            previousPhaseState = phaseState;
            phaseState = newState;

            if(((previousPhaseState ^ newState) & 3) == 3)
            {
                // both phases changed at once, the decoder cannot tell the direction
                RaiseFlag(ErrorFlag);
                return;
            }

            // clockwise: A leads, (A, B) = 00 -> 10 -> 11 -> 01 -> 00
            var clockwise = ClockwiseSuccessor[previousPhaseState] == newState;
            var directionChanged = clockwise != direction;
            direction = clockwise;
            Connections[1].Set(direction);
            Pulse(0);
            RaiseFlag(CountFlag);
            if(directionChanged)
            {
                RaiseFlag(DirectionFlag);
            }
            else
            {
                Pulse(2);
                RaiseFlag(PeriodClockFlag);
            }

            if(indexPending)
            {
                indexPending = false;
                OnIndex();
            }
        }

        private void OnIndex()
        {
            var markerMode = (quadratureControl >> 4) & 3;
            if(markerMode == 2 || (markerMode == 1 && !firstIndexSeen))
            {
                Pulse(3);
                RaiseFlag(IndexFlag);
            }
            if(!firstIndexSeen)
            {
                firstIndexSeen = true;
                firstIndexDirection = direction;
            }
            if(direction == firstIndexDirection)
            {
                // revolution counter output
                Pulse(4);
            }
        }

        private void SetFlagAndPulse(uint flag, int output)
        {
            Pulse(output);
            RaiseFlag(flag);
        }

        private void RaiseFlag(uint flag)
        {
            interruptFlags |= flag;
            if((interruptEnable & flag) == 0)
            {
                return;
            }
            // service request line: PFLGE bit (flag position + 16) selects SR1
            var line = (interruptEnable & (flag << 16)) != 0 ? 1 : 0;
            Pulse(OutputCount + line);
        }

        private void Pulse(int connection)
        {
            Connections[connection].Set(true);
            Connections[connection].Set(false);
        }

        private uint pconf;
        private uint suspend;
        private bool running;
        private uint currentHallPattern;
        private uint expectedHallPattern;
        private uint shadowHallPattern;
        private uint sampledHallPattern;
        private uint multiChannelPattern;
        private uint multiChannelShadowPattern;
        private bool updateEnabled;
        private bool correctHallEvent;
        private bool samplePending;
        private uint quadratureControl;
        private uint interruptFlags;
        private uint interruptEnable;
        private int phaseState;
        private int previousPhaseState;
        private bool direction;
        private bool indexLevel;
        private bool indexPending;
        private bool firstIndexSeen;
        private bool firstIndexDirection;
        private readonly bool[] inputs = new bool[InputCount];

        private const int OutputCount = 7;
        private const int ServiceRequestCount = 2;
        private const int MultiChannelOutputBase = 16;
        private const int MultiChannelOutputCount = 16;
        private const int InputCount = 32;

        // Interrupt flags (PFLG bits), the enable bit has the same position in PFLGE, the SR1 select bit is 16 higher
        private const uint HallCorrectEventFlag = 1u << 0;
        private const uint HallWrongEventFlag = 1u << 1;
        private const uint HallInputUpdateFlag = 1u << 2;
        private const uint MultiChannelTransferFlag = 1u << 4;
        private const uint IndexFlag = 1u << 8;
        private const uint ErrorFlag = 1u << 9;
        private const uint CountFlag = 1u << 10;
        private const uint DirectionFlag = 1u << 11;
        private const uint PeriodClockFlag = 1u << 12;
        private const uint FlagMask = 0x1F17;
        private const uint FlagEnableMask = FlagMask | (FlagMask << 16);

        // Successor of a phase state (bit 0 = A, bit 1 = B) when turning clockwise
        private static readonly int[] ClockwiseSuccessor = { 1, 3, 0, 2 };

        private enum Registers : long
        {
            Configuration = 0x00,
            Suspend = 0x04,
            RunSet = 0x08,
            RunClear = 0x0C,
            RunStatus = 0x10,
            ModuleIdentification = 0x20,
            HallPatterns = 0x30,
            HallShadowPatterns = 0x34,
            MultiChannelPattern = 0x40,
            MultiChannelShadowPattern = 0x44,
            MultiChannelControlSet = 0x48,
            MultiChannelControlClear = 0x4C,
            MultiChannelControlFlag = 0x50,
            QuadratureDecoderControl = 0x60,
            InterruptFlags = 0x70,
            InterruptEnable = 0x74,
            InterruptSet = 0x78,
            InterruptClear = 0x7C,
            Debug = 0x100,
        }
    }
}
