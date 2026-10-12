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
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    // Capture/Compare Unit 4 (CCU4) of the Infineon XMC4xxx family, one instance per unit (CCU40..CCU43, four slices each).
    // Reference Manual v1.6, chapter 22.
    //
    // Modeled: compare mode (edge and center aligned, single shot), 16 bit period/compare with shadow transfer,
    // normal prescaler (fTCLK = fCCU / 2^PSIV), run/idle control (GIDLS/GIDLC, prescaler run bit), interrupt flags and the
    // four service request lines, the status bit (ST) and the slice output (OUT = ST xor passive level), the input selector
    // with the external functions start, stop, count and gating, and the timer concatenation of adjacent slices.
    // Capture mode (CMC.CAP0S / CAP1S, capture registers CV0-CV3 with full flags, FIFO-less scheme, same-event scheme, timer
    // clear on capture) and the external up/down function (CMC.UDS) of an externally counted slice are modeled for the POSIF examples.
    // Not modeled: external load / trap / modulation / status override, dither, floating prescaler
    // (the registers are stored), input low pass filters and the one-cycle delays of ST and the service request pulses.
    //
    // Connections: 0-3 = service request lines SR0-SR3 (a one-event pulse each), 4-7 = slice outputs OUT0-OUT3.
    // Inputs (OnGPIO): number 0-15 is the input letter A-P (CCU4x.INyA-P) shared by all slices of the unit; 12-15 are always
    // the status bits ST0-ST3 of the unit itself, 8 is the SCU trigger GSC4x, the others are free for a test to drive.
    // Numbers 16-79 address one input of one slice only (16 + 16 * slice + letter), they are or-ed with the unit wide input.
    // This is how the POSIF outputs are wired, the device routes them to different letters for different slices.
    public sealed class XMC4XXX_CCU4 : BasicDoubleWordPeripheral, IKnownSize, INumberedGPIOOutput, IGPIOReceiver
    {
        public XMC4XXX_CCU4(IMachine machine, XMC4XXX_SCU scu = null, uint clockFrequency = 120000000) : base(machine)
        {
            this.scu = scu;
            this.fallbackClockFrequency = clockFrequency;

            var connections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < SliceCount + ServiceRequestCount; i++)
            {
                connections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(connections);

            slices = new Slice[SliceCount];
            for(var i = 0; i < SliceCount; i++)
            {
                slices[i] = new Slice(this, i);
            }
            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            idleMask = 0xF;
            prescalerRunning = false;
            Array.Clear(inputLevels, 0, inputLevels.Length);
            foreach(var slice in slices)
            {
                slice.Reset();
            }
            for(var i = 0; i < Connections.Count; i++)
            {
                Connections[i].Unset();
            }
        }

        public void OnGPIO(int number, bool value)
        {
            if(number >= InputCount && number < InputCount * (SliceCount + 1))
            {
                slices[(number - InputCount) / InputCount].SetOwnInput((number - InputCount) % InputCount, value);
                return;
            }
            if(number < 0 || number >= InputCount)
            {
                this.Log(LogLevel.Warning, "Input {0} does not exist", number);
                return;
            }
            SetInput(number, value);
        }

        public long Size => 0x1000;

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        // fCCU of the module, follows the SCU clock tree when an SCU is attached
        public long ClockFrequency
        {
            get
            {
                var frequency = scu != null ? scu.CcuClockFrequency : fallbackClockFrequency;
                return Math.Max(frequency, 1u);
            }
        }

        private void DefineRegisters()
        {
            Registers.GlobalControl.Define(this)
                .WithValueField(0, 3, name: "PRBC")
                .WithReservedBits(3, 1)
                .WithValueField(4, 2, name: "PCIS")
                .WithReservedBits(6, 2)
                .WithValueField(8, 2, name: "SUSCFG")
                .WithValueField(10, 4, name: "MSE")
                .WithValueField(14, 2, name: "MSDE")
                .WithReservedBits(16, 16);

            Registers.GlobalStatus.Define(this, 0xF)
                .WithValueField(0, 4, FieldMode.Read, name: "SxI", valueProviderCallback: _ => idleMask)
                .WithReservedBits(4, 4)
                .WithFlag(8, FieldMode.Read, name: "PRB", valueProviderCallback: _ => prescalerRunning)
                .WithReservedBits(9, 23);

            Registers.GlobalIdleSet.Define(this)
                .WithValueField(0, 4, FieldMode.Write, name: "SSxI", writeCallback: (_, value) => SetIdle((uint)value, true))
                .WithReservedBits(4, 4)
                .WithFlag(8, FieldMode.Write, name: "CPRB", writeCallback: (_, value) => { if(value) SetPrescalerRunning(false); })
                .WithFlag(9, FieldMode.Write, name: "PSIC")
                .WithReservedBits(10, 22);

            Registers.GlobalIdleClear.Define(this)
                .WithValueField(0, 4, FieldMode.Write, name: "CSxI", writeCallback: (_, value) => SetIdle((uint)value, false))
                .WithReservedBits(4, 4)
                .WithFlag(8, FieldMode.Write, name: "SPRB", writeCallback: (_, value) => { if(value) SetPrescalerRunning(true); })
                .WithReservedBits(9, 23);

            // GCSS/GCSC: per slice (4 bit stride): bit 0 = shadow transfer (period, compare, passive level), 1 = dither, 2 = prescaler; 16+y = status bit
            Registers.GlobalChannelSet.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "GCSS", writeCallback: (_, value) => WriteChannelSetClear((uint)value, true));
            Registers.GlobalChannelClear.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "GCSC", writeCallback: (_, value) => WriteChannelSetClear((uint)value, false));

            Registers.GlobalChannelStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "GCST", valueProviderCallback: _ =>
                {
                    var result = 0u;
                    for(var i = 0; i < SliceCount; i++)
                    {
                        result |= slices[i].ShadowTransferStatus << (4 * i);
                        result |= slices[i].Status ? 1u << (16 + i) : 0u;
                    }
                    return result;
                });

            Registers.ExtendedCaptureRead.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "ECRD");

            // Module identification: module number 0xA9 (CCU4), type 0xC0, revision 0x15 (CCU4, V1.15)
            Registers.ModuleIdentification.Define(this, 0x00A9C015)
                .WithValueField(0, 32, FieldMode.Read, name: "MIDR");

            for(var i = 0; i < SliceCount; i++)
            {
                DefineSliceRegisters(slices[i]);
            }
        }

        private void DefineSliceRegisters(Slice s)
        {
            var baseOffset = SliceBase * (s.Index + 1);
            Action<SliceRegisters, string, Func<uint>, Action<uint>, uint> defineReadWrite = (register, name, getter, setter, mask) =>
            {
                ((Registers)(baseOffset + (long)register)).Define(this)
                    .WithValueField(0, 32, name: name, valueProviderCallback: _ => getter(), writeCallback: (_, value) => setter((uint)value & mask));
            };

            defineReadWrite(SliceRegisters.InputSelector, "INS", () => s.InputSelector, v => s.InputSelector = v, 0x7FFF0FFF);
            defineReadWrite(SliceRegisters.ConnectionMatrix, "CMC", () => s.ConnectionMatrix, v => s.ConnectionMatrix = v, s.Index == 0 ? 0xFFFFFu : 0x1FFFFFu);

            ((Registers)(baseOffset + (long)SliceRegisters.TimerStatus)).Define(this)
                .WithFlag(0, FieldMode.Read, name: "TRB", valueProviderCallback: _ => s.RunBit)
                .WithFlag(1, FieldMode.Read, name: "CDIR", valueProviderCallback: _ => s.CountingDown)
                .WithReservedBits(2, 30);

            ((Registers)(baseOffset + (long)SliceRegisters.TimerRunSet)).Define(this)
                .WithFlag(0, FieldMode.Write, name: "TRBS", writeCallback: (_, value) => { if(value) s.SetRunBit(true); })
                .WithReservedBits(1, 31);

            ((Registers)(baseOffset + (long)SliceRegisters.TimerClear)).Define(this)
                .WithFlag(0, FieldMode.Write, name: "TRBC", writeCallback: (_, value) => { if(value) s.SetRunBit(false); })
                .WithFlag(1, FieldMode.Write, name: "TCC", writeCallback: (_, value) => { if(value) s.ClearTimer(); })
                .WithFlag(2, FieldMode.Write, name: "DITC")
                .WithReservedBits(3, 29);

            // CMOD (bit 3) is set by hardware in capture mode
            defineReadWrite(SliceRegisters.TimerControl, "TC", () => s.TimerControl | (s.CaptureMode ? 8u : 0u), v => s.SetTimerControl(v), 0x03E3FF77);
            defineReadWrite(SliceRegisters.PassiveLevel, "PSL", () => s.PassiveLevel ? 1u : 0u, v => s.PassiveLevelShadow = (v & 1) != 0, 1);
            defineReadWrite(SliceRegisters.Dither, "DIT", () => s.Dither, v => { }, 0);
            defineReadWrite(SliceRegisters.DitherShadow, "DITS", () => s.DitherShadow, v => s.DitherShadow = v, 0xF);
            defineReadWrite(SliceRegisters.PrescalerControl, "PSC", () => s.PrescalerControl, v => s.SetPrescalerControl(v), 0xF);
            defineReadWrite(SliceRegisters.FloatingPrescaler, "FPC", () => s.FloatingPrescaler, v => { }, 0);
            defineReadWrite(SliceRegisters.FloatingPrescalerShadow, "FPCS", () => s.FloatingPrescalerShadow, v => s.FloatingPrescalerShadow = v, 0xF);
            defineReadWrite(SliceRegisters.Period, "PR", () => s.Period, v => { }, 0);
            defineReadWrite(SliceRegisters.PeriodShadow, "PRS", () => s.PeriodShadow, v => s.PeriodShadow = v, 0xFFFF);
            defineReadWrite(SliceRegisters.Compare, "CR", () => s.Compare, v => { }, 0);
            defineReadWrite(SliceRegisters.CompareShadow, "CRS", () => s.CompareShadow, v => s.CompareShadow = v, 0xFFFF);
            defineReadWrite(SliceRegisters.Timer, "TIMER", () => s.TimerValue, v => s.WriteTimerValue(v), 0xFFFF);
            for(var i = 0; i < 4; i++)
            {
                var register = i;
                ((Registers)(baseOffset + (long)SliceRegisters.Capture0 + 4 * i)).Define(this)
                    .WithValueField(0, 16, FieldMode.Read, name: $"CAPTV{i}",
                        valueProviderCallback: _ => s.CaptureValues[register],
                        readCallback: (_, __) => s.CaptureFull[register] = false)
                    .WithValueField(16, 4, FieldMode.Read, name: $"FPCV{i}")
                    .WithFlag(20, FieldMode.Read, name: $"FFL{i}", valueProviderCallback: _ => s.CaptureFull[register])
                    .WithReservedBits(21, 11);
            }

            ((Registers)(baseOffset + (long)SliceRegisters.InterruptStatus)).Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "INTS", valueProviderCallback: _ => s.InterruptStatus);
            defineReadWrite(SliceRegisters.InterruptEnable, "INTE", () => s.InterruptEnable, v => s.InterruptEnable = v, 0x70F);
            defineReadWrite(SliceRegisters.ServiceRequestSelector, "SRS", () => s.ServiceRequestSelector, v => s.ServiceRequestSelector = v, 0x3F0F);
            ((Registers)(baseOffset + (long)SliceRegisters.InterruptSet)).Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "SWS", writeCallback: (_, value) => s.SoftwareSetFlags((uint)value & 0xF0F));
            ((Registers)(baseOffset + (long)SliceRegisters.InterruptClear)).Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "SWR", writeCallback: (_, value) => s.InterruptStatus &= ~((uint)value & 0xF0F));
        }

        private void WriteChannelSetClear(uint value, bool set)
        {
            for(var i = 0; i < SliceCount; i++)
            {
                var transferBits = (value >> (4 * i)) & 0x7;
                if(transferBits != 0)
                {
                    slices[i].RequestShadowTransfer(transferBits, set);
                }
                if((value & (1u << (16 + i))) != 0)
                {
                    slices[i].SetStatus(set);
                }
            }
        }

        private void SetIdle(uint mask, bool idle)
        {
            if(idle)
            {
                idleMask |= mask & 0xF;
            }
            else
            {
                idleMask &= ~mask & 0xF;
            }
            ReconfigureAll();
        }

        private void SetPrescalerRunning(bool running)
        {
            prescalerRunning = running;
            ReconfigureAll();
        }

        private void ReconfigureAll()
        {
            foreach(var slice in slices)
            {
                slice.Reconfigure();
            }
        }

        private void SetInput(int index, bool value)
        {
            if(inputLevels[index] == value)
            {
                return;
            }
            inputLevels[index] = value;
            foreach(var slice in slices)
            {
                slice.OnInputChanged(index, value);
            }
        }

        private void PulseServiceRequest(int line)
        {
            Connections[line].Set(true);
            Connections[line].Set(false);
        }

        private void UpdateOutput(int slice, bool value)
        {
            Connections[ServiceRequestCount + slice].Set(value);
        }

        private bool IsIdle(int slice)
        {
            return (idleMask & (1u << slice)) != 0;
        }

        private readonly XMC4XXX_SCU scu;
        private readonly uint fallbackClockFrequency;
        private readonly Slice[] slices;
        private readonly bool[] inputLevels = new bool[InputCount];
        private uint idleMask = 0xF;
        private bool prescalerRunning;

        private const int SliceCount = 4;
        private const int ServiceRequestCount = 4;
        private const int InputCount = 16;
        private const int SliceBase = 0x100;
        private const int StatusInputBase = 12;

        private enum EventKind
        {
            CompareUp,
            PeriodMatch,
            OneMatch,
            CompareDown,
            Wrap,
        }

        // Bits of INTS / INTE / SWS / SWR
        private const uint PeriodMatchUp = 1u << 0;
        private const uint OneMatchDown = 1u << 1;
        private const uint CompareMatchUp = 1u << 2;
        private const uint CompareMatchDown = 1u << 3;
        private const uint Event0Detected = 1u << 8;

        private sealed class Slice
        {
            public Slice(XMC4XXX_CCU4 unit, int index)
            {
                this.unit = unit;
                Index = index;
                timers = new LimitTimer[eventKindCount];
                for(var i = 0; i < eventKindCount; i++)
                {
                    var kind = (EventKind)i;
                    var timer = new LimitTimer(unit.machine.ClockSource, 1000000, unit, $"slice{index}_{kind}", direction: Direction.Ascending, workMode: WorkMode.Periodic, eventEnabled: true);
                    timer.LimitReached += () => OnTimerEvent(kind);
                    timers[i] = timer;
                }
            }

            public void Reset()
            {
                foreach(var timer in timers)
                {
                    timer.Enabled = false;
                }
                timersRunning = false;
                InputSelector = 0;
                ConnectionMatrix = 0;
                TimerControl = 0;
                PassiveLevel = false;
                PassiveLevelShadow = false;
                Dither = 0;
                DitherShadow = 0;
                PrescalerControl = 0;
                FloatingPrescaler = 0;
                FloatingPrescalerShadow = 0;
                Period = 0;
                PeriodShadow = 0;
                Compare = 0;
                CompareShadow = 0;
                InterruptStatus = 0;
                InterruptEnable = 0;
                ServiceRequestSelector = 0;
                ShadowTransferStatus = 0;
                RunBit = false;
                CountingDown = false;
                Status = false;
                position = 0;
                Array.Clear(CaptureValues, 0, CaptureValues.Length);
                Array.Clear(CaptureFull, 0, CaptureFull.Length);
                Array.Clear(ownInputs, 0, ownInputs.Length);
            }

            public int Index { get; }
            public uint InputSelector { get; set; }
            public uint TimerControl { get; private set; }
            public bool PassiveLevel { get; private set; }
            public bool PassiveLevelShadow { get; set; }
            public uint Dither { get; private set; }
            public uint DitherShadow { get; set; }
            public uint PrescalerControl { get; private set; }
            public uint FloatingPrescaler { get; private set; }
            public uint FloatingPrescalerShadow { get; set; }
            public uint Period { get; private set; }
            public uint PeriodShadow { get; set; }
            public uint Compare { get; private set; }
            public uint CompareShadow { get; set; }
            public uint InterruptStatus { get; set; }
            public uint InterruptEnable { get; set; }
            public uint ServiceRequestSelector { get; set; }
            // bit 0 period/compare/passive level, bit 1 dither, bit 2 prescaler
            public uint ShadowTransferStatus { get; private set; }
            public bool RunBit { get; private set; }
            public bool CountingDown { get; private set; }
            public bool Status { get; private set; }
            public bool CaptureMode => (connectionMatrix & 0xF0) != 0;
            public uint[] CaptureValues { get; } = new uint[4];
            public bool[] CaptureFull { get; } = new bool[4];

            // Input of this slice alone (see the class comment), edges are delivered like those of the unit wide inputs
            public void SetOwnInput(int letter, bool value)
            {
                if(ownInputs[letter] == value)
                {
                    return;
                }
                var before = ownInputs[letter] || unit.inputLevels[letter];
                ownInputs[letter] = value;
                var after = value || unit.inputLevels[letter];
                if(before != after)
                {
                    OnInputChanged(letter, after);
                }
            }

            public uint ConnectionMatrix
            {
                get => connectionMatrix;
                set
                {
                    connectionMatrix = value;
                    Reconfigure();
                }
            }

            public uint TimerValue
            {
                get
                {
                    var current = CurrentPosition;
                    if(CenterAligned && current > Period)
                    {
                        var mirrored = 2 * (long)Period + 1 - current;
                        return (uint)Math.Max(mirrored, 0);
                    }
                    return Math.Min(current, 0xFFFF);
                }
            }

            public void SetTimerControl(uint value)
            {
                TimerControl = value;
                Reconfigure();
            }

            public void SetPrescalerControl(uint value)
            {
                PrescalerControl = value;
                Reconfigure();
            }

            public void WriteTimerValue(uint value)
            {
                if(RunBit)
                {
                    unit.Log(LogLevel.Warning, "CC4{0}: TIMER written while the timer is running, ignored", Index);
                    return;
                }
                position = value;
            }

            public void SetRunBit(bool value)
            {
                if(RunBit == value)
                {
                    return;
                }
                RunBit = value;
                Reconfigure();
            }

            public void ClearTimer()
            {
                position = 0;
                CountingDown = false;
                if(ClearTriggersShadowTransfer && (ShadowTransferStatus & 1) != 0)
                {
                    TransferShadow();
                }
                Reconfigure(0);
            }

            public void RequestShadowTransfer(uint bits, bool set)
            {
                if(set)
                {
                    ShadowTransferStatus |= bits;
                    if(!RunBit)
                    {
                        // a stopped timer transfers immediately
                        TransferShadow();
                    }
                }
                else
                {
                    ShadowTransferStatus &= ~bits;
                }
            }

            public void SetStatus(bool value)
            {
                if(Status == value)
                {
                    return;
                }
                Status = value;
                UpdateOutput();
                unit.SetInput(StatusInputBase + Index, value);
            }

            public void SoftwareSetFlags(uint flags)
            {
                for(var bit = 0u; bit < 12; bit++)
                {
                    if((flags & (1u << (int)bit)) != 0)
                    {
                        SetFlag(1u << (int)bit);
                    }
                }
            }

            // (Re)programs the event timers from the current position; called on every change of an input of the slice behavior.
            public void Reconfigure(uint? newPosition = null)
            {
                if(newPosition.HasValue)
                {
                    position = newPosition.Value;
                }
                else
                {
                    SyncPosition();
                }
                foreach(var timer in timers)
                {
                    timer.Enabled = false;
                }
                timersRunning = false;

                if(!Active || ExternallyClocked)
                {
                    return;
                }

                var length = CycleLength;
                var divider = Divider;
                activeLength = length;
                activeDivider = divider;
                var frequency = unit.ClockFrequency;
                var count = FillEvents();
                for(var i = 0; i < count; i++)
                {
                    var timer = timers[(int)eventKinds[i]];
                    if(timer.Frequency != (ulong)frequency)
                    {
                        timer.Frequency = (ulong)frequency;
                    }
                    // The timers run at fCCU; one timer clock cycle (prescaled) is `divider` of these ticks
                    var distance = (long)eventPositions[i] - position;
                    distance = ((distance % length) + length) % length;
                    if(distance == 0)
                    {
                        distance = length;
                    }
                    timer.Limit = (ulong)(length * divider);
                    timer.Value = (ulong)((length - distance) * divider);
                    timer.Enabled = true;
                }
                timersRunning = true;
            }

            public void OnInputChanged(int input, bool value)
            {
                for(var e = 0; e < 3; e++)
                {
                    if(EventInput(e) != input)
                    {
                        continue;
                    }

                    var edgeMode = (InputSelector >> (16 + 2 * e)) & 3;
                    var rising = value && (edgeMode & 1) != 0;
                    var falling = !value && (edgeMode & 2) != 0;
                    if(rising || falling)
                    {
                        OnEvent(e);
                    }
                    if(((connectionMatrix >> 8) & 3) == e + 1)
                    {
                        // gating follows the level of the event
                        Reconfigure();
                    }
                }
            }

            // Pulse from the external count function or from the lower slice of a concatenation
            public void CountPulse()
            {
                if(!Active || !ExternallyClocked)
                {
                    return;
                }

                // external up/down function: the timer counts down while the selected event is active
                var directionEvent = (connectionMatrix >> 10) & 3;
                if(directionEvent != 0)
                {
                    CountingDown = EventLevel((int)directionEvent - 1);
                    if(CountingDown)
                    {
                        position = position == 0 ? EffectivePeriod : position - 1;
                        return;
                    }
                }

                position++;
                var count = FillEvents();
                Span<EventKind> matched = stackalloc EventKind[eventKindCount];
                var matches = 0;
                for(var i = 0; i < count; i++)
                {
                    if(eventPositions[i] == position)
                    {
                        matched[matches++] = eventKinds[i];
                    }
                }
                for(var i = 0; i < matches; i++)
                {
                    ProcessEvent(matched[i]);
                }
            }

            private void OnEvent(int e)
            {
                SetFlag(Event0Detected << e);

                var selector = (uint)(e + 1);
                if(CaptureMode)
                {
                    Capture(selector);
                }
                if((connectionMatrix & 3) == selector)
                {
                    // external start: STRM = 1 clears the timer first
                    if(((TimerControl >> 10) & 1) != 0)
                    {
                        ClearTimer();
                    }
                    SetRunBit(true);
                }
                if(((connectionMatrix >> 2) & 3) == selector)
                {
                    var stopMode = (TimerControl >> 8) & 3;
                    if(stopMode != 0)
                    {
                        ClearTimer();
                    }
                    if(stopMode != 1)
                    {
                        SetRunBit(false);
                    }
                }
                if(((connectionMatrix >> 14) & 3) == selector && !Concatenated)
                {
                    CountPulse();
                }
            }

            // Capture trigger: CAP0S fills CV1 (the old CV1 moves to CV0), CAP1S fills CV3 (old CV3 to CV2),
            // with TC.SCE one trigger (CAP1S) shifts through all four registers. A full register chain blocks the capture unless TC.CCS is set.
            private void Capture(uint selector)
            {
                var captureLow = ((connectionMatrix >> 4) & 3) == selector;
                var captureHigh = ((connectionMatrix >> 6) & 3) == selector;
                var sameEvent = (TimerControl & (1u << 11)) != 0;
                var ignoreFull = (TimerControl & (1u << 12)) != 0;
                var value = TimerValue;
                var captured = false;
                var clearMask = 0u;

                if(sameEvent)
                {
                    if(captureHigh)
                    {
                        if(ignoreFull || !(CaptureFull[3] && CaptureFull[2] && CaptureFull[1] && CaptureFull[0]))
                        {
                            for(var i = 0; i < 3; i++)
                            {
                                if(ignoreFull || (CaptureFull[i + 1] && !CaptureFull[i]))
                                {
                                    CaptureValues[i] = CaptureValues[i + 1];
                                    CaptureFull[i] = CaptureFull[i + 1] || ignoreFull;
                                }
                            }
                            CaptureValues[3] = value;
                            CaptureFull[3] = true;
                            captured = true;
                            clearMask = 3;
                        }
                    }
                }
                else
                {
                    if(captureLow && CaptureInto(0, value, ignoreFull))
                    {
                        captured = true;
                        clearMask |= 1;
                    }
                    if(captureHigh && CaptureInto(2, value, ignoreFull))
                    {
                        captured = true;
                        clearMask |= 2;
                    }
                }

                if(!captured)
                {
                    return;
                }
                SetStatus(true);
                // TC.CAPC: 1 clears the timer on a capture into CV3/CV2, 2 on a capture into CV1/CV0, 3 on any capture
                var clearMode = (TimerControl >> 5) & 3;
                if(((clearMode & 2) != 0 && (clearMask & 1) != 0) || ((clearMode & 1) != 0 && (clearMask & 2) != 0))
                {
                    ClearTimer();
                }
            }

            private bool CaptureInto(int low, uint value, bool ignoreFull)
            {
                var high = low + 1;
                if(!ignoreFull && CaptureFull[high] && CaptureFull[low])
                {
                    return false;
                }
                if(ignoreFull || (CaptureFull[high] && !CaptureFull[low]))
                {
                    CaptureValues[low] = CaptureValues[high];
                    CaptureFull[low] = true;
                }
                CaptureValues[high] = value;
                CaptureFull[high] = true;
                return true;
            }

            private void OnTimerEvent(EventKind kind)
            {
                // the position of timer driven slices is read from the timers, a fired event sits exactly on its own position
                ProcessEvent(kind);
            }

            private void ProcessEvent(EventKind kind)
            {
                switch(kind)
                {
                case EventKind.CompareUp:
                    SetFlag(CompareMatchUp);
                    SetStatus(true);
                    break;
                case EventKind.PeriodMatch:
                    SetFlag(PeriodMatchUp);
                    CountingDown = true;
                    TransferShadowAt(false);
                    break;
                case EventKind.OneMatch:
                    SetFlag(OneMatchDown);
                    TransferShadowAt(true);
                    break;
                case EventKind.CompareDown:
                    SetFlag(CompareMatchDown);
                    SetStatus(false);
                    break;
                case EventKind.Wrap:
                    OnWrap();
                    break;
                }
            }

            // Edge aligned: period match, timer cleared. Center aligned: timer counted down to zero.
            private void OnWrap()
            {
                if(CenterAligned)
                {
                    CountingDown = false;
                }
                else
                {
                    SetFlag(PeriodMatchUp);
                }

                var alwaysActive = Compare == 0 && !CaptureMode;
                if(alwaysActive)
                {
                    SetFlag(CenterAligned ? CompareMatchDown : CompareMatchUp);
                }
                else if(!CenterAligned)
                {
                    SetStatus(false);
                }
                if(alwaysActive)
                {
                    SetStatus(true);
                }

                position = 0;
                var changed = false;
                if(!CenterAligned && (ShadowTransferStatus & 1) != 0)
                {
                    changed = TransferShadow();
                }
                if(SingleShot)
                {
                    RunBit = false;
                    SetStatus(false);
                    changed = true;
                }
                if(changed)
                {
                    Reconfigure(0);
                }

                // the lower slice period overflow clocks the next slice of a concatenation
                var next = Index + 1;
                if(next < SliceCount && unit.slices[next].Concatenated)
                {
                    unit.slices[next].CountPulse();
                }
            }

            private void TransferShadowAt(bool oneMatch)
            {
                if((ShadowTransferStatus & 1) == 0)
                {
                    return;
                }
                if(TransferShadow())
                {
                    // continue at the same side of the cycle with the new period
                    Reconfigure(oneMatch ? 2 * Period : Period);
                }
            }

            // Returns true when the period or the compare value changed
            private bool TransferShadow()
            {
                var changed = false;
                if((ShadowTransferStatus & 1) != 0)
                {
                    changed = Period != PeriodShadow || Compare != CompareShadow;
                    Period = PeriodShadow;
                    Compare = CompareShadow;
                    if(PassiveLevel != PassiveLevelShadow)
                    {
                        PassiveLevel = PassiveLevelShadow;
                        UpdateOutput();
                    }
                }
                if((ShadowTransferStatus & 2) != 0)
                {
                    Dither = DitherShadow;
                }
                if((ShadowTransferStatus & 4) != 0)
                {
                    FloatingPrescaler = FloatingPrescalerShadow;
                }
                ShadowTransferStatus = 0;
                return changed;
            }

            private void SetFlag(uint flag)
            {
                InterruptStatus |= flag;
                if((InterruptEnable & flag) == 0)
                {
                    return;
                }

                int shift;
                if((flag & (PeriodMatchUp | OneMatchDown)) != 0)
                {
                    shift = 0;
                }
                else if((flag & (CompareMatchUp | CompareMatchDown)) != 0)
                {
                    shift = 2;
                }
                else
                {
                    shift = 8 + 2 * (BitIndex(flag) - 8);
                }
                unit.PulseServiceRequest((int)((ServiceRequestSelector >> shift) & 3));
            }

            private static int BitIndex(uint flag)
            {
                var index = 0;
                while((flag >>= 1) != 0)
                {
                    index++;
                }
                return index;
            }

            private void UpdateOutput()
            {
                unit.UpdateOutput(Index, Status ^ PassiveLevel);
            }

            private void SyncPosition()
            {
                if(timersRunning)
                {
                    var wrapTimer = timers[(int)EventKind.Wrap];
                    position = (uint)((wrapTimer.Value / (ulong)activeDivider) % (ulong)activeLength);
                }
            }

            private uint CurrentPosition
            {
                get
                {
                    SyncPosition();
                    return position;
                }
            }

            // Fills eventKinds/eventPositions with the events of the cycle; positions are counted in timer clock cycles from the cycle start
            private int FillEvents()
            {
                var count = 0;
                var period = EffectivePeriod;
                var compare = Compare;
                if(!CenterAligned)
                {
                    if(!CaptureMode && compare >= 1 && compare <= period)
                    {
                        Add(ref count, EventKind.CompareUp, compare);
                    }
                    Add(ref count, EventKind.Wrap, period + 1);
                }
                else
                {
                    if(compare >= 1 && compare <= period)
                    {
                        Add(ref count, EventKind.CompareUp, compare);
                        Add(ref count, EventKind.CompareDown, 2 * period + 2 - compare);
                    }
                    Add(ref count, EventKind.PeriodMatch, period);
                    if(period >= 1)
                    {
                        Add(ref count, EventKind.OneMatch, 2 * period);
                    }
                    Add(ref count, EventKind.Wrap, 2 * period + 2);
                }
                return count;
            }

            private void Add(ref int count, EventKind kind, uint at)
            {
                eventKinds[count] = kind;
                eventPositions[count] = at;
                count++;
            }

            private int EventInput(int e)
            {
                return (int)((InputSelector >> (4 * e)) & 0xF);
            }

            private bool EventLevel(int e)
            {
                var level = unit.inputLevels[EventInput(e)] || ownInputs[EventInput(e)];
                var activeLow = ((InputSelector >> (22 + e)) & 1) != 0;
                return level ^ activeLow;
            }

            private bool CenterAligned => (TimerControl & 1) != 0 && !Concatenated && !CaptureMode;
            // with a second capture event (CAP1S) the period register holds capture data, the timer then counts the full 16 bit range
            private uint EffectivePeriod => ((connectionMatrix >> 6) & 3) != 0 ? 0xFFFFu : Period;
            private bool SingleShot => (TimerControl & 2) != 0;
            private bool ClearTriggersShadowTransfer => (TimerControl & 4) != 0;
            private bool Concatenated => Index > 0 && (connectionMatrix & (1u << 20)) != 0;
            private bool ExternallyClocked => Concatenated || ((connectionMatrix >> 14) & 3) != 0;
            private int Divider => 1 << (int)(PrescalerControl & 0xF);
            private long CycleLength => CenterAligned ? 2 * (long)Period + 2 : (long)EffectivePeriod + 1;

            private bool Active
            {
                get
                {
                    if(!RunBit || unit.IsIdle(Index) || !unit.prescalerRunning)
                    {
                        return false;
                    }
                    var gate = (connectionMatrix >> 8) & 3;
                    return gate == 0 || EventLevel((int)gate - 1);
                }
            }

            private uint connectionMatrix;
            private readonly bool[] ownInputs = new bool[InputCount];
            private uint position;
            private bool timersRunning;
            private long activeLength = 1;
            private int activeDivider = 1;

            private readonly XMC4XXX_CCU4 unit;
            private readonly LimitTimer[] timers;
            private readonly EventKind[] eventKinds = new EventKind[eventKindCount];
            private readonly uint[] eventPositions = new uint[eventKindCount];

            private const int eventKindCount = 5;
        }

        private enum Registers : long
        {
            GlobalControl = 0x00,
            GlobalStatus = 0x04,
            GlobalIdleSet = 0x08,
            GlobalIdleClear = 0x0C,
            GlobalChannelSet = 0x10,
            GlobalChannelClear = 0x14,
            GlobalChannelStatus = 0x18,
            ExtendedCaptureRead = 0x50,
            ModuleIdentification = 0x80,
        }

        // Offsets inside a slice, the slice y starts at 0x100 * (y + 1)
        private enum SliceRegisters : long
        {
            InputSelector = 0x00,
            ConnectionMatrix = 0x04,
            TimerStatus = 0x08,
            TimerRunSet = 0x0C,
            TimerClear = 0x10,
            TimerControl = 0x14,
            PassiveLevel = 0x18,
            Dither = 0x1C,
            DitherShadow = 0x20,
            PrescalerControl = 0x24,
            FloatingPrescaler = 0x28,
            FloatingPrescalerShadow = 0x2C,
            Period = 0x30,
            PeriodShadow = 0x34,
            Compare = 0x38,
            CompareShadow = 0x3C,
            Timer = 0x70,
            Capture0 = 0x74,
            InterruptStatus = 0xA0,
            InterruptEnable = 0xA4,
            ServiceRequestSelector = 0xA8,
            InterruptSet = 0xAC,
            InterruptClear = 0xB0,
        }
    }
}
