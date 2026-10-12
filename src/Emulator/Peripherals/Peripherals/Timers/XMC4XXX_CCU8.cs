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
    // Capture/Compare Unit 8 (CCU8) of the Infineon XMC4xxx family, one instance per unit (CCU80, CCU81; four slices each).
    // Reference Manual v1.6, chapter 23. The class follows the structure of XMC4XXX_CCU4 (its own copy, the two units differ in
    // the compare channels, the output stage and the register layout of SRS/CHC/DTC).
    //
    // Modeled: compare mode with two compare channels per slice (edge and center aligned, single shot, asymmetric PWM), 16 bit
    // period/compare with shadow transfer (PR, CR1, CR2, passive levels), normal prescaler (fTCLK = fCCU / 2^PSIV), run/idle control
    // (GIDLS/GIDLC, prescaler run bit), the status bits ST1/ST2 and the four outputs per slice (path selection CHC.OCS1-4, dead time
    // generators DTC/DC1R/DC2R, passive levels), interrupt flags and the four service request lines, the input selector with the
    // external functions start, stop, count and gating, capture mode (CV0-CV3 with full flags, CAP0S/CAP1S, SCE, CCS, CAPC), and the
    // timer concatenation of adjacent slices.
    // Not modeled: external load / up-down / trap / modulation / status override, multi channel mode, dither, floating prescaler
    // (the registers are stored), extended capture read (ECRD), parity checker (GPCHK is stored), input low pass filters and the
    // one-cycle delays of ST and the service request pulses.
    //
    // Connections: 0-3 = service request lines SR0-SR3 (a one-event pulse each), 4 + 4 * slice + n = output OUTsn (n = 0-3).
    // Inputs (OnGPIO): number 0-15 is the input letter A-P (CCU8x.INyA-P) shared by all slices of the unit; 12-15 are always the
    // status bits ST0-ST3 of the unit itself, 7 is the SCU trigger GSC8x, the others are free for a test or the .repl to drive.
    public sealed class XMC4XXX_CCU8 : BasicDoubleWordPeripheral, IKnownSize, INumberedGPIOOutput, IGPIOReceiver
    {
        public XMC4XXX_CCU8(IMachine machine, XMC4XXX_SCU scu = null, uint clockFrequency = 120000000) : base(machine)
        {
            this.scu = scu;
            this.fallbackClockFrequency = clockFrequency;

            var connections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < SliceCount * OutputsPerSlice + ServiceRequestCount; i++)
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
                .WithReservedBits(9, 1)
                .WithFlag(10, FieldMode.Read, name: "PCRB")
                .WithReservedBits(11, 21);

            Registers.GlobalIdleSet.Define(this)
                .WithValueField(0, 4, FieldMode.Write, name: "SSxI", writeCallback: (_, value) => SetIdle((uint)value, true))
                .WithReservedBits(4, 4)
                .WithFlag(8, FieldMode.Write, name: "CPRB", writeCallback: (_, value) => { if(value) SetPrescalerRunning(false); })
                .WithFlag(9, FieldMode.Write, name: "PSIC")
                .WithFlag(10, FieldMode.Write, name: "CPCH")
                .WithReservedBits(11, 21);

            Registers.GlobalIdleClear.Define(this)
                .WithValueField(0, 4, FieldMode.Write, name: "CSxI", writeCallback: (_, value) => SetIdle((uint)value, false))
                .WithReservedBits(4, 4)
                .WithFlag(8, FieldMode.Write, name: "SPRB", writeCallback: (_, value) => { if(value) SetPrescalerRunning(true); })
                .WithReservedBits(9, 1)
                .WithFlag(10, FieldMode.Write, name: "SPCH")
                .WithReservedBits(11, 21);

            // GCSS/GCSC: per slice (4 bit stride): bit 0 = shadow transfer (period, compare, passive level), 1 = dither, 2 = prescaler;
            // 16+y = status bit ST1 of slice y, 20+y = status bit ST2
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
                        result |= slices[i].GetStatus(0) ? 1u << (16 + i) : 0u;
                        result |= slices[i].GetStatus(1) ? 1u << (20 + i) : 0u;
                    }
                    return result;
                });

            Registers.ParityCheckerConfiguration.Define(this)
                .WithValueField(0, 32, name: "GPCHK");

            Registers.ExtendedCaptureRead.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "ECRD");

            // Module identification: module number 0xA7 (CCU8), type 0xC0
            Registers.ModuleIdentification.Define(this, 0x00A7C015)
                .WithValueField(0, 32, FieldMode.Read, name: "MIDR");

            for(var i = 0; i < SliceCount; i++)
            {
                DefineSliceRegisters(slices[i]);
            }
        }

        private void DefineSliceRegisters(Slice s)
        {
            var baseOffset = SliceBase * (s.Index + 1);
            Action<SliceRegisters, string, Func<uint>, Action<uint>, uint, uint> defineReadWrite = (register, name, getter, setter, mask, reset) =>
            {
                ((Registers)(baseOffset + (long)register)).Define(this, reset)
                    .WithValueField(0, 32, name: name, valueProviderCallback: _ => getter(), writeCallback: (_, value) => setter((uint)value & mask));
            };

            defineReadWrite(SliceRegisters.InputSelector, "INS", () => s.InputSelector, v => s.InputSelector = v, 0x7FFF0FFF, 0);
            defineReadWrite(SliceRegisters.ConnectionMatrix, "CMC", () => s.ConnectionMatrix, v => s.ConnectionMatrix = v, s.Index == 0 ? 0xFFFFFu : 0x1FFFFFu, 0);

            ((Registers)(baseOffset + (long)SliceRegisters.TimerStatus)).Define(this)
                .WithFlag(0, FieldMode.Read, name: "TRB", valueProviderCallback: _ => s.RunBit)
                .WithFlag(1, FieldMode.Read, name: "CDIR", valueProviderCallback: _ => s.CountingDown)
                .WithReservedBits(2, 1)
                .WithFlag(3, FieldMode.Read, name: "DTR1", valueProviderCallback: _ => s.DeadTimeRunning(0))
                .WithFlag(4, FieldMode.Read, name: "DTR2", valueProviderCallback: _ => s.DeadTimeRunning(1))
                .WithReservedBits(5, 27);

            ((Registers)(baseOffset + (long)SliceRegisters.TimerRunSet)).Define(this)
                .WithFlag(0, FieldMode.Write, name: "TRBS", writeCallback: (_, value) => { if(value) s.SetRunBit(true); })
                .WithReservedBits(1, 31);

            ((Registers)(baseOffset + (long)SliceRegisters.TimerClear)).Define(this)
                .WithFlag(0, FieldMode.Write, name: "TRBC", writeCallback: (_, value) => { if(value) s.SetRunBit(false); })
                .WithFlag(1, FieldMode.Write, name: "TCC", writeCallback: (_, value) => { if(value) s.ClearTimer(); })
                .WithFlag(2, FieldMode.Write, name: "DITC")
                .WithFlag(3, FieldMode.Write, name: "DTC1C", writeCallback: (_, value) => { if(value) s.ClearDeadTime(0); })
                .WithFlag(4, FieldMode.Write, name: "DTC2C", writeCallback: (_, value) => { if(value) s.ClearDeadTime(1); })
                .WithReservedBits(5, 27);

            // CMOD (bit 3) is set by hardware when an event is linked to a capture function; bit 31 is reserved
            defineReadWrite(SliceRegisters.TimerControl, "TC", () => s.TimerControlRead, v => s.SetTimerControl(v), 0x7FFFFFF7, 0x18000000);
            defineReadWrite(SliceRegisters.PassiveLevel, "PSL", () => s.PassiveLevel, v => s.PassiveLevelShadow = v, 0xF, 0);
            defineReadWrite(SliceRegisters.Dither, "DIT", () => s.Dither, v => { }, 0, 0);
            defineReadWrite(SliceRegisters.DitherShadow, "DITS", () => s.DitherShadow, v => s.DitherShadow = v, 0xF, 0);
            defineReadWrite(SliceRegisters.PrescalerControl, "PSC", () => s.PrescalerControl, v => s.SetPrescalerControl(v), 0xF, 0);
            defineReadWrite(SliceRegisters.FloatingPrescaler, "FPC", () => s.FloatingPrescalerRead, v => { }, 0, 0);
            defineReadWrite(SliceRegisters.FloatingPrescalerShadow, "FPCS", () => s.FloatingPrescalerShadow, v => s.FloatingPrescalerShadow = v, 0xF, 0);
            defineReadWrite(SliceRegisters.Period, "PR", () => s.Period, v => { }, 0, 0);
            defineReadWrite(SliceRegisters.PeriodShadow, "PRS", () => s.PeriodShadow, v => s.PeriodShadow = v, 0xFFFF, 0);
            defineReadWrite(SliceRegisters.Compare1, "CR1", () => s.CaptureModeLow ? 0 : s.Compare[0], v => { }, 0, 0);
            defineReadWrite(SliceRegisters.Compare1Shadow, "CR1S", () => s.CaptureModeLow ? 0 : s.CompareShadow[0], v => s.CompareShadow[0] = v, 0xFFFF, 0);
            defineReadWrite(SliceRegisters.Compare2, "CR2", () => s.CaptureModeHigh ? 0 : s.Compare[1], v => { }, 0, 0);
            defineReadWrite(SliceRegisters.Compare2Shadow, "CR2S", () => s.CaptureModeHigh ? 0 : s.CompareShadow[1], v => s.CompareShadow[1] = v, 0xFFFF, 0);
            defineReadWrite(SliceRegisters.ChannelControl, "CHC", () => s.ChannelControl, v => s.SetChannelControl(v), 0x1F, 0);
            defineReadWrite(SliceRegisters.DeadTimeControl, "DTC", () => s.DeadTimeControl, v => s.SetDeadTimeControl(v), 0xFF, 0);
            defineReadWrite(SliceRegisters.DeadTimeValues1, "DC1R", () => s.DeadTimeValues[0], v => s.DeadTimeValues[0] = v, 0xFFFF, 0);
            defineReadWrite(SliceRegisters.DeadTimeValues2, "DC2R", () => s.DeadTimeValues[1], v => s.DeadTimeValues[1] = v, 0xFFFF, 0);
            defineReadWrite(SliceRegisters.Timer, "TIMER", () => s.TimerValue, v => s.WriteTimerValue(v), 0xFFFF, 0);
            for(var i = 0; i < 4; i++)
            {
                var index = i;
                ((Registers)(baseOffset + (long)SliceRegisters.Capture0 + 4 * i)).Define(this)
                    .WithValueField(0, 32, FieldMode.Read, name: $"CV{i}", valueProviderCallback: _ => s.ReadCaptureRegister(index));
            }

            ((Registers)(baseOffset + (long)SliceRegisters.InterruptStatus)).Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "INTS", valueProviderCallback: _ => s.InterruptStatus);
            defineReadWrite(SliceRegisters.InterruptEnable, "INTE", () => s.InterruptEnable, v => s.InterruptEnable = v, 0x73F, 0);
            defineReadWrite(SliceRegisters.ServiceRequestSelector, "SRS", () => s.ServiceRequestSelector, v => s.ServiceRequestSelector = v, 0x3F3F, 0);
            ((Registers)(baseOffset + (long)SliceRegisters.InterruptSet)).Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "SWS", writeCallback: (_, value) => s.SoftwareSetFlags((uint)value & 0xF3F));
            ((Registers)(baseOffset + (long)SliceRegisters.InterruptClear)).Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: "SWR", writeCallback: (_, value) => s.InterruptStatus &= ~((uint)value & 0xF3F));
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
                    slices[i].SetStatus(0, set);
                }
                if((value & (1u << (20 + i))) != 0)
                {
                    slices[i].SetStatus(1, set);
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
            foreach(var slice in slices)
            {
                slice.UpdateOutputs();
            }
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

        private void SetOutput(int slice, int output, bool value)
        {
            Connections[ServiceRequestCount + OutputsPerSlice * slice + output].Set(value);
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
        private const int OutputsPerSlice = 4;
        private const int ServiceRequestCount = 4;
        private const int InputCount = 16;
        private const int SliceBase = 0x100;
        private const int StatusInputBase = 12;

        private enum EventKind
        {
            Compare1Up,
            Compare2Up,
            PeriodMatch,
            OneMatch,
            Compare1Down,
            Compare2Down,
            Wrap,
        }

        // Bits of INTS / INTE / SWS / SWR
        private const uint PeriodMatchUp = 1u << 0;
        private const uint OneMatchDown = 1u << 1;
        private const uint Compare1MatchUp = 1u << 2;
        private const uint Compare1MatchDown = 1u << 3;
        private const uint Compare2MatchUp = 1u << 4;
        private const uint Compare2MatchDown = 1u << 5;
        private const uint Event0Detected = 1u << 8;

        private sealed class Slice
        {
            public Slice(XMC4XXX_CCU8 unit, int index)
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
                // dead time counters: [2 * channel] delays the status path, [2 * channel + 1] the inverted status path
                deadTimeTimers = new LimitTimer[4];
                for(var i = 0; i < 4; i++)
                {
                    var gate = i;
                    var timer = new LimitTimer(unit.machine.ClockSource, 1000000, unit, $"slice{index}_deadtime{i}", direction: Direction.Ascending, workMode: WorkMode.OneShot, eventEnabled: true);
                    timer.LimitReached += () => OnDeadTimeElapsed(gate);
                    deadTimeTimers[i] = timer;
                }
            }

            public void Reset()
            {
                foreach(var timer in timers)
                {
                    timer.Enabled = false;
                }
                foreach(var timer in deadTimeTimers)
                {
                    timer.Enabled = false;
                }
                timersRunning = false;
                InputSelector = 0;
                ConnectionMatrix = 0;
                TimerControl = 0x18000000;
                PassiveLevel = 0;
                PassiveLevelShadow = 0;
                Dither = 0;
                DitherShadow = 0;
                PrescalerControl = 0;
                FloatingPrescaler = 0;
                FloatingPrescalerShadow = 0;
                Period = 0;
                PeriodShadow = 0;
                Array.Clear(Compare, 0, 2);
                Array.Clear(CompareShadow, 0, 2);
                ChannelControl = 0;
                DeadTimeControl = 0;
                Array.Clear(DeadTimeValues, 0, 2);
                InterruptStatus = 0;
                InterruptEnable = 0;
                ServiceRequestSelector = 0;
                ShadowTransferStatus = 0;
                RunBit = false;
                CountingDown = false;
                status[0] = status[1] = false;
                statusOutput = false;
                for(var i = 0; i < 4; i++)
                {
                    gates[i] = true;
                    outputLevels[i] = false;
                    captureValues[i] = 0;
                    capturePrescaler[i] = 0;
                    captureFull[i] = false;
                }
                position = 0;
            }

            public int Index { get; }
            public uint InputSelector { get; set; }
            public uint TimerControl { get; private set; }
            public uint PassiveLevel { get; private set; }
            public uint PassiveLevelShadow { get; set; }
            public uint Dither { get; private set; }
            public uint DitherShadow { get; set; }
            public uint PrescalerControl { get; private set; }
            public uint FloatingPrescaler { get; private set; }
            public uint FloatingPrescalerShadow { get; set; }
            public uint Period { get; private set; }
            public uint PeriodShadow { get; set; }
            public uint[] Compare { get; } = new uint[2];
            public uint[] CompareShadow { get; } = new uint[2];
            public uint ChannelControl { get; private set; }
            public uint DeadTimeControl { get; private set; }
            public uint[] DeadTimeValues { get; } = new uint[2];
            public uint InterruptStatus { get; set; }
            public uint InterruptEnable { get; set; }
            public uint ServiceRequestSelector { get; set; }
            // bit 0 period/compare/passive level, bit 1 dither, bit 2 prescaler
            public uint ShadowTransferStatus { get; private set; }
            public bool RunBit { get; private set; }
            public bool CountingDown { get; private set; }

            // CMOD (bit 3) is set by hardware while an event is linked to a capture function
            public uint TimerControlRead => TimerControl | (CaptureMode ? 8u : 0u);

            // PVAL (the actual prescaler value) is bits 11:8, PCMP is bits 3:0
            public uint FloatingPrescalerRead => ((PrescalerControl & 0xF) << 8) | FloatingPrescaler;

            // the compare registers are not accessible while their capture registers are in use
            public bool CaptureModeLow => (connectionMatrix & 0x30) != 0;
            public bool CaptureModeHigh => (connectionMatrix & 0xC0) != 0;

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

            public bool GetStatus(int channel)
            {
                return status[channel];
            }

            public void SetTimerControl(uint value)
            {
                TimerControl = value;
                UpdateStatusOutput();
                Reconfigure();
            }

            public void SetPrescalerControl(uint value)
            {
                PrescalerControl = value;
                Reconfigure();
            }

            public void SetChannelControl(uint value)
            {
                ChannelControl = value;
                UpdateOutputs();
            }

            public void SetDeadTimeControl(uint value)
            {
                DeadTimeControl = value;
                UpdateOutputs();
            }

            public void WriteTimerValue(uint value)
            {
                if(RunBit)
                {
                    unit.Log(LogLevel.Warning, "CC8{0}: TIMER written while the timer is running, ignored", Index);
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
                if(value && position == 0)
                {
                    // a compare value of zero is active from the very beginning
                    ApplyAlwaysActive();
                }
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

            public void SetStatus(int channel, bool value)
            {
                if(status[channel] == value)
                {
                    return;
                }
                status[channel] = value;
                StartDeadTime(channel, value);
                UpdateOutputs();
                UpdateStatusOutput();
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

            public bool DeadTimeRunning(int channel)
            {
                return deadTimeTimers[2 * channel].Enabled || deadTimeTimers[2 * channel + 1].Enabled;
            }

            public void ClearDeadTime(int channel)
            {
                deadTimeTimers[2 * channel].Enabled = false;
                deadTimeTimers[2 * channel + 1].Enabled = false;
                gates[2 * channel] = true;
                gates[2 * channel + 1] = true;
                UpdateOutputs();
            }

            // Reading a capture register returns the value, the prescaler value at the time of the capture and the full flag; clears the flag
            public uint ReadCaptureRegister(int register)
            {
                if(!CaptureMode)
                {
                    return 0;
                }
                var result = captureValues[register] | (capturePrescaler[register] << 16) | (captureFull[register] ? 1u << 20 : 0u);
                captureFull[register] = false;
                return result;
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

            public void UpdateOutputs()
            {
                // an idle slice keeps its outputs passive
                var enabled = !unit.IsIdle(Index);
                var asymmetric = AsymmetricPwm;
                for(var output = 0; output < 4; output++)
                {
                    var channel = output / 2;
                    var path = false;
                    if(enabled && !(asymmetric && channel == 1))
                    {
                        var st = status[channel];
                        var timedChannel = (DeadTimeControl & (1u << channel)) != 0;
                        var gatedStPath = timedChannel && (DeadTimeControl & (1u << (2 + 2 * channel))) != 0;
                        var gatedInvertedPath = timedChannel && (DeadTimeControl & (1u << (3 + 2 * channel))) != 0;
                        var stPath = st && (!gatedStPath || gates[2 * channel]);
                        var invertedPath = !st && (!gatedInvertedPath || gates[2 * channel + 1]);
                        // OCS1 and OCS3: 0 = status path, OCS2 and OCS4: 0 = inverted status path
                        var selected = (ChannelControl & (2u << output)) != 0;
                        path = (output & 1) == 0 ? (selected ? invertedPath : stPath) : (selected ? stPath : invertedPath);
                    }
                    var level = path ^ ((PassiveLevel & (1u << output)) != 0);
                    if(outputLevels[output] != level)
                    {
                        outputLevels[output] = level;
                        unit.SetOutput(Index, output, level);
                    }
                }
            }

            private void ApplyAlwaysActive()
            {
                if(CaptureMode)
                {
                    return;
                }
                if(AsymmetricPwm)
                {
                    if(Compare[0] == 0)
                    {
                        SetStatus(0, true);
                    }
                    return;
                }
                for(var channel = 0; channel < 2; channel++)
                {
                    if(Compare[channel] == 0)
                    {
                        SetStatus(channel, true);
                    }
                }
            }

            private void StartDeadTime(int channel, bool rising)
            {
                var timedChannel = (DeadTimeControl & (1u << channel)) != 0;
                var delayedGate = 2 * channel + (rising ? 0 : 1);
                var releasedGate = 2 * channel + (rising ? 1 : 0);
                deadTimeTimers[releasedGate].Enabled = false;
                gates[releasedGate] = true;
                deadTimeTimers[delayedGate].Enabled = false;

                var delay = timedChannel ? (DeadTimeValues[channel] >> (rising ? 0 : 8)) & 0xFF : 0;
                if(delay == 0)
                {
                    gates[delayedGate] = true;
                    return;
                }
                gates[delayedGate] = false;
                // dead time clock fDCLK = fTCLK / 2^DTCC
                var ticks = (ulong)delay * (1UL << (int)((DeadTimeControl >> 6) & 3)) * (ulong)Divider;
                var timer = deadTimeTimers[delayedGate];
                var frequency = unit.ClockFrequency;
                if(timer.Frequency != (ulong)frequency)
                {
                    timer.Frequency = (ulong)frequency;
                }
                timer.Value = 0;
                timer.Limit = ticks;
                timer.Enabled = true;
            }

            private void OnDeadTimeElapsed(int gate)
            {
                deadTimeTimers[gate].Enabled = false;
                gates[gate] = true;
                UpdateOutputs();
            }

            private void UpdateStatusOutput()
            {
                bool value;
                switch((TimerControl >> 29) & 3)
                {
                case 0:
                    value = status[0];
                    break;
                case 1:
                    value = status[1];
                    break;
                default:
                    value = status[0] && status[1];
                    break;
                }
                if(value != statusOutput)
                {
                    statusOutput = value;
                    unit.SetInput(StatusInputBase + Index, value);
                }
            }

            private void OnEvent(int e)
            {
                SetFlag(Event0Detected << e);

                var selector = (uint)(e + 1);
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
                var sameCaptureEvent = (TimerControl & (1u << 11)) != 0;
                if(((connectionMatrix >> 4) & 3) == selector && !sameCaptureEvent)
                {
                    Capture(0);
                }
                if(((connectionMatrix >> 6) & 3) == selector)
                {
                    Capture(sameCaptureEvent ? 2 : 1);
                }
                if(((connectionMatrix >> 14) & 3) == selector && !Concatenated)
                {
                    CountPulse();
                }
            }

            // Captures the timer into the register chain: set 0 = CV1 (old CV1 moves to CV0), set 1 = CV3 (CV3 to CV2),
            // set 2 (SCE = 1) = chain CV3 -> CV2 -> CV1 -> CV0. A register that was not read (full flag) is not overwritten
            // unless the continuous capture bit CCS is set.
            private void Capture(int set)
            {
                var value = CurrentPosition;
                var prescaler = PrescalerControl & 0xF;
                var continuous = (TimerControl & (1u << 12)) != 0;
                var first = set == 1 ? 2 : 0;
                var last = set == 0 ? 1 : 3;

                var oldFull = new bool[4];
                Array.Copy(captureFull, oldFull, 4);
                var oldValues = new uint[4];
                Array.Copy(captureValues, oldValues, 4);
                var oldPrescaler = new uint[4];
                Array.Copy(capturePrescaler, oldPrescaler, 4);

                // a register takes over its upper neighbour when that one is full and not all registers below and including it are full
                var allFullBelow = true;
                for(var i = first; i < last; i++)
                {
                    allFullBelow &= oldFull[i];
                    if(continuous || (oldFull[i + 1] && !allFullBelow))
                    {
                        captureValues[i] = oldValues[i + 1];
                        capturePrescaler[i] = oldPrescaler[i + 1];
                        captureFull[i] = oldFull[i + 1] || captureFull[i];
                    }
                }
                allFullBelow &= oldFull[last];
                if(!continuous && allFullBelow)
                {
                    return;
                }
                captureValues[last] = value;
                capturePrescaler[last] = prescaler;
                captureFull[last] = true;

                var clearMode = (TimerControl >> 5) & 3;
                var clear = clearMode == 3
                    || (clearMode == 1 && set != 0)
                    || (clearMode == 2 && set != 1)
                    || (SameCaptureEvent && clearMode != 0);
                if(clear)
                {
                    ClearTimer();
                }
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
                case EventKind.Compare1Up:
                    SetFlag(Compare1MatchUp);
                    SetStatus(0, true);
                    break;
                case EventKind.Compare2Up:
                    SetFlag(Compare2MatchUp);
                    if(AsymmetricPwm)
                    {
                        SetStatus(0, false);
                    }
                    else
                    {
                        SetStatus(1, true);
                    }
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
                case EventKind.Compare1Down:
                    SetFlag(Compare1MatchDown);
                    SetStatus(0, false);
                    break;
                case EventKind.Compare2Down:
                    SetFlag(Compare2MatchDown);
                    SetStatus(AsymmetricPwm ? 0 : 1, false);
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

                var changed = false;
                if(!CenterAligned && !CaptureMode && (ShadowTransferStatus & 1) != 0)
                {
                    // the new compare values decide about the status bits of the next period
                    changed = TransferShadow();
                }

                if(!CaptureMode)
                {
                    EvaluateZeroMatch();
                }

                position = 0;
                if(SingleShot)
                {
                    RunBit = false;
                    SetStatus(0, false);
                    SetStatus(1, false);
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

            // Status bits at timer value zero: an edge aligned channel is cleared, a channel with compare value zero is (still) active
            private void EvaluateZeroMatch()
            {
                if(AsymmetricPwm)
                {
                    if(Compare[1] == 0)
                    {
                        SetFlag(CenterAligned ? Compare2MatchDown : Compare2MatchUp);
                        SetStatus(0, false);
                    }
                    if(Compare[0] == 0)
                    {
                        SetFlag(CenterAligned ? Compare1MatchDown : Compare1MatchUp);
                        SetStatus(0, true);
                    }
                    return;
                }
                for(var channel = 0; channel < 2; channel++)
                {
                    var upFlag = channel == 0 ? Compare1MatchUp : Compare2MatchUp;
                    var downFlag = channel == 0 ? Compare1MatchDown : Compare2MatchDown;
                    if(Compare[channel] == 0)
                    {
                        SetFlag(CenterAligned ? downFlag : upFlag);
                        SetStatus(channel, true);
                    }
                    else if(!CenterAligned)
                    {
                        SetStatus(channel, false);
                    }
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

            // Returns true when the period or a compare value changed
            private bool TransferShadow()
            {
                var changed = false;
                if((ShadowTransferStatus & 1) != 0)
                {
                    changed = Period != PeriodShadow || Compare[0] != CompareShadow[0] || Compare[1] != CompareShadow[1];
                    Period = PeriodShadow;
                    Compare[0] = CompareShadow[0];
                    Compare[1] = CompareShadow[1];
                    if(PassiveLevel != PassiveLevelShadow)
                    {
                        PassiveLevel = PassiveLevelShadow;
                        UpdateOutputs();
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

                // SRS: POSR [1:0], CM1SR [3:2], CM2SR [5:4], E0SR [9:8], E1SR [11:10], E2SR [13:12]
                int shift;
                if((flag & (PeriodMatchUp | OneMatchDown)) != 0)
                {
                    shift = 0;
                }
                else if((flag & (Compare1MatchUp | Compare1MatchDown)) != 0)
                {
                    shift = 2;
                }
                else if((flag & (Compare2MatchUp | Compare2MatchDown)) != 0)
                {
                    shift = 4;
                }
                else if((flag & (Event0Detected << 3)) != 0)
                {
                    // trap flag, no service request of its own
                    return;
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
                var period = Period;
                if(CaptureMode)
                {
                    // the timer just runs up to the period, the captures are triggered by the events
                    Add(ref count, EventKind.Wrap, period + 1);
                    return count;
                }
                var asymmetric = AsymmetricPwm;
                var compare1 = Compare[0];
                var compare2 = Compare[1];
                if(!CenterAligned)
                {
                    if(compare1 >= 1 && compare1 <= period)
                    {
                        Add(ref count, EventKind.Compare1Up, compare1);
                    }
                    if(compare2 >= 1 && compare2 <= period)
                    {
                        Add(ref count, EventKind.Compare2Up, compare2);
                    }
                    Add(ref count, EventKind.Wrap, period + 1);
                }
                else
                {
                    if(compare1 >= 1 && compare1 <= period)
                    {
                        Add(ref count, EventKind.Compare1Up, compare1);
                        if(!asymmetric)
                        {
                            Add(ref count, EventKind.Compare1Down, 2 * period + 2 - compare1);
                        }
                    }
                    if(compare2 >= 1 && compare2 <= period)
                    {
                        if(!asymmetric)
                        {
                            Add(ref count, EventKind.Compare2Up, compare2);
                        }
                        Add(ref count, EventKind.Compare2Down, 2 * period + 2 - compare2);
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
                var level = unit.inputLevels[EventInput(e)];
                var activeLow = ((InputSelector >> (22 + e)) & 1) != 0;
                return level ^ activeLow;
            }

            private bool CaptureMode => (connectionMatrix & 0xF0) != 0;
            private bool SameCaptureEvent => (TimerControl & (1u << 11)) != 0;
            private bool CenterAligned => (TimerControl & 1) != 0 && !Concatenated && !CaptureMode;
            private bool SingleShot => (TimerControl & 2) != 0;
            private bool ClearTriggersShadowTransfer => (TimerControl & 4) != 0;
            private bool AsymmetricPwm => (ChannelControl & 1) != 0;
            private bool Concatenated => Index > 0 && (connectionMatrix & (1u << 20)) != 0;
            private bool ExternallyClocked => Concatenated || ((connectionMatrix >> 14) & 3) != 0;
            private int Divider => 1 << (int)(PrescalerControl & 0xF);
            private long CycleLength => CenterAligned ? 2 * (long)Period + 2 : (long)Period + 1;

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
            private uint position;
            private bool timersRunning;
            private bool statusOutput;
            private long activeLength = 1;
            private int activeDivider = 1;

            private readonly XMC4XXX_CCU8 unit;
            private readonly LimitTimer[] timers;
            private readonly LimitTimer[] deadTimeTimers;
            private readonly bool[] status = new bool[2];
            // dead time gates: [2 * channel] enables the status path, [2 * channel + 1] the inverted status path
            private readonly bool[] gates = new bool[4] { true, true, true, true };
            private readonly bool[] outputLevels = new bool[4];
            private readonly uint[] captureValues = new uint[4];
            private readonly uint[] capturePrescaler = new uint[4];
            private readonly bool[] captureFull = new bool[4];
            private readonly EventKind[] eventKinds = new EventKind[eventKindCount];
            private readonly uint[] eventPositions = new uint[eventKindCount];

            private const int eventKindCount = 7;
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
            ParityCheckerConfiguration = 0x1C,
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
            Compare1 = 0x38,
            Compare1Shadow = 0x3C,
            Compare2 = 0x40,
            Compare2Shadow = 0x44,
            ChannelControl = 0x48,
            DeadTimeControl = 0x4C,
            DeadTimeValues1 = 0x50,
            DeadTimeValues2 = 0x54,
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
