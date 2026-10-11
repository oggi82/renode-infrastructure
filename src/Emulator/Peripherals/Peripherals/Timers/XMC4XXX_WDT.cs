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
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    // Window watchdog timer of the Infineon XMC4xxx family (Reference Manual XMC4500 v1.6, chapter 9).
    // A 32-bit counter counts up with fWDT (selected in the SCU, WDTCLKCR) while CTR.ENB is set. A service (magic word
    // 0xABADCAFE written to SRV) is only valid while WLB <= TIM <= WUB and restarts the counter. A service outside of the
    // window or with a wrong word is a bad service and requests a system reset at once.
    // Time-out mode: crossing WUB requests a system reset (ResetRequest line, the SCU performs the reset and records it in RSTSTAT).
    // Pre-warning mode (CTR.PRE): the first crossing sets WDTSTS.ALMS and pulses Alarm (SCU service request PRWARN), the counter
    // then restarts from 0 and the next crossing requests the reset unless ALMS has been cleared by WDTCLR.ALMC meanwhile.
    // The counter stops while the WDT clock is disabled in the SCU (CLKSTAT.WDTCST = 0). Asserting the peripheral reset
    // (PRSET2.WDTRS) resets the module; the reset state of PRSTAT2.WDTRS itself does not block register access.
    // The service indication pulse (wdt_service) and the debug suspend (CTR.DSP) are not modeled.
    public class XMC4XXX_WDT : BasicDoubleWordPeripheral, IKnownSize
    {
        public XMC4XXX_WDT(IMachine machine, XMC4XXX_SCU scu) : base(machine)
        {
            this.scu = scu;
            Alarm = new GPIO();
            ResetRequest = new GPIO();
            timer = new LimitTimer(machine.ClockSource, 1000000, this, "wdt", DefaultUpperBound, direction: Direction.Ascending, workMode: WorkMode.Periodic, enabled: false, eventEnabled: true);
            timer.LimitReached += OnUpperBoundCrossed;
            DefineRegisters();
            scu.WatchdogClockChanged += UpdateTimer;
            scu.PeripheralResetAsserted += OnPeripheralReset;
        }

        public override void Reset()
        {
            base.Reset();
            alarmStatus = false;
            timer.Enabled = false;
            timer.Value = 0;
            timer.Limit = DefaultUpperBound;
            Alarm.Unset();
            ResetRequest.Unset();
            UpdateTimer();
        }

        public long Size => 0x100;

        public GPIO Alarm { get; }

        public GPIO ResetRequest { get; }

        // Number of reset requests and pre-warning alarms since the start of the simulation (not cleared by reset; for tests)
        public uint ResetRequestCount { get; private set; }

        public uint PreWarningCount { get; private set; }

        private void DefineRegisters()
        {
            Registers.Id.Define(this, 0x00ADC000)
                .WithValueField(0, 32, FieldMode.Read, name: "ID");

            Registers.Control.Define(this)
                .WithFlag(0, out enabled, name: "ENB", changeCallback: (_, __) => UpdateTimer())
                .WithFlag(1, out preWarning, name: "PRE")
                .WithReservedBits(2, 2)
                .WithFlag(4, name: "DSP")
                .WithReservedBits(5, 3)
                .WithValueField(8, 8, name: "SPW")
                .WithReservedBits(16, 16);

            Registers.Service.Define(this)
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) => Service((uint)value), name: "SRV");

            Registers.Timer.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => timer.Value, name: "TIM");

            Registers.WindowLowerBound.Define(this)
                .WithValueField(0, 32, out lowerBound, name: "WLB");

            Registers.WindowUpperBound.Define(this, DefaultUpperBound)
                .WithValueField(0, 32, out upperBound, name: "WUB", changeCallback: (_, value) => timer.Limit = value == 0 ? 1 : value);

            Registers.Status.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => alarmStatus, name: "ALMS")
                .WithReservedBits(1, 31);

            Registers.Clear.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        alarmStatus = false;
                    }
                }, name: "ALMC")
                .WithReservedBits(1, 31);
        }

        private void Service(uint value)
        {
            if(!enabled.Value)
            {
                this.Log(LogLevel.Debug, "Service while the watchdog is disabled, ignored");
                return;
            }
            var current = timer.Value;
            if(value != MagicWord)
            {
                this.Log(LogLevel.Warning, "Bad service: wrong magic word 0x{0:X}", value);
                RequestSystemReset();
            }
            else if(current < lowerBound.Value)
            {
                this.Log(LogLevel.Warning, "Bad service: timer value {0} is below the window lower bound {1}", current, lowerBound.Value);
                RequestSystemReset();
            }
            else
            {
                this.Log(LogLevel.Debug, "Serviced at timer value {0}", current);
                timer.Value = 0;
            }
        }

        private void OnUpperBoundCrossed()
        {
            if(preWarning.Value && !alarmStatus)
            {
                this.Log(LogLevel.Info, "Pre-warning alarm");
                alarmStatus = true;
                PreWarningCount++;
                Alarm.Set();
                Alarm.Unset();
                // the counter restarts for the second crossing (periodic timer wrapped to 0)
                return;
            }
            this.Log(LogLevel.Warning, "Timer crossed the window upper bound, requesting system reset");
            RequestSystemReset();
        }

        private void RequestSystemReset()
        {
            timer.Enabled = false;
            ResetRequestCount++;
            ResetRequest.Set();
            ResetRequest.Unset();
        }

        // The timer runs while the watchdog is enabled and its clock is on
        private void UpdateTimer()
        {
            var frequency = scu.WatchdogClockFrequency;
            if(frequency != 0)
            {
                timer.Frequency = frequency;
            }
            timer.Enabled = enabled.Value && frequency != 0;
        }

        private void OnPeripheralReset(int register, uint bits)
        {
            if(register == 2 && (bits & WatchdogResetBit) != 0)
            {
                // PRSET2.WDTRS: the module is reset, the registers return to their reset values
                Reset();
            }
        }

        private IFlagRegisterField enabled;
        private IFlagRegisterField preWarning;
        private IValueRegisterField lowerBound;
        private IValueRegisterField upperBound;
        private bool alarmStatus;

        private readonly LimitTimer timer;
        private readonly XMC4XXX_SCU scu;

        private const uint MagicWord = 0xABADCAFE;
        private const uint DefaultUpperBound = 0xFFFFFFFF;
        private const uint WatchdogResetBit = 1u << 1;

        private enum Registers : long
        {
            Id = 0x00,
            Control = 0x04,
            Service = 0x08,
            Timer = 0x0C,
            WindowLowerBound = 0x10,
            WindowUpperBound = 0x14,
            Status = 0x18,
            Clear = 0x1C,
        }
    }
}
