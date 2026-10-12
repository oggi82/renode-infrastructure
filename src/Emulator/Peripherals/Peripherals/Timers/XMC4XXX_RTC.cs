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
    // Real time clock of the hibernate domain of the Infineon XMC4xxx family (Reference Manual XMC4500 v1.6, chapter 10).
    // The 32.768 kHz clock (fOSI or fULP, both are 32.768 kHz here) is divided by CTR.DIV + 1; every tick of the divider
    // increments the seconds, with carry into minutes, hours, days (and day of week), months and years (leap years included).
    // Every increment of a field sets the matching periodic bit in RAWSTAT; if all compare fields of ATIM0/ATIM1
    // (seconds, minutes, hours, days, months, years) equal the time after a tick, RAWSTAT.RAI is set.
    // Requests are signalled to the SCU as pulses on PeriodicIRQ (SRRAW.PI) and AlarmIRQ (SRRAW.AI) when the matching
    // MSKSR bit is set. WakeupIRQ is the wake-up trigger for the hibernate control (CTR.TAE and the E*C compare enables).
    // The RTC belongs to the hibernate domain: it is not reset by a watchdog reset or by a wake-up from hibernate.
    // The mirror register transfer between the core and the hibernate domain completes instantly (SCU MIRRSTS stays 0, the
    // update bits RTC_CTR..RTC_TIM1 in SRRAW are set after each write). TIM0/TIM1 can only be written while ENB = 0.
    public class XMC4XXX_RTC : BasicDoubleWordPeripheral, IKnownSize
    {
        public XMC4XXX_RTC(IMachine machine, XMC4XXX_SCU scu) : base(machine)
        {
            this.scu = scu;
            PeriodicIRQ = new GPIO();
            AlarmIRQ = new GPIO();
            WakeupIRQ = new GPIO();
            timer = new LimitTimer(machine.ClockSource, ClockFrequency, this, "rtc", DefaultDivider + 1, direction: Direction.Ascending, workMode: WorkMode.Periodic, enabled: false, eventEnabled: true);
            timer.LimitReached += Tick;
            DefineRegisters();
        }

        public override void Reset()
        {
            if(scu.LastResetKeptHibernateDomain)
            {
                return;
            }
            base.Reset();
            timer.Enabled = false;
            timer.Value = 0;
            timer.Limit = DefaultDivider + 1;
            seconds = minutes = hours = days = daysOfWeek = months = 0;
            year = 0;
            alarmSeconds = alarmMinutes = alarmHours = alarmDays = alarmMonths = 0;
            alarmYear = 0;
            rawStatus = 0;
            mask = 0;
        }

        public long Size => 0x100;

        public GPIO PeriodicIRQ { get; }

        public GPIO AlarmIRQ { get; }

        public GPIO WakeupIRQ { get; }

        private void DefineRegisters()
        {
            Registers.Id.Define(this, 0x00A3C000)
                .WithValueField(0, 32, FieldMode.Read, name: "ID");

            Registers.Control.Define(this, 0x7FFF0000)
                .WithFlag(0, out enabled, name: "ENB")
                .WithReservedBits(1, 1)
                .WithFlag(2, out alarmWakeup, name: "TAE")
                .WithReservedBits(3, 5)
                .WithValueField(8, 7, out wakeupFields, name: "ESEC_EMIC_EHOC_EDAC_(res)_EMOC_EYEC")
                .WithReservedBits(15, 1)
                .WithValueField(16, 16, out divider, name: "DIV")
                .WithChangeCallback((_, __) =>
                {
                    timer.Limit = divider.Value + 1;
                    timer.Enabled = enabled.Value;
                    scu.NotifyMirrorUpdate(XMC4XXX_SCU.RtcMirror.Control);
                });

            Registers.RawStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => rawStatus, name: "RAWSTAT");

            Registers.Status.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => rawStatus & mask, name: "STSSR");

            Registers.Mask.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => mask, writeCallback: (_, value) => mask = (uint)value & StatusMask, name: "MSKSR");

            Registers.Clear.Define(this)
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) => rawStatus &= ~(uint)value, name: "CLRSR");

            Registers.AlarmTime0.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => Pack0(alarmSeconds, alarmMinutes, alarmHours, alarmDays), writeCallback: (_, value) =>
                {
                    alarmSeconds = Limited(value, 0, 6, 60);
                    alarmMinutes = Limited(value, 8, 6, 60);
                    alarmHours = Limited(value, 16, 5, 24);
                    alarmDays = Limited(value, 24, 5, 31);
                    scu.NotifyMirrorUpdate(XMC4XXX_SCU.RtcMirror.AlarmTime0);
                }, name: "ATIM0");

            Registers.AlarmTime1.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => (alarmMonths << 8) | (alarmYear << 16), writeCallback: (_, value) =>
                {
                    alarmMonths = Limited(value, 8, 4, 12);
                    alarmYear = (uint)(value >> 16) & 0xFFFF;
                    scu.NotifyMirrorUpdate(XMC4XXX_SCU.RtcMirror.AlarmTime1);
                }, name: "ATIM1");

            Registers.Time0.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => Pack0(seconds, minutes, hours, days), writeCallback: (_, value) =>
                {
                    if(enabled.Value)
                    {
                        this.Log(LogLevel.Warning, "TIM0 can only be written while the RTC is disabled");
                        return;
                    }
                    seconds = Limited(value, 0, 6, 60);
                    minutes = Limited(value, 8, 6, 60);
                    hours = Limited(value, 16, 5, 24);
                    days = Limited(value, 24, 5, 31);
                    scu.NotifyMirrorUpdate(XMC4XXX_SCU.RtcMirror.Time0);
                }, name: "TIM0");

            Registers.Time1.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => daysOfWeek | (months << 8) | (year << 16), writeCallback: (_, value) =>
                {
                    if(enabled.Value)
                    {
                        this.Log(LogLevel.Warning, "TIM1 can only be written while the RTC is disabled");
                        return;
                    }
                    daysOfWeek = Limited(value, 0, 3, 7);
                    months = Limited(value, 8, 4, 12);
                    year = (uint)(value >> 16) & 0xFFFF;
                    if(days >= DaysInMonth(year, months))
                    {
                        days = 0;
                    }
                    scu.NotifyMirrorUpdate(XMC4XXX_SCU.RtcMirror.Time1);
                }, name: "TIM1");
        }

        private void Tick()
        {
            var before = rawStatus;
            rawStatus |= SecondsEvent;
            seconds++;
            if(seconds >= 60)
            {
                seconds = 0;
                minutes++;
                rawStatus |= MinutesEvent;
                if(minutes >= 60)
                {
                    minutes = 0;
                    hours++;
                    rawStatus |= HoursEvent;
                    if(hours >= 24)
                    {
                        hours = 0;
                        days++;
                        daysOfWeek = (daysOfWeek + 1) % 7;
                        rawStatus |= DaysEvent;
                        if(days >= DaysInMonth(year, months))
                        {
                            days = 0;
                            months++;
                            rawStatus |= MonthsEvent;
                            if(months >= 12)
                            {
                                months = 0;
                                year++;
                                rawStatus |= YearsEvent;
                            }
                        }
                    }
                }
            }

            var alarm = seconds == alarmSeconds && minutes == alarmMinutes && hours == alarmHours
                && days == alarmDays && months == alarmMonths && year == alarmYear;
            if(alarm)
            {
                this.Log(LogLevel.Debug, "Alarm at {0:D4}-{1:D2}-{2:D2} {3:D2}:{4:D2}:{5:D2}", year, months + 1, days + 1, hours, minutes, seconds);
                rawStatus |= AlarmEvent;
            }

            var newEvents = rawStatus & ~before;
            if((newEvents & mask & PeriodicEvents) != 0)
            {
                PeriodicIRQ.Set();
                PeriodicIRQ.Unset();
            }
            if((newEvents & mask & AlarmEvent) != 0)
            {
                AlarmIRQ.Set();
                AlarmIRQ.Unset();
            }
            // wake-up trigger for the hibernate control: alarm (CTR.TAE) or a periodic event of an enabled field (CTR.E*C)
            if((alarm && alarmWakeup.Value) || (newEvents & (uint)wakeupFields.Value & PeriodicEvents) != 0)
            {
                WakeupIRQ.Set();
                WakeupIRQ.Unset();
            }
        }

        private static uint Limited(ulong value, int position, int width, uint limit)
        {
            var field = (uint)(value >> position) & ((1u << width) - 1);
            return field >= limit ? 0 : field;
        }

        private static uint Pack0(uint seconds, uint minutes, uint hours, uint days)
        {
            return seconds | (minutes << 8) | (hours << 16) | (days << 24);
        }

        private static uint DaysInMonth(uint year, uint month)
        {
            switch(month)
            {
            case 1:
                return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 29u : 28u;
            case 3:
            case 5:
            case 8:
            case 10:
                return 30;
            default:
                return 31;
            }
        }

        private IFlagRegisterField enabled;
        private IFlagRegisterField alarmWakeup;
        private IValueRegisterField wakeupFields;
        private IValueRegisterField divider;

        private uint rawStatus;
        private uint mask;
        private uint seconds;
        private uint minutes;
        private uint hours;
        private uint days;
        private uint daysOfWeek;
        private uint months;
        private uint year;
        private uint alarmSeconds;
        private uint alarmMinutes;
        private uint alarmHours;
        private uint alarmDays;
        private uint alarmMonths;
        private uint alarmYear;

        private readonly LimitTimer timer;
        private readonly XMC4XXX_SCU scu;

        private const long ClockFrequency = 32768;
        private const uint DefaultDivider = 0x7FFF;

        private const uint SecondsEvent = 1u << 0;
        private const uint MinutesEvent = 1u << 1;
        private const uint HoursEvent = 1u << 2;
        private const uint DaysEvent = 1u << 3;
        private const uint MonthsEvent = 1u << 5;
        private const uint YearsEvent = 1u << 6;
        private const uint AlarmEvent = 1u << 8;
        private const uint PeriodicEvents = SecondsEvent | MinutesEvent | HoursEvent | DaysEvent | MonthsEvent | YearsEvent;
        private const uint StatusMask = PeriodicEvents | AlarmEvent;

        private enum Registers : long
        {
            Id = 0x00,
            Control = 0x04,
            RawStatus = 0x08,
            Status = 0x0C,
            Mask = 0x10,
            Clear = 0x14,
            AlarmTime0 = 0x18,
            AlarmTime1 = 0x1C,
            Time0 = 0x20,
            Time1 = 0x24,
        }
    }
}
