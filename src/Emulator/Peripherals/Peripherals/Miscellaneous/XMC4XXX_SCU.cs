//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.IRQControllers;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // System Control Unit of the Infineon XMC4xxx family (Reference Manual XMC4500 v1.6, chapter 11).
    // Modeled blocks:
    //  - service requests (SRSTAT/SRRAW/SRMSK/SRCLR/SRSET, NMIREQEN) with the combined interrupt line (SCU_0, NVIC IRQ 0)
    //    and the promotion of requests to NMI,
    //  - trap generation (TRAPSTAT/TRAPRAW/TRAPDIS/TRAPCLR/TRAPSET) towards the NMI,
    //  - power control (PWRSTAT/PWRSET/PWRCLR, EVR supply measurement, PWRMON brown-out monitor),
    //  - hibernate domain (HDSTAT/HDCLR/HDSET/HDCR, OSCSICTRL, OSCULCTRL) incl. hibernate request and wake-up,
    //  - reset control (RSTSTAT/RSTSET/RSTCLR with the sticky reset cause, PRSTATn/PRSETn/PRCLRn),
    //  - clock control (CLKSTAT/CLKSET/CLKCLR, the clock dividers, WDT clock, SLEEPCR/DSLEEPCR),
    //  - the high performance oscillator and the system PLL; the SysTick clock of the NVIC follows the configured fCPU.
    // Status bits that the hardware updates asynchronously (PLL lock, oscillator watchdog, mirror updates of the hibernate
    // domain) are reflected immediately. Parity checking, the USB PLL lock and the die temperature sensor are not modeled.
    //
    // Inputs (OnGPIO number): 0 WDT pre-warning alarm, 1 RTC periodic event, 2 RTC alarm event, 3 DLR overrun,
    // 4 WDT reset request, 5 RTC hibernate wake-up trigger, 6 hibernate wake-up pin event (rising edge, positive edge event),
    // 7 hibernate wake-up pin event (falling edge), 16-19 ERU0 outputs 0-3 (NMI request promotion).
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class XMC4XXX_SCU : BasicDoubleWordPeripheral, IKnownSize, IGPIOReceiver
    {
        public XMC4XXX_SCU(IMachine machine, NVIC nvic = null, uint oscillatorFrequency = 12000000) : base(machine)
        {
            this.nvic = nvic;
            this.oscillatorFrequency = oscillatorFrequency;
            IRQ = new GPIO();
            ResetPersistentState();
            ResetCoreDomainState();
            DefineRegisters();
        }

        public override void Reset()
        {
            // The hibernate domain (and with it the reset cause) survives a system reset requested by the WDT or by a hibernate
            // wake-up; every other reset behaves like a power-on reset.
            lastResetWasWarm = warmResetPending;
            warmResetPending = false;
            var cause = pendingResetCause;
            pendingResetCause = ResetCausePowerOn;
            if(lastResetWasWarm)
            {
                resetCause = cause;
            }
            else
            {
                ResetPersistentState();
            }

            base.Reset();
            ResetCoreDomainState();
            IRQ.Unset();
            UpdateCpuClock();
            WatchdogClockChanged?.Invoke();
        }

        public void OnGPIO(int number, bool value)
        {
            if(!value)
            {
                return;
            }
            switch(number)
            {
            case 0:
                RaiseServiceRequest(ServiceRequest.WatchdogPreWarning);
                break;
            case 1:
                RaiseServiceRequest(ServiceRequest.RtcPeriodic);
                break;
            case 2:
                RaiseServiceRequest(ServiceRequest.RtcAlarm);
                break;
            case 3:
                RaiseServiceRequest(ServiceRequest.DlrOverrun);
                break;
            case 4:
                RequestWatchdogReset();
                break;
            case 5:
                HibernateEvent(HibernateStatus.RtcEvent);
                break;
            case 6:
                HibernateEvent(HibernateStatus.PositiveEdgeEvent);
                break;
            case 7:
                HibernateEvent(HibernateStatus.NegativeEdgeEvent);
                break;
            case 16:
            case 17:
            case 18:
            case 19:
                // ERU0 output n can be promoted to an NMI (NMIREQEN bits 16-19)
                if((nmiRequestEnable & (1u << number)) != 0)
                {
                    nvic?.SetPendingIRQ(NmiException);
                }
                break;
            default:
                this.Log(LogLevel.Warning, "Unsupported input line {0}", number);
                break;
            }
        }

        // Called by the RTC after a write to one of its registers; the update of the hibernate domain mirror is complete at once.
        public void NotifyMirrorUpdate(RtcMirror register)
        {
            RaiseServiceRequest((ServiceRequest)(1u << (24 + (int)register)));
        }

        public long Size => 0x800;

        public GPIO IRQ { get; }

        // Frequency of fWDT in Hz; 0 while the WDT clock is disabled (CLKSTAT.WDTCST = 0)
        public uint WatchdogClockFrequency
        {
            get
            {
                if((clockStatus & ClockWatchdog) == 0)
                {
                    return 0;
                }
                var selection = (RegistersCollection.Read((long)Registers.WatchdogClockControl) >> 16) & 3;
                uint source;
                switch(selection)
                {
                case 1:
                    source = StandbyClockFrequency;
                    break;
                case 2:
                    source = GetSystemPllFrequency();
                    break;
                default:
                    source = InternalOscillatorFrequency;
                    break;
                }
                return source / ((RegistersCollection.Read((long)Registers.WatchdogClockControl) & 0xFF) + 1);
            }
        }

        // true after a system reset that did not reset the hibernate domain (watchdog reset, wake-up from hibernate)
        public bool LastResetKeptHibernateDomain => lastResetWasWarm;

        public bool IsHibernateDomainEnabled => hibernateDomainEnabled;

        // Called by the WDT (and tests) whenever fWDT may have changed
        public event Action WatchdogClockChanged;

        // Called with the index of PRSET register and the bits that were set
        public event Action<int, uint> PeripheralResetAsserted;

        // Voltage at the VDDP pin in volts, evaluated by the EVR measurement and the brown-out monitor (PWRMON)
        public double SupplyVoltage
        {
            get => supplyVoltage;
            set
            {
                supplyVoltage = value;
                CheckSupplyMonitor();
            }
        }

        private void DefineRegisters()
        {
            DefineGeneralRegisters();
            DefineServiceRequestRegisters();
            DefineParityRegisters();
            DefineTrapRegisters();
            DefinePowerRegisters();
            DefineHibernateRegisters();
            DefineResetRegisters();
            DefineClockRegisters();
            DefineOscillatorAndPllRegisters();
        }

        private void DefineGeneralRegisters()
        {
            Registers.Id.Define(this, 0x00A0C003)
                .WithValueField(0, 32, FieldMode.Read, name: "ID");

            Registers.ChipId.Define(this, 0x00045008)
                .WithValueField(0, 32, FieldMode.Read, name: "IDCHIP");

            Registers.ManufacturerId.Define(this, 0x000003E0)
                .WithValueField(0, 32, FieldMode.Read, name: "IDMANUF");

            Registers.StartupConfiguration.Define(this)
                .WithValueField(0, 2, FieldMode.Read, name: "HWCON")
                .WithReservedBits(2, 6)
                .WithValueField(8, 4, name: "SWCON")
                .WithReservedBits(12, 20);

            Registers.GeneralPurpose0.Define(this)
                .WithValueField(0, 32, name: "GPR0");
            Registers.GeneralPurpose1.Define(this)
                .WithValueField(0, 32, name: "GPR1");

            Registers.CcuGlobalStart.Define(this)
                .WithValueField(0, 4, name: "GSC4x")
                .WithReservedBits(4, 4)
                .WithValueField(8, 2, name: "GSC8x")
                .WithReservedBits(10, 22);

            // The hibernate domain, the RTC and the retention memory are written through mirror registers; updates are applied instantly.
            Registers.MirrorStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => 0, name: "MIRRSTS");
        }

        private void DefineServiceRequestRegisters()
        {
            Registers.ServiceRequestStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => serviceRequestRaw & serviceRequestMask, name: "SRSTAT");

            Registers.ServiceRequestRaw.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => serviceRequestRaw, name: "SRRAW");

            Registers.ServiceRequestMask.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => serviceRequestMask, writeCallback: (_, value) =>
                {
                    serviceRequestMask = (uint)value & ServiceRequestValidMask;
                    UpdateInterrupts();
                }, name: "SRMSK");

            Registers.ServiceRequestClear.Define(this)
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) =>
                {
                    serviceRequestRaw &= ~(uint)value;
                    UpdateInterrupts();
                }, name: "SRCLR");

            Registers.ServiceRequestSet.Define(this)
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) => RaiseServiceRequest((ServiceRequest)((uint)value & ServiceRequestValidMask)), name: "SRSET");

            Registers.NmiRequestEnable.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => nmiRequestEnable, writeCallback: (_, value) =>
                {
                    nmiRequestEnable = (uint)value & NmiRequestValidMask;
                    UpdateInterrupts();
                }, name: "NMIREQEN");
        }

        private void DefineParityRegisters()
        {
            // Parity errors are never generated; the registers keep their values so that the XMC library can configure them.
            Registers.ParityErrorEnable.Define(this)
                .WithValueField(0, 32, name: "PEEN");
            Registers.MemoryCheckingControl.Define(this)
                .WithValueField(0, 32, name: "MCHKCON");
            Registers.ParityErrorTrapEnable.Define(this)
                .WithValueField(0, 32, name: "PETE");
            Registers.ParityErrorResetEnable.Define(this)
                .WithValueField(0, 32, name: "PERSTEN");
            Registers.ParityErrorFlag.Define(this)
                .WithValueField(0, 32, FieldMode.Read | FieldMode.WriteOneToClear, name: "PEFLAG");
            Registers.ParityMemoryTestPattern.Define(this)
                .WithValueField(0, 32, name: "PMTPR");
            Registers.ParityMemoryTestSelect.Define(this)
                .WithValueField(0, 32, name: "PMTSR");
        }

        private void DefineTrapRegisters()
        {
            Registers.TrapStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => trapRaw & ~trapDisable & TrapValidMask, name: "TRAPSTAT");

            Registers.TrapRaw.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => trapRaw, name: "TRAPRAW");

            Registers.TrapDisable.Define(this, TrapValidMask)
                .WithValueField(0, 32, valueProviderCallback: _ => trapDisable, writeCallback: (_, value) =>
                {
                    trapDisable = (uint)value & TrapValidMask;
                    UpdateInterrupts();
                }, name: "TRAPDIS");

            Registers.TrapClear.Define(this)
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) =>
                {
                    trapRaw &= ~(uint)value;
                    UpdateInterrupts();
                }, name: "TRAPCLR");

            Registers.TrapSet.Define(this)
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) => SetTrap((uint)value & TrapValidMask), name: "TRAPSET");
        }

        private void DefinePowerRegisters()
        {
            DefineSetClear(Registers.PowerStatus, "PWR", Registers.PowerStatus + 4, Registers.PowerStatus + 8,
                () => (hibernateDomainEnabled ? 1u : 0u) | (usbPowerStatus << 16),
                bits =>
                {
                    hibernateDomainEnabled |= (bits & 1) != 0;
                    usbPowerStatus |= (bits >> 16) & 7;
                },
                bits =>
                {
                    hibernateDomainEnabled &= (bits & 1) == 0;
                    usbPowerStatus &= ~((bits >> 16) & 7);
                });

            Registers.EvrStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "EVRSTAT");

            Registers.EvrVadcStatus.Define(this)
                .WithValueField(0, 8, FieldMode.Read, valueProviderCallback: _ => (ulong)Math.Min(255, (int)(SupplyVoltageCore13 / Lsb13V) + 1), name: "VADC13V")
                .WithValueField(8, 8, FieldMode.Read, valueProviderCallback: _ => SupplyMeasurement, name: "VADC33V")
                .WithReservedBits(16, 16);

            // The supply is compared against THRS once when the monitor is enabled and whenever the voltage changes; INTV is not used for timing.
            Registers.PowerMonitor.Define(this)
                .WithValueField(0, 8, out supplyThreshold, name: "THRS")
                .WithValueField(8, 8, name: "INTV")
                .WithFlag(16, out supplyMonitorEnabled, name: "ENB")
                .WithReservedBits(17, 15)
                .WithChangeCallback((_, __) => CheckSupplyMonitor());
        }

        private void DefineHibernateRegisters()
        {
            DefineSetClear(Registers.HibernateStatus, "HD", Registers.HibernateStatus + 8, Registers.HibernateStatus + 4,
                () => hibernateStatus,
                bits =>
                {
                    hibernateStatus |= bits & HibernateStatusMask;
                    MirrorWritten(ServiceRequest.HdsetUpdated);
                },
                bits =>
                {
                    hibernateStatus &= ~(bits & HibernateStatusMask);
                    MirrorWritten(ServiceRequest.HdclrUpdated);
                });

            Registers.HibernateControl.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => hibernateControl, writeCallback: (_, value) =>
                {
                    hibernateControl = (uint)value & HibernateControlWriteMask;
                    MirrorWritten(ServiceRequest.HdcrUpdated);
                    if((hibernateControl & HibernateRequest) != 0)
                    {
                        EnterHibernate();
                    }
                }, name: "HDCR");

            Registers.OscillatorInternalControl.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => internalOscillatorControl, writeCallback: (_, value) =>
                {
                    internalOscillatorControl = (uint)value & 1;
                    MirrorWritten(ServiceRequest.OscSiCtrlUpdated);
                }, name: "OSCSICTRL");

            Registers.OscillatorUltraLowPowerStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "OSCULSTAT");

            Registers.OscillatorUltraLowPowerControl.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => ultraLowPowerOscillatorControl, writeCallback: (_, value) =>
                {
                    ultraLowPowerOscillatorControl = (uint)value & 0x31;
                    MirrorWritten(ServiceRequest.OscUlCtrlUpdated);
                }, name: "OSCULCTRL");
        }

        private void DefineResetRegisters()
        {
            // RSTSTAT: [7:0] cause of the last reset (sticky, cleared by RSTCLR.RSCLR), HIBWK, HIBRS, LCKEN
            Registers.ResetStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => resetCause | (hibernateWakeup ? 1u << 8 : 0u) | (hibernateReset ? 1u << 9 : 0u) | (lockupResetEnabled ? 1u << 10 : 0u), name: "RSTSTAT");

            (Registers.ResetStatus + 4).Define(this)
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) =>
                {
                    hibernateWakeup |= (value & (1 << 8)) != 0;
                    hibernateReset |= (value & (1 << 9)) != 0;
                    lockupResetEnabled |= (value & (1 << 10)) != 0;
                }, name: "RSTSET");
            (Registers.ResetStatus + 8).Define(this)
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) =>
                {
                    if((value & 1) != 0)
                    {
                        resetCause = 0;
                    }
                    hibernateWakeup &= (value & (1 << 8)) == 0;
                    hibernateReset &= (value & (1 << 9)) == 0;
                    lockupResetEnabled &= (value & (1 << 10)) == 0;
                }, name: "RSTCLR");

            for(var i = 0; i < PeripheralResetRegisterCount; i++)
            {
                var index = i;
                var registerBase = Registers.PeripheralResetStatus0 + i * SetClearGroupStride;
                DefineSetClear(registerBase, $"PR{i}", registerBase + 4, registerBase + 8,
                    () => peripheralReset[index],
                    bits =>
                    {
                        var changed = bits & PeripheralResetMasks[index] & ~peripheralReset[index];
                        peripheralReset[index] |= bits & PeripheralResetMasks[index];
                        if(changed != 0)
                        {
                            PeripheralResetAsserted?.Invoke(index, changed);
                        }
                    },
                    bits => peripheralReset[index] &= ~bits);
            }
        }

        private void DefineClockRegisters()
        {
            DefineSetClear(Registers.ClockStatus, "CLK", Registers.ClockStatus + 4, Registers.ClockStatus + 8,
                () => clockStatus,
                bits =>
                {
                    clockStatus |= bits & ClockMask;
                    WatchdogClockChanged?.Invoke();
                },
                bits =>
                {
                    clockStatus &= ~bits;
                    WatchdogClockChanged?.Invoke();
                });

            Registers.SystemClockControl.Define(this)
                .WithValueField(0, 8, name: "SYSDIV")
                .WithReservedBits(8, 8)
                .WithFlag(16, name: "SYSSEL")
                .WithReservedBits(17, 15)
                .WithChangeCallback((_, __) => UpdateCpuClock());

            Registers.CpuClockControl.Define(this)
                .WithFlag(0, name: "CPUDIV")
                .WithReservedBits(1, 31)
                .WithChangeCallback((_, __) => UpdateCpuClock());

            Registers.PeripheralBusClockControl.Define(this)
                .WithFlag(0, name: "PBDIV")
                .WithReservedBits(1, 31);

            Registers.UsbClockControl.Define(this)
                .WithValueField(0, 3, name: "USBDIV")
                .WithReservedBits(3, 13)
                .WithFlag(16, name: "USBSEL")
                .WithReservedBits(17, 15);

            Registers.ExternalBusClockControl.Define(this)
                .WithValueField(0, 6, name: "EBUDIV")
                .WithReservedBits(6, 26);

            Registers.CcuClockControl.Define(this)
                .WithFlag(0, name: "CCUDIV")
                .WithReservedBits(1, 31);

            Registers.WatchdogClockControl.Define(this)
                .WithValueField(0, 8, name: "WDTDIV")
                .WithReservedBits(8, 8)
                .WithValueField(16, 2, name: "WDTSEL")
                .WithReservedBits(18, 14)
                .WithChangeCallback((_, __) => WatchdogClockChanged?.Invoke());

            Registers.ExternalClockControl.Define(this)
                .WithValueField(0, 2, name: "ECKSEL")
                .WithReservedBits(2, 14)
                .WithValueField(16, 9, name: "ECKDIV")
                .WithReservedBits(25, 7);

            // Sleep mode: the CPU clock source (SYSSEL) and which peripheral clocks stay on (USB, MMC, ETH0, EBU, CCU, WDT).
            // Entering sleep is done by the CPU (WFI, SCB.SCR.SLEEPDEEP) and handled by the core model; the settings are stored only.
            Registers.SleepControl.Define(this)
                .WithValueField(0, 32, name: "SLEEPCR");

            // Deep sleep mode: as SLEEPCR plus the power-down request for flash (FPDN), PLL (PLLPDN) and VCO (VCOPDN).
            Registers.DeepSleepControl.Define(this)
                .WithValueField(0, 32, name: "DSLEEPCR");
        }

        private void DefineOscillatorAndPllRegisters()
        {
            Registers.OscillatorHighPerformanceStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "OSCHPSTAT");
            Registers.OscillatorHighPerformanceControl.Define(this, 0x3C)
                .WithValueField(0, 32, name: "OSCHPCTRL");
            Registers.ClockCalibrationConstant.Define(this)
                .WithValueField(0, 32, name: "CLKCALCONST");

            DefineSystemPll();

            Registers.UsbPllStatus.Define(this, 0x2)
                .WithValueField(0, 32, FieldMode.Read, name: "USBPLLSTAT");
            Registers.UsbPllControl.Define(this, 0x00010003)
                .WithValueField(0, 32, name: "USBPLLCON");

            Registers.ClockMultiplexingStatus.Define(this)
                .WithValueField(0, 2, FieldMode.Read, name: "SYSCLKMUX", valueProviderCallback: _ => SystemClockFromPll ? 2u : 1u)
                .WithReservedBits(2, 30);
        }

        private void DefineSystemPll()
        {
            // The PLL is powered down and bypassed after reset.
            Registers.SystemPllControl0.Define(this, 0x00030003)
                .WithFlag(0, out vcoBypass, name: "VCOBYP")
                .WithFlag(1, out vcoPowerDown, name: "VCOPWD")
                .WithFlag(2, name: "VCOTR")
                .WithReservedBits(3, 1)
                .WithFlag(4, name: "FINDIS")
                .WithReservedBits(5, 1)
                .WithFlag(6, name: "OSCDISCDIS")
                .WithReservedBits(7, 9)
                .WithFlag(16, out pllPowerDown, name: "PLLPWD")
                .WithFlag(17, out oscillatorWatchdogReset, name: "OSCRES")
                .WithFlag(18, FieldMode.Write, name: "RESLD")
                .WithFlag(19, name: "AOTREN")
                .WithFlag(20, name: "FOTR")
                .WithReservedBits(21, 11)
                .WithChangeCallback((_, __) => UpdateCpuClock());

            Registers.SystemPllControl1.Define(this)
                .WithValueField(0, 7, out k1Divider, name: "K1DIV")
                .WithReservedBits(7, 1)
                .WithValueField(8, 7, out feedbackDivider, name: "NDIV")
                .WithReservedBits(15, 1)
                .WithValueField(16, 7, out k2Divider, name: "K2DIV")
                .WithReservedBits(23, 1)
                .WithValueField(24, 4, out inputDivider, name: "PDIV")
                .WithReservedBits(28, 4)
                .WithChangeCallback((_, __) => UpdateCpuClock());

            Registers.SystemPllControl2.Define(this, 0x1)
                .WithFlag(0, out pllInputInternal, name: "PINSEL")
                .WithReservedBits(1, 7)
                .WithFlag(8, name: "K1INSEL")
                .WithReservedBits(9, 23)
                .WithChangeCallback((_, __) => UpdateCpuClock());

            Registers.SystemPllStatus.Define(this)
                .WithFlag(0, FieldMode.Read, name: "VCOBYST", valueProviderCallback: _ => vcoBypass.Value)
                .WithFlag(1, FieldMode.Read, name: "PWDSTAT", valueProviderCallback: _ => pllPowerDown.Value)
                .WithFlag(2, FieldMode.Read, name: "VCOLOCK", valueProviderCallback: _ => PllLocked)
                .WithReservedBits(3, 1)
                .WithFlag(4, FieldMode.Read, name: "K1RDY", valueProviderCallback: _ => PllLocked)
                .WithFlag(5, FieldMode.Read, name: "K2RDY", valueProviderCallback: _ => PllLocked)
                .WithFlag(6, FieldMode.Read, name: "BY", valueProviderCallback: _ => pllPowerDown.Value)
                // PLLLV, PLLHV, PLLSP: all three are set when the oscillator watchdog of the PLL considers the oscillator usable.
                .WithValueField(7, 3, FieldMode.Read, name: "PLLLV_PLLHV_PLLSP", valueProviderCallback: _ => oscillatorWatchdogReset.Value ? 0u : 0x7u)
                .WithReservedBits(10, 22);
        }

        // Defines the STAT/SET/CLR register triple used throughout the SCU: writing 1 to SET sets, writing 1 to CLR clears the status bit.
        // The order of the SET and CLR registers differs between the blocks (HDSTAT is followed by HDCLR, then HDSET).
        private void DefineSetClear(Registers statusRegister, string name, Registers setRegister, Registers clearRegister,
            Func<uint> status, Action<uint> set, Action<uint> clear)
        {
            statusRegister.Define(this, name: $"{name}STAT")
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => status(), name: "STATUS");

            setRegister.Define(this, name: $"{name}SET")
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) => set((uint)value), name: "SET");

            clearRegister.Define(this, name: $"{name}CLR")
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, value) => clear((uint)value), name: "CLR");
        }

        private void ResetPersistentState()
        {
            // Hibernate domain and reset cause: reset only by a power-on reset
            hibernateDomainEnabled = false;
            hibernateStatus = 0;
            hibernateControl = HibernateControlResetValue;
            internalOscillatorControl = 1;
            ultraLowPowerOscillatorControl = 0x20;
            hibernateWakeup = false;
            hibernateReset = true;
            hibernating = false;
            resetCause = ResetCausePowerOn;
            pendingResetCause = ResetCausePowerOn;
        }

        private void ResetCoreDomainState()
        {
            usbPowerStatus = 0;
            lockupResetEnabled = false;
            clockStatus = 0;
            Array.Copy(PeripheralResetValues, peripheralReset, PeripheralResetRegisterCount);
            serviceRequestRaw = 0;
            serviceRequestMask = 0;
            nmiRequestEnable = 0;
            trapRaw = 0;
            trapDisable = TrapValidMask;
            lastNmiRequest = 0;
        }

        private void RaiseServiceRequest(ServiceRequest request)
        {
            serviceRequestRaw |= (uint)request & ServiceRequestValidMask;
            UpdateInterrupts();
        }

        private void MirrorWritten(ServiceRequest request)
        {
            RaiseServiceRequest(request);
        }

        private void SetTrap(uint traps)
        {
            trapRaw |= traps;
            UpdateInterrupts();
        }

        // The combined service request is a level (SRSTAT != 0); the NVIC pends the interrupt again if the firmware leaves it set.
        // The NMI is raised whenever a new NMI source gets active.
        private void UpdateInterrupts()
        {
            var status = serviceRequestRaw & serviceRequestMask;
            IRQ.Set(status != 0);

            var nmiRequest = ((ulong)(trapRaw & ~trapDisable & TrapValidMask) << 32) | (status & nmiRequestEnable & ServiceRequestNmiMask);
            if((nmiRequest & ~lastNmiRequest) != 0)
            {
                this.Log(LogLevel.Noisy, "NMI request {0:X}", nmiRequest);
                nvic?.SetPendingIRQ(NmiException);
            }
            lastNmiRequest = nmiRequest;
        }

        private void CheckSupplyMonitor()
        {
            if(supplyMonitorEnabled == null || !supplyMonitorEnabled.Value)
            {
                return;
            }
            if(SupplyMeasurement < supplyThreshold.Value)
            {
                this.Log(LogLevel.Info, "Supply voltage {0} V below the brown-out threshold", supplyVoltage);
                SetTrap(TrapBrownOut);
            }
        }

        private void RequestWatchdogReset()
        {
            this.Log(LogLevel.Info, "Watchdog reset requested");
            pendingResetCause = ResetCauseWatchdog;
            warmResetPending = true;
            machine.RequestReset();
        }

        private void EnterHibernate()
        {
            if(!hibernateDomainEnabled)
            {
                this.Log(LogLevel.Warning, "Hibernate request ignored, the hibernate domain is not enabled (PWRSTAT.HIBEN = 0)");
                hibernateControl &= ~HibernateRequest;
                return;
            }
            if(hibernating)
            {
                return;
            }
            this.Log(LogLevel.Info, "Entering hibernate mode");
            hibernating = true;
            hibernateWakeup = false;
            hibernateStatus |= HibernateStatusOutput;
            foreach(var cpu in machine.SystemBus.GetCPUs().OfType<ICPU>())
            {
                cpu.IsHalted = true;
            }
        }

        // Wake-up event of the hibernate domain; the event is only effective if enabled in HDCR (WKPEP, WKPEN, RTCE)
        private void HibernateEvent(HibernateStatus status)
        {
            var enabled = (hibernateControl & (uint)status) != 0;
            if(!enabled)
            {
                return;
            }
            hibernateStatus |= (uint)status;
            if(!hibernating)
            {
                return;
            }
            this.Log(LogLevel.Info, "Wake-up from hibernate mode (status {0:X})", hibernateStatus);
            hibernating = false;
            hibernateControl &= ~HibernateRequest;
            hibernateStatus &= ~HibernateStatusOutput;
            hibernateWakeup = true;
            warmResetPending = true;
            pendingResetCause = ResetCausePowerOn;
            foreach(var cpu in machine.SystemBus.GetCPUs().OfType<ICPU>())
            {
                cpu.IsHalted = false;
            }
            machine.RequestReset();
        }

        // Keeps the SysTick of the NVIC running at the CPU clock: fCPU = fSYS / (CPUDIV + 1), fSYS = (OFI or PLL) / (SYSDIV + 1)
        private void UpdateCpuClock()
        {
            WatchdogClockChanged?.Invoke();
            if(nvic == null)
            {
                return;
            }

            var systemClockControl = RegistersCollection.Read((long)Registers.SystemClockControl);
            var cpuClockControl = RegistersCollection.Read((long)Registers.CpuClockControl);
            var frequency = SystemClockFromPll ? GetSystemPllFrequency() : InternalOscillatorFrequency;
            frequency /= (systemClockControl & 0xFF) + 1;
            frequency /= (cpuClockControl & 1) + 1;
            nvic.Frequency = frequency;
        }

        // Same formulas as SystemCoreClockUpdate of the XMC library: fPLL = fIN * (NDIV + 1) / ((PDIV + 1) * (K2DIV + 1)) in normal mode,
        // fIN / (K1DIV + 1) in prescaler mode (VCOBYP = 1) and fIN in bypass (PLLPWD = 1)
        private uint GetSystemPllFrequency()
        {
            var input = pllInputInternal != null && pllInputInternal.Value ? InternalOscillatorFrequency : oscillatorFrequency;
            if(pllPowerDown.Value)
            {
                return input;
            }
            if(vcoBypass.Value || vcoPowerDown.Value)
            {
                return (uint)(input / (k1Divider.Value + 1));
            }
            return (uint)((ulong)input * (feedbackDivider.Value + 1) / ((inputDivider.Value + 1) * (k2Divider.Value + 1)));
        }

        private bool SystemClockFromPll => (RegistersCollection.Read((long)Registers.SystemClockControl) & SystemClockSourcePll) != 0;

        private bool PllLocked => !vcoPowerDown.Value && !pllPowerDown.Value;

        // VADC33V = VDDP / LSB33V + 1
        private ulong SupplyMeasurement => (ulong)Math.Max(0, Math.Min(255, (int)(supplyVoltage / Lsb33V) + 1));

        private const double SupplyVoltageCore13 = 1.3;

        private IValueRegisterField k1Divider;
        private IValueRegisterField feedbackDivider;
        private IValueRegisterField k2Divider;
        private IValueRegisterField inputDivider;
        private IValueRegisterField supplyThreshold;
        private IFlagRegisterField vcoBypass;
        private IFlagRegisterField vcoPowerDown;
        private IFlagRegisterField pllPowerDown;
        private IFlagRegisterField pllInputInternal;
        private IFlagRegisterField oscillatorWatchdogReset;
        private IFlagRegisterField supplyMonitorEnabled;

        // service requests and traps
        private uint serviceRequestRaw;
        private uint serviceRequestMask;
        private uint nmiRequestEnable;
        private uint trapRaw;
        private uint trapDisable;
        private ulong lastNmiRequest;

        // power, hibernate domain and reset cause
        private bool hibernateDomainEnabled;
        private uint usbPowerStatus;
        private uint hibernateStatus;
        private uint hibernateControl;
        private uint internalOscillatorControl;
        private uint ultraLowPowerOscillatorControl;
        private bool hibernating;
        private bool hibernateWakeup;
        private bool hibernateReset;
        private bool lockupResetEnabled;
        private uint resetCause;
        private uint pendingResetCause;
        private bool warmResetPending;
        private bool lastResetWasWarm;
        private double supplyVoltage = 3.3;

        // clocks and peripheral resets
        private uint clockStatus;
        private readonly uint[] peripheralReset = new uint[PeripheralResetRegisterCount];

        private readonly NVIC nvic;
        private readonly uint oscillatorFrequency;

        private const double Lsb33V = 0.0225;
        private const double Lsb13V = 0.0058;
        private const uint InternalOscillatorFrequency = 24000000;
        private const uint StandbyClockFrequency = 32768;
        private const uint SystemClockSourcePll = 1u << 16;
        private const int PeripheralResetRegisterCount = 4;
        private const int SetClearGroupStride = 12;
        private const int NmiException = 2;

        private const uint ResetCausePowerOn = 0x01;
        private const uint ResetCauseWatchdog = 0x20;

        private const uint ClockWatchdog = 1u << 5;
        private const uint ClockMask = 0x3F;

        private const uint HibernateRequest = 1u << 4;
        private const uint HibernateControlResetValue = 0x000C2000;
        private const uint HibernateControlWriteMask = 0x00FF35DF;
        private const uint HibernateStatusMask = 0x0F;
        private const uint HibernateStatusOutput = 1u << 4;

        private const uint ServiceRequestValidMask = 0x3FAE000F;
        private const uint NmiRequestValidMask = 0x000F0007;
        private const uint ServiceRequestNmiMask = 0x00000007;

        private const uint TrapValidMask = 0x1FD;
        private const uint TrapBrownOut = 1u << 5;

        private static readonly uint[] PeripheralResetMasks = { 0x00010F9F, 0x000003F9, 0x000000F6, 0x00000004 };
        private static readonly uint[] PeripheralResetValues = { 0x00010F9F, 0x000001F9, 0x000000F6, 0x00000004 };

        [Flags]
        private enum ServiceRequest : uint
        {
            WatchdogPreWarning = 1u << 0,
            RtcPeriodic = 1u << 1,
            RtcAlarm = 1u << 2,
            DlrOverrun = 1u << 3,
            HdclrUpdated = 1u << 17,
            HdsetUpdated = 1u << 18,
            HdcrUpdated = 1u << 19,
            OscSiCtrlUpdated = 1u << 21,
            OscUlCtrlUpdated = 1u << 23,
        }

        // Bit 0 of the status is the first event; this enum is also the bit order of HDSTAT and the enables in HDCR (WKPEP, WKPEN, RTCE)
        private enum HibernateStatus : uint
        {
            PositiveEdgeEvent = 1u << 0,
            NegativeEdgeEvent = 1u << 1,
            RtcEvent = 1u << 2,
        }

        public enum RtcMirror
        {
            Control = 0,
            AlarmTime0 = 1,
            AlarmTime1 = 2,
            Time0 = 3,
            Time1 = 4,
        }

        private enum Registers : long
        {
            Id = 0x000,
            ChipId = 0x004,
            ManufacturerId = 0x008,
            StartupConfiguration = 0x010,
            GeneralPurpose0 = 0x02C,
            GeneralPurpose1 = 0x030,
            CcuGlobalStart = 0x04C,
            ServiceRequestStatus = 0x074,
            ServiceRequestRaw = 0x078,
            ServiceRequestMask = 0x07C,
            ServiceRequestClear = 0x080,
            ServiceRequestSet = 0x084,
            NmiRequestEnable = 0x088,
            MirrorStatus = 0x0C4,

            ParityErrorEnable = 0x13C,
            MemoryCheckingControl = 0x140,
            ParityErrorTrapEnable = 0x144,
            ParityErrorResetEnable = 0x148,
            ParityErrorFlag = 0x150,
            ParityMemoryTestPattern = 0x154,
            ParityMemoryTestSelect = 0x158,
            TrapStatus = 0x160,
            TrapRaw = 0x164,
            TrapDisable = 0x168,
            TrapClear = 0x16C,
            TrapSet = 0x170,

            PowerStatus = 0x200,
            PowerSet = 0x204,
            PowerClear = 0x208,
            EvrStatus = 0x210,
            EvrVadcStatus = 0x214,
            PowerMonitor = 0x22C,

            HibernateStatus = 0x300,
            HibernateClear = 0x304,
            HibernateSet = 0x308,
            HibernateControl = 0x30C,
            OscillatorInternalControl = 0x314,
            OscillatorUltraLowPowerStatus = 0x318,
            OscillatorUltraLowPowerControl = 0x31C,

            ResetStatus = 0x400,
            ResetSet = 0x404,
            ResetClear = 0x408,
            PeripheralResetStatus0 = 0x40C,

            ClockStatus = 0x600,
            ClockSet = 0x604,
            ClockClear = 0x608,
            SystemClockControl = 0x60C,
            CpuClockControl = 0x610,
            PeripheralBusClockControl = 0x614,
            UsbClockControl = 0x618,
            ExternalBusClockControl = 0x61C,
            CcuClockControl = 0x620,
            WatchdogClockControl = 0x624,
            ExternalClockControl = 0x628,
            SleepControl = 0x630,
            DeepSleepControl = 0x634,

            OscillatorHighPerformanceStatus = 0x700,
            OscillatorHighPerformanceControl = 0x704,
            ClockCalibrationConstant = 0x70C,
            SystemPllStatus = 0x710,
            SystemPllControl0 = 0x714,
            SystemPllControl1 = 0x718,
            SystemPllControl2 = 0x71C,
            UsbPllStatus = 0x720,
            UsbPllControl = 0x724,
            ClockMultiplexingStatus = 0x738,
        }
    }
}
