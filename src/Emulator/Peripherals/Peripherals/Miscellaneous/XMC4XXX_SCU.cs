//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.IRQControllers;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // System Control Unit of the Infineon XMC4xxx family.
    // The model covers what is needed to run the XMC Peripheral Library clock setup:
    // power/reset/clock-gating status-set-clear registers, hibernate domain handshake,
    // the high performance oscillator and the system PLL. Status bits that the hardware
    // updates asynchronously (PLL lock, oscillator watchdog, hibernate mirror update) are
    // reflected immediately. Interrupts, parity, trap and the remaining SCU blocks are not modeled.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class XMC4XXX_SCU : BasicDoubleWordPeripheral, IKnownSize
    {
        public XMC4XXX_SCU(IMachine machine, NVIC nvic = null, uint oscillatorFrequency = 12000000) : base(machine)
        {
            this.nvic = nvic;
            this.oscillatorFrequency = oscillatorFrequency;
            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            UpdateCpuClock();
        }

        public long Size => 0x1000;

        private void DefineRegisters()
        {
            // The hibernate domain is written through a mirror; updates are applied instantly here.
            Registers.MirrorStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => 0, name: "MIRRSTS");

            DefineStatusSetClear(Registers.PowerStatus, "PWR");
            DefineStatusSetClear(Registers.HibernateStatus, "HD");
            Registers.HibernateControl.Define(this)
                .WithValueField(0, 32, name: "HDCR");
            Registers.OscillatorUltraLowPowerControl.Define(this)
                .WithValueField(0, 32, name: "OSCULCTRL");

            DefineStatusSetClear(Registers.ResetStatus, "RST");
            for(var i = 0; i < PeripheralResetRegisterCount; i++)
            {
                DefineStatusSetClear(Registers.PeripheralResetStatus0 + i * SetClearGroupStride, $"PR{i}");
            }

            DefineStatusSetClear(Registers.ClockStatus, "CLK");
            for(var offset = Registers.SystemClockControl; offset <= Registers.DeepSleepControl; offset += 4)
            {
                offset.Define(this)
                    .WithValueField(0, 32, name: $"CLKCON_{(long)offset:X3}", changeCallback: (_, __) => UpdateCpuClock());
            }
            for(var i = 0; i < ClockGatingRegisterCount; i++)
            {
                DefineStatusSetClear(Registers.ClockGatingStatus0 + i * SetClearGroupStride, $"CGAT{i}");
            }

            Registers.OscillatorHighPerformanceStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "OSCHPSTAT");
            Registers.OscillatorHighPerformanceControl.Define(this)
                .WithValueField(0, 32, name: "OSCHPCTRL");
            Registers.ClockCalibrationConstant.Define(this)
                .WithValueField(0, 32, name: "CLKCALCONST");

            DefineSystemPll();

            Registers.UsbPllStatus.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "USBPLLSTAT");
            Registers.UsbPllControl.Define(this, 0x00030003)
                .WithValueField(0, 32, name: "USBPLLCON");
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
                .WithFlag(18, name: "RESLD")
                .WithFlag(19, name: "AOTREN")
                .WithFlag(20, name: "FOTR")
                .WithReservedBits(21, 11);

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

            Registers.SystemPllControl2.Define(this)
                .WithValueField(0, 32, name: "PLLCON2", changeCallback: (_, __) => UpdateCpuClock());

            Registers.SystemPllStatus.Define(this)
                .WithFlag(0, FieldMode.Read, name: "VCOBYST", valueProviderCallback: _ => vcoBypass.Value)
                .WithFlag(1, FieldMode.Read, name: "PWDSTAT", valueProviderCallback: _ => pllPowerDown.Value)
                .WithFlag(2, FieldMode.Read, name: "VCOLOCK", valueProviderCallback: _ => PllLocked)
                .WithReservedBits(3, 2)
                .WithFlag(5, FieldMode.Read, name: "K1RDY", valueProviderCallback: _ => PllLocked)
                .WithFlag(6, FieldMode.Read, name: "K2RDY", valueProviderCallback: _ => PllLocked)
                // All three bits are set when the oscillator watchdog considers the high performance oscillator usable.
                .WithValueField(7, 3, FieldMode.Read, name: "OSC_USABLE", valueProviderCallback: _ => oscillatorWatchdogReset.Value ? 0u : 0x7u)
                .WithReservedBits(10, 22);
        }

        // Defines the STAT/SET/CLR register triple used throughout the SCU: writing 1 to SET sets, writing 1 to CLR clears the status bit.
        private void DefineStatusSetClear(Registers statusRegister, string name)
        {
            var status = statusRegister.Define(this, name: $"{name}STAT")
                .WithValueField(0, 32, FieldMode.Read, name: "STATUS");

            (statusRegister + 4).Define(this, name: $"{name}SET")
                .WithValueField(0, 32, FieldMode.Write, name: "SET", writeCallback: (_, value) => status.Value |= (uint)value);

            (statusRegister + 8).Define(this, name: $"{name}CLR")
                .WithValueField(0, 32, FieldMode.Write, name: "CLR", writeCallback: (_, value) => status.Value &= ~(uint)value);
        }

        // Keeps the SysTick of the NVIC running at the CPU clock: fCPU = fSYS / (CPUDIV + 1), fSYS = (OFI or PLL) / (SYSDIV + 1)
        private void UpdateCpuClock()
        {
            if(nvic == null)
            {
                return;
            }

            var systemClockControl = RegistersCollection.Read((long)Registers.SystemClockControl);
            var cpuClockControl = RegistersCollection.Read((long)Registers.CpuClockControl);
            var frequency = (systemClockControl & SystemClockSourcePll) != 0 ? GetPllFrequency() : InternalOscillatorFrequency;
            frequency /= (systemClockControl & 0xFF) + 1;
            frequency /= (cpuClockControl & 1) + 1;
            nvic.Frequency = frequency;
        }

        // Same formulas as SystemCoreClockUpdate of the XMC library: fPLL = fIN * (NDIV + 1) / ((PDIV + 1) * (K2DIV + 1)) when locked, fIN / (K1DIV + 1) otherwise
        private uint GetPllFrequency()
        {
            var input = (RegistersCollection.Read((long)Registers.SystemPllControl2) & 1) != 0 ? InternalOscillatorFrequency : oscillatorFrequency;
            if(PllLocked)
            {
                return (uint)((ulong)input * (feedbackDivider.Value + 1) / ((inputDivider.Value + 1) * (k2Divider.Value + 1)));
            }
            return (uint)(input / (k1Divider.Value + 1));
        }

        private bool PllLocked => !vcoPowerDown.Value && !pllPowerDown.Value;

        private IValueRegisterField k1Divider;
        private IValueRegisterField feedbackDivider;
        private IValueRegisterField k2Divider;
        private IValueRegisterField inputDivider;
        private IFlagRegisterField vcoBypass;
        private IFlagRegisterField vcoPowerDown;
        private IFlagRegisterField pllPowerDown;
        private IFlagRegisterField oscillatorWatchdogReset;

        private readonly NVIC nvic;
        private readonly uint oscillatorFrequency;

        private const uint InternalOscillatorFrequency = 24000000;
        private const uint SystemClockSourcePll = 1u << 16;
        private const int PeripheralResetRegisterCount = 4;
        private const int ClockGatingRegisterCount = 3;
        private const int SetClearGroupStride = 12;

        private enum Registers : long
        {
            MirrorStatus = 0x0C4,

            PowerStatus = 0x200,
            PowerSet = 0x204,
            PowerClear = 0x208,

            HibernateStatus = 0x300,
            HibernateSet = 0x304,
            HibernateClear = 0x308,
            HibernateControl = 0x30C,
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
            PeripheralClockControl = 0x614,
            UsbClockControl = 0x618,
            ExternalBusClockControl = 0x61C,
            CcuClockControl = 0x620,
            WatchdogClockControl = 0x624,
            ExternalClockControl = 0x628,
            SleepControl = 0x62C,
            DeepSleepControl = 0x630,
            ClockGatingStatus0 = 0x640,

            OscillatorHighPerformanceStatus = 0x700,
            OscillatorHighPerformanceControl = 0x704,
            ClockCalibrationConstant = 0x708,
            SystemPllStatus = 0x710,
            SystemPllControl0 = 0x714,
            SystemPllControl1 = 0x718,
            SystemPllControl2 = 0x71C,
            UsbPllStatus = 0x720,
            UsbPllControl = 0x724,
        }
    }
}
