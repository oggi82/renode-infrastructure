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

namespace Antmicro.Renode.Peripherals.CRC
{
    // Flexible CRC Engine (FCE) of the Infineon XMC4500 with its four CRC kernels, Reference Manual v1.6, chapter 6 (0x50020000).
    //
    //   kernel 0 and 1: IEEE 802.3 CRC32, polynomial 0x04C11DB7, 32 bit input     registers at 0x20 + 0x20 * kernel
    //   kernel 2:       CCITT CRC16,      polynomial 0x1021,     16 bit input
    //   kernel 3:       SAE J1850 CRC8,   polynomial 0x1D,       8 bit input
    //
    // Every write to IR feeds the input word MSB first into the CRC register (CRC); with CFG.REFIN each byte of the word is bit-reversed first.
    // RES is the CRC register, bit-reversed over the kernel width with CFG.REFOUT and then XORed with all ones with CFG.XSEL. With REFIN, REFOUT
    // and XSEL set (CFG reset value 0x700) and the seed 0xFFFFFFFF the CRC32 kernels yield the usual CRC-32 of a byte stream when the bytes are
    // written as big endian words (the XMC library does this with __REV in XMC_FCE_CalculateCRC32Ex). This was checked against the real hardware
    // results shown in the FCE_CRC example and the expected value of the DMA_FCE example.
    //
    // Automatic signature check: with CFG.CCE every write to IR decrements LENGTH; when it reaches 0 RES is compared with CHECK (mismatch: STS.CMF)
    // and LENGTH is reloaded with its configured value if CFG.ALR is set. A write to IR with LENGTH = 0 sets STS.LEF and LENGTH to all ones, which
    // is kept until LENGTH is written again. CHECK and LENGTH can only be written right after a write of 0xFACECAFE (otherwise the write is ignored).
    // CTR.FCM / FRM_CFG / FRM_CHECK (0 to 1) force the mismatch / configuration error flag. Flags are cleared by writing 1 to STS and generate one
    // interrupt pulse (IRQ, NVIC 104) when they are set and enabled in CFG (CMI, CEI, LEI, BEI) while they were clear.
    //
    // Not modeled: bus errors for write accesses narrower than the kernel width (BEF is never set, narrow writes are treated as full width
    // writes), the redundant CFG/CHECK registers (CEF is only set by CTR). Writes to registers other than CLC are ignored (with a warning) while the
    // module is disabled (CLC.DISR, which is set after reset), like the bus error of the hardware.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class XMC4XXX_FCE : IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize
    {
        public XMC4XXX_FCE(IMachine machine)
        {
            RegistersCollection = new DoubleWordRegisterCollection(this);
            IRQ = new GPIO();
            kernels = new Kernel[KernelCount];
            for(var i = 0; i < KernelCount; i++)
            {
                kernels[i] = new Kernel(this, i);
            }
            DefineRegisters();
            Reset();
        }

        public void Reset()
        {
            RegistersCollection.Reset();
            moduleDisabled = true;
            foreach(var kernel in kernels)
            {
                kernel.Reset();
            }
            IRQ.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(moduleDisabled && offset != (long)Registers.ClockControl)
            {
                this.Log(LogLevel.Warning, "Write of 0x{0:X8} to offset 0x{1:X} while the module is disabled (CLC.DISR), the hardware answers with a bus error", value, offset);
                return;
            }
            RegistersCollection.Write(offset, value);
        }

        public long Size => 0x4000;

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public GPIO IRQ { get; }

        private void DefineRegisters()
        {
            Registers.ClockControl.Define(this, 0x3)
                .WithFlag(0, name: "DISR", valueProviderCallback: _ => moduleDisabled, writeCallback: (_, value) => moduleDisabled = value)
                .WithFlag(1, FieldMode.Read, name: "DISS", valueProviderCallback: _ => moduleDisabled)
                .WithReservedBits(2, 30);

            Registers.Id.Define(this, 0x00CAC001)
                .WithValueField(0, 32, FieldMode.Read, name: "ID");

            foreach(var kernel in kernels)
            {
                kernel.DefineRegisters(RegistersCollection);
            }
        }

        private bool moduleDisabled;
        private readonly Kernel[] kernels;

        private const int KernelCount = 4;
        private const uint ConfigMask = 0x73F;

        private class Kernel
        {
            public Kernel(XMC4XXX_FCE fce, int index)
            {
                this.fce = fce;
                this.index = index;
                width = index < 2 ? 32 : index == 2 ? 16 : 8;
                polynomial = index < 2 ? 0x04C11DB7u : index == 2 ? 0x1021u : 0x1Du;
                mask = width == 32 ? 0xFFFFFFFFu : (1u << width) - 1;
            }

            public void Reset()
            {
                crc = 0;
                config = 0x700;
                check = 0;
                length = 0;
                lengthReload = 0;
                status = 0;
                test = 0;
                checkUnlocked = false;
                lengthUnlocked = false;
                lengthHeld = false;
            }

            public void DefineRegisters(DoubleWordRegisterCollection registers)
            {
                var b = KernelBase + KernelStride * index;
                registers.DefineRegister(b + 0x00)
                    .WithValueField(0, 32, name: "IR",
                        valueProviderCallback: _ => 0,
                        writeCallback: (_, value) => Input((uint)value));
                registers.DefineRegister(b + 0x04, mask)
                    .WithValueField(0, 32, FieldMode.Read, name: "RES", valueProviderCallback: _ => Result);
                registers.DefineRegister(b + 0x08, 0x700)
                    .WithValueField(0, 32, name: "CFG",
                        valueProviderCallback: _ => config,
                        writeCallback: (_, value) => config = (uint)value & ConfigMask);
                registers.DefineRegister(b + 0x0C)
                    .WithValueField(0, 32, name: "STS",
                        valueProviderCallback: _ => status,
                        writeCallback: (_, value) => status &= ~(uint)value & 0xF);
                registers.DefineRegister(b + 0x10)
                    .WithValueField(0, 32, name: "LENGTH",
                        valueProviderCallback: _ => length,
                        writeCallback: (_, value) => WriteProtected(value, ref lengthUnlocked, "LENGTH", v =>
                        {
                            length = v & 0xFFFF;
                            lengthReload = length;
                            lengthHeld = false;
                        }));
                registers.DefineRegister(b + 0x14)
                    .WithValueField(0, 32, name: "CHECK",
                        valueProviderCallback: _ => check,
                        writeCallback: (_, value) => WriteProtected(value, ref checkUnlocked, "CHECK", v => check = v & mask));
                registers.DefineRegister(b + 0x18)
                    .WithValueField(0, 32, name: "CRC",
                        valueProviderCallback: _ => crc,
                        writeCallback: (_, value) => crc = (uint)value & mask);
                registers.DefineRegister(b + 0x1C)
                    .WithValueField(0, 32, name: "CTR",
                        valueProviderCallback: _ => test,
                        writeCallback: (_, value) => WriteTest((uint)value & 7));
            }

            // RES: the CRC register after the output reflection and the final XOR
            private uint Result
            {
                get
                {
                    var result = (config & ReflectOutBit) != 0 ? Reflect(crc, width) : crc;
                    return (config & XorSelectBit) != 0 ? (result ^ mask) & mask : result;
                }
            }

            private void Input(uint value)
            {
                var data = value & mask;
                if((config & ReflectInBit) != 0)
                {
                    var reflected = 0u;
                    for(var i = 0; i < width / 8; i++)
                    {
                        reflected |= Reflect((data >> (8 * i)) & 0xFF, 8) << (8 * i);
                    }
                    data = reflected;
                }
                for(var bit = width - 1; bit >= 0; bit--)
                {
                    var feedback = ((crc >> (width - 1)) & 1) ^ ((data >> bit) & 1);
                    crc = (crc << 1) & mask;
                    if(feedback != 0)
                    {
                        crc ^= polynomial;
                    }
                }

                if((config & CheckEnableBit) == 0)
                {
                    return;
                }
                if(length == 0 || lengthHeld)
                {
                    lengthHeld = true;
                    length = 0xFFFF;
                    SetFlag(LengthErrorFlag, LengthErrorInterrupt);
                    return;
                }
                if(--length == 0)
                {
                    if(Result != check)
                    {
                        SetFlag(MismatchFlag, MismatchInterrupt);
                    }
                    if((config & AutoReloadBit) != 0)
                    {
                        length = lengthReload;
                    }
                }
            }

            // CHECK and LENGTH are written after 0xFACECAFE, which itself is not stored
            private void WriteProtected(ulong value, ref bool unlocked, string name, System.Action<uint> store)
            {
                if(value == UnlockKey)
                {
                    unlocked = true;
                    return;
                }
                if(!unlocked)
                {
                    fce.Log(LogLevel.Warning, "Kernel {0}: write to {1} without the 0x{2:X8} key ignored", index, name, UnlockKey);
                    return;
                }
                unlocked = false;
                store((uint)value);
            }

            // A 0 to 1 transition of a CTR bit forces an error flag
            private void WriteTest(uint value)
            {
                var rising = value & ~test;
                test = value;
                if((rising & 1) != 0)
                {
                    SetFlag(MismatchFlag, MismatchInterrupt);
                }
                if((rising & 6) != 0)
                {
                    SetFlag(ConfigurationErrorFlag, ConfigurationErrorInterrupt);
                }
            }

            // Only a flag that was clear generates an interrupt, and only when enabled in CFG
            private void SetFlag(uint flag, uint interruptEnable)
            {
                var wasSet = (status & flag) != 0;
                status |= flag;
                if(!wasSet && (config & interruptEnable) != 0)
                {
                    fce.IRQ.Blink();
                }
            }

            private static uint Reflect(uint value, int bits)
            {
                var result = 0u;
                for(var i = 0; i < bits; i++)
                {
                    if((value & (1u << i)) != 0)
                    {
                        result |= 1u << (bits - 1 - i);
                    }
                }
                return result;
            }

            private readonly XMC4XXX_FCE fce;
            private readonly int index;
            private readonly int width;
            private readonly uint polynomial;
            private readonly uint mask;

            private uint crc;
            private uint config;
            private uint check;
            private uint length;
            private uint lengthReload;
            private uint status;
            private uint test;
            private bool checkUnlocked;
            private bool lengthUnlocked;
            private bool lengthHeld;

            private const long KernelBase = 0x20;
            private const long KernelStride = 0x20;
            private const uint UnlockKey = 0xFACECAFE;
            private const uint MismatchInterrupt = 1 << 0;
            private const uint ConfigurationErrorInterrupt = 1 << 1;
            private const uint LengthErrorInterrupt = 1 << 2;
            private const uint CheckEnableBit = 1 << 4;
            private const uint AutoReloadBit = 1 << 5;
            private const uint ReflectInBit = 1 << 8;
            private const uint ReflectOutBit = 1 << 9;
            private const uint XorSelectBit = 1 << 10;
            private const uint MismatchFlag = 1 << 0;
            private const uint ConfigurationErrorFlag = 1 << 1;
            private const uint LengthErrorFlag = 1 << 2;
        }

        private enum Registers : long
        {
            ClockControl = 0x00,
            Id = 0x08,
        }
    }
}
