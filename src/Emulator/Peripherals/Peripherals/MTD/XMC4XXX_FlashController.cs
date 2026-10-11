//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Memory;

namespace Antmicro.Renode.Peripherals.MTD
{
    // Flash module FLASH0 of the Infineon XMC4500 (Reference Manual v1.6, chapter 8 "Flash and Program Memory Unit (PMU)").
    //
    // The controller owns the command interpreter. The CPU does not write to the program flash directly: every write
    // to the non-cached flash segment (0x0C000000, see XMC4XXX_FlashArray) is a command cycle, and only complete
    // command sequences (Table 8-4) change the flash content:
    //   - Erase Sector / Erase Physical Sector set all bits of the sector to 0 (the erased state of this flash is '0'),
    //   - Write Page programs a 256 byte page from the assembly buffer; programming can only turn bits to 1, so the
    //     page is OR-ed into the old content (writing a page again without erasing cannot clear bits).
    // Incorrect sequences set FSR.SQER and leave the content untouched, writes to protected sectors set FSR.PROER.
    //
    // Simplifications: operations complete instantly (FSR.PBUSY is never observed as 1, the end-of-busy interrupt is
    // raised at once), ECC/margin/sleep functions are only register storage, read protection does not block reads.
    // The user configuration blocks (UCB0-2) are modelled as non-volatile storage whose protection information is
    // evaluated at (re)set, like the hardware does during ramp up. The model is not volatile across a machine reset.
    public sealed class XMC4XXX_FlashController : BasicDoubleWordPeripheral, IKnownSize
    {
        public XMC4XXX_FlashController(IMachine machine, MappedMemory flash) : base(machine)
        {
            this.flash = flash;
            IRQ = new GPIO();
            for(var i = 0; i < UcbCount; i++)
            {
                ucb[i] = new byte[UcbSize];
            }
            DefineRegisters();
            ResetState();
        }

        public override void Reset()
        {
            base.Reset();
            lock(locker)
            {
                ResetState();
            }
            IRQ.Unset();
        }

        // Called by XMC4XXX_FlashArray for every write to the non-cached flash segment. 'offset' is relative to the
        // start of the flash, 'width' is the access size in bytes.
        public void WriteCommandCycle(long offset, uint value, int width)
        {
            var raised = false;
            lock(locker)
            {
                raised = ProcessCommandCycle((uint)offset, value, width);
            }
            if(raised)
            {
                IRQ.Set();
                IRQ.Unset();
            }
        }

        public long Size => 0x400;

        public GPIO IRQ { get; }

        private void DefineRegisters()
        {
            Registers.ModuleIdentification.Define(this, 0x00A2C001)
                .WithValueField(0, 32, FieldMode.Read, name: "ID (MOD_REV, MOD_TYPE, MOD_NUMBER)");

            Registers.Status.Define(this)
                .WithFlag(0, FieldMode.Read, name: "PBUSY", valueProviderCallback: _ => false)
                .WithFlag(1, FieldMode.Read, name: "FABUSY", valueProviderCallback: _ => false)
                .WithReservedBits(2, 2)
                .WithFlag(4, FieldMode.Read, name: "PROG", valueProviderCallback: _ => progFlag)
                .WithFlag(5, FieldMode.Read, name: "ERASE", valueProviderCallback: _ => eraseFlag)
                .WithFlag(6, FieldMode.Read, name: "PFPAGE", valueProviderCallback: _ => pageMode)
                .WithReservedBits(7, 1)
                .WithFlag(8, FieldMode.Read, name: "PFOPER", valueProviderCallback: _ => false)
                .WithReservedBits(9, 1)
                .WithFlag(10, FieldMode.Read, name: "SQER", valueProviderCallback: _ => sequenceError)
                .WithFlag(11, FieldMode.Read, name: "PROER", valueProviderCallback: _ => protectionError)
                .WithFlag(12, FieldMode.Read, name: "PFSBER", valueProviderCallback: _ => false)
                .WithReservedBits(13, 1)
                .WithFlag(14, FieldMode.Read, name: "PFDBER", valueProviderCallback: _ => false)
                .WithReservedBits(15, 1)
                .WithFlag(16, FieldMode.Read, name: "PROIN", valueProviderCallback: _ => ucbConfirmed[0] || ucbConfirmed[1] || ucbConfirmed[2])
                .WithReservedBits(17, 1)
                .WithFlag(18, FieldMode.Read, name: "RPROIN", valueProviderCallback: _ => (procon[0] & ReadProtectionBit) != 0 && ucbConfirmed[0])
                .WithFlag(19, FieldMode.Read, name: "RPRODIS", valueProviderCallback: _ => readProtectionDisabled)
                .WithReservedBits(20, 1)
                .WithFlag(21, FieldMode.Read, name: "WPROIN0", valueProviderCallback: _ => ucbConfirmed[0])
                .WithFlag(22, FieldMode.Read, name: "WPROIN1", valueProviderCallback: _ => ucbConfirmed[1])
                .WithFlag(23, FieldMode.Read, name: "WPROIN2", valueProviderCallback: _ => ucbConfirmed[2])
                .WithReservedBits(24, 1)
                .WithFlag(25, FieldMode.Read, name: "WPRODIS0", valueProviderCallback: _ => writeProtectionDisabled[0])
                .WithFlag(26, FieldMode.Read, name: "WPRODIS1", valueProviderCallback: _ => writeProtectionDisabled[1])
                .WithReservedBits(27, 1)
                .WithFlag(28, FieldMode.Read, name: "SLM", valueProviderCallback: _ => false)
                .WithReservedBits(29, 2)
                .WithFlag(31, FieldMode.Read, name: "VER", valueProviderCallback: _ => false);

            Registers.Configuration.Define(this, 0x6)
                .WithValueField(0, 4, out waitStates, name: "WSPFLASH")
                .WithFlag(4, name: "WSECPF")
                .WithReservedBits(5, 8)
                .WithFlag(13, name: "IDLE")
                .WithFlag(14, name: "ESLDIS")
                .WithFlag(15, name: "SLEEP")
                .WithFlag(16, FieldMode.Read, name: "RPA", valueProviderCallback: _ => ReadProtectionActive)
                // DCF and DDF can always be set, but be cleared only while the read protection is not active.
                .WithFlag(17, name: "DCF",
                    valueProviderCallback: _ => disableCodeFetch,
                    writeCallback: (_, value) => { if(value || !ReadProtectionActive) disableCodeFetch = value; })
                .WithFlag(18, name: "DDF",
                    valueProviderCallback: _ => disableDataFetch,
                    writeCallback: (_, value) => { if(value || !ReadProtectionActive) disableDataFetch = value; })
                .WithReservedBits(19, 5)
                .WithFlag(24, out verifyOperationErrorMask, name: "VOPERM")
                .WithFlag(25, out sequenceErrorMask, name: "SQERM")
                .WithFlag(26, out protectionErrorMask, name: "PROERM")
                .WithFlag(27, name: "PFSBERM")
                .WithReservedBits(28, 1)
                .WithFlag(29, name: "PFDBERM")
                .WithReservedBits(30, 1)
                .WithFlag(31, out endOfBusyMask, name: "EOBM");

            Registers.MarginControl.Define(this)
                .WithValueField(0, 4, name: "MARGIN")
                .WithReservedBits(4, 11)
                .WithFlag(15, name: "TRAPDIS")
                .WithReservedBits(16, 16);

            for(var user = 0; user < UcbCount; user++)
            {
                var index = user;
                (Registers.ProtectionConfiguration0 + 4 * user).Define(this, name: $"PROCON{user}")
                    .WithValueField(0, 16, FieldMode.Read, name: $"PROCON{user}", valueProviderCallback: _ => procon[index])
                    .WithReservedBits(16, 16);
            }
        }

        private void ResetState()
        {
            pageMode = false;
            progFlag = false;
            eraseFlag = false;
            sequenceError = false;
            protectionError = false;
            readProtectionDisabled = false;
            writeProtectionDisabled = new bool[UcbCount];
            passwordLockout = false;
            disableCodeFetch = false;
            disableDataFetch = false;
            state = CommandState.Idle;
            assemblyPointer = 0;
            LoadProtectionFromUcb();
        }

        // The PMU evaluates the UCBs only during start-up (ramp up).
        private void LoadProtectionFromUcb()
        {
            for(var user = 0; user < UcbCount; user++)
            {
                procon[user] = (uint)(ucb[user][0] | (ucb[user][1] << 8)) & ProtectionMask;
                ucbConfirmed[user] = BitConverter.ToUInt32(ucb[user], ConfirmationOffset) == ConfirmationCode;
            }
        }

        private bool ReadProtectionActive => (procon[0] & ReadProtectionBit) != 0 && !readProtectionDisabled;

        // Returns true if an interrupt request has to be generated.
        private bool ProcessCommandCycle(uint offset, uint value, int width)
        {
            var address = offset & 0xFFFF;
            var data = value & 0xFF;
            var wide = width == 4;
            var interrupt = false;

            // Reset to Read is accepted in every state; it aborts a running sequence and page mode.
            if(address == AddressFirst && data == ResetToReadData && state != CommandState.Idle)
            {
                state = CommandState.Idle;
                pageMode = false;
                ClearErrorFlags();
                return false;
            }

            switch(state)
            {
            case CommandState.Idle:
                if(address == AddressFirst && data == ResetToReadData)
                {
                    pageMode = false;
                    ClearErrorFlags();
                }
                else if(address == AddressFirst && (data & 0xF0) == EnterPageModeData && (data & 0x0F) == 0)
                {
                    if(pageMode)
                    {
                        // A repeated Enter Page Mode is executed, but flagged.
                        interrupt |= RaiseSequenceError("Enter Page Mode while in page mode", keepPageMode: true);
                    }
                    pageMode = true;
                    assemblyPointer = 0;
                }
                else if(address == AddressLoadLow)
                {
                    interrupt |= LoadPageLow(value, wide);
                }
                else if(address == AddressFirst && data == ResumeProtectionData)
                {
                    ResumeProtection();
                }
                else if(address == AddressFirst && data == ClearStatusData)
                {
                    progFlag = false;
                    eraseFlag = false;
                    ClearErrorFlags();
                }
                else if(address == AddressFirst && data == 0xAA)
                {
                    state = CommandState.Unlock1;
                }
                else
                {
                    interrupt |= RaiseSequenceError($"invalid first command cycle at 0x{offset:X}, data 0x{value:X}");
                }
                break;

            case CommandState.LoadHigh:
                if(address == AddressLoadHigh && wide)
                {
                    if(!discardLoad)
                    {
                        Array.Copy(BitConverter.GetBytes(pendingLow), 0, assemblyBuffer, assemblyPointer * 8, 4);
                        Array.Copy(BitConverter.GetBytes(value), 0, assemblyBuffer, assemblyPointer * 8 + 4, 4);
                        assemblyPointer++;
                    }
                    state = CommandState.Idle;
                }
                else
                {
                    interrupt |= RaiseSequenceError("Load Page: high word expected at 0x55F4");
                }
                break;

            case CommandState.Unlock1:
                if(address == AddressSecond && data == 0x55)
                {
                    state = CommandState.Unlock2;
                }
                else
                {
                    interrupt |= RaiseSequenceError("second command cycle must be 0x55 at 0xAAA8");
                }
                break;

            case CommandState.Unlock2:
                if(address == AddressFirst && data == WritePageData)
                {
                    state = CommandState.WritePageAddress;
                }
                else if(address == AddressFirst && data == WriteUcbPageData)
                {
                    state = CommandState.WriteUcbPageAddress;
                }
                else if(address == AddressFirst && data == EraseData)
                {
                    state = CommandState.Erase1;
                }
                else if(address == AddressProtectedFirst)
                {
                    protectedUser = data;
                    state = CommandState.Password0;
                }
                else
                {
                    interrupt |= RaiseSequenceError("invalid third command cycle");
                }
                break;

            case CommandState.WritePageAddress:
                state = CommandState.Idle;
                if(data == 0xAA)
                {
                    interrupt |= WritePage(offset);
                }
                else
                {
                    interrupt |= RaiseSequenceError("Write Page: last cycle must be 0xAA");
                }
                break;

            case CommandState.WriteUcbPageAddress:
                state = CommandState.Idle;
                if(data == 0xAA)
                {
                    interrupt |= WriteUcbPage(offset);
                }
                else
                {
                    interrupt |= RaiseSequenceError("Write User Configuration Page: last cycle must be 0xAA");
                }
                break;

            case CommandState.Erase1:
                if(address == AddressFirst && data == 0xAA)
                {
                    state = CommandState.Erase2;
                }
                else
                {
                    interrupt |= RaiseSequenceError("erase: fourth command cycle must be 0xAA at 0x5554");
                }
                break;

            case CommandState.Erase2:
                if(address == AddressSecond && data == 0x55)
                {
                    state = CommandState.EraseTarget;
                }
                else
                {
                    interrupt |= RaiseSequenceError("erase: fifth command cycle must be 0x55 at 0xAAA8");
                }
                break;

            case CommandState.EraseTarget:
                state = CommandState.Idle;
                switch(data)
                {
                case EraseSectorData:
                    interrupt |= EraseSector(offset, physical: false);
                    break;
                case ErasePhysicalSectorData:
                    interrupt |= EraseSector(offset, physical: true);
                    break;
                case EraseUcbData:
                    interrupt |= EraseUcb(offset);
                    break;
                default:
                    interrupt |= RaiseSequenceError($"unknown erase command 0x{data:X2}");
                    break;
                }
                break;

            case CommandState.Password0:
                if(address == AddressSecond && wide)
                {
                    password0 = value;
                    state = CommandState.Password1;
                }
                else
                {
                    interrupt |= RaiseSequenceError("protected command: password 0 expected as 32 bit write at 0xAAA8");
                }
                break;

            case CommandState.Password1:
                if(address == AddressSecond && wide)
                {
                    password1 = value;
                    state = CommandState.ProtectedFinal;
                }
                else
                {
                    interrupt |= RaiseSequenceError("protected command: password 1 expected as 32 bit write at 0xAAA8");
                }
                break;

            case CommandState.ProtectedFinal:
                state = CommandState.Idle;
                if(address == AddressProtectedLast && data == DisableSectorProtectionData && protectedUser <= 1)
                {
                    interrupt |= DisableWriteProtection((int)protectedUser);
                }
                else if(address == AddressProtectedLast && data == DisableReadProtectionData && protectedUser == 0)
                {
                    interrupt |= DisableReadProtection();
                }
                else
                {
                    interrupt |= RaiseSequenceError("protected command: invalid last command cycle");
                }
                break;
            }
            return interrupt;
        }

        private bool LoadPageLow(uint value, bool wide)
        {
            if(!pageMode)
            {
                return RaiseSequenceError("Load Page while not in page mode");
            }
            if(!wide)
            {
                return RaiseSequenceError("Load Page needs 32 bit transfers");
            }
            pendingLow = value;
            discardLoad = assemblyPointer >= AssemblyWords / 2;
            state = CommandState.LoadHigh;
            // The overflow is reported with the first cycle of the superfluous Load Page.
            return discardLoad && RaiseSequenceError("Load Page: assembly buffer overflow", keepPageMode: true);
        }

        private bool WritePage(uint pageAddress)
        {
            if(!pageMode)
            {
                return RaiseSequenceError("Write Page while not in page mode");
            }
            if(pageAddress % PageSize != 0 || pageAddress >= FlashSize)
            {
                return RaiseSequenceError($"Write Page: 0x{pageAddress:X} is not a valid page address");
            }
            var sector = FindSector(pageAddress);
            if(sector < 0)
            {
                return RaiseSequenceError($"Write Page: no flash at 0x{pageAddress:X}");
            }

            var interrupt = false;
            pageMode = false;
            if(assemblyPointer < AssemblyWords / 2)
            {
                // Buffer underflow: the page is programmed with the previous buffer content, but SQER is reported.
                interrupt |= RaiseSequenceError("Write Page: too few words loaded", keepPageMode: false);
            }
            if(IsSectorLocked(sector))
            {
                return interrupt | RaiseProtectionError($"Write Page: sector {sector} is write protected");
            }

            // Programming sets bits to 1, it cannot clear them.
            var current = flash.ReadBytes(pageAddress, PageSize);
            for(var i = 0; i < PageSize; i++)
            {
                current[i] |= assemblyBuffer[i];
            }
            flash.WriteBytes(pageAddress, current, 0, PageSize);
            progFlag = true;
            return interrupt | EndOfBusy();
        }

        private bool EraseSector(uint address, bool physical)
        {
            var index = Array.IndexOf(SectorStart, address);
            if(physical)
            {
                index = Array.IndexOf(PhysicalSectorStart, address);
            }
            if(index < 0)
            {
                return RaiseSequenceError($"Erase {(physical ? "Physical " : "")}Sector: 0x{address:X} is not a sector start address");
            }

            var start = address;
            var size = physical ? PhysicalSectorSize[index] : SectorSize[index];
            for(var sector = FindSector(start); sector >= 0 && sector < SectorStart.Length && SectorStart[sector] < start + size; sector++)
            {
                if(IsSectorLocked(sector))
                {
                    return RaiseProtectionError($"Erase Sector: sector {sector} is write protected");
                }
            }
            flash.WriteBytes(start, new byte[size], 0, (int)size);
            eraseFlag = true;
            return EndOfBusy();
        }

        private bool WriteUcbPage(uint address)
        {
            if(!pageMode)
            {
                return RaiseSequenceError("Write User Configuration Page while not in page mode");
            }
            pageMode = false;
            if(address % PageSize != 0 || address >= UcbCount * UcbSize)
            {
                return RaiseSequenceError($"Write User Configuration Page: 0x{address:X} is not a UCB page address");
            }
            var user = (int)(address / UcbSize);
            if(IsUcbLocked(user))
            {
                return RaiseProtectionError($"Write User Configuration Page: UCB{user} is write protected");
            }
            var offset = (int)(address % UcbSize);
            for(var i = 0; i < PageSize; i++)
            {
                ucb[user][offset + i] |= assemblyBuffer[i];
            }
            progFlag = true;
            return EndOfBusy();
        }

        private bool EraseUcb(uint address)
        {
            if(address % UcbSize != 0 || address >= UcbCount * UcbSize)
            {
                return RaiseSequenceError($"Erase User Configuration Block: 0x{address:X} is not a UCB start address");
            }
            var user = (int)(address / UcbSize);
            if(IsUcbLocked(user))
            {
                return RaiseProtectionError($"Erase User Configuration Block: UCB{user} is write protected");
            }
            Array.Clear(ucb[user], 0, UcbSize);
            eraseFlag = true;
            return EndOfBusy();
        }

        private bool DisableWriteProtection(int user)
        {
            if(!PasswordMatches(user))
            {
                return RaiseProtectionError($"Disable Sector Write Protection: wrong password for user {user}");
            }
            writeProtectionDisabled[user] = true;
            return false;
        }

        private bool DisableReadProtection()
        {
            if(!PasswordMatches(0))
            {
                return RaiseProtectionError("Disable Read Protection: wrong password");
            }
            readProtectionDisabled = true;
            return false;
        }

        // A failed password check blocks all further password commands until the next reset.
        private bool PasswordMatches(int user)
        {
            if(passwordLockout)
            {
                return false;
            }
            var stored0 = BitConverter.ToUInt32(ucb[user], PasswordOffset);
            var stored1 = BitConverter.ToUInt32(ucb[user], PasswordOffset + 4);
            if(stored0 != password0 || stored1 != password1)
            {
                passwordLockout = true;
                return false;
            }
            return true;
        }

        private void ResumeProtection()
        {
            readProtectionDisabled = false;
            for(var user = 0; user < 2; user++)
            {
                // The disabled state of an UCB that is not confirmed is not cleared.
                if(ucbConfirmed[user])
                {
                    writeProtectionDisabled[user] = false;
                }
            }
        }

        private bool IsSectorLocked(int sector)
        {
            if(ReadProtectionActive)
            {
                return true;
            }
            var bit = 1u << Math.Min(sector, 10);
            var user0 = (procon[0] & bit) != 0;
            var user1 = (procon[1] & bit) != 0;
            var user2 = (procon[2] & bit) != 0;
            return user2
                || (user0 && !writeProtectionDisabled[0])
                || (user1 && !user0 && !writeProtectionDisabled[1]);
        }

        private bool IsUcbLocked(int user)
        {
            if(ReadProtectionActive)
            {
                return true;
            }
            if(!ucbConfirmed[user])
            {
                return false;
            }
            switch(user)
            {
            case 0:
                return !writeProtectionDisabled[0];
            case 1:
                return !writeProtectionDisabled[0] && !writeProtectionDisabled[1];
            default:
                return true;
            }
        }

        private static int FindSector(uint address)
        {
            for(var i = SectorStart.Length - 1; i >= 0; i--)
            {
                if(address >= SectorStart[i])
                {
                    return address < SectorStart[i] + SectorSize[i] ? i : -1;
                }
            }
            return -1;
        }

        private void ClearErrorFlags()
        {
            sequenceError = false;
            protectionError = false;
        }

        private bool EndOfBusy()
        {
            return endOfBusyMask.Value;
        }

        private bool RaiseSequenceError(string reason, bool keepPageMode = false)
        {
            this.Log(LogLevel.Warning, "Flash sequence error: {0}", reason);
            sequenceError = true;
            state = CommandState.Idle;
            if(!keepPageMode)
            {
                pageMode = false;
            }
            return sequenceErrorMask.Value;
        }

        private bool RaiseProtectionError(string reason)
        {
            this.Log(LogLevel.Warning, "Flash protection error: {0}", reason);
            protectionError = true;
            state = CommandState.Idle;
            return protectionErrorMask.Value;
        }

        private readonly MappedMemory flash;
        private readonly object locker = new object();
        private readonly byte[][] ucb = new byte[UcbCount][];
        private readonly bool[] ucbConfirmed = new bool[UcbCount];
        private readonly uint[] procon = new uint[UcbCount];
        private readonly byte[] assemblyBuffer = new byte[PageSize];

        private IValueRegisterField waitStates;
        private IFlagRegisterField verifyOperationErrorMask;
        private IFlagRegisterField sequenceErrorMask;
        private IFlagRegisterField protectionErrorMask;
        private IFlagRegisterField endOfBusyMask;

        private CommandState state;
        private bool pageMode;
        private bool progFlag;
        private bool eraseFlag;
        private bool sequenceError;
        private bool protectionError;
        private bool readProtectionDisabled;
        private bool[] writeProtectionDisabled = new bool[UcbCount];
        private bool passwordLockout;
        private bool disableCodeFetch;
        private bool disableDataFetch;
        private int assemblyPointer;
        private uint pendingLow;
        private bool discardLoad;
        private uint protectedUser;
        private uint password0;
        private uint password1;

        private const int PageSize = 256;
        private const int AssemblyWords = PageSize / 4;
        private const uint FlashSize = 0x100000;
        private const int UcbCount = 3;
        private const int UcbSize = 0x400;
        private const int PasswordOffset = 16;
        private const int ConfirmationOffset = 0x200;
        private const uint ConfirmationCode = 0x8AFE15C3;
        private const uint ProtectionMask = 0x87FF;
        private const uint ReadProtectionBit = 0x8000;

        // Command cycle addresses (lower 16 bits) and data (Table 8-4).
        private const uint AddressFirst = 0x5554;
        private const uint AddressSecond = 0xAAA8;
        private const uint AddressLoadLow = 0x55F0;
        private const uint AddressLoadHigh = 0x55F4;
        private const uint AddressProtectedFirst = 0x553C;
        private const uint AddressProtectedLast = 0x5558;
        private const uint ResetToReadData = 0xF0;
        private const uint EnterPageModeData = 0x50;
        private const uint ResumeProtectionData = 0x5E;
        private const uint ClearStatusData = 0xF5;
        private const uint WritePageData = 0xA0;
        private const uint WriteUcbPageData = 0xC0;
        private const uint EraseData = 0x80;
        private const uint EraseSectorData = 0x30;
        private const uint ErasePhysicalSectorData = 0x40;
        private const uint EraseUcbData = 0xC0;
        private const uint DisableSectorProtectionData = 0x05;
        private const uint DisableReadProtectionData = 0x08;

        // Table 8-2: sectors S0-S11 of the 1 MB program flash and their physical sectors PS0, PS4, S8-S11.
        private static readonly uint[] SectorStart = { 0x0, 0x4000, 0x8000, 0xC000, 0x10000, 0x14000, 0x18000, 0x1C000, 0x20000, 0x40000, 0x80000, 0xC0000 };
        private static readonly uint[] SectorSize = { 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x4000, 0x20000, 0x40000, 0x40000, 0x40000 };
        private static readonly uint[] PhysicalSectorStart = { 0x0, 0x10000, 0x20000, 0x40000, 0x80000, 0xC0000 };
        private static readonly uint[] PhysicalSectorSize = { 0x10000, 0x10000, 0x20000, 0x40000, 0x40000, 0x40000 };

        private enum CommandState
        {
            Idle,
            LoadHigh,
            Unlock1,
            Unlock2,
            WritePageAddress,
            WriteUcbPageAddress,
            Erase1,
            Erase2,
            EraseTarget,
            Password0,
            Password1,
            ProtectedFinal,
        }

        private enum Registers : long
        {
            ModuleIdentification = 0x008, // FLASH0_ID
            Status = 0x010,               // FLASH0_FSR
            Configuration = 0x014,        // FLASH0_FCON
            MarginControl = 0x018,        // FLASH0_MARP
            ProtectionConfiguration0 = 0x020, // FLASH0_PROCON0, PROCON1 at 0x24, PROCON2 at 0x28
        }
    }
}
