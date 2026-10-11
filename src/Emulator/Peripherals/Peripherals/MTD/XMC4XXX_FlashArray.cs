//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Memory;
using Antmicro.Renode.Utilities;

using Endianess = ELFSharp.ELF.Endianess;

namespace Antmicro.Renode.Peripherals.MTD
{
    // Non-cached view (0x0C000000) of the XMC4500 program flash.
    // Reads return the flash content. Every write of the CPU is a command cycle for XMC4XXX_FlashController and does
    // not change the content by itself. Writes by the loader (LoadELF, sysbus WriteBytes, ...) are multi-byte
    // writes and go straight to the storage, so firmware images can still be loaded.
    // The same storage is mapped at the cached address 0x08000000 for fast code execution; writes to this alias are
    // not intercepted and change the content directly.
    public sealed class XMC4XXX_FlashArray : IMemory, ICanLoadFiles, IEndiannessAware
    {
        public XMC4XXX_FlashArray(MappedMemory flash, XMC4XXX_FlashController controller)
        {
            this.flash = flash;
            this.controller = controller;
        }

        public void Reset()
        {
            // The content of the flash is non-volatile.
        }

        public byte ReadByte(long offset)
        {
            return flash.ReadByte(offset);
        }

        public ushort ReadWord(long offset)
        {
            return flash.ReadWord(offset);
        }

        public uint ReadDoubleWord(long offset)
        {
            return flash.ReadDoubleWord(offset);
        }

        public ulong ReadQuadWord(long offset)
        {
            return flash.ReadQuadWord(offset);
        }

        public void WriteByte(long offset, byte value)
        {
            controller.WriteCommandCycle(offset, value, 1);
        }

        public void WriteWord(long offset, ushort value)
        {
            controller.WriteCommandCycle(offset, value, 2);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            controller.WriteCommandCycle(offset, value, 4);
        }

        public void WriteQuadWord(long offset, ulong value)
        {
            controller.WriteCommandCycle(offset, (uint)value, 4);
            controller.WriteCommandCycle(offset + 4, (uint)(value >> 32), 4);
        }

        public byte[] ReadBytes(long offset, int count, IPeripheral context = null)
        {
            return flash.ReadBytes(offset, count, context);
        }

        public void WriteBytes(long offset, byte[] array, int startingIndex, int count, IPeripheral context = null)
        {
            flash.WriteBytes(offset, array, startingIndex, count, context);
        }

        public void LoadFileChunks(string path, IEnumerable<FileChunk> chunks, IPeripheral cpu)
        {
            this.LoadFileChunks(chunks, cpu);
        }

        public long Size => flash.Size;

        public Endianess Endianness => Endianess.LittleEndian;

        private readonly MappedMemory flash;
        private readonly XMC4XXX_FlashController controller;
    }
}
