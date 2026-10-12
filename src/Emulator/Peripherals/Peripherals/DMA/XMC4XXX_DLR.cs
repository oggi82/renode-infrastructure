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

namespace Antmicro.Renode.Peripherals.DMA
{
    // DMA Line Router (DLR) of the Infineon XMC4500, Reference Manual v1.6, chapter 4.4 (at 0x50004900, part of the SCU address range).
    // The DLR selects one DMA capable service request source for each of the 12 DMA lines (lines 0-7 belong to GPDMA0, lines 8-11
    // to GPDMA1), stores a request until the GPDMA acknowledges it and flags a request that arrives while one is still pending (overrun).
    //
    // Inputs (OnGPIO): the number is the index of the service request source (see the table below); a rising edge is one
    // service request pulse. The peripherals blink their service request lines, so every pulse is one request.
    //   0-3   ERU0.SR0-3        4-7   VADC.C0SR0-3     8-11  VADC.G0SR0-3     12-15 VADC.G1SR0-3    16-19 VADC.G2SR0-3
    //   20-23 VADC.G3SR0-3      24-27 DSD.SRM0-3       28-29 DAC.SR0-1        30-31 CCU40.SR0-1     32-33 CCU41.SR0-1
    //   34-35 CCU42.SR0-1       36-37 CCU43.SR0-1      38-39 CCU80.SR0-1      40-41 CCU81.SR0-1     42-45 CAN.SR0-3
    //   46-47 USIC0.SR0-1       48-49 USIC1.SR0-1      50-53 USIC2.SR0-3
    // Connections: 0 = the overrun service request (SCU SRRAW.DLROVR), high while any OVRSTAT bit is set.
    //
    // Modeled: SRSEL0/1 (source selection, Table 4-5), LNEN (line enable, enabling or disabling a line drops a pending request), OVRSTAT/OVRCLR. The handshake with the GPDMA units is the Acknowledge call.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class XMC4XXX_DLR : BasicDoubleWordPeripheral, IKnownSize, IGPIOReceiver, INumberedGPIOOutput
    {
        public XMC4XXX_DLR(IMachine machine) : base(machine)
        {
            Connections = new ReadOnlyDictionary<int, IGPIO>(new Dictionary<int, IGPIO> { { 0, new GPIO() } });
            clients = new List<Client>();
            selectedSource = new int[LineCount];
            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            pending = 0;
            overrun = 0;
            lineEnabled = 0;
            for(var i = 0; i < LineCount; i++)
            {
                selectedSource[i] = SourceOfSelection(i, 0);
            }
            UpdateOverrunInterrupt();
        }

        public void OnGPIO(int number, bool value)
        {
            if(!value)
            {
                return;
            }
            for(var line = 0; line < LineCount; line++)
            {
                if((lineEnabled & (1u << line)) == 0 || selectedSource[line] != number)
                {
                    continue;
                }
                if((pending & (1u << line)) != 0)
                {
                    this.Log(LogLevel.Debug, "Service request {0} overruns DMA line {1}", number, line);
                    overrun |= 1u << line;
                    UpdateOverrunInterrupt();
                    continue;
                }
                this.Log(LogLevel.Noisy, "Service request {0} pending on DMA line {1}", number, line);
                pending |= 1u << line;
                NotifyRequest(line);
            }
        }

        // Called by a GPDMA unit: lines firstLine .. firstLine + count - 1 belong to it.
        // The callbacks get the unit relative line number.
        public void AttachDma(int firstLine, int count, Action<int> onRequest, Action<int> onLineReset)
        {
            clients.Add(new Client { FirstLine = firstLine, Count = count, OnRequest = onRequest, OnLineReset = onLineReset });
        }

        public bool IsRequestPending(int line)
        {
            return (pending & (1u << line)) != 0;
        }

        // Handshake: the GPDMA has serviced the request of the line
        public void Acknowledge(int line)
        {
            pending &= ~(1u << line);
        }

        public long Size => 0x100;

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        private void DefineRegisters()
        {
            Registers.OverrunStatus.Define(this)
                .WithValueField(0, LineCount, FieldMode.Read, name: "OVRSTAT", valueProviderCallback: _ => overrun)
                .WithReservedBits(LineCount, 32 - LineCount);

            Registers.OverrunClear.Define(this)
                .WithValueField(0, LineCount, FieldMode.Write, name: "OVRCLR", writeCallback: (_, value) =>
                {
                    overrun &= ~(uint)value;
                    UpdateOverrunInterrupt();
                })
                .WithReservedBits(LineCount, 32 - LineCount);

            // Lines 0-7 of GPDMA0 (4 bits each)
            Registers.ServiceRequestSelection0.Define(this)
                .WithValueField(0, 32, name: "SRSEL0", changeCallback: (_, __) => UpdateSelection(0, 8));
            // Lines 8-11 of GPDMA1
            Registers.ServiceRequestSelection1.Define(this)
                .WithValueField(0, 16, name: "SRSEL1", changeCallback: (_, __) => UpdateSelection(8, 4))
                .WithReservedBits(16, 16);

            Registers.LineEnable.Define(this)
                .WithValueField(0, LineCount, name: "LNEN", writeCallback: (_, value) => WriteLineEnable((uint)value))
                .WithReservedBits(LineCount, 32 - LineCount);
        }

        private void WriteLineEnable(uint value)
        {
            for(var line = 0; line < LineCount; line++)
            {
                var bit = 1u << line;
                var wasEnabled = (lineEnabled & bit) != 0;
                var enable = (value & bit) != 0;
                // a pending request is reset when the line is enabled or disabled (the library clears a request by disabling and enabling the line)
                if(enable != wasEnabled)
                {
                    if((pending & bit) != 0)
                    {
                        pending &= ~bit;
                        ResetLine(line);
                    }
                }
            }
            lineEnabled = value;
        }

        private void UpdateSelection(int firstLine, int count)
        {
            var register = firstLine == 0 ? Registers.ServiceRequestSelection0 : Registers.ServiceRequestSelection1;
            var value = RegistersCollection.Read((long)register);
            for(var i = 0; i < count; i++)
            {
                selectedSource[firstLine + i] = SourceOfSelection(firstLine + i, (int)((value >> (4 * i)) & 0xF));
            }
        }

        private void UpdateOverrunInterrupt()
        {
            Connections[0].Set(overrun != 0);
        }

        private void NotifyRequest(int line)
        {
            foreach(var client in clients)
            {
                if(line >= client.FirstLine && line < client.FirstLine + client.Count)
                {
                    client.OnRequest(line - client.FirstLine);
                }
            }
        }

        private void ResetLine(int line)
        {
            foreach(var client in clients)
            {
                if(line >= client.FirstLine && line < client.FirstLine + client.Count)
                {
                    client.OnLineReset(line - client.FirstLine);
                }
            }
        }

        // Table 4-5: the source selected for a DMA line by the 4 bit value of its RSx field, -1 for the reserved codes
        private static int SourceOfSelection(int line, int selection)
        {
            var table = line < 8 ? SourcesGpdma0[line] : SourcesGpdma1[line - 8];
            return selection < table.Length ? table[selection] : -1;
        }

        private uint pending;
        private uint overrun;
        private uint lineEnabled;

        private readonly int[] selectedSource;
        private readonly List<Client> clients;

        private const int LineCount = 12;

        // Source numbers (see the class comment)
        private const int Eru0 = 0;
        private const int VadcC0 = 4;
        private const int VadcG0 = 8;
        private const int VadcG1 = 12;
        private const int VadcG2 = 16;
        private const int VadcG3 = 20;
        private const int Dsd = 24;
        private const int Dac = 28;
        private const int Ccu40 = 30;
        private const int Ccu41 = 32;
        private const int Ccu42 = 34;
        private const int Ccu43 = 36;
        private const int Ccu80 = 38;
        private const int Ccu81 = 40;
        private const int Can = 42;
        private const int Usic0 = 46;
        private const int Usic1 = 48;
        private const int Usic2 = 50;
        private const int Reserved = -1;

        // Table 4-5, DMA lines 0-7 (GPDMA0), indexed by the RSx value
        private static readonly int[][] SourcesGpdma0 =
        {
            new[] { Eru0 + 0, VadcC0 + 0, VadcG0 + 3, VadcG2 + 0, VadcG2 + 3, Dsd + 0, Ccu40 + 0, Ccu80 + 0, Reserved, Can + 0, Usic0 + 0, Usic1 + 0, Reserved, VadcG3 + 3, Ccu42 + 0, Reserved },
            new[] { Eru0 + 3, VadcC0 + 1, VadcG0 + 2, VadcG1 + 0, VadcG2 + 2, Dac + 0, Ccu40 + 0, Ccu80 + 0, Reserved, Can + 0, Usic0 + 0, Usic1 + 0, Reserved, VadcG3 + 0, Ccu42 + 0, Reserved },
            new[] { Eru0 + 1, VadcC0 + 2, VadcC0 + 3, VadcG1 + 3, VadcG2 + 1, Dsd + 1, Dsd + 3, Ccu40 + 1, Ccu80 + 1, Reserved, Can + 1, Usic0 + 1, Usic1 + 1, VadcG3 + 2, Ccu42 + 1, Reserved },
            new[] { Eru0 + 2, VadcC0 + 2, VadcC0 + 3, VadcG1 + 1, VadcG1 + 2, Dsd + 2, Dac + 1, Ccu40 + 1, Ccu80 + 1, Reserved, Can + 1, Usic0 + 1, Usic1 + 1, VadcG3 + 1, Ccu42 + 1, Reserved },
            new[] { Eru0 + 2, VadcG0 + 0, VadcG0 + 1, VadcG2 + 1, VadcG2 + 2, Dsd + 2, Dac + 1, Ccu41 + 0, Ccu81 + 0, Reserved, Can + 2, Usic0 + 0, Usic1 + 0, VadcG3 + 1, Ccu43 + 0, Reserved },
            new[] { Eru0 + 1, VadcG0 + 0, VadcG0 + 1, VadcG1 + 2, VadcG2 + 0, Dac + 0, Ccu41 + 0, Ccu81 + 0, Reserved, Can + 2, Usic0 + 0, Usic1 + 0, Reserved, VadcG3 + 2, Ccu43 + 0, Reserved },
            new[] { Eru0 + 3, VadcC0 + 1, VadcG0 + 2, VadcG1 + 1, VadcG2 + 3, Dsd + 1, Dsd + 3, Ccu41 + 1, Ccu81 + 1, Reserved, Can + 3, Usic0 + 1, Usic1 + 1, VadcG3 + 0, Ccu43 + 1, Reserved },
            new[] { Eru0 + 0, VadcC0 + 0, VadcG0 + 3, VadcG1 + 0, VadcG1 + 3, Dsd + 0, Ccu41 + 1, Ccu81 + 1, Reserved, Can + 3, Usic0 + 1, Usic1 + 1, Reserved, VadcG3 + 3, Ccu43 + 1, Reserved },
        };

        // Table 4-5, DMA lines 8-11 (GPDMA1), indexed by the RSx value (codes 8-15 are reserved)
        private static readonly int[][] SourcesGpdma1 =
        {
            new[] { Eru0 + 0, VadcC0 + 0, VadcG3 + 0, Dsd + 0, Dac + 0, Ccu42 + 0, Usic2 + 0, Usic2 + 2 },
            new[] { Eru0 + 1, VadcC0 + 1, VadcG3 + 1, Dsd + 1, Dac + 1, Ccu42 + 1, Usic2 + 1, Usic2 + 3 },
            new[] { Eru0 + 2, VadcC0 + 2, VadcG3 + 2, Dsd + 2, Dac + 0, Ccu43 + 0, Usic2 + 0, Usic2 + 2 },
            new[] { Eru0 + 3, VadcC0 + 3, VadcG3 + 3, Dsd + 3, Dac + 1, Ccu43 + 1, Usic2 + 1, Usic2 + 3 },
        };

        private struct Client
        {
            public int FirstLine;
            public int Count;
            public Action<int> OnRequest;
            public Action<int> OnLineReset;
        }

        private enum Registers : long
        {
            OverrunStatus = 0x00,
            OverrunClear = 0x04,
            ServiceRequestSelection0 = 0x08,
            ServiceRequestSelection1 = 0x0C,
            LineEnable = 0x10,
        }
    }
}
