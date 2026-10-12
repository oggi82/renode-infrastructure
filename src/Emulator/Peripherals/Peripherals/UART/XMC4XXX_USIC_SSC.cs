//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System.Collections.Generic;

using Antmicro.Renode.Logging;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    // Synchronous serial channel (SSC, SPI), Reference Manual chapter 17.4.
    // Master: a word written to TBUF/IN is exchanged at once with the SPI peripherals selected by PCR.SELO
    // (registered with the number of the select line), with the channel set as Peer, or with itself in loopback.
    // A frame ends after SCTR.FLE + 1 bits (FLE = 63: not limited), with TCSR.EOF, or when PCR.SELO changes.
    // Slave: the master (via its Peer property) shifts a word through the slave; the slave answers with
    // its TBUF word (or the passive data level) and receives the master's word.
    public partial class XMC4XXX_USIC
    {
        // Used by the master of a connected Peer; returns the word the slave shifts out
        internal uint SscSlaveExchange(uint word, int bits)
        {
            var mask = bits >= 32 ? uint.MaxValue : (1u << bits) - 1;
            if(CurrentMode != ModeSsc || IsSscMaster)
            {
                return 0;
            }
            uint answer;
            if(tdv || (!tcsrSingleShot.Value && tbufWritten))
            {
                answer = tbufData;
                if(tdv && tcsrSingleShot.Value)
                {
                    ConsumeTransmitBuffer();
                }
            }
            else
            {
                answer = passiveDataLevel.Value ? uint.MaxValue : 0;
            }
            RaiseReceiveStart();
            RaiseTransmitShift();
            ReceiveWord(new ReceivedWord
            {
                Data = word & mask,
                WordLength = (uint)(bits - 1),
                StartOfFrame = sscSlaveStart,
                ProtocolError = sscSlaveStart,
            });
            sscSlaveStart = false;
            Pump();
            return answer & mask;
        }

        internal void SscSlaveEndOfFrame()
        {
            sscSlaveStart = true;
        }

        private bool IsSscMaster => (pcr.Value & 1) != 0;

        private uint SscStatus(uint value)
        {
            return (value & ~3u) | (sscFrameActive ? 1u : 0u) | (InputStageState(2) ? 2u : 0u);
        }

        private void SscClearSelect()
        {
            if(sscFrameActive)
            {
                SscEndFrame((int)(pcr.Value >> 16 & 0xFF));
            }
        }

        private void OnProtocolControlChanged(uint oldValue, uint newValue)
        {
            if(CurrentMode == ModeSsc && sscFrameActive && ((oldValue ^ newValue) & 0xFF0000) != 0)
            {
                SscEndFrame((int)(oldValue >> 16 & 0xFF));
            }
        }

        private void OnSelectTrigger()
        {
            if(CurrentMode == ModeSsc)
            {
                RaiseProtocolEvent(1u << 3, (pcr.Value & 1u << 15) != 0);
            }
        }

        private bool SscStartTransmission()
        {
            if(!IsSscMaster || transferMode.Value != 1)
            {
                return false;
            }
            var selected = (int)(pcr.Value >> 16 & 0xFF);
            if(tcsrStartOfFrame.Value)
            {
                tcsrStartOfFrame.Value = false;
                if(sscFrameActive)
                {
                    SscEndFrame(selected);
                }
            }
            if(sscBusy)
            {
                return false;
            }
            var bits = (int)wordLengthField.Value + 1;
            var mask = (1u << bits) - 1;
            var word = tbufData & mask;
            var endOfFrame = tcsrEndOfFrame.Value;
            tcsrEndOfFrame.Value = false;

            var startOfFrame = !sscFrameActive;
            if(startOfFrame)
            {
                sscFrameActive = true;
                sscFrameBits = 0;
                RaiseProtocolEvent(1u << 2, (pcr.Value & 1u << 14) != 0);
            }
            ConsumeTransmitBuffer();
            RaiseReceiveStart();

            // The word is shifted at fSCLK = fPDIV / 2; the exchange and the receive events happen at its end
            sscBusy = true;
            var current = generation;
            var sclk = ProtocolDividerFrequency() / 2;
            var seconds = bits / (sclk > 0 ? sclk : 1000000.0);
            machine.ScheduleAction(TimeInterval.FromSeconds(seconds), _ => SscWordDone(current, word, bits, selected, startOfFrame, endOfFrame), name: "XMC4XXX_USIC shift");
            return true;
        }

        private void SscWordDone(int expectedGeneration, uint word, int bits, int selected, bool startOfFrame, bool endOfFrame)
        {
            if(expectedGeneration != generation)
            {
                return;
            }
            var answer = SscExchange(word, bits, selected, startOfFrame) & ((1u << bits) - 1);
            sscFrameBits += bits;

            RaiseTransmitShift();
            ReceiveWord(new ReceivedWord
            {
                Data = answer,
                WordLength = (uint)(bits - 1),
                StartOfFrame = startOfFrame,
                ProtocolError = startOfFrame,
            });

            var frameLength = frameLengthField.Value == 63 ? 0 : (int)frameLengthField.Value + 1;
            if(endOfFrame || (frameLength > 0 && sscFrameBits >= frameLength))
            {
                SscEndFrame(selected);
            }
            sscBusy = false;
            Pump();
        }

        private uint SscExchange(uint word, int bits, int selected, bool startOfFrame)
        {
            this.Log(LogLevel.Debug, "SSC word 0x{0:X} ({1} bits), selected 0x{2:X}, peer {3}", word, bits, selected, Peer == null ? "none" : "set");
            if(Peer != null && Peer != this)
            {
                if(startOfFrame)
                {
                    Peer.sscSlaveStart = true;
                }
                return Peer.SscSlaveExchange(word, bits);
            }

            var answered = false;
            var received = new List<bool>();
            var byteCount = (bits + 7) / 8;
            var msbFirst = shiftMsbFirst.Value;
            for(var b = 0; b < byteCount; b++)
            {
                byte transmit = 0;
                for(var k = 0; k < 8; k++)
                {
                    var position = b * 8 + k;
                    var bit = false;
                    if(position < bits)
                    {
                        bit = ((word >> (msbFirst ? bits - 1 - position : position)) & 1) != 0;
                    }
                    transmit = (byte)(transmit << 1 | (bit ? 1 : 0));
                }
                byte receive = 0;
                for(var line = 0; line < 8; line++)
                {
                    if((selected >> line & 1) != 0 && spiContainer.TryGetByAddress(line, out var device))
                    {
                        receive = device.Transmit(transmit);
                        answered = true;
                    }
                }
                for(var k = 0; k < 8; k++)
                {
                    received.Add((receive >> (7 - k) & 1) != 0);
                }
            }

            if(!answered)
            {
                // Nothing attached: only the internal loop-back (input G) or a constant 1 (DSEL = 7) is visible
                if(dxSelect[0].Value == 6)
                {
                    return word;
                }
                return dxSelect[0].Value == 7 ? uint.MaxValue : 0;
            }

            uint result = 0;
            for(var i = 0; i < bits; i++)
            {
                if(received[i])
                {
                    result |= 1u << (msbFirst ? bits - 1 - i : i);
                }
            }
            return result;
        }

        private void SscEndFrame(int selected)
        {
            sscFrameActive = false;
            sscFrameBits = 0;
            RaiseProtocolEvent(1u << 2, (pcr.Value & 1u << 14) != 0);
            if(Peer != null && Peer != this)
            {
                Peer.SscSlaveEndOfFrame();
            }
            for(var line = 0; line < 8; line++)
            {
                if((selected >> line & 1) != 0 && spiContainer.TryGetByAddress(line, out var device))
                {
                    device.FinishTransmission();
                }
            }
        }

        private bool sscBusy;
        private bool sscFrameActive;
        private int sscFrameBits;
        private bool sscSlaveStart = true;
    }
}
