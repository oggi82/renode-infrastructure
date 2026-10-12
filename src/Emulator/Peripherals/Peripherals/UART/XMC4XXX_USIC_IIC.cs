//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System.Collections.Generic;
using System.Linq;

using Antmicro.Renode.Peripherals.I2C;

using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.UART
{
    // Inter-IC bus channel (IIC, I2C), Reference Manual chapter 17.5.
    // Master: the TDF code in TBUF[10:8] drives the bus (start, repeated start, send, receive with ACK/NACK, stop).
    // The addressed device is a Renode I2C peripheral registered with the 7-bit address, or the channel set as Peer.
    // Bus symbols take no time. Slave: the channel is addressed by its Peer's master; data bytes are received
    // into RBUF (first byte of a frame as alternative receive) and a master read returns a prepared TDF = 1 word.
    public partial class XMC4XXX_USIC
    {
        internal bool IicSlaveStart(int addressByte, bool restart)
        {
            if(CurrentMode != ModeIic)
            {
                return false;
            }
            var address = addressByte >> 1;
            var general = addressByte == 0 && (pcr.Value & 1u << 16) != 0;
            if(address != (int)(pcr.Value >> 9 & 0x7F) && !general)
            {
                return false;
            }
            psr |= 1;
            iicSlaveFirstByte = true;
            if(restart)
            {
                RaiseProtocolEvent(1u << 3, (pcr.Value & 1u << 19) != 0);
            }
            else
            {
                RaiseProtocolEvent(1u << 2, (pcr.Value & 1u << 18) != 0);
            }
            if((addressByte & 1) != 0)
            {
                RaiseProtocolEvent(1u << 7, (pcr.Value & 1u << 23) != 0);
            }
            return true;
        }

        internal void IicSlaveReceive(byte data)
        {
            var first = iicSlaveFirstByte;
            iicSlaveFirstByte = false;
            ReceiveWord(new ReceivedWord
            {
                Data = data | (first ? 0x200u : 0u),
                WordLength = 7,
                ProtocolError = first,
            });
        }

        internal byte IicSlaveSend()
        {
            if(tdv && ((tbufData >> 8) & 7) == 1)
            {
                var data = (byte)tbufData;
                ConsumeTransmitBuffer();
                Pump();
                return data;
            }
            this.Log(LogLevel.Warning, "Master read from the IIC slave without a prepared TDF = 1 word in TBUF");
            return 0xFF;
        }

        internal void IicSlaveStop()
        {
            RaiseProtocolEvent(1u << 4, (pcr.Value & 1u << 20) != 0);
        }

        private void ResetProtocolState()
        {
            sscBusy = false;
            sscFrameActive = false;
            sscFrameBits = 0;
            sscSlaveStart = true;
            iicMasterActive = false;
            iicMasterRead = false;
            iicDevice = null;
            iicPeer = null;
            iicWrite.Clear();
            iicSlaveFirstByte = true;
            iicMasterFirstByte = true;
        }

        private bool IicStartTransmission()
        {
            var tdf = (int)(tbufData >> 8 & 7);
            var data = (byte)tbufData;
            switch(tdf)
            {
                case 1:
                    // Slave send word: stays in TBUF until the master reads
                    return false;
                case 4:
                case 5:
                    ConsumeTransmitBuffer();
                    IicMasterStart(data, tdf == 5);
                    return true;
                case 0:
                    ConsumeTransmitBuffer();
                    IicMasterSend(data);
                    return true;
                case 2:
                case 3:
                    ConsumeTransmitBuffer();
                    IicMasterReceive(tdf == 2);
                    return true;
                case 6:
                    ConsumeTransmitBuffer();
                    IicMasterStop();
                    return true;
                default:
                    ConsumeTransmitBuffer();
                    RaiseProtocolEvent(1u << 1, (pcr.Value & 1u << 24) != 0);
                    return true;
            }
        }

        private void IicMasterStart(byte addressByte, bool restart)
        {
            if(iicMasterActive)
            {
                IicFlushWrite();
                if(!restart)
                {
                    RaiseProtocolEvent(1u << 1, (pcr.Value & 1u << 24) != 0);
                }
            }
            iicDevice = null;
            iicPeer = null;
            var acknowledged = false;
            var address = addressByte >> 1;
            if(Peer != null && Peer != this)
            {
                iicPeer = Peer;
                acknowledged = Peer.IicSlaveStart(addressByte, restart);
            }
            else if(i2cContainer.TryGetByAddress(address, out var device))
            {
                iicDevice = device;
                acknowledged = true;
            }
            iicMasterActive = acknowledged;
            iicMasterRead = (addressByte & 1) != 0;
            iicMasterFirstByte = true;
            if(restart)
            {
                RaiseProtocolEvent(1u << 3, (pcr.Value & 1u << 19) != 0);
            }
            else
            {
                RaiseProtocolEvent(1u << 2, (pcr.Value & 1u << 18) != 0);
            }
            IicAcknowledge(acknowledged);
        }

        private void IicMasterSend(byte data)
        {
            if(!iicMasterActive || iicMasterRead)
            {
                RaiseProtocolEvent(1u << 1, (pcr.Value & 1u << 24) != 0);
                return;
            }
            if(iicPeer != null)
            {
                iicPeer.IicSlaveReceive(data);
            }
            else
            {
                iicWrite.Add(data);
            }
            IicAcknowledge(true);
        }

        private void IicMasterReceive(bool acknowledge)
        {
            if(!iicMasterActive || !iicMasterRead)
            {
                RaiseProtocolEvent(1u << 1, (pcr.Value & 1u << 24) != 0);
                return;
            }
            byte data;
            if(iicPeer != null)
            {
                data = iicPeer.IicSlaveSend();
            }
            else
            {
                var bytes = iicDevice.Read(1);
                data = bytes != null && bytes.Length > 0 ? bytes[0] : (byte)0xFF;
            }
            var first = iicMasterFirstByte;
            iicMasterFirstByte = false;
            ReceiveWord(new ReceivedWord
            {
                Data = data | (acknowledge ? 0u : 0x100u) | (first ? 0x200u : 0u) | 0x400u,
                WordLength = 7,
                ProtocolArgument = !acknowledge,
                ProtocolError = first,
            });
            if(!acknowledge)
            {
                iicMasterRead = false;
            }
        }

        private void IicMasterStop()
        {
            IicFlushWrite();
            if(iicMasterActive)
            {
                iicDevice?.FinishTransmission();
                iicPeer?.IicSlaveStop();
            }
            iicMasterActive = false;
            iicDevice = null;
            iicPeer = null;
            RaiseProtocolEvent(1u << 4, (pcr.Value & 1u << 20) != 0);
        }

        private void IicFlushWrite()
        {
            if(iicWrite.Count > 0 && iicDevice != null)
            {
                iicDevice.Write(iicWrite.ToArray());
            }
            iicWrite.Clear();
        }

        private void IicAcknowledge(bool acknowledged)
        {
            if(acknowledged)
            {
                RaiseProtocolEvent(1u << 9, (pcr.Value & 1u << 30) != 0);
            }
            else
            {
                RaiseProtocolEvent(1u << 5, (pcr.Value & 1u << 21) != 0);
            }
        }

        private bool iicMasterActive;
        private bool iicMasterRead;
        private bool iicMasterFirstByte;
        private bool iicSlaveFirstByte;
        private II2CPeripheral iicDevice;
        private XMC4XXX_USIC iicPeer;
        private readonly List<byte> iicWrite = new List<byte>();
    }
}
