//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Mocks
{
    // SPI device that answers every byte with the byte it just received (MISO wired to MOSI), as the SPI_DMA example expects on the kit
    public class SpiLoopbackDevice : ISPIPeripheral
    {
        public void FinishTransmission()
        {
        }

        public void Reset()
        {
        }

        public byte Transmit(byte data)
        {
            return data;
        }
    }
}
