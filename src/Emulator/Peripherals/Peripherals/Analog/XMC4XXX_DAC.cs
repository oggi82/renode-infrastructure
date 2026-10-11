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
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.IRQControllers;

namespace Antmicro.Renode.Peripherals.Analog
{
    // Digital to Analog Converter (DAC) of the Infineon XMC4500 with two 12 bit channels, Reference Manual v1.6, chapter 21.
    //
    // The output of each channel is not driven to a pin but is available to the test: OutputValue (the processed 12 bit
    // value as seen by the converter, signed or unsigned depending on SIGN), OutputCode (0..4095, signed values are shifted
    // by 2048) and OutputVoltage (code * 3.3 V / 4096; 0 V while the output is in standby). Changes of the output that are
    // caused by register writes are logged at info level.
    //
    // Modes: single value, data (the output follows the data register; FIFO and DMA service requests are not modeled),
    // pattern generator (32 samples per period from the nine quarter-wave entries), noise generator (20 bit LFSR) and ramp.
    // The output of the free running modes is computed from the virtual time instead of being updated by events: the internal
    // trigger is the peripheral clock divided by FREQ (minimum 16), the nth trigger since the start of the mode produces
    // sample n-1; before the first trigger the output is 0. External triggers are GPIO inputs 0-7 (TRIGSEL), software
    // triggers come from the SWTRIG bit.
    // GPIO outputs: service request 0 and 1 (NVIC 42, 43); they are only raised for software and external triggers in data mode.
    // The peripheral clock is taken from the NVIC SysTick frequency (fCPU) shifted right by peripheralClockShift (PBDIV).
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class XMC4XXX_DAC : IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize, INumberedGPIOOutput, IGPIOReceiver
    {
        public XMC4XXX_DAC(IMachine machine, NVIC nvic = null, int peripheralClockShift = 0)
        {
            this.machine = machine;
            this.nvic = nvic;
            this.peripheralClockShift = peripheralClockShift;

            RegistersCollection = new DoubleWordRegisterCollection(this);
            var connections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < ChannelCount; i++)
            {
                connections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(connections);

            channels = new Channel[ChannelCount];
            for(var i = 0; i < ChannelCount; i++)
            {
                channels[i] = new Channel(this, i);
            }
            DefineRegisters();
            Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            RegistersCollection.Write(offset, value);
        }

        public void Reset()
        {
            RegistersCollection.Reset();
            foreach(var channel in channels)
            {
                channel.Reset();
            }
            simultaneousData = 0;
        }

        // External trigger inputs 0-7 (rising edge)
        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= 8)
            {
                this.Log(LogLevel.Warning, "Trigger input {0} does not exist", number);
                return;
            }
            if(!value)
            {
                return;
            }
            foreach(var channel in channels)
            {
                if(channel.Master.TriggerMode == 1 && channel.Master.TriggerSelect == number && channel == channel.Master)
                {
                    channel.Trigger();
                }
            }
        }

        // Test interface: the output of channel 0 or 1
        public uint GetOutputCode(uint channel)
        {
            return GetChannel(channel).OutputCode;
        }

        public int GetOutputValue(uint channel)
        {
            return GetChannel(channel).OutputValue;
        }

        public double GetOutputVoltage(uint channel)
        {
            return GetChannel(channel).OutputVoltage;
        }

        // Number of triggers (output updates) since the current mode was started
        public ulong GetSampleNumber(uint channel)
        {
            return GetChannel(channel).SampleNumber;
        }

        public uint Output0Code => GetOutputCode(0);

        public uint Output1Code => GetOutputCode(1);

        public double Output0Voltage => GetOutputVoltage(0);

        public double Output1Voltage => GetOutputVoltage(1);

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public long Size => 0x100;

        private Channel GetChannel(uint channel)
        {
            if(channel >= ChannelCount)
            {
                throw new RecoverableException("DAC channel must be 0 or 1");
            }
            return channels[channel];
        }

        private double CurrentTime => machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds;

        private double PeripheralClock
        {
            get
            {
                try
                {
                    return (nvic != null ? nvic.Frequency : DefaultClock) / (double)(1 << peripheralClockShift);
                }
                catch(Exception)
                {
                    return DefaultClock;
                }
            }
        }

        private void DefineRegisters()
        {
            DefineRegister(Registers.Id, "ID", () => 0x00A5C001u, null);

            for(var i = 0; i < ChannelCount; i++)
            {
                var channel = channels[i];
                DefineRegister(Registers.Dac0Config0 + 8 * i, $"DAC{i}CFG0", () => channel.ReadConfig0(), value => channel.WriteConfig0(value));
                DefineRegister(Registers.Dac0Config1 + 8 * i, $"DAC{i}CFG1", () => channel.ReadConfig1(), value => channel.WriteConfig1(value));
                DefineRegister(Registers.Dac0Data + 4 * i, $"DAC{i}DATA", () => channel.Data, value => channel.WriteData(value & 0xFFF));
                DefineRegister(Registers.Dac0PatternLow + 8 * i, $"DAC{i}PATL", () => channel.PatternLow, value => channel.PatternLow = value & 0x3FFFFFFF);
                DefineRegister(Registers.Dac0PatternHigh + 8 * i, $"DAC{i}PATH", () => channel.PatternHigh, value => channel.PatternHigh = value & 0x7FFF);
            }
            // Simultaneous data register: DATA0 in bits 11:0 and DATA1 in bits 27:16 (also the stop value of the ramp generator)
            DefineRegister(Registers.Dac01Data, "DAC01DATA", () => simultaneousData, value =>
            {
                simultaneousData = value & 0x0FFF0FFF;
                channels[0].WriteSimultaneousData();
                channels[1].WriteSimultaneousData();
            });
        }

        private void DefineRegister(Registers register, string name, Func<uint> read, Action<uint> write)
        {
            var mode = (read != null ? FieldMode.Read : 0) | (write != null ? FieldMode.Write : 0);
            register.Define(this, name: name)
                .WithValueField(0, 32, mode, name: name,
                    valueProviderCallback: read == null ? (Func<ulong, ulong>)null : _ => read(),
                    writeCallback: write == null ? (Action<ulong, ulong>)null : (_, value) => write((uint)value));
        }

        private readonly IMachine machine;
        private readonly NVIC nvic;
        private readonly int peripheralClockShift;
        private readonly Channel[] channels;
        private uint simultaneousData;

        private const int ChannelCount = 2;
        private const double DefaultClock = 120000000;
        private const double ReferenceVoltage = 3.3;
        private const int SamplesPerPeriod = 32;

        private enum Mode
        {
            Disabled = 0,
            SingleValue = 1,
            Data = 2,
            Pattern = 3,
            Noise = 4,
            Ramp = 5,
        }

        private enum Registers : long
        {
            Id = 0x00,
            Dac0Config0 = 0x04,
            Dac0Config1 = 0x08,
            Dac1Config0 = 0x0C,
            Dac1Config1 = 0x10,
            Dac0Data = 0x14,
            Dac1Data = 0x18,
            Dac01Data = 0x1C,
            Dac0PatternLow = 0x20,
            Dac0PatternHigh = 0x24,
            Dac1PatternLow = 0x28,
            Dac1PatternHigh = 0x2C,
        }

        private sealed class Channel
        {
            public Channel(XMC4XXX_DAC owner, int index)
            {
                this.owner = owner;
                this.index = index;
            }

            public void Reset()
            {
                config0 = 0;
                config1 = 0;
                Data = 0;
                PatternLow = 0x3568B0C0;
                PatternHigh = 0x00007FDD;
                SampleNumber = 0;
                lastSync = 0;
                held = 0;
                noiseState = 0;
                noiseSteps = 0;
                lastLogged = int.MinValue;
            }

            // Channel 1 uses the mode, trigger and frequency of channel 0 when DATMOD of channel 0 selects simultaneous data handling
            public Channel Master => index == 1 && (owner.channels[0].config1 & (1u << 15)) != 0 ? owner.channels[0] : this;

            public uint Data;
            public uint PatternLow;
            public uint PatternHigh;
            public ulong SampleNumber;

            public Mode CurrentMode => (Mode)((config0 >> 20) & 7);

            public uint TriggerMode => (config1 >> 17) & 3;

            public uint TriggerSelect => (config1 >> 12) & 7;

            public bool Signed => (config0 & (1u << 23)) != 0;

            public bool OutputEnabled => (config1 & (1u << 24)) != 0;

            public int OutputValue
            {
                get
                {
                    var master = Master;
                    master.Synchronize();
                    switch(master.CurrentMode)
                    {
                    case Mode.Disabled:
                        return held;
                    case Mode.SingleValue:
                    case Mode.Data:
                        return Process(DataValue(), false);
                    case Mode.Pattern:
                        return master.SampleNumber == 0 ? 0 : Process(master.PatternSample(master.SampleNumber - 1), false);
                    case Mode.Noise:
                        return master.SampleNumber == 0 ? 0 : Process((int)master.NoiseSample(), true);
                    case Mode.Ramp:
                        return Process((int)master.RampValue(this), true);
                    default:
                        return held;
                    }
                }
            }

            public uint OutputCode
            {
                get
                {
                    var value = OutputValue;
                    return (uint)(Signed ? value + 2048 : value);
                }
            }

            public double OutputVoltage => OutputEnabled ? OutputCode * ReferenceVoltage / 4096 : 0;

            public uint ReadConfig0()
            {
                return config0 | (CurrentMode != Mode.Disabled ? 1u << 31 : 0u);
            }

            public uint ReadConfig1()
            {
                return config1;
            }

            public void WriteConfig0(uint value)
            {
                var newConfig = value & 0x70FFFFFF;
                var master = Master;
                var oldMode = CurrentMode;
                var newMode = (Mode)((newConfig >> 20) & 7);
                // Time that elapsed with the old divider is accounted for before FREQ changes
                master.Synchronize();
                var before = OutputValue;
                config0 = newConfig;
                if(newMode != oldMode)
                {
                    if(newMode == Mode.Disabled)
                    {
                        held = before;
                    }
                    StartMode(newMode);
                }
                LogOutput("configuration");
            }

            public void WriteConfig1(uint value)
            {
                Master.Synchronize();
                config1 = value & 0xF1FEFFFF;
                if((value & (1u << 16)) != 0 && TriggerMode == 2)
                {
                    Trigger();
                }
                LogOutput("configuration");
            }

            public void WriteData(uint value)
            {
                Data = value;
                LogOutput("data register");
            }

            public void WriteSimultaneousData()
            {
                if(index == 0 || (owner.channels[0].config1 & (1u << 15)) != 0)
                {
                    Data = (owner.simultaneousData >> (16 * index)) & 0xFFF;
                    LogOutput("data register");
                }
            }

            // One trigger pulse from software or an external source
            public void Trigger()
            {
                var master = Master;
                master.Synchronize();
                master.SampleNumber++;
                if(master.CurrentMode == Mode.Data && (master.config0 & (1u << 30)) != 0)
                {
                    owner.Connections[master.index].Blink();
                }
            }

            public void Synchronize()
            {
                if(TriggerMode != 0 || CurrentMode < Mode.Data)
                {
                    return;
                }
                var now = owner.CurrentTime;
                var divider = Math.Max(config0 & 0xFFFFF, 16u);
                var period = divider / owner.PeripheralClock;
                var elapsed = (ulong)Math.Floor((now - lastSync) / period);
                if(elapsed > 0)
                {
                    SampleNumber += elapsed;
                    lastSync += elapsed * period;
                }
            }

            private void StartMode(Mode mode)
            {
                SampleNumber = 0;
                lastSync = owner.CurrentTime;
                noiseState = 0;
                noiseSteps = 0;
                lastLogged = int.MinValue;
                owner.Log(LogLevel.Debug, "DAC{0}: mode {1}", index, mode);
            }

            private int DataValue()
            {
                return Signed ? ((int)(Data << 20)) >> 20 : (int)Data;
            }

            // Offset, scaling and negation of the output stage (Figure 21-4), limited to the range of the converter
            private int Process(int raw, bool unsignedSource)
            {
                long value = raw;
                var offset = (config1 >> 4) & 0xFF;
                value += Signed && !unsignedSource ? (long)(sbyte)offset : (long)offset;
                var scale = (int)(config1 & 7);
                value = (config1 & 8) != 0 ? value << scale : value >> scale;
                if((config0 & (1u << 28)) != 0)
                {
                    value = -value;
                }
                return (int)(Signed ? Math.Max(-2048, Math.Min(2047, value)) : Math.Max(0, Math.Min(4095, value)));
            }

            // Sample n (0-based) of the 32 samples per period: nine entries describe a quarter wave that is mirrored, then inverted
            private int PatternSample(ulong n)
            {
                var k = (int)(n % SamplesPerPeriod);
                var half = k % 16;
                var entry = half <= 8 ? half : 16 - half;
                var value = entry < 6 ? (int)((PatternLow >> (5 * entry)) & 0x1F) : (int)((PatternHigh >> (5 * (entry - 6))) & 0x1F);
                return k >= 16 ? -value : value;
            }

            // 20 bit LFSR with XNOR feedback from bits 20 and 17; the 12 bit output is sampled at each trigger
            private uint NoiseSample()
            {
                var steps = Math.Min(SampleNumber - noiseSteps, 1u << 20);
                for(ulong i = 0; i < steps; i++)
                {
                    var feedback = ((noiseState >> 19) ^ (noiseState >> 16)) & 1;
                    noiseState = ((noiseState << 1) | (feedback ^ 1)) & 0xFFFFF;
                }
                noiseSteps = SampleNumber;
                return noiseState & 0xFFF;
            }

            // Ramp from the start value (DATA of the channel) up to the stop value (DAC01DATA) and back to the start
            private uint RampValue(Channel output)
            {
                var start = output.Data;
                var stop = (owner.simultaneousData >> (16 * output.index)) & 0xFFF;
                if(stop < start)
                {
                    return start;
                }
                return (uint)(start + SampleNumber % (stop - start + 1));
            }

            private void LogOutput(string cause)
            {
                var master = Master;
                if(master.CurrentMode == Mode.Disabled || master.CurrentMode >= Mode.Pattern)
                {
                    return;
                }
                var code = (int)OutputCode;
                if(code != lastLogged)
                {
                    lastLogged = code;
                    owner.Log(LogLevel.Info, "DAC{0}: output code {1} ({2:F3} V) after write of {3}", index, code, OutputVoltage, cause);
                }
            }

            private uint config0;
            private uint config1;
            private double lastSync;
            private int held;
            private uint noiseState;
            private ulong noiseSteps;
            private int lastLogged;

            private readonly XMC4XXX_DAC owner;
            private readonly int index;
        }
    }
}
