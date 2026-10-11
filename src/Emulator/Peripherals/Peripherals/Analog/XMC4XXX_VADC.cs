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
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Analog
{
    // Versatile Analog-to-Digital Converter (VADC) of the Infineon XMC4500, Reference Manual v1.6, chapter 19.
    //
    // The model contains the four converter groups G0..G3 (8 channels, 16 result registers each), the three request
    // sources (queue = source 0, group scan = source 1, background scan = source 2), the arbiter with wait-for-start
    // and cancel-inject-repeat, result handling (alignment, wait-for-read, accumulation, global result register),
    // channel/result/source events with their node pointers, the shared service requests and the external
    // multiplexer control (steady, single-step and sequence mode).
    //
    // The analog side is replaced by test inputs: SetVoltage(group, channel, volts) defines the voltage of an input channel
    // (reference 3.3 V) and SetEmuxVoltage(group, externalChannel, volts) the voltage behind an external multiplexer.
    // A conversion returns floor(U / 3.3 V * 4096), limited to 0..4095, reduced to the width of the input class.
    // Trigger and gate inputs of the request sources (Table 19-13) are shared by all groups and are GPIO inputs:
    // lines 0-15 are the trigger inputs REQTRA..REQTRP, lines 16-31 the gate inputs REQGTA..REQGTP. They can be driven by
    // other peripherals (CCU4/CCU8/ERU) or from tests with SetTriggerInput/SetGateInput/PulseTrigger.
    // GPIO outputs (Connections): 0-15 group service request lines G0SR0..G3SR3, 16-19 common lines C0SR0..C0SR3,
    // 20-22 EMUX interface 0 select lines, 23-25 EMUX interface 1 select lines.
    //
    // Not modeled: result FIFOs, filter and difference mode, synchronized conversions (parallel sampling), equidistant
    // sampling (timer mode), broken wire detection, boundary flag generation (the registers only store software settings),
    // diagnostics, calibration time (start-up calibration finishes immediately) and the exact arbiter round timing.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public sealed class XMC4XXX_VADC : IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize, INumberedGPIOOutput, IGPIOReceiver
    {
        public XMC4XXX_VADC(IMachine machine, NVIC nvic = null, uint minimumConversionTime = 10)
        {
            this.machine = machine;
            this.nvic = nvic;
            this.minimumConversionTime = Math.Max(1u, minimumConversionTime);

            RegistersCollection = new DoubleWordRegisterCollection(this);
            var connections = new Dictionary<int, IGPIO>();
            for(var i = 0; i < OutputLineCount; i++)
            {
                connections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(connections);

            backgroundSource = new ScanSource(this, GroupCount, true);
            globalResult = new ResultRegister();
            groups = new Group[GroupCount];
            for(var i = 0; i < GroupCount; i++)
            {
                groups[i] = new Group(this, i);
            }

            DefineGlobalRegisters();
            foreach(var group in groups)
            {
                group.DefineRegisters();
            }
            ResetState();
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
            ResetState();
        }

        public void OnGPIO(int number, bool value)
        {
            if(number >= 0 && number < InputLineCount)
            {
                SetTriggerInput((uint)number, value);
            }
            else if(number >= InputLineCount && number < 2 * InputLineCount)
            {
                SetGateInput((uint)(number - InputLineCount), value);
            }
            else
            {
                this.Log(LogLevel.Warning, "Input line {0} does not exist", number);
            }
        }

        // Test and integration interface

        // Sets the voltage at analog input channel `channel` of group `group` (V, reference voltage is 3.3 V)
        public void SetVoltage(uint group, uint channel, double volts)
        {
            CheckChannel(group, channel);
            voltages[group, channel] = volts;
        }

        public double GetVoltage(uint group, uint channel)
        {
            CheckChannel(group, channel);
            return voltages[group, channel];
        }

        // Sets the voltage at input `externalChannel` (0-7) of the external multiplexer that is controlled by group `group`
        public void SetEmuxVoltage(uint group, uint externalChannel, double volts)
        {
            CheckChannel(group, externalChannel);
            emuxVoltages[group, externalChannel] = volts;
        }

        public double GetEmuxVoltage(uint group, uint externalChannel)
        {
            CheckChannel(group, externalChannel);
            return emuxVoltages[group, externalChannel];
        }

        // Value on the EMUX select outputs of group `group` (binary or Gray coded, as configured)
        public uint GetEmuxSelect(uint group)
        {
            CheckChannel(group, 0);
            return groups[group].EmuxOutputCode;
        }

        // RESULT field of result register `register` of group `group` without clearing the valid flag (debug view)
        public uint GetResult(uint group, uint register)
        {
            CheckChannel(group, 0);
            if(register >= ResultRegisterCount)
            {
                throw new RecoverableException("Result register number must be 0-15");
            }
            return groups[group].Results[register].Result;
        }

        public void SetTriggerInput(uint line, bool level)
        {
            CheckLine(line);
            var old = triggerLevels[line];
            triggerLevels[line] = level;
            if(old != level)
            {
                DispatchTriggerEdge((int)line, level, false);
                ArbitrateAll();
            }
        }

        public void SetGateInput(uint line, bool level)
        {
            CheckLine(line);
            var old = gateLevels[line];
            gateLevels[line] = level;
            if(old != level)
            {
                DispatchTriggerEdge((int)line, level, true);
                ArbitrateAll();
            }
        }

        // A rising and a falling edge on trigger input `line`
        public void PulseTrigger(uint line)
        {
            SetTriggerInput(line, true);
            SetTriggerInput(line, false);
        }

        public ulong ConversionCount { get; private set; }

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public long Size => 0x1400;

        private void ResetState()
        {
            ConversionCount = 0;
            clockControlDisabled = true;
            sleepModeDisabled = false;
            globalConfiguration = 0xF;
            Array.Clear(triggerLevels, 0, triggerLevels.Length);
            Array.Clear(gateLevels, 0, gateLevels.Length);
            globalClass[0] = globalClass[1] = 0;
            globalBoundary = 0;
            globalSourceEvent = false;
            globalResultEvent = false;
            globalEventNodePointer = 0;
            globalTestFunctions = 0;
            emuxSelect = 0;
            outputControlStatus = 0;
            backgroundSource.Reset();
            globalResult.Reset();
            foreach(var group in groups)
            {
                group.Reset();
            }
            foreach(var connection in Connections.Values)
            {
                connection.Unset();
            }
        }

        private void CheckChannel(uint group, uint channel)
        {
            if(group >= GroupCount || channel >= ChannelsPerGroup)
            {
                throw new RecoverableException($"Group must be 0-{GroupCount - 1} and channel 0-{ChannelsPerGroup - 1}");
            }
        }

        private void CheckLine(uint line)
        {
            if(line >= InputLineCount)
            {
                throw new RecoverableException($"Trigger/gate line must be 0-{InputLineCount - 1} (A-P)");
            }
        }

        private void ArbitrateAll()
        {
            foreach(var group in groups)
            {
                group.Arbitrate();
            }
        }

        // An edge on trigger input `line` (or on gate input `line` if `gate` is set) can generate trigger events in every request source
        private void DispatchTriggerEdge(int line, bool rising, bool gate)
        {
            foreach(var group in groups)
            {
                group.Queue.OnInputEdge(line, rising, gate);
                group.GroupScan.OnInputEdge(line, rising, gate);
            }
            backgroundSource.OnInputEdge(line, rising, gate);
        }

        private void Pulse(int line)
        {
            Connections[line].Blink();
        }

        // Node pointer of a group event: 0-3 are the service request lines of the group, 4-7 the shared lines of the common block C0
        private void RaiseGroupServiceRequest(int group, uint nodePointer)
        {
            if(nodePointer < 4)
            {
                Pulse(group * 4 + (int)nodePointer);
            }
            else if(nodePointer < 8)
            {
                Pulse(CommonLineBase + (int)nodePointer - 4);
            }
            else
            {
                this.Log(LogLevel.Warning, "Reserved service request node pointer {0}", nodePointer);
            }
        }

        // Node pointer of a global event (background source): 0-3 are the shared lines of C0, 4-7 belong to a second common block that the XMC4500 does not have
        private void RaiseGlobalServiceRequest(uint nodePointer)
        {
            if(nodePointer < 4)
            {
                Pulse(CommonLineBase + (int)nodePointer);
            }
            else
            {
                this.Log(LogLevel.Warning, "Service request node pointer {0} selects a common block that does not exist in the XMC4500", nodePointer);
            }
        }

        private double ConversionClock
        {
            get
            {
                try
                {
                    return nvic != null ? nvic.Frequency : DefaultModuleClock;
                }
                catch(Exception)
                {
                    return DefaultModuleClock;
                }
            }
        }

        // Defines a 32 bit register whose read and write behavior is implemented by the given callbacks
        private void DefineRegister(long offset, string name, Func<uint> read, Action<uint> write)
        {
            var mode = (read != null ? FieldMode.Read : 0) | (write != null ? FieldMode.Write : 0);
            ((Registers)offset).Define(this, name: name)
                .WithValueField(0, 32, mode, name: name,
                    valueProviderCallback: read == null ? (Func<ulong, ulong>)null : _ => read(),
                    writeCallback: write == null ? (Action<ulong, ulong>)null : (_, value) => write((uint)value));
        }

        // Register that only stores the bits selected by `mask`
        private void DefineStoredRegister(long offset, string name, Func<uint> get, Action<uint> set, uint mask, Action afterWrite = null)
        {
            DefineRegister(offset, name, get, value =>
            {
                set(value & mask);
                afterWrite?.Invoke();
            });
        }

        private void DefineGlobalRegisters()
        {
            DefineRegister(0x00, "CLC", () => (clockControlDisabled ? 3u : 0u) | (sleepModeDisabled ? 8u : 0u), value =>
            {
                clockControlDisabled = (value & 1) != 0;
                sleepModeDisabled = (value & 8) != 0;
            });
            DefineRegister(0x08, "ID", () => 0x00C5C001u, null);
            DefineStoredRegister(0x28, "OCS", () => outputControlStatus, v => outputControlStatus = v, 0x0F000007);

            DefineRegister(0x80, "GLOBCFG", () => globalConfiguration, value =>
            {
                // DIVA, DCMSB and DIVD are only written together with DIVWC
                var dividerMask = (value & (1u << 15)) != 0 ? 0x039Fu : 0u;
                globalConfiguration = (globalConfiguration & ~(dividerMask | 0x000F0000)) | (value & dividerMask) | (value & 0x000F0000);
                if((value & (1u << 31)) != 0)
                {
                    // Start-up calibration: finished immediately, ARBCFG.CAL never reads as 1
                    this.Log(LogLevel.Debug, "Start-up calibration requested");
                }
            });
            DefineStoredRegister(0xA0, "GLOBICLASS0", () => globalClass[0], v => globalClass[0] = v, ClassMask);
            DefineStoredRegister(0xA4, "GLOBICLASS1", () => globalClass[1], v => globalClass[1] = v, ClassMask);
            DefineStoredRegister(0xB8, "GLOBBOUND", () => globalBoundary, v => globalBoundary = v, 0x0FFF0FFF);

            DefineRegister(0xE0, "GLOBEFLAG", () => (globalSourceEvent ? 1u : 0u) | (globalResultEvent ? 1u << 8 : 0u), value =>
            {
                if((value & (1u << 0)) != 0)
                {
                    globalSourceEvent = true;
                    RaiseGlobalServiceRequest(globalEventNodePointer & 0xF);
                }
                if((value & (1u << 8)) != 0)
                {
                    globalResultEvent = true;
                    RaiseGlobalServiceRequest((globalEventNodePointer >> 16) & 0xF);
                }
                if((value & (1u << 16)) != 0)
                {
                    globalSourceEvent = false;
                }
                if((value & (1u << 24)) != 0)
                {
                    globalResultEvent = false;
                }
            });
            DefineStoredRegister(0x140, "GLOBEVNP", () => globalEventNodePointer, v => globalEventNodePointer = v, 0x000F000F);
            DefineStoredRegister(0x160, "GLOBTF", () => globalTestFunctions, v => globalTestFunctions = v, 0x00010F00 | 0xF0);

            for(var i = 0; i < GroupCount; i++)
            {
                var lane = i;
                DefineRegister(0x180 + 4 * i, $"BRSSEL{i}", () => backgroundSource.Select[lane], value => backgroundSource.Select[lane] = value & 0xFF);
                DefineRegister(0x1C0 + 4 * i, $"BRSPND{i}", () => backgroundSource.Pending[lane], value =>
                {
                    // Writing a pending register updates the select register and generates a load event
                    backgroundSource.Select[lane] = value & 0xFF;
                    backgroundSource.LoadEvent();
                });
            }
            backgroundSource.DefineControlRegisters(0x200, 0x204);

            DefineRegister(0x280, "GLOBRCR", () => globalResult.ControlRaw, value => globalResult.ControlRaw = value & 0x810F0000);
            DefineRegister(0x300, "GLOBRES", () =>
            {
                var composed = globalResult.Compose(true);
                globalResult.Valid = false;
                ArbitrateAll();
                return composed;
            }, value =>
            {
                if((value & (1u << 31)) != 0)
                {
                    globalResult.Valid = false;
                    globalResult.AccumulationLeft = 0;
                    ArbitrateAll();
                }
                globalResult.Result = value & 0xFFFF;
            });
            DefineRegister(0x380, "GLOBRESD", () => globalResult.Compose(true), null);

            DefineRegister(0x3F0, "EMUXSEL", () => emuxSelect, value =>
            {
                emuxSelect = value & 0xFF;
                foreach(var group in groups)
                {
                    group.UpdateEmuxOutputs();
                }
            });
        }

        private bool GateLevel(uint select)
        {
            return gateLevels[select & 0xF];
        }

        private static uint Nibble(uint value, int index)
        {
            return (value >> (4 * index)) & 0xF;
        }

        private static int HighestBit(uint value)
        {
            for(var i = 31; i >= 0; i--)
            {
                if((value & (1u << i)) != 0)
                {
                    return i;
                }
            }
            return -1;
        }

        private readonly IMachine machine;
        private readonly NVIC nvic;
        private readonly uint minimumConversionTime;
        private readonly Group[] groups;
        private readonly ScanSource backgroundSource;
        private readonly ResultRegister globalResult;
        private readonly double[,] voltages = new double[GroupCount, ChannelsPerGroup];
        private readonly double[,] emuxVoltages = new double[GroupCount, ChannelsPerGroup];
        private readonly bool[] triggerLevels = new bool[InputLineCount];
        private readonly bool[] gateLevels = new bool[InputLineCount];
        private readonly uint[] globalClass = new uint[2];

        private bool clockControlDisabled;
        private bool sleepModeDisabled;
        private uint globalConfiguration;
        private uint globalBoundary;
        private uint globalEventNodePointer;
        private uint globalTestFunctions;
        private uint outputControlStatus;
        private uint emuxSelect;
        private bool globalSourceEvent;
        private bool globalResultEvent;

        private const int GroupCount = 4;
        private const int ChannelsPerGroup = 8;
        private const int ResultRegisterCount = 16;
        private const int QueueDepth = 8;
        private const int InputLineCount = 16;
        private const int CommonLineBase = 16;
        private const int EmuxLineBase = 20;
        private const int OutputLineCount = 26;
        private const double ReferenceVoltage = 3.3;
        private const double DefaultModuleClock = 120000000;
        private const uint ClassMask = 0x071F071F;

        private enum Registers : long
        {
        }

        // Conversion request that a request source offers to the arbiter of a group
        private sealed class Request
        {
            public Group Group;
            public SourceBase Source;
            public int SourceIndex;
            public int Channel;
            public QueueEntry Entry;
            public bool EmuxActive;
            public uint EmuxSetting;
            public bool IsContinuation;
        }

        private sealed class QueueEntry
        {
            public int Channel;
            public bool Refill;
            public bool EnableInterrupt;
            public bool ExternalTrigger;

            public QueueEntry Clone()
            {
                return (QueueEntry)MemberwiseClone();
            }
        }

        private sealed class ResultRegister
        {
            public void Reset()
            {
                Result = 0;
                Drc = 0;
                Channel = 0;
                Emux = 0;
                RequestSource = 0;
                FastCompare = false;
                Valid = false;
                ControlRaw = 0;
                Accumulator = 0;
                AccumulationLeft = 0;
            }

            // Stores a new conversion result; returns false if the result is only a partial accumulation
            public bool Store(uint value, uint channel, uint emux, uint source, bool fastCompare, bool fastCompareResult)
            {
                Channel = channel;
                Emux = emux;
                RequestSource = source;
                if(fastCompare)
                {
                    FastCompare = fastCompareResult;
                    Valid = true;
                    return true;
                }
                if(DataReductionControl != 0 && DataModificationMode == 0)
                {
                    if(AccumulationLeft == 0)
                    {
                        Accumulator = 0;
                        AccumulationLeft = DataReductionControl + 1;
                    }
                    Accumulator += value;
                    AccumulationLeft--;
                    Drc = AccumulationLeft;
                    if(AccumulationLeft != 0)
                    {
                        return false;
                    }
                    value = Accumulator;
                }
                else
                {
                    Drc = 0;
                }
                Result = value & 0xFFFF;
                Valid = true;
                return true;
            }

            // The group view has DRC in bits 19:16, the global view GNR (group number) at the same place
            public uint Compose(bool global)
            {
                return (Result & 0xFFFF)
                    | ((global ? GroupNumber & 0xF : Drc & 0xF) << 16)
                    | ((Channel & 0x1F) << 20)
                    | ((Emux & 7) << 25)
                    | ((RequestSource & 3) << 28)
                    | (FastCompare ? 1u << 30 : 0u)
                    | (Valid ? 1u << 31 : 0u);
            }

            public uint Result;
            public uint Drc;
            public uint Channel;
            public uint Emux;
            public uint RequestSource;
            public uint GroupNumber;
            public bool FastCompare;
            public bool Valid;
            public uint ControlRaw;
            public uint Accumulator;
            public uint AccumulationLeft;

            public uint DataReductionControl => (ControlRaw >> 16) & 0xF;
            public uint DataModificationMode => (ControlRaw >> 20) & 3;
            public bool WaitForRead => (ControlRaw & (1u << 24)) != 0;
            public bool ServiceRequestEnable => (ControlRaw & (1u << 31)) != 0;
        }

        // Common trigger/gate/result-register control of a request source (QCTRL0/ASCTRL/BRSCTRL and the ENGT/ENTR/RPTDIS mode bits)
        private abstract class SourceBase
        {
            protected SourceBase(XMC4XXX_VADC owner)
            {
                this.owner = owner;
            }

            public virtual void Reset()
            {
                ResultTarget = 0;
                TriggerSelect = 0;
                TriggerMode = 0;
                GateSelect = 0;
                TimerMode = false;
                EnableGate = 0;
                EnableTrigger = false;
                RepeatDisable = false;
            }

            public uint ReadControl()
            {
                var triggerLevel = TriggerSelect == 15 ? owner.GateLevel(GateSelect) : owner.triggerLevels[TriggerSelect];
                return ResultTarget
                    | (TriggerSelect << 8)
                    | (triggerLevel ? 1u << 12 : 0u)
                    | (TriggerMode << 13)
                    | (GateSelect << 16)
                    | (owner.GateLevel(GateSelect) ? 1u << 20 : 0u)
                    | (TimerMode ? 1u << 28 : 0u);
            }

            public void WriteControl(uint value)
            {
                ResultTarget = value & 0xF;
                if((value & (1u << 15)) != 0)
                {
                    TriggerSelect = (value >> 8) & 0xF;
                    TriggerMode = (value >> 13) & 3;
                }
                if((value & (1u << 23)) != 0)
                {
                    GateSelect = (value >> 16) & 0xF;
                }
                if((value & (1u << 31)) != 0)
                {
                    TimerMode = (value & (1u << 28)) != 0;
                }
                owner.ArbitrateAll();
            }

            public bool GateOpen
            {
                get
                {
                    switch(EnableGate)
                    {
                    case 1:
                        return true;
                    case 2:
                        return owner.GateLevel(GateSelect);
                    case 3:
                        return !owner.GateLevel(GateSelect);
                    default:
                        return false;
                    }
                }
            }

            // Edge on an external trigger (gate = false) or gate input; trigger select 15 uses the selected gate input as trigger
            public void OnInputEdge(int line, bool rising, bool gate)
            {
                if(!EnableTrigger || TriggerMode == 0)
                {
                    return;
                }
                var matches = TriggerSelect == 15 ? (gate && GateSelect == line) : (!gate && TriggerSelect == line);
                var edgeSelected = rising ? (TriggerMode & 2) != 0 : (TriggerMode & 1) != 0;
                if(matches && edgeSelected)
                {
                    ExternalTrigger();
                }
            }

            public abstract bool TryGetRequest(Group group, out Request request);
            public abstract void Started(Request request);
            public abstract void Completed(Request request);
            public abstract void Aborted(Request request);

            public uint ResultTarget;
            public uint TriggerSelect;
            public uint TriggerMode;
            public uint GateSelect;
            public bool TimerMode;
            public uint EnableGate;
            public bool EnableTrigger;
            public bool RepeatDisable;

            protected abstract void ExternalTrigger();

            protected readonly XMC4XXX_VADC owner;
        }

        // Request source 0: queue with 8 stages and a backup stage for aborted conversions
        private sealed class QueueSource : SourceBase
        {
            public QueueSource(XMC4XXX_VADC owner, Group group) : base(owner)
            {
                this.group = group;
            }

            public override void Reset()
            {
                base.Reset();
                Stages.Clear();
                Backup = null;
                running = null;
                Event = false;
            }

            public void TriggerEvent()
            {
                // The event is only detected while a valid entry is in the queue
                if(Stages.Count > 0 || Backup != null)
                {
                    Event = true;
                }
            }

            public void Add(uint value)
            {
                if(Stages.Count >= QueueDepth)
                {
                    return;
                }
                Stages.Add(new QueueEntry
                {
                    Channel = (int)(value & 0x1F),
                    Refill = (value & (1u << 5)) != 0,
                    EnableInterrupt = (value & (1u << 6)) != 0,
                    ExternalTrigger = (value & (1u << 7)) != 0,
                });
            }

            public override bool TryGetRequest(Group requestingGroup, out Request request)
            {
                request = null;
                if(!GateOpen)
                {
                    return false;
                }
                var entry = Backup ?? (Stages.Count > 0 ? Stages[0] : null);
                if(entry == null)
                {
                    return false;
                }
                // An aborted conversion was triggered before and is repeated without waiting for another trigger
                if(Backup == null && entry.ExternalTrigger && !Event)
                {
                    return false;
                }
                if(group.IsBlockedByWaitForRead(entry.Channel, ResultTarget, false))
                {
                    return false;
                }
                request = new Request { Group = group, Source = this, SourceIndex = 0, Channel = entry.Channel, Entry = entry };
                return true;
            }

            public override void Started(Request request)
            {
                if(Backup != null && ReferenceEquals(request.Entry, Backup))
                {
                    running = Backup;
                    Backup = null;
                    return;
                }
                running = Stages[0];
                Stages.RemoveAt(0);
                if(running.Refill)
                {
                    Stages.Add(running.Clone());
                }
                Event = false;
            }

            public override void Completed(Request request)
            {
                if(running != null && running.EnableInterrupt)
                {
                    group.RaiseSourceEvent(0);
                }
                running = null;
            }

            public override void Aborted(Request request)
            {
                if(!RepeatDisable)
                {
                    Backup = running;
                }
                running = null;
            }

            public void DefineRegisters(long baseOffset)
            {
                owner.DefineRegister(baseOffset + 0x00, "QCTRL0", ReadControl, WriteControl);
                owner.DefineRegister(baseOffset + 0x04, "QMR0", () => EnableGate | (EnableTrigger ? 4u : 0u) | (RepeatDisable ? 1u << 16 : 0u), value =>
                {
                    EnableGate = value & 3;
                    EnableTrigger = (value & 4) != 0;
                    RepeatDisable = (value & (1u << 16)) != 0;
                    if((value & (1u << 8)) != 0)
                    {
                        // CLRV: invalidate the backup entry if there is one, otherwise stage 0
                        if(Backup != null)
                        {
                            Backup = null;
                        }
                        else if(Stages.Count > 0)
                        {
                            Stages.RemoveAt(0);
                        }
                        Event = false;
                    }
                    if((value & (1u << 10)) != 0)
                    {
                        Stages.Clear();
                        Backup = null;
                        Event = false;
                    }
                    if((value & (1u << 11)) != 0)
                    {
                        Event = false;
                    }
                    if((value & (1u << 9)) != 0)
                    {
                        TriggerEvent();
                    }
                    owner.ArbitrateAll();
                });
                owner.DefineRegister(baseOffset + 0x08, "QSR0", () =>
                    (uint)(Stages.Count > 0 ? Stages.Count - 1 : 0)
                    | (Stages.Count == 0 ? 1u << 5 : 0u)
                    | (owner.GateLevel(GateSelect) ? 1u << 7 : 0u)
                    | (Event ? 1u << 8 : 0u), null);
                owner.DefineRegister(baseOffset + 0x0C, "Q0R0", () => Stages.Count > 0 ? Encode(Stages[0]) : 0, null);
                owner.DefineRegister(baseOffset + 0x10, "QINR0/QBUR0", () => Backup != null ? Encode(Backup) : 0, value =>
                {
                    Add(value);
                    owner.ArbitrateAll();
                });
            }

            public readonly List<QueueEntry> Stages = new List<QueueEntry>();
            public QueueEntry Backup;
            public bool Event;

            protected override void ExternalTrigger()
            {
                TriggerEvent();
            }

            private static uint Encode(QueueEntry entry)
            {
                return (uint)entry.Channel
                    | (entry.Refill ? 1u << 5 : 0u)
                    | (entry.EnableInterrupt ? 1u << 6 : 0u)
                    | (entry.ExternalTrigger ? 1u << 7 : 0u)
                    | (1u << 8);
            }

            private QueueEntry running;
            private readonly Group group;
        }

        // Request sources 1 (group scan, one lane) and 2 (background scan, one lane per group)
        private sealed class ScanSource : SourceBase
        {
            public ScanSource(XMC4XXX_VADC owner, int lanes, bool background, Group group = null) : base(owner)
            {
                this.background = background;
                this.group = group;
                Select = new uint[lanes];
                Pending = new uint[lanes];
            }

            public override void Reset()
            {
                base.Reset();
                Array.Clear(Select, 0, Select.Length);
                Array.Clear(Pending, 0, Pending.Length);
                EnableInterrupt = false;
                Autoscan = false;
                Combine = false;
                running = 0;
            }

            // Copies (or ORs, in combine mode) the select registers to the pending registers
            public void LoadEvent()
            {
                for(var lane = 0; lane < Select.Length; lane++)
                {
                    Pending[lane] = (Combine ? Pending[lane] : 0) | (Select[lane] & ChannelMask(lane));
                }
                owner.ArbitrateAll();
            }

            public void ClearPending()
            {
                Array.Clear(Pending, 0, Pending.Length);
            }

            public override bool TryGetRequest(Group requestingGroup, out Request request)
            {
                request = null;
                var lane = background ? requestingGroup.Index : 0;
                if(!GateOpen)
                {
                    return false;
                }
                var channel = HighestBit(Pending[lane] & ChannelMask(lane));
                if(channel < 0)
                {
                    return false;
                }
                if(requestingGroup.IsBlockedByWaitForRead(channel, ResultTarget, background))
                {
                    return false;
                }
                request = new Request { Group = requestingGroup, Source = this, SourceIndex = background ? 2 : 1, Channel = channel };
                return true;
            }

            public override void Started(Request request)
            {
                Pending[background ? request.Group.Index : 0] &= ~(1u << request.Channel);
                running++;
            }

            public override void Completed(Request request)
            {
                running--;
                if(running == 0 && AllPendingCleared())
                {
                    SourceEvent();
                    if(Autoscan)
                    {
                        LoadEvent();
                    }
                }
            }

            public override void Aborted(Request request)
            {
                running--;
                if(!RepeatDisable)
                {
                    Pending[background ? request.Group.Index : 0] |= 1u << request.Channel;
                }
            }

            public void DefineControlRegisters(long controlOffset, long modeOffset)
            {
                owner.DefineRegister(controlOffset, "SCANCTRL", ReadControl, WriteControl);
                owner.DefineRegister(modeOffset, "SCANMR", () =>
                    EnableGate
                    | (EnableTrigger ? 1u << 2 : 0u)
                    | (EnableInterrupt ? 1u << 3 : 0u)
                    | (Autoscan ? 1u << 4 : 0u)
                    | (Combine ? 1u << 5 : 0u)
                    | (owner.GateLevel(GateSelect) ? 1u << 7 : 0u)
                    | (RepeatDisable ? 1u << 16 : 0u), value =>
                {
                    EnableGate = value & 3;
                    EnableTrigger = (value & (1u << 2)) != 0;
                    EnableInterrupt = (value & (1u << 3)) != 0;
                    Autoscan = (value & (1u << 4)) != 0;
                    Combine = (value & (1u << 5)) != 0;
                    RepeatDisable = (value & (1u << 16)) != 0;
                    if((value & (1u << 8)) != 0)
                    {
                        ClearPending();
                    }
                    if((value & (1u << 9)) != 0)
                    {
                        LoadEvent();
                    }
                    owner.ArbitrateAll();
                });
            }

            public readonly uint[] Select;
            public readonly uint[] Pending;
            public bool EnableInterrupt;
            public bool Autoscan;
            public bool Combine;

            protected override void ExternalTrigger()
            {
                LoadEvent();
            }

            // Priority channels of a group cannot be converted by the background source
            private uint ChannelMask(int lane)
            {
                return background ? ~owner.groups[lane].PriorityChannels & 0xFF : 0xFF;
            }

            private bool AllPendingCleared()
            {
                for(var lane = 0; lane < Pending.Length; lane++)
                {
                    if((Pending[lane] & ChannelMask(lane)) != 0)
                    {
                        return false;
                    }
                }
                return true;
            }

            private void SourceEvent()
            {
                if(background)
                {
                    owner.globalSourceEvent = true;
                    if(EnableInterrupt)
                    {
                        owner.RaiseGlobalServiceRequest(owner.globalEventNodePointer & 0xF);
                    }
                }
                else
                {
                    group.RaiseSourceEvent(1, EnableInterrupt);
                }
            }

            private int running;
            private readonly bool background;
            private readonly Group group;
        }

        // One converter group: arbiter, converter, queue and scan source, channels and result registers
        private sealed class Group
        {
            public Group(XMC4XXX_VADC owner, int index)
            {
                this.owner = owner;
                Index = index;
                Queue = new QueueSource(owner, this);
                GroupScan = new ScanSource(owner, 1, false, this);
                Results = new ResultRegister[ResultRegisterCount];
                for(var i = 0; i < ResultRegisterCount; i++)
                {
                    Results[i] = new ResultRegister { GroupNumber = (uint)index };
                }
                timer = new LimitTimer(owner.machine.ClockSource, 1000000, owner, $"vadc_g{index}", direction: Direction.Ascending, workMode: WorkMode.OneShot, eventEnabled: true);
                timer.LimitReached += OnConversionFinished;
            }

            public void Reset()
            {
                timer.Enabled = false;
                busy = false;
                current = null;
                continuation = null;
                Queue.Reset();
                GroupScan.Reset();
                foreach(var result in Results)
                {
                    result.Reset();
                }
                Array.Clear(channelControl, 0, channelControl.Length);
                arbitrationConfiguration = 0;
                arbitrationPriority = 0;
                PriorityChannels = 0;
                classes[0] = classes[1] = 0;
                alias = 0x0100;
                boundary = 0;
                synchronization = 0;
                boundaryFlags = 0;
                boundaryFlagControl = 0;
                boundaryFlagNodePointer = 0xFFFF;
                channelEvents = 0;
                resultEvents = 0;
                sourceEvents = 0;
                channelEventNodePointer = 0;
                resultEventNodePointer0 = 0;
                resultEventNodePointer1 = 0;
                sourceEventNodePointer = 0;
                emuxControl = 0;
                emuxActual = 0;
                EmuxOutputCode = 0;
            }

            public int Index { get; }

            public QueueSource Queue { get; }

            public ScanSource GroupScan { get; }

            public ResultRegister[] Results { get; }

            public uint PriorityChannels { get; private set; }

            public uint EmuxOutputCode { get; private set; }

            private bool Enabled => (arbitrationConfiguration & 3) == 3;

            private uint EmuxSetting => emuxControl & 7;

            private uint EmuxMode => (emuxControl >> 26) & 3;

            public void RaiseSourceEvent(int source, bool raiseServiceRequest = true)
            {
                sourceEvents |= 1u << source;
                if(raiseServiceRequest)
                {
                    owner.RaiseGroupServiceRequest(Index, Nibble(sourceEventNodePointer, source));
                }
            }

            public bool IsBlockedByWaitForRead(int channel, uint sourceResultRegister, bool background)
            {
                var register = ResolveResultRegister(channel, sourceResultRegister, background);
                return register.WaitForRead && register.Valid;
            }

            // Selects the request source that gets the converter: highest priority wins, equal priorities are resolved by the lower slot number
            public void Arbitrate()
            {
                if(arbitrating || !Enabled)
                {
                    return;
                }
                arbitrating = true;
                try
                {
                    if(continuation != null)
                    {
                        if(!busy && !IsBlockedByWaitForRead(continuation.Channel, continuation.Source.ResultTarget, continuation.SourceIndex == 2))
                        {
                            var next = continuation;
                            continuation = null;
                            Start(next);
                        }
                        return;
                    }

                    Request best = null;
                    SourceBase[] sources = { Queue, GroupScan, owner.backgroundSource };
                    for(var slot = 0; slot < 3; slot++)
                    {
                        if((arbitrationPriority & (1u << (24 + slot))) == 0)
                        {
                            continue;
                        }
                        if(sources[slot].TryGetRequest(this, out var request) && (best == null || Priority(slot) > Priority(best.SourceIndex)))
                        {
                            best = request;
                        }
                    }
                    if(best == null)
                    {
                        return;
                    }
                    if(!busy)
                    {
                        Start(best);
                    }
                    else if(current.Source != best.Source && CancelInjectRepeat(best.SourceIndex) && Priority(best.SourceIndex) > Priority(current.SourceIndex))
                    {
                        AbortCurrent();
                        Start(best);
                    }
                }
                finally
                {
                    arbitrating = false;
                }
            }

            public void UpdateEmuxOutputs()
            {
                var code = (emuxControl & (1u << 28)) != 0 ? emuxActual ^ (emuxActual >> 1) : emuxActual;
                EmuxOutputCode = code;
                for(var interfaceNumber = 0; interfaceNumber < 2; interfaceNumber++)
                {
                    if(Nibble(owner.emuxSelect, interfaceNumber) != Index)
                    {
                        continue;
                    }
                    for(var bit = 0; bit < 3; bit++)
                    {
                        owner.Connections[EmuxLineBase + 3 * interfaceNumber + bit].Set((code & (1u << bit)) != 0);
                    }
                }
            }

            public void DefineRegisters()
            {
                var b = 0x400 + 0x400 * Index;

                owner.DefineRegister(b + 0x80, "ARBCFG", () =>
                    (arbitrationConfiguration & 0xB3)
                    | (Enabled ? 3u << 16 : 0u)
                    | (busy ? 1u << 30 : 0u), value =>
                {
                    arbitrationConfiguration = value & 0xB3;
                    if(!Enabled)
                    {
                        AbortCurrent();
                    }
                    owner.ArbitrateAll();
                });
                owner.DefineRegister(b + 0x84, "ARBPR", () => arbitrationPriority, value =>
                {
                    arbitrationPriority = value & 0x07000BBB;
                    owner.ArbitrateAll();
                });
                owner.DefineRegister(b + 0x88, "CHASS", () => PriorityChannels, value => PriorityChannels = value & 0xFF);
                owner.DefineStoredRegister(b + 0xA0, "ICLASS0", () => classes[0], v => classes[0] = v, ClassMask);
                owner.DefineStoredRegister(b + 0xA4, "ICLASS1", () => classes[1], v => classes[1] = v, ClassMask);
                owner.DefineStoredRegister(b + 0xB0, "ALIAS", () => alias, v => alias = v, 0x1F1F);
                owner.DefineStoredRegister(b + 0xB8, "BOUND", () => boundary, v => boundary = v, 0x0FFF0FFF);
                owner.DefineStoredRegister(b + 0xC0, "SYNCTR", () => synchronization, v => synchronization = v, 0x73);

                // The boundary flags are only stored; they are not updated by conversions
                owner.DefineRegister(b + 0xC8, "BFL", () => boundaryFlags, value => boundaryFlags = value & 0x000F0F0F);
                owner.DefineRegister(b + 0xCC, "BFLS", null, value =>
                {
                    boundaryFlags |= (value >> 16) & 0xF;
                    boundaryFlags &= ~(value & 0xF);
                });
                owner.DefineStoredRegister(b + 0xD0, "BFLC", () => boundaryFlagControl, v => boundaryFlagControl = v, 0xFFFF);
                owner.DefineStoredRegister(b + 0xD4, "BFLNP", () => boundaryFlagNodePointer, v => boundaryFlagNodePointer = v, 0xFFFF);

                Queue.DefineRegisters(b + 0x100);
                GroupScan.DefineControlRegisters(b + 0x120, b + 0x124);
                owner.DefineRegister(b + 0x128, "ASSEL", () => GroupScan.Select[0], value => GroupScan.Select[0] = value & 0xFF);
                owner.DefineRegister(b + 0x12C, "ASPND", () => GroupScan.Pending[0], value =>
                {
                    GroupScan.Select[0] = value & 0xFF;
                    GroupScan.LoadEvent();
                });

                DefineEventRegisters(b);

                owner.DefineRegister(b + 0x1C8, "SRACT", null, value =>
                {
                    for(var i = 0; i < 4; i++)
                    {
                        if((value & (1u << i)) != 0)
                        {
                            owner.Pulse(Index * 4 + i);
                        }
                        if((value & (1u << (8 + i))) != 0)
                        {
                            owner.Pulse(CommonLineBase + i);
                        }
                    }
                });

                DefineEmuxRegisters(b);

                owner.DefineRegister(b + 0x1F8, "VFR", () =>
                {
                    var flags = 0u;
                    for(var i = 0; i < ResultRegisterCount; i++)
                    {
                        flags |= Results[i].Valid ? 1u << i : 0u;
                    }
                    return flags;
                }, value =>
                {
                    for(var i = 0; i < ResultRegisterCount; i++)
                    {
                        if((value & (1u << i)) != 0)
                        {
                            Results[i].Valid = false;
                            Results[i].AccumulationLeft = 0;
                            Results[i].Drc = 0;
                        }
                    }
                    owner.ArbitrateAll();
                });

                for(var i = 0; i < ChannelsPerGroup; i++)
                {
                    var channel = i;
                    owner.DefineRegister(b + 0x200 + 4 * i, $"CHCTR{i}", () => channelControl[channel], value => channelControl[channel] = value & 0x703F0FF3);
                }
                for(var i = 0; i < ResultRegisterCount; i++)
                {
                    var result = Results[i];
                    var registerNumber = i;
                    owner.DefineRegister(b + 0x280 + 4 * i, $"RCR{i}", () => result.ControlRaw, value =>
                    {
                        result.ControlRaw = value & 0x873F0000;
                        if(((value >> 25) & 3) != 0 || ((value >> 20) & 3) != 0)
                        {
                            owner.Log(LogLevel.Warning, "Group {0}: result FIFO, filter and difference modes of RCR{1} are not modeled", Index, registerNumber);
                        }
                    });
                    // Application view: reading clears the valid flag
                    owner.DefineRegister(b + 0x300 + 4 * i, $"RES{i}", () =>
                    {
                        var composed = result.Compose(false);
                        if(result.Valid)
                        {
                            result.Valid = false;
                            owner.ArbitrateAll();
                        }
                        return composed;
                    }, value => result.Result = value & 0xFFFF);
                    owner.DefineRegister(b + 0x380 + 4 * i, $"RESD{i}", () => result.Compose(false), null);
                }
            }

            private void DefineEventRegisters(long b)
            {
                owner.DefineRegister(b + 0x180, "CEFLAG", () => channelEvents, value =>
                {
                    for(var i = 0; i < ChannelsPerGroup; i++)
                    {
                        if((value & (1u << i)) != 0)
                        {
                            RaiseChannelEvent(i);
                        }
                    }
                });
                owner.DefineRegister(b + 0x184, "REFLAG", () => resultEvents, value =>
                {
                    for(var i = 0; i < ResultRegisterCount; i++)
                    {
                        if((value & (1u << i)) != 0)
                        {
                            RaiseResultEvent(i, true);
                        }
                    }
                });
                owner.DefineRegister(b + 0x188, "SEFLAG", () => sourceEvents, value =>
                {
                    for(var i = 0; i < 2; i++)
                    {
                        if((value & (1u << i)) != 0)
                        {
                            RaiseSourceEvent(i);
                        }
                    }
                });
                owner.DefineRegister(b + 0x190, "CEFCLR", null, value => channelEvents &= ~(value & 0xFF));
                owner.DefineRegister(b + 0x194, "REFCLR", null, value => resultEvents &= ~(value & 0xFFFF));
                owner.DefineRegister(b + 0x198, "SEFCLR", null, value => sourceEvents &= ~(value & 3));
                owner.DefineStoredRegister(b + 0x1A0, "CEVNP0", () => channelEventNodePointer, v => channelEventNodePointer = v, 0xFFFFFFFF);
                owner.DefineStoredRegister(b + 0x1B0, "REVNP0", () => resultEventNodePointer0, v => resultEventNodePointer0 = v, 0xFFFFFFFF);
                owner.DefineStoredRegister(b + 0x1B4, "REVNP1", () => resultEventNodePointer1, v => resultEventNodePointer1 = v, 0xFFFFFFFF);
                owner.DefineStoredRegister(b + 0x1C0, "SEVNP", () => sourceEventNodePointer, v => sourceEventNodePointer = v, 0xFF);
            }

            private void DefineEmuxRegisters(long b)
            {
                owner.DefineRegister(b + 0x1F0, "EMUXCTR", () => (emuxControl & ~0x700u) | (emuxActual << 8), value =>
                {
                    var controlMask = (value & (1u << 31)) != 0 ? 0x7C000000u : 0u;
                    emuxControl = (emuxControl & ~(controlMask | 0x03FF0007u)) | (value & controlMask) | (value & 0x03FF0007u);
                    // The actual setting is loaded from the start selection
                    emuxActual = EmuxSetting;
                    UpdateEmuxOutputs();
                });
            }

            private uint Priority(int slot)
            {
                return (arbitrationPriority >> (4 * slot)) & 3;
            }

            private bool CancelInjectRepeat(int slot)
            {
                return (arbitrationPriority & (1u << (4 * slot + 3))) != 0;
            }

            // The channel number used for an external multiplexer conversion: the channel itself or any channel enabled in EMUXCH
            private bool EmuxApplies(int channel)
            {
                if(EmuxMode == 0)
                {
                    return false;
                }
                var selection = (emuxControl >> 16) & 0x3FF;
                return (emuxControl & (1u << 30)) != 0 ? (selection & (1u << channel)) != 0 : (selection & 0x1F) == channel;
            }

            private ResultRegister ResolveResultRegister(int channel, uint sourceResultRegister, bool background)
            {
                var control = channelControl[channel];
                if(background && (control & (1u << 20)) != 0)
                {
                    return owner.globalResult;
                }
                return Results[sourceResultRegister != 0 ? sourceResultRegister : (control >> 16) & 0xF];
            }

            private uint InputClass(uint control, bool emux, out uint sampleTime, out uint mode)
            {
                var selected = control & 3;
                var value = selected < 2 ? classes[selected] : owner.globalClass[selected - 2];
                sampleTime = emux ? (value >> 16) & 0x1F : value & 0x1F;
                mode = emux ? (value >> 24) & 7 : (value >> 8) & 7;
                return value;
            }

            private static uint ConversionBits(uint mode)
            {
                switch(mode)
                {
                case 1:
                    return 10;
                case 2:
                    return 8;
                case 5:
                    return 2;
                default:
                    return 12;
                }
            }

            private void Start(Request request)
            {
                request.EmuxActive = EmuxApplies(request.Channel);
                request.EmuxSetting = emuxActual;
                if(!request.IsContinuation)
                {
                    request.Source.Started(request);
                }
                request.IsContinuation = false;
                current = request;
                busy = true;

                // Conversion time (19.7.2): (2 + STC + N + PC) * tADCI + 2 * tADC
                var control = channelControl[request.Channel];
                InputClass(control, request.EmuxActive, out var sampleTime, out var mode);
                var stc = sampleTime < 16 ? sampleTime : 16 * (sampleTime - 15);
                var postCalibration = (owner.globalConfiguration & (1u << (16 + Index))) == 0 ? 2u : 0u;
                var divider = Math.Max((int)(owner.globalConfiguration & 0x1F), 1) + 1;
                var moduleClock = owner.ConversionClock;
                var seconds = ((2 + stc + ConversionBits(mode) + postCalibration) * divider + 2) / moduleClock;
                var microseconds = Math.Max((ulong)Math.Ceiling(seconds * 1000000), owner.minimumConversionTime);
                timer.Enabled = false;
                timer.Limit = microseconds;
                timer.ResetValue();
                timer.Enabled = true;
            }

            private void AbortCurrent()
            {
                var aborted = current;
                timer.Enabled = false;
                busy = false;
                current = null;
                continuation = null;
                aborted?.Source.Aborted(aborted);
            }

            private void OnConversionFinished()
            {
                if(!busy)
                {
                    return;
                }
                var request = current;
                owner.ConversionCount++;

                var channel = request.Channel;
                // The alias feature redirects requests for channels 0 and 1 to another input but keeps their settings
                var input = channel;
                if(channel == 0 && (alias & 0x1F) != 0)
                {
                    input = (int)(alias & 0x1F) & 7;
                }
                else if(channel == 1 && ((alias >> 8) & 0x1F) != 1)
                {
                    input = (int)((alias >> 8) & 0x1F) & 7;
                }
                var volts = request.EmuxActive ? owner.emuxVoltages[Index, request.EmuxSetting] : owner.voltages[Index, input];
                var code = (uint)Math.Max(0, Math.Min(4095, Math.Floor(volts / ReferenceVoltage * 4096)));
                StoreResult(request, code);

                var sequenceContinues = false;
                if(request.EmuxActive)
                {
                    switch(EmuxMode)
                    {
                    case 2:
                        emuxActual = emuxActual == 0 ? EmuxSetting : emuxActual - 1;
                        break;
                    case 3:
                        if(emuxActual == 0)
                        {
                            emuxActual = EmuxSetting;
                        }
                        else
                        {
                            emuxActual--;
                            sequenceContinues = true;
                        }
                        break;
                    }
                    UpdateEmuxOutputs();
                }

                busy = false;
                if(sequenceContinues)
                {
                    request.IsContinuation = true;
                    continuation = request;
                    current = request;
                }
                else
                {
                    current = null;
                    request.Source.Completed(request);
                }
                owner.ArbitrateAll();
            }

            private void StoreResult(Request request, uint code)
            {
                var channel = request.Channel;
                var control = channelControl[channel];
                var background = request.SourceIndex == 2;
                InputClass(control, request.EmuxActive, out _, out var mode);
                var register = ResolveResultRegister(channel, request.Source.ResultTarget, background);
                var global = ReferenceEquals(register, owner.globalResult);
                register.GroupNumber = (uint)Index;
                var bits = ConversionBits(mode);
                var fastCompare = mode == 5;
                var shift = (int)(12 - bits);

                // Value as it is compared with the boundaries: left aligned 12 bit
                var leftAligned = (code >> shift) << shift;
                var stored = (control & (1u << 21)) != 0 ? code >> shift : leftAligned;
                var fastCompareResult = false;
                if(fastCompare)
                {
                    // 10 bit compare value in RESULT[11:2], hysteresis from BOUND (19.7.6)
                    var reference = (register.Result >> 2) & 0x3FF;
                    var compareValue = register.FastCompare
                        ? (long)(reference << 2) - ((boundary >> 16) & 0xFFF)
                        : (long)(reference << 2) + (boundary & 0xFFF);
                    fastCompareResult = code > compareValue;
                }
                var previousFastCompare = register.FastCompare;
                var emux = request.EmuxActive && (global || ReferenceEquals(register, Results[0])) ? request.EmuxSetting : 0;
                var complete = register.Store(stored, (uint)channel, emux, (uint)request.SourceIndex, fastCompare, fastCompareResult);

                if(complete)
                {
                    if(global)
                    {
                        owner.globalResultEvent = true;
                        if(register.ServiceRequestEnable)
                        {
                            owner.RaiseGlobalServiceRequest((owner.globalEventNodePointer >> 16) & 0xF);
                        }
                    }
                    else
                    {
                        RaiseResultEvent(Array.IndexOf(Results, register), register.ServiceRequestEnable);
                    }
                    GenerateChannelEvent(channel, control, leftAligned, fastCompare, previousFastCompare, fastCompareResult);
                }
            }

            private void GenerateChannelEvent(int channel, uint control, uint leftAligned, bool fastCompare, bool previousFastCompare, bool fastCompareResult)
            {
                var eventMode = (control >> 8) & 3;
                bool generate;
                if(fastCompare)
                {
                    // Event if the result becomes high (mode 1), low (mode 2) or changes (mode 3)
                    generate = (eventMode == 1 && fastCompareResult && !previousFastCompare)
                        || (eventMode == 2 && !fastCompareResult && previousFastCompare)
                        || (eventMode == 3 && fastCompareResult != previousFastCompare);
                }
                else
                {
                    var upper = SelectBoundary((control >> 6) & 3);
                    var lower = SelectBoundary((control >> 4) & 3);
                    var inside = leftAligned >= lower && leftAligned <= upper;
                    generate = eventMode == 3 || (eventMode == 1 && inside) || (eventMode == 2 && !inside);
                }
                if(generate)
                {
                    RaiseChannelEvent(channel);
                }
            }

            private uint SelectBoundary(uint select)
            {
                switch(select)
                {
                case 0:
                    return boundary & 0xFFF;
                case 1:
                    return (boundary >> 16) & 0xFFF;
                case 2:
                    return owner.globalBoundary & 0xFFF;
                default:
                    return (owner.globalBoundary >> 16) & 0xFFF;
                }
            }

            private void RaiseChannelEvent(int channel)
            {
                channelEvents |= 1u << channel;
                owner.RaiseGroupServiceRequest(Index, Nibble(channelEventNodePointer, channel));
            }

            private void RaiseResultEvent(int register, bool raiseServiceRequest)
            {
                resultEvents |= 1u << register;
                if(raiseServiceRequest)
                {
                    var pointers = register < 8 ? resultEventNodePointer0 : resultEventNodePointer1;
                    owner.RaiseGroupServiceRequest(Index, Nibble(pointers, register & 7));
                }
            }

            private readonly XMC4XXX_VADC owner;
            private readonly LimitTimer timer;
            private readonly uint[] channelControl = new uint[ChannelsPerGroup];
            private readonly uint[] classes = new uint[2];

            private Request current;
            private Request continuation;
            private bool busy;
            private bool arbitrating;
            private uint arbitrationConfiguration;
            private uint arbitrationPriority;
            private uint alias;
            private uint boundary;
            private uint synchronization;
            private uint boundaryFlags;
            private uint boundaryFlagControl;
            private uint boundaryFlagNodePointer;
            private uint channelEvents;
            private uint resultEvents;
            private uint sourceEvents;
            private uint channelEventNodePointer;
            private uint resultEventNodePointer0;
            private uint resultEventNodePointer1;
            private uint sourceEventNodePointer;
            private uint emuxControl;
            private uint emuxActual;
        }
    }
}
