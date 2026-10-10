//
// RP2350 TIMER0/TIMER1: 64-bit 1 MHz counter with four 32-bit alarms.
//
// The counter is virtual time in microseconds plus an offset set by writes to
// TIMEHW/TIMELW. An alarm fires when the low 32 bits of the counter equal
// ALARMn, which clears its ARMED bit and latches INTR. The ChibiOS system
// tick and ArduPilot's micros() both run off TIMER0.
//
// Every RP2350 interrupt reaches both cores, so each alarm drives two GPIO
// outputs: n for core0's NVIC and 4+n for core1's. A core only takes the
// interrupt if it has enabled it in its own NVIC.
//
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_RP2350_Timer : IDoubleWordPeripheral, IKnownSize, INumberedGPIOOutput
    {
        public AP_RP2350_Timer(IMachine machine)
        {
            this.machine = machine;
            var outputs = new Dictionary<int, IGPIO>();
            for(var i = 0; i < 2 * NumAlarms; i++)
            {
                outputs[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(outputs);
            for(var i = 0; i < NumAlarms; i++)
            {
                var n = i;
                alarmTimers[i] = new LimitTimer(machine.ClockSource, 1000000, this, "alarm" + i,
                    limit: 1, direction: Direction.Ascending, enabled: false,
                    workMode: WorkMode.OneShot, eventEnabled: true);
                alarmTimers[i].LimitReached += () => Fire(n);
            }
            Reset();
        }

        public long Size => 0x4000;

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public void Reset()
        {
            offset = 0;
            latchedHigh = 0;
            armed = 0;
            intr = 0;
            inte = 0;
            intf = 0;
            for(var i = 0; i < NumAlarms; i++)
            {
                alarms[i] = 0;
                alarmTimers[i].Enabled = false;
            }
            UpdateInterrupts();
        }

        public uint ReadDoubleWord(long address)
        {
            var reg = address & 0xFFC;
            var now = Now;
            switch(reg)
            {
                case TimeLR:
                    latchedHigh = (uint)(now >> 32);
                    return (uint)now;
                case TimeHR:
                    return latchedHigh;
                case TimeRawL:
                    return (uint)now;
                case TimeRawH:
                    return (uint)(now >> 32);
                case Armed:
                    return armed;
                case Intr:
                    return intr;
                case Inte:
                    return inte;
                case Intf:
                    return intf;
                case Ints:
                    return Status;
                default:
                    if(reg >= Alarm0 && reg < Alarm0 + 4 * NumAlarms)
                    {
                        return alarms[(reg - Alarm0) >> 2];
                    }
                    return 0;
            }
        }

        public void WriteDoubleWord(long address, uint value)
        {
            var reg = address & 0xFFC;
            var alias = address >> 12;
            // INTE/INTF are ordinary read-write registers and honour the
            // atomic aliases; the rest are write-to-act.
            if(reg == Inte || reg == Intf)
            {
                var current = reg == Inte ? inte : intf;
                switch(alias)
                {
                    case 1: value = current ^ value; break;
                    case 2: value = current | value; break;
                    case 3: value = current & ~value; break;
                }
                if(reg == Inte)
                {
                    inte = value & AlarmMask;
                }
                else
                {
                    intf = value & AlarmMask;
                }
                UpdateInterrupts();
                return;
            }
            if(alias == 3)
            {
                // clearing bits of write-to-act registers does nothing
                return;
            }
            switch(reg)
            {
                case TimeLW:
                    pendingLow = value;
                    break;
                case TimeHW:
                    offset = (((ulong)value << 32) | pendingLow) - RawMicros;
                    for(var i = 0; i < NumAlarms; i++)
                    {
                        if((armed & (1u << i)) != 0)
                        {
                            Schedule(i);
                        }
                    }
                    break;
                case Armed:
                    // write 1 to disarm
                    for(var i = 0; i < NumAlarms; i++)
                    {
                        if((value & (1u << i)) != 0)
                        {
                            armed &= ~(1u << i);
                            alarmTimers[i].Enabled = false;
                        }
                    }
                    break;
                case Intr:
                    intr &= ~(value & AlarmMask);
                    UpdateInterrupts();
                    break;
                default:
                    if(reg >= Alarm0 && reg < Alarm0 + 4 * NumAlarms)
                    {
                        var n = (int)((reg - Alarm0) >> 2);
                        alarms[n] = value;
                        armed |= 1u << n;
                        Schedule(n);
                    }
                    break;
            }
        }

        private void Schedule(int n)
        {
            var timer = alarmTimers[n];
            timer.Enabled = false;
            var delta = alarms[n] - (uint)Now;
            if(delta == 0)
            {
                Fire(n);
                return;
            }
            timer.Value = 0;
            timer.Limit = delta;
            timer.Enabled = true;
        }

        private void Fire(int n)
        {
            alarmTimers[n].Enabled = false;
            if((armed & (1u << n)) == 0)
            {
                return;
            }
            armed &= ~(1u << n);
            intr |= 1u << n;
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            var status = Status;
            for(var i = 0; i < NumAlarms; i++)
            {
                var level = (status & (1u << i)) != 0;
                Connections[i].Set(level);
                Connections[NumAlarms + i].Set(level);
            }
        }

        private uint Status => (intr | intf) & inte;

        private ulong RawMicros => (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;

        private ulong Now => RawMicros + offset;

        private readonly IMachine machine;
        private readonly LimitTimer[] alarmTimers = new LimitTimer[NumAlarms];
        private readonly uint[] alarms = new uint[NumAlarms];
        private ulong offset;
        private uint latchedHigh;
        private uint pendingLow;
        private uint armed;
        private uint intr;
        private uint inte;
        private uint intf;

        private const int NumAlarms = 4;
        private const uint AlarmMask = 0xF;
        private const long TimeHW = 0x00;
        private const long TimeLW = 0x04;
        private const long TimeHR = 0x08;
        private const long TimeLR = 0x0C;
        private const long Alarm0 = 0x10;
        private const long Armed = 0x20;
        private const long TimeRawH = 0x24;
        private const long TimeRawL = 0x28;
        private const long Intr = 0x3C;
        private const long Inte = 0x40;
        private const long Intf = 0x44;
        private const long Ints = 0x48;
    }
}
