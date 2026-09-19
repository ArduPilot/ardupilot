// Reconstructs the PWM waveform on STM32 timer output pins for the sigrok
// logic analyser. Renode's STM32_Timer toggles its output-compare pins with
// the granularity of its own event scheduling, so the generator routes PWM
// pins here instead of to the GPIO fan-out: this peripheral mirrors the
// PWM-relevant registers from a bus write hook, takes each frame boundary
// from the timer model's overflow event, and schedules the edges of every
// frame analytically from PSC, ARR and CCRx. It follows the hardware rules
// that matter for ArduPilot's RCOutput: ARR and CCRx are preloaded (ARPE,
// OCxPE) and take effect at the next update event, EGR.UG forces one and
// restarts the counter (OneShot), CEN=0 freezes the counter and therefore
// holds the pin at its current level, a disabled channel (CCxE=0) idles low,
// and an advanced timer drops every output when BDTR.MOE is cleared. Timers
// running a DShot bit period are identified but not reconstructed.
//
// Every edge is produced on the emulation thread from the machine clock,
// which is exact there. The optional write log records each register write
// with that timestamp. Note that the stock timer model overflows after ARR
// ticks rather than ARR+1, so reconstructed frames follow the emulated
// firmware's real timer and are one tick shorter than PeriodUs reports.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32_Timer_Waveform : IDoubleWordPeripheral, IKnownSize,
        IAPSigrokEdgeSource
    {
        public AP_STM32_Timer_Waveform(IMachine machine, STM32_Timer timer,
            IAPSigrok analyzer, int channel1 = Unmapped, int channel2 = Unmapped,
            int channel3 = Unmapped, int channel4 = Unmapped,
            bool complementary1 = false, bool complementary2 = false,
            bool complementary3 = false, bool complementary4 = false,
            bool advanced = false, string name = "TIM")
        {
            this.machine = machine;
            this.timer = timer;
            this.analyzer = analyzer;
            this.advanced = advanced;
            this.name = name;
            channels = new int[] { channel1, channel2, channel3, channel4 };
            complementary = new bool[] {
                complementary1, complementary2, complementary3, complementary4
            };
            compare = new uint[ChannelCount];
            pendingCompare = new uint[ChannelCount];
            hasPendingCompare = new bool[ChannelCount];
            mode = new uint[ChannelCount];
            preloadCompare = new bool[ChannelCount];
            outputEnable = new bool[ChannelCount];
            polarity = new bool[ChannelCount];
            ResetState();
            analyzer.RegisterEdgeSource(this);
            timer.LimitReached += HandleOverflow;
            // Platform entries are all constructed before any is registered,
            // so the timer may not be on the bus yet; retry as peripherals
            // are added.
            TryInstallHook();
            if(!hooked)
            {
                machine.PeripheralsChanged += OnPeripheralsChanged;
            }
        }

        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public long Size => 4;

        public void Reset()
        {
            lock(analyzer.CaptureLock)
            {
                var now = CurrentTimeNs();
                ResetState();
                Reschedule(now);
            }
        }

        // Log every register write with its timestamp. Enable with
        // "logLevel 1 sysbus.timerNWaveform" so the Info lines are shown.
        public bool LogWrites { get; set; }

        // Reconstructed frame period in microseconds (0 when not running).
        public double PeriodUs
        {
            get
            {
                lock(analyzer.CaptureLock)
                {
                    return running ? periodNs / 1000.0 : 0.0;
                }
            }
        }

        // Reconstructed pulse width in microseconds for timer channel 1..4.
        public double PulseWidthUs(int channel)
        {
            if(channel < 1 || channel > ChannelCount)
            {
                throw new ArgumentOutOfRangeException(nameof(channel));
            }
            lock(analyzer.CaptureLock)
            {
                var index = channel - 1;
                if(!running || !OutputEnabled(index))
                {
                    return 0.0;
                }
                var ticks = mode[index] == PwmMode2 ?
                    (double)Math.Max(0L, (long)autoReload + 1 - compare[index]) :
                    (double)Math.Min((ulong)compare[index], autoReload + 1UL);
                return ticks * tickNs / 1000.0;
            }
        }

        // A capture begins: publish the current level of every pin and the
        // remaining edge of the frame in progress.
        public void Restart(long nowNs)
        {
            Reschedule(Math.Max(nowNs, lastEventNs));
        }

        // Edges are produced by the timer's own events, not by the sampler.
        public void Extend(long throughNs)
        {
        }

        private void OnPeripheralsChanged(IMachine changedMachine,
            PeripheralsChangedEventArgs args)
        {
            TryInstallHook();
        }

        private void TryInstallHook()
        {
            if(hooked)
            {
                return;
            }
            try
            {
                machine.SystemBus.SetHookBeforePeripheralWrite<uint>(timer,
                    (value, offset) =>
                    {
                        HandleWrite(offset, value);
                        return value;
                    });
            }
            catch(RecoverableException)
            {
                return;
            }
            hooked = true;
            machine.PeripheralsChanged -= OnPeripheralsChanged;
        }

        // The counter wrapped: a new frame starts and preloads take effect.
        private void HandleOverflow()
        {
            lock(analyzer.CaptureLock)
            {
                if(!running)
                {
                    return;
                }
                var now = CurrentTimeNs();
                periodStartNs = now;
                started = true;
                ApplyPending();
                ScheduleFrame(now);
            }
        }

        private void HandleWrite(long offset, uint value)
        {
            lock(analyzer.CaptureLock)
            {
                var now = CurrentTimeNs();
                if(LogWrites)
                {
                    this.Log(LogLevel.Info, "{0} {1} <- 0x{2:X8} at {3:F1} us",
                        name, RegisterName(offset), value, now / 1000.0);
                }
                switch(offset)
                {
                case Control1:
                    SetRunning((value & CounterEnable) != 0, now);
                    autoReloadPreload = (value & AutoReloadPreload) != 0;
                    break;
                case EventGeneration:
                    if((value & UpdateGeneration) == 0)
                    {
                        return;
                    }
                    periodStartNs = now;
                    frozenNs = now;
                    started = true;
                    ApplyPending();
                    break;
                case CaptureCompareMode1:
                    SetMode(0, value);
                    SetMode(1, value >> 8);
                    break;
                case CaptureCompareMode2:
                    SetMode(2, value);
                    SetMode(3, value >> 8);
                    break;
                case CaptureCompareEnable:
                    for(var channel = 0; channel < ChannelCount; channel++)
                    {
                        // a pin declared TIMx_CHyN is driven from CCxNE and
                        // CCxNP, two bits up the nibble; ChibiOS leaves CCxE
                        // clear for such a channel, so reading only CCxE
                        // would hold the pin low for ever.  With CCxE clear,
                        // CCxNE set and MOE set the pin follows OCxREF xor
                        // CCxNP, so no extra inversion is needed here.
                        var shift = channel * 4 + (complementary[channel] ? 2 : 0);
                        outputEnable[channel] = (value & (1U << shift)) != 0;
                        polarity[channel] = (value & (2U << shift)) != 0;
                    }
                    break;
                case Counter:
                    periodStartNs = now - value * tickNs;
                    break;
                case Prescaler:
                    pendingPrescaler = value;
                    hasPendingPrescaler = true;
                    return;
                case AutoReload:
                    if(autoReloadPreload)
                    {
                        pendingAutoReload = value;
                        hasPendingAutoReload = true;
                        return;
                    }
                    autoReload = value;
                    RecomputePeriod();
                    break;
                case CaptureCompare1:
                case CaptureCompare2:
                case CaptureCompare3:
                case CaptureCompare4:
                    var index = (int)((offset - CaptureCompare1) / 4);
                    if(preloadCompare[index])
                    {
                        pendingCompare[index] = value;
                        hasPendingCompare[index] = true;
                        return;
                    }
                    compare[index] = value;
                    break;
                case BreakDeadTime:
                    mainOutputEnable = (value & MainOutputEnable) != 0;
                    break;
                default:
                    return;
                }
                Reschedule(now);
            }
        }

        private void SetMode(int channel, uint bits)
        {
            mode[channel] = (bits >> 4) & 0x7;
            preloadCompare[channel] = (bits & OutputComparePreload) != 0;
        }

        private void SetRunning(bool enable, long now)
        {
            if(enable == running)
            {
                return;
            }
            if(enable)
            {
                if(!started)
                {
                    periodStartNs = now;
                    started = true;
                }
                else
                {
                    // resume from the frozen counter
                    periodStartNs += now - frozenNs;
                }
            }
            else
            {
                frozenNs = now;
            }
            running = enable;
        }

        private void ApplyPending()
        {
            if(hasPendingPrescaler)
            {
                prescaler = pendingPrescaler;
                hasPendingPrescaler = false;
            }
            if(hasPendingAutoReload)
            {
                autoReload = pendingAutoReload;
                hasPendingAutoReload = false;
            }
            for(var channel = 0; channel < ChannelCount; channel++)
            {
                if(hasPendingCompare[channel])
                {
                    compare[channel] = pendingCompare[channel];
                    hasPendingCompare[channel] = false;
                }
            }
            RecomputePeriod();
        }

        private void RecomputePeriod()
        {
            tickNs = timer.Frequency == 0 ? 0.0 :
                (prescaler + 1.0) * NanosecondsPerSecond / timer.Frequency;
            periodNs = (autoReload + 1.0) * tickNs;
        }

        // A frame starts at now: every pin takes its counter-zero level and
        // the compare match later in the frame is scheduled ahead of time.
        private void ScheduleFrame(long now)
        {
            if(!analyzer.Capturing || periodNs < MinimumPeriodNs)
            {
                return;
            }
            for(var channel = 0; channel < ChannelCount; channel++)
            {
                if(channels[channel] == Unmapped)
                {
                    continue;
                }
                analyzer.AddEdge(channels[channel], Level(channel, 0.0), now);
                ScheduleMatch(channel, now);
            }
        }

        // Timer state changed at now: drop what was scheduled beyond now,
        // publish the level every pin has now, and re-schedule the rest of
        // the frame in progress from the new state.
        // Tell the analyser which pads this timer is actually driving.  Done
        // whether or not a capture is running, because the GPIO fan-out
        // updates idle levels too.
        private void UpdateOwnership()
        {
            for(var channel = 0; channel < ChannelCount; channel++)
            {
                if(channels[channel] == Unmapped)
                {
                    continue;
                }
                analyzer.SetAnalyticOwned(channels[channel], OutputEnabled(channel));
            }
        }

        private void Reschedule(long now)
        {
            UpdateOwnership();
            if(!analyzer.Capturing)
            {
                return;
            }
            for(var channel = 0; channel < ChannelCount; channel++)
            {
                if(channels[channel] == Unmapped)
                {
                    continue;
                }
                if(!OutputEnabled(channel))
                {
                    // the timer is not driving this pad, so firmware may be
                    // using it as a GPIO; leave the channel to the GPIO
                    // fan-out rather than holding it low
                    continue;
                }
                analyzer.CancelEdges(channels[channel], now);
                analyzer.AddEdge(channels[channel], CurrentLevel(channel, now), now);
                if(running && periodNs >= MinimumPeriodNs)
                {
                    ScheduleMatch(channel, now);
                }
            }
        }

        private void ScheduleMatch(int channel, long now)
        {
            var ticks = compare[channel];
            if(ticks == 0 || ticks > autoReload)
            {
                return;
            }
            var at = periodStartNs + ticks * tickNs;
            if(at > now && at < periodStartNs + periodNs)
            {
                analyzer.AddEdge(channels[channel], Level(channel, ticks), (long)at);
            }
        }

        private bool CurrentLevel(int channel, long now)
        {
            if(!started || tickNs <= 0.0)
            {
                return false;
            }
            var reference = running ? now : frozenNs;
            var ticks = (reference - periodStartNs) / tickNs;
            if(periodNs > 0.0)
            {
                ticks -= Math.Floor(ticks / (autoReload + 1.0)) * (autoReload + 1.0);
            }
            return Level(channel, ticks);
        }

        private bool Level(int channel, double ticks)
        {
            if(!OutputEnabled(channel))
            {
                return false;
            }
            bool active;
            switch(mode[channel])
            {
            case PwmMode1:
                active = ticks < compare[channel];
                break;
            case PwmMode2:
                active = ticks >= compare[channel];
                break;
            case ForceInactive:
                active = false;
                break;
            case ForceActive:
                active = true;
                break;
            default:
                return false;
            }
            return active ^ polarity[channel];
        }

        private bool OutputEnabled(int channel)
        {
            return outputEnable[channel] && (!advanced || mainOutputEnable);
        }

        // Emulation-thread machine time, kept monotonic.
        private long CurrentTimeNs()
        {
            var now = analyzer.NowNs;
            if(now < lastEventNs)
            {
                now = lastEventNs;
            }
            lastEventNs = now;
            return now;
        }

        private void ResetState()
        {
            running = false;
            started = false;
            autoReloadPreload = false;
            mainOutputEnable = false;
            prescaler = 0;
            autoReload = 0;
            hasPendingPrescaler = false;
            hasPendingAutoReload = false;
            Array.Clear(compare, 0, compare.Length);
            Array.Clear(hasPendingCompare, 0, hasPendingCompare.Length);
            Array.Clear(mode, 0, mode.Length);
            Array.Clear(preloadCompare, 0, preloadCompare.Length);
            Array.Clear(outputEnable, 0, outputEnable.Length);
            Array.Clear(polarity, 0, polarity.Length);
            periodStartNs = 0.0;
            frozenNs = 0;
            RecomputePeriod();
        }

        private static string RegisterName(long offset)
        {
            switch(offset)
            {
            case Control1: return "CR1";
            case 0x04: return "CR2";
            case 0x08: return "SMCR";
            case 0x0C: return "DIER";
            case 0x10: return "SR";
            case EventGeneration: return "EGR";
            case CaptureCompareMode1: return "CCMR1";
            case CaptureCompareMode2: return "CCMR2";
            case CaptureCompareEnable: return "CCER";
            case Counter: return "CNT";
            case Prescaler: return "PSC";
            case AutoReload: return "ARR";
            case 0x30: return "RCR";
            case CaptureCompare1: return "CCR1";
            case CaptureCompare2: return "CCR2";
            case CaptureCompare3: return "CCR3";
            case CaptureCompare4: return "CCR4";
            case BreakDeadTime: return "BDTR";
            case 0x48: return "DCR";
            case 0x4C: return "DMAR";
            default: return String.Format("0x{0:X2}", offset);
            }
        }

        private readonly IMachine machine;
        private readonly STM32_Timer timer;
        private readonly IAPSigrok analyzer;
        private readonly bool advanced;
        private readonly string name;
        private readonly int[] channels;
        private readonly bool[] complementary;
        private readonly uint[] compare;
        private readonly uint[] pendingCompare;
        private readonly bool[] hasPendingCompare;
        private readonly uint[] mode;
        private readonly bool[] preloadCompare;
        private readonly bool[] outputEnable;
        private readonly bool[] polarity;

        private bool hooked;
        private bool running;
        private bool started;
        private bool autoReloadPreload;
        private bool mainOutputEnable;
        private uint prescaler;
        private uint pendingPrescaler;
        private bool hasPendingPrescaler;
        private uint autoReload;
        private uint pendingAutoReload;
        private bool hasPendingAutoReload;
        private double tickNs;
        private double periodNs;
        private double periodStartNs;
        private long frozenNs;
        private long lastEventNs;

        private const int Unmapped = -1;
        private const int ChannelCount = 4;
        private const long Control1 = 0x00;
        private const long EventGeneration = 0x14;
        private const long CaptureCompareMode1 = 0x18;
        private const long CaptureCompareMode2 = 0x1C;
        private const long CaptureCompareEnable = 0x20;
        private const long Counter = 0x24;
        private const long Prescaler = 0x28;
        private const long AutoReload = 0x2C;
        private const long CaptureCompare1 = 0x34;
        private const long CaptureCompare2 = 0x38;
        private const long CaptureCompare3 = 0x3C;
        private const long CaptureCompare4 = 0x40;
        private const long BreakDeadTime = 0x44;
        private const uint CounterEnable = 1U << 0;
        private const uint AutoReloadPreload = 1U << 7;
        private const uint UpdateGeneration = 1U << 0;
        private const uint OutputComparePreload = 1U << 3;
        private const uint MainOutputEnable = 1U << 15;
        private const uint ForceInactive = 4;
        private const uint ForceActive = 5;
        private const uint PwmMode1 = 6;
        private const uint PwmMode2 = 7;
        private const double NanosecondsPerSecond = 1000000000.0;
        // Below this frame period the timer carries a DShot or serial bit
        // stream whose CCR changes by DMA; that is not a servo waveform.
        private const double MinimumPeriodNs = 20000.0;
    }
}
