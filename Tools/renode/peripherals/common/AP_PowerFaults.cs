//
// Power and reset fault injection shared by the STM32 RCC, PWR and IWDG
// models.
//
// AP_ResetCause lets the peripheral that resets a machine tell the RCC
// why, so the reset flags the firmware reads match the cause. Renode
// resets every peripheral the same way, so without this a watchdog reset
// looks like any other.
//
// AP_VoltageDetector models the programmable voltage detector (PVD): a
// comparator between the supply voltage and a selectable threshold,
// with hysteresis. Its output is the PVDO status bit and drives the
// PVD EXTI line. The supply voltage is set from the monitor, either
// directly or as a timed dip. The comparator thresholds come from the
// datasheets and haven't been checked on hardware: the IOMCU and FMU
// share the 3.3V rail, so a dip can't be applied to one MCU alone.
//
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public static class AP_ResetCause
    {
        public enum Cause
        {
            // a software (SYSRESETREQ) or monitor-requested reset
            Other,
            PowerOn,
            IndependentWatchdog,
        }

        // record why the next reset of this machine happens
        public static void Note(IMachine machine, Cause cause)
        {
            causes.GetValue(machine, _ => new Box()).Value = cause;
        }

        // the cause of the reset in progress, which is then forgotten
        public static Cause Take(IMachine machine)
        {
            var box = causes.GetValue(machine, _ => new Box());
            var cause = box.Value;
            box.Value = Cause.Other;
            return cause;
        }

        private class Box
        {
            public Cause Value;
        }

        private static readonly ConditionalWeakTable<IMachine, Box> causes =
            new ConditionalWeakTable<IMachine, Box>();
    }

    public class AP_VoltageDetector
    {
        // fallingThresholds[level] is the typical falling-edge threshold
        // for PLS=level from the datasheet; rising adds the hysteresis
        public AP_VoltageDetector(IMachine machine, IPeripheral owner,
            double[] fallingThresholds, double hysteresis = 0.1)
        {
            this.fallingThresholds = fallingThresholds;
            this.hysteresis = hysteresis;
            dipTimer = new LimitTimer(machine.ClockSource, 1000000, owner,
                "pvd dip", limit: 1, direction: Direction.Ascending,
                enabled: false, workMode: WorkMode.OneShot, eventEnabled: true);
            dipTimer.LimitReached += EndDip;
        }

        public GPIO Output { get; } = new GPIO();

        // supply voltage in volts
        public double Vdd
        {
            get { lock(sync) { return vdd; } }
            set { lock(sync) { vdd = value; Update(); } }
        }

        // PVDO: the supply is below the threshold
        public bool Below
        {
            get { lock(sync) { return below; } }
        }

        // number of times the comparator output has gone high
        public ulong Trips
        {
            get { lock(sync) { return trips; } }
        }

        public void Configure(bool enable, int level)
        {
            lock(sync)
            {
                if(enable && (!enabled || level != this.level))
                {
                    // a newly enabled comparator has no previous state
                    // for the hysteresis to hold
                    below = false;
                }
                enabled = enable;
                this.level = level;
                Update();
            }
        }

        // drop the supply to volts for milliseconds of virtual time
        public void Dip(double volts, double milliseconds)
        {
            lock(sync)
            {
                if(!dipping)
                {
                    restoreVdd = vdd;
                }
                dipping = true;
                vdd = volts;
                Update();
                dipTimer.Enabled = false;
                dipTimer.Value = 0;
                dipTimer.Limit = Math.Max(1UL, (ulong)Math.Round(milliseconds * 1000));
                dipTimer.Enabled = true;
            }
        }

        public void Reset()
        {
            lock(sync)
            {
                // the supply and any dip in progress are outside the MCU,
                // so a reset only turns the comparator off
                enabled = false;
                below = false;
                Update();
            }
        }

        private void EndDip()
        {
            lock(sync)
            {
                dipTimer.Enabled = false;
                dipping = false;
                vdd = restoreVdd;
                Update();
            }
        }

        private void Update()
        {
            var wasBelow = below;
            if(!enabled || level < 0 || level >= fallingThresholds.Length)
            {
                below = false;
            }
            else if(vdd < fallingThresholds[level])
            {
                below = true;
            }
            else if(vdd > fallingThresholds[level] + hysteresis)
            {
                below = false;
            }
            if(below && !wasBelow)
            {
                trips++;
            }
            Output.Set(below);
        }

        private readonly object sync = new object();
        private readonly double[] fallingThresholds;
        private readonly double hysteresis;
        private readonly LimitTimer dipTimer;
        private double vdd = 3.3;
        private double restoreVdd = 3.3;
        private bool dipping;
        private bool enabled;
        private int level;
        private bool below;
        private ulong trips;
    }
}
