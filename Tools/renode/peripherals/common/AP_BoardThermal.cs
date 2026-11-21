// IMU heater and board temperature.
//
// Boards with an IMU heater regulate the IMU temperature through a heater
// duty cycle, set either on an IOMCU register or as a randomised GPIO pattern.
// AP_BoardThermalState turns that duty into a first-order IMU temperature for
// the emulated IMUs, and exposes the duty so a compass model can add the
// heater current's magnetic interference that the firmware compensates for.
//
using System;
using System.Runtime.CompilerServices;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public sealed class AP_BoardThermalState
    {
        public static AP_BoardThermalState ForMachine(IMachine machine)
        {
            return states.GetValue(machine, key => new AP_BoardThermalState(key));
        }

        public static AP_BoardThermalState ForMachine(IMachine machine, string name)
        {
            if(string.IsNullOrEmpty(name))
            {
                return ForMachine(machine);
            }
            if(!EmulationManager.Instance.CurrentEmulation.TryGetMachine(name, out var target))
            {
                throw new RecoverableException(string.Format(
                    "No machine named {0}", name));
            }
            return ForMachine(target);
        }

        // percent; the IOMCU drops a duty not refreshed for 3s, as here
        public double HeaterDuty
        {
            get
            {
                lock(sync)
                {
                    return NowSeconds - dutySetSeconds > DutyTimeoutSeconds ? 0.0 : duty;
                }
            }
        }

        public void SetHeaterDuty(double percent)
        {
            lock(sync)
            {
                Integrate();
                duty = Math.Max(0.0, Math.Min(100.0, percent));
                dutySetSeconds = NowSeconds;
            }
        }

        public double TemperatureC
        {
            get
            {
                lock(sync)
                {
                    Integrate();
                    return temperatureC;
                }
            }
        }

        private AP_BoardThermalState(IMachine machine)
        {
            this.machine = machine;
            physics = AP_PhysicsState.ForMachine(machine);
            temperatureC = AmbientC;
            lastSeconds = NowSeconds;
            dutySetSeconds = double.NegativeInfinity;
        }

        private void Integrate()
        {
            var now = NowSeconds;
            var dt = now - lastSeconds;
            lastSeconds = now;
            if(dt <= 0.0)
            {
                return;
            }
            var currentDuty = now - dutySetSeconds > DutyTimeoutSeconds ? 0.0 : duty;
            var target = AmbientC + HeaterRiseC * currentDuty * 0.01;
            temperatureC += (target - temperatureC) * (1.0 - Math.Exp(-dt / TimeConstantSeconds));
        }

        // electronics run warmer than the surrounding air
        private double AmbientC => physics.Current.TemperatureK - 273.15 + SelfHeatingC;
        private double NowSeconds => machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds;

        private readonly IMachine machine;
        private readonly AP_PhysicsState physics;
        private readonly object sync = new object();
        private double temperatureC;
        private double lastSeconds;
        private double duty;
        private double dutySetSeconds;

        private const double SelfHeatingC = 10.0;
        private const double HeaterRiseC = 40.0;
        private const double TimeConstantSeconds = 40.0;
        private const double DutyTimeoutSeconds = 3.0;
        private static readonly ConditionalWeakTable<IMachine, AP_BoardThermalState> states =
            new ConditionalWeakTable<IMachine, AP_BoardThermalState>();
    }

    // Measures the duty cycle of a heater GPIO, possibly on a coprocessor.
    public class AP_HeaterMonitor : IGPIOReceiver, IDoubleWordPeripheral, IKnownSize
    {
        public AP_HeaterMonitor(IMachine machine, bool activeHigh = true,
            string boardMachine = null)
        {
            this.machine = machine;
            this.activeHigh = activeHigh;
            state = AP_BoardThermalState.ForMachine(machine, boardMachine);
            Reset();
            // a pin held on or off still closes each averaging window
            ticker = machine.ObtainManagedThread(Tick, TicksPerSecond,
                name: "AP heater monitor", owner: this);
            ticker.Start();
        }

        private void Tick()
        {
            lock(sync)
            {
                Accumulate();
            }
        }

        public void OnGPIO(int number, bool value)
        {
            lock(sync)
            {
                Accumulate();
                on = value == activeHigh;
            }
        }

        public void Reset()
        {
            lock(sync)
            {
                on = false;
                windowStart = lastChange = NowSeconds;
                onSeconds = 0.0;
            }
        }

        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public long Size => 4;

        // the firmware toggles the pin on a random pattern, so average it
        private void Accumulate()
        {
            var now = NowSeconds;
            if(on)
            {
                onSeconds += now - lastChange;
            }
            lastChange = now;
            if(now - windowStart >= WindowSeconds)
            {
                state.SetHeaterDuty(100.0 * onSeconds / (now - windowStart));
                windowStart = now;
                onSeconds = 0.0;
            }
        }

        private double NowSeconds => machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds;

        private readonly IMachine machine;
        private readonly bool activeHigh;
        private readonly AP_BoardThermalState state;
        private readonly IManagedThread ticker;
        private readonly object sync = new object();
        private bool on;
        private double windowStart;
        private double lastChange;
        private double onSeconds;

        private const double WindowSeconds = 0.5;
        private const uint TicksPerSecond = 10;
    }
}
