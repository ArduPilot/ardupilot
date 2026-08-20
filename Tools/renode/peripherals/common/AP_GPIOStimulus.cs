// External GPIO stimulus for ArduPilot RPM and wheel-encoder inputs. Signals
// are scheduled only while a generator is active, so unused GPIOs add no work
// to Renode's clock hot path.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_GPIOStimulus : IDoubleWordPeripheral, IKnownSize,
                                   INumberedGPIOOutput
    {
        public AP_GPIOStimulus(IMachine machine)
        {
            this.machine = machine;
            versions = new uint[NumberOfGPIOs];
            var connections = new Dictionary<int, IGPIO>();
            for(var gpio = 0; gpio < NumberOfGPIOs; gpio++)
            {
                connections[gpio] = new GPIO();
            }
            Connections = connections;
        }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public long Size => 4;

        public uint ReadDoubleWord(long offset)
        {
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public void Reset()
        {
            var version = NewVersion();
            for(var gpio = 0; gpio < NumberOfGPIOs; gpio++)
            {
                versions[gpio] = version;
                Connections[gpio].Unset();
            }
        }

        public void Set(int gpio, bool value)
        {
            CheckGPIO(gpio);
            versions[gpio] = NewVersion();
            Connections[gpio].Set(value);
        }

        public void Stop(int gpio)
        {
            Set(gpio, false);
        }

        public void StopAll()
        {
            Reset();
        }

        public void StartPulse(int gpio, uint frequencyHz)
        {
            CheckGPIO(gpio);
            var interval = Interval(frequencyHz, 2);
            var version = NewVersion();
            versions[gpio] = version;
            Connections[gpio].Unset();
            SchedulePulse(gpio, version, interval);
        }

        public void StartQuadrature(int gpioA, int gpioB,
                                    uint cyclesPerSecond, bool reverse = false)
        {
            CheckGPIO(gpioA);
            CheckGPIO(gpioB);
            if(gpioA == gpioB)
            {
                throw new RecoverableException(
                    "quadrature inputs must use different GPIOs");
            }
            var interval = Interval(cyclesPerSecond, 4);
            var version = NewVersion();
            versions[gpioA] = version;
            versions[gpioB] = version;
            Connections[gpioA].Unset();
            Connections[gpioB].Unset();
            ScheduleQuadrature(gpioA, gpioB, version, interval, reverse, 0);
        }

        private void SchedulePulse(int gpio, uint version,
                                   TimeInterval interval)
        {
            machine.ScheduleAction(interval, _ =>
            {
                if(versions[gpio] != version)
                {
                    return;
                }
                Connections[gpio].Toggle();
                SchedulePulse(gpio, version, interval);
            }, "ArduPilot GPIO pulse");
        }

        private void ScheduleQuadrature(int gpioA, int gpioB, uint version,
                                        TimeInterval interval, bool reverse,
                                        int phase)
        {
            machine.ScheduleAction(interval, _ =>
            {
                if(versions[gpioA] != version ||
                   versions[gpioB] != version)
                {
                    return;
                }
                var nextPhase = (phase + (reverse ? 3 : 1)) & 3;
                Connections[gpioA].Set(
                    nextPhase == 1 || nextPhase == 2);
                Connections[gpioB].Set(nextPhase >= 2);
                ScheduleQuadrature(gpioA, gpioB, version, interval,
                    reverse, nextPhase);
            }, "ArduPilot GPIO quadrature");
        }

        private static TimeInterval Interval(uint frequency, uint edges)
        {
            if(frequency == 0)
            {
                throw new RecoverableException("GPIO frequency must be non-zero");
            }
            var ticks = TimeInterval.TicksPerSecond / frequency / edges;
            if(ticks == 0)
            {
                throw new RecoverableException("GPIO frequency is too high");
            }
            return TimeInterval.FromTicks(ticks);
        }

        private uint NewVersion()
        {
            nextVersion++;
            if(nextVersion == 0)
            {
                nextVersion++;
            }
            return nextVersion;
        }

        private static void CheckGPIO(int gpio)
        {
            if(gpio < 0 || gpio >= NumberOfGPIOs)
            {
                throw new RecoverableException(String.Format(
                    "GPIO must be 0..{0}", NumberOfGPIOs - 1));
            }
        }

        private readonly IMachine machine;
        private readonly uint[] versions;
        private uint nextVersion;

        private const int NumberOfGPIOs = 256;
    }
}
