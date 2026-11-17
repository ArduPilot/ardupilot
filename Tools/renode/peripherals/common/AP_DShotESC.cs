//
// Lightweight AM32-style bidirectional DShot ESC for the Renode DShot bridge.
//
// Follows AM32's signal handling (Src/dshot.c, Src/main.c) closely enough
// for the flight controller side:
// - bidirectional mode is detected from the idle-high line while disarmed
//   and kept until reset; it decides which CRC is valid and whether to reply
// - arming needs a second of zero input and more than 30 zero frames;
//   commands count as zero input, throttle restarts the wait
// - commands are accepted while armed and stopped, repeated six times
//   (beacons at once): 7/8 set the configured direction, 9/10 select 3D
//   (bidirectional throttle), 12 saves, 13/14 switch EDT, 20/21 spin normal
//   or reversed relative to the configured direction
// - losing signal for 0.5s armed or 2s disarmed resets the ESC, dropping
//   unsaved settings, as AM32's NVIC_SystemReset does
// - EDT current/voltage/temperature frames are interleaved with eRPM frames
//   at AM32's rates, with AM32's init/deinit status frames
// The motor is a first-order model of a KV-rated motor.
using System;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public sealed class AP_DShotESC
    {
        public AP_DShotESC(int poles, double kv, double batteryVoltage,
            double maxCurrent)
        {
            this.poles = Math.Max(2, poles);
            this.kv = kv;
            this.batteryVoltage = batteryVoltage;
            this.maxCurrent = maxCurrent;
            PowerOn();
        }

        // power cycle: settings revert to the saved ones
        public void PowerOn()
        {
            lock(sync)
            {
                Restart();
                mechanicalRpm = 0;
                temperatureC = AmbientC;
                lastUpdateS = -1;
            }
        }

        public string State(double nowS)
        {
            lock(sync)
            {
                CheckSignal(nowS);
                return string.Format(
                    "armed={0} bidir={1} edt={2} 3d={3} reversed={4} forward={5} rpm={6:F0} V={7:F2} A={8:F1} T={9:F1}",
                    armed, bidirectional, extendedTelemetry, biDirection, dirReversed, forward,
                    mechanicalRpm, Voltage, Current, temperatureC);
            }
        }

        // Handle one frame at nowS; returns the 16 bit telemetry frame to send
        // back (12 bit value and inverted CRC, before GCR), or null for none.
        public ushort? Frame(ushort frame, bool idleHigh, double nowS)
        {
            lock(sync)
            {
                CheckSignal(nowS);
                // any frame with valid timing counts as signal, even a bad CRC
                lastSignalS = nowS;
                Update(nowS);
                if(!armed && idleHigh)
                {
                    bidirectional = true;
                }
                var value = frame >> 4;
                var crc = (value ^ (value >> 4) ^ (value >> 8)) & 0xF;
                if(bidirectional)
                {
                    crc = ~crc & 0xF;
                }
                if((frame & 0xF) != crc)
                {
                    return null;
                }
                Input(value >> 1, nowS);
                return bidirectional ? Telemetry() : (ushort?)null;
            }
        }

        // AM32 resets on signal loss: RAM state and unsaved settings are lost
        private void CheckSignal(double nowS)
        {
            if(lastSignalS >= 0 &&
               nowS - lastSignalS > (armed ? ArmedSignalTimeoutS : DisarmedSignalTimeoutS))
            {
                Restart();
            }
        }

        private void Restart()
        {
            dirReversed = savedDirReversed;
            biDirection = savedBiDirection;
            forward = !dirReversed;
            armed = false;
            bidirectional = false;
            lastSignalS = -1;
            zeroSinceS = -1;
            zeroCount = 0;
            throttle = 0;
            extendedTelemetry = false;
            sendInit = false;
            sendDeinit = false;
            lastSentExtended = false;
            currentCount = 0;
            voltageCount = 0;
            temperatureCount = 0;
            lastCommand = 0;
            commandCount = 0;
        }

        private void Input(int input, double nowS)
        {
            throttle = 0;
            if(input > 47)
            {
                commandCount = 0;
                Throttle(input);
            }
            else if(input == 0)
            {
                commandCount = 0;
            }
            else
            {
                Command(input);
            }
            if(armed)
            {
                return;
            }
            // zero input, including commands, counts towards arming
            if(input <= 47)
            {
                zeroCount++;
                if(zeroSinceS < 0)
                {
                    zeroSinceS = nowS;
                }
                armed = nowS - zeroSinceS >= ArmDelayS && zeroCount > ArmZeroFrames;
            }
            else
            {
                zeroCount = 0;
                zeroSinceS = -1;
            }
        }

        private void Throttle(int input)
        {
            if(!armed)
            {
                return;
            }
            if(!biDirection)
            {
                throttle = (input - 47) / 2000.0;
                return;
            }
            // 3D: 1048-2047 spins normal, 48-1047 reversed, each scaled to 47-2047
            var wantForward = input > 1047;
            var adjusted = wantForward ? (input - 1048) * 2 + 47 : (input - 48) * 2 + 47;
            var direction = wantForward ? !dirReversed : dirReversed;
            if(forward != direction)
            {
                // AM32 only reverses once the motor has slowed, coasting until then
                if(mechanicalRpm * poles / 2 > ReverseErpm)
                {
                    return;
                }
                forward = direction;
            }
            throttle = (adjusted - 47) / 2000.0;
        }

        private void Command(int command)
        {
            if(!armed || mechanicalRpm > 0)
            {
                return;
            }
            if(command != lastCommand)
            {
                lastCommand = command;
                commandCount = 0;
            }
            if(command <= 5)
            {
                commandCount = CommandRepeats;
            }
            if(++commandCount < CommandRepeats)
            {
                return;
            }
            commandCount = 0;
            switch(command)
            {
            case 7:
                dirReversed = false;
                forward = true;
                break;
            case 8:
                dirReversed = true;
                forward = false;
                break;
            case 9:
                biDirection = false;
                break;
            case 10:
                biDirection = true;
                break;
            case 12:
                savedDirReversed = dirReversed;
                savedBiDirection = biDirection;
                break;
            case 13:
                extendedTelemetry = true;
                sendInit = true;
                break;
            case 14:
                extendedTelemetry = false;
                sendDeinit = true;
                break;
            case 20:
                forward = !dirReversed;
                break;
            case 21:
                forward = dirReversed;
                break;
            }
        }

        private void Update(double nowS)
        {
            if(lastUpdateS < 0)
            {
                lastUpdateS = nowS;
                return;
            }
            var dt = Math.Max(0, nowS - lastUpdateS);
            lastUpdateS = nowS;
            var alpha = 1 - Math.Exp(-dt / MotorTimeConstantS);
            mechanicalRpm += (TargetRpm - mechanicalRpm) * alpha;
            if(mechanicalRpm < StoppedRpm && TargetRpm == 0)
            {
                mechanicalRpm = 0;
            }
            var heat = AmbientC + Current * Current * HeatingCPerA2;
            temperatureC += (heat - temperatureC) * (1 - Math.Exp(-dt / ThermalTimeConstantS));
        }

        private double TargetRpm => throttle * kv * batteryVoltage;
        private double Current => maxCurrent * Math.Pow(throttle, 1.5) *
            Math.Min(1, mechanicalRpm / Math.Max(1, TargetRpm));
        private double Voltage => batteryVoltage - Current * InternalResistanceOhm;

        private ushort Telemetry()
        {
            var extended = 0;
            if(extendedTelemetry)
            {
                // interleave so every EDT frame follows an eRPM frame
                if(lastSentExtended)
                {
                    lastSentExtended = false;
                }
                else
                {
                    currentCount++;
                    voltageCount++;
                    temperatureCount++;
                    if(currentCount >= CurrentDivisor)
                    {
                        extended = 0x600 | (int)Math.Min(255, Current);
                        currentCount = 0;
                    }
                    else if(voltageCount >= VoltageDivisor)
                    {
                        extended = 0x400 | (int)Math.Min(255, Voltage / 0.25);
                        voltageCount = 0;
                    }
                    else if(temperatureCount >= TemperatureDivisor)
                    {
                        extended = 0x200 | (int)Math.Max(0, Math.Min(255, temperatureC));
                        temperatureCount = 0;
                    }
                }
            }
            if(sendInit)
            {
                extended = 0xE00;
                sendInit = false;
            }
            if(sendDeinit)
            {
                extended = 0xEFF;
                sendDeinit = false;
            }
            int value;
            if(extended != 0)
            {
                value = extended;
                lastSentExtended = true;
            }
            else
            {
                value = EncodePeriod(CommutationTimeUs());
            }
            var crc = ~(value ^ (value >> 4) ^ (value >> 8)) & 0xF;
            return (ushort)((value << 4) | crc);
        }

        // AM32 reports the electrical revolution period in microseconds
        private int CommutationTimeUs()
        {
            var erpm = mechanicalRpm * poles / 2;
            if(erpm < 1)
            {
                return MaxCommutationTimeUs;
            }
            return (int)Math.Min(MaxCommutationTimeUs, Math.Round(60000000 / erpm));
        }

        // eee mmmmmmmmm with the shift from the highest set bit, as AM32 does
        private static int EncodePeriod(int comTime)
        {
            var shift = 0;
            for(var bit = 15; bit >= 9; bit--)
            {
                if(comTime >> bit == 1)
                {
                    shift = bit + 1 - 9;
                    break;
                }
            }
            return (shift << 9) | (comTime >> shift);
        }

        private readonly int poles;
        private readonly double kv;
        private readonly double batteryVoltage;
        private readonly double maxCurrent;
        private readonly object sync = new object();

        // eeprom settings the commands change, and the saved copies
        private bool dirReversed;
        private bool biDirection;
        private bool savedDirReversed;
        private bool savedBiDirection;

        private bool armed;
        private bool bidirectional;
        private bool forward;
        private double zeroSinceS;
        private int zeroCount;
        private double lastSignalS;
        private double throttle;
        private double mechanicalRpm;
        private double temperatureC;
        private double lastUpdateS;
        private bool extendedTelemetry;
        private bool sendInit;
        private bool sendDeinit;
        private bool lastSentExtended;
        private int currentCount;
        private int voltageCount;
        private int temperatureCount;
        private int lastCommand;
        private int commandCount;

        private const double ArmDelayS = 1.0;
        private const int ArmZeroFrames = 30;
        private const double ArmedSignalTimeoutS = 0.5;
        private const double DisarmedSignalTimeoutS = 2.0;
        private const int CommandRepeats = 6;
        private const int CurrentDivisor = 40;
        private const int VoltageDivisor = 200;
        private const int TemperatureDivisor = 200;
        private const int MaxCommutationTimeUs = 65535;
        // AM32 reverse_speed_threshold: 1500us per commutation step, six per cycle
        private const double ReverseErpm = 60000000.0 / (6 * 1500);
        private const double MotorTimeConstantS = 0.05;
        private const double StoppedRpm = 50;
        private const double AmbientC = 25;
        private const double HeatingCPerA2 = 0.05;
        private const double ThermalTimeConstantS = 30;
        private const double InternalResistanceOhm = 0.02;
    }
}
