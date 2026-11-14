//
// Samples servo PWM from the RP2350 PWM block for the lockstep physics
// protocol. The block itself stays an AP_RP2350_Block (plain register
// storage); this tap only reads it.
//
// A PWM channel is numbered slice * 2 + (0 for A, 1 for B), which for GPIO0-15
// is the GPIO number. outputN names the channel that drives physics output N,
// or -1. The counter runs at clk_sys / (DIV.INT + DIV.FRAC / 16), and a
// phase-correct slice counts up and down, doubling both period and pulse.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_RP2350_PWM_Actuators : IDoubleWordPeripheral, IKnownSize,
        IAP_PhysicsActuatorSource
    {
        public AP_RP2350_PWM_Actuators(IMachine machine, AP_RP2350_Block pwm,
            long frequency, int output0 = Unmapped, int output1 = Unmapped,
            int output2 = Unmapped, int output3 = Unmapped,
            int output4 = Unmapped, int output5 = Unmapped,
            int output6 = Unmapped, int output7 = Unmapped)
        {
            this.pwm = pwm;
            this.frequency = frequency;
            channels = new int[] {
                output0, output1, output2, output3, output4, output5, output6, output7
            };
            foreach(var channel in channels)
            {
                if(channel < Unmapped || channel >= Slices * 2)
                {
                    throw new ArgumentOutOfRangeException(
                        nameof(output0), "PWM channel is out of range");
                }
            }
            AP_PhysicsState.ForMachine(machine).RegisterActuatorSource(this);
        }

        public void Sample(AP_PhysicsActuator[] actuators)
        {
            for(var output = 0; output < channels.Length; output++)
            {
                var channel = channels[output];
                if(channel == Unmapped)
                {
                    continue;
                }
                var slice = channel / 2;
                var baseOffset = slice * SliceStride;
                var csr = pwm.ReadDoubleWord(baseOffset + Csr);
                var div = pwm.ReadDoubleWord(baseOffset + Div) & 0xFFF;
                var cc = pwm.ReadDoubleWord(baseOffset + Cc);
                var top = pwm.ReadDoubleWord(baseOffset + Top) & 0xFFFF;
                var compare = (channel % 2 == 0 ? cc : cc >> 16) & 0xFFFF;
                // DIV.INT of 0 means 256
                var divider = (div >> 4 == 0 ? 256.0 : (div >> 4)) + (div & 0xF) / 16.0;
                var scale = (csr & PhaseCorrect) != 0 ? 2.0 : 1.0;
                var tickUs = frequency == 0 ? 0.0 : divider * MicrosecondsPerSecond / frequency;
                var periodUs = (top + 1) * tickUs * scale;
                var pulseUs = compare * tickUs * scale;
                var valid = (csr & Enable) != 0 && compare <= top + 1 &&
                    periodUs >= MinimumServoPeriodUs;
                var value = (ushort)Math.Min(UInt16.MaxValue, Math.Max(0.0, Math.Round(pulseUs)));
                actuators[output] = new AP_PhysicsActuator(
                    value, AP_PhysicsActuator.ProtocolPwm,
                    valid ? AP_PhysicsActuator.FlagValid : (byte)0);
            }
        }

        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset() { }
        public long Size => 4;

        private readonly AP_RP2350_Block pwm;
        private readonly long frequency;
        private readonly int[] channels;

        private const int Unmapped = -1;
        private const int Slices = 12;
        private const long SliceStride = 0x14;
        private const long Csr = 0x00;
        private const long Div = 0x04;
        private const long Cc = 0x0C;
        private const long Top = 0x10;
        private const uint Enable = 1U << 0;
        private const uint PhaseCorrect = 1U << 1;
        private const double MicrosecondsPerSecond = 1000000.0;
        // anything faster than 2.5 kHz is not a servo frame
        private const double MinimumServoPeriodUs = 400.0;
    }
}
