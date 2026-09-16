// Samples ordinary PWM outputs from Renode's STM32 timer model for the
// lockstep physics protocol. DShot timers have a bit-period rather than a
// servo frame-period and carry a changing bitstream in CCR, so they are
// identified but remain invalid until a DMA-aware decoder can reconstruct a
// complete frame.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32_Timer_Actuators : IDoubleWordPeripheral, IKnownSize,
        IAP_PhysicsActuatorSource
    {
        public AP_STM32_Timer_Actuators(IMachine machine, STM32_Timer timer,
            int output1 = Unmapped, int output2 = Unmapped,
            int output3 = Unmapped, int output4 = Unmapped,
            bool complementary1 = false, bool complementary2 = false,
            bool complementary3 = false, bool complementary4 = false)
        {
            this.timer = timer;
            outputs = new int[] { output1, output2, output3, output4 };
            complementary = new bool[] {
                complementary1, complementary2, complementary3, complementary4
            };
            foreach(var output in outputs)
            {
                if(output < Unmapped || output >= AP_PhysicsActuator.Count)
                {
                    throw new ArgumentOutOfRangeException(
                        nameof(output1), "physics actuator output is out of range");
                }
            }
            AP_PhysicsState.ForMachine(machine).RegisterActuatorSource(this);
        }

        public void Sample(AP_PhysicsActuator[] actuators)
        {
            var enabled = (timer.ReadDoubleWord(Control1) & CounterEnable) != 0;
            var channelEnable = timer.ReadDoubleWord(CaptureCompareEnable);
            var divider = timer.ReadDoubleWord(Prescaler) + 1UL;
            var autoReload = timer.ReadDoubleWord(AutoReload);
            var periodUs = timer.Frequency == 0 ? 0.0 :
                (autoReload + 1UL) * divider * MicrosecondsPerSecond / timer.Frequency;
            var dshot = periodUs > 0.0 && periodUs <= DshotMaximumBitPeriodUs;
            for(var channel = 0; channel < outputs.Length; channel++)
            {
                var output = outputs[channel];
                if(output == Unmapped)
                {
                    continue;
                }
                if(dshot)
                {
                    actuators[output] = new AP_PhysicsActuator(
                        0, AP_PhysicsActuator.ProtocolDshot, 0);
                    continue;
                }
                var modeRegister = timer.ReadDoubleWord(
                    channel < 2 ? CaptureCompareMode1 : CaptureCompareMode2);
                var modeShift = (channel % 2) * 8 + 4;
                var mode = (modeRegister >> modeShift) & 0x7;
                // Renode's STM32 timer stores CCxE, but models CCxNE as an
                // unimplemented tag which always reads back as zero.  The
                // generator therefore supplies complementary channel usage
                // from hwdef.dat; a running PWM timer implies that output is
                // enabled once its firmware has configured the channel.
                var channelEnabled = complementary[channel] ||
                    (channelEnable & (1U << (channel * 4))) != 0;
                var compare = timer.ReadDoubleWord(CaptureCompare1 + channel * 4);
                var valid = enabled && channelEnabled && compare <= autoReload &&
                    (mode == PwmMode1 || mode == PwmMode2);
                var pulseUs = timer.Frequency == 0 ? 0.0 :
                    compare * divider * MicrosecondsPerSecond / timer.Frequency;
                var value = (ushort)Math.Min(UInt16.MaxValue,
                    Math.Max(0.0, Math.Round(pulseUs)));
                actuators[output] = new AP_PhysicsActuator(
                    value, AP_PhysicsActuator.ProtocolPwm,
                    valid ? AP_PhysicsActuator.FlagValid : (byte)0);
            }
        }

        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset() { }
        public long Size => 4;

        private readonly STM32_Timer timer;
        private readonly int[] outputs;
        private readonly bool[] complementary;

        private const int Unmapped = -1;
        private const long Control1 = 0x00;
        private const long CaptureCompareMode1 = 0x18;
        private const long CaptureCompareMode2 = 0x1C;
        private const long CaptureCompareEnable = 0x20;
        private const long Prescaler = 0x28;
        private const long AutoReload = 0x2C;
        private const long CaptureCompare1 = 0x34;
        private const uint CounterEnable = 1U << 0;
        private const uint PwmMode1 = 6;
        private const uint PwmMode2 = 7;
        private const double MicrosecondsPerSecond = 1000000.0;
        private const double DshotMaximumBitPeriodUs = 10.0;
    }
}
