//
// i.MX RT eFlexPWM: the submodule registers ArduPilot's outputs are programmed
// through, and the actuator tap that turns them into physics motor commands.
//
// This is the RT1176's counterpart of AP_STM32_Timer_Actuators, and it exists
// for the same reason: the emulated board can only fly if the firmware's motor
// outputs reach the physics model. On the STM32 boards that tap reads Renode's
// own timer model; here the model and the tap are one peripheral, because
// nothing needs to be shared with a second consumer and the register set is
// small enough to carry.
//
// What the guest does with it (drivers/pwm/pwm_mcux.c and the MCUX fsl_pwm.c):
//
//     SM[n].INIT = 0                     edge aligned, counting up from zero
//     SM[n].VAL1 = period_cycles - 1     the period
//     SM[n].VAL2 = 0, VAL3 = pulse       channel A duty
//     SM[n].VAL4 = 0, VAL5 = pulse       channel B duty
//     SM[n].CTRL PRSC                    prescaler, 1 << PRSC
//     OUTEN PWMA_EN/PWMB_EN              output enable, one bit per submodule
//     MCTRL RUN                          submodule counters running
//     MCTRL LDOK                         load the VALx into the active set
//
// and reads back only what it wrote (CTRL for the prescaler, CTRL2 and MCTRL as
// read-modify-write). There is no polling loop to satisfy, so a plain register
// store is a complete model for this guest. LDOK is accepted and recorded but
// the written values are used immediately: buffering them would only delay the
// motor command by one physics step.
//
// pulse microseconds = pulse_cycles * prescaler / clockFrequency, so
// clockFrequency must be the rate the FIRMWARE believes the PWM clock runs at -
// IMX_CCM_PWM_CLK resolves to the bus clock root
// (clock_control_mcux_ccm_rev2.c). A mismatch shows up as motors that are
// uniformly too fast or too slow; the model logs the period it derives the
// first time a submodule is armed, and warns when that period is not a
// plausible servo frame, which is what makes such a mismatch visible rather
// than mysterious.
//
// This file is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This file is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_IMXRT_FlexPWM : IWordPeripheral, IDoubleWordPeripheral, IKnownSize,
        IAP_PhysicsActuatorSource
    {
        public AP_IMXRT_FlexPWM(IMachine machine, ulong clockFrequency = DefaultClockFrequency,
            int output0 = Unmapped, int output1 = Unmapped,
            int output2 = Unmapped, int output3 = Unmapped,
            bool channelB0 = false, bool channelB1 = false,
            bool channelB2 = false, bool channelB3 = false)
        {
            this.clockFrequency = clockFrequency;
            outputs = new[] { output0, output1, output2, output3 };
            channelB = new[] { channelB0, channelB1, channelB2, channelB3 };
            foreach(var output in outputs)
            {
                if(output < Unmapped || output >= AP_PhysicsActuator.Count)
                {
                    throw new ArgumentOutOfRangeException(nameof(output0),
                        "physics actuator output is out of range");
                }
            }
            registers = new ushort[RegisterWords];
            reported = new bool[SubmoduleCount];
            AP_PhysicsState.ForMachine(machine).RegisterActuatorSource(this);
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            Array.Clear(reported, 0, reported.Length);
            // Reset values that are not zero: the fault disable map lets every
            // channel through, which is what fsl_pwm expects to find.
            for(var submodule = 0; submodule < SubmoduleCount; submodule++)
            {
                Write(SubmoduleBase(submodule) + Dismap0, 0xFFFF);
                Write(SubmoduleBase(submodule) + Dismap1, 0xFFFF);
            }
        }

        public ushort ReadWord(long offset) => Read(offset);

        public void WriteWord(long offset, ushort value) => Write(offset, value);

        // The MCUX driver uses 16-bit accesses throughout; 32-bit ones are
        // served as two halves so a memory-dump or a compiler that widens an
        // access still sees the right thing.
        public uint ReadDoubleWord(long offset)
        {
            return (uint)Read(offset) | ((uint)Read(offset + 2) << 16);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            Write(offset, (ushort)value);
            Write(offset + 2, (ushort)(value >> 16));
        }

        public void Sample(AP_PhysicsActuator[] actuators)
        {
            var moduleControl = Read(Mctrl);
            var outputEnable = Read(Outen);
            for(var submodule = 0; submodule < SubmoduleCount; submodule++)
            {
                var output = outputs[submodule];
                if(output == Unmapped)
                {
                    continue;
                }
                var b = channelB[submodule];
                var baseOffset = SubmoduleBase(submodule);
                var prescaler = 1UL << ((Read(baseOffset + Ctrl) & PrescalerMask) >> PrescalerShift);
                // Unsigned. PWM_SetPeriodRegister writes INIT = 0 and
                // VAL1 = period - 1 for the edge-aligned mode this driver uses,
                // and the driver allows a period up to UINT16_MAX, so a period
                // register above 32767 is ordinary rather than negative: at this
                // board's 1.875 MHz counter that is every output slower than
                // 57 Hz, which includes the 50 Hz default every unmapped channel
                // sits at. Reading them signed made those read as invalid
                // actuators and silenced this model's own mismatch warning.
                int init = Read(baseOffset + Init);
                int modulo = Read(baseOffset + Val1);
                var periodCycles = modulo - init + 1;
                int low = Read(baseOffset + (b ? Val4 : Val2));
                int high = Read(baseOffset + (b ? Val5 : Val3));
                var pulseCycles = high - low;

                var running = (moduleControl & (RunMask << submodule)) != 0;
                var enableMask = (b ? PwmBEnableBase : PwmAEnableBase) << submodule;
                var enabled = (outputEnable & enableMask) != 0;
                var periodUs = clockFrequency == 0 ? 0.0
                    : periodCycles * (double)prescaler * MicrosecondsPerSecond / clockFrequency;
                var pulseUs = clockFrequency == 0 ? 0.0
                    : pulseCycles * (double)prescaler * MicrosecondsPerSecond / clockFrequency;

                if(running && enabled && periodCycles > 0 && !reported[submodule])
                {
                    reported[submodule] = true;
                    var level = periodUs >= MinimumServoPeriodUs && periodUs <= MaximumServoPeriodUs
                        ? LogLevel.Info : LogLevel.Warning;
                    this.Log(level,
                        "submodule {0} -> physics output {1}: period {2} cycles, prescaler {3}, " +
                        "clock {4} Hz = {5:F1} us ({6:F0} Hz){7}",
                        submodule, output, periodCycles, prescaler, clockFrequency, periodUs,
                        periodUs > 0 ? MicrosecondsPerSecond / periodUs : 0.0,
                        level == LogLevel.Warning
                            ? " - NOT a servo frame; the clockFrequency this model was given "
                              + "probably differs from the one the firmware computed"
                            : "");
                }

                var valid = running && enabled && periodCycles > 0 &&
                    pulseCycles >= 0 && pulseCycles <= periodCycles;
                var value = (ushort)Math.Min(ushort.MaxValue, Math.Max(0.0, Math.Round(pulseUs)));
                actuators[output] = new AP_PhysicsActuator(value, AP_PhysicsActuator.ProtocolPwm,
                    valid ? AP_PhysicsActuator.FlagValid : (byte)0);
            }
        }

        public long Size => 0x4000;

        private ushort Read(long offset)
        {
            var index = offset / 2;
            if(index < 0 || index >= registers.Length)
            {
                return 0;
            }
            if(offset == Fsts)
            {
                // No faults are ever raised here, and fsl_pwm clears this by
                // writing back what it read.
                return 0;
            }
            return registers[index];
        }

        private void Write(long offset, ushort value)
        {
            var index = offset / 2;
            if(index < 0 || index >= registers.Length)
            {
                return;
            }
            if(offset == Mctrl)
            {
                // CLDOK clears the load-ok bits it names; LDOK is recorded but
                // the written VALx are already live, see the file comment.
                var cleared = (ushort)((value & ClearLoadOkMask) >> ClearLoadOkShift);
                value &= unchecked((ushort)~ClearLoadOkMask);
                value &= (ushort)~cleared;
            }
            registers[index] = value;
        }

        private static long SubmoduleBase(int submodule) => submodule * SubmoduleStride;

        private readonly ushort[] registers;
        private readonly bool[] reported;
        private readonly int[] outputs;
        private readonly bool[] channelB;
        private readonly ulong clockFrequency;

        private const int Unmapped = -1;
        private const int SubmoduleCount = 4;
        private const long SubmoduleStride = 0x60;
        private const int RegisterWords = 0x4000 / 2;

        // Submodule register offsets, relative to the submodule base.
        private const long Init = 0x02;
        private const long Ctrl = 0x06;
        private const long Val1 = 0x0E;
        private const long Val2 = 0x12;
        private const long Val3 = 0x16;
        private const long Val4 = 0x1A;
        private const long Val5 = 0x1E;
        private const long Dismap0 = 0x2C;
        private const long Dismap1 = 0x2E;

        // Module register offsets.
        private const long Outen = 0x180;
        private const long Mctrl = 0x188;
        private const long Fsts = 0x18E;

        private const ushort PrescalerMask = 0x70;
        private const int PrescalerShift = 4;
        private const ushort RunMask = 0x100;          // MCTRL[RUN], one bit per submodule
        private const ushort ClearLoadOkMask = 0xF0;   // MCTRL[CLDOK]
        private const int ClearLoadOkShift = 4;
        private const ushort PwmAEnableBase = 0x100;   // OUTEN[PWMA_EN], one bit per submodule
        private const ushort PwmBEnableBase = 0x10;    // OUTEN[PWMB_EN]

        private const ulong DefaultClockFrequency = 240000000;
        private const double MicrosecondsPerSecond = 1000000.0;
        private const double MinimumServoPeriodUs = 1000.0;    // 1 kHz
        private const double MaximumServoPeriodUs = 25000.0;   // 40 Hz
    }
}
