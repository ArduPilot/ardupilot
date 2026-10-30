// Emulated R/C transmitter and receiver.
//
// AP_RCInput holds the stick positions for a machine and is driven from the
// Renode monitor. A disabled transmitter stops all receiver output, which the
// firmware sees as a lost link, so disabling it in flight tests R/C failsafe.
// AP_SBusInput is an SBUS receiver on a UART, either on the flight controller
// or on a real IOMCU's RC input; the synthetic AP_IOMCU reports the same
// channels through its RC input registers.
//
using System;
using System.Linq;
using System.Runtime.CompilerServices;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public sealed class AP_RCInputState
    {
        public static AP_RCInputState ForMachine(IMachine machine)
        {
            return states.GetValue(machine, _ => new AP_RCInputState());
        }

        // a coprocessor such as a real IOMCU uses the main machine's sticks
        public static AP_RCInputState ForMachine(IMachine machine, string name)
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

        public bool Enabled { get; set; }

        // returns null while the transmitter is off
        public ushort[] Channels
        {
            get
            {
                lock(sync)
                {
                    return Enabled ? (ushort[])channels.Clone() : null;
                }
            }
        }

        public void SetChannels(ushort[] values)
        {
            lock(sync)
            {
                Array.Copy(values, channels, Math.Min(values.Length, ChannelCount));
            }
        }

        public const int ChannelCount = 16;
        public const ushort MinimumPwm = 800;
        public const ushort MaximumPwm = 2200;

        private AP_RCInputState()
        {
            for(var i = 0; i < ChannelCount; i++)
            {
                channels[i] = 1500;
            }
        }

        private readonly ushort[] channels = new ushort[ChannelCount];
        private readonly object sync = new object();
        private static readonly ConditionalWeakTable<IMachine, AP_RCInputState> states =
            new ConditionalWeakTable<IMachine, AP_RCInputState>();
    }

    public class AP_RCInput : IDoubleWordPeripheral, IKnownSize
    {
        public AP_RCInput(IMachine machine)
        {
            state = AP_RCInputState.ForMachine(machine);
        }

        public bool Enabled
        {
            get => state.Enabled;
            set => state.Enabled = value;
        }

        // comma separated PWM values starting at channel 1
        public void SetChannels(string values)
        {
            ushort[] parsed;
            try
            {
                parsed = values.Split(',').Select(value => ushort.Parse(value.Trim())).ToArray();
            }
            catch(Exception error) when(error is FormatException || error is OverflowException)
            {
                throw new RecoverableException("channels must be comma separated PWM values");
            }
            if(parsed.Length == 0 || parsed.Length > AP_RCInputState.ChannelCount ||
               parsed.Any(value => value < AP_RCInputState.MinimumPwm ||
                                   value > AP_RCInputState.MaximumPwm))
            {
                throw new RecoverableException(string.Format(
                    "expected 1 to {0} PWM values from {1} to {2}",
                    AP_RCInputState.ChannelCount, AP_RCInputState.MinimumPwm,
                    AP_RCInputState.MaximumPwm));
            }
            state.SetChannels(parsed);
        }

        public string Channels =>
            string.Join(",", state.Channels ?? new ushort[0]);

        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        // stick positions survive a flight controller reset
        public void Reset() { }
        public long Size => 4;

        private readonly AP_RCInputState state;
    }
}

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_SBusInput : AP_UARTFrameDevice
    {
        public AP_SBusInput(IMachine machine, string controlMachine = null)
            : base(machine, FramesPerSecond, BaudRate100k)
        {
            state = AP_RCInputState.ForMachine(machine, controlMachine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var channels = state.Channels;
            if(channels == null)
            {
                return null;
            }
            var frame = new byte[FrameSize];
            frame[0] = StartByte;
            var bit = 0;
            foreach(var pwm in channels)
            {
                // inverse of the firmware's 200..1800 -> 1000..2000 mapping
                var raw = (int)Math.Round((pwm - 875) * 1.6);
                raw = Math.Max(0, Math.Min(0x7FF, raw));
                for(var i = 0; i < 11; i++, bit++)
                {
                    if((raw & (1 << i)) != 0)
                    {
                        frame[1 + bit / 8] |= (byte)(1 << (bit % 8));
                    }
                }
            }
            return frame;
        }

        private readonly AP_RCInputState state;

        private const uint FramesPerSecond = 71; // 14ms SBUS frame interval
        private const uint BaudRate100k = 100000;
        private const int FrameSize = 25;
        private const byte StartByte = 0x0F;
    }
}
