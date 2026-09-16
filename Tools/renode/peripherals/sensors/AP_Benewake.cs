// Physics-driven Benewake-compatible serial rangefinder using the common 0x59
// frame accepted by the AP_RangeFinder serial backends.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_Benewake : AP_UARTFrameDevice
    {
        public AP_Benewake(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceCentimetres = DistanceCentimetres();
            var frame = new byte[] {
                0x59, 0x59,
                (byte)(distanceCentimetres & 0xFF),
                (byte)(distanceCentimetres >> 8),
                0x64, 0x00, // signal strength 100
                0x07, 0x00, // reliable TF02 signal / reserved
                0x00,
            };
            byte checksum = 0;
            for(var i = 0; i < frame.Length - 1; i++)
            {
                checksum += frame[i];
            }
            frame[frame.Length - 1] = checksum;
            return frame;
        }

        private ushort DistanceCentimetres()
        {
            var truth = physics.Current;
            var distanceM = truth.TimestampUs == 0
                ? DefaultDistanceM
                : truth.RangefinderM[RangefinderIndex];
            return (ushort)Math.Max(0, Math.Min(ushort.MaxValue,
                Math.Round(distanceM * 100.0)));
        }

        public uint RangefinderIndex { get; set; }

        private readonly AP_PhysicsState physics;
        private const uint ReadingsPerSecond = 20;
        private const double DefaultDistanceM = 5.0;
    }
}
