// Physics-driven LightWare serial lidar using the legacy ASCII protocol.
using System;
using System.Globalization;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_LightWare : AP_UARTFrameDevice
    {
        public AP_LightWare(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var truth = physics.Current;
            var distanceM = truth.TimestampUs == 0
                ? DefaultDistanceM
                : truth.RangefinderM[RangefinderIndex];
            return Encoding.ASCII.GetBytes(
                distanceM.ToString("F2", CultureInfo.InvariantCulture) + "\r");
        }

        public uint RangefinderIndex { get; set; }

        private readonly AP_PhysicsState physics;
        private const uint ReadingsPerSecond = 20;
        private const double DefaultDistanceM = 5.0;
    }
}
