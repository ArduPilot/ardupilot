// Physics-driven serial optical-flow sensors.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_SerialOpticalFlow : AP_UARTFrameDevice
    {
        protected AP_SerialOpticalFlow(IMachine machine, uint framesPerSecond) :
            base(machine, framesPerSecond, BaudRate)
        {
            physics = AP_PhysicsState.ForMachine(machine);
        }

        protected override byte[] BuildFrame()
        {
            var truth = physics.Current;
            var bodyVelocity = NedToBody(truth.VelocityNedMS, truth.Quaternion);
            var height = truth.RangefinderM.Length > 0 ? truth.RangefinderM[0] : 0.0f;
            if(height < MinimumHeightM)
            {
                height = MinimumHeightM;
            }

            var flowRateX = truth.GyroRadS[0] - bodyVelocity[1] / height;
            var flowRateY = truth.GyroRadS[1] + bodyVelocity[0] / height;
            return BuildFlowFrame(flowRateX, flowRateY);
        }

        protected abstract byte[] BuildFlowFrame(double flowRateX,
            double flowRateY);

        protected static short ClampInt16(double value)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value)));
        }

        protected static void PutInt16(byte[] buffer, int offset, short value)
        {
            buffer[offset] = (byte)(value & 0xFF);
            buffer[offset + 1] = (byte)(((ushort)value) >> 8);
        }

        protected static void PutUInt16(byte[] buffer, int offset, ushort value)
        {
            buffer[offset] = (byte)(value & 0xFF);
            buffer[offset + 1] = (byte)(value >> 8);
        }

        private static float[] NedToBody(float[] ned, float[] quaternion)
        {
            var w = quaternion[0];
            var x = quaternion[1];
            var y = quaternion[2];
            var z = quaternion[3];
            return new float[] {
                (1 - 2 * (y * y + z * z)) * ned[0] +
                    2 * (x * y + w * z) * ned[1] +
                    2 * (x * z - w * y) * ned[2],
                2 * (x * y - w * z) * ned[0] +
                    (1 - 2 * (x * x + z * z)) * ned[1] +
                    2 * (y * z + w * x) * ned[2],
                2 * (x * z + w * y) * ned[0] +
                    2 * (y * z - w * x) * ned[1] +
                    (1 - 2 * (x * x + y * y)) * ned[2],
            };
        }

        private readonly AP_PhysicsState physics;

        private const uint BaudRate = 19200;
        private const float MinimumHeightM = 0.05f;
    }

    public class AP_CXOF : AP_SerialOpticalFlow
    {
        public AP_CXOF(IMachine machine) : base(machine, FramesPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFlowFrame(double flowRateX,
            double flowRateY)
        {
            var frame = new byte[FrameLength];
            frame[0] = Header;
            PutInt16(frame, 2, FlowIntegral(flowRateX));
            PutInt16(frame, 4, FlowIntegral(flowRateY));
            frame[7] = Quality;
            frame[8] = Footer;
            return frame;
        }

        private static short FlowIntegral(double rateRadS)
        {
            return ClampInt16(rateRadS * IntegrationTimeSeconds /
                PixelScaling);
        }

        private const uint FramesPerSecond = 25;
        private const int FrameLength = 9;
        private const double IntegrationTimeSeconds = 1.0 / FramesPerSecond;
        private const double PixelScaling = 1.76e-3;
        private const byte Header = 0xFE;
        private const byte Footer = 0xAA;
        // The backend maps the sensor's 64--78 range onto 0--255.
        private const byte Quality = 75;
    }

    public class AP_UPFLOW : AP_SerialOpticalFlow
    {
        public AP_UPFLOW(IMachine machine) : base(machine, FramesPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFlowFrame(double flowRateX,
            double flowRateY)
        {
            var frame = new byte[FrameLength];
            frame[0] = Header0;
            frame[1] = Header1;
            PutInt16(frame, 2, FlowIntegral(flowRateX));
            PutInt16(frame, 4, FlowIntegral(flowRateY));
            PutUInt16(frame, 6, IntegrationTimeUs);
            PutUInt16(frame, 8, GroundDistanceReserved);
            frame[10] = Quality;
            frame[11] = HardwareVersion;

            byte checksum = 0;
            for(var index = 2; index < 12; index++)
            {
                checksum ^= frame[index];
            }
            frame[12] = (byte)(checksum ^ (CorruptChecksum ? 0xFF : 0));
            frame[13] = Footer;
            return frame;
        }

        public bool CorruptChecksum { get; set; }

        private static short FlowIntegral(double rateRadS)
        {
            // The production backend negates both sensor axes.
            return ClampInt16(-rateRadS * IntegrationTimeUs * 1.0e-6 /
                PixelScaling);
        }

        private const uint FramesPerSecond = 25;
        private const int FrameLength = 14;
        private const ushort IntegrationTimeUs = 40000;
        private const double PixelScaling = 1.0e-4;
        private const ushort GroundDistanceReserved = 999;
        private const byte Quality = 245;
        private const byte HardwareVersion = 1;
        private const byte Header0 = 0xFE;
        private const byte Header1 = 0x0A;
        private const byte Footer = 0x55;
    }
}
