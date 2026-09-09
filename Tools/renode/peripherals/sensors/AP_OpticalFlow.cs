// Physics-driven PX4Flow I2C integral-frame sensor.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_PX4Flow : II2CPeripheral
    {
        public AP_PX4Flow(IMachine machine)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            Reset();
        }

        public void Reset()
        {
            pointer = 0;
            frameCount = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length > 0)
            {
                pointer = data[0];
            }
        }

        public byte[] Read(int count = 1)
        {
            if(pointer != IntegralFrame)
            {
                return new byte[count];
            }

            var frame = BuildFrame();
            var result = new byte[count];
            Array.Copy(frame, result, Math.Min(frame.Length, result.Length));
            return result;
        }

        public void FinishTransmission()
        {
        }

        private byte[] BuildFrame()
        {
            var truth = physics.Current;
            var bodyVelocity = NedToBody(truth.VelocityNedMS, truth.Quaternion);
            var height = truth.RangefinderM.Length > 0 ? truth.RangefinderM[0] : 0.0f;
            if(height < MinimumHeightM)
            {
                height = MinimumHeightM;
            }

            var bodyRateX = truth.GyroRadS[0];
            var bodyRateY = truth.GyroRadS[1];
            var flowRateX = bodyRateX - bodyVelocity[1] / height;
            var flowRateY = bodyRateY + bodyVelocity[0] / height;
            frameCount++;

            var frame = new byte[IntegralFrameSize];
            PutUInt16(frame, 0, frameCount);
            PutInt16(frame, 2, Integral(flowRateX));
            PutInt16(frame, 4, Integral(flowRateY));
            PutInt16(frame, 6, Integral(bodyRateX));
            PutInt16(frame, 8, Integral(bodyRateY));
            PutInt16(frame, 10, Integral(truth.GyroRadS[2]));
            PutUInt32(frame, 12, IntegrationTimeUs);
            PutUInt32(frame, 16, (uint)(truth.TimestampUs & 0xFFFFFFFF));
            PutUInt16(frame, 20, ClampUInt16(height * 1000.0));
            PutInt16(frame, 22, ClampInt16(
                (truth.TemperatureK - 273.15) * 100.0));
            frame[24] = Quality;
            return frame;
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

        private static short Integral(double rateRadS)
        {
            return ClampInt16(rateRadS * IntegrationTimeUs * 1.0e-6 / 1.0e-4);
        }

        private static short ClampInt16(double value)
        {
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value)));
        }

        private static ushort ClampUInt16(double value)
        {
            return (ushort)Math.Max(UInt16.MinValue,
                Math.Min(UInt16.MaxValue, Math.Round(value)));
        }

        private static void PutUInt16(byte[] buffer, int offset, ushort value)
        {
            buffer[offset] = (byte)(value & 0xFF);
            buffer[offset + 1] = (byte)(value >> 8);
        }

        private static void PutInt16(byte[] buffer, int offset, short value)
        {
            PutUInt16(buffer, offset, (ushort)value);
        }

        private static void PutUInt32(byte[] buffer, int offset, uint value)
        {
            buffer[offset] = (byte)(value & 0xFF);
            buffer[offset + 1] = (byte)((value >> 8) & 0xFF);
            buffer[offset + 2] = (byte)((value >> 16) & 0xFF);
            buffer[offset + 3] = (byte)((value >> 24) & 0xFF);
        }

        private readonly AP_PhysicsState physics;
        private byte pointer;
        private ushort frameCount;

        private const byte IntegralFrame = 0x16;
        private const int IntegralFrameSize = 25;
        private const uint IntegrationTimeUs = 100000;
        private const byte Quality = 200;
        private const float MinimumHeightM = 0.05f;
    }
}
