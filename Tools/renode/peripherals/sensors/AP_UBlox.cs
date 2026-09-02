// Physics-driven u-blox receiver. It emits a 3D fix using UBX NAV-PVT and
// NAV-TIMEGPS messages understood by the production AP_GPS_UBLOX backend.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_UBlox : AP_UARTFrameDevice
    {
        public AP_UBlox(IMachine machine) : base(
            machine, MessagesPerSecond, BaudRateBitsPerSecond)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            itow = InitialTimeOfWeekMs;
        }

        protected override byte[] BuildFrame()
        {
            itow += 1000U / MessagesPerSecond;
            var truth = physics.Current;
            var output = new List<byte>();

            var pvt = new List<byte>();
            AddUInt32(pvt, itow);
            AddUInt16(pvt, 2026);
            pvt.AddRange(new byte[] { 8, 8, 12, 0, 0, 0x07 });
            AddUInt32(pvt, 10000);       // UTC time accuracy, ns
            AddInt32(pvt, 0);            // fractional UTC time, ns
            pvt.AddRange(new byte[] { 3, 1, 0, 15 }); // 3D fix, valid, satellites
            AddInt32(pvt, ScaledInt32(truth.LongitudeDeg, 1e7));
            AddInt32(pvt, ScaledInt32(truth.LatitudeDeg, 1e7));
            AddInt32(pvt, ScaledInt32(truth.AltitudeM, 1000.0));
            AddInt32(pvt, ScaledInt32(truth.AltitudeM, 1000.0));
            AddUInt32(pvt, 500);         // horizontal accuracy, mm
            AddUInt32(pvt, 800);         // vertical accuracy, mm
            AddInt32(pvt, ScaledInt32(truth.VelocityNedMS[0], 1000.0));
            AddInt32(pvt, ScaledInt32(truth.VelocityNedMS[1], 1000.0));
            AddInt32(pvt, ScaledInt32(truth.VelocityNedMS[2], 1000.0));
            var groundSpeed = Math.Sqrt(
                truth.VelocityNedMS[0] * truth.VelocityNedMS[0] +
                truth.VelocityNedMS[1] * truth.VelocityNedMS[1]);
            AddInt32(pvt, ScaledInt32(groundSpeed, 1000.0));
            var courseDeg = Math.Atan2(
                truth.VelocityNedMS[1], truth.VelocityNedMS[0]) * 180.0 / Math.PI;
            if(courseDeg < 0.0)
            {
                courseDeg += 360.0;
            }
            AddInt32(pvt, ScaledInt32(courseDeg, 1e5));
            AddUInt32(pvt, 100);         // speed accuracy, mm/s
            AddUInt32(pvt, 10000);       // heading accuracy, 1e-5 degrees
            AddUInt16(pvt, 120);         // position DOP, 0.01
            pvt.AddRange(new byte[6]);    // flags3 and reserved1
            AddInt32(pvt, 0);            // vehicle heading
            AddInt16(pvt, 0);            // magnetic declination
            AddUInt16(pvt, 0);           // magnetic accuracy
            AppendFrame(output, 0x01, 0x07, pvt);

            var timeGps = new List<byte>();
            AddUInt32(timeGps, itow);
            AddInt32(timeGps, 0);
            AddUInt16(timeGps, GpsWeek);
            timeGps.Add(18);             // leap seconds
            timeGps.Add(0x07);           // TOW, week and leap seconds valid
            AddUInt32(timeGps, 10000);
            AppendFrame(output, 0x01, 0x20, timeGps);
            return output.ToArray();
        }

        private void AppendFrame(List<byte> output, byte messageClass,
            byte messageId, List<byte> payload)
        {
            var body = new List<byte> {
                messageClass, messageId,
                (byte)payload.Count, (byte)(payload.Count >> 8),
            };
            body.AddRange(payload);
            byte checksumA = 0;
            byte checksumB = 0;
            foreach(var value in body)
            {
                checksumA += value;
                checksumB += checksumA;
            }
            output.Add(0xB5);
            output.Add(0x62);
            output.AddRange(body);
            output.Add(checksumA);
            output.Add(checksumB);
        }

        private static void AddInt16(List<byte> output, short value) =>
            AddUInt16(output, unchecked((ushort)value));
        private static int ScaledInt32(double value, double scale)
        {
            value *= scale;
            if(value > Int32.MaxValue)
            {
                return Int32.MaxValue;
            }
            if(value < Int32.MinValue)
            {
                return Int32.MinValue;
            }
            return (int)Math.Round(value);
        }
        private static void AddUInt16(List<byte> output, ushort value)
        {
            output.Add((byte)value);
            output.Add((byte)(value >> 8));
        }
        private static void AddInt32(List<byte> output, int value) =>
            AddUInt32(output, unchecked((uint)value));
        private static void AddUInt32(List<byte> output, uint value)
        {
            output.Add((byte)value);
            output.Add((byte)(value >> 8));
            output.Add((byte)(value >> 16));
            output.Add((byte)(value >> 24));
        }

        private readonly AP_PhysicsState physics;
        private uint itow = InitialTimeOfWeekMs;

        private const uint MessagesPerSecond = 5;
        private const uint BaudRateBitsPerSecond = 230400;
        private const uint InitialTimeOfWeekMs = 518400000;
        private const ushort GpsWeek = 2430;
    }
}
