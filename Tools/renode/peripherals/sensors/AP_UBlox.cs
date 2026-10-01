// Physics-driven u-blox receiver. It emits a 3D fix using UBX NAV-PVT and
// NAV-TIMEGPS messages understood by the production AP_GPS_UBLOX backend.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
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

    public class AP_NMEAGPS : AP_UARTFrameDevice
    {
        public AP_NMEAGPS(IMachine machine) : base(
            machine, MessagesPerSecond, BaudRateBitsPerSecond)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var truth = physics.Current;
            string latitude;
            string latitudeHemisphere;
            string longitude;
            string longitudeHemisphere;
            FormatCoordinate(
                truth.LatitudeDeg, true, out latitude, out latitudeHemisphere);
            FormatCoordinate(
                truth.LongitudeDeg, false, out longitude,
                out longitudeHemisphere);
            var north = truth.VelocityNedMS[0];
            var east = truth.VelocityNedMS[1];
            var speedKnots = Math.Sqrt(north * north + east * east) /
                MetresPerSecondPerKnot;
            var course = Math.Atan2(east, north) * 180.0 / Math.PI;
            if(course < 0.0)
            {
                course += 360.0;
            }

            var rmc = String.Format(CultureInfo.InvariantCulture,
                "GPRMC,120000.00,A,{0},{1},{2},{3},{4:F3},{5:F3},280826,,,A",
                latitude, latitudeHemisphere, longitude,
                longitudeHemisphere, speedKnots, course);
            var gga = String.Format(CultureInfo.InvariantCulture,
                "GPGGA,120000.00,{0},{1},{2},{3},1,15,1.20,{4:F2},M,0.0,M,,",
                latitude, latitudeHemisphere, longitude,
                longitudeHemisphere, truth.AltitudeM);
            return Encoding.ASCII.GetBytes(
                AddChecksum(rmc) + "\r\n" + AddChecksum(gga) + "\r\n");
        }

        private static void FormatCoordinate(double coordinate, bool latitude,
            out string value, out string hemisphere)
        {
            var positive = coordinate >= 0.0;
            var absolute = Math.Abs(coordinate);
            var degrees = (int)Math.Floor(absolute);
            var minutes = (absolute - degrees) * 60.0;
            value = String.Format(CultureInfo.InvariantCulture,
                latitude ? "{0:00}{1:00.0000000}" : "{0:000}{1:00.0000000}",
                degrees, minutes);
            hemisphere = latitude
                ? (positive ? "N" : "S")
                : (positive ? "E" : "W");
        }

        private static string AddChecksum(string sentence)
        {
            byte checksum = 0;
            foreach(var value in Encoding.ASCII.GetBytes(sentence))
            {
                checksum ^= value;
            }
            return String.Format(CultureInfo.InvariantCulture,
                "${0}*{1:X2}", sentence, checksum);
        }

        private readonly AP_PhysicsState physics;

        private const uint MessagesPerSecond = 5;
        private const uint BaudRateBitsPerSecond = 230400;
        private const double MetresPerSecondPerKnot = 0.514444444444444;
    }

    public class AP_SIRFGPS : AP_UARTFrameDevice
    {
        public AP_SIRFGPS(IMachine machine) : base(
            machine, MessagesPerSecond, BaudRateBitsPerSecond)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var truth = physics.Current;
            var payload = new List<byte> { GeoNavigationMessage };
            AddUInt16(payload, 0);       // fix valid
            AddUInt16(payload, Fix3D);
            AddUInt16(payload, GpsWeek);
            AddUInt32(payload, TimeOfWeekCentiseconds);
            AddUInt16(payload, 2026);
            payload.AddRange(new byte[] { 8, 28, 12, 0 });
            AddUInt16(payload, 0);       // seconds, 0.001 s
            AddUInt32(payload, 0);       // satellites-used bitmap
            AddInt32(payload, ScaledInt32(truth.LatitudeDeg, 1e7));
            AddInt32(payload, ScaledInt32(truth.LongitudeDeg, 1e7));
            var altitudeCm = ScaledInt32(truth.AltitudeM, 100.0);
            AddInt32(payload, altitudeCm);
            AddInt32(payload, altitudeCm);
            payload.Add(0);              // WGS84 map datum
            var north = truth.VelocityNedMS[0];
            var east = truth.VelocityNedMS[1];
            var speed = Math.Sqrt(north * north + east * east);
            AddInt16(payload, ScaledInt16(speed, 100.0));
            var course = Math.Atan2(east, north) * 180.0 / Math.PI;
            if(course < 0.0)
            {
                course += 360.0;
            }
            AddUInt16(payload, (ushort)Math.Round(course * 100.0));
            AddInt16(payload, 0);        // reserved
            AddInt16(payload, ScaledInt16(-truth.VelocityNedMS[2], 100.0));
            AddUInt16(payload, 0);       // heading rate
            AddUInt32(payload, 50);      // horizontal position error, cm
            AddUInt32(payload, 80);      // vertical position error, cm
            AddUInt32(payload, 1000);    // time error
            AddUInt16(payload, 10);      // horizontal velocity error, cm/s
            AddInt32(payload, 0);        // clock bias
            AddUInt32(payload, 0);       // clock bias error
            AddInt32(payload, 0);        // clock drift
            AddUInt32(payload, 0);       // clock drift error
            AddUInt32(payload, 0);       // distance
            AddUInt16(payload, 0);       // distance error
            AddUInt16(payload, 10);      // heading error, 0.01 degrees
            payload.AddRange(new byte[] { 15, 120, 0 });

            var output = new List<byte> { Preamble1, Preamble2 };
            AddUInt16(output, (ushort)payload.Count);
            output.AddRange(payload);
            var checksum = 0;
            foreach(var value in payload)
            {
                checksum = (checksum + value) & 0x7FFF;
            }
            AddUInt16(output, (ushort)checksum);
            output.Add(Postamble1);
            output.Add(Postamble2);
            return output.ToArray();
        }

        private static short ScaledInt16(double value, double scale)
        {
            value *= scale;
            return (short)Math.Max(Int16.MinValue,
                Math.Min(Int16.MaxValue, Math.Round(value)));
        }

        private static int ScaledInt32(double value, double scale)
        {
            value *= scale;
            return (int)Math.Max(Int32.MinValue,
                Math.Min(Int32.MaxValue, Math.Round(value)));
        }

        private static void AddInt16(List<byte> output, short value) =>
            AddUInt16(output, unchecked((ushort)value));

        private static void AddUInt16(List<byte> output, ushort value)
        {
            output.Add((byte)(value >> 8));
            output.Add((byte)value);
        }

        private static void AddInt32(List<byte> output, int value) =>
            AddUInt32(output, unchecked((uint)value));

        private static void AddUInt32(List<byte> output, uint value)
        {
            output.Add((byte)(value >> 24));
            output.Add((byte)(value >> 16));
            output.Add((byte)(value >> 8));
            output.Add((byte)value);
        }

        private readonly AP_PhysicsState physics;

        private const byte Preamble1 = 0xA0;
        private const byte Preamble2 = 0xA2;
        private const byte Postamble1 = 0xB0;
        private const byte Postamble2 = 0xB3;
        private const byte GeoNavigationMessage = 0x29;
        private const ushort Fix3D = 0x06;
        private const ushort GpsWeek = 2434;
        private const uint TimeOfWeekCentiseconds = 43200000;
        private const uint MessagesPerSecond = 5;
        private const uint BaudRateBitsPerSecond = 38400;
    }

    public class AP_ERBGPS : AP_UARTFrameDevice
    {
        public AP_ERBGPS(IMachine machine) : base(
            machine, MessagesPerSecond, BaudRateBitsPerSecond)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            timeOfWeekMs += 1000U / MessagesPerSecond;
            var truth = physics.Current;
            var output = new List<byte>();

            var status = new List<byte>();
            AddUInt32(status, timeOfWeekMs);
            AddUInt16(status, GpsWeek);
            status.Add(FixSingle);
            status.Add(FixValid);
            status.Add(15);
            AppendFrame(output, StatusMessage, status);

            var position = new List<byte>();
            AddUInt32(position, timeOfWeekMs);
            AddDouble(position, truth.LongitudeDeg);
            AddDouble(position, truth.LatitudeDeg);
            AddDouble(position, truth.AltitudeM);
            AddDouble(position, truth.AltitudeM);
            AddUInt32(position, 500);    // horizontal accuracy, mm
            AddUInt32(position, 800);    // vertical accuracy, mm
            AppendFrame(output, PositionMessage, position);

            var velocity = new List<byte>();
            AddUInt32(velocity, timeOfWeekMs);
            AddInt32(velocity, ScaledInt32(truth.VelocityNedMS[0], 100.0));
            AddInt32(velocity, ScaledInt32(truth.VelocityNedMS[1], 100.0));
            AddInt32(velocity, ScaledInt32(truth.VelocityNedMS[2], 100.0));
            var speed = Math.Sqrt(
                truth.VelocityNedMS[0] * truth.VelocityNedMS[0] +
                truth.VelocityNedMS[1] * truth.VelocityNedMS[1]);
            AddUInt32(velocity, (uint)Math.Round(speed * 100.0));
            var course = Math.Atan2(
                truth.VelocityNedMS[1], truth.VelocityNedMS[0]) *
                180.0 / Math.PI;
            if(course < 0.0)
            {
                course += 360.0;
            }
            AddInt32(velocity, ScaledInt32(course, 1e5));
            AddUInt32(velocity, 10);     // speed accuracy, cm/s
            AppendFrame(output, VelocityMessage, velocity);
            return output.ToArray();
        }

        private static void AppendFrame(List<byte> output, byte messageId,
            List<byte> payload)
        {
            var lengthLow = (byte)payload.Count;
            var lengthHigh = (byte)(payload.Count >> 8);
            byte checksumA = messageId;
            byte checksumB = messageId;
            foreach(var value in new byte[] { lengthLow, lengthHigh })
            {
                checksumA += value;
                checksumB += checksumA;
            }
            foreach(var value in payload)
            {
                checksumA += value;
                checksumB += checksumA;
            }
            output.Add(Preamble1);
            output.Add(Preamble2);
            output.Add(messageId);
            output.Add(lengthLow);
            output.Add(lengthHigh);
            output.AddRange(payload);
            output.Add(checksumA);
            output.Add(checksumB);
        }

        private static int ScaledInt32(double value, double scale)
        {
            value *= scale;
            return (int)Math.Max(Int32.MinValue,
                Math.Min(Int32.MaxValue, Math.Round(value)));
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

        private static void AddDouble(List<byte> output, double value)
        {
            var bytes = BitConverter.GetBytes(value);
            if(!BitConverter.IsLittleEndian)
            {
                Array.Reverse(bytes);
            }
            output.AddRange(bytes);
        }

        private readonly AP_PhysicsState physics;
        private uint timeOfWeekMs = InitialTimeOfWeekMs;

        private const byte Preamble1 = 0x45;
        private const byte Preamble2 = 0x52;
        private const byte PositionMessage = 0x02;
        private const byte StatusMessage = 0x03;
        private const byte VelocityMessage = 0x05;
        private const byte FixSingle = 0x01;
        private const byte FixValid = 0x01;
        private const ushort GpsWeek = 2434;
        private const uint InitialTimeOfWeekMs = 432000000;
        private const uint MessagesPerSecond = 5;
        private const uint BaudRateBitsPerSecond = 230400;
    }

    public class AP_NOVAGPS : AP_UARTFrameDevice
    {
        public AP_NOVAGPS(IMachine machine) : base(
            machine, MessagesPerSecond, BaudRateBitsPerSecond)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            timeOfWeekMs += 1000U / MessagesPerSecond;
            var truth = physics.Current;
            var output = new List<byte>();

            var dop = new List<byte>();
            foreach(var value in new float[] { 1.5f, 1.2f, 1.2f, 1.2f, 1.0f, 0.0f })
            {
                AddSingle(dop, value);
            }
            AddUInt32(dop, 15);
            AppendFrame(output, DOPMessage, dop);

            var velocity = new List<byte>();
            AddUInt32(velocity, SolutionComputed);
            AddUInt32(velocity, PositionSingle);
            AddSingle(velocity, 0.0f);
            AddSingle(velocity, 0.0f);
            var speed = Math.Sqrt(
                truth.VelocityNedMS[0] * truth.VelocityNedMS[0] +
                truth.VelocityNedMS[1] * truth.VelocityNedMS[1]);
            AddDouble(velocity, speed);
            var course = Math.Atan2(
                truth.VelocityNedMS[1], truth.VelocityNedMS[0]) *
                180.0 / Math.PI;
            if(course < 0.0)
            {
                course += 360.0;
            }
            AddDouble(velocity, course);
            AddDouble(velocity, -truth.VelocityNedMS[2]);
            AddSingle(velocity, 0.0f);
            AppendFrame(output, VelocityMessage, velocity);

            var position = new List<byte>();
            AddUInt32(position, SolutionComputed);
            AddUInt32(position, PositionSingle);
            AddDouble(position, truth.LatitudeDeg);
            AddDouble(position, truth.LongitudeDeg);
            AddDouble(position, truth.AltitudeM);
            AddSingle(position, 0.0f);   // undulation
            AddUInt32(position, Wgs84Datum);
            AddSingle(position, 0.2f);
            AddSingle(position, 0.2f);
            AddSingle(position, 0.3f);
            AddUInt32(position, 0);      // base station ID
            AddSingle(position, 0.0f);   // differential age
            AddSingle(position, 0.0f);   // solution age
            position.AddRange(new byte[] { 15, 15, 15, 15, 0, 0, 0, 0 });
            AppendFrame(output, PositionMessage, position);
            return output.ToArray();
        }

        private void AppendFrame(List<byte> output, ushort messageId,
            List<byte> payload)
        {
            sequence++;
            var header = new List<byte> { Preamble1, Preamble2, Preamble3,
                                          HeaderLength };
            AddUInt16(header, messageId);
            header.Add(0);               // message type
            header.Add(0);               // port address
            AddUInt16(header, (ushort)payload.Count);
            AddUInt16(header, sequence);
            header.Add(0);               // idle time
            header.Add(TimeStatusFine);
            AddUInt16(header, GpsWeek);
            AddUInt32(header, timeOfWeekMs);
            AddUInt32(header, 0);        // receiver status
            AddUInt16(header, 0);
            AddUInt16(header, 0);        // receiver software version

            var crc = Crc32(0, header);
            crc = Crc32(crc, payload);
            output.AddRange(header);
            output.AddRange(payload);
            AddUInt32(output, crc);
        }

        private static uint Crc32(uint crc, IEnumerable<byte> values)
        {
            foreach(var value in values)
            {
                crc ^= value;
                for(var bit = 0; bit < 8; bit++)
                {
                    var mask = unchecked((uint)-(int)(crc & 1));
                    crc = (crc >> 1) ^ (CrcPolynomial & mask);
                }
            }
            return crc;
        }

        private static void AddUInt16(List<byte> output, ushort value)
        {
            output.Add((byte)value);
            output.Add((byte)(value >> 8));
        }

        private static void AddUInt32(List<byte> output, uint value)
        {
            output.Add((byte)value);
            output.Add((byte)(value >> 8));
            output.Add((byte)(value >> 16));
            output.Add((byte)(value >> 24));
        }

        private static void AddSingle(List<byte> output, float value)
        {
            AddBytes(output, BitConverter.GetBytes(value));
        }

        private static void AddDouble(List<byte> output, double value)
        {
            AddBytes(output, BitConverter.GetBytes(value));
        }

        private static void AddBytes(List<byte> output, byte[] bytes)
        {
            if(!BitConverter.IsLittleEndian)
            {
                Array.Reverse(bytes);
            }
            output.AddRange(bytes);
        }

        private readonly AP_PhysicsState physics;
        private uint timeOfWeekMs = InitialTimeOfWeekMs;
        private ushort sequence;

        private const byte Preamble1 = 0xAA;
        private const byte Preamble2 = 0x44;
        private const byte Preamble3 = 0x12;
        private const byte HeaderLength = 28;
        private const byte TimeStatusFine = 160;
        private const ushort PositionMessage = 42;
        private const ushort VelocityMessage = 99;
        private const ushort DOPMessage = 174;
        private const ushort GpsWeek = 2434;
        private const uint InitialTimeOfWeekMs = 432000000;
        private const uint SolutionComputed = 0;
        private const uint PositionSingle = 16;
        private const uint Wgs84Datum = 61;
        private const uint CrcPolynomial = 0xEDB88320;
        private const uint MessagesPerSecond = 5;
        private const uint BaudRateBitsPerSecond = 19200;
    }

    public abstract class AP_SBPGPSBase : AP_UARTFrameDevice
    {
        protected AP_SBPGPSBase(IMachine machine, bool version2) : base(
            machine, MessagesPerSecond, BaudRateBitsPerSecond)
        {
            this.version2 = version2;
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            timeOfWeekMs = InitialTimeOfWeekMs;
        }

        protected override byte[] BuildFrame()
        {
            timeOfWeekMs += 1000U / MessagesPerSecond;
            var truth = physics.Current;
            var output = new List<byte>();

            AppendFrame(output, HeartbeatMessage,
                new List<byte> { 0, 0, version2 ? (byte)2 : (byte)0, 0 });

            var time = new List<byte>();
            AddUInt16(time, GpsWeek);
            AddUInt32(time, timeOfWeekMs);
            AddInt32(time, 0);           // nanosecond remainder
            time.Add(version2 ? (byte)1 : (byte)0);
            AppendFrame(output, version2 ? GpsTimeV2Message : GpsTimeMessage,
                time);

            var dops = new List<byte>();
            AddUInt32(dops, timeOfWeekMs);
            AddUInt16(dops, 150);        // GDOP, 0.01
            AddUInt16(dops, 120);        // PDOP, 0.01
            AddUInt16(dops, 100);        // TDOP, 0.01
            AddUInt16(dops, 100);        // HDOP, 0.01
            AddUInt16(dops, 120);        // VDOP, 0.01
            if(version2)
            {
                dops.Add(FixSingle);
            }
            AppendFrame(output, version2 ? DopsV2Message : DopsMessage, dops);

            var position = new List<byte>();
            AddUInt32(position, timeOfWeekMs);
            AddDouble(position, truth.LatitudeDeg);
            AddDouble(position, truth.LongitudeDeg);
            AddDouble(position, truth.AltitudeM);
            AddUInt16(position, 500);    // horizontal accuracy, mm
            AddUInt16(position, 800);    // vertical accuracy, mm
            position.Add(15);
            position.Add(version2 ? FixRtkFixedV2 : FixRtkFixed);
            AppendFrame(output,
                version2 ? PositionV2Message : PositionMessage, position);

            var velocity = new List<byte>();
            AddUInt32(velocity, timeOfWeekMs);
            AddInt32(velocity,
                ScaledInt32(truth.VelocityNedMS[0], 1000.0));
            AddInt32(velocity,
                ScaledInt32(truth.VelocityNedMS[1], 1000.0));
            AddInt32(velocity,
                ScaledInt32(truth.VelocityNedMS[2], 1000.0));
            AddUInt16(velocity, 100);    // horizontal accuracy, mm/s
            AddUInt16(velocity, 100);    // vertical accuracy, mm/s
            velocity.Add(15);
            velocity.Add(version2 ? VelocityMeasured : (byte)0);
            AppendFrame(output,
                version2 ? VelocityV2Message : VelocityMessage, velocity);
            return output.ToArray();
        }

        private static int ScaledInt32(double value, double scale)
        {
            value *= scale;
            return (int)Math.Max(Int32.MinValue,
                Math.Min(Int32.MaxValue, Math.Round(value)));
        }

        private static void AppendFrame(List<byte> output, ushort messageType,
            List<byte> payload)
        {
            var body = new List<byte>();
            AddUInt16(body, messageType);
            AddUInt16(body, SenderId);
            body.Add((byte)payload.Count);
            body.AddRange(payload);
            var crc = Crc16Ccitt(body);
            output.Add(Preamble);
            output.AddRange(body);
            AddUInt16(output, crc);
        }

        private static ushort Crc16Ccitt(IEnumerable<byte> values)
        {
            ushort crc = 0;
            foreach(var value in values)
            {
                crc ^= (ushort)(value << 8);
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (ushort)((crc & 0x8000) != 0
                        ? (crc << 1) ^ CrcPolynomial
                        : crc << 1);
                }
            }
            return crc;
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

        private static void AddDouble(List<byte> output, double value)
        {
            var bytes = BitConverter.GetBytes(value);
            if(!BitConverter.IsLittleEndian)
            {
                Array.Reverse(bytes);
            }
            output.AddRange(bytes);
        }

        private readonly AP_PhysicsState physics;
        private readonly bool version2;
        private uint timeOfWeekMs = InitialTimeOfWeekMs;

        private const byte Preamble = 0x55;
        private const byte FixSingle = 1;
        private const byte FixRtkFixed = 1;
        private const byte FixRtkFixedV2 = 4;
        private const byte VelocityMeasured = 1;
        private const ushort SenderId = 0x2222;
        private const ushort GpsWeek = 2434;
        private const ushort HeartbeatMessage = 0xFFFF;
        private const ushort GpsTimeMessage = 0x0100;
        private const ushort PositionMessage = 0x0201;
        private const ushort VelocityMessage = 0x0205;
        private const ushort DopsMessage = 0x0206;
        private const ushort GpsTimeV2Message = 0x0102;
        private const ushort DopsV2Message = 0x0208;
        private const ushort PositionV2Message = 0x020A;
        private const ushort VelocityV2Message = 0x020E;
        private const ushort CrcPolynomial = 0x1021;
        private const uint InitialTimeOfWeekMs = 432000000;
        private const uint MessagesPerSecond = 5;
        private const uint BaudRateBitsPerSecond = 115200;
    }

    public class AP_SBPGPS : AP_SBPGPSBase
    {
        public AP_SBPGPS(IMachine machine) : base(machine, false)
        {
        }
    }

    public class AP_SBP2GPS : AP_SBPGPSBase
    {
        public AP_SBP2GPS(IMachine machine) : base(machine, true)
        {
        }
    }

    public class AP_SBFGPS : AP_UARTFrameDevice
    {
        public AP_SBFGPS(IMachine machine) : base(
            machine, MessagesPerSecond, BaudRateBitsPerSecond)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            command.Clear();
            Configured = false;
            configurationPending = false;
            configurationSettlingFrames = 0;
            timeOfWeekMs = InitialTimeOfWeekMs;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            if(value == (byte)'\n')
            {
                ProcessCommand();
                command.Clear();
                return;
            }
            if(value != (byte)'\r' && command.Count < MaximumCommandLength)
            {
                command.Add(value);
            }
        }

        protected override byte[] BuildFrame()
        {
            if(configurationPending)
            {
                configurationSettlingFrames++;
                if(configurationSettlingFrames >= ConfigurationSettlingFrames)
                {
                    configurationPending = false;
                    Configured = true;
                }
                return new byte[0];
            }
            if(!Configured)
            {
                return new byte[0];
            }

            timeOfWeekMs += 1000U / MessagesPerSecond;
            var truth = physics.Current;
            var output = new List<byte>();

            var position = new List<byte>();
            AddUInt32(position, timeOfWeekMs);
            AddUInt16(position, GpsWeek);
            position.Add(FixRtkFixed);
            position.Add(0);             // no PVT error
            AddDouble(position, truth.LatitudeDeg * Math.PI / 180.0);
            AddDouble(position, truth.LongitudeDeg * Math.PI / 180.0);
            AddDouble(position, truth.AltitudeM);
            AddSingle(position, DoNotUse);
            AddSingle(position, (float)truth.VelocityNedMS[0]);
            AddSingle(position, (float)truth.VelocityNedMS[1]);
            AddSingle(position, (float)-truth.VelocityNedMS[2]);
            var course = Math.Atan2(
                truth.VelocityNedMS[1], truth.VelocityNedMS[0]) *
                180.0 / Math.PI;
            if(course < 0.0)
            {
                course += 360.0;
            }
            AddSingle(position, (float)course);
            AddDouble(position, DoNotUse);
            AddSingle(position, DoNotUse);
            position.Add(255);           // time system unavailable
            position.Add(255);           // datum unavailable
            position.Add(15);            // satellites
            position.Add(0);             // wide-area correction info
            AddUInt16(position, UInt16.MaxValue);
            AddUInt16(position, UInt16.MaxValue);
            AddUInt32(position, 0);       // signal info
            position.Add(0);             // alert flag
            position.Add(0);             // base stations
            AddUInt16(position, 0);       // PPP info
            AddUInt16(position, 0);       // latency
            AddUInt16(position, 100);     // 0.50 m horizontal accuracy
            AddUInt16(position, 160);     // 0.80 m vertical accuracy
            position.Add(0);              // miscellaneous flags
            position.Add(0);              // four-byte packet alignment
            AppendBlock(output, PositionMessage, position);

            var dops = new List<byte>();
            AddUInt32(dops, timeOfWeekMs);
            AddUInt16(dops, GpsWeek);
            dops.Add(15);
            dops.Add(0);
            AddUInt16(dops, 120);
            AddUInt16(dops, 100);
            AddUInt16(dops, 100);
            AddUInt16(dops, 120);
            AddSingle(dops, 1.0f);
            AddSingle(dops, 1.0f);
            AppendBlock(output, DopsMessage, dops);
            return output.ToArray();
        }

        private void ProcessCommand()
        {
            if(command.Count == 0)
            {
                return;
            }
            var text = Encoding.ASCII.GetString(command.ToArray());
            if(text.Contains("SSSSSSSSSS"))
            {
                TransmitFrame(Encoding.ASCII.GetBytes("COM1>"));
                return;
            }
            var response = Encoding.ASCII.GetBytes(
                "$R: " + text + "\r\nCOM1>");
            TransmitFrame(response);
            if(text.StartsWith("sso,Stream2,Dsk1",
                StringComparison.OrdinalIgnoreCase))
            {
                configurationPending = true;
                configurationSettlingFrames = 0;
            }
            else if(text.StartsWith("snt,", StringComparison.OrdinalIgnoreCase) ||
                text.StartsWith("sst,", StringComparison.OrdinalIgnoreCase) ||
                text.StartsWith("ssbc,", StringComparison.OrdinalIgnoreCase))
            {
                configurationPending = false;
                configurationSettlingFrames = 0;
            }
            else if(text.StartsWith("sga,", StringComparison.OrdinalIgnoreCase))
            {
                configurationPending = true;
                configurationSettlingFrames = 0;
            }
        }

        private static void AppendBlock(List<byte> output, ushort blockId,
            List<byte> payload)
        {
            var length = checked((ushort)(payload.Count + HeaderLength));
            var checkedBytes = new List<byte>();
            AddUInt16(checkedBytes, blockId);
            AddUInt16(checkedBytes, length);
            checkedBytes.AddRange(payload);
            var crc = Crc16Ccitt(checkedBytes);
            output.Add((byte)'$');
            output.Add((byte)'@');
            AddUInt16(output, crc);
            output.AddRange(checkedBytes);
        }

        private static ushort Crc16Ccitt(IEnumerable<byte> values)
        {
            ushort crc = 0;
            foreach(var value in values)
            {
                crc ^= (ushort)(value << 8);
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (ushort)((crc & 0x8000) != 0
                        ? (crc << 1) ^ CrcPolynomial
                        : crc << 1);
                }
            }
            return crc;
        }

        private static void AddUInt16(List<byte> output, ushort value)
        {
            output.Add((byte)value);
            output.Add((byte)(value >> 8));
        }

        private static void AddUInt32(List<byte> output, uint value)
        {
            output.Add((byte)value);
            output.Add((byte)(value >> 8));
            output.Add((byte)(value >> 16));
            output.Add((byte)(value >> 24));
        }

        private static void AddSingle(List<byte> output, float value)
        {
            AddBytes(output, BitConverter.GetBytes(value));
        }

        private static void AddDouble(List<byte> output, double value)
        {
            AddBytes(output, BitConverter.GetBytes(value));
        }

        private static void AddBytes(List<byte> output, byte[] bytes)
        {
            if(!BitConverter.IsLittleEndian)
            {
                Array.Reverse(bytes);
            }
            output.AddRange(bytes);
        }

        private readonly AP_PhysicsState physics;
        private readonly List<byte> command = new List<byte>();
        public bool Configured { get; private set; }

        private bool configurationPending;
        private uint configurationSettlingFrames;
        private uint timeOfWeekMs = InitialTimeOfWeekMs;

        private const ushort PositionMessage = 4007;
        private const ushort DopsMessage = 4001;
        private const ushort GpsWeek = 2434;
        private const ushort CrcPolynomial = 0x1021;
        private const byte FixRtkFixed = 4;
        private const ushort HeaderLength = 8;
        private const int MaximumCommandLength = 255;
        private const uint ConfigurationSettlingFrames = 2;
        private const float DoNotUse = -2e10f;
        private const uint InitialTimeOfWeekMs = 432000000;
        private const uint MessagesPerSecond = 5;
        private const uint BaudRateBitsPerSecond = 230400;
    }

    public class AP_GSOFGPS : AP_UARTFrameDevice
    {
        public AP_GSOFGPS(IMachine machine) : base(machine, 5, 230400)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            command.Clear();
            requestedRecords.Clear();
            Configured = false;
            timeOfWeekMs = 432000000;
            sequence = 0;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            if(command.Count == 0 && value != Stx)
            {
                return;
            }
            command.Add(value);
            if(command.Count < DcolHeaderLength ||
                command.Count < command[3] + DcolFramingLength)
            {
                return;
            }
            var checksum = (byte)0;
            for(var index = 1; index < command.Count - 2; index++)
            {
                checksum += command[index];
            }
            var valid = command[command.Count - 1] == Etx &&
                command[command.Count - 2] == checksum;
            if(valid && command.Count >= 21 && command[2] == 0x64 &&
                command[11] == 0x07 && command[12] == 0x06 &&
                command[13] == 0x0A)
            {
                requestedRecords.Add(command[17]);
                Configured = RequiredRecords.IsSubsetOf(requestedRecords);
            }
            TransmitFrame(new byte[] { valid ? Ack : Nack });
            command.Clear();
        }

        protected override byte[] BuildFrame()
        {
            if(!Configured)
            {
                return new byte[0];
            }
            timeOfWeekMs += 200;
            var truth = physics.Current;
            var records = new List<byte>();

            AddRecord(records, 1, record => {
                AddUInt32(record, timeOfWeekMs);
                AddUInt16(record, 2434);
                record.AddRange(new byte[] { 15, 0xAF, 0x07, 17 });
            });
            AddRecord(records, 2, record => {
                AddDouble(record, truth.LatitudeDeg * Math.PI / 180.0);
                AddDouble(record, truth.LongitudeDeg * Math.PI / 180.0);
                AddDouble(record, truth.AltitudeM);
            });
            AddRecord(records, 8, record => {
                record.Add(0x07);
                var north = truth.VelocityNedMS[0];
                var east = truth.VelocityNedMS[1];
                AddSingle(record, (float)Math.Sqrt(north * north + east * east));
                var heading = Math.Atan2(east, north);
                if(heading < 0.0)
                {
                    heading += 2.0 * Math.PI;
                }
                AddSingle(record, (float)heading);
                AddSingle(record, (float)-truth.VelocityNedMS[2]);
            });
            AddRecord(records, 9, record => {
                foreach(var value in new float[] { 1.2f, 1.0f, 1.2f, 1.0f })
                {
                    AddSingle(record, value);
                }
            });
            AddRecord(records, 12, record => {
                foreach(var value in new float[] {
                    0.5f, 0.5f, 0.5f, 0.0f, 0.8f,
                    0.5f, 0.5f, 0.0f, 1.0f })
                {
                    AddSingle(record, value);
                }
                AddUInt16(record, 1);
            });

            var data = new List<byte> { sequence++, 0, 0 };
            data.AddRange(records);
            var output = new List<byte> { Stx, 0xA8, 0x40, (byte)data.Count };
            output.AddRange(data);
            byte checksum = 0;
            for(var index = 1; index < output.Count; index++)
            {
                checksum += output[index];
            }
            output.Add(checksum);
            output.Add(Etx);
            return output.ToArray();
        }

        private static void AddRecord(List<byte> output, byte type,
            Action<List<byte>> populate)
        {
            var payload = new List<byte>();
            populate(payload);
            output.Add(type);
            output.Add((byte)payload.Count);
            output.AddRange(payload);
        }
        private static void AddUInt16(List<byte> output, ushort value)
        {
            output.Add((byte)(value >> 8));
            output.Add((byte)value);
        }
        private static void AddUInt32(List<byte> output, uint value)
        {
            output.Add((byte)(value >> 24));
            output.Add((byte)(value >> 16));
            output.Add((byte)(value >> 8));
            output.Add((byte)value);
        }
        private static void AddSingle(List<byte> output, float value) =>
            AddBytes(output, BitConverter.GetBytes(value));
        private static void AddDouble(List<byte> output, double value) =>
            AddBytes(output, BitConverter.GetBytes(value));
        private static void AddBytes(List<byte> output, byte[] bytes)
        {
            if(BitConverter.IsLittleEndian)
            {
                Array.Reverse(bytes);
            }
            output.AddRange(bytes);
        }

        public bool Configured { get; private set; }
        private readonly AP_PhysicsState physics;
        private readonly List<byte> command = new List<byte>();
        private readonly HashSet<byte> requestedRecords = new HashSet<byte>();
        private static readonly HashSet<byte> RequiredRecords =
            new HashSet<byte> { 1, 2, 8, 9, 12 };
        private uint timeOfWeekMs = 432000000;
        private byte sequence;
        private const byte Stx = 0x02;
        private const byte Etx = 0x03;
        private const byte Ack = 0x06;
        private const byte Nack = 0x15;
        private const int DcolHeaderLength = 4;
        private const int DcolFramingLength = 6;
    }
}
