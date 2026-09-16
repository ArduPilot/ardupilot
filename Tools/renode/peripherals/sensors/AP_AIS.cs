// Physics-driven NMEA 0183 AIS receiver.
using System;
using System.Globalization;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_AISReceiver : AP_UARTFrameDevice
    {
        public AP_AISReceiver(IMachine machine) :
            base(machine, FramesPerSecond, BaudRate)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            LatitudeOffsetDegrees = DefaultLatitudeOffsetDegrees;
            LongitudeOffsetDegrees = DefaultLongitudeOffsetDegrees;
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var truth = physics.Current;
            var latitude = truth.LatitudeDeg + LatitudeOffsetDegrees;
            var longitude = truth.LongitudeDeg + LongitudeOffsetDegrees;
            var speed = Math.Sqrt(
                truth.VelocityNedMS[0] * truth.VelocityNedMS[0] +
                truth.VelocityNedMS[1] * truth.VelocityNedMS[1]);
            var course = Math.Atan2(
                truth.VelocityNedMS[1], truth.VelocityNedMS[0]) *
                180.0 / Math.PI;
            if(course < 0.0)
            {
                course += 360.0;
            }

            var sequence = sequenceId;
            sequenceId = (sequenceId + 1) % 10;
            var staticFrames = BuildStaticFrames(sequence);
            var position = BuildPositionFrame(latitude, longitude, speed, course);
            var result = new byte[staticFrames.Length + position.Length];
            Array.Copy(staticFrames, result, staticFrames.Length);
            Array.Copy(position, 0, result, staticFrames.Length, position.Length);
            return result;
        }

        public double LatitudeOffsetDegrees { get; set; }
        public double LongitudeOffsetDegrees { get; set; }
        public bool CorruptChecksum { get; set; }

        private byte[] BuildPositionFrame(double latitude, double longitude,
            double speedMS, double courseDegrees)
        {
            var payload = new byte[28];
            SetBits(payload, 0, 5, 1);
            SetBits(payload, 8, 37, Mmsi);
            SetBits(payload, 38, 41, NavigationalStatusUnderWay);
            SetBits(payload, 42, 49, -128);
            SetBits(payload, 50, 59, Math.Min(1022,
                Math.Round(speedMS * MetresPerSecondToKnots * 10.0)));
            SetBits(payload, 60, 60, 1);
            SetBits(payload, 61, 88, Math.Round(longitude * 600000.0));
            SetBits(payload, 89, 115, Math.Round(latitude * 600000.0));
            SetBits(payload, 116, 127, Math.Round(courseDegrees * 10.0));
            SetBits(payload, 128, 136, Math.Round(courseDegrees) % 360);
            SetBits(payload, 137, 142, 60);
            return BuildSentence(String.Format(
                CultureInfo.InvariantCulture, "AIVDM,1,1,,A,{0},0",
                Armor(payload)));
        }

        private byte[] BuildStaticFrames(int sequence)
        {
            var payload = new byte[71];
            SetBits(payload, 0, 5, 5);
            SetBits(payload, 8, 37, Mmsi);
            SetBits(payload, 38, 39, 1);
            SetText(payload, 70, 111, Callsign);
            SetText(payload, 112, 231, VesselName);
            SetBits(payload, 232, 239, VesselTypeCargo);
            SetBits(payload, 240, 248, DimensionBowM);
            SetBits(payload, 249, 257, DimensionSternM);
            SetBits(payload, 258, 263, DimensionPortM);
            SetBits(payload, 264, 269, DimensionStarboardM);
            SetBits(payload, 270, 273, 1);
            SetText(payload, 302, 421, String.Empty);

            var armored = Armor(payload);
            var first = BuildSentence(String.Format(
                CultureInfo.InvariantCulture, "AIVDM,2,1,{0},A,{1},0",
                sequence, armored.Substring(0, FirstFragmentCharacters)));
            var second = BuildSentence(String.Format(
                CultureInfo.InvariantCulture, "AIVDM,2,2,{0},A,{1},2",
                sequence, armored.Substring(FirstFragmentCharacters)));
            var result = new byte[first.Length + second.Length];
            Array.Copy(first, result, first.Length);
            Array.Copy(second, 0, result, first.Length, second.Length);
            return result;
        }

        private byte[] BuildSentence(string body)
        {
            byte checksum = 0;
            foreach(var value in Encoding.ASCII.GetBytes(body))
            {
                checksum ^= value;
            }
            if(CorruptChecksum)
            {
                checksum ^= 0xFF;
            }
            return Encoding.ASCII.GetBytes(String.Format(
                CultureInfo.InvariantCulture, "!{0}*{1:X2}\r\n", body,
                checksum));
        }

        private static string Armor(byte[] payload)
        {
            var result = new char[payload.Length];
            for(var index = 0; index < payload.Length; index++)
            {
                var value = payload[index];
                result[index] = (char)(value + (value > 39 ? 56 : 48));
            }
            return new string(result);
        }

        private static void SetText(byte[] payload, int low, int high,
            string value)
        {
            var characters = (high - low + 1) / 6;
            for(var index = 0; index < characters; index++)
            {
                var encoded = index < value.Length ?
                    (byte)(Char.ToUpperInvariant(value[index]) & 0x3F) :
                    (byte)0;
                SetBits(payload, low + index * 6, low + index * 6 + 5,
                    encoded);
            }
        }

        private static void SetBits(byte[] payload, int low, int high,
            double value)
        {
            var width = high - low + 1;
            var signedValue = (long)value;
            var mask = (1UL << width) - 1;
            var encoded = (ulong)signedValue & mask;
            for(var bit = low; bit <= high; bit++)
            {
                var source = high - bit;
                if(((encoded >> source) & 1) != 0)
                {
                    payload[bit / 6] |= (byte)(1 << (5 - bit % 6));
                }
            }
        }

        private readonly AP_PhysicsState physics;
        private int sequenceId;

        private const uint FramesPerSecond = 2;
        private const uint BaudRate = 38400;
        private const int FirstFragmentCharacters = 50;
        private const uint Mmsi = 123456789;
        private const int NavigationalStatusUnderWay = 0;
        private const int VesselTypeCargo = 70;
        private const int DimensionBowM = 40;
        private const int DimensionSternM = 15;
        private const int DimensionPortM = 6;
        private const int DimensionStarboardM = 8;
        private const string Callsign = "RENODE";
        private const string VesselName = "RENODE VESSEL";
        private const double DefaultLatitudeOffsetDegrees = 0.01;
        private const double DefaultLongitudeOffsetDegrees = -0.02;
        private const double MetresPerSecondToKnots = 1.9438444924406;
    }
}
