// Physics-driven serial sensors using NMEA 0183 sentences.
using System;
using System.Globalization;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_NMEASerialDevice : AP_UARTFrameDevice
    {
        protected AP_NMEASerialDevice(IMachine machine, uint framesPerSecond,
            uint baudRate = 4800) : base(machine, framesPerSecond, baudRate)
        {
        }

        protected static byte[] BuildSentence(string body)
        {
            byte checksum = 0;
            foreach(var value in Encoding.ASCII.GetBytes(body))
            {
                checksum ^= value;
            }
            return Encoding.ASCII.GetBytes(String.Format(
                CultureInfo.InvariantCulture, "${0}*{1:X2}\r\n", body, checksum));
        }
    }

    public class AP_NMEAAirspeed : AP_NMEASerialDevice
    {
        public AP_NMEAAirspeed(IMachine machine) :
            base(machine, FramesPerSecond, BaudRate)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var truth = physics.Current;
            var speedKmh = Math.Max(0.0, truth.AirspeedMS) * KilometresPerHourPerMetrePerSecond;
            var temperatureC = truth.TemperatureK - CelsiusToKelvin;
            var speed = BuildSentence(String.Format(
                CultureInfo.InvariantCulture,
                "IIVHW,0.0,T,0.0,M,0.0,N,{0:F2},K", speedKmh));
            var temperature = BuildSentence(String.Format(
                CultureInfo.InvariantCulture,
                "IIMTW,{0:F2},C", temperatureC));
            var frame = new byte[speed.Length + temperature.Length];
            Array.Copy(speed, 0, frame, 0, speed.Length);
            Array.Copy(temperature, 0, frame, speed.Length, temperature.Length);
            return frame;
        }

        private readonly AP_PhysicsState physics;
        private const uint FramesPerSecond = 5;
        private const uint BaudRate = 4800;
        private const double KilometresPerHourPerMetrePerSecond = 3.6;
        private const double CelsiusToKelvin = 273.15;
    }

    public class AP_NMEAWindVane : AP_NMEASerialDevice
    {
        public AP_NMEAWindVane(IMachine machine) : base(machine, FramesPerSecond)
        {
            WindSpeedMS = DefaultWindSpeedMS;
            WindDirectionDegrees = DefaultWindDirectionDegrees;
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var direction = WindDirectionDegrees % 360.0;
            if(direction < 0.0)
            {
                direction += 360.0;
            }
            return BuildSentence(String.Format(
                CultureInfo.InvariantCulture,
                "IIMWV,{0:F2},R,{1:F2},M,A",
                direction, Math.Max(0.0, WindSpeedMS)));
        }

        public double WindSpeedMS { get; set; }
        public double WindDirectionDegrees { get; set; }

        private const uint FramesPerSecond = 5;
        private const double DefaultWindSpeedMS = 5.0;
        private const double DefaultWindDirectionDegrees = 45.0;
    }
}
