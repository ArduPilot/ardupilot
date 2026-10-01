// Receiver and analyser for ArduPilot NMEA output.
using System;
using System.Globalization;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_NMEAOutputAnalyzer : AP_UARTFrameDevice
    {
        public AP_NMEAOutputAnalyzer(IMachine machine) : base(machine, 1, BaudRate)
        {
            line = new StringBuilder();
        }

        protected override byte[] BuildFrame()
        {
            return new byte[0];
        }

        public override void Reset()
        {
            base.Reset();
            line.Clear();
            ValidSentences = 0;
            InvalidSentences = 0;
            GGASentences = 0;
            RMCSentences = 0;
            PASHRSentences = 0;
            LastLatitudeE7 = 0;
            LastLongitudeE7 = 0;
            LastAltitudeCm = 0;
            LastSpeedCentiKnots = 0;
            LastCourseCentiDegrees = 0;
            LastPashrHeadingCentiDegrees = 0;
            LastFixQuality = 0;
            LastSatellites = 0;
            LastRmcValid = false;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            if(value == '\n')
            {
                ParseLine(line.ToString());
                line.Clear();
                return;
            }
            if(value == '\r')
            {
                return;
            }
            if(line.Length >= MaximumSentenceLength)
            {
                line.Clear();
                InvalidSentences++;
            }
            line.Append((char)value);
        }

        public ulong ValidSentences { get; private set; }
        public ulong InvalidSentences { get; private set; }
        public ulong GGASentences { get; private set; }
        public ulong RMCSentences { get; private set; }
        public ulong PASHRSentences { get; private set; }
        public int LastLatitudeE7 { get; private set; }
        public int LastLongitudeE7 { get; private set; }
        public int LastAltitudeCm { get; private set; }
        public int LastSpeedCentiKnots { get; private set; }
        public int LastCourseCentiDegrees { get; private set; }
        public int LastPashrHeadingCentiDegrees { get; private set; }
        public int LastFixQuality { get; private set; }
        public int LastSatellites { get; private set; }
        public bool LastRmcValid { get; private set; }

        private void ParseLine(string sentence)
        {
            var checksumOffset = sentence.LastIndexOf('*');
            if(sentence.Length < 5 || sentence[0] != '$' ||
                checksumOffset < 2 || checksumOffset + 3 != sentence.Length)
            {
                InvalidSentences++;
                return;
            }
            byte expectedChecksum;
            if(!Byte.TryParse(sentence.Substring(checksumOffset + 1, 2),
                NumberStyles.HexNumber, CultureInfo.InvariantCulture,
                out expectedChecksum))
            {
                InvalidSentences++;
                return;
            }
            byte checksum = 0;
            for(var index = 1; index < checksumOffset; index++)
            {
                checksum ^= (byte)sentence[index];
            }
            if(checksum != expectedChecksum)
            {
                InvalidSentences++;
                return;
            }
            ValidSentences++;
            var fields = sentence.Substring(
                1, checksumOffset - 1).Split(',');
            if(fields[0] == "GPGGA")
            {
                ParseGga(fields);
            }
            else if(fields[0] == "GPRMC")
            {
                ParseRmc(fields);
            }
            else if(fields[0] == "PASHR")
            {
                ParsePashr(fields);
            }
        }

        private void ParseGga(string[] fields)
        {
            if(fields.Length < 10)
            {
                InvalidSentences++;
                return;
            }
            GGASentences++;
            LastLatitudeE7 = ParseCoordinate(fields[2], fields[3]);
            LastLongitudeE7 = ParseCoordinate(fields[4], fields[5]);
            LastFixQuality = ParseInteger(fields[6]);
            LastSatellites = ParseInteger(fields[7]);
            LastAltitudeCm = ParseScaled(fields[9], 100.0);
        }

        private void ParseRmc(string[] fields)
        {
            if(fields.Length < 10)
            {
                InvalidSentences++;
                return;
            }
            RMCSentences++;
            LastRmcValid = fields[2] == "A";
            LastLatitudeE7 = ParseCoordinate(fields[3], fields[4]);
            LastLongitudeE7 = ParseCoordinate(fields[5], fields[6]);
            LastSpeedCentiKnots = ParseScaled(fields[7], 100.0);
            LastCourseCentiDegrees = ParseScaled(fields[8], 100.0);
        }

        private void ParsePashr(string[] fields)
        {
            if(fields.Length < 3)
            {
                InvalidSentences++;
                return;
            }
            PASHRSentences++;
            LastPashrHeadingCentiDegrees = ParseScaled(fields[2], 100.0);
        }

        private static int ParseCoordinate(string value, string hemisphere)
        {
            double degreesMinutes;
            if(!Double.TryParse(value, NumberStyles.Float,
                CultureInfo.InvariantCulture, out degreesMinutes))
            {
                return 0;
            }
            var degrees = Math.Floor(degreesMinutes / 100.0);
            var decimalDegrees = degrees +
                (degreesMinutes - degrees * 100.0) / 60.0;
            if(hemisphere == "S" || hemisphere == "W")
            {
                decimalDegrees = -decimalDegrees;
            }
            return (int)Math.Round(decimalDegrees * 1.0e7);
        }

        private static int ParseInteger(string value)
        {
            int result;
            return Int32.TryParse(value, NumberStyles.Integer,
                CultureInfo.InvariantCulture, out result) ? result : 0;
        }

        private static int ParseScaled(string value, double scale)
        {
            double result;
            return Double.TryParse(value, NumberStyles.Float,
                CultureInfo.InvariantCulture, out result) ?
                (int)Math.Round(result * scale) : 0;
        }

        private readonly StringBuilder line;

        private const uint BaudRate = 38400;
        private const int MaximumSentenceLength = 255;
    }
}
