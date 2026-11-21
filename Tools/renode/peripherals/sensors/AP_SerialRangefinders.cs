// Physics-driven serial rangefinders with simple ASCII protocols.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_SerialRangefinder : AP_UARTFrameDevice
    {
        protected AP_SerialRangefinder(IMachine machine, uint readingsPerSecond,
            uint baudRate = 115200) : base(machine, readingsPerSecond, baudRate)
        {
            physics = AP_PhysicsState.ForMachine(machine);
        }

        protected double DistanceMetres()
        {
            var truth = physics.Current;
            return Math.Max(0, truth.TimestampUs == 0
                ? DefaultDistanceM
                : truth.RangefinderM[RangefinderIndex]);
        }

        protected ushort DistanceCentimetres()
        {
            return (ushort)Math.Min(ushort.MaxValue,
                Math.Round(DistanceMetres() * 100.0));
        }

        protected ushort DistanceMillimetres()
        {
            return (ushort)Math.Min(ushort.MaxValue,
                Math.Round(DistanceMetres() * 1000.0));
        }

        protected static ushort ModbusCrc(byte[] values, int count)
        {
            ushort crc = 0xFFFF;
            for(var index = 0; index < count; index++)
            {
                crc ^= values[index];
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (ushort)((crc & 1) != 0
                        ? (crc >> 1) ^ 0xA001
                        : crc >> 1);
                }
            }
            return crc;
        }

        protected static byte Crc8(byte[] values, int count)
        {
            byte crc = 0;
            for(var index = 0; index < count; index++)
            {
                crc ^= values[index];
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (byte)((crc & 0x80) != 0
                        ? (crc << 1) ^ 0x07
                        : crc << 1);
                }
            }
            return crc;
        }

        protected static ushort Crc16CcittRight(byte[] values, int count)
        {
            ushort crc = 0xFFFF;
            for(var index = 0; index < count; index++)
            {
                crc ^= values[index];
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (ushort)((crc & 1) != 0
                        ? (crc >> 1) ^ 0x8408
                        : crc >> 1);
                }
            }
            return (ushort)(crc ^ 0xFFFF);
        }

        protected static ushort Crc16Xmodem(byte[] values, int count)
        {
            ushort crc = 0;
            for(var index = 0; index < count; index++)
            {
                crc ^= (ushort)(values[index] << 8);
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (ushort)((crc & 0x8000) != 0
                        ? (crc << 1) ^ 0x1021
                        : crc << 1);
                }
            }
            return crc;
        }

        public uint RangefinderIndex { get; set; }

        private readonly AP_PhysicsState physics;
        private const double DefaultDistanceM = 5.0;
    }

    public class AP_MaxSonarSerial : AP_SerialRangefinder
    {
        public AP_MaxSonarSerial(IMachine machine) : base(machine, ReadingsPerSecond, BaudRate)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceInches = (uint)Math.Max(0, Math.Min(MaxDistanceInches,
                Math.Round(DistanceMetres() / MetresPerInch)));
            return Encoding.ASCII.GetBytes(String.Format(
                CultureInfo.InvariantCulture, "R{0:D3}\r", distanceInches));
        }

        private const uint ReadingsPerSecond = 20;
        private const uint BaudRate = 9600;
        private const uint MaxDistanceInches = 999;
        private const double MetresPerInch = 0.0254;
    }

    public class AP_NMEARangefinder : AP_SerialRangefinder
    {
        public AP_NMEARangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var body = String.Format(
                CultureInfo.InvariantCulture, "SDDPT,{0:F2},0.0",
                DistanceMetres());
            byte checksum = 0;
            foreach(var value in Encoding.ASCII.GetBytes(body))
            {
                checksum ^= value;
            }
            return Encoding.ASCII.GetBytes(String.Format(
                CultureInfo.InvariantCulture, "${0}*{1:X2}\r\n", body, checksum));
        }

        private const uint ReadingsPerSecond = 20;
    }

    public class AP_GYUS42v2 : AP_SerialRangefinder
    {
        public AP_GYUS42v2(IMachine machine) : base(machine, ReadingsPerSecond, BaudRate)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceCm = DistanceCentimetres();
            var frame = new byte[] {
                0x5A, 0x00, 0x00, 0x00,
                (byte)(distanceCm >> 8),
                (byte)distanceCm,
                0x00,
            };
            for(var index = 0; index < frame.Length - 1; index++)
            {
                frame[frame.Length - 1] += frame[index];
            }
            return frame;
        }

        private const uint ReadingsPerSecond = 20;
        private const uint BaudRate = 9600;
    }

    public class AP_LanbaoRangefinder : AP_SerialRangefinder
    {
        public AP_LanbaoRangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceMm = DistanceMillimetres();
            var frame = new byte[] {
                0xA5, 0x5A,
                (byte)(distanceMm >> 8),
                (byte)distanceMm,
                0x00, 0x00,
            };
            var crc = ModbusCrc(frame, 4);
            frame[4] = (byte)crc;
            frame[5] = (byte)(crc >> 8);
            return frame;
        }

        private const uint ReadingsPerSecond = 20;
    }

    public class AP_TeraRangerSerial : AP_SerialRangefinder
    {
        public AP_TeraRangerSerial(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceMm = DistanceMillimetres();
            var frame = new byte[] {
                0x54,
                (byte)(distanceMm >> 8),
                (byte)distanceMm,
                0xC0,
                0x00,
            };
            frame[4] = Crc8(frame, 4);
            return frame;
        }

        private const uint ReadingsPerSecond = 20;
    }

    public class AP_USD1Rangefinder : AP_SerialRangefinder
    {
        public AP_USD1Rangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceCm = DistanceCentimetres();
            var frame = new byte[] {
                0xFE, 0x00,
                (byte)distanceCm,
                (byte)(distanceCm >> 8),
                0x00, 0x00,
            };
            frame[5] = (byte)(frame[1] + frame[2] + frame[3] + frame[4]);
            return frame;
        }

        private const uint ReadingsPerSecond = 20;
    }

    public class AP_NoopLoopRangefinder : AP_SerialRangefinder
    {
        public AP_NoopLoopRangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceMm = Math.Max(1, (int)DistanceMillimetres());
            var scaledDistance = distanceMm * 256;
            var frame = new byte[] {
                0x57, 0x00, 0xFF, 0x00,
                0x9E, 0x8F, 0x00, 0x00,
                (byte)(scaledDistance >> 8),
                (byte)(scaledDistance >> 16),
                (byte)(scaledDistance >> 24),
                0x00, 0x03, 0x00, 0x06, 0x00,
            };
            for(var index = 0; index < frame.Length - 1; index++)
            {
                frame[frame.Length - 1] += frame[index];
            }
            return frame;
        }

        private const uint ReadingsPerSecond = 20;
    }

    public class AP_JRERangefinder : AP_SerialRangefinder
    {
        public AP_JRERangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceCm = DistanceCentimetres();
            var frame = new byte[16];
            frame[0] = (byte)'R';
            frame[1] = (byte)'A';
            frame[2] = 1;
            frame[3] = frameCounter++;
            frame[4] = (byte)(distanceCm >> 8);
            frame[5] = (byte)distanceCm;
            frame[10] = 2;
            var crc = Crc16CcittRight(frame, 14);
            frame[14] = (byte)crc;
            frame[15] = (byte)(crc >> 8);
            return frame;
        }

        private byte frameCounter;
        private const uint ReadingsPerSecond = 20;
    }

    public class AP_AinsteinLRD1Rangefinder : AP_SerialRangefinder
    {
        public AP_AinsteinLRD1Rangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceCm = DistanceCentimetres();
            var frame = new byte[32];
            frame[0] = 0xEB;
            frame[1] = 0x90;
            frame[3] = 28;
            frame[6] = (byte)(distanceCm >> 8);
            frame[7] = (byte)distanceCm;
            frame[8] = 100;
            for(var index = 11; index < frame.Length - 1; index++)
            {
                frame[index] = 0xFF;
            }
            frame[11] = 1;
            for(var index = 3; index < frame.Length - 1; index++)
            {
                frame[frame.Length - 1] += frame[index];
            }
            return frame;
        }

        private const uint ReadingsPerSecond = 10;
    }

    public class AP_RDS02UFRangefinder : AP_SerialRangefinder
    {
        public AP_RDS02UFRangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceCm = DistanceCentimetres();
            var frame = new byte[21];
            frame[0] = 0x55;
            frame[1] = 0x55;
            frame[4] = 0xFF;
            frame[5] = 0x03;
            frame[6] = 10;
            frame[8] = 0x0C;
            frame[9] = 0x07;
            frame[13] = (byte)distanceCm;
            frame[14] = (byte)(distanceCm >> 8);
            frame[18] = RdsCrc(frame, 2, 16);
            frame[19] = 0xAA;
            frame[20] = 0xAA;
            return frame;
        }

        private static byte RdsCrc(byte[] values, int offset, int count)
        {
            byte crc = 0;
            for(var index = offset; index < offset + count; index++)
            {
                crc = RdsCrcTable[crc ^ values[index]];
            }
            return crc;
        }

        private const uint ReadingsPerSecond = 20;
        private static readonly byte[] RdsCrcTable = {
            0x93, 0x98, 0xE4, 0x46, 0xEB, 0xBA, 0x04, 0x4C,
            0xFA, 0x40, 0xB8, 0x96, 0x0E, 0xB2, 0xB7, 0xC0,
            0x0C, 0x32, 0x9B, 0x80, 0xFF, 0x30, 0x7F, 0x9D,
            0xB3, 0x81, 0x58, 0xE7, 0xF1, 0x19, 0x7E, 0xB6,
            0xCD, 0xF7, 0xB4, 0xCB, 0xBC, 0x5C, 0xD6, 0x09,
            0x20, 0x0A, 0xE0, 0x37, 0x51, 0x67, 0x24, 0x95,
            0xE1, 0x62, 0xF8, 0x5E, 0x38, 0x15, 0x54, 0x77,
            0x63, 0x57, 0x6D, 0xE9, 0x89, 0x76, 0xBE, 0x41,
            0x5D, 0xF9, 0xB1, 0x4D, 0x6C, 0x53, 0x9C, 0xA2,
            0x23, 0xC4, 0x8E, 0xC8, 0x05, 0x42, 0x61, 0x71,
            0xC5, 0x00, 0x18, 0x6F, 0x5F, 0xFB, 0x7B, 0x11,
            0x65, 0x2D, 0x8C, 0xED, 0x14, 0xAB, 0x88, 0xD5,
            0xD9, 0xC2, 0x36, 0x34, 0x7C, 0x5B, 0x3C, 0xF6,
            0x48, 0x0B, 0xEE, 0x02, 0x83, 0x79, 0x17, 0xE6,
            0xA8, 0x78, 0xF5, 0xD3, 0x4E, 0x50, 0x52, 0x91,
            0xD8, 0xC6, 0x22, 0xEC, 0x3B, 0xE5, 0x3F, 0x86,
            0x06, 0xCF, 0x2B, 0x2F, 0x3D, 0x59, 0x1C, 0x87,
            0xEF, 0x4F, 0x10, 0xD2, 0x7D, 0xDA, 0x72, 0xA0,
            0x9F, 0xDE, 0x6B, 0x75, 0x56, 0xBD, 0xC7, 0xC1,
            0x70, 0x1D, 0x25, 0x92, 0xA5, 0x31, 0xE2, 0xD7,
            0xD0, 0x9A, 0xAF, 0xA9, 0xC9, 0x97, 0x08, 0x33,
            0x5A, 0x99, 0xC3, 0x16, 0x84, 0x82, 0x8A, 0xF3,
            0x4A, 0xCE, 0xDB, 0x29, 0x0F, 0xAE, 0x6E, 0xE3,
            0x8B, 0x07, 0x3A, 0x74, 0x47, 0xB0, 0xBB, 0xB5,
            0x7A, 0xAA, 0x2C, 0xD4, 0x03, 0x3E, 0x1A, 0xA7,
            0x27, 0x64, 0x06, 0xBF, 0x55, 0x73, 0x1E, 0xFE,
            0x49, 0x01, 0x39, 0x28, 0xF4, 0x26, 0xDF, 0xDD,
            0x44, 0x0D, 0x21, 0xF2, 0x85, 0xB9, 0xEA, 0x4B,
            0xDC, 0x6A, 0xCA, 0xAC, 0x12, 0xFC, 0x2E, 0x2A,
            0xA3, 0xF0, 0x66, 0xE8, 0x60, 0x45, 0xA1, 0x8D,
            0x68, 0x35, 0xFD, 0x8F, 0x9E, 0x1F, 0x13, 0xD1,
            0xAD, 0x69, 0xCC, 0xA4, 0x94, 0x90, 0x1B, 0x43,
        };
    }

    public class AP_DTS6012MRangefinder : AP_SerialRangefinder
    {
        public AP_DTS6012MRangefinder(IMachine machine) : base(machine, ReadingsPerSecond, BaudRate)
        {
            command = new List<byte>();
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            command.Clear();
            Configured = false;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            command.Add(value);
            while(command.Count > 0 && command[0] != 0xA5)
            {
                command.RemoveAt(0);
            }
            if(command.Count < CommandLength)
            {
                return;
            }
            var crc = ModbusCrc(command.ToArray(), CommandLength - 2);
            if(command[1] == 0x03 && command[2] == 0x20 && command[3] == 0x01 &&
                command[4] == 0 && command[5] == 0 && command[6] == 0 &&
                command[7] == (byte)(crc >> 8) && command[8] == (byte)crc)
            {
                Configured = true;
            }
            command.RemoveAt(0);
        }

        protected override byte[] BuildFrame()
        {
            if(!Configured)
            {
                return null;
            }
            var distanceMm = DistanceMillimetres();
            var frame = new byte[] {
                0xA5, 0x03, 0x20, 0x01, 0x00, 0x00, 0x0E,
                0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
                (byte)distanceMm, (byte)(distanceMm >> 8),
                0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                0x00, 0x00,
            };
            var crc = ModbusCrc(frame, frame.Length - 2);
            frame[frame.Length - 2] = (byte)(crc >> 8);
            frame[frame.Length - 1] = (byte)crc;
            return frame;
        }

        public bool Configured { get; private set; }

        private readonly List<byte> command;
        private const int CommandLength = 9;
        private const uint ReadingsPerSecond = 10;
        private const uint BaudRate = 921600;
    }

    public class AP_BLPingRangefinder : AP_SerialRangefinder
    {
        public AP_BLPingRangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            command = new List<byte>();
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            command.Clear();
            intervalSet = false;
            continuousStarted = false;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            command.Add(value);
            ParseCommands();
        }

        protected override byte[] BuildFrame()
        {
            if(!Configured)
            {
                return null;
            }
            var distanceMm = (uint)Math.Round(DistanceMetres() * 1000.0);
            var frame = new byte[] {
                0x42, 0x52, 0x05, 0x00, 0xBB, 0x04, 0x01, 0x00,
                (byte)distanceMm,
                (byte)(distanceMm >> 8),
                (byte)(distanceMm >> 16),
                (byte)(distanceMm >> 24),
                100,
                0x00, 0x00,
            };
            ushort checksum = 0;
            for(var index = 0; index < frame.Length - 2; index++)
            {
                checksum += frame[index];
            }
            frame[frame.Length - 2] = (byte)checksum;
            frame[frame.Length - 1] = (byte)(checksum >> 8);
            return frame;
        }

        private void ParseCommands()
        {
            while(command.Count > 0)
            {
                if(command[0] != 0x42 ||
                    (command.Count > 1 && command[1] != 0x52))
                {
                    command.RemoveAt(0);
                    continue;
                }
                if(command.Count < 4)
                {
                    return;
                }
                var payloadLength = command[2] | command[3] << 8;
                var frameLength = payloadLength + 10;
                if(payloadLength > MaximumPayloadLength)
                {
                    command.RemoveAt(0);
                    continue;
                }
                if(command.Count < frameLength)
                {
                    return;
                }
                ushort checksum = 0;
                for(var index = 0; index < frameLength - 2; index++)
                {
                    checksum += command[index];
                }
                var receivedChecksum = command[frameLength - 2] |
                    command[frameLength - 1] << 8;
                if(checksum == receivedChecksum)
                {
                    var messageId = command[4] | command[5] << 8;
                    if(messageId == SetPingIntervalMessageId)
                    {
                        intervalSet = true;
                    }
                    else if(messageId == ContinuousStartMessageId &&
                        payloadLength >= 2 &&
                        (command[8] | command[9] << 8) == DistanceSimpleMessageId)
                    {
                        continuousStarted = true;
                    }
                }
                command.RemoveRange(0, frameLength);
            }
        }

        public bool Configured => intervalSet && continuousStarted;

        private readonly List<byte> command;
        private bool intervalSet;
        private bool continuousStarted;
        private const int MaximumPayloadLength = 20;
        private const int SetPingIntervalMessageId = 1004;
        private const int DistanceSimpleMessageId = 1211;
        private const int ContinuousStartMessageId = 1400;
        private const uint ReadingsPerSecond = 20;
    }

    public class AP_WaspRangefinder : AP_SerialRangefinder
    {
        public AP_WaspRangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            command = new List<byte>();
            configuredCommands = new HashSet<string>(StringComparer.Ordinal);
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            command.Clear();
            configuredCommands.Clear();
            Configured = false;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            command.Add(value);
            if(command.Count > MaximumCommandLength)
            {
                command.Clear();
                return;
            }
            if(value != (byte)'\n')
            {
                return;
            }

            var line = Encoding.ASCII.GetString(command.ToArray()).TrimEnd('\r', '\n');
            command.Clear();
            if(line.Length < 2 || line[0] != '>')
            {
                return;
            }
            var response = line.Substring(1);
            var separator = response.IndexOf(' ');
            var name = separator < 0 ? response : response.Substring(0, separator);
            foreach(var required in RequiredCommands)
            {
                if(name == required)
                {
                    configuredCommands.Add(name);
                    break;
                }
            }
            Configured = true;
            foreach(var required in RequiredCommands)
            {
                if(!configuredCommands.Contains(required))
                {
                    Configured = false;
                    break;
                }
            }
            TransmitFrame(Encoding.ASCII.GetBytes(response + "\n"));
        }

        protected override byte[] BuildFrame()
        {
            if(!Configured)
            {
                return null;
            }
            return Encoding.ASCII.GetBytes(String.Format(
                CultureInfo.InvariantCulture, "{0:F6}\n", DistanceMetres()));
        }

        public bool Configured { get; private set; }

        private readonly List<byte> command;
        private readonly HashSet<string> configuredCommands;
        private static readonly string[] RequiredCommands = {
            "BAUD", "LBE", "FMT", "FRQ", "GO", "AUT", "MAVG", "MEDF", "AVG", "AUV",
        };
        private const int MaximumCommandLength = 64;
        private const uint ReadingsPerSecond = 20;
    }

    public abstract class AP_LightWareBinaryDevice : AP_SerialRangefinder
    {
        protected AP_LightWareBinaryDevice(IMachine machine, uint readingsPerSecond) :
            base(machine, readingsPerSecond)
        {
            command = new List<byte>();
        }

        public override void Reset()
        {
            base.Reset();
            command.Clear();
            ResetDevice();
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            command.Add(value);
            ParseCommands();
        }

        private void ParseCommands()
        {
            while(command.Count > 0)
            {
                if(command[0] != Preamble)
                {
                    command.RemoveAt(0);
                    continue;
                }
                if(command.Count < 3)
                {
                    return;
                }
                var flags = command[1] | command[2] << 8;
                var includedPayloadLength = flags >> 6;
                if(includedPayloadLength < 1 || includedPayloadLength > MaximumIncludedPayloadLength)
                {
                    command.RemoveAt(0);
                    continue;
                }
                var payloadLength = includedPayloadLength - 1;
                var frameLength = payloadLength + 6;
                if(command.Count < frameLength)
                {
                    return;
                }
                var frame = command.GetRange(0, frameLength).ToArray();
                var checksum = Crc16Xmodem(frame, frame.Length - 2);
                var receivedChecksum = frame[frame.Length - 2] |
                    frame[frame.Length - 1] << 8;
                if(checksum != receivedChecksum)
                {
                    command.RemoveAt(0);
                    continue;
                }
                var messageId = frame[3];
                var payload = new byte[payloadLength];
                Array.Copy(frame, 4, payload, 0, payloadLength);
                ProcessCommand(messageId, payload);
                command.RemoveRange(0, frameLength);
            }
        }

        protected abstract void ProcessCommand(byte messageId, byte[] payload);

        protected virtual void ResetDevice()
        {
        }

        protected static byte[] BuildMessage(byte messageId, byte[] payload)
        {
            var frame = new byte[payload.Length + 6];
            frame[0] = Preamble;
            var flags = ((payload.Length + 1) << 6) | 1;
            frame[1] = (byte)flags;
            frame[2] = (byte)(flags >> 8);
            frame[3] = messageId;
            Array.Copy(payload, 0, frame, 4, payload.Length);
            var checksum = Crc16Xmodem(frame, frame.Length - 2);
            frame[frame.Length - 2] = (byte)checksum;
            frame[frame.Length - 1] = (byte)(checksum >> 8);
            return frame;
        }

        private readonly List<byte> command;
        private const byte Preamble = 0xAA;
        private const int MaximumIncludedPayloadLength = 257;
    }

    public class AP_LightWareGRFRangefinder : AP_LightWareBinaryDevice
    {
        public AP_LightWareGRFRangefinder(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            if(!Configured)
            {
                return null;
            }
            var distanceDecimetres = (uint)Math.Round(DistanceMetres() * 10.0);
            var payload = new byte[] {
                (byte)distanceDecimetres,
                (byte)(distanceDecimetres >> 8),
                (byte)(distanceDecimetres >> 16),
                (byte)(distanceDecimetres >> 24),
                100, 0, 0, 0,
            };
            return BuildMessage(DistanceDataCmMessageId, payload);
        }

        protected override void ProcessCommand(byte messageId, byte[] payload)
        {
            if(messageId == ProductNameMessageId && payload.Length == 0)
            {
                TransmitFrame(BuildMessage(messageId, new byte[] {
                    (byte)'G', (byte)'R', (byte)'F', (byte)'2', (byte)'5', (byte)'0', 0,
                }));
                return;
            }
            if(payload.Length != 4)
            {
                return;
            }
            if(messageId == UpdateRateMessageId || messageId == DistanceOutputMessageId)
            {
                TransmitFrame(BuildMessage(messageId, payload));
                return;
            }
            if(messageId == StreamMessageId)
            {
                Configured = payload[0] == DistanceCmStream &&
                    payload[1] == 0 && payload[2] == 0 && payload[3] == 0;
                TransmitFrame(BuildMessage(messageId, payload));
            }
        }

        protected override void ResetDevice()
        {
            Configured = false;
        }

        public bool Configured { get; private set; }

        private const byte ProductNameMessageId = 0;
        private const byte DistanceOutputMessageId = 27;
        private const byte StreamMessageId = 30;
        private const byte DistanceDataCmMessageId = 44;
        private const byte UpdateRateMessageId = 74;
        private const byte DistanceCmStream = 5;
        private const uint ReadingsPerSecond = 50;
    }

    public class AP_LightWareSF40CProximity : AP_LightWareBinaryDevice
    {
        public AP_LightWareSF40CProximity(IMachine machine) : base(machine, FramesPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            if(!Configured)
            {
                return null;
            }
            var distanceCm = DistanceCentimetres();
            var payload = new byte[14 + PointCount * 2];
            payload[8] = PointCount;
            payload[10] = PointCount;
            for(var index = 0; index < PointCount; index++)
            {
                var offset = 14 + index * 2;
                payload[offset] = (byte)distanceCm;
                payload[offset + 1] = (byte)(distanceCm >> 8);
            }
            return BuildMessage(DistanceOutputMessageId, payload);
        }

        protected override void ProcessCommand(byte messageId, byte[] payload)
        {
            if(messageId == MotorStateMessageId && payload.Length == 0)
            {
                TransmitFrame(BuildMessage(messageId, new byte[] { 3 }));
                return;
            }
            if(messageId == TokenMessageId && payload.Length == 0)
            {
                TransmitFrame(BuildMessage(messageId, Token));
                return;
            }
            if(messageId == OutputRateMessageId && payload.Length == 1)
            {
                outputRateSet = payload[0] == DesiredOutputRate;
                TransmitFrame(BuildMessage(messageId, payload));
                return;
            }
            if(messageId == StreamMessageId && payload.Length == 4)
            {
                streaming = payload[0] == DistanceStream &&
                    payload[1] == 0 && payload[2] == 0 && payload[3] == 0;
                TransmitFrame(BuildMessage(messageId, payload));
                return;
            }
            if(messageId == ResetMessageId && payload.Length == Token.Length)
            {
                outputRateSet = false;
                streaming = false;
                TransmitFrame(BuildMessage(messageId, new byte[0]));
            }
        }

        protected override void ResetDevice()
        {
            outputRateSet = false;
            streaming = false;
        }

        public bool Configured => outputRateSet && streaming;

        private bool outputRateSet;
        private bool streaming;
        private const byte TokenMessageId = 10;
        private const byte ResetMessageId = 14;
        private const byte StreamMessageId = 30;
        private const byte DistanceOutputMessageId = 48;
        private const byte MotorStateMessageId = 106;
        private const byte OutputRateMessageId = 108;
        private const byte DesiredOutputRate = 3;
        private const byte DistanceStream = 3;
        private const byte PointCount = 72;
        private const uint FramesPerSecond = 10;
        private static readonly byte[] Token = { 0x12, 0x34 };
    }

    public class AP_LightWareSF45BProximity : AP_LightWareBinaryDevice
    {
        public AP_LightWareSF45BProximity(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            if(!Configured)
            {
                return null;
            }
            var distanceCm = DistanceCentimetres();
            var angleCentidegrees = (short)Math.Round(angleDegrees * 100.0);
            var payload = new byte[] {
                (byte)distanceCm,
                (byte)(distanceCm >> 8),
                (byte)angleCentidegrees,
                (byte)(angleCentidegrees >> 8),
            };
            var frame = BuildMessage(DistanceDataCmMessageId, payload);
            angleDegrees += direction * AngleStepDegrees;
            if(angleDegrees >= MaximumAngleDegrees)
            {
                angleDegrees = MaximumAngleDegrees;
                direction = -1;
            }
            else if(angleDegrees <= MinimumAngleDegrees)
            {
                angleDegrees = MinimumAngleDegrees;
                direction = 1;
            }
            return frame;
        }

        protected override void ProcessCommand(byte messageId, byte[] payload)
        {
            if(messageId == UpdateRateMessageId && payload.Length == 1)
            {
                updateRateSet = payload[0] == DesiredUpdateRate;
                TransmitFrame(BuildMessage(messageId, payload));
                return;
            }
            if(messageId == DistanceOutputMessageId && payload.Length == 4)
            {
                fieldsSet = payload[0] == 1 && payload[1] == 1 &&
                    payload[2] == 0 && payload[3] == 0;
                TransmitFrame(BuildMessage(messageId, payload));
                return;
            }
            if(messageId == StreamMessageId && payload.Length == 4)
            {
                streaming = payload[0] == DistanceCmStream &&
                    payload[1] == 0 && payload[2] == 0 && payload[3] == 0;
                TransmitFrame(BuildMessage(messageId, payload));
            }
        }

        protected override void ResetDevice()
        {
            updateRateSet = false;
            fieldsSet = false;
            streaming = false;
            angleDegrees = MinimumAngleDegrees;
            direction = 1;
        }

        public bool Configured => updateRateSet && fieldsSet && streaming;

        private bool updateRateSet;
        private bool fieldsSet;
        private bool streaming;
        private double angleDegrees = MinimumAngleDegrees;
        private int direction = 1;
        private const byte DistanceOutputMessageId = 27;
        private const byte StreamMessageId = 30;
        private const byte DistanceDataCmMessageId = 44;
        private const byte UpdateRateMessageId = 66;
        private const byte DesiredUpdateRate = 6;
        private const byte DistanceCmStream = 5;
        private const double MinimumAngleDegrees = -170.0;
        private const double MaximumAngleDegrees = 170.0;
        private const double AngleStepDegrees = 5.0;
        private const uint ReadingsPerSecond = 129;
    }

    public class AP_LeddarOneRangefinder : AP_SerialRangefinder
    {
        public AP_LeddarOneRangefinder(IMachine machine) : base(machine, PollRate)
        {
            request = new List<byte>();
        }

        public override void Reset()
        {
            base.Reset();
            request.Clear();
            RequestsReceived = 0;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            request.Add(value);
            ParseRequests();
        }

        protected override byte[] BuildFrame()
        {
            return null;
        }

        private void ParseRequests()
        {
            while(request.Count > 0)
            {
                if(request[0] != Address ||
                    (request.Count > 1 && request[1] != ReadInputRegisters))
                {
                    request.RemoveAt(0);
                    continue;
                }
                if(request.Count < RequestLength)
                {
                    return;
                }
                var frame = request.GetRange(0, RequestLength).ToArray();
                var checksum = ModbusCrc(frame, RequestLength - 2);
                var valid = frame[2] == 0 && frame[3] == FirstRegister &&
                    frame[4] == 0 && frame[5] == RegisterCount &&
                    frame[6] == (byte)checksum && frame[7] == (byte)(checksum >> 8);
                request.RemoveRange(0, RequestLength);
                if(!valid)
                {
                    continue;
                }
                RequestsReceived++;
                TransmitFrame(BuildResponse());
            }
        }

        private byte[] BuildResponse()
        {
            var distanceMm = DistanceMillimetres();
            var frame = new byte[ResponseLength];
            frame[0] = Address;
            frame[1] = ReadInputRegisters;
            frame[2] = ResponseDataLength;
            frame[7] = 0xDD;
            frame[8] = 0x04;
            frame[10] = 1;
            frame[11] = (byte)(distanceMm >> 8);
            frame[12] = (byte)distanceMm;
            frame[14] = 37;
            var checksum = ModbusCrc(frame, ResponseLength - 2);
            frame[ResponseLength - 2] = (byte)checksum;
            frame[ResponseLength - 1] = (byte)(checksum >> 8);
            return frame;
        }

        public bool Configured => RequestsReceived > 0;
        public ulong RequestsReceived { get; private set; }

        private readonly List<byte> request;
        private const byte Address = 1;
        private const byte ReadInputRegisters = 4;
        private const byte FirstRegister = 20;
        private const byte RegisterCount = 10;
        private const byte ResponseDataLength = RegisterCount * 2;
        private const int RequestLength = 8;
        private const int ResponseLength = ResponseDataLength + 5;
        private const uint PollRate = 20;
    }

    public class AP_LeddarVu8Rangefinder : AP_SerialRangefinder
    {
        public AP_LeddarVu8Rangefinder(IMachine machine) : base(machine, PollRate)
        {
            request = new List<byte>();
        }

        public override void Reset()
        {
            base.Reset();
            request.Clear();
            RequestsReceived = 0;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            request.Add(value);
            ParseRequests();
        }

        protected override byte[] BuildFrame()
        {
            return null;
        }

        private void ParseRequests()
        {
            while(request.Count > 0)
            {
                if(request[0] != Address ||
                    (request.Count > 1 && request[1] != ReadInputRegisters))
                {
                    request.RemoveAt(0);
                    continue;
                }
                if(request.Count < RequestLength)
                {
                    return;
                }
                var frame = request.GetRange(0, RequestLength).ToArray();
                var checksum = ModbusCrc(frame, RequestLength - 2);
                var valid = frame[2] == 0 && frame[3] == FirstDistanceRegister &&
                    frame[4] == 0 && frame[5] == SegmentCount &&
                    frame[6] == (byte)checksum && frame[7] == (byte)(checksum >> 8);
                request.RemoveRange(0, RequestLength);
                if(!valid)
                {
                    continue;
                }
                RequestsReceived++;
                TransmitFrame(BuildResponse());
            }
        }

        private byte[] BuildResponse()
        {
            var distanceCm = DistanceCentimetres();
            var frame = new byte[ResponseLength];
            frame[0] = Address;
            frame[1] = ReadInputRegisters;
            frame[2] = ResponseDataLength;
            frame[3] = (byte)(distanceCm >> 8);
            frame[4] = (byte)distanceCm;
            var checksum = ModbusCrc(frame, ResponseLength - 2);
            frame[ResponseLength - 2] = (byte)checksum;
            frame[ResponseLength - 1] = (byte)(checksum >> 8);
            return frame;
        }

        public bool Configured => RequestsReceived > 0;
        public ulong RequestsReceived { get; private set; }

        private readonly List<byte> request;
        private const byte Address = 1;
        private const byte ReadInputRegisters = 4;
        private const byte FirstDistanceRegister = 16;
        private const byte SegmentCount = 8;
        private const byte ResponseDataLength = SegmentCount * 2;
        private const int RequestLength = 8;
        private const int ResponseLength = ResponseDataLength + 5;
        private const uint PollRate = 20;
    }

    public class AP_TeraRangerTowerProximity : AP_SerialRangefinder
    {
        public AP_TeraRangerTowerProximity(IMachine machine) : base(machine, ReadingsPerSecond, BaudRate)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceMm = DistanceMillimetres();
            var frame = new byte[19];
            frame[0] = (byte)'T';
            frame[1] = (byte)'H';
            for(var index = 0; index < 8; index++)
            {
                frame[2 + index * 2] = (byte)(distanceMm >> 8);
                frame[3 + index * 2] = (byte)distanceMm;
            }
            frame[18] = Crc8(frame, 18);
            return frame;
        }

        private const uint ReadingsPerSecond = 5;
        private const uint BaudRate = 921600;
    }

    public class AP_TeraRangerTowerEvoProximity : AP_SerialRangefinder
    {
        public AP_TeraRangerTowerEvoProximity(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            command = new List<byte>();
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            command.Clear();
            configurationStep = 0;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            command.Add(value);
            ParseCommands();
        }

        protected override byte[] BuildFrame()
        {
            if(!Configured)
            {
                return null;
            }
            var distanceMm = DistanceMillimetres();
            var frame = new byte[20];
            frame[0] = (byte)'T';
            frame[1] = (byte)'H';
            for(var index = 0; index < 8; index++)
            {
                frame[2 + index * 2] = (byte)(distanceMm >> 8);
                frame[3 + index * 2] = (byte)distanceMm;
            }
            frame[18] = SensorMask;
            frame[19] = Crc8(frame, 19);
            return frame;
        }

        private void ParseCommands()
        {
            while(configurationStep < ConfigurationCommands.Length && command.Count > 0)
            {
                var expected = ConfigurationCommands[configurationStep];
                var compareLength = Math.Min(command.Count, expected.Length);
                var matches = true;
                for(var index = 0; index < compareLength; index++)
                {
                    if(command[index] != expected[index])
                    {
                        matches = false;
                        break;
                    }
                }
                if(!matches)
                {
                    command.RemoveAt(0);
                    continue;
                }
                if(command.Count < expected.Length)
                {
                    return;
                }
                command.RemoveRange(0, expected.Length);
                configurationStep++;
                TransmitFrame(Acknowledgement);
            }
        }

        public bool Configured => configurationStep == ConfigurationCommands.Length;

        private readonly List<byte> command;
        private int configurationStep;
        private const byte SensorMask = 0xFF;
        private const uint ReadingsPerSecond = 100;
        private static readonly byte[] Acknowledgement = { 0x00, 0x00, 0x00, 0x00 };
        private static readonly byte[][] ConfigurationCommands = {
            new byte[] { 0x00, 0x11, 0x02, 0x4C },
            new byte[] { 0x00, 0x31, 0x03, 0xE5 },
            new byte[] { 0x00, 0x52, 0x03, 0x03, 0xC4 },
            new byte[] { 0x00, 0x52, 0x02, 0x01, 0xDF },
        };
    }

    public class AP_CygbotD1Proximity : AP_SerialRangefinder
    {
        public AP_CygbotD1Proximity(IMachine machine) : base(machine, ReadingsPerSecond)
        {
            this.machine = machine;
            command = new List<byte>();
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            command.Clear();
            Configured = false;
            streamStartUs = 0;
            firstFrameSent = false;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            command.Add(value);
            while(command.Count > 0)
            {
                var compareLength = Math.Min(command.Count, StartCommand.Length);
                var matches = true;
                for(var index = 0; index < compareLength; index++)
                {
                    if(command[index] != StartCommand[index])
                    {
                        matches = false;
                        break;
                    }
                }
                if(!matches)
                {
                    command.RemoveAt(0);
                    continue;
                }
                if(command.Count < StartCommand.Length)
                {
                    return;
                }
                command.RemoveRange(0, StartCommand.Length);
                Configured = true;
                streamStartUs = NowUs;
                firstFrameSent = false;
            }
        }

        protected override byte[] BuildFrame()
        {
            if(!Configured)
            {
                return null;
            }
            var streamAgeUs = NowUs - streamStartUs;
            if(streamAgeUs < FirstFrameDelayUs ||
                (firstFrameSent && streamAgeUs < ContinuousStreamDelayUs))
            {
                return null;
            }
            firstFrameSent = true;
            var distanceMm = DistanceMillimetres();
            var frame = new byte[PacketLength];
            frame[0] = 0x5A;
            frame[1] = 0x77;
            frame[2] = 0xFF;
            frame[3] = (byte)(PayloadLength & 0xFF);
            frame[4] = (byte)(PayloadLength >> 8);
            frame[5] = 0x01;
            for(var sample = 0; sample < SampleCount; sample++)
            {
                var offset = 6 + sample * 2;
                frame[offset] = (byte)(distanceMm >> 8);
                frame[offset + 1] = (byte)distanceMm;
            }
            byte checksum = frame[3];
            for(var index = 4; index < frame.Length - 1; index++)
            {
                checksum ^= frame[index];
            }
            frame[frame.Length - 1] = checksum;
            return frame;
        }

        public bool Configured { get; private set; }

        private ulong NowUs =>
            (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;

        private readonly IMachine machine;
        private readonly List<byte> command;
        private ulong streamStartUs;
        private bool firstFrameSent;
        private const int SampleCount = 161;
        private const ushort PayloadLength = 1 + SampleCount * 2;
        private const int PacketLength = 3 + 2 + PayloadLength + 1;
        private const ulong FirstFrameDelayUs = 900000;
        private const ulong ContinuousStreamDelayUs = 1100000;
        private const uint ReadingsPerSecond = 10;
        private static readonly byte[] StartCommand = {
            0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x03,
        };
    }

    public class AP_RPLidarA2Proximity : AP_SerialRangefinder
    {
        public AP_RPLidarA2Proximity(IMachine machine) : base(machine, SamplesPerSecond)
        {
            command = new List<byte>();
            StartTransmitter();
        }

        public override void Reset()
        {
            base.Reset();
            command.Clear();
            scanning = false;
            angleDegrees = 0;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            command.Add(value);
            while(command.Count > 0)
            {
                if(command[0] != Preamble)
                {
                    command.RemoveAt(0);
                    continue;
                }
                if(command.Count < 2)
                {
                    return;
                }
                ProcessCommand(command[1]);
                command.RemoveRange(0, 2);
            }
        }

        protected override byte[] BuildFrame()
        {
            if(!scanning)
            {
                return null;
            }
            var distanceQ2 = (uint)DistanceMillimetres() * 4;
            distanceQ2 = Math.Min(ushort.MaxValue, distanceQ2);
            var angleQ6 = (ushort)Math.Round(angleDegrees * 64.0);
            var angleField = (ushort)((angleQ6 << 1) | 1);
            var start = angleDegrees == 0;
            var frame = new byte[] {
                (byte)((Quality << 2) | (start ? 1 : 2)),
                (byte)angleField,
                (byte)(angleField >> 8),
                (byte)distanceQ2,
                (byte)(distanceQ2 >> 8),
            };
            angleDegrees += AngleStepDegrees;
            if(angleDegrees >= 360)
            {
                angleDegrees = 0;
            }
            return frame;
        }

        private void ProcessCommand(byte value)
        {
            switch(value)
            {
            case GetDeviceInfoCommand:
                TransmitFrame(DeviceInfoResponse);
                break;
            case GetHealthCommand:
                TransmitFrame(HealthResponse);
                break;
            case ScanCommand:
                TransmitFrame(ScanDescriptor);
                scanning = true;
                break;
            case StopCommand:
                scanning = false;
                break;
            case ResetCommand:
                scanning = false;
                angleDegrees = 0;
                TransmitFrame(FirmwareInformation);
                break;
            }
        }

        public bool Configured => scanning;

        private readonly List<byte> command;
        private bool scanning;
        private double angleDegrees;
        private const byte Preamble = 0xA5;
        private const byte ScanCommand = 0x20;
        private const byte StopCommand = 0x25;
        private const byte ResetCommand = 0x40;
        private const byte GetDeviceInfoCommand = 0x50;
        private const byte GetHealthCommand = 0x52;
        private const byte Quality = 17;
        private const double AngleStepDegrees = 5.0;
        private const uint SamplesPerSecond = 200;
        private static readonly byte[] ScanDescriptor = {
            0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81,
        };
        private static readonly byte[] HealthResponse = {
            0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06,
            0x00, 0x00, 0x00,
        };
        private static readonly byte[] DeviceInfoResponse = {
            0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04,
            0x28, 17, 42, 6,
            0, 1, 2, 3, 4, 5, 6, 7,
            8, 9, 10, 11, 12, 13, 14, 15,
        };
        private static readonly byte[] FirmwareInformation = Encoding.ASCII.GetBytes(
            "R12345678901234567890123456789012345678901234567890123456789012");
    }

    public class AP_LD06Proximity : AP_SerialRangefinder
    {
        public AP_LD06Proximity(IMachine machine) : base(machine, ReadingsPerSecond, BaudRate)
        {
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var distanceMm = DistanceMillimetres();
            var startAngle = (ushort)(packetIndex * AngleStepCentidegrees);
            var endAngle = (ushort)(startAngle + PacketSpanCentidegrees);
            packetIndex = (packetIndex + 1) % PacketsPerRevolution;

            var frame = new byte[47];
            frame[0] = 0x54;
            frame[1] = MeasurementCount;
            frame[2] = (byte)(AngularRateDegreesPerSecond & 0xFF);
            frame[3] = (byte)(AngularRateDegreesPerSecond >> 8);
            frame[4] = (byte)startAngle;
            frame[5] = (byte)(startAngle >> 8);
            for(var index = 0; index < MeasurementCount; index++)
            {
                var offset = 6 + index * 3;
                frame[offset] = (byte)distanceMm;
                frame[offset + 1] = (byte)(distanceMm >> 8);
                frame[offset + 2] = Confidence;
            }
            frame[42] = (byte)endAngle;
            frame[43] = (byte)(endAngle >> 8);
            frame[46] = GenericCrc8(frame, 46, 0x4D);
            return frame;
        }

        private static byte GenericCrc8(byte[] values, int count, byte polynomial)
        {
            byte crc = 0;
            for(var index = 0; index < count; index++)
            {
                crc ^= values[index];
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (byte)((crc & 0x80) != 0
                        ? (crc << 1) ^ polynomial
                        : crc << 1);
                }
            }
            return crc;
        }

        private uint packetIndex;
        private const byte MeasurementCount = 12;
        private const byte Confidence = 100;
        private const ushort AngularRateDegreesPerSecond = 1800;
        private const ushort AngleStepCentidegrees = 3000;
        private const ushort PacketSpanCentidegrees = 2750;
        private const uint PacketsPerRevolution = 12;
        private const uint ReadingsPerSecond = 100;
        private const uint BaudRate = 230400;
    }
}
