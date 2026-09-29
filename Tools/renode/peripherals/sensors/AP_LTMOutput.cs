// Receiver and analyser for ArduPilot LightTelemetry output.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_LTMOutputAnalyzer : AP_UARTFrameDevice
    {
        public AP_LTMOutputAnalyzer(IMachine machine) : base(machine, 1, BaudRate)
        {
            frame = new byte[MaximumFrameLength];
        }

        protected override byte[] BuildFrame()
        {
            return new byte[0];
        }

        public override void Reset()
        {
            base.Reset();
            frameLength = 0;
            frameOffset = 0;
            ValidFrames = 0;
            InvalidFrames = 0;
            GFrames = 0;
            AFrames = 0;
            SFrames = 0;
            LastLatitudeE7 = 0;
            LastLongitudeE7 = 0;
            LastGroundSpeedMS = 0;
            LastRelativeAltitudeCm = 0;
            LastSatellites = 0;
            LastFixType = 0;
            LastPitchDegrees = 0;
            LastRollDegrees = 0;
            LastHeadingDegrees = 0;
            LastVoltageMillivolts = 0;
            LastCurrentCentiamps = 0;
            LastRssi = 0;
            LastAirspeedMS = 0;
            LastStatus = 0;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            if(frameOffset == 0)
            {
                if(value == HeaderFirst)
                {
                    frame[frameOffset++] = value;
                }
                return;
            }
            if(frameOffset == 1)
            {
                if(value == HeaderSecond)
                {
                    frame[frameOffset++] = value;
                }
                else
                {
                    Restart(value);
                }
                return;
            }
            if(frameOffset == 2)
            {
                frameLength = FrameLength(value);
                if(frameLength == 0)
                {
                    InvalidFrames++;
                    Restart(value);
                    return;
                }
            }
            frame[frameOffset++] = value;
            if(frameLength != 0 && frameOffset == frameLength)
            {
                ParseFrame();
                frameLength = 0;
                frameOffset = 0;
            }
        }

        public ulong ValidFrames { get; private set; }
        public ulong InvalidFrames { get; private set; }
        public ulong GFrames { get; private set; }
        public ulong AFrames { get; private set; }
        public ulong SFrames { get; private set; }
        public int LastLatitudeE7 { get; private set; }
        public int LastLongitudeE7 { get; private set; }
        public int LastGroundSpeedMS { get; private set; }
        public int LastRelativeAltitudeCm { get; private set; }
        public int LastSatellites { get; private set; }
        public int LastFixType { get; private set; }
        public int LastPitchDegrees { get; private set; }
        public int LastRollDegrees { get; private set; }
        public int LastHeadingDegrees { get; private set; }
        public int LastVoltageMillivolts { get; private set; }
        public int LastCurrentCentiamps { get; private set; }
        public int LastRssi { get; private set; }
        public int LastAirspeedMS { get; private set; }
        public int LastStatus { get; private set; }

        private void Restart(byte value)
        {
            frameLength = 0;
            frameOffset = 0;
            if(value == HeaderFirst)
            {
                frame[frameOffset++] = value;
            }
        }

        private void ParseFrame()
        {
            byte checksum = 0;
            for(var index = 3; index < frameLength - 1; index++)
            {
                checksum ^= frame[index];
            }
            if(checksum != frame[frameLength - 1])
            {
                InvalidFrames++;
                return;
            }
            ValidFrames++;
            if(frame[2] == GFrame)
            {
                GFrames++;
                LastLatitudeE7 = ReadInt32(3);
                LastLongitudeE7 = ReadInt32(7);
                LastGroundSpeedMS = frame[11];
                LastRelativeAltitudeCm = ReadInt32(12);
                LastSatellites = frame[16] >> 2;
                LastFixType = frame[16] & 0x3;
            }
            else if(frame[2] == AFrame)
            {
                AFrames++;
                LastPitchDegrees = ReadInt16(3);
                LastRollDegrees = ReadInt16(5);
                LastHeadingDegrees = ReadInt16(7);
            }
            else if(frame[2] == SFrame)
            {
                SFrames++;
                LastVoltageMillivolts = ReadUInt16(3);
                LastCurrentCentiamps = ReadUInt16(5);
                LastRssi = frame[7];
                LastAirspeedMS = frame[8];
                LastStatus = frame[9];
            }
        }

        private int ReadInt16(int offset)
        {
            return (short)ReadUInt16(offset);
        }

        private int ReadUInt16(int offset)
        {
            return frame[offset] | frame[offset + 1] << 8;
        }

        private int ReadInt32(int offset)
        {
            return frame[offset] |
                frame[offset + 1] << 8 |
                frame[offset + 2] << 16 |
                frame[offset + 3] << 24;
        }

        private static int FrameLength(byte frameType)
        {
            switch(frameType)
            {
            case GFrame:
                return GFrameLength;
            case AFrame:
                return AFrameLength;
            case SFrame:
                return SFrameLength;
            default:
                return 0;
            }
        }

        private readonly byte[] frame;
        private int frameLength;
        private int frameOffset;

        private const uint BaudRate = 2400;
        private const byte HeaderFirst = 0x24;
        private const byte HeaderSecond = 0x54;
        private const byte GFrame = 0x47;
        private const byte AFrame = 0x41;
        private const byte SFrame = 0x53;
        private const int GFrameLength = 18;
        private const int AFrameLength = 10;
        private const int SFrameLength = 11;
        private const int MaximumFrameLength = GFrameLength;
    }
}
