// Receiver and analyser for ArduPilot Devo telemetry output.
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_DevoOutputAnalyzer : AP_UARTFrameDevice
    {
        public AP_DevoOutputAnalyzer(IMachine machine) : base(machine, 1, BaudRate)
        {
            frame = new byte[FrameLength];
        }

        protected override byte[] BuildFrame()
        {
            return new byte[0];
        }

        public override void Reset()
        {
            base.Reset();
            frameOffset = 0;
            ValidFrames = 0;
            InvalidFrames = 0;
            LastLongitudeDmsE7 = 0;
            LastLatitudeDmsE7 = 0;
            LastRelativeAltitudeCm = 0;
            LastSpeed = 0;
            LastMode = 0;
            LastVoltageDecivolts = 0;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            if(frameOffset == 0 && value != Header)
            {
                return;
            }
            frame[frameOffset++] = value;
            if(frameOffset != FrameLength)
            {
                return;
            }
            ParseFrame();
            frameOffset = 0;
        }

        public ulong ValidFrames { get; private set; }
        public ulong InvalidFrames { get; private set; }
        public int LastLongitudeDmsE7 { get; private set; }
        public int LastLatitudeDmsE7 { get; private set; }
        public int LastRelativeAltitudeCm { get; private set; }
        public int LastSpeed { get; private set; }
        public int LastMode { get; private set; }
        public int LastVoltageDecivolts { get; private set; }

        private void ParseFrame()
        {
            byte checksum = 0;
            for(var index = 0; index < FrameLength - 1; index++)
            {
                checksum = (byte)(checksum + frame[index]);
            }
            if(checksum != frame[FrameLength - 1])
            {
                InvalidFrames++;
                return;
            }
            ValidFrames++;
            LastLongitudeDmsE7 = ReadInt32(1);
            LastLatitudeDmsE7 = ReadInt32(5);
            LastRelativeAltitudeCm = ReadInt32(9);
            LastSpeed = ReadInt16(13);
            LastMode = ReadInt16(15);
            LastVoltageDecivolts = ReadInt16(17);
        }

        private int ReadInt16(int offset)
        {
            return (short)(frame[offset] | frame[offset + 1] << 8);
        }

        private int ReadInt32(int offset)
        {
            return frame[offset] |
                frame[offset + 1] << 8 |
                frame[offset + 2] << 16 |
                frame[offset + 3] << 24;
        }

        private readonly byte[] frame;
        private int frameOffset;

        private const uint BaudRate = 38400;
        private const byte Header = 0xAA;
        private const int FrameLength = 20;
    }
}
