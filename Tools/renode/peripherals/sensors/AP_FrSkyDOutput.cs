// Receiver and analyser for ArduPilot FrSky D telemetry output.
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_FrSkyDOutputAnalyzer : AP_UARTFrameDevice
    {
        public AP_FrSkyDOutputAnalyzer(IMachine machine) : base(machine, 1, BaudRate)
        {
        }

        protected override byte[] BuildFrame()
        {
            return new byte[0];
        }

        public override void Reset()
        {
            base.Reset();
            state = State.WaitingForHeader;
            identifier = 0;
            data = 0;
            ValidFrames = 0;
            InvalidFrames = 0;
            StuffedBytes = 0;
            StatusFrames = 0;
            PositionFrames = 0;
            SpeedFrames = 0;
            AltitudeFrames = 0;
            LastGpsStatus = 0;
            LastLatitudeDegreeMinutes = 0;
            LastLatitudeMinuteFraction = 0;
            LastLatitudeHemisphere = 0;
            LastLongitudeDegreeMinutes = 0;
            LastLongitudeMinuteFraction = 0;
            LastLongitudeHemisphere = 0;
            LastSpeedMeters = 0;
            LastSpeedCentimeters = 0;
            LastGpsAltitudeMeters = 0;
            LastGpsAltitudeCentimeters = 0;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            switch(state)
            {
            case State.WaitingForHeader:
                if(value == Header)
                {
                    state = State.WaitingForIdentifier;
                }
                return;
            case State.WaitingForIdentifier:
                if(value == Header)
                {
                    return;
                }
                identifier = value;
                state = State.WaitingForDataLow;
                return;
            case State.WaitingForDataLow:
            case State.WaitingForDataHigh:
                AcceptDataByte(value);
                return;
            case State.WaitingForStuffedDataLow:
            case State.WaitingForStuffedDataHigh:
                AcceptStuffedByte(value);
                return;
            }
        }

        public ulong ValidFrames { get; private set; }
        public ulong InvalidFrames { get; private set; }
        public ulong StuffedBytes { get; private set; }
        public ulong StatusFrames { get; private set; }
        public ulong PositionFrames { get; private set; }
        public ulong SpeedFrames { get; private set; }
        public ulong AltitudeFrames { get; private set; }
        public int LastGpsStatus { get; private set; }
        public int LastLatitudeDegreeMinutes { get; private set; }
        public int LastLatitudeMinuteFraction { get; private set; }
        public int LastLatitudeHemisphere { get; private set; }
        public int LastLongitudeDegreeMinutes { get; private set; }
        public int LastLongitudeMinuteFraction { get; private set; }
        public int LastLongitudeHemisphere { get; private set; }
        public int LastSpeedMeters { get; private set; }
        public int LastSpeedCentimeters { get; private set; }
        public int LastGpsAltitudeMeters { get; private set; }
        public int LastGpsAltitudeCentimeters { get; private set; }

        private void AcceptDataByte(byte value)
        {
            if(value == Header)
            {
                InvalidFrames++;
                state = State.WaitingForIdentifier;
                return;
            }
            if(value == Escape)
            {
                state = state == State.WaitingForDataLow
                    ? State.WaitingForStuffedDataLow
                    : State.WaitingForStuffedDataHigh;
                return;
            }
            StoreDataByte(value);
        }

        private void AcceptStuffedByte(byte value)
        {
            if(value != EscapedHeader && value != EscapedEscape)
            {
                InvalidFrames++;
                state = value == Header
                    ? State.WaitingForIdentifier
                    : State.WaitingForHeader;
                return;
            }
            StuffedBytes++;
            StoreDataByte(value == EscapedHeader ? Header : Escape);
        }

        private void StoreDataByte(byte value)
        {
            if(state == State.WaitingForDataLow ||
                state == State.WaitingForStuffedDataLow)
            {
                data = value;
                state = State.WaitingForDataHigh;
                return;
            }
            data |= (ushort)(value << 8);
            ParseFrame();
            state = State.WaitingForHeader;
        }

        private void ParseFrame()
        {
            ValidFrames++;
            switch(identifier)
            {
            case DataIdGpsStatus:
                StatusFrames++;
                LastGpsStatus = data;
                break;
            case DataIdGpsLatitudeDegreeMinutes:
                PositionFrames++;
                LastLatitudeDegreeMinutes = data;
                break;
            case DataIdGpsLatitudeMinuteFraction:
                LastLatitudeMinuteFraction = data;
                break;
            case DataIdGpsLatitudeHemisphere:
                LastLatitudeHemisphere = data;
                break;
            case DataIdGpsLongitudeDegreeMinutes:
                LastLongitudeDegreeMinutes = data;
                break;
            case DataIdGpsLongitudeMinuteFraction:
                LastLongitudeMinuteFraction = data;
                break;
            case DataIdGpsLongitudeHemisphere:
                LastLongitudeHemisphere = data;
                break;
            case DataIdGpsSpeedMeters:
                SpeedFrames++;
                LastSpeedMeters = data;
                break;
            case DataIdGpsSpeedCentimeters:
                LastSpeedCentimeters = data;
                break;
            case DataIdGpsAltitudeMeters:
                AltitudeFrames++;
                LastGpsAltitudeMeters = data;
                break;
            case DataIdGpsAltitudeCentimeters:
                LastGpsAltitudeCentimeters = data;
                break;
            }
        }

        private State state;
        private byte identifier;
        private ushort data;

        private const uint BaudRate = 9600;
        private const byte Header = 0x5E;
        private const byte Escape = 0x5D;
        private const byte EscapedHeader = 0x3E;
        private const byte EscapedEscape = 0x3D;
        private const byte DataIdGpsAltitudeMeters = 0x01;
        private const byte DataIdGpsAltitudeCentimeters = 0x09;
        private const byte DataIdGpsSpeedMeters = 0x11;
        private const byte DataIdGpsLongitudeDegreeMinutes = 0x12;
        private const byte DataIdGpsLatitudeDegreeMinutes = 0x13;
        private const byte DataIdGpsStatus = 0x05;
        private const byte DataIdGpsSpeedCentimeters = 0x19;
        private const byte DataIdGpsLongitudeMinuteFraction = 0x1A;
        private const byte DataIdGpsLatitudeMinuteFraction = 0x1B;
        private const byte DataIdGpsLongitudeHemisphere = 0x22;
        private const byte DataIdGpsLatitudeHemisphere = 0x23;

        private enum State
        {
            WaitingForHeader,
            WaitingForIdentifier,
            WaitingForDataLow,
            WaitingForDataHigh,
            WaitingForStuffedDataLow,
            WaitingForStuffedDataHigh,
        }
    }
}
