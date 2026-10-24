// Physics-driven Nooploop UWB beacon receiver.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_NooploopBeacon : AP_UARTFrameDevice
    {
        public AP_NooploopBeacon(IMachine machine) :
            base(machine, FramesPerSecond, BaudRate)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            firmwareRequest = new byte[SettingFrameLength];
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var truth = physics.Current;
            var result = new byte[NodeFrameLength];
            result[0] = NodeHeader;
            result[1] = NodeFrameFunction;
            result[2] = (byte)result.Length;
            result[3] = (byte)(result.Length >> 8);
            AddUInt32(result, 6, truth.Sequence);
            result[10] = PositionPrecisionCentimetres;
            result[11] = PositionPrecisionCentimetres;
            result[12] = PositionPrecisionCentimetres;
            AddInt24(result, 13, Millimetres(truth.PositionNedM[1]));
            AddInt24(result, 16, Millimetres(truth.PositionNedM[0]));
            AddInt24(result, 19, Millimetres(-truth.PositionNedM[2]));
            result[ValidNodesOffset] = (byte)AnchorsNedM.Length;
            for(var index = 0; index < AnchorsNedM.Length; index++)
            {
                var anchor = AnchorsNedM[index];
                var north = truth.PositionNedM[0] - anchor[0];
                var east = truth.PositionNedM[1] - anchor[1];
                var down = truth.PositionNedM[2] - anchor[2];
                var distance = Math.Sqrt(
                    north * north + east * east + down * down);
                var offset = NodeBlockOffset + index * NodeBlockLength;
                result[offset + 1] = (byte)index;
                AddInt24(result, offset + 2, Millimetres(distance));
            }
            AddChecksum(result);
            return result;
        }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            if(value != ExpectedRequestByte(firmwareRequestLength))
            {
                firmwareRequestLength = value == SettingHeader ? 1 : 0;
                return;
            }
            firmwareRequest[firmwareRequestLength++] = value;
            if(firmwareRequestLength != SettingFrameLength)
            {
                return;
            }
            firmwareRequestLength = 0;
            SettingRequests++;
            TransmitFrame(BuildSettingFrame());
        }

        public bool CorruptChecksum { get; set; }
        public ulong SettingRequests { get; private set; }

        private byte[] BuildSettingFrame()
        {
            var result = new byte[SettingFrameLength];
            result[0] = SettingHeader;
            result[1] = SettingFrameFunction;
            for(var index = 0; index < AnchorsNedM.Length; index++)
            {
                var anchor = AnchorsNedM[index];
                var offset = AnchorPositionsOffset + index * AnchorPositionLength;
                AddInt24(result, offset, Millimetres(anchor[1]));
                AddInt24(result, offset + 3, Millimetres(anchor[0]));
                AddInt24(result, offset + 6, Millimetres(-anchor[2]));
            }
            AddChecksum(result);
            return result;
        }

        private byte ExpectedRequestByte(int offset)
        {
            if(offset == 0)
            {
                return SettingHeader;
            }
            if(offset == 2)
            {
                return 1;
            }
            if(offset == SettingFrameLength - 1)
            {
                return NodeHeader;
            }
            return 0;
        }

        private void AddChecksum(byte[] data)
        {
            byte checksum = 0;
            for(var index = 0; index < data.Length - 1; index++)
            {
                checksum = (byte)(checksum + data[index]);
            }
            data[data.Length - 1] = CorruptChecksum ?
                (byte)(checksum ^ 0xFF) : checksum;
        }

        private static int Millimetres(double metres)
        {
            return (int)Math.Round(metres * 1000.0);
        }

        private static void AddInt24(byte[] result, int offset, int value)
        {
            result[offset] = (byte)value;
            result[offset + 1] = (byte)(value >> 8);
            result[offset + 2] = (byte)(value >> 16);
        }

        private static void AddUInt32(byte[] result, int offset, uint value)
        {
            result[offset] = (byte)value;
            result[offset + 1] = (byte)(value >> 8);
            result[offset + 2] = (byte)(value >> 16);
            result[offset + 3] = (byte)(value >> 24);
        }

        private readonly AP_PhysicsState physics;
        private readonly byte[] firmwareRequest;
        private int firmwareRequestLength;

        private const uint FramesPerSecond = 10;
        private const uint BaudRate = 115200;
        private const byte NodeHeader = 0x55;
        private const byte NodeFrameFunction = 4;
        private const int NodeFrameLength = 172;
        private const int ValidNodesOffset = 118;
        private const int NodeBlockOffset = 119;
        private const int NodeBlockLength = 13;
        private const byte SettingHeader = 0x54;
        private const byte SettingFrameFunction = 0;
        private const int SettingFrameLength = 128;
        private const int AnchorPositionsOffset = 37;
        private const int AnchorPositionLength = 9;
        private const byte PositionPrecisionCentimetres = 5;

        private static readonly double[][] AnchorsNedM = {
            new double[] { 10.0, 0.0, 0.0 },
            new double[] { 0.0, 10.0, 0.0 },
            new double[] { -10.0, 0.0, 0.0 },
            new double[] { 0.0, -10.0, 0.0 },
        };
    }
}
