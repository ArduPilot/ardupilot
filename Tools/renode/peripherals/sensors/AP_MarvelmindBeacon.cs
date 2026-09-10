// Physics-driven Marvelmind UWB beacon receiver.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_MarvelmindBeacon : AP_UARTFrameDevice
    {
        public AP_MarvelmindBeacon(IMachine machine) :
            base(machine, FramesPerSecond, BaudRate)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var truth = physics.Current;
            var result = new List<byte>();
            result.AddRange(AnchorPositions());
            result.AddRange(VehiclePosition(truth));
            result.AddRange(AnchorDistances(truth));
            return result.ToArray();
        }

        public bool CorruptChecksum { get; set; }

        private byte[] AnchorPositions()
        {
            var payload = new List<byte> { (byte)AnchorsNedM.Length };
            for(var index = 0; index < AnchorsNedM.Length; index++)
            {
                var anchor = AnchorsNedM[index];
                payload.Add((byte)(index + 1));
                AddInt32(payload, Millimetres(anchor[1]));
                AddInt32(payload, Millimetres(anchor[0]));
                AddInt32(payload, Millimetres(-anchor[2]));
                payload.Add(0);
            }
            return BuildDatagram(AnchorPositionsHighResolution, payload);
        }

        private byte[] VehiclePosition(AP_PhysicsTruth truth)
        {
            var payload = new List<byte>();
            AddUInt32(payload, truth.Sequence);
            AddInt32(payload, Millimetres(truth.PositionNedM[1]));
            AddInt32(payload, Millimetres(truth.PositionNedM[0]));
            AddInt32(payload, Millimetres(-truth.PositionNedM[2]));
            payload.Add(0);
            payload.Add(HedgeAddress);
            while(payload.Count < VehiclePositionPayloadLength)
            {
                payload.Add(0);
            }
            return BuildDatagram(VehiclePositionHighResolution, payload);
        }

        private byte[] AnchorDistances(AP_PhysicsTruth truth)
        {
            var payload = new List<byte> { 0 };
            for(var index = 0; index < AnchorsNedM.Length; index++)
            {
                var anchor = AnchorsNedM[index];
                var north = truth.PositionNedM[0] - anchor[0];
                var east = truth.PositionNedM[1] - anchor[1];
                var down = truth.PositionNedM[2] - anchor[2];
                var distance = Math.Sqrt(
                    north * north + east * east + down * down);
                payload.Add((byte)(index + 1));
                AddUInt32(payload, (uint)Math.Round(distance * 1000.0));
                payload.Add(0);
            }
            while(payload.Count < AnchorDistancesPayloadLength)
            {
                payload.Add(0);
            }
            return BuildDatagram(AnchorDistancesId, payload);
        }

        private byte[] BuildDatagram(ushort id, List<byte> payload)
        {
            var result = new List<byte> {
                HeaderFirst, HeaderSecond, (byte)id, (byte)(id >> 8),
                (byte)payload.Count,
            };
            result.AddRange(payload);
            var checksum = ModbusCrc(result);
            if(CorruptChecksum)
            {
                checksum ^= 0xFFFF;
            }
            result.Add((byte)checksum);
            result.Add((byte)(checksum >> 8));
            return result.ToArray();
        }

        private static ushort ModbusCrc(List<byte> data)
        {
            ushort crc = 0xFFFF;
            foreach(var value in data)
            {
                crc ^= value;
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (ushort)((crc & 1) != 0 ?
                        (crc >> 1) ^ 0xA001 : crc >> 1);
                }
            }
            return crc;
        }

        private static int Millimetres(double metres)
        {
            return (int)Math.Round(metres * 1000.0);
        }

        private static void AddInt32(List<byte> result, int value)
        {
            AddUInt32(result, (uint)value);
        }

        private static void AddUInt32(List<byte> result, uint value)
        {
            result.Add((byte)value);
            result.Add((byte)(value >> 8));
            result.Add((byte)(value >> 16));
            result.Add((byte)(value >> 24));
        }

        private readonly AP_PhysicsState physics;

        private const uint FramesPerSecond = 10;
        private const uint BaudRate = 115200;
        private const byte HeaderFirst = 0xFF;
        private const byte HeaderSecond = 0x47;
        private const ushort AnchorDistancesId = 0x0004;
        private const ushort VehiclePositionHighResolution = 0x0011;
        private const ushort AnchorPositionsHighResolution = 0x0012;
        private const byte VehiclePositionPayloadLength = 0x16;
        private const byte AnchorDistancesPayloadLength = 32;
        private const byte HedgeAddress = 42;

        private static readonly double[][] AnchorsNedM = {
            new double[] { 10.0, 0.0, 0.0 },
            new double[] { 0.0, 10.0, 0.0 },
            new double[] { -10.0, 0.0, 0.0 },
            new double[] { 0.0, -10.0, 0.0 },
        };
    }
}
