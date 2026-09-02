// Physics-driven Pozyx UWB beacon receiver.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_PozyxBeacon : AP_UARTFrameDevice
    {
        public AP_PozyxBeacon(IMachine machine) :
            base(machine, FramesPerSecond, BaudRate)
        {
            physics = AP_PhysicsState.ForMachine(machine);
            StartTransmitter();
        }

        protected override byte[] BuildFrame()
        {
            var truth = physics.Current;
            var result = new List<byte>();
            for(var index = 0; index < AnchorsNedM.Length; index++)
            {
                var anchor = AnchorsNedM[index];
                var configuration = new List<byte> {
                    (byte)index,
                    (byte)AnchorsNedM.Length,
                };
                AddInt32(configuration, Millimetres(anchor[0]));
                AddInt32(configuration, Millimetres(anchor[1]));
                AddInt32(configuration, Millimetres(-anchor[2]));
                result.AddRange(BuildMessage(BeaconConfiguration,
                    configuration));

                var deltaNorth = truth.PositionNedM[0] - anchor[0];
                var deltaEast = truth.PositionNedM[1] - anchor[1];
                var deltaDown = truth.PositionNedM[2] - anchor[2];
                var distance = Math.Sqrt(
                    deltaNorth * deltaNorth + deltaEast * deltaEast +
                    deltaDown * deltaDown);
                var range = new List<byte> { (byte)index };
                AddUInt32(range, (uint)Math.Round(distance * 1000.0));
                result.AddRange(BuildMessage(BeaconDistance, range));
            }

            var position = new List<byte>();
            AddInt32(position, Millimetres(truth.PositionNedM[0]));
            AddInt32(position, Millimetres(truth.PositionNedM[1]));
            AddInt32(position, Millimetres(-truth.PositionNedM[2]));
            AddInt16(position, PositionError);
            result.AddRange(BuildMessage(VehiclePosition, position));
            return result.ToArray();
        }

        public bool CorruptChecksum { get; set; }

        private byte[] BuildMessage(byte messageId, List<byte> payload)
        {
            var length = payload.Count + 1;
            byte checksum = (byte)(messageId ^ length);
            foreach(var value in payload)
            {
                checksum ^= value;
            }
            if(CorruptChecksum)
            {
                checksum ^= 0xFF;
            }

            var result = new List<byte> { Header, messageId, (byte)length };
            result.AddRange(payload);
            result.Add(checksum);
            return result.ToArray();
        }

        private static int Millimetres(double metres)
        {
            return (int)Math.Round(metres * 1000.0);
        }

        private static void AddInt16(List<byte> result, short value)
        {
            result.Add((byte)value);
            result.Add((byte)(value >> 8));
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
        private const byte Header = 0x01;
        private const byte BeaconConfiguration = 0x02;
        private const byte BeaconDistance = 0x03;
        private const byte VehiclePosition = 0x04;
        private const short PositionError = 5;

        private static readonly double[][] AnchorsNedM = {
            new double[] { 10.0, 0.0, 0.0 },
            new double[] { 0.0, 10.0, 0.0 },
            new double[] { -10.0, 0.0, 0.0 },
            new double[] { 0.0, -10.0, 0.0 },
        };
    }
}
