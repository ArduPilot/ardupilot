// Minimal Benewake-compatible serial rangefinder.  It emits a steady 5 m
// reading using the common 0x59 frame accepted by the AP_RangeFinder serial
// backends.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_Benewake : IUART, IDoubleWordPeripheral, IKnownSize
    {
        public AP_Benewake(IMachine machine)
        {
            transmitter = machine.ObtainManagedThread(
                SendReading, ReadingsPerSecond, name: "AP Benewake range", owner: this);
            transmitter.Start();
        }

        public void Reset()
        {
        }

        public void WriteChar(byte value)
        {
        }

        public uint BaudRate => 115200;
        public Parity ParityBit => Parity.None;
        public Bits StopBits => Bits.One;
        public long Size => 4;

        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        public event Action<byte> CharReceived;

        private void SendReading()
        {
            var frame = new byte[] {
                0x59, 0x59,
                (byte)(DistanceCentimetres & 0xFF),
                (byte)(DistanceCentimetres >> 8),
                0x64, 0x00, // signal strength 100
                0x07, 0x00, // reliable TF02 signal / reserved
                0x00,
            };
            byte checksum = 0;
            for(var i = 0; i < frame.Length - 1; i++)
            {
                checksum += frame[i];
            }
            frame[frame.Length - 1] = checksum;
            foreach(var value in frame)
            {
                CharReceived?.Invoke(value);
            }
        }

        private readonly IManagedThread transmitter;
        private const uint ReadingsPerSecond = 20;
        private const ushort DistanceCentimetres = 500;
    }
}
