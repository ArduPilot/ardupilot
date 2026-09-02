// Minimal LightWare serial lidar using the legacy ASCII protocol.
// A steady 5 m sample is accepted by AP_RangeFinder_LightWareSerial.
using System;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class AP_LightWare : IUART, IDoubleWordPeripheral, IKnownSize
    {
        public AP_LightWare(IMachine machine)
        {
            reading = Encoding.ASCII.GetBytes("5.00\r");
            transmitter = machine.ObtainManagedThread(
                SendReading, ReadingsPerSecond,
                name: "AP LightWare range", owner: this);
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
            foreach(var value in reading)
            {
                CharReceived?.Invoke(value);
            }
        }

        private readonly byte[] reading;
        private readonly IManagedThread transmitter;
        private const uint ReadingsPerSecond = 20;
    }
}
