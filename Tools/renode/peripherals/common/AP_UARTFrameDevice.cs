// Reusable timed UART frame source for ArduPilot serial peripheral models.
using System;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public abstract class AP_UARTFrameDevice : IUART, IDoubleWordPeripheral, IKnownSize
    {
        protected AP_UARTFrameDevice(IMachine machine, uint framesPerSecond,
            uint baudRate = 115200)
        {
            this.machine = machine;
            this.baudRate = baudRate;
            transmitter = machine.ObtainManagedThread(
                SendFrame, framesPerSecond, name: "AP UART peripheral frame", owner: this);
        }

        protected void StartTransmitter()
        {
            transmitter.Start();
        }

        public virtual void Reset()
        {
            generation++;
            transmissionAvailableUs = NowUs;
        }

        public virtual void WriteChar(byte value)
        {
            LastFirmwareByte = value;
            FirmwareBytesReceived++;
        }

        public uint BaudRate => baudRate;
        public Parity ParityBit => Parity.None;
        public Bits StopBits => Bits.One;
        public long Size => 4;

        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        public bool SuppressOutput { get; set; }
        public byte OutputXorMask { get; set; }
        public ulong FramesSent { get; private set; }
        public ulong BytesSent { get; private set; }
        public ulong FirmwareBytesReceived { get; private set; }
        public byte LastFirmwareByte { get; private set; }

        [field: Transient]
        public event Action<byte> CharReceived;

        protected abstract byte[] BuildFrame();

        protected void TransmitFrame(byte[] frame)
        {
            if(SuppressOutput || frame == null || frame.Length == 0)
            {
                return;
            }
            FramesSent++;
            var scheduledGeneration = generation;
            var delayUs = (ulong)Math.Ceiling(FrameBits * 1000000.0 / BaudRate);
            var nowUs = NowUs;
            var startUs = Math.Max(nowUs, transmissionAvailableUs);
            transmissionAvailableUs = startUs + (ulong)frame.Length * delayUs;
            for(var index = 0; index < frame.Length; index++)
            {
                var value = (byte)(frame[index] ^ OutputXorMask);
                var offsetUs = startUs - nowUs + (ulong)index * delayUs;
                machine.ScheduleAction(TimeInterval.FromMicroseconds(offsetUs), _ =>
                {
                    if(scheduledGeneration != generation || SuppressOutput)
                    {
                        return;
                    }
                    BytesSent++;
                    CharReceived?.Invoke(value);
                }, name: "AP UART peripheral output");
            }
        }

        private void SendFrame()
        {
            if(SuppressOutput)
            {
                return;
            }
            var frame = BuildFrame();
            TransmitFrame(frame);
        }

        private readonly IMachine machine;
        private readonly IManagedThread transmitter;
        private readonly uint baudRate;
        private uint generation;
        private ulong transmissionAvailableUs;

        private ulong NowUs =>
            (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;

        // One start bit, eight data bits and one stop bit.
        private const uint FrameBits = 10;
    }
}
