// STM32F7_USART advertises buffer-state flow control, causing a host terminal
// to inject a complete socket write as one atomic virtual-time event. Real
// UART bytes arrive over time, giving firmware time to rearm DMA between
// bounce buffers. Pace host-to-firmware traffic at the configured baud
// rate with a small minimum service interval; firmware output remains
// immediate.
//
using System;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_UARTPacer : IUARTWithBufferState, IDoubleWordPeripheral, IKnownSize
    {
        public AP_UARTPacer(IMachine machine, IUART uart)
        {
            this.machine = machine;
            this.uart = uart;
            uart.CharReceived += value => CharReceived?.Invoke(value);
        }

        public void Reset()
        {
            generation++;
            BufferState = BufferState.Empty;
        }

        public void WriteChar(byte value)
        {
            BufferState = BufferState.Full;
            uart.WriteChar(value);

            var baudRate = BaudRate;
            var delayUs = Math.Max(MinimumDelayUs,
                (uint)Math.Ceiling(BitsPerByte * 1000000.0 / baudRate));
            var scheduledGeneration = generation;
            machine.ScheduleAction(TimeInterval.FromMicroseconds(delayUs), _ =>
            {
                if(scheduledGeneration == generation)
                {
                    BufferState = BufferState.Ready;
                }
            }, name: "AP UART host input");
        }

        public uint BaudRate
        {
            get
            {
                try
                {
                    var baudRate = uart.BaudRate;
                    if(baudRate != 0)
                    {
                        lastBaudRate = baudRate;
                    }
                }
                catch(DivideByZeroException)
                {
                    // STM32F7_USART divides by BRR in its BaudRate getter.
                    // BRR is briefly zero while the peripheral is resetting.
                }
                return lastBaudRate;
            }
        }
        public Parity ParityBit => uart.ParityBit;
        public Bits StopBits => uart.StopBits;
        public long Size => 4;

        public BufferState BufferState
        {
            get => bufferState;
            private set
            {
                if(bufferState == value)
                {
                    return;
                }
                bufferState = value;
                BufferStateChanged?.Invoke(value);
            }
        }

        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        [field: Transient]
        public event Action<byte> CharReceived;

        [field: Transient]
        public event Action<BufferState> BufferStateChanged;

        private readonly IMachine machine;
        private readonly IUART uart;
        private BufferState bufferState;
        private uint generation;
        private uint lastBaudRate = DefaultBaudRate;

        private const uint BitsPerByte = 10;
        private const uint DefaultBaudRate = 115200;
        // Renode's interrupt and DMA handoff need more virtual time than
        // the modeled USART frame at common telemetry baud rates.
        private const uint MinimumDelayUs = 500;
    }
}
