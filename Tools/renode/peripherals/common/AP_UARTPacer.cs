// STM32F7_USART advertises buffer-state flow control, causing a host terminal
// to inject a complete socket write as one atomic virtual-time event. Real
// UART bytes arrive over time, giving firmware time to consume each DMA
// request. Pace host-to-firmware traffic at the configured baud rate and
// do not release the next host byte until the USART has consumed the
// previous one; firmware output remains immediate.
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
            uartWithBufferState = uart as IUARTWithBufferState;
            if(uartWithBufferState != null)
            {
                uartWithBufferState.BufferStateChanged += UnderlyingBufferStateChanged;
            }
            uart.CharReceived += value => CharReceived?.Invoke(value);
        }

        public void Reset()
        {
            generation++;
            frameTimeElapsed = true;
            BufferState = BufferState.Empty;
        }

        public void WriteChar(byte value)
        {
            BufferState = BufferState.Full;
            frameTimeElapsed = false;
            var scheduledGeneration = ++generation;
            uart.WriteChar(value);

            var baudRate = BaudRate;
            var delayUs = (uint)Math.Ceiling(FrameBits * 1000000.0 / baudRate);
            machine.ScheduleAction(TimeInterval.FromMicroseconds(delayUs), _ =>
            {
                if(scheduledGeneration == generation)
                {
                    frameTimeElapsed = true;
                    TryReleaseHostByte();
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

        private void UnderlyingBufferStateChanged(BufferState state)
        {
            if(state == BufferState.Empty)
            {
                TryReleaseHostByte();
            }
        }

        private void TryReleaseHostByte()
        {
            if(!frameTimeElapsed || (uartWithBufferState != null &&
                uartWithBufferState.BufferState != BufferState.Empty))
            {
                return;
            }
            BufferState = BufferState.Ready;
        }

        private double FrameBits
        {
            get
            {
                var result = (double)StartAndDataBits;
                if(ParityBit != Parity.None)
                {
                    result++;
                }
                switch(StopBits)
                {
                case Bits.None:
                    return result;
                case Bits.Half:
                    return result + 0.5;
                case Bits.OneAndAHalf:
                    return result + 1.5;
                case Bits.Two:
                    return result + 2;
                default:
                    return result + 1;
                }
            }
        }

        [field: Transient]
        public event Action<byte> CharReceived;

        [field: Transient]
        public event Action<BufferState> BufferStateChanged;

        private readonly IMachine machine;
        private readonly IUART uart;
        private readonly IUARTWithBufferState uartWithBufferState;
        private BufferState bufferState;
        private bool frameTimeElapsed = true;
        private uint generation;
        private uint lastBaudRate = DefaultBaudRate;

        // One start bit plus the eight data bits modeled by STM32F7_USART.
        private const uint StartAndDataBits = 9;
        private const uint DefaultBaudRate = 115200;
    }
}
