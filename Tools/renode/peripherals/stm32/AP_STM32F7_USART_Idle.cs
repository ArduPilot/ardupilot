//
// Adds the USART IDLE condition used by ChibiOS to harvest a partial
// UART RX DMA buffer. STM32F7_USART models RXNE and DMA requests, but
// its IDLE status and interrupt-enable bits are tags. ArduPilot then
// receives bytes into DMA memory without being told that a MAVLink
// frame ended until the entire bounce buffer fills.
//
// The transition back to an empty USART FIFO happens after DMA reads
// RDR. Raise IDLE then; the CPU normally handles the interrupt after a
// complete host-side burst, while repeated transitions coalesce just
// like a level interrupt. ICR.IDLECF clears the synthetic condition.
//
using System.Reflection;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32F7_USART_Idle : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32F7_USART_Idle(STM32F7_USART uart)
        {
            var updateInterrupt = typeof(STM32F7_USART).GetMethod("UpdateInterrupt",
                BindingFlags.NonPublic | BindingFlags.Instance);
            if(updateInterrupt == null)
            {
                throw new ConstructionException("STM32F7_USART no longer has UpdateInterrupt - revisit this fix");
            }

            uart.RegistersCollection.AddAfterReadHook(InterruptAndStatus,
                (long offset, uint value) => idlePending ? value | Idle : value);
            uart.RegistersCollection.AddAfterWriteHook(InterruptFlagClear,
                (long offset, uint value) =>
                {
                    if((value & Idle) == 0)
                    {
                        return;
                    }
                    idlePending = false;
                    updateInterrupt.Invoke(uart, null);
                });

            uart.BufferStateChanged += state =>
            {
                if(state == BufferState.Ready)
                {
                    receivedData = true;
                    return;
                }
                if(state != BufferState.Empty || !receivedData)
                {
                    return;
                }
                receivedData = false;
                idlePending = true;
                uart.IRQ.Set(true);
            };
        }

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        public void Reset()
        {
            idlePending = false;
            receivedData = false;
        }

        private bool idlePending;
        private bool receivedData;

        private const long InterruptAndStatus = 0x1C;
        private const long InterruptFlagClear = 0x20;
        private const uint Idle = 1 << 4;
    }
}
