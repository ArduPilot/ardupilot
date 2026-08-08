//
// Replays lost UART RX DMA requests. A real USART's DMA request is a
// level signal: if the stream is disabled when a byte arrives, the
// request is simply pending and serviced the moment the stream is
// re-enabled. Renode's STM32_UART instead emits an edge (Blink) at
// receive time only - and ArduPilot's RX path routinely has the stream
// disabled for a bounce-buffer harvest (IDLE interrupt) when traffic
// arrives. Each byte that lands in such a window loses its request,
// the stream runs permanently behind the fifo, and RX eventually goes
// deaf while TX keeps working.
//
// This helper hooks the stream's CR register: whenever EN is written 1
// it blinks one request per byte already queued in the UART's receive
// fifo. Reflection is only used to read the private fifo reference
// once at construction.
//
using System;
using System.Collections.Generic;
using System.Reflection;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.DMA;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_UartRxDmaPump : IDoubleWordPeripheral, IKnownSize
    {
        public AP_UartRxDmaPump(STM32DMA dma, int stream, STM32_UART uart)
        {
            var fifoField = typeof(STM32_UART).GetField("receiveFifo", BindingFlags.NonPublic | BindingFlags.Instance);
            if(fifoField == null)
            {
                throw new ConstructionException("STM32_UART no longer has a receiveFifo field - revisit this fix");
            }
            var fifo = (Queue<byte>)fifoField.GetValue(uart);

            dma.RegistersCollection.AddAfterWriteHook(0x10 + 0x18 * stream, (long offset, uint value) =>
            {
                if((value & 1) == 0)
                {
                    return;
                }
                // stream just enabled: service every request that went
                // missing while it was down
                var pending = fifo.Count;
                for(var i = 0; i < pending; i++)
                {
                    uart.DMARequest.Blink();
                }
            });
        }

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset() { }
    }
}
