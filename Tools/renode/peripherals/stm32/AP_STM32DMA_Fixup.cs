//
// Fixes a stock-STM32DMA behaviour that corrupts ArduPilot's UART RX:
// each stream's internal memory offset ("dataOffset") only resets when
// NDTR counts down to zero. ArduPilot's RX path routinely disables a
// stream part-way through the bounce buffer (on the UART IDLE
// interrupt), rewrites NDTR and re-enables - on real hardware that
// restarts from M0AR, but the stock model keeps the stale offset and
// scribbles the next fill further into memory. The same pattern hits
// aborted SPI transfers (20ms timeout path).
//
// Real hardware reloads its internal pointers whenever NDTR is written
// (writes are only legal with the stream disabled), so an after-write
// hook on every SxNDTR register that zeroes the stream's dataOffset is
// hardware-faithful. Reflection is only used to reach the private
// stream array and offset field once at construction.
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

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32DMA_Fixup : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32DMA_Fixup(STM32DMA dma)
        {
            var streamsField = typeof(STM32DMA).GetField("streams", BindingFlags.NonPublic | BindingFlags.Instance);
            if(streamsField == null)
            {
                throw new ConstructionException("STM32DMA no longer has a streams field - revisit this fix");
            }
            var streams = (Array)streamsField.GetValue(dma);
            var offsetField = streams.GetValue(0).GetType().GetField("dataOffset", BindingFlags.NonPublic | BindingFlags.Instance);
            if(offsetField == null)
            {
                throw new ConstructionException("STM32DMA stream no longer has a dataOffset field - revisit this fix");
            }

            for(var i = 0; i < streams.Length; i++)
            {
                var stream = streams.GetValue(i);
                // SxNDTR = 0x14 + 0x18 * stream
                dma.RegistersCollection.AddAfterWriteHook(0x14 + 0x18 * i, (long offset, uint value) =>
                {
                    offsetField.SetValue(stream, 0UL);
                });
            }
        }

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset() { }
    }
}
