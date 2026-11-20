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
// The stock model also completes memory-to-memory and memory-to-peripheral
// transfers immediately but leaves SxCR.EN set. Hardware clears EN when the
// transfer finishes; bootloader flash ECC checks and UART TX both rely on
// that transition before starting another transfer.
// Clear it after the model has performed the transfer.
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
using Antmicro.Migrant;
using Antmicro.Migrant.Hooks;
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
            var tcStatusField = typeof(STM32DMA).GetField("transferCompleteIrqStatus", BindingFlags.NonPublic | BindingFlags.Instance);
            var updateInterrupts = typeof(STM32DMA).GetMethod("UpdateInterrupts", BindingFlags.NonPublic | BindingFlags.Instance);
            if(tcStatusField == null || updateInterrupts == null)
            {
                throw new ConstructionException("STM32DMA transfer complete internals changed - revisit this fix");
            }
            var tcStatus = (Array)tcStatusField.GetValue(dma);
            var enabled = new bool[streams.Length];

            for(var i = 0; i < streams.Length; i++)
            {
                // SxCR = 0x10 + 0x18 * stream
                var configurationOffset = 0x10 + 0x18 * i;
                var resetter = new OffsetResetter(streams.GetValue(i));
                // SxNDTR = 0x14 + 0x18 * stream
                dma.RegistersCollection.AddAfterWriteHook(0x14 + 0x18 * i, resetter.Reset);
                var stream = i;
                dma.RegistersCollection.AddAfterWriteHook(configurationOffset, (offset, value) =>
                {
                    const uint enable = 1U << 0;
                    const uint directionMask = 3U << 6;
                    const uint peripheralToMemory = 0;
                    // hardware sets TCIF when software disables an active
                    // stream, ArduPilot's F4 UART IDLE handler relies on it
                    // to harvest a partly filled RX buffer
                    var wasEnabled = enabled[stream];
                    enabled[stream] = (value & enable) != 0;
                    if(wasEnabled && !enabled[stream] &&
                        (value & directionMask) == peripheralToMemory &&
                        dma.ReadDoubleWord(0x14 + 0x18 * stream) != 0)
                    {
                        ((IFlagRegisterField)tcStatus.GetValue(stream)).Value = true;
                        updateInterrupts.Invoke(dma, null);
                    }
                    if((value & enable) != 0 &&
                        (value & directionMask) != peripheralToMemory)
                    {
                        dma.WriteDoubleWord(offset, value & ~enable);
                    }
                });
            }
        }

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset() { }

        private sealed class OffsetResetter
        {
            public OffsetResetter(object stream)
            {
                this.stream = stream;
                ResolveOffsetField();
            }

            public void Reset(long offset, uint value)
            {
                offsetField.SetValue(stream, 0UL);
            }

            [PostDeserialization]
            private void AfterDeserialization()
            {
                ResolveOffsetField();
            }

            private void ResolveOffsetField()
            {
                offsetField = stream.GetType().GetField("dataOffset", BindingFlags.NonPublic | BindingFlags.Instance);
                if(offsetField == null)
                {
                    throw new ConstructionException("STM32DMA stream no longer has a dataOffset field - revisit this fix");
                }
            }

            private readonly object stream;

            [Transient]
            private FieldInfo offsetField;
        }
    }
}
