// Adds circular-stream reload behavior to Renode's STM32 DMA model.
// CIRC is only a tag in the stock model, so it is lost on a firmware
// read/modify/write and NDTR remains zero after the first buffer.
using System;
using System.Reflection;
using Antmicro.Migrant;
using Antmicro.Migrant.Hooks;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.DMA;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32DMA_Circular : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32DMA_Circular(IMachine machine, STM32DMA dma, int stream)
        {
            var streamsField = typeof(STM32DMA).GetField(
                "streams", BindingFlags.NonPublic | BindingFlags.Instance);
            if(streamsField == null)
            {
                throw new ConstructionException(
                    "STM32DMA no longer has a streams field - revisit this fix");
            }
            var streams = (Array)streamsField.GetValue(dma);
            dmaStream = streams.GetValue(stream);
            ResolveFields();

            // SxCR = 0x10 + 0x18 * stream.  Use a before-write hook so this
            // can coexist with the UART helpers' after-write hooks.
            dma.RegistersCollection.AddBeforeWriteHook(
                0x10 + 0x18 * stream, ConfigurationWritten);
            poller = machine.ObtainManagedThread(
                Reload, PollFrequency,
                name: "AP STM32 DMA circular reload", owner: this);
            poller.Start();
        }

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }

        public void Reset()
        {
            circular = false;
            reloadCount = 0;
        }

        private uint? ConfigurationWritten(long offset, uint value)
        {
            // Keep this sticky because the stock tagged bit reads back as 0.
            circular |= (value & CircularMode) != 0;
            return null;
        }

        private void Reload()
        {
            var count = numberOfData.Value;
            if(count > reloadCount)
            {
                reloadCount = count;
            }
            else if(circular && reloadCount != 0 && count == 0)
            {
                numberOfData.Value = reloadCount;
            }
        }

        [PostDeserialization]
        private void AfterDeserialization()
        {
            ResolveFields();
        }

        private void ResolveFields()
        {
            var field = dmaStream.GetType().GetField(
                "nrOfData", BindingFlags.NonPublic | BindingFlags.Instance);
            if(field == null)
            {
                throw new ConstructionException(
                    "STM32DMA stream no longer has an nrOfData field - revisit this fix");
            }
            numberOfData = (IValueRegisterField)field.GetValue(dmaStream);
        }

        private readonly object dmaStream;
        private readonly IManagedThread poller;
        private bool circular;
        private ulong reloadCount;

        [Transient]
        private IValueRegisterField numberOfData;

        private const uint CircularMode = 1U << 8;
        private const uint PollFrequency = 100000;
    }
}
