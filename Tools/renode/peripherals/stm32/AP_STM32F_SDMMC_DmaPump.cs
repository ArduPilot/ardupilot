// Corrects STM32F4/F7 SDIO/SDMMC DMA ordering after both halves of a transfer
// are armed. Renode emits read requests while filling its FIFO, before ChibiOS
// enables the selected stream. In the other direction, Renode performs a
// complete memory-to-peripheral transfer when the stream is enabled, before
// the SD write command has prepared its FIFO.
//
// The hardware request is level-sensitive. Re-checking the FIFO replays reads;
// deferring only SDMMC-bound write enables until CMD24/25 prepares the FIFO
// lets the stock SDMMC and DMA models perform the actual transfer.
//
using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.DMA;
using Antmicro.Renode.Peripherals.SD;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32F_SDMMC_DmaPump : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32F_SDMMC_DmaPump(IMachine machine, STM32FSDMMC sdmmc,
                                      STM32DMA dma, int stream, uint fifoAddress)
        {
            this.machine = machine;
            this.sdmmc = sdmmc;
            this.dma = dma;
            streamControl = StreamControlBase + StreamStride * stream;
            sdmmcFifoAddress = fifoAddress;
            sdmmc.RegistersCollection.AddAfterWriteHook(Command, CommandWritten);
            sdmmc.RegistersCollection.AddAfterWriteHook(DataControl, PumpRequests);
            dma.RegistersCollection.AddBeforeWriteHook(streamControl, DmaControlWriting);
        }

        public long Size => 4;
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public void Reset() { }

        private void PumpRequests(long offset, uint value)
        {
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ => PumpNow());
        }

        private void CommandWritten(long offset, uint value)
        {
            currentCommand = value & CommandIndexMask;
            PumpRequests(offset, value);
        }

        private uint? DmaControlWriting(long offset, uint value)
        {
            if(enablingWriteDma || (value & StreamEnable) == 0 ||
               (value & StreamDirectionMask) != StreamMemoryToPeripheral ||
               dma.ReadDoubleWord(streamControl + StreamPeripheralOffset) != sdmmcFifoAddress)
            {
                return null;
            }

            pendingWriteControl = value;
            return value & ~StreamEnable;
        }

        private void PumpNow()
        {
            var pendingBytes = sdmmc.ReadDoubleWord(FifoCount);
            var pendingWords = pendingBytes / sizeof(uint);
            if(pendingWords > 0 && pendingWriteControl != 0 && IsWriteCommand(currentCommand))
            {
                enablingWriteDma = true;
                dma.WriteDoubleWord(streamControl, pendingWriteControl);
                enablingWriteDma = false;
                pendingWriteControl = 0;
            }
            for(var i = 0; i < pendingWords; i++)
            {
                sdmmc.DMAReceive.Blink();
            }
            var control = dma.ReadDoubleWord(streamControl);
            var transferComplete =
                (sdmmc.ReadDoubleWord(Status) & DataEnd) != 0 &&
                dma.ReadDoubleWord(streamControl + StreamCountOffset) == 0;
            if((control & StreamEnable) != 0 && transferComplete)
            {
                dma.WriteDoubleWord(streamControl, control & ~StreamEnable);
            }
            if(pendingWords > 0 && currentCommand == ReadSingleBlockCommand)
            {
                var card = sdmmc.Children.Select(child => child.Peripheral).FirstOrDefault();
                card?.HandleCommand(StopTransmissionCommand, 0);
            }
        }

        private static bool IsWriteCommand(uint command)
        {
            return command == WriteSingleBlockCommand || command == WriteMultipleBlocksCommand;
        }

        private readonly IMachine machine;
        private readonly STM32FSDMMC sdmmc;
        private readonly STM32DMA dma;
        private readonly long streamControl;
        private readonly uint sdmmcFifoAddress;
        private bool enablingWriteDma;
        private uint pendingWriteControl;
        private uint currentCommand;

        private const long Command = 0x0C;
        private const long DataControl = 0x2C;
        private const long Status = 0x34;
        private const long FifoCount = 0x48;
        private const long StreamControlBase = 0x10;
        private const long StreamStride = 0x18;
        private const long StreamCountOffset = 0x04;
        private const long StreamPeripheralOffset = 0x08;
        private const uint StreamEnable = 1u << 0;
        private const uint StreamDirectionMask = 3u << 6;
        private const uint StreamMemoryToPeripheral = 1u << 6;
        private const uint DataEnd = 1u << 8;
        private const uint CommandIndexMask = 0x3Fu;
        private const uint ReadSingleBlockCommand = 17;
        private const uint WriteSingleBlockCommand = 24;
        private const uint WriteMultipleBlocksCommand = 25;
        private const uint StopTransmissionCommand = 12;
    }
}
