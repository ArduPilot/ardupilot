// Adapt Renode's SDCard SPI model to the STM32/ChibiOS transfer sequences.
// The model emits an extra inter-block read byte, while STM32 receive DMA
// clocks a fixed 0xFF value which Renode does not currently preserve.

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Peripherals.SD;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_SPI_SDCard : SDCard, ISPIPeripheral
    {
        public AP_SPI_SDCard(string fileName, long size, bool persistent = true) :
            base(fileName, size, persistent, spiMode: true)
        {
        }

        public new byte Transmit(byte data)
        {
            if(multipleBlockRead && readPayloadBytesRemaining == 0 &&
                data == StopTransmissionCommand)
            {
                // Consume the model's inter-block dummy before CMD12.
                base.Transmit(DummyByte);
            }

            var writeByte = TrackWriteByte(data);
            if(!writeByte)
            {
                TrackCommandByte(data);
            }

            var payloadInProgress = readPayloadBytesRemaining > 0;
            var receiveClock = data == 0 && !writeByte &&
                commandBytesRemaining == 0;
            // Renode's STM32 DMA currently clocks zero after the first byte of
            // a receive. Real ChibiOS hardware uses its fixed 0xFF dummy word,
            // and the SD model requires that value while returning data or a
            // command response.
            var response = base.Transmit(
                payloadInProgress || receiveClock ? DummyByte : data);
            if(payloadInProgress)
            {
                readPayloadBytesRemaining--;
            }
            else if(multipleBlockRead && response == DataToken)
            {
                readPayloadBytesRemaining = BlockSize + CrcSize;
            }
            return response;
        }

        public new void FinishTransmission()
        {
            base.FinishTransmission();
            multipleBlockRead = false;
            writeMode = WriteMode.None;
            writeBytesRemaining = 0;
            writeResponsePending = false;
            commandBytesRemaining = 0;
            readPayloadBytesRemaining = 0;
        }

        private bool TrackWriteByte(byte data)
        {
            if(writeMode == WriteMode.None)
            {
                return false;
            }
            if(writeBytesRemaining > 0)
            {
                writeBytesRemaining--;
                if(writeBytesRemaining == 0)
                {
                    writeResponsePending = true;
                }
                return true;
            }
            if(writeResponsePending)
            {
                writeResponsePending = false;
                if(writeMode == WriteMode.Single)
                {
                    writeMode = WriteMode.None;
                }
                return true;
            }
            if(data == SingleWriteToken || data == MultipleWriteToken)
            {
                writeBytesRemaining = BlockSize + CrcSize;
            }
            else if(writeMode == WriteMode.Multiple && data == StopWriteToken)
            {
                writeMode = WriteMode.None;
            }
            return true;
        }

        private void TrackCommandByte(byte data)
        {
            if(commandBytesRemaining == 0)
            {
                if((data & CommandMask) == CommandPrefix)
                {
                    pendingCommand = (byte)(data & CommandNumberMask);
                    commandBytesRemaining = CommandTailSize;
                }
                return;
            }

            commandBytesRemaining--;
            if(commandBytesRemaining != 0)
            {
                return;
            }
            if(pendingCommand == ReadMultipleBlockCommand)
            {
                multipleBlockRead = true;
            }
            else if(pendingCommand == WriteSingleBlockCommand)
            {
                writeMode = WriteMode.Single;
            }
            else if(pendingCommand == WriteMultipleBlockCommand)
            {
                writeMode = WriteMode.Multiple;
            }
            else if(pendingCommand == StopTransmissionCommandNumber)
            {
                multipleBlockRead = false;
            }
        }

        private enum WriteMode
        {
            None,
            Single,
            Multiple,
        }

        private bool multipleBlockRead;
        private byte pendingCommand;
        private int commandBytesRemaining;
        private int readPayloadBytesRemaining;
        private WriteMode writeMode;
        private int writeBytesRemaining;
        private bool writeResponsePending;

        private const byte CommandMask = 0xC0;
        private const byte CommandPrefix = 0x40;
        private const byte CommandNumberMask = 0x3F;
        private const byte ReadMultipleBlockCommand = 18;
        private const byte WriteSingleBlockCommand = 24;
        private const byte WriteMultipleBlockCommand = 25;
        private const byte StopTransmissionCommandNumber = 12;
        private const byte StopTransmissionCommand =
            CommandPrefix | StopTransmissionCommandNumber;
        private const byte DummyByte = 0xFF;
        private const byte DataToken = 0xFE;
        private const byte SingleWriteToken = 0xFE;
        private const byte MultipleWriteToken = 0xFC;
        private const byte StopWriteToken = 0xFD;
        private const int CommandTailSize = 5;
        private const int BlockSize = 512;
        private const int CrcSize = 2;
    }

    public static class AP_SPI_SDCardExtensions
    {
        public static void AP_SpiSdCardFromFile(
            this IMachine machine,
            string fileName,
            IRegisterablePeripheral<ISPIPeripheral, NumberRegistrationPoint<int>> attachTo,
            int port,
            long size,
            bool persistent = true,
            string name = null)
        {
            var card = new AP_SPI_SDCard(fileName, size, persistent);
            attachTo.Register(card, new NumberRegistrationPoint<int>(port));
            machine.SetLocalName(card, name ?? "sdCard");
        }
    }
}
