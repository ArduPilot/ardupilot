//
// STM32H7 SDMMC with a working internal DMA (IDMA).
//
// Renode's STM32HSDMMC declares the IDMA registers but leaves them as
// tags, so a data transfer set up through IDMA never moves a byte and
// never raises DATAEND. ChibiOS's H7 SD driver uses IDMA exclusively -
// it does not touch the FIFO - so the mount blocks forever and, on
// ArduPilot, the vehicle never gets past sdcard_init(). (With no card
// attached at all the driver hangs even earlier: sdc_lld_send_cmd_none
// spins on CMDSENT, which the model only raises when a card answers.)
//
// The base model's FIFO paths already do everything needed per word,
// including raising DATAEND when the last word moves, so IDMA here is
// a pump between those paths and memory.
//
// The base model defers command data handling to the next synchronized
// state. Queue the IDMA pump at the same boundary, after both halves of
// the transfer have been armed. ChibiOS normally writes DCTRL after the
// command for block I/O, but writes it before the command for short
// card-register reads, so either write can complete the setup.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.SD;

namespace Antmicro.Renode.Peripherals.SD
{
    public class AP_STM32H7_SDMMC : STM32HSDMMC
    {
        public AP_STM32H7_SDMMC(IMachine machine) : base(machine)
        {
            this.machine = machine;

            this.AddAfterWriteHook<uint, DoubleWordRegisterCollection>(IDMACtrl,
                (offset, value) => idmaEnabled = (value & 1) != 0);
            this.AddAfterWriteHook<uint, DoubleWordRegisterCollection>(IDMABase0,
                (offset, value) => idmaBase = value);
            this.AddAfterWriteHook<uint, DoubleWordRegisterCollection>(DataCtrl,
                (offset, value) =>
                {
                    dataTransferEnabled = (value & DataTransferEnable) != 0;
                    QueueIdmaPump();
                });
            this.AddAfterWriteHook<uint, DoubleWordRegisterCollection>(Cmd,
                (offset, value) =>
                {
                    currentCommand = value & CommandIndexMask;
                    QueueIdmaPump();
                });
        }

        protected override void ReadCard(SDCard sdCard, uint size)
        {
            base.ReadCard(sdCard, size);

            // Renode's image-backed SDCard keeps CMD17 in the DATA state
            // after ReadData(), unlike its finite register-read path. A
            // real card implicitly returns to TRAN after the single block.
            // CMD18 is deliberately excluded: its CMD12 is sent by the
            // ChibiOS driver after the complete multi-block transaction.
            if(currentCommand == ReadSingleBlockCommand)
            {
                sdCard.HandleCommand(StopTransmissionCommand, 0);
            }
        }

        private void QueueIdmaPump()
        {
            if(!idmaEnabled || !dataTransferEnabled || idmaBase == 0)
            {
                return;
            }
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ => PumpIdma());
        }

        private void PumpIdma()
        {
            if(!idmaEnabled || !dataTransferEnabled || idmaBase == 0)
            {
                return;
            }
            var address = (ulong)idmaBase;
            while(ReadDataBuffer.Count >= 4)
            {
                machine.GetSystemBus(this).WriteDoubleWord(address, ReadBuffer());
                address += 4;
            }
            while(WriteDataLeft >= 4)
            {
                WriteBuffer(machine.GetSystemBus(this).ReadDoubleWord(address));
                address += 4;
            }
        }

        private readonly IMachine machine;
        private bool idmaEnabled;
        private bool dataTransferEnabled;
        private uint idmaBase;
        private uint currentCommand;

        private const long Cmd = 0x0C;
        private const long DataCtrl = 0x2C;
        private const long IDMACtrl = 0x50;
        private const long IDMABase0 = 0x58;
        private const uint CommandIndexMask = 0x3Fu;
        private const uint DataTransferEnable = 1u << 0;
        private const uint ReadSingleBlockCommand = 17;
        private const uint StopTransmissionCommand = 12;
    }
}
