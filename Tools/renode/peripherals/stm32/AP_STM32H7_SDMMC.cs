//
// STM32H7 SDMMC with a working internal DMA (IDMA), for both guest drivers.
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
// TWO WAYS TO ARM A TRANSFER, not one. The pump used to key only off
// DCTRL[DTEN], which is what ChibiOS sets (hal_sdc_lld.c:165-168, :761-765,
// :825-828). Zephyr goes through the ST HAL instead, and the ST HAL leaves
// DTEN CLEAR: HAL_SD_ReadBlocks_DMA passes config.DPSM = SDMMC_DPSM_DISABLE
// to SDMMC_ConfigData (stm32h7xx_hal_sd.c:1284) and then starts the data path
// with __SDMMC_CMDTRANS_ENABLE, CMD bit 6 (stm32h7xx_ll_sdmmc.h:1124).
// The observed DCTRL write for a 512-byte block read is 0x92 - block size 9,
// direction from card, DTEN clear. So CMD[CMDTRANS] arms the transfer here
// as well as DCTRL[DTEN]. ChibiOS never writes CMDTRANS: its driver's CMD
// writes are hal_sdc_lld.c:560 (CPSMEN alone), :586, :619 and :654 (CPSMEN
// plus WAITRESP). The only other CMD writer in a ChibiOS-HAL firmware is the
// crash-dump SD path, AP_HAL_ChibiOS/CrashDump_SD.cpp:845 and :869 - compiled
// only with AP_CRASHDUMP_FATFS_ENABLED=1 (default 0, unset for CubeOrange) and
// reached only from the fault handler - and it sets no bit 6 either. So the
// extra arming condition cannot change what ChibiOS sees, and on ChibiOS
// TransferArmed reduces exactly to the old dataTransferEnabled gate.
//
// Without this, the first FAT boot-sector read never completes and the guest
// never comes back: Zephyr's stm32_sdmmc_access_read waits on a semaphore with
// K_FOREVER (sdmmc_stm32.c:583) that only HAL_SD_RxCpltCallback gives, and that
// callback is reached only from HAL_SD_IRQHandler's DATAEND branch. The board
// parks in the idle thread with no output at all, which is how it presented.
//
// DTIMEOUT IS SYNTHESIZED HERE because the base models it as a tag that reads
// permanently false (STM32SDMMC.cs:176), as it does RXOVERR (:178), and pins
// DCRCFAIL to false (:174). A guest that enables exactly those three plus
// DATAEND - which is what both drivers do, ST HAL at stm32h7xx_hal_sd.c:1317
// and ChibiOS at hal_sdc_lld.c:158-161 - has no way out at all if the model
// fails to complete a transfer: no completion and no error. The watchdog below
// turns that into a data timeout, so the driver reports a read failure, the
// mount fails, and the board still boots. It is a safety net, not the fix: it
// only fires on a transfer this model left unfinished, and it checks three
// separate signs of a live transfer first so a slow but healthy one cannot
// trip it.
//
// What the net does and does not cover, read off the code (2026-09-12), so
// nobody relies on it for more than it does. The ST HAL writes CMD twice per
// transfer (CMDTRANS-enable, then the command, ~50 instructions apart) and
// each write queues a RunTransfer through ExecuteInNearestSyncedState, which
// never runs inline: both run after the next sync point, by which time both
// writes have landed and transferGeneration already holds its final value.
// So the check that gets scheduled carries the right generation and a
// transfer that stalls is reported on the FIRST period, ~1000 ms. Only a
// quantum boundary falling between the two writes costs a second period.
// The real limit is later: after the first completed DMA transfer the net
// stops arming on the ST HAL path altogether, because DBCKEND stays set in
// the model and the ST HAL's DMA completion branch never clears it, so
// TransferEnded reads true from then on. It covers the case it was written
// for - the very first FAT read at boot, where the silent hang was - and not
// later ones. ChibiOS block I/O never reaches ScheduleAction at all: it
// writes the command before DTEN, so the pump completes inside the same
// RunTransfer.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.SD;
using Antmicro.Renode.Time;

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
                    ArmTransfer();
                });
            this.AddAfterWriteHook<uint, DoubleWordRegisterCollection>(Cmd,
                (offset, value) =>
                {
                    commandTransfer = (value & CommandTransferEnable) != 0;
                    // Only a write that actually starts the command path carries
                    // the index. The ST HAL writes CMD twice per transfer - once
                    // for __SDMMC_CMDTRANS_ENABLE alone, with CPSMEN clear - and
                    // the low six bits of that first write are not a command.
                    if((value & CommandPathEnable) != 0)
                    {
                        currentCommand = value & CommandIndexMask;
                    }
                    ArmTransfer();
                });

            // DTIMEOUT, DTIMEOUTC and DTIMEOUTIE are all tags or no-ops in the
            // base model, so the flag is added on top of its register values
            // rather than inside them: redefining IntStatus would orphan the
            // nine status fields the base keeps private references to.
            this.AddAfterReadHook<uint, DoubleWordRegisterCollection>(IntStatus,
                (offset, value) => dataTimeout ? (uint?)(value | DataTimeoutFlag) : null);
            this.AddAfterWriteHook<uint, DoubleWordRegisterCollection>(IntClear,
                (offset, value) =>
                {
                    if((value & DataTimeoutFlag) != 0)
                    {
                        dataTimeout = false;
                    }
                    // Both drivers clear DATAEND as the first thing they do once
                    // a transfer has finished - ST HAL at stm32h7xx_hal_sd.c:1550,
                    // ChibiOS with ICR = all flags at hal_sdc_lld.c:286 - so this
                    // is the guest saying the transfer it was waiting on is done.
                    if((value & DataEndFlag) != 0)
                    {
                        completionSeen = true;
                    }
                    RefreshTimeoutIrq();
                });
            this.AddAfterWriteHook<uint, DoubleWordRegisterCollection>(IntMask,
                (offset, value) =>
                {
                    dataTimeoutEnabled = (value & DataTimeoutFlag) != 0;
                    RefreshTimeoutIrq();
                });
        }

        public override void Reset()
        {
            base.Reset();
            idmaEnabled = false;
            idmaBase = 0;
            dataTransferEnabled = false;
            commandTransfer = false;
            currentCommand = 0;
            dataTimeout = false;
            dataTimeoutEnabled = false;
            completionSeen = false;
            timeoutScheduled = false;
            transferGeneration = 0;
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

        private bool TransferArmed => dataTransferEnabled || commandTransfer;

        // True once the model has finished a data phase. DATAEND and DBCKEND
        // are sticky until the guest clears them through IntClear, which is
        // also what sets completionSeen, so between them the two cover both
        // "finished, not yet acknowledged" and "finished and acknowledged".
        private bool TransferEnded =>
            (RegistersCollection.Read(IntStatus) & (DataEndFlag | DataBlockEndFlag)) != 0;

        private void ArmTransfer()
        {
            if(!TransferArmed)
            {
                return;
            }
            dataTimeout = false;
            completionSeen = false;
            transferGeneration++;
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ => RunTransfer());
        }

        private void RunTransfer()
        {
            PumpIdma();
            ScheduleTimeoutIfUnfinished();
        }

        private void PumpIdma()
        {
            if(!idmaEnabled || !TransferArmed || idmaBase == 0)
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

        // Deliberately NOT armed on every transfer. machine.ScheduleAction
        // synchronizes the CPU's time and forces it back out of translated code
        // on each call (ExecuteInNearestSyncedState, used by ArmTransfer, only
        // enqueues and does not), so putting one on every block would tax a
        // ChibiOS flight that does tens of thousands of them. A transfer this model can carry
        // is already finished by the time RunTransfer returns - the pump moved
        // it, or the guest is draining the FIFO itself - so nothing is scheduled
        // in the normal case, and the cost falls only on a transfer that stalled.
        // That holds on the ST HAL path too, where two RunTransfer calls are
        // queued per transfer: both run after the same sync point, the first
        // moves the data and the second finds it ended.
        private void ScheduleTimeoutIfUnfinished()
        {
            if(timeoutScheduled || !TransferArmed || completionSeen || TransferEnded)
            {
                return;
            }
            timeoutScheduled = true;
            var generation = transferGeneration;
            machine.ScheduleAction(TimeInterval.FromMilliseconds(DataTimeoutMilliseconds),
                _ => ReportTimeoutIfStalled(generation));
        }

        private void ReportTimeoutIfStalled(ulong generation)
        {
            timeoutScheduled = false;

            // Each of these says the transfer is not stuck: the guest tore the
            // data path down (ST HAL writes DCTRL = 0 at stm32h7xx_hal_sd.c:1600,
            // ChibiOS at hal_sdc_lld.c:278), the guest acknowledged completion,
            // or the model raised DATAEND and the guest has not read it yet.
            if(!TransferArmed || completionSeen || TransferEnded)
            {
                return;
            }
            if(generation != transferGeneration)
            {
                // A later transfer was armed while this check was pending. Give
                // that one a full timeout of its own rather than failing it early.
                ScheduleTimeoutIfUnfinished();
                return;
            }

            this.WarningLog("No data transfer completion {0} ms after the data path was armed; " +
                            "raising DTIMEOUT so the guest driver fails the transfer instead of waiting forever",
                            DataTimeoutMilliseconds);
            dataTimeout = true;
            RefreshTimeoutIrq();
        }

        // The base recomputes IRQ from its own flags on every IntClear and
        // IntMask write (STM32SDMMC.cs:234, :267), which are the only two
        // registers a guest uses to change interrupt state, so the line only
        // has to be driven back up here when the synthesized flag is pending.
        private void RefreshTimeoutIrq()
        {
            if(dataTimeout && dataTimeoutEnabled)
            {
                IRQ.Set(true);
            }
        }

        private readonly IMachine machine;
        private bool idmaEnabled;
        private bool dataTransferEnabled;
        private bool commandTransfer;
        private uint idmaBase;
        private uint currentCommand;
        private bool dataTimeout;
        private bool dataTimeoutEnabled;
        private bool completionSeen;
        private bool timeoutScheduled;
        private ulong transferGeneration;

        private const long Cmd = 0x0C;
        private const long DataCtrl = 0x2C;
        private const long IntStatus = 0x34;
        private const long IntClear = 0x38;
        private const long IntMask = 0x3C;
        private const long IDMACtrl = 0x50;
        private const long IDMABase0 = 0x58;
        private const uint CommandIndexMask = 0x3Fu;
        private const uint CommandTransferEnable = 1u << 6;
        // CPSMEN sits above the H7's 8-bit command field offset.
        private const uint CommandPathEnable = 1u << 12;
        private const uint DataTransferEnable = 1u << 0;
        private const uint DataTimeoutFlag = 1u << 3;
        private const uint DataEndFlag = 1u << 8;
        private const uint DataBlockEndFlag = 1u << 10;
        private const uint ReadSingleBlockCommand = 17;
        private const uint StopTransmissionCommand = 12;
        private const ulong DataTimeoutMilliseconds = 1000;
    }
}
