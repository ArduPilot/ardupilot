//
// The RT1176 SD host controller, with a card behind it.
//
// This replaces a stub that reported NO CARD. That stub let the boot get past
// USDHC_SetSdClock(), which spins on PRSSTAT.SDSTB, and past the self-clearing
// resets in SYS_CTRL - both of which hang forever against plain memory - but it
// answered no card, so disk_access_init("SD") failed, the volume never mounted,
// and every log write came back FR_NOT_READY. ArduPilot maps that to EBUSY, so
// an emulated flight showed only "Failed to create log directory /APM/logs :
// EBUSY", repeated for as long as AP_Logger kept retrying.
//
// WIRED AS THE BOARD IS WIRED, which is the point of modelling it at all:
//
//   usdhc1, mr_vmu_rt1176_mimxrt1176_cm7.dts:723
//     no-1-8-v    - 3.3V only, so the driver never runs a voltage switch
//     cd-gpios    - gpio3 pin 31, ACTIVE_LOW. Card detect is a GPIO on this
//                   board, NOT PRSSTAT.CINST and NOT the DAT3 pull: the
//                   devicetree sets neither detect-dat3 nor detect-cd, so
//                   imx_usdhc_get_card_present() takes the detect_gpio branch
//                   (drivers/sdhc/imx_usdhc.c:968) and reads the pin. A model
//                   that only sets CINST would still be told there is no card.
//     pwr-gpios   - gpio1 pin 1, driven by the host; nothing to model.
//
// The transfer path is ADMA2, not PIO, because the build sets
// CONFIG_IMX_USDHC_DMA_SUPPORT=y and CONFIG_SDHC_SUPPORTS_SCATTER_GATHER_TRANSFER=y.
// Modelling the FIFO instead would have been much less work and would have
// meant the emulated board never ran the path the real one runs.
//
using System;
using System.Collections.Generic;
using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.SD;

namespace Antmicro.Renode.Peripherals.SD
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_IMXRT_USDHC : NullRegistrationPointPeripheralContainer<SDCard>,
                                  IDoubleWordPeripheral, IKnownSize
    {
        public AP_IMXRT_USDHC(IMachine machine) : base(machine)
        {
            this.machine = machine;
            registers = new Dictionary<long, uint>();
            IRQ = new Antmicro.Renode.Core.GPIO();
            Reset();
        }

        public Antmicro.Renode.Core.GPIO IRQ { get; private set; }

        public long Size => 0x4000;

        public override void Reset()
        {
            registers.Clear();
            // Reset values that the driver reads before writing. HOST_CTRL_CAP
            // advertises 3.3V support and a 4 KB block cap; the driver refuses
            // to come up if the host claims no voltage it can use.
            registers[HostCtrlCap] = VoltageSupport33 | MaxBlockLength512;
            intStatus = 0;
            UpdateInterrupt();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case PresState:
                return PresentState;
            case IntStatus:
                return intStatus;
            default:
                uint value;
                return registers.TryGetValue(offset, out value) ? value : 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case SysCtrl:
                // RSTA/RSTC/RSTD and INITA are self-clearing on silicon and
                // complete well inside one guest instruction here. Written
                // into plain storage they would stay set and the driver's
                // wait-for-clear would never end.
                registers[offset] = value & ~(Rsta | Rstc | Rstd | Inita);
                return;

            case IntStatus:
                // write-1-to-clear
                intStatus &= ~value;
                UpdateInterrupt();
                return;

            case IntStatusEn:
            case IntSignalEn:
                registers[offset] = value;
                UpdateInterrupt();
                return;

            case CmdXfrTyp:
                registers[offset] = value;
                ExecuteCommand(value);
                return;

            default:
                registers[offset] = value;
                return;
            }
        }

        // PRSSTAT. SDSTB says the clock has settled - the driver spins on it.
        // CINST is set when a card is registered, for completeness and for any
        // board that does use the controller's own detect; this board reads a
        // GPIO instead, so CINST alone would not be enough (see the header).
        private uint PresentState
        {
            get
            {
                // Idle bus: the CMD line and all four DAT lines are pulled HIGH.
                // This is not decoration. After every data transfer the Zephyr
                // SD stack calls imx_usdhc_card_busy(), which reads DLSL and
                // reports the card busy while DAT0..3 are low - a real card
                // holds DAT0 low only while it is programming. With these
                // bits clear the driver spun on USDHC_GetPresentStatusFlags()
                // forever after the first CMD17: 4000 consecutive reads of
                // this register in the trace, boot never reached a heartbeat.
                var value = Sdstb | CmdLineLevel | DataLinesHigh;
                if(RegisteredPeripheral != null)
                {
                    value |= Cinst | Cdpl;
                }
                return value;
            }
        }

        private void ExecuteCommand(uint transferType)
        {
            var card = RegisteredPeripheral;
            if(card == null)
            {
                // No card: time the command out rather than hang the driver,
                // which is what the hardware does with nothing on the bus.
                intStatus |= Ctoe;
                UpdateInterrupt();
                return;
            }

            var index = (uint)((transferType >> CmdIndexShift) & 0x3F);
            var responseType = (transferType >> ResponseTypeShift) & 0x3;
            var isData = (transferType & DataPresentSelect) != 0;
            uint argument;
            registers.TryGetValue(CmdArg, out argument);

            var response = card.HandleCommand(index, argument);
            var bytes = response == null ? null : response.AsByteArray();

            // CMD8 SEND_IF_COND: silicon echoes the 8-bit check pattern from the
            // argument back in the low byte of R7, and Zephyr's SD stack refuses
            // the card unless it does ((resp & 0xFF) == 0xAA, subsys/sd/sd.c).
            // Renode's SDCard returns 00 01 00 00 - voltage accepted, pattern
            // dropped - measured with a probe. Reproduce the card's real
            // behaviour here rather than patch every SD stack that meets it.
            if(index == SendIfCondCommand && bytes != null && bytes.Length >= 1)
            {
                bytes[0] = (byte)(argument & 0xFF);
            }
            StoreResponse(bytes, responseType);

            if(isData)
            {
                RunDataTransfer(card);

                // Renode's image-backed SDCard stays in the DATA state after a
                // CMD17 ReadSingleBlock, where a real card returns to TRAN by
                // itself once the one block is out. The Zephyr SD stack then
                // polls CMD13 waiting for TRAN and never sees it: 173,037
                // SEND_STATUS commands in one boot, every one answering
                // state=DATA, mount never completing. Same quirk, same fix as
                // the H7 model (AP_STM32H7_SDMMC.cs ReadCard): tell the card
                // the transfer is over. CMD18 is deliberately excluded - the
                // driver sends its own CMD12 after a multi-block read.
                if(index == ReadSingleBlockCommand)
                {
                    card.HandleCommand(StopTransmissionCommand, 0);
                }
            }

            intStatus |= Cc;
            UpdateInterrupt();
        }

        // What Renode's SDCard actually returns - MEASURED with a probe, because
        // the first version of this inferred it and was wrong twice over:
        //   - AsByteArray() is LITTLE-ENDIAN payload only: CMD55 gives
        //     20 01 00 00 = 0x00000120. There is no command-index byte and no
        //     CRC to strip; packing it big-endian produced 0x20010000 and the
        //     SD stack retried CMD0/CMD8 468 times.
        //   - a short response is 32 bits -> CMD_RSP0.
        //   - a long response (R2, CID/CSD) is 128 bits. The MCUX driver
        //     rebuilds it from CMD_RSP0..3 holding the value SHIFTED RIGHT BY
        //     EIGHT (fsl_usdhc.c USDHC_ReceiveCommandResponse: response[0] <<= 8,
        //     response[1] = (rsp1 << 8) | (rsp0 >> 24), ...), which is the
        //     uSDHC's real layout - the CRC byte is not in the registers.
        private void StoreResponse(byte[] response, uint responseType)
        {
            registers[CmdRsp0] = 0;
            registers[CmdRsp1] = 0;
            registers[CmdRsp2] = 0;
            registers[CmdRsp3] = 0;
            if(response == null || response.Length == 0)
            {
                return;
            }

            if(responseType == ResponseLong && response.Length >= 16)
            {
                // 128-bit little-endian value, then >> 8 into four words
                var value = new System.Numerics.BigInteger(
                    response.Take(16).Concat(new byte[] { 0 }).ToArray());   // extra 0: keep it positive
                value >>= 8;
                var mask = (System.Numerics.BigInteger.One << 32) - 1;
                registers[CmdRsp0] = (uint)(value & mask);
                registers[CmdRsp1] = (uint)((value >> 32) & mask);
                registers[CmdRsp2] = (uint)((value >> 64) & mask);
                registers[CmdRsp3] = (uint)((value >> 96) & mask);
                return;
            }

            registers[CmdRsp0] = ToWordLittleEndian(response, 0);
        }

        private static uint ToWordLittleEndian(byte[] bytes, int offset)
        {
            uint value = 0;
            for(var i = 3; i >= 0; i--)
            {
                value <<= 8;
                if(offset + i < bytes.Length)
                {
                    value |= bytes[offset + i];
                }
            }
            return value;
        }

        // ADMA2 scatter-gather. The descriptor table is what the driver built
        // in its __nocache DMA buffer; walk it and move each chunk between the
        // card and system memory. MIX_CTRL.DTDSEL picks the direction.
        private void RunDataTransfer(SDCard card)
        {
            uint mixCtrl;
            registers.TryGetValue(MixCtrl, out mixCtrl);
            var readFromCard = (mixCtrl & DataTransferDirectionSelect) != 0;

            uint blockAttributes;
            registers.TryGetValue(BlkAtt, out blockAttributes);
            var blockSize = blockAttributes & 0x1FFF;
            var blockCount = (blockAttributes >> 16) & 0xFFFF;
            if(blockCount == 0)
            {
                blockCount = 1;
            }
            var remaining = blockSize * blockCount;

            if((mixCtrl & DmaEnable) == 0)
            {
                this.Log(LogLevel.Warning,
                    "data transfer with DMA disabled is not modelled; this board's "
                    + "driver is built with CONFIG_IMX_USDHC_DMA_SUPPORT=y");
                intStatus |= Dtoe;
                return;
            }

            uint descriptorAddress;
            registers.TryGetValue(AdmaSysAddr, out descriptorAddress);
            var bus = machine.GetSystemBus(this);

            // A bounded walk: a malformed table must not spin the emulator.
            for(var step = 0; step < MaxDescriptors && remaining > 0; step++)
            {
                var word0 = bus.ReadDoubleWord(descriptorAddress);
                var word1 = bus.ReadDoubleWord(descriptorAddress + 4);
                var attribute = word0 & 0xFFFF;
                var length = (word0 >> 16) & 0xFFFF;

                if((attribute & AdmaValid) == 0)
                {
                    break;
                }

                var action = (attribute >> AdmaActionShift) & 0x3;
                if(action == AdmaActionLink)
                {
                    descriptorAddress = word1;
                    continue;
                }
                if(action != AdmaActionTransfer)
                {
                    // NOP and reserved: step over it.
                    descriptorAddress += DescriptorSize;
                    if((attribute & AdmaEnd) != 0)
                    {
                        break;
                    }
                    continue;
                }

                if(length == 0)
                {
                    length = 65536;   // ADMA2 encodes a full 64 KB chunk as zero
                }
                var chunk = Math.Min(length, remaining);
                TransferChunk(card, bus, word1, chunk, readFromCard);
                remaining -= chunk;

                descriptorAddress += DescriptorSize;
                if((attribute & AdmaEnd) != 0)
                {
                    break;
                }
            }

            if(remaining > 0)
            {
                this.Log(LogLevel.Warning,
                    "ADMA2 table ended with {0} bytes of the transfer unmoved", remaining);
            }
            intStatus |= Tc;
        }

        private void TransferChunk(SDCard card, IBusController bus, uint address, uint length,
                                   bool readFromCard)
        {
            if(readFromCard)
            {
                var data = card.ReadData(length);
                bus.WriteBytes(data, address);
            }
            else
            {
                var data = bus.ReadBytes(address, (int)length);
                card.WriteData(data);
            }
        }

        private void UpdateInterrupt()
        {
            uint statusEnable, signalEnable;
            registers.TryGetValue(IntStatusEn, out statusEnable);
            registers.TryGetValue(IntSignalEn, out signalEnable);
            IRQ.Set((intStatus & statusEnable & signalEnable) != 0);
        }

        // Register offsets, i.MX RT1170 RM chapter "uSDHC". Note these are NOT
        // SDHCI's: uSDHC re-lays the same functions out as 32-bit registers,
        // which is why Renode's own SD.SDHCI model cannot be reused here.
        private const long DsAddr = 0x00;
        private const long BlkAtt = 0x04;
        private const long CmdArg = 0x08;
        private const long CmdXfrTyp = 0x0C;
        private const long CmdRsp0 = 0x10;
        private const long CmdRsp1 = 0x14;
        private const long CmdRsp2 = 0x18;
        private const long CmdRsp3 = 0x1C;
        private const long PresState = 0x24;
        private const long SysCtrl = 0x2C;
        private const long IntStatus = 0x30;
        private const long IntStatusEn = 0x34;
        private const long IntSignalEn = 0x38;
        private const long HostCtrlCap = 0x40;
        private const long MixCtrl = 0x48;
        private const long AdmaSysAddr = 0x58;

        private const uint Sdstb = 1u << 3;      // PRSSTAT.SDSTB, clock stable
        private const uint Cinst = 1u << 16;     // PRSSTAT.CINST, card inserted
        private const uint Cdpl = 1u << 18;      // PRSSTAT.CDPL, card detect level
        private const uint CmdLineLevel = 1u << 23;      // PRSSTAT.CLSL, CMD line high
        private const uint DataLinesHigh = 0xFu << 24;   // PRSSTAT.DLSL[3:0], DAT0..3 high

        private const uint Rsta = 1u << 24;
        private const uint Rstc = 1u << 25;
        private const uint Rstd = 1u << 26;
        private const uint Inita = 1u << 27;

        private const uint Cc = 1u << 0;         // INT_STATUS.CC, command complete
        private const uint Tc = 1u << 1;         // INT_STATUS.TC, transfer complete
        private const uint Ctoe = 1u << 16;      // INT_STATUS.CTOE, command timeout
        private const uint Dtoe = 1u << 20;      // INT_STATUS.DTOE, data timeout

        private const uint VoltageSupport33 = 1u << 24;
        private const uint MaxBlockLength512 = 0u << 16;

        private const int CmdIndexShift = 24;
        private const int ResponseTypeShift = 16;
        private const uint DataPresentSelect = 1u << 21;
        private const uint ResponseLong = 1;
        private const uint SendIfCondCommand = 8;
        private const uint StopTransmissionCommand = 12;
        private const uint ReadSingleBlockCommand = 17;

        private const uint DmaEnable = 1u << 0;                     // MIX_CTRL.DMAEN
        private const uint DataTransferDirectionSelect = 1u << 4;   // MIX_CTRL.DTDSEL, 1 = read

        private const uint AdmaValid = 1u << 0;
        private const uint AdmaEnd = 1u << 1;
        private const int AdmaActionShift = 4;
        private const uint AdmaActionTransfer = 2;
        private const uint AdmaActionLink = 3;
        private const uint DescriptorSize = 8;
        private const int MaxDescriptors = 4096;

        private uint intStatus;
        private readonly IMachine machine;
        private readonly Dictionary<long, uint> registers;
    }
}
