//
// RP2350 QMI with a SPI NOR flash behind its direct mode.
//
// ArduPilot's parameter storage erases and programs the boot flash itself
// (modules/ChibiOS os/hal/ports/RP/RP2350/hal_efl_lld.c): it asserts CS0 with
// DIRECT_CSR.ASSERT_CS0N, shifts serial NOR commands through DIRECT_TX and
// reads the replies from DIRECT_RX. This model decodes those commands against
// the flash memory region, so erase and program land where XIP reads them.
// Operations complete at once, so the status register never reads busy.
//
// The memory-mapped XIP window itself is the platform's flash MappedMemory;
// the M0/M1 timing and format registers are plain storage.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Memory;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_RP2350_QMI : AP_RP2350_Block
    {
        public AP_RP2350_QMI(IMachine machine, MappedMemory flash) : base(machine)
        {
            this.flash = flash;
        }

        // Lay out ArduPilot parameter storage the way a formatted board has
        // it: every sector erased, each starting with an AP_FlashStorage
        // header in the "available" state. AP_FlashStorage::init() then finds
        // valid empty sectors and starts without erasing anything.
        //   offset:     flash offset of the first sector
        //              (STORAGE_FLASH_PAGE * 4 KB)
        //   sectorSize: bytes per sector (4 KB pages per sector * 4 KB)
        //   header:     the 32-bit header word, e.g. 0x51685BFF for the
        //              F4-style header RP2350 uses (state 0xFF, signature
        //              0x51685B)
        public void FormatParameterStorage(uint offset, uint sectorSize, uint header)
        {
            var blank = new byte[sectorSize];
            for(var i = 0; i < blank.Length; i++)
            {
                blank[i] = 0xFF;
            }
            var headerBytes = System.BitConverter.GetBytes(header);
            for(var sector = 0u; sector < 2; sector++)
            {
                var start = offset + sector * sectorSize;
                flash.WriteBytes(Wrap(start), blank, 0, blank.Length);
                flash.WriteBytes(Wrap(start), headerBytes, 0, headerBytes.Length);
            }
        }

        public override void Reset()
        {
            base.Reset();
            rx.Clear();
            session.Clear();
            csAsserted = false;
            writeEnabled = false;
        }

        protected override uint Read(long reg)
        {
            switch(reg)
            {
                case DirectCsr:
                    var csr = (base.Read(reg) & ~(CsrBusy | CsrTxFull | CsrRxEmpty | CsrRxFull)) | CsrTxEmpty;
                    if(rx.Count == 0)
                    {
                        csr |= CsrRxEmpty;
                    }
                    if(rx.Count >= RxDepth)
                    {
                        csr |= CsrRxFull;
                    }
                    return csr;
                case DirectRx:
                    return rx.Count > 0 ? rx.Dequeue() : 0u;
                default:
                    return base.Read(reg);
            }
        }

        protected override void Write(long reg, uint value)
        {
            switch(reg)
            {
                case DirectCsr:
                    base.Write(reg, value);
                    var asserted = (value & CsrAssertCs0n) != 0;
                    if(asserted && !csAsserted)
                    {
                        session.Clear();
                    }
                    else if(!asserted && csAsserted)
                    {
                        EndSession();
                    }
                    csAsserted = asserted;
                    break;
                case DirectTx:
                    var reply = csAsserted ? Shift((byte)value) : (byte)0xFF;
                    if((value & TxNoPush) == 0 && rx.Count < RxDepth)
                    {
                        rx.Enqueue(reply);
                    }
                    break;
                default:
                    base.Write(reg, value);
                    break;
            }
        }

        // One byte in on the command bus, one byte out.
        private byte Shift(byte data)
        {
            session.Add(data);
            var index = session.Count - 1;
            var cmd = session[0];
            if(index == 0)
            {
                return 0xFF;
            }
            switch(cmd)
            {
                case CmdReadStatus:
                    return (byte)(writeEnabled ? StatusWel : 0);
                case CmdReadStatus2:
                    return 0x02; // QE set: quad mode stays enabled
                case CmdJedecId:
                    return index <= 3 ? JedecId[index - 1] : (byte)0;
                case CmdUniqueId:
                    // four dummy bytes, then the 64-bit ID
                    return index >= 5 && index <= 12 ? UniqueId[index - 5] : (byte)0;
                case CmdRead:
                    if(index >= 4)
                    {
                        return flash.ReadByte(Wrap(Address + (uint)(index - 4)));
                    }
                    return 0xFF;
                case CmdPageProgram:
                    if(index >= 4 && writeEnabled)
                    {
                        // programming can only clear bits; the address wraps
                        // within the 256-byte page
                        var pageBase = Address & ~0xFFu;
                        var offset = (Address + (uint)(index - 4)) & 0xFFu;
                        var at = Wrap(pageBase + offset);
                        flash.WriteByte(at, (byte)(flash.ReadByte(at) & data));
                    }
                    return 0xFF;
                default:
                    return 0xFF;
            }
        }

        private void EndSession()
        {
            if(session.Count == 0)
            {
                return;
            }
            this.Log(LogLevel.Debug, "cmd 0x{0:X2} bytes={1} addr=0x{2:X6} wel={3}",
                session[0], session.Count, session.Count >= 4 ? Address : 0, writeEnabled);
            switch(session[0])
            {
                case CmdWriteEnable:
                    writeEnabled = true;
                    break;
                case CmdWriteDisable:
                    writeEnabled = false;
                    break;
                case CmdPageProgram:
                    writeEnabled = false;
                    break;
                case CmdSectorErase:
                    Erase(0x1000);
                    break;
                case CmdBlockErase32:
                    Erase(0x8000);
                    break;
                case CmdBlockErase64:
                    Erase(0x10000);
                    break;
            }
            session.Clear();
        }

        private void Erase(uint size)
        {
            if(session.Count < 4)
            {
                return;
            }
            if(!writeEnabled)
            {
                this.Log(LogLevel.Warning, "erase at 0x{0:X6} without write enable ignored", Address);
                return;
            }
            var start = Address & ~(size - 1);
            var blank = new byte[size];
            for(var i = 0; i < blank.Length; i++)
            {
                blank[i] = 0xFF;
            }
            flash.WriteBytes(Wrap(start), blank, 0, (int)size);
            writeEnabled = false;
        }

        private uint Address => ((uint)session[1] << 16) | ((uint)session[2] << 8) | session[3];

        private long Wrap(uint address)
        {
            return address % (uint)flash.Size;
        }

        private readonly MappedMemory flash;
        private readonly Queue<uint> rx = new Queue<uint>();
        private readonly List<byte> session = new List<byte>();
        private bool csAsserted;
        private bool writeEnabled;

        private const int RxDepth = 4;
        private const long DirectCsr = 0x0;
        private const long DirectTx = 0x4;
        private const long DirectRx = 0x8;

        private const uint CsrBusy = 1u << 1;
        private const uint CsrAssertCs0n = 1u << 2;
        private const uint CsrTxFull = 1u << 10;
        private const uint CsrTxEmpty = 1u << 11;
        private const uint CsrRxEmpty = 1u << 16;
        private const uint CsrRxFull = 1u << 17;
        private const uint TxNoPush = 1u << 20;

        private const byte CmdWriteEnable = 0x06;
        private const byte CmdWriteDisable = 0x04;
        private const byte CmdReadStatus = 0x05;
        private const byte CmdReadStatus2 = 0x35;
        private const byte CmdJedecId = 0x9F;
        private const byte CmdUniqueId = 0x4B;
        private const byte CmdRead = 0x03;
        private const byte CmdPageProgram = 0x02;
        private const byte CmdSectorErase = 0x20;
        private const byte CmdBlockErase32 = 0x52;
        private const byte CmdBlockErase64 = 0xD8;
        private const byte StatusWel = 0x02;

        // Winbond W25Q32 (4 MB), as fitted to RPI_UAVFC
        private static readonly byte[] JedecId = { 0xEF, 0x40, 0x16 };
        private static readonly byte[] UniqueId = { 0x52, 0x50, 0x32, 0x33, 0x35, 0x30, 0x00, 0x01 };
    }
}
