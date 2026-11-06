//
// RP2350 I2C0/I2C1: Synopsys DesignWare APB I2C in master mode.
//
// Each IC_DATA_CMD entry is carried out at once against the device at
// IC_TAR: writes are gathered and handed over as one block when the bus turns
// round (a read, a RESTART, a STOP or a new target), each read command reads
// one byte into the receive FIFO, and STOP ends the transaction. A target
// nobody answers aborts with ABRT_7B_ADDR_NOACK, which is how ArduPilot's
// probes find an empty address.
//
// The transmit FIFO therefore never fills. Timing registers are storage only.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_RP2350_I2C : SimpleContainer<II2CPeripheral>, IDoubleWordPeripheral, IKnownSize
    {
        public AP_RP2350_I2C(IMachine machine) : base(machine)
        {
            IRQ = new GPIO();
            Reset();
        }

        public GPIO IRQ { get; }

        public long Size => 0x4000;

        public override void Reset()
        {
            regs.Clear();
            rx.Clear();
            pendingWrite.Clear();
            raw = 0;
            mask = 0x8FF;
            abortSource = 0;
            enabled = false;
            inTransaction = false;
            regs[IcCon] = 0x65;
            regs[IcTar] = 0x55;
            Update();
        }

        public uint ReadDoubleWord(long offset)
        {
            var reg = offset & 0xFFC;
            switch(reg)
            {
                case IcDataCmd:
                    if(rx.Count == 0)
                    {
                        raw |= RawRxUnder;
                        Update();
                        return 0;
                    }
                    var b = rx.Dequeue();
                    Update();
                    return b;
                case IcIntrStat: return raw & mask;
                case IcIntrMask: return mask;
                case IcRawIntrStat: return raw;
                case IcClrIntr:
                    raw &= ~(RawRxUnder | RawRxOver | RawTxOver | RawTxAbrt | RawStopDet | RawStartDet | RawActivity | RawRestartDet);
                    abortSource = 0;
                    Update();
                    return 0;
                case IcClrRxUnder: return Clear(RawRxUnder);
                case IcClrRxOver: return Clear(RawRxOver);
                case IcClrTxOver: return Clear(RawTxOver);
                case IcClrTxAbrt:
                    abortSource = 0;
                    return Clear(RawTxAbrt);
                case IcClrActivity: return Clear(RawActivity);
                case IcClrStopDet: return Clear(RawStopDet);
                case IcClrStartDet: return Clear(RawStartDet);
                case IcClrRestartDet: return Clear(RawRestartDet);
                case IcEnable: return enabled ? 1u : 0u;
                case IcStatus:
                    var status = StatusTfnf | StatusTfe;
                    if(rx.Count > 0)
                    {
                        status |= StatusRfne;
                    }
                    if(rx.Count >= FifoDepth)
                    {
                        status |= StatusRff;
                    }
                    return status;
                case IcTxflr: return 0;
                case IcRxflr: return (uint)rx.Count;
                case IcTxAbrtSource: return abortSource;
                case IcEnableStatus: return enabled ? 1u : 0u;
                case IcCompParam1: return 0x000F0F00 | (2 << 2) | 2; // 16-deep FIFOs, fast mode, 32-bit APB
                case IcCompVersion: return 0x3230312A;
                case IcCompType: return 0x44570140;
                default:
                    return regs.TryGetValue(reg, out var v) ? v : 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var reg = offset & 0xFFC;
            var alias = offset >> 12;
            if(alias != 0 && reg != IcDataCmd)
            {
                var current = ReadRegisterForAlias(reg);
                switch(alias)
                {
                    case 1: value = current ^ value; break;
                    case 2: value = current | value; break;
                    case 3: value = current & ~value; break;
                }
            }
            switch(reg)
            {
                case IcDataCmd:
                    if(alias == 3)
                    {
                        return;
                    }
                    DataCmd(value);
                    break;
                case IcIntrMask:
                    mask = value & 0x1FFF;
                    break;
                case IcEnable:
                    var wasEnabled = enabled;
                    enabled = (value & 1) != 0;
                    if(wasEnabled && !enabled)
                    {
                        EndTransaction(false);
                        rx.Clear();
                    }
                    regs[reg] = value;
                    break;
                case IcTar:
                    if(inTransaction)
                    {
                        EndTransaction(false);
                    }
                    regs[reg] = value;
                    break;
                default:
                    regs[reg] = value;
                    break;
            }
            Update();
        }

        private uint ReadRegisterForAlias(long reg)
        {
            switch(reg)
            {
                case IcIntrMask: return mask;
                case IcEnable: return enabled ? 1u : 0u;
                default: return regs.TryGetValue(reg, out var v) ? v : 0;
            }
        }

        private uint Clear(uint bits)
        {
            raw &= ~bits;
            Update();
            return 0;
        }

        private void DataCmd(uint value)
        {
            if(!enabled)
            {
                return;
            }
            var target = (int)((regs.TryGetValue(IcTar, out var t) ? t : 0) & 0x7F);
            // a transaction already aborted discards the rest of its commands
            if(aborted)
            {
                if((value & CmdStop) != 0)
                {
                    aborted = false;
                    raw |= RawStopDet;
                }
                return;
            }
            if(!inTransaction)
            {
                inTransaction = true;
                raw |= RawStartDet | RawActivity;
                if(!TryGetByAddress(target, out _))
                {
                    abortSource = AbrtAddrNoAck;
                    raw |= RawTxAbrt;
                    inTransaction = false;
                    aborted = (value & CmdStop) == 0;
                    if(!aborted)
                    {
                        raw |= RawStopDet;
                    }
                    return;
                }
            }
            TryGetByAddress(target, out var device);
            if((value & CmdRestart) != 0)
            {
                FlushWrite(device);
                raw |= RawRestartDet;
            }
            if((value & CmdRead) != 0)
            {
                FlushWrite(device);
                var data = device.Read(1);
                var b = data != null && data.Length > 0 ? data[0] : (byte)0xFF;
                if(rx.Count >= FifoDepth)
                {
                    raw |= RawRxOver;
                }
                else
                {
                    rx.Enqueue(b);
                }
            }
            else
            {
                pendingWrite.Add((byte)value);
            }
            if((value & CmdStop) != 0)
            {
                EndTransaction(true);
            }
        }

        private void FlushWrite(II2CPeripheral device)
        {
            if(pendingWrite.Count > 0 && device != null)
            {
                device.Write(pendingWrite.ToArray());
            }
            pendingWrite.Clear();
        }

        private void EndTransaction(bool stop)
        {
            if(!inTransaction)
            {
                return;
            }
            var target = (int)((regs.TryGetValue(IcTar, out var t) ? t : 0) & 0x7F);
            if(TryGetByAddress(target, out var device))
            {
                FlushWrite(device);
                device.FinishTransmission();
            }
            pendingWrite.Clear();
            inTransaction = false;
            if(stop)
            {
                raw |= RawStopDet;
            }
        }

        private void Update()
        {
            // the transmit FIFO is always empty and the receive level follows
            // IC_RX_TL
            raw |= RawTxEmpty;
            var rxThreshold = (int)((regs.TryGetValue(IcRxTl, out var tl) ? tl : 0) & 0xFF);
            if(rx.Count > rxThreshold)
            {
                raw |= RawRxFull;
            }
            else
            {
                raw &= ~RawRxFull;
            }
            IRQ.Set((raw & mask) != 0);
        }

        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private readonly Queue<byte> rx = new Queue<byte>();
        private readonly List<byte> pendingWrite = new List<byte>();
        private uint raw;
        private uint mask;
        private uint abortSource;
        private bool enabled;
        private bool inTransaction;
        private bool aborted;

        private const int FifoDepth = 16;

        private const long IcCon = 0x00;
        private const long IcTar = 0x04;
        private const long IcDataCmd = 0x10;
        private const long IcIntrStat = 0x2C;
        private const long IcIntrMask = 0x30;
        private const long IcRawIntrStat = 0x34;
        private const long IcRxTl = 0x38;
        private const long IcClrIntr = 0x40;
        private const long IcClrRxUnder = 0x44;
        private const long IcClrRxOver = 0x48;
        private const long IcClrTxOver = 0x4C;
        private const long IcClrTxAbrt = 0x54;
        private const long IcClrActivity = 0x5C;
        private const long IcClrStopDet = 0x60;
        private const long IcClrStartDet = 0x64;
        private const long IcEnable = 0x6C;
        private const long IcStatus = 0x70;
        private const long IcTxflr = 0x74;
        private const long IcRxflr = 0x78;
        private const long IcTxAbrtSource = 0x80;
        private const long IcEnableStatus = 0x9C;
        private const long IcClrRestartDet = 0xA8;
        private const long IcCompParam1 = 0xF4;
        private const long IcCompVersion = 0xF8;
        private const long IcCompType = 0xFC;

        private const uint CmdRead = 1u << 8;
        private const uint CmdStop = 1u << 9;
        private const uint CmdRestart = 1u << 10;

        private const uint RawRxUnder = 1u << 0;
        private const uint RawRxOver = 1u << 1;
        private const uint RawRxFull = 1u << 2;
        private const uint RawTxOver = 1u << 3;
        private const uint RawTxEmpty = 1u << 4;
        private const uint RawTxAbrt = 1u << 6;
        private const uint RawActivity = 1u << 8;
        private const uint RawStopDet = 1u << 9;
        private const uint RawStartDet = 1u << 10;
        private const uint RawRestartDet = 1u << 12;

        private const uint StatusTfnf = 1u << 1;
        private const uint StatusTfe = 1u << 2;
        private const uint StatusRfne = 1u << 3;
        private const uint StatusRff = 1u << 4;

        private const uint AbrtAddrNoAck = 1u << 0;
    }
}
