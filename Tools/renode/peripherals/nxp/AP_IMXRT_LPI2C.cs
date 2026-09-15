//
// i.MX RT LPI2C master, to the depth ArduPilot's sensor traffic needs.
//
// Renode has no LPI2C model of any kind - the i.MX RT platform file used to say
// so and leave the barometers and compass off the emulated board entirely.
// Without this there is no baro and no compass, the EKF never gets a height or
// a heading, and the board cannot fly.
//
// The LPI2C master is command-driven rather than register-driven: the guest
// writes 16-bit words to MTDR, each a 3-bit command plus a data byte
// (fsl_lpi2c.c:43-50).
//
//     kStartCmd  (4)  START/repeated START, data = (address << 1) | read
//     kTxDataCmd (0)  one byte of payload
//     kRxDataCmd (1)  read (data + 1) bytes into the RX FIFO
//     kStopCmd   (2)  STOP
//
// so the transaction shape is visible without any of the pin-level modelling a
// register-per-bit I2C model would need. Renode's II2CPeripheral is written for
// exactly this shape - Write(bytes) / Read(count) / FinishTransmission() - so
// the mapping is direct: payload bytes accumulate until something ends the
// write phase, then go to the addressed child in one Write(); a read command
// asks the child for that many bytes and fills the RX FIFO.
//
// Everything is synchronous. Silicon would raise TDF/RDF as the FIFOs drain and
// the guest would come back later; here the child answers inside the MTDR
// write, so TDF is always true and RDF is true whenever the RX FIFO has
// anything in it. That is a simplification the guest cannot detect: the MCUX
// driver only ever polls those flags, and a flag that is already set just means
// the poll returns immediately.
//
// DMA is modelled because this board uses it: CONFIG_I2C_MCUX_LPI2C_EDMA=y and
// the DTS gives each LPI2C one request line (lpi2c2 dmas = <&edma0 23 49>), so
// the eDMA copies the command words in and the received bytes out. A single
// GPIO carries both directions, as the silicon does, gated by MDER[TDDE] and
// MDER[RDDE]; the eDMA model (AP_IMXRT_EDMA.cs) treats it as a level and runs
// minor loops while it is asserted.
//
// This file is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This file is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class AP_IMXRT_LPI2C : SimpleContainer<II2CPeripheral>, IDoubleWordPeripheral,
        IWordPeripheral, IBytePeripheral, IKnownSize
    {
        public AP_IMXRT_LPI2C(IMachine machine) : base(machine)
        {
            IRQ = new GPIO();
            DmaRequest = new GPIO();
            writeBuffer = new List<byte>();
            rxFifo = new Queue<byte>();
            Reset();
        }

        public override void Reset()
        {
            writeBuffer.Clear();
            rxFifo.Clear();
            currentSlave = null;
            currentAddress = -1;
            control = 0;
            status = 0;
            interruptEnable = 0;
            dmaEnable = 0;
            config1 = 0;
            fifoControl = 0;
            UpdateSignals();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Verid:
                // Version 2.0, the value the MCUX driver expects to be able to read.
                return 0x02000003;
            case Param:
                // Four-word TX and RX FIFOs, as the RT1170 has: the driver reads
                // this to size its bursts.
                return (4u << 8) | 4u;
            case Mcr:
                return control;
            case Msr:
                return CurrentStatus;
            case Mier:
                return interruptEnable;
            case Mder:
                return dmaEnable;
            case Mcfgr1:
                return config1;
            case Mfcr:
                return fifoControl;
            case Mfsr:
                // TX count is always zero: a command written to MTDR has already
                // been carried out by the time the write returns.
                return ((uint)rxFifo.Count << RxCountShift) & RxCountMask;
            case Mrdr:
                if(rxFifo.Count == 0)
                {
                    return RxEmpty;
                }
                {
                    var value = (uint)rxFifo.Dequeue();
                    UpdateSignals();
                    return value;
                }
            default:
                uint stored;
                return registers.TryGetValue(offset, out stored) ? stored : 0u;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case Mcr:
                control = value & ~(McrRst | McrRtf | McrRrf);
                if((value & McrRst) != 0)
                {
                    Reset();
                    return;
                }
                if((value & McrRtf) != 0)
                {
                    writeBuffer.Clear();
                }
                if((value & McrRrf) != 0)
                {
                    rxFifo.Clear();
                }
                UpdateSignals();
                return;
            case Msr:
                // Write-1-to-clear, and only on the bits silicon lets software
                // clear: the busy and data flags are live state.
                status &= ~(value & ClearableFlags);
                UpdateSignals();
                return;
            case Mier:
                interruptEnable = value;
                UpdateSignals();
                return;
            case Mder:
                dmaEnable = value;
                UpdateSignals();
                return;
            case Mcfgr1:
                config1 = value;
                return;
            case Mfcr:
                fifoControl = value;
                UpdateSignals();
                return;
            case Mtdr:
                Command(value);
                return;
            default:
                registers[offset] = value;
                return;
            }
        }

        // The guest reaches these registers at three widths and the width is not
        // decoration: the eDMA copies the command words to MTDR sixteen bits at
        // a time (i2c_mcux_lpi2c.c sets source/dest_data_size = 2) and takes the
        // received bytes out of MRDR eight at a time (size = 1). A peripheral
        // that declares only IDoubleWordPeripheral and no [AllowedTranslations]
        // gets Renode's NotTranslated stubs for the other two widths, which log
        // a warning and then drop the write or return zero - so every command
        // would vanish and every read would time out, with the driver reporting
        // success on the writes. AP_IMXRT_EDMA.cs carries the same note about
        // its own registers.
        public ushort ReadWord(long offset)
        {
            if(offset == Mrdr)
            {
                return (ushort)ReadDoubleWord(offset);
            }
            var aligned = offset & ~3;
            var shift = (int)(offset & 3) * 8;
            return (ushort)(ReadDoubleWord(aligned) >> shift);
        }

        public void WriteWord(long offset, ushort value)
        {
            if(offset == Mtdr)
            {
                // The whole command: three bits of opcode and a data byte.
                Command(value);
                return;
            }
            var aligned = offset & ~3;
            var shift = (int)(offset & 3) * 8;
            var merged = (ReadStored(aligned) & ~(0xFFFFu << shift)) | ((uint)value << shift);
            WriteDoubleWord(aligned, merged);
        }

        public byte ReadByte(long offset)
        {
            if(offset >= Mrdr && offset < Mrdr + 4)
            {
                // One byte out of the receive FIFO, which is how the eDMA drains
                // it. Only the low byte carries data; the RXEMPTY flag lives
                // above it and a byte read cannot see it.
                var word = ReadDoubleWord(Mrdr);
                return (byte)(word >> ((int)(offset - Mrdr) * 8));
            }
            var aligned = offset & ~3;
            var shift = (int)(offset & 3) * 8;
            return (byte)(ReadDoubleWord(aligned) >> shift);
        }

        public void WriteByte(long offset, byte value)
        {
            var aligned = offset & ~3;
            var shift = (int)(offset & 3) * 8;
            if(aligned == Mtdr)
            {
                this.Log(LogLevel.Warning, "byte write to MTDR at 0x{0:X}: a command is 16 bits", offset);
                return;
            }
            var merged = (ReadStored(aligned) & ~(0xFFu << shift)) | ((uint)value << shift);
            WriteDoubleWord(aligned, merged);
        }

        public long Size => 0x4000;

        public GPIO IRQ { get; }

        // One line for both directions, as the silicon has: which way it is
        // serving is decided by MDER, and the eDMA channel the guest armed.
        public GPIO DmaRequest { get; }

        // Readable from the monitor (`sysbus.lpi2c2 Commands`), for the same
        // reason as the LPSPI counters: to tell a bus that was never driven
        // from one that was driven and answered by nobody.
        public ulong Commands { get; private set; }

        public ulong Nacks { get; private set; }

        // Reads for a read-modify-write must not have the side effects a guest
        // read has: taking a byte out of the receive FIFO to merge a write into
        // an unrelated register would lose it.
        private uint ReadStored(long offset)
        {
            switch(offset)
            {
            case Mcr:
                return control;
            case Msr:
                return status;
            case Mier:
                return interruptEnable;
            case Mder:
                return dmaEnable;
            case Mcfgr1:
                return config1;
            case Mfcr:
                return fifoControl;
            default:
                uint stored;
                return registers.TryGetValue(offset, out stored) ? stored : 0u;
            }
        }

        private void Command(uint word)
        {
            Commands++;
            var command = (word & CommandMask) >> CommandShift;
            var data = (byte)(word & DataMask);
            switch(command)
            {
            case TxDataCommand:
                writeBuffer.Add(data);
                break;

            case RxDataCommand:
                FlushWrite();
                if(currentSlave == null)
                {
                    // Nothing at that address. Silicon NACKs the address byte;
                    // the driver aborts the transfer and ArduPilot's probe moves
                    // on, which is exactly what should happen for a sensor this
                    // board does not have.
                    status |= MsrNackDetect;
                    break;
                }
                {
                    var count = data + 1;
                    var bytes = currentSlave.Read(count);
                    for(var index = 0; index < count; index++)
                    {
                        rxFifo.Enqueue(index < bytes.Length ? bytes[index] : (byte)0);
                    }
                }
                break;

            case StopCommand:
                FlushWrite();
                EndTransaction();
                status |= MsrStopDetect | MsrEndPacket;
                break;

            case StartCommand:
                // A repeated START ends the previous phase without a STOP. The
                // child keeps its register pointer across FinishTransmission()
                // (AP_I2CRegisterDevice.cs:53), which is what makes the usual
                // write-register-then-read-bytes pair work.
                FlushWrite();
                EndTransaction();
                currentAddress = data >> 1;
                II2CPeripheral slave;
                if(!TryGetByAddress(currentAddress, out slave))
                {
                    currentSlave = null;
                    status |= MsrNackDetect;
                    Nacks++;
                    this.Log(LogLevel.Debug, "no device at 0x{0:X2} - NACK", currentAddress);
                    break;
                }
                currentSlave = slave;
                status |= MsrMasterBusy | MsrBusBusy;
                break;

            default:
                this.Log(LogLevel.Warning, "unsupported LPI2C command {0} (word 0x{1:X})", command, word);
                break;
            }
            UpdateSignals();
        }

        private void FlushWrite()
        {
            if(writeBuffer.Count == 0)
            {
                return;
            }
            if(currentSlave == null)
            {
                writeBuffer.Clear();
                status |= MsrNackDetect;
                return;
            }
            currentSlave.Write(writeBuffer.ToArray());
            writeBuffer.Clear();
        }

        private void EndTransaction()
        {
            if(currentSlave != null)
            {
                currentSlave.FinishTransmission();
            }
            currentSlave = null;
            currentAddress = -1;
            status &= ~(MsrMasterBusy | MsrBusBusy);
        }

        private uint CurrentStatus
        {
            get
            {
                // TDF: the FIFO always has room, because a command is executed
                // as it is written. RDF: real, and what the read path waits on.
                var live = MsrTxReady;
                if(rxFifo.Count > 0)
                {
                    live |= MsrRxReady;
                }
                return status | live;
            }
        }

        private void UpdateSignals()
        {
            var enabled = (control & McrMasterEnable) != 0;
            IRQ.Set(enabled && (CurrentStatus & interruptEnable & InterruptFlags) != 0);

            // The request is a level. TX: the guest may send more commands, so
            // it is asserted for as long as it has the channel armed. RX: only
            // while there is something to take.
            var transmit = (dmaEnable & MderTxDma) != 0;
            var receive = (dmaEnable & MderRxDma) != 0 && rxFifo.Count > 0;
            DmaRequest.Set(enabled && (transmit || receive));
        }

        private readonly List<byte> writeBuffer;
        private readonly Queue<byte> rxFifo;
        private readonly Dictionary<long, uint> registers = new Dictionary<long, uint>();

        private II2CPeripheral currentSlave;
        private int currentAddress;
        private uint control;
        private uint status;
        private uint interruptEnable;
        private uint dmaEnable;
        private uint config1;
        private uint fifoControl;

        // Register offsets, from the RT1170 LPI2C register layout.
        private const long Verid = 0x00;
        private const long Param = 0x04;
        private const long Mcr = 0x10;
        private const long Msr = 0x14;
        private const long Mier = 0x18;
        private const long Mder = 0x1C;
        private const long Mcfgr1 = 0x24;
        private const long Mfcr = 0x58;
        private const long Mfsr = 0x5C;
        private const long Mtdr = 0x60;
        private const long Mrdr = 0x70;

        private const uint McrMasterEnable = 0x1;
        private const uint McrRst = 0x2;
        private const uint McrRtf = 0x100;
        private const uint McrRrf = 0x200;

        private const uint MsrTxReady = 0x1;
        private const uint MsrRxReady = 0x2;
        private const uint MsrEndPacket = 0x100;
        private const uint MsrStopDetect = 0x200;
        private const uint MsrNackDetect = 0x400;
        private const uint MsrArbitrationLost = 0x800;
        private const uint MsrFifoError = 0x1000;
        private const uint MsrPinLowTimeout = 0x2000;
        private const uint MsrDataMatch = 0x4000;
        private const uint MsrMasterBusy = 0x1000000;
        private const uint MsrBusBusy = 0x2000000;

        private const uint ClearableFlags = MsrEndPacket | MsrStopDetect | MsrNackDetect |
                                            MsrArbitrationLost | MsrFifoError | MsrPinLowTimeout |
                                            MsrDataMatch;
        private const uint InterruptFlags = MsrTxReady | MsrRxReady | MsrEndPacket | MsrStopDetect |
                                            MsrNackDetect | MsrArbitrationLost | MsrFifoError |
                                            MsrPinLowTimeout | MsrDataMatch;

        private const uint MderTxDma = 0x1;
        private const uint MderRxDma = 0x2;

        private const uint RxEmpty = 0x4000;
        private const uint RxCountMask = 0x70000;
        private const int RxCountShift = 16;

        private const uint CommandMask = 0x700;
        private const int CommandShift = 8;
        private const uint DataMask = 0xFF;

        private const uint TxDataCommand = 0;
        private const uint RxDataCommand = 1;
        private const uint StopCommand = 2;
        private const uint StartCommand = 4;
    }
}
