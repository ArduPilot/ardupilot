//
// i.MX RT LPSPI master, to the depth ArduPilot's IMU traffic needs.
//
// Renode has an SPI.IMXRT_LPSPI, and the i.MX RT platform used it, but it has
// no DMA request output - the platform file says so where it declines to wire
// the LPSPI request sources into the DMAMUX. That is fatal here rather than
// merely slow: this board's Zephyr build has CONFIG_SPI_NXP_LPSPI_DMA=y and the
// DTS gives lpspi1..3 two eDMA channels each, so the driver takes the DMA path
// and nothing else. Without a request line the transfer never starts, the IMU
// never answers, and the board cannot fly.
//
// So: a small master of our own, with the request lines the eDMA model
// (AP_IMXRT_EDMA.cs, proven in service) already knows how to consume.
//
// Transfers are synchronous. A write to TDR hands the byte to the selected
// child and takes the answer back in the same call, so TDF is always set and
// RDF is set whenever the RX FIFO has something in it. The guest only polls
// those flags, so it cannot tell the difference.
//
// Chip select is NOT done here. This board's DTS drives the three IMU selects
// with GPIOs (lpspi1 cs-gpios = <&gpio2 11>), not the LPSPI's own PCS, so the
// selection and the transaction framing both arrive at the sensor models
// through Miscellaneous.AP_SPIMultiplexer and its GPIO inputs - the same path
// the STM32 boards use. This model therefore talks to exactly one child (the
// multiplexer) and never calls FinishTransmission(): the sensor models take
// their framing from the chip select, as AP_ICM42688's IGPIOReceiver does.
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

namespace Antmicro.Renode.Peripherals.SPI
{
    public class AP_IMXRT_LPSPI : NullRegistrationPointPeripheralContainer<ISPIPeripheral>,
        IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral, IKnownSize
    {
        public AP_IMXRT_LPSPI(IMachine machine) : base(machine)
        {
            IRQ = new GPIO();
            TransmitDMA = new GPIO();
            ReceiveDMA = new GPIO();
            rxFifo = new Queue<byte>();
            Reset();
        }

        public override void Reset()
        {
            control = 0;
            ResetExceptControl();
        }

        // Everything CR[RST] clears: all internal logic and registers other
        // than the Control Register itself.
        private void ResetExceptControl()
        {
            rxFifo.Clear();
            status = 0;
            interruptEnable = 0;
            dmaEnable = 0;
            transmitCommand = 0;
            fifoControl = 0;
            registers.Clear();
            UpdateSignals();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case Verid:
                return 0x01000004;
            case Param:
                // 16-word FIFOs, as the RT1170 LPSPI has.
                return (16u << 8) | 16u;
            case Cr:
                return control;
            case Sr:
                return CurrentStatus;
            case Ier:
                return interruptEnable;
            case Der:
                return dmaEnable;
            case Fsr:
                // TX count stays zero: a word written to TDR has already been
                // clocked out by the time the write returns.
                return ((uint)rxFifo.Count << RxCountShift) & RxCountMask;
            case Tcr:
                return transmitCommand;
            case Fcr:
                return fifoControl;
            case Rsr:
                return rxFifo.Count == 0 ? RsrRxEmpty : 0u;
            case Rdr:
                if(rxFifo.Count == 0)
                {
                    return 0;
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
            case Cr:
                // CR[RST] "resets all internal logic and registers, EXCEPT the
                // Control Register" (i.MX RT1170 RM, LPSPI Control Register).
                // Module enable therefore survives a software reset, and this
                // is not a detail: AP_HAL_Zephyr resets the block before every
                // transfer (SPIDevice.cpp ap_spi_bus_reset) and the Zephyr
                // driver only reprograms the peripheral when the config pointer
                // changed - so on the paths where it does not, MEN is never
                // rewritten. A model that cleared MEN here left the block
                // disabled, no DMA request could assert, and every large
                // transfer ended in -ETIMEDOUT while short ones (which do take
                // the reconfigure path) worked.
                //
                // RTF/RRF are self-clearing FIFO flushes, so they are not
                // stored; RST is, because it "remains set until cleared by
                // software".
                control = value & ~(CrRtf | CrRrf);
                if((value & CrRrf) != 0)
                {
                    rxFifo.Clear();
                }
                if((value & CrRst) != 0)
                {
                    ResetExceptControl();
                }
                UpdateSignals();
                return;
            case Sr:
                status &= ~(value & ClearableFlags);
                UpdateSignals();
                return;
            case Ier:
                interruptEnable = value;
                UpdateSignals();
                return;
            case Der:
                dmaEnable = value;
                UpdateSignals();
                this.Log(LogLevel.Info, "DER=0x{0:X} -> TX req {1}, RX req {2} (rxFifo {3}, CR 0x{4:X}, TCR 0x{5:X})",
                         value, TransmitDMA.IsSet, ReceiveDMA.IsSet, rxFifo.Count, control, transmitCommand);
                return;
            case Tcr:
                transmitCommand = value;
                return;
            case Fcr:
                fifoControl = value;
                UpdateSignals();
                return;
            case Tdr:
                Transfer(value);
                return;
            default:
                registers[offset] = value;
                return;
            }
        }

        // Byte and word accesses are not decoration here: spi_nxp_lpspi_dma.c
        // programs source_data_size = dest_data_size = 1, so the eDMA writes
        // TDR and reads RDR one byte at a time. Declaring only
        // IDoubleWordPeripheral would give those accesses Renode's NotTranslated
        // stubs - a logged warning, the write dropped, the read zero - and the
        // IMU would never see a register address or answer with its WHOAMI.
        public ushort ReadWord(long offset)
        {
            if(offset == Rdr)
            {
                return (ushort)ReadDoubleWord(offset);
            }
            var aligned = offset & ~3;
            var shift = (int)(offset & 3) * 8;
            return (ushort)(ReadDoubleWord(aligned) >> shift);
        }

        public void WriteWord(long offset, ushort value)
        {
            if(offset == Tdr)
            {
                Transfer(value);
                return;
            }
            var aligned = offset & ~3;
            var shift = (int)(offset & 3) * 8;
            var merged = (ReadStored(aligned) & ~(0xFFFFu << shift)) | ((uint)value << shift);
            WriteDoubleWord(aligned, merged);
        }

        public byte ReadByte(long offset)
        {
            if(offset >= Rdr && offset < Rdr + 4)
            {
                var word = ReadDoubleWord(Rdr);
                return (byte)(word >> ((int)(offset - Rdr) * 8));
            }
            var aligned = offset & ~3;
            var shift = (int)(offset & 3) * 8;
            return (byte)(ReadDoubleWord(aligned) >> shift);
        }

        public void WriteByte(long offset, byte value)
        {
            if(offset == Tdr)
            {
                Transfer(value);
                return;
            }
            var aligned = offset & ~3;
            var shift = (int)(offset & 3) * 8;
            var merged = (ReadStored(aligned) & ~(0xFFu << shift)) | ((uint)value << shift);
            WriteDoubleWord(aligned, merged);
        }

        public long Size => 0x4000;

        public GPIO IRQ { get; }

        public GPIO TransmitDMA { get; }

        public GPIO ReceiveDMA { get; }

        // Readable from the monitor (`sysbus.lpspi1 Words`). A transfer that
        // never happens and a transfer that happens and returns nothing look
        // identical from the guest side; these tell them apart without a
        // debugger, and let a test assert that the bus really ran.
        public ulong Words { get; private set; }

        public ulong SelectedWords { get; private set; }

        // Side-effect-free, for the read-modify-write paths above: a guest read
        // of RDR pops the receive FIFO and merging a write must not.
        private uint ReadStored(long offset)
        {
            switch(offset)
            {
            case Cr:
                return control;
            case Sr:
                return status;
            case Ier:
                return interruptEnable;
            case Der:
                return dmaEnable;
            case Tcr:
                return transmitCommand;
            case Fcr:
                return fifoControl;
            default:
                uint stored;
                return registers.TryGetValue(offset, out stored) ? stored : 0u;
            }
        }

        private void Transfer(uint word)
        {
            Words++;
            var child = RegisteredPeripheral;
            if(child == null)
            {
                // Nothing on the bus. Give the guest the idle line it would see.
                if((transmitCommand & TcrRxMask) == 0)
                {
                    rxFifo.Enqueue(0xFF);
                }
                status |= SrTransferComplete | SrFrameComplete;
                UpdateSignals();
                return;
            }

            // FRAMESZ is bits-minus-one. ArduPilot's sensor buses are byte
            // framed (8-bit words); anything wider is sent most significant
            // byte first, as the LPSPI does.
            var bits = (int)((transmitCommand & TcrFrameSizeMask) >> TcrFrameSizeShift) + 1;
            var bytes = (bits + 7) / 8;
            for(var index = bytes - 1; index >= 0; index--)
            {
                var outgoing = (transmitCommand & TcrTxMask) != 0
                    ? (byte)0
                    : (byte)(word >> (8 * index));
                var incoming = child.Transmit(outgoing);
                SelectedWords++;
                // Per-byte trace, off unless this peripheral's log level is
                // raised on its own (`logLevel 0 sysbus.lpspi1` on the monitor):
                // what the guest sent and what came back is the only way to tell
                // "the bus works" from "the bus works and the answer is wrong".
                this.Log(LogLevel.Debug, "xfer {0}: out 0x{1:X2} in 0x{2:X2} (TCR 0x{3:X}, DER 0x{4:X})",
                         SelectedWords, outgoing, incoming, transmitCommand, dmaEnable);
                if((transmitCommand & TcrRxMask) == 0)
                {
                    rxFifo.Enqueue(incoming);
                }
            }
            status |= SrTransferComplete | SrFrameComplete | SrWordComplete;
            UpdateSignals();
        }

        private uint CurrentStatus
        {
            get
            {
                var live = SrTxReady;
                if(rxFifo.Count > 0)
                {
                    live |= SrRxReady;
                }
                return status | live;
            }
        }

        private void UpdateSignals()
        {
            var enabled = (control & CrModuleEnable) != 0;
            IRQ.Set(enabled && (CurrentStatus & interruptEnable & InterruptFlags) != 0);

            // Levels, not edges: the eDMA model latches them and runs minor
            // loops for as long as they stay asserted, exactly as it does for
            // the LPUARTs.
            TransmitDMA.Set(enabled && (dmaEnable & DerTxDma) != 0);
            ReceiveDMA.Set(enabled && (dmaEnable & DerRxDma) != 0 && rxFifo.Count > 0);
        }

        private readonly Queue<byte> rxFifo;
        private readonly Dictionary<long, uint> registers = new Dictionary<long, uint>();

        private uint control;
        private uint status;
        private uint interruptEnable;
        private uint dmaEnable;
        private uint transmitCommand;
        private uint fifoControl;

        // Register offsets, from the RT1170 LPSPI register layout.
        private const long Verid = 0x00;
        private const long Param = 0x04;
        private const long Cr = 0x10;
        private const long Sr = 0x14;
        private const long Ier = 0x18;
        private const long Der = 0x1C;
        private const long Fcr = 0x58;
        private const long Fsr = 0x5C;
        private const long Tcr = 0x60;
        private const long Tdr = 0x64;
        private const long Rsr = 0x70;
        private const long Rdr = 0x74;

        private const uint CrModuleEnable = 0x1;
        private const uint CrRst = 0x2;
        private const uint CrRtf = 0x100;
        private const uint CrRrf = 0x200;

        private const uint SrTxReady = 0x1;
        private const uint SrRxReady = 0x2;
        private const uint SrWordComplete = 0x100;
        private const uint SrFrameComplete = 0x200;
        private const uint SrTransferComplete = 0x400;
        private const uint SrTransmitError = 0x800;
        private const uint SrReceiveError = 0x1000;
        private const uint SrDataMatch = 0x2000;

        private const uint ClearableFlags = SrWordComplete | SrFrameComplete | SrTransferComplete |
                                            SrTransmitError | SrReceiveError | SrDataMatch;
        private const uint InterruptFlags = SrTxReady | SrRxReady | SrWordComplete | SrFrameComplete |
                                            SrTransferComplete | SrTransmitError | SrReceiveError |
                                            SrDataMatch;

        private const uint DerTxDma = 0x1;
        private const uint DerRxDma = 0x2;

        private const uint RsrRxEmpty = 0x2;
        private const uint RxCountMask = 0x1F0000;
        private const int RxCountShift = 16;

        private const uint TcrFrameSizeMask = 0xFFF;
        private const int TcrFrameSizeShift = 0;
        private const uint TcrTxMask = 0x40000;
        private const uint TcrRxMask = 0x80000;
    }
}
