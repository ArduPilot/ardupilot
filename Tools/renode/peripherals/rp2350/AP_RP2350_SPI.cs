//
// RP2350 SPI0/SPI1: an ARM PL022 in master mode with DMA requests.
//
// Every frame written to DR is exchanged at once with the attached SPI
// peripheral (normally an AP_SPIMultiplexer, which picks the device from its
// chip-select GPIO inputs) and the reply is queued for DR reads. The transmit
// FIFO is therefore never full, and the receive FIFO is unbounded so that a
// DMA burst on the transmit side cannot overrun it before the receive channel
// drains it.
//
// DMA requests follow DMACR: DmaTx is asserted whenever TXDMAE is set, DmaRx
// while RXDMAE is set and data is waiting. Wire them to the DMA model's DREQ
// inputs (SPI0: 24/25, SPI1: 26/27).
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_RP2350_SPI : NullRegistrationPointPeripheralContainer<ISPIPeripheral>, IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral, IKnownSize
    {
        public AP_RP2350_SPI(IMachine machine) : base(machine)
        {
            IRQ = new GPIO();
            DmaTx = new GPIO();
            DmaRx = new GPIO();
            Reset();
        }

        public GPIO IRQ { get; }
        public GPIO DmaTx { get; }
        public GPIO DmaRx { get; }

        public long Size => 0x4000;

        public override void Reset()
        {
            rx.Clear();
            cr0 = 0;
            cr1 = 0;
            cpsr = 0;
            imsc = 0;
            dmacr = 0;
            overrun = false;
            Update();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset & 0xFFC)
            {
                case Cr0: return cr0;
                case Cr1: return cr1;
                case Dr:
                    var value = rx.Count > 0 ? rx.Dequeue() : 0u;
                    Update();
                    return value;
                case Sr: return SrValue;
                case Cpsr: return cpsr;
                case Imsc: return imsc;
                case Ris: return RisValue;
                case Mis: return RisValue & imsc;
                case Dmacr: return dmacr;
                // PL022 peripheral and PrimeCell ID registers
                case 0xFE0: return 0x22;
                case 0xFE4: return 0x10;
                case 0xFE8: return 0x34;
                case 0xFEC: return 0x00;
                case 0xFF0: return 0x0D;
                case 0xFF4: return 0xF0;
                case 0xFF8: return 0x05;
                case 0xFFC: return 0xB1;
                default: return 0;
            }
        }

        // Narrow accesses are handled directly rather than by Renode's
        // read-modify-write translation: reading DR pops the receive FIFO, so
        // a DMA byte write to DR must not read it first.
        public byte ReadByte(long offset)
        {
            return (byte)(ReadDoubleWord(offset & ~3L) >> (int)((offset & 3) * 8));
        }

        public void WriteByte(long offset, byte value)
        {
            WriteDoubleWord(offset & ~3L, (uint)value << (int)((offset & 3) * 8));
        }

        public ushort ReadWord(long offset)
        {
            return (ushort)(ReadDoubleWord(offset & ~3L) >> (int)((offset & 2) * 8));
        }

        public void WriteWord(long offset, ushort value)
        {
            WriteDoubleWord(offset & ~3L, (uint)value << (int)((offset & 2) * 8));
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var reg = offset & 0xFFC;
            var alias = offset >> 12;
            var current = ReadRegister(reg);
            switch(alias)
            {
                case 1: value = current ^ value; break;
                case 2: value = current | value; break;
                case 3: value = current & ~value; break;
            }
            switch(reg)
            {
                case Cr0: cr0 = value & 0xFFFF; break;
                case Cr1: cr1 = value & 0xF; break;
                case Dr:
                    Exchange(value);
                    break;
                case Cpsr: cpsr = value & 0xFE; break;
                case Imsc: imsc = value & 0xF; break;
                case Icr:
                    if((value & 1) != 0)
                    {
                        overrun = false;
                    }
                    break;
                case Dmacr: dmacr = value & 0x3; break;
            }
            Update();
        }

        private uint ReadRegister(long reg)
        {
            switch(reg)
            {
                case Cr0: return cr0;
                case Cr1: return cr1;
                case Cpsr: return cpsr;
                case Imsc: return imsc;
                case Dmacr: return dmacr;
                default: return 0;
            }
        }

        private void Exchange(uint value)
        {
            var bits = (int)(cr0 & 0xF) + 1;
            var device = RegisteredPeripheral;
            uint reply = 0;
            if(bits <= 8)
            {
                reply = device == null ? 0xFFu : device.Transmit((byte)value);
            }
            else
            {
                // wider frames go out most significant byte first
                var hi = device == null ? (byte)0xFF : device.Transmit((byte)(value >> 8));
                var lo = device == null ? (byte)0xFF : device.Transmit((byte)value);
                reply = ((uint)hi << 8) | lo;
            }
            if(rx.Count >= MaxQueued)
            {
                overrun = true;
                return;
            }
            rx.Enqueue(reply & ((1u << bits) - 1));
        }

        private uint SrValue
        {
            get
            {
                var sr = SrTfe | SrTnf;
                if(rx.Count > 0)
                {
                    sr |= SrRne;
                }
                if(rx.Count >= FifoDepth)
                {
                    sr |= SrRff;
                }
                return sr;
            }
        }

        private uint RisValue
        {
            get
            {
                var ris = RisTx;
                if(overrun)
                {
                    ris |= RisRor;
                }
                if(rx.Count > 0)
                {
                    ris |= RisRt;
                }
                if(rx.Count >= FifoDepth / 2)
                {
                    ris |= RisRx;
                }
                return ris;
            }
        }

        private void Update()
        {
            IRQ.Set((RisValue & imsc) != 0);
            DmaTx.Set((dmacr & DmaTxe) != 0);
            DmaRx.Set((dmacr & DmaRxe) != 0 && rx.Count > 0);
        }

        private readonly Queue<uint> rx = new Queue<uint>();
        private uint cr0;
        private uint cr1;
        private uint cpsr;
        private uint imsc;
        private uint dmacr;
        private bool overrun;

        private const int FifoDepth = 8;
        private const int MaxQueued = 65536;

        private const long Cr0 = 0x00;
        private const long Cr1 = 0x04;
        private const long Dr = 0x08;
        private const long Sr = 0x0C;
        private const long Cpsr = 0x10;
        private const long Imsc = 0x14;
        private const long Ris = 0x18;
        private const long Mis = 0x1C;
        private const long Icr = 0x20;
        private const long Dmacr = 0x24;

        private const uint SrTfe = 1u << 0;
        private const uint SrTnf = 1u << 1;
        private const uint SrRne = 1u << 2;
        private const uint SrRff = 1u << 3;
        private const uint RisRor = 1u << 0;
        private const uint RisRt = 1u << 1;
        private const uint RisRx = 1u << 2;
        private const uint RisTx = 1u << 3;
        private const uint DmaRxe = 1u << 0;
        private const uint DmaTxe = 1u << 1;
    }
}
