//
// RP2350 UART0/UART1: an ARM PL011 with DMA requests, on Renode's UARTBase so
// that terminals, socket servers and analyzer windows attach as usual.
//
// Transmit is immediate, so the transmit FIFO always reads empty. Received
// characters queue in a 32-entry FIFO. As on the PL011 the receive interrupts
// are latched events, cleared by UARTICR and raised again only by new data:
// RXRIS when an arriving character takes the FIFO to the IFLS level (and
// dropped once reads take it below), RTRIS when a character arrives (standing
// in for the 32-bit idle timeout; dropped when the FIFO empties). Level-style
// status here would re-assert straight after the handler clears it and hold
// the core in the UART interrupt. TXRIS stays set, since there is always room;
// ChibiOS only enables TXIM while it has data to send.
//
// DMA requests follow DMACR: DmaTx whenever TXDMAE is set, DmaRx while RXDMAE
// is set and a character is waiting. Wire them to the DMA model's DREQ inputs
// (UART0: 28/29, UART1: 30/31).
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_RP2350_UART : UARTBase, IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral, IKnownSize
    {
        public AP_RP2350_UART(IMachine machine, uint frequency = 150000000) : base(machine)
        {
            this.machine = machine;
            clockFrequency = frequency;
            IRQ = new GPIO();
            DmaTx = new GPIO();
            DmaRx = new GPIO();
            Reset();
        }

        public GPIO IRQ { get; }
        public GPIO DmaTx { get; }
        public GPIO DmaRx { get; }

        public long Size => 0x4000;

        public override Bits StopBits => (lcrH & LcrStp2) != 0 ? Bits.Two : Bits.One;

        public override Parity ParityBit
        {
            get
            {
                if((lcrH & LcrPen) == 0)
                {
                    return Parity.None;
                }
                return (lcrH & LcrEps) != 0 ? Parity.Even : Parity.Odd;
            }
        }

        public override uint BaudRate
        {
            get
            {
                var divisor = ibrd * 64 + fbrd;
                return divisor == 0 ? 0 : (uint)(clockFrequency * 4UL / divisor);
            }
        }

        public override void Reset()
        {
            base.Reset();
            ibrd = 0;
            fbrd = 0;
            lcrH = 0;
            cr = 0x300;
            ifls = 0x12;
            imsc = 0;
            dmacr = 0;
            overrun = false;
            rxEvent = false;
            timeoutEvent = false;
            Update();
        }

        public uint ReadDoubleWord(long offset)
        {
            lock(innerLock)
            {
                switch(offset & 0xFFC)
                {
                    case Dr:
                        var value = TryGetCharacter(out var c) ? c : 0u;
                        Update();
                        return value;
                    case Rsr: return overrun ? 0x8u : 0u;
                    case Fr: return FrValue;
                    case Ibrd: return ibrd;
                    case Fbrd: return fbrd;
                    case LcrH: return lcrH;
                    case Cr: return cr;
                    case Ifls: return ifls;
                    case Imsc: return imsc;
                    case Ris: return RisValue;
                    case Mis: return RisValue & imsc;
                    case Dmacr: return dmacr;
                    // PL011 peripheral and PrimeCell ID registers
                    case 0xFE0: return 0x11;
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
            lock(innerLock)
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
                    case Dr:
                        if((cr & (CrUarten | CrTxe)) == (CrUarten | CrTxe))
                        {
                            TransmitCharacter((byte)value);
                        }
                        break;
                    case Rsr: overrun = false; break;
                    case Ibrd: ibrd = value & 0xFFFF; break;
                    case Fbrd: fbrd = value & 0x3F; break;
                    case LcrH: lcrH = value & 0xFF; break;
                    case Cr: cr = value & 0xFF87; break;
                    case Ifls: ifls = value & 0x3F; break;
                    case Imsc: imsc = value & 0x7FF; break;
                    case Icr:
                        if((value & RisOe) != 0)
                        {
                            overrun = false;
                        }
                        if((value & RisRx) != 0)
                        {
                            rxEvent = false;
                        }
                        if((value & RisRt) != 0)
                        {
                            timeoutEvent = false;
                        }
                        break;
                    case Dmacr: dmacr = value & 0x7; break;
                }
                Update();
            }
        }

        protected override bool IsReceiveEnabled => (cr & (CrUarten | CrRxe)) == (CrUarten | CrRxe);

        protected override void CharWritten()
        {
            // characters arrive from the host side, outside the CPU threads
            machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
            {
                lock(innerLock)
                {
                    if(Count > FifoDepth)
                    {
                        overrun = true;
                    }
                    timeoutEvent = true;
                    if(Count >= RxTriggerLevel)
                    {
                        rxEvent = true;
                    }
                    Update();
                }
            });
        }

        protected override void QueueEmptied()
        {
            Update();
        }

        private uint ReadRegister(long reg)
        {
            switch(reg)
            {
                case Ibrd: return ibrd;
                case Fbrd: return fbrd;
                case LcrH: return lcrH;
                case Cr: return cr;
                case Ifls: return ifls;
                case Imsc: return imsc;
                case Dmacr: return dmacr;
                default: return 0;
            }
        }

        private uint FrValue
        {
            get
            {
                var fr = FrTxfe;
                if(Count == 0)
                {
                    fr |= FrRxfe;
                }
                if(Count >= FifoDepth)
                {
                    fr |= FrRxff;
                }
                return fr;
            }
        }

        private uint RisValue
        {
            get
            {
                var ris = RisTx;
                if(timeoutEvent)
                {
                    ris |= RisRt;
                }
                if(rxEvent)
                {
                    ris |= RisRx;
                }
                if(overrun)
                {
                    ris |= RisOe;
                }
                return ris;
            }
        }

        // IFLS.RXIFLSEL: 1/8, 1/4, 1/2, 3/4 or 7/8 of the 32-entry FIFO
        private int RxTriggerLevel
        {
            get
            {
                switch((ifls >> 3) & 0x7)
                {
                    case 0: return 4;
                    case 1: return 8;
                    case 2: return 16;
                    case 3: return 24;
                    default: return 28;
                }
            }
        }

        private void Update()
        {
            // reading below the trigger level drops RXRIS, emptying the FIFO
            // drops RTRIS
            if(Count < RxTriggerLevel)
            {
                rxEvent = false;
            }
            if(Count == 0)
            {
                timeoutEvent = false;
            }
            IRQ.Set((RisValue & imsc) != 0);
            DmaTx.Set((dmacr & DmaTxe) != 0);
            DmaRx.Set((dmacr & DmaRxe) != 0 && Count > 0);
        }

        private readonly IMachine machine;
        private readonly uint clockFrequency;
        private uint ibrd;
        private uint fbrd;
        private uint lcrH;
        private uint cr;
        private uint ifls;
        private uint imsc;
        private uint dmacr;
        private bool overrun;
        private bool rxEvent;
        private bool timeoutEvent;

        private const int FifoDepth = 32;

        private const long Dr = 0x00;
        private const long Rsr = 0x04;
        private const long Fr = 0x18;
        private const long Ibrd = 0x24;
        private const long Fbrd = 0x28;
        private const long LcrH = 0x2C;
        private const long Cr = 0x30;
        private const long Ifls = 0x34;
        private const long Imsc = 0x38;
        private const long Ris = 0x3C;
        private const long Mis = 0x40;
        private const long Icr = 0x44;
        private const long Dmacr = 0x48;

        private const uint FrRxfe = 1u << 4;
        private const uint FrRxff = 1u << 6;
        private const uint FrTxfe = 1u << 7;
        private const uint LcrPen = 1u << 1;
        private const uint LcrEps = 1u << 2;
        private const uint LcrStp2 = 1u << 3;
        private const uint CrUarten = 1u << 0;
        private const uint CrTxe = 1u << 8;
        private const uint CrRxe = 1u << 9;
        private const uint RisRx = 1u << 4;
        private const uint RisTx = 1u << 5;
        private const uint RisRt = 1u << 6;
        private const uint RisOe = 1u << 10;
        private const uint DmaRxe = 1u << 0;
        private const uint DmaTxe = 1u << 1;
    }
}
