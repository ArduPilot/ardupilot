//
// RP2350 DMA: 16 channels, DREQ pacing, chaining, four interrupt lines.
//
// Each channel has four register views (the AL1-AL3 aliases), each ending in
// a trigger register. A channel moves data element by element: unpaced
// (TREQ_SEL 0x3F, or a DMA timer) it runs to completion when triggered; paced
// by a peripheral DREQ it moves one element each time it finds that DREQ
// asserted. Peripherals drive their DREQs into this model's GPIO inputs,
// numbered as TREQ_SEL (SPI0 TX = 24 ... UART1 RX = 31), and the level is
// re-checked after every element, so a peripheral that deasserts its DREQ
// stops the channel.
//
// Transfers happen immediately in virtual time; there is no bus timing.
//
// GPIO outputs: 0-3 = DMA_IRQ_0..3 for core0, 4-7 = for core1.
//
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_RP2350_DMA : IDoubleWordPeripheral, IKnownSize, INumberedGPIOOutput, IGPIOReceiver
    {
        public AP_RP2350_DMA(IMachine machine)
        {
            this.machine = machine;
            var outputs = new Dictionary<int, IGPIO>();
            for(var i = 0; i < 8; i++)
            {
                outputs[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(outputs);
            for(var i = 0; i < NumChannels; i++)
            {
                channels[i] = new Channel();
            }
            Reset();
        }

        public long Size => 0x4000;

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public void Reset()
        {
            foreach(var ch in channels)
            {
                ch.Reset();
            }
            intr = 0;
            for(var i = 0; i < 4; i++)
            {
                inte[i] = 0;
                intf[i] = 0;
            }
            for(var i = 0; i < dreq.Length; i++)
            {
                dreq[i] = false;
            }
            UpdateInterrupts();
        }

        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= dreq.Length)
            {
                return;
            }
            dreq[number] = value;
            if(value)
            {
                Pump();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            var reg = offset & 0xFFF;
            if(reg < ChannelsEnd)
            {
                return ReadChannel(channels[reg >> 6], reg & 0x3F);
            }
            switch(reg)
            {
                case Intr0:
                case Intr0 + 0x10:
                case Intr0 + 0x20:
                case Intr0 + 0x30:
                    return intr;
                case ChanAbort:
                    return 0;
                case NChannels:
                    return NumChannels;
                default:
                    if(reg >= Intr0 && reg < Intr0 + 0x40)
                    {
                        var line = (int)((reg - Intr0) >> 4);
                        switch((reg - Intr0) & 0xF)
                        {
                            case 0x4: return inte[line];
                            case 0x8: return intf[line];
                            case 0xC: return Status(line);
                        }
                    }
                    return other.TryGetValue(reg, out var v) ? v : 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var reg = offset & 0xFFF;
            var alias = offset >> 12;
            if(alias != 0)
            {
                if(IsWriteToAct(reg))
                {
                    // INTR/INTS/CHAN_ABORT/MULTI_CHAN_TRIGGER act on the bits
                    // written through SET or XOR; CLR has nothing to act on
                    if(alias == 3)
                    {
                        return;
                    }
                }
                else
                {
                    var current = ReadDoubleWord(reg);
                    switch(alias)
                    {
                        case 1: value = current ^ value; break;
                        case 2: value = current | value; break;
                        case 3: value = current & ~value; break;
                    }
                }
            }
            if(reg < ChannelsEnd)
            {
                WriteChannel((int)(reg >> 6), reg & 0x3F, value);
                return;
            }
            if(reg >= Intr0 && reg < Intr0 + 0x40)
            {
                var line = (int)((reg - Intr0) >> 4);
                switch((reg - Intr0) & 0xF)
                {
                    case 0x0:
                    case 0xC:
                        // INTR and INTSn: write 1 to clear the raw status
                        intr &= ~(value & 0xFFFF);
                        break;
                    case 0x4:
                        inte[line] = value & 0xFFFF;
                        break;
                    case 0x8:
                        intf[line] = value & 0xFFFF;
                        break;
                }
                UpdateInterrupts();
                return;
            }
            switch(reg)
            {
                case MultiChanTrigger:
                    for(var i = 0; i < NumChannels; i++)
                    {
                        if((value & (1u << i)) != 0)
                        {
                            Trigger(i);
                        }
                    }
                    Pump();
                    return;
                case ChanAbort:
                    for(var i = 0; i < NumChannels; i++)
                    {
                        if((value & (1u << i)) != 0)
                        {
                            channels[i].Busy = false;
                        }
                    }
                    return;
                default:
                    other[reg] = value;
                    return;
            }
        }

        private static bool IsWriteToAct(long reg)
        {
            if(reg >= Intr0 && reg < Intr0 + 0x40)
            {
                var sub = (reg - Intr0) & 0xF;
                return sub == 0x0 || sub == 0xC;
            }
            return reg == ChanAbort || reg == MultiChanTrigger;
        }

        // Register order of the four views, as indices into Channel.Regs:
        // 0 = READ_ADDR, 1 = WRITE_ADDR, 2 = TRANS_COUNT, 3 = CTRL.
        private static readonly int[][] Views =
        {
            new[] { 0, 1, 2, 3 },
            new[] { 3, 0, 1, 2 },
            new[] { 3, 2, 0, 1 },
            new[] { 3, 1, 2, 0 },
        };

        private uint ReadChannel(Channel ch, long reg)
        {
            var which = Views[reg >> 4][(reg >> 2) & 3];
            switch(which)
            {
                case 0: return ch.ReadAddr;
                case 1: return ch.WriteAddr;
                case 2: return ch.Busy ? ch.Remaining : ch.TransCount;
                default: return ch.Ctrl | (ch.Busy ? CtrlBusy : 0);
            }
        }

        private void WriteChannel(int n, long reg, uint value)
        {
            var ch = channels[n];
            var which = Views[reg >> 4][(reg >> 2) & 3];
            switch(which)
            {
                case 0: ch.ReadAddr = value; break;
                case 1: ch.WriteAddr = value; break;
                case 2: ch.TransCount = value; break;
                default: ch.Ctrl = value & ~(CtrlBusy | 0xE0000000u); break;
            }
            // the last register of each view is the trigger; writing zero
            // there is a null trigger
            if(((reg >> 2) & 3) == 3 && value != 0)
            {
                Trigger(n);
                Pump();
            }
        }

        private void Trigger(int n)
        {
            var ch = channels[n];
            if((ch.Ctrl & CtrlEn) == 0)
            {
                return;
            }
            ch.Busy = true;
            ch.Remaining = ch.TransCount & 0x0FFFFFFF;
            ch.Mode = ch.TransCount >> 28;
            if(ch.Remaining == 0)
            {
                Complete(n);
            }
        }

        // Run every busy channel whose request is present, until nothing moves.
        private void Pump()
        {
            if(pumping)
            {
                repump = true;
                return;
            }
            pumping = true;
            try
            {
                do
                {
                    repump = false;
                    for(var n = 0; n < NumChannels; n++)
                    {
                        var ch = channels[n];
                        while(ch.Busy && RequestPresent(ch))
                        {
                            Step(n);
                        }
                    }
                }
                while(repump);
            }
            finally
            {
                pumping = false;
            }
        }

        private bool RequestPresent(Channel ch)
        {
            var treq = (int)((ch.Ctrl >> 17) & 0x3F);
            if(treq == TreqPermanent || (treq >= TreqTimer0 && treq <= TreqTimer3))
            {
                return true;
            }
            return treq < dreq.Length && dreq[treq];
        }

        private void Step(int n)
        {
            var ch = channels[n];
            var size = 1 << (int)((ch.Ctrl >> 2) & 3);
            var bus = machine.SystemBus;
            uint data;
            switch(size)
            {
                case 1: data = bus.ReadByte(ch.ReadAddr); break;
                case 2: data = bus.ReadWord(ch.ReadAddr); break;
                default: data = bus.ReadDoubleWord(ch.ReadAddr); break;
            }
            switch(size)
            {
                case 1: bus.WriteByte(ch.WriteAddr, (byte)data); break;
                case 2: bus.WriteWord(ch.WriteAddr, (ushort)data); break;
                default: bus.WriteDoubleWord(ch.WriteAddr, data); break;
            }
            ch.ReadAddr = Advance(ch, ch.ReadAddr, size, (ch.Ctrl & CtrlIncrRead) != 0,
                (ch.Ctrl & CtrlIncrReadRev) != 0, (ch.Ctrl & CtrlRingSel) == 0);
            ch.WriteAddr = Advance(ch, ch.WriteAddr, size, (ch.Ctrl & CtrlIncrWrite) != 0,
                (ch.Ctrl & CtrlIncrWriteRev) != 0, (ch.Ctrl & CtrlRingSel) != 0);
            if(--ch.Remaining == 0)
            {
                Complete(n);
            }
        }

        private static uint Advance(Channel ch, uint addr, int size, bool incr, bool rev, bool ringApplies)
        {
            if(!incr)
            {
                return addr;
            }
            var next = rev ? addr - (uint)size : addr + (uint)size;
            var ring = (int)((ch.Ctrl >> 8) & 0xF);
            if(ringApplies && ring != 0)
            {
                var mask = (1u << ring) - 1;
                next = (addr & ~mask) | (next & mask);
            }
            return next;
        }

        private void Complete(int n)
        {
            var ch = channels[n];
            ch.Busy = false;
            if(ch.Mode == ModeTriggerSelf || ch.Mode == ModeEndless)
            {
                // endless and self-retriggering channels start over
                if((ch.Ctrl & CtrlIrqQuiet) == 0)
                {
                    intr |= 1u << n;
                }
                ch.Busy = true;
                ch.Remaining = ch.TransCount & 0x0FFFFFFF;
                UpdateInterrupts();
                return;
            }
            if((ch.Ctrl & CtrlIrqQuiet) == 0)
            {
                intr |= 1u << n;
                UpdateInterrupts();
            }
            var chainTo = (int)((ch.Ctrl >> 13) & 0xF);
            if(chainTo != n)
            {
                Trigger(chainTo);
            }
        }

        private uint Status(int line)
        {
            return (intr | intf[line]) & inte[line];
        }

        private void UpdateInterrupts()
        {
            for(var line = 0; line < 4; line++)
            {
                var level = Status(line) != 0;
                Connections[line].Set(level);
                Connections[4 + line].Set(level);
            }
        }

        private class Channel
        {
            public void Reset()
            {
                ReadAddr = 0;
                WriteAddr = 0;
                TransCount = 0;
                Ctrl = 0;
                Busy = false;
                Remaining = 0;
                Mode = 0;
            }

            public uint ReadAddr;
            public uint WriteAddr;
            public uint TransCount;
            public uint Ctrl;
            public bool Busy;
            public uint Remaining;
            public uint Mode;
        }

        private readonly IMachine machine;
        private readonly Channel[] channels = new Channel[NumChannels];
        private readonly bool[] dreq = new bool[64];
        private readonly uint[] inte = new uint[4];
        private readonly uint[] intf = new uint[4];
        private readonly Dictionary<long, uint> other = new Dictionary<long, uint>();
        private uint intr;
        private bool pumping;
        private bool repump;

        private const int NumChannels = 16;
        private const long ChannelsEnd = NumChannels * 0x40;
        private const long Intr0 = 0x400;
        private const long MultiChanTrigger = 0x450;
        private const long ChanAbort = 0x464;
        private const long NChannels = 0x468;

        private const uint CtrlEn = 1u << 0;
        private const uint CtrlIncrRead = 1u << 4;
        private const uint CtrlIncrReadRev = 1u << 5;
        private const uint CtrlIncrWrite = 1u << 6;
        private const uint CtrlIncrWriteRev = 1u << 7;
        private const uint CtrlRingSel = 1u << 12;
        private const uint CtrlIrqQuiet = 1u << 23;
        private const uint CtrlBusy = 1u << 26;

        private const int TreqTimer0 = 0x3B;
        private const int TreqTimer3 = 0x3E;
        private const int TreqPermanent = 0x3F;
        private const uint ModeTriggerSelf = 0x1;
        private const uint ModeEndless = 0xF;
    }
}
