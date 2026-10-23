//
// RP2350 register blocks the ChibiOS start-up code waits on.
//
// Every RP2350 APB/AHB block has three atomic aliases after the normal view:
// +0x1000 XOR, +0x2000 SET, +0x3000 CLR. AP_RP2350_Block is registered with a
// 0x4000 window and decodes the alias from address bits 13:12, so a subclass
// only models the normal view. Reads through an alias see the normal view.
//
// The subclasses fake just the status bits the firmware polls, from the
// RP2350 datasheet and the CMSIS layout in modules/ChibiOS rp2350.h.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_RP2350_Block : IDoubleWordPeripheral, IKnownSize
    {
        public AP_RP2350_Block(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public long Size => 0x4000;

        public virtual void Reset()
        {
            Array.Clear(regs, 0, regs.Length);
        }

        public uint ReadDoubleWord(long offset)
        {
            return Read(offset & 0xFFC);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var reg = offset & 0xFFC;
            var current = regs[reg >> 2];
            switch(offset >> 12)
            {
                case 1: value = current ^ value; break;
                case 2: value = current | value; break;
                case 3: value = current & ~value; break;
            }
            Write(reg, value);
        }

        protected virtual uint Read(long reg)
        {
            return regs[reg >> 2];
        }

        protected virtual void Write(long reg, uint value)
        {
            regs[reg >> 2] = value;
        }

        protected readonly IMachine machine;
        protected readonly uint[] regs = new uint[0x400];
    }

    // RESET_DONE follows RESET immediately: a block is out of reset as soon as
    // its RESET bit is clear.
    public class AP_RP2350_Resets : AP_RP2350_Block
    {
        public AP_RP2350_Resets(IMachine machine) : base(machine) {}

        public override void Reset()
        {
            base.Reset();
            regs[0] = AllBlocks;
        }

        protected override uint Read(long reg)
        {
            return reg == ResetDone ? ~regs[0] & AllBlocks : base.Read(reg);
        }

        private const long ResetDone = 0x8;
        private const uint AllBlocks = 0x1FFFFFFF;
    }

    // Clock muxes switch instantly: every CLK[n].SELECTED reads all ones, which
    // satisfies both the glitchless (1 << SRC) and the "always 1" clocks.
    public class AP_RP2350_Clocks : AP_RP2350_Block
    {
        public AP_RP2350_Clocks(IMachine machine) : base(machine) {}

        protected override uint Read(long reg)
        {
            if(reg < ClkEnd && reg % 12 == 8)
            {
                return 0xFFFFFFFF;
            }
            return base.Read(reg);
        }

        private const long ClkEnd = 10 * 12;
    }

    // XOSC STATUS.STABLE (bit 31) and PLL CS.LOCK (bit 31) are always set.
    public class AP_RP2350_XOSC : AP_RP2350_Block
    {
        public AP_RP2350_XOSC(IMachine machine) : base(machine) {}

        protected override uint Read(long reg)
        {
            return reg == 0x4 ? base.Read(reg) | 0x80000000u : base.Read(reg);
        }
    }

    public class AP_RP2350_PLL : AP_RP2350_Block
    {
        public AP_RP2350_PLL(IMachine machine) : base(machine) {}

        protected override uint Read(long reg)
        {
            return reg == 0x0 ? base.Read(reg) | 0x80000000u : base.Read(reg);
        }
    }

    // POWMAN: writes up to 0xAC carry the 0x5AFE password in the upper 16 bits,
    // which the hardware does not store. The regulator settles at once:
    // VREG_STS.VOUT_OK (bit 4) is set and VREG.UPDATE_IN_PROGRESS (bit 15)
    // never is. rp2350_vreg_init() in rp_clocks.c waits on both.
    public class AP_RP2350_Powman : AP_RP2350_Block
    {
        public AP_RP2350_Powman(IMachine machine) : base(machine) {}

        protected override void Write(long reg, uint value)
        {
            if(reg <= PasswordEnd)
            {
                value &= 0xFFFF;
            }
            base.Write(reg, value);
        }

        protected override uint Read(long reg)
        {
            switch(reg)
            {
                case VregSts: return base.Read(reg) | 0x10u;
                case Vreg: return base.Read(reg) & ~0x8000u;
                default: return base.Read(reg);
            }
        }

        private const long VregSts = 0x08;
        private const long Vreg = 0x0C;
        private const long PasswordEnd = 0xAC;
    }

    // ADC: conversions complete instantly, so CS.READY (bit 8) is always set,
    // and nothing is ever queued, so FCS.EMPTY (bit 8) is too. Results read
    // zero until inputs are modelled.
    public class AP_RP2350_ADC : AP_RP2350_Block
    {
        public AP_RP2350_ADC(IMachine machine) : base(machine) {}

        protected override uint Read(long reg)
        {
            switch(reg)
            {
                case Cs: return base.Read(reg) | 0x100u;
                case Fcs: return (base.Read(reg) & ~0x000F0000u) | 0x100u;
                default: return base.Read(reg);
            }
        }

        private const long Cs = 0x0;
        private const long Fcs = 0x8;
    }

    // PIO0/1/2 placeholder until the state machines are modelled: FSTAT
    // reports every RX and TX FIFO empty, so the PIO UART drivers see no data
    // and never wait for room. Everything else is plain storage.
    public class AP_RP2350_PIO : AP_RP2350_Block
    {
        public AP_RP2350_PIO(IMachine machine) : base(machine) {}

        protected override uint Read(long reg)
        {
            // RXEMPTY is bits 11:8, TXEMPTY bits 27:24
            return reg == 0x04 ? 0x0F000F00u : base.Read(reg);
        }
    }

    // PSM: a subsystem is DONE unless it is forced off.
    public class AP_RP2350_PSM : AP_RP2350_Block
    {
        public AP_RP2350_PSM(IMachine machine) : base(machine) {}

        protected override uint Read(long reg)
        {
            return reg == Done ? ~regs[FrceOff >> 2] & AllDomains : base.Read(reg);
        }

        private const long FrceOff = 0x4;
        private const long Done = 0xC;
        private const uint AllDomains = 0x01FFFFFF;
    }
}
