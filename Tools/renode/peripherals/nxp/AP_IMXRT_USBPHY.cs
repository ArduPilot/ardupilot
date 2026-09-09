//
// The RT1176 USB PHY, to the depth a boot needs.
//
// CLOCK_EnableUsbhs0PhyPllClock() brings the PHY PLL up and then spins:
//
//     while (0UL == (USBPHY1->PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK_MASK)) { }
//
// PLL_LOCK is read-only, so with no model behind the address the board stops
// there. It stops during driver init, before ArduPilot starts, and because this
// board's console is USB CDC it stops without printing anything at all.
//
// This is not a USB model and does not pretend to be one: Renode has no device
// controller for this part, so the CDC console cannot come up under emulation
// no matter what happens here. What the emulated board needs is for USB init to
// finish rather than hang, so that the MAVLink stream on LPUART4 - which is
// what the boot check reads - gets a chance to start.
//
// The one behaviour worth having right is the SET/CLR/TOG aliasing every i.MX
// register block uses, because the PHY driver configures itself entirely
// through those aliases. Treating them as four independent registers would let
// a driver read back a value it never wrote.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_IMXRT_USBPHY : IDoubleWordPeripheral, IKnownSize
    {
        public AP_IMXRT_USBPHY(IMachine machine)
        {
            this.machine = machine;
            registers = new Dictionary<long, uint>();
        }

        public long Size => 0x1000;

        public void Reset()
        {
            registers.Clear();
        }

        public uint ReadDoubleWord(long offset)
        {
            var value = Stored(Base(offset));
            uint forced;
            if(ReadyBits.TryGetValue(Base(offset), out forced))
            {
                value |= forced;
            }
            return value;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var register = Base(offset);
            var current = Stored(register);
            switch(offset & 0xc)
            {
            case 0x4:                       // <reg>_SET
                registers[register] = current | value;
                break;
            case 0x8:                       // <reg>_CLR
                registers[register] = current & ~value;
                break;
            case 0xc:                       // <reg>_TOG
                registers[register] = current ^ value;
                break;
            default:
                registers[register] = value;
                break;
            }
        }

        // The register a SET/CLR/TOG alias belongs to. Every block in this
        // peripheral is a quad on a 16-byte boundary.
        private static long Base(long offset)
        {
            return offset & ~0xcL;
        }

        private uint Stored(long offset)
        {
            uint value;
            return registers.TryGetValue(offset, out value) ? value : 0;
        }

        // offset -> bits that always read as set
        private static readonly Dictionary<long, uint> ReadyBits = new Dictionary<long, uint>
        {
            // PLL_SIC.PLL_LOCK - the PHY PLL has locked
            { 0x0a0, 0x80000000 },
        };

        private readonly IMachine machine;
        private readonly Dictionary<long, uint> registers;
    }
}
