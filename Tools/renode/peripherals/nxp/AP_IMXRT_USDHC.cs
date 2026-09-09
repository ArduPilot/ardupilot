//
// The RT1176 SD host controller, to the depth a boot needs.
//
// Renode has no USDHC model - its own i.MX RT platform tags the range and stops
// there - and the driver cannot run against plain memory. USDHC_SetSdClock()
// programs the divider and then waits for the clock to settle:
//
//     base->SYS_CTRL = ...;
//     while (0U == (base->PRSSTAT & USDHC_PRSSTAT_SDSTB_MASK)) { }
//
// PRSSTAT is read-only, so nothing the driver writes can end that wait. The
// resets in SYS_CTRL are self-clearing on silicon and have the same problem in
// reverse: written into memory they stay set and the driver waits forever.
//
// This is a stub, not a controller. It reports a stable clock, completes each
// reset immediately, and reports no card present - so the SD stack initialises
// and then concludes there is no card, which is the truthful answer here: no
// image is attached to this machine. Logging therefore does not work under this
// platform, and a board that needs SD emulated needs a real model, not this.
//
// Without it the board stops inside SD driver init, which on this board is
// before ArduPilot starts and before any console exists.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_IMXRT_USDHC : IDoubleWordPeripheral, IKnownSize
    {
        public AP_IMXRT_USDHC(IMachine machine)
        {
            this.machine = machine;
            registers = new Dictionary<long, uint>();
        }

        public long Size => 0x4000;

        public void Reset()
        {
            registers.Clear();
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == PrsstatOffset)
            {
                // Clock stable, no command or data line busy, no card. CINS
                // stays clear: there is nothing behind this to hold an image.
                return Sdstb;
            }
            uint value;
            return registers.TryGetValue(offset, out value) ? value : 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == SysCtrlOffset)
            {
                // Every reset here completes within the write on silicon's
                // timescale, and the driver's next read must see it finished.
                value &= ~(Rsta | Rstc | Rstd | Inita);
            }
            registers[offset] = value;
        }

        private const long PrsstatOffset = 0x24;
        private const long SysCtrlOffset = 0x2c;

        private const uint Sdstb = 0x00000008;   // PRSSTAT.SDSTB
        private const uint Rsta = 0x01000000;    // SYS_CTRL.RSTA - all
        private const uint Rstc = 0x02000000;    // SYS_CTRL.RSTC - command line
        private const uint Rstd = 0x04000000;    // SYS_CTRL.RSTD - data line
        private const uint Inita = 0x08000000;   // SYS_CTRL.INITA - init sequence

        private readonly IMachine machine;
        private readonly Dictionary<long, uint> registers;
    }
}
