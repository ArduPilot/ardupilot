//
// The RT1176 clock controller, to the depth a boot needs.
//
// Renode has no CCM model for this part, and plain memory cannot stand in for
// one. CLOCK_ControlGate() switches a peripheral clock on by writing
// LPCG[n].DIRECT and then spinning until LPCG[n].STATUS0 reports the gate open:
//
//     CCM->LPCG[name].DIRECT = lpcgVal;
//     while ((CCM->LPCG[name].STATUS0 & ON) != (lpcgVal & ON)) { }
//
// STATUS0 is read-only, so nothing the firmware writes can ever set it, and the
// board stops at the first peripheral Zephyr clocks - before the console, so it
// stops silently. Mirroring DIRECT into STATUS0 is the whole of what is needed:
// every clock is present in an emulator, so a gate is open the moment it is
// asked for.
//
// Everything else here is a register file. CLOCK_SetRootClock() and the LPCG
// authority registers are written and read back but never waited on, so storing
// them is enough.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_IMXRT_CCM : IDoubleWordPeripheral, IKnownSize
    {
        public AP_IMXRT_CCM(IMachine machine)
        {
            this.machine = machine;
            registers = new Dictionary<long, uint>();
        }

        public long Size => 0x10000;

        public void Reset()
        {
            registers.Clear();
        }

        public uint ReadDoubleWord(long offset)
        {
            long direct;
            if(TryGetDirectOfStatus(offset, out direct))
            {
                return Stored(direct);
            }
            return Stored(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            registers[offset] = value;
        }

        // Both gate arrays have the same shape: a writable DIRECT at the start
        // of each 0x20-byte entry and a read-only STATUS0 0x10 further on. Only
        // the LPCG array is polled during a boot; OSCPLL is laid out the same
        // way and is mirrored for the same reason.
        private static bool TryGetDirectOfStatus(long offset, out long direct)
        {
            direct = 0;
            foreach(var start in GateArrays)
            {
                if(offset < start.Item1 || offset >= start.Item2)
                {
                    continue;
                }
                if((offset & 0x1f) != StatusOffset)
                {
                    continue;
                }
                direct = offset & ~0x1fL;
                return true;
            }
            return false;
        }

        private uint Stored(long offset)
        {
            uint value;
            return registers.TryGetValue(offset, out value) ? value : 0;
        }

        private const long StatusOffset = 0x10;

        // (first byte, one past the last) of each array.
        // OSCPLL: 29 entries from 0x5000. LPCG: 138 entries from 0x6000.
        private static readonly List<System.Tuple<long, long>> GateArrays =
            new List<System.Tuple<long, long>>
        {
            System.Tuple.Create(0x5000L, 0x5000L + 29 * 0x20L),
            System.Tuple.Create(0x6000L, 0x6000L + 138 * 0x20L),
        };

        private readonly IMachine machine;
        private readonly Dictionary<long, uint> registers;
    }
}
