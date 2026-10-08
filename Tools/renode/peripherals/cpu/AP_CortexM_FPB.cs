// Cortex-M FPB revision 2, eight instruction comparators. Requires the
// DebugMonitor CPU patch in Tools/renode/patches/cortex-m-debug-monitor.patch.
using System;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public sealed class AP_CortexM_FPB : IDoubleWordPeripheral, IKnownSize
    {
        public AP_CortexM_FPB(CortexM cpu)
        {
            this.cpu = cpu;
            Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == 0)
            {
                return 0x10000080u | control;
            }
            if(offset >= 8 && offset < 40 && (offset & 3) == 0)
            {
                return comparators[(offset - 8) / 4];
            }
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == 0 && (value & 2) != 0)
            {
                control = value & 1;
                cpu.SetFPBControl(control);
            }
            else if(offset >= 8 && offset < 40 && (offset & 3) == 0)
            {
                var index = (uint)(offset - 8) / 4;
                comparators[index] = value;
                cpu.SetFPBComparator(index, value);
            }
        }

        public void Reset()
        {
            control = 0;
            cpu.SetFPBControl(0);
            for(uint i = 0; i < comparators.Length; ++i)
            {
                comparators[i] = 0;
                cpu.SetFPBComparator(i, 0);
            }
        }

        public long Size => 0x1000;
        private readonly CortexM cpu;
        private readonly uint[] comparators = new uint[8];
        private uint control;
    }
}
