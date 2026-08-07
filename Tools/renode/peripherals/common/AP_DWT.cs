//
// Minimal Cortex-M DWT: just CTRL.CYCCNTENA and a CYCCNT that follows
// virtual time at the CPU clock rate. Renode leaves the DWT block
// unimplemented, so CYCCNT reads 0 forever and every ChibiOS
// chSysPolledDelayX() - the OTG PHY delays in usb_lld_start are the
// first in the ArduPilot boot - spins for good. Granularity is the
// virtual-time resolution at the point of read, which is enough for
// the wait-at-least semantics polled delays want.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_DWT : IDoubleWordPeripheral, IKnownSize
    {
        public AP_DWT(IMachine machine, uint frequency = 168000000)
        {
            this.machine = machine;
            this.frequency = frequency;
        }

        public long Size => 0x1000;

        public void Reset()
        {
            control = 0;
            cyccntOffset = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case CTRL:
                return control;
            case CYCCNT:
                return CycleCount() + cyccntOffset;
            default:
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case CTRL:
                control = value & 1;
                return;
            case CYCCNT:
                cyccntOffset = value - CycleCount();
                return;
            }
        }

        private uint CycleCount()
        {
            var us = (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;
            return (uint)(us * (frequency / 1000000));
        }

        private const long CTRL = 0x0;
        private const long CYCCNT = 0x4;

        private readonly IMachine machine;
        private readonly uint frequency;
        private uint control;
        private uint cyccntOffset;
    }
}
