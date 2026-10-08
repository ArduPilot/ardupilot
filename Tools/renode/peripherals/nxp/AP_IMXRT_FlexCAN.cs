//
// The RT1176 FlexCAN mode handshake, to the depth a boot needs.
//
// Renode ships no FlexCAN model for this part - its own i.MX RT platform only
// tags the address range - and plain memory deadlocks the driver. Every mode
// change in fsl_flexcan.c is a request bit in MCR followed by a spin on the
// matching acknowledge bit, and the acknowledges are read-only:
//
//     base->MCR |= CAN_MCR_MDIS_MASK;
//     while (0U == (base->MCR & CAN_MCR_LPMACK_MASK)) { }   // disable
//     base->MCR &= ~CAN_MCR_MDIS_MASK;
//     while (0U != (base->MCR & CAN_MCR_LPMACK_MASK)) { }   // enable
//
// One constant cannot answer both, which is why a Renode Tag with a fixed read
// value does not help here. Mirroring each request into its acknowledge does,
// and costs nothing: a mode change is instantaneous in an emulator.
//
// This is a stub, not a CAN controller. It moves no frames and has no bus
// behind it. It exists so that a board whose devicetree enables CAN reaches
// main() instead of stopping in FLEXCAN_Init() during driver init - which on
// this board is before the console exists, so it stops silently.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_IMXRT_FlexCAN : IDoubleWordPeripheral, IKnownSize
    {
        public AP_IMXRT_FlexCAN(IMachine machine)
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
            uint value;
            if(!registers.TryGetValue(offset, out value))
            {
                // Out of reset the module is disabled and frozen, which is the
                // state the driver expects to find before it asks for anything.
                return offset == McrOffset
                    ? Mdis | Frz | Halt | LpmAck | FrzAck | NotRdy
                    : 0;
            }
            return offset == McrOffset ? WithAcknowledges(value) : value;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == McrOffset)
            {
                // A soft reset completes before the write returns, so the bit
                // the driver polls for is already clear when it looks.
                value &= ~SoftRst;
            }
            registers[offset] = value;
        }

        private static uint WithAcknowledges(uint mcr)
        {
            var disabled = (mcr & Mdis) != 0;
            var frozen = (mcr & Frz) != 0 && (mcr & Halt) != 0;

            mcr &= ~(LpmAck | FrzAck | NotRdy);
            if(disabled)
            {
                mcr |= LpmAck;
            }
            if(frozen)
            {
                mcr |= FrzAck;
            }
            // NOT_RDY reports "not taking part on the bus", which covers both.
            if(disabled || frozen)
            {
                mcr |= NotRdy;
            }
            return mcr;
        }

        private const long McrOffset = 0x00;

        private const uint LpmAck = 0x00100000;
        private const uint FrzAck = 0x01000000;
        private const uint SoftRst = 0x02000000;
        private const uint NotRdy = 0x08000000;
        private const uint Halt = 0x10000000;
        private const uint Frz = 0x40000000;
        private const uint Mdis = 0x80000000;

        private readonly IMachine machine;
        private readonly Dictionary<long, uint> registers;
    }
}
