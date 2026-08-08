// Minimal STM32 OTG register window for boards that use USB only as a
// console. It lets the ChibiOS OTGv1 core reset complete while leaving the
// link disconnected; board testing uses a hardware UART socket instead.
//
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class AP_STM32_OTG_Stub : IDoubleWordPeripheral, IKnownSize
    {
        public void Reset()
        {
        }

        public uint ReadDoubleWord(long offset)
        {
            // ChibiOS waits for AHBIDL before and after requesting CSRST.
            // Reset and FIFO-flush request bits are self-clearing.
            return offset == ResetControl ? AhbIdle : 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public long Size => 0x40000;

        private const long ResetControl = 0x10;
        private const uint AhbIdle = 1U << 31;
    }
}
