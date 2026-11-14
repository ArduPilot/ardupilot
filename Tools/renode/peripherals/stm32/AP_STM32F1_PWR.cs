//
// STM32F1 power controller: CR and CSR storage plus the programmable
// voltage detector (PVD). Set the supply from the monitor to exercise it:
//
//   pwr Vdd 2.5        # hold VDD at 2.5V
//   pwr Dip 2.5 0.2    # 2.5V for 0.2ms of virtual time, then back
//
// IRQ is the PVD output, for EXTI line 16; it is high while the PVD is
// enabled and VDD is below the PLS threshold.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32F1_PWR : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32F1_PWR(IMachine machine)
        {
            detector = new AP_VoltageDetector(machine, this, FallingThresholds);
            Reset();
        }

        public long Size => 0x400;

        public GPIO IRQ => detector.Output;

        public double Vdd
        {
            get => detector.Vdd;
            set => detector.Vdd = value;
        }

        public ulong PvdTrips => detector.Trips;

        public void Dip(double volts, double milliseconds)
        {
            detector.Dip(volts, milliseconds);
        }

        public void Reset()
        {
            cr = 0;
            csr = 0;
            detector.Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case CR: return cr;
            case CSR: return csr | (detector.Below ? PVDO : 0);
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case CR:
                // CWUF and CSBF clear status flags and read as zero
                cr = value & ~(CWUF | CSBF);
                detector.Configure((cr & PVDE) != 0, (int)((cr >> PLS_SHIFT) & 7));
                return;
            case CSR:
                csr = value & EWUP;
                return;
            }
        }

        private uint cr;
        private uint csr;
        private readonly AP_VoltageDetector detector;

        private const long CR = 0x00;
        private const long CSR = 0x04;

        private const uint CWUF = 1u << 2;
        private const uint CSBF = 1u << 3;
        private const uint PVDE = 1u << 4;
        private const int PLS_SHIFT = 5;
        private const uint PVDO = 1u << 2;
        private const uint EWUP = 1u << 8;

        // STM32F103xx datasheet Table 11, typical falling-edge thresholds
        // for PLS 0-7; the rising edge is 0.1V higher
        private static readonly double[] FallingThresholds = {
            2.08, 2.18, 2.28, 2.38, 2.48, 2.58, 2.68, 2.78
        };
    }
}
