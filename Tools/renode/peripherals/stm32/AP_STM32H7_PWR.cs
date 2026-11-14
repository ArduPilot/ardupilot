//
// Minimal STM32H7 PWR for ChibiOS clock init. Renode has no H7 PWR
// model (the stock platform fakes it with bus Tags). ChibiOS
// init_pwr() writes CR3 (LDO supply config) and spins unbounded on
// CSR1.ACTVOSRDY, then programs D3CR.VOS (plus SYSCFG PWRCR ODEN for
// 480MHz overdrive, unpolled) and spins on D3CR.VOSRDY. Everything
// here is plain storage with those two ready bits always reading set,
// apart from the programmable voltage detector (PVD) on VDD:
//
//   pwr Vdd 2.5        # hold VDD at 2.5V
//   pwr Dip 2.5 0.2    # 2.5V for 0.2ms of virtual time, then back
//
// IRQ is the PVD output, for EXTI line 16. PLS=7 selects the PVD_IN
// pin, which isn't modelled, so it never trips.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32H7_PWR : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32H7_PWR(IMachine machine)
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
            detector.Reset();
            cr1 = 0;
            cr2 = 0;
            cr3 = 0x6;   // reset value: LDOEN | SCUEN
            cpucr = 0;
            d3cr = 0;
            wkupepr = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case CR1: return cr1;
            case CSR1: return ACTVOSRDY | (detector.Below ? PVDO : 0);
            case CR2: return cr2;
            case CR3: return cr3;
            case CPUCR: return cpucr;
            case D3CR: return d3cr | VOSRDY;
            case WKUPFR: return 0;
            case WKUPEPR: return wkupepr;
            default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case CR1:
                cr1 = value;
                detector.Configure((cr1 & PVDEN) != 0, (int)((cr1 >> PLS_SHIFT) & 7));
                return;
            case CR2: cr2 = value; return;
            case CR3: cr3 = value; return;
            case CPUCR: cpucr = value; return;
            case D3CR: d3cr = value; return;
            case WKUPCR: return; // write-to-clear wakeup flags
            case WKUPEPR: wkupepr = value; return;
            default: return;
            }
        }

        private uint cr1, cr2, cr3, cpucr, d3cr, wkupepr;
        private readonly AP_VoltageDetector detector;

        private const long CR1 = 0x00;
        private const long CSR1 = 0x04;
        private const long CR2 = 0x08;
        private const long CR3 = 0x0C;
        private const long CPUCR = 0x10;
        private const long D3CR = 0x18;
        private const long WKUPCR = 0x20;
        private const long WKUPFR = 0x24;
        private const long WKUPEPR = 0x28;

        private const uint ACTVOSRDY = 1u << 13;
        private const uint VOSRDY = 1u << 13;
        private const uint PVDEN = 1u << 4;
        private const int PLS_SHIFT = 5;
        private const uint PVDO = 1u << 4;

        // STM32H743 datasheet Table 27, typical falling-edge thresholds
        // for PLS 0-6; the rising edge is 0.1V higher
        private static readonly double[] FallingThresholds = {
            1.86, 2.01, 2.15, 2.31, 2.45, 2.61, 2.76
        };
    }
}
