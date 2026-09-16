//
// Minimal STM32H7 PWR for ChibiOS clock init. Renode has no H7 PWR
// model (the stock platform fakes it with bus Tags). ChibiOS
// init_pwr() writes CR3 (LDO supply config) and spins unbounded on
// CSR1.ACTVOSRDY, then programs D3CR.VOS (plus SYSCFG PWRCR ODEN for
// 480MHz overdrive, unpolled) and spins on D3CR.VOSRDY. Everything
// here is plain storage with those two ready bits always reading set.
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
        public long Size => 0x400;

        public void Reset()
        {
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
            case CSR1: return ACTVOSRDY;
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
            case CR1: cr1 = value; return;
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
    }
}
