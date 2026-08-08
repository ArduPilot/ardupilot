//
// Minimal STM32H7 RCC for ChibiOS. The stock STM32H7_RCC model lacks
// most of the register map (no AHB/APB enable/reset registers except
// AHB4ENR, no D1/D2/D3 domain prescaler registers, no kernel clock
// selection registers), and ChibiOS both read-modify-writes those and
// polls several ready bits unbounded. This model is a plain-storage
// register file with the ON->RDY mirrors ChibiOS spins on:
//
//   CR: HSION->HSIRDY, HSEON->HSERDY, CSION->CSIRDY, HSI48ON->HSI48RDY,
//       PLL1/2/3ON->PLL1/2/3RDY (polled together)
//   CFGR: SW (bits 0-2) mirrored into SWS (bits 3-5) - HSI first,
//       PLL1_P after the switch
//
// Everything else (PLLCKSELR/PLLCFGR/DIVRs/FRACRs, D1/D2/D3CFGR, the
// CCIPR kernel-clock selects, every RSTR/ENR/LPENR including the
// SRAM1-3 enables in AHB2ENR the app performs itself, BDCR, CSR, RSR)
// is stored and read back verbatim. hal_lld_init mass-writes the RSTR
// registers after clock init; nothing here acts on any of it because
// no Renode model checks a clock gate.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32H7_RCC : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32H7_RCC()
        {
            registers = new Dictionary<long, uint>();
            Reset();
        }

        public long Size => 0x400;

        public void Reset()
        {
            registers.Clear();
            registers[CR] = HSION; // HSI on out of reset
            // RSR: power-on reset flags set, as after a cold boot
            registers[RSR] = PORRSTF | PINRSTF | BORRSTF;
        }

        public uint ReadDoubleWord(long offset)
        {
            uint value;
            registers.TryGetValue(offset, out value);
            switch(offset)
            {
            case CR:
                return value | ReadyBits(value);
            case CFGR:
                // SWS mirrors SW
                return (value & ~SWS_MASK) | ((value & SW_MASK) << 3);
            default:
                return value;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case CSR:
                // LSION -> LSIRDY mirror lives in CSR on the H7
                registers[CSR] = value | ((value & 1u) << 1);
                return;
            case RSR:
                // RMVF (bit 16) clears the reset flags
                if((value & (1u << 16)) != 0)
                {
                    registers[RSR] = 0;
                }
                return;
            default:
                registers[offset] = value;
                return;
            }
        }

        private static uint ReadyBits(uint cr)
        {
            uint ready = 0;
            if((cr & HSION) != 0)   ready |= HSIRDY;
            if((cr & CSION) != 0)   ready |= CSIRDY;
            if((cr & HSI48ON) != 0) ready |= HSI48RDY;
            if((cr & HSEON) != 0)   ready |= HSERDY;
            if((cr & PLL1ON) != 0)  ready |= PLL1RDY;
            if((cr & PLL2ON) != 0)  ready |= PLL2RDY;
            if((cr & PLL3ON) != 0)  ready |= PLL3RDY;
            return ready;
        }

        private readonly Dictionary<long, uint> registers;

        private const long CR = 0x00;
        private const long CFGR = 0x10;
        private const long RSR = 0xD0;
        private const long CSR = 0x74;

        private const uint HSION = 1u << 0;
        private const uint HSIRDY = 1u << 2;
        private const uint HSI48ON = 1u << 12;
        private const uint HSI48RDY = 1u << 13;
        private const uint CSION = 1u << 7;
        private const uint CSIRDY = 1u << 8;
        private const uint HSEON = 1u << 16;
        private const uint HSERDY = 1u << 17;
        private const uint PLL1ON = 1u << 24;
        private const uint PLL1RDY = 1u << 25;
        private const uint PLL2ON = 1u << 26;
        private const uint PLL2RDY = 1u << 27;
        private const uint PLL3ON = 1u << 28;
        private const uint PLL3RDY = 1u << 29;

        private const uint SW_MASK = 0x7;
        private const uint SWS_MASK = 0x7 << 3;

        private const uint PORRSTF = 1u << 23;
        private const uint PINRSTF = 1u << 22;
        private const uint BORRSTF = 1u << 21;
    }
}
