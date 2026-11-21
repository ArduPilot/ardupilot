// Register-driven STM32F1/F3 clock controller.  It covers the F103/F105 and
// F303 layouts used by ArduPilot AP_Periph targets, including the F105 PLL2
// predivider path and the F303 peripheral clock selections.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32F1F3_RCC : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32F1F3_RCC(
            IPeripheral nvic = null, IPeripheral dwt = null,
            IPeripheral usart1 = null, IPeripheral usart2 = null,
            IPeripheral usart3 = null, IPeripheral uart4 = null,
            IPeripheral uart5 = null,
            IPeripheral timer1 = null, IPeripheral timer2 = null,
            IPeripheral timer3 = null, IPeripheral timer4 = null,
            IPeripheral timer6 = null, IPeripheral timer7 = null,
            IPeripheral timer8 = null, IPeripheral timer15 = null,
            IPeripheral timer16 = null, IPeripheral timer17 = null,
            IPeripheral timer20 = null,
            uint hseFrequency = DefaultHseFrequency,
            uint lseFrequency = DefaultLseFrequency,
            bool isF3 = false, bool isF105 = false)
        {
            this.nvic = nvic;
            this.dwt = dwt;
            uarts = new[] { usart1, usart2, usart3, uart4, uart5 };
            timers = new Dictionary<int, IPeripheral> {
                { 1, timer1 }, { 2, timer2 }, { 3, timer3 }, { 4, timer4 },
                { 6, timer6 }, { 7, timer7 }, { 8, timer8 }, { 15, timer15 },
                { 16, timer16 }, { 17, timer17 }, { 20, timer20 },
            };
            this.hseFrequency = hseFrequency;
            this.lseFrequency = lseFrequency;
            this.isF3 = isF3;
            this.isF105 = isF105;
            registers = new Dictionary<long, uint>();
            appliedFrequencies = new Dictionary<IPeripheral, ulong>();
            frequencyErrors = new HashSet<Type>();
            Reset();
        }

        public long Size => 0x400;

        public uint HseFrequency
        {
            get { return hseFrequency; }
            set { hseFrequency = value; UpdateClocks(); }
        }

        public bool IsF3
        {
            get { return isF3; }
            set { isF3 = value; UpdateClocks(); }
        }

        public bool IsF105
        {
            get { return isF105; }
            set { isF105 = value; UpdateClocks(); }
        }

        public void Reset()
        {
            registers.Clear();
            registers[CR] = 0x83;
            registers[CSR] = PORRSTF | PINRSTF | BORRSTF;
            appliedFrequencies.Clear();
            UpdateClocks();
        }

        public uint ReadDoubleWord(long offset)
        {
            var value = Register(offset);
            switch(offset)
            {
            case CR: return value | ReadyBits(value);
            case CFGR: return (value & ~SWS_MASK) | ((value & SW_MASK) << 2);
            case BDCR: return value | ((value & LSEON) << 1);
            case CSR: return value | ((value & LSION) << 1);
            default: return value;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case CR:
                registers[CR] = value & ~CR_READ_ONLY_MASK;
                break;
            case CFGR:
                registers[CFGR] = value & ~SWS_MASK;
                break;
            case BDCR:
                registers[BDCR] = value & ~LSERDY;
                break;
            case CSR:
                if((value & RMVF) != 0) value &= ~RESET_FLAGS;
                registers[CSR] = value & ~(LSIRDY | RMVF);
                break;
            case CIR:
                registers[CIR] = value & 0x000000FF;
                break;
            default:
                registers[offset] = value;
                break;
            }
            if(offset == CR || offset == CFGR || offset == CFGR2 ||
                offset == CFGR3 || offset == BDCR || offset == CSR)
            {
                UpdateClocks();
            }
        }

        public ulong GetClockFrequency(string name)
        {
            if(String.IsNullOrEmpty(name)) return 0;
            switch(name.Replace("_", "").ToUpperInvariant())
            {
            case "HSI": return HsiClock;
            case "HSE": return HseClock;
            case "LSE": return LseClock;
            case "LSI": return LsiClock;
            case "PLL": return PllClock;
            case "PLL2": return Pll2Clock;
            case "PLL3": return Pll3Clock;
            case "SYS": case "SYSCLK": return SystemClock;
            case "HCLK": return HClock;
            case "PCLK1": return PClock1;
            case "PCLK2": return PClock2;
            case "TIMCLK1": return TimerClock1;
            case "TIMCLK2": return TimerClock2;
            case "USART1": return UartClock(0, PClock2);
            case "USART2": return UartClock(1, PClock1);
            case "USART3": return UartClock(2, PClock1);
            case "UART4": return UartClock(3, PClock1);
            case "UART5": return UartClock(4, PClock1);
            case "SPI1": return PClock2;
            case "SPI2": case "SPI3": return PClock1;
            case "I2C1": return I2cClock(0);
            case "I2C2": return I2cClock(1);
            case "I2C3": return SystemClock;
            case "CAN": case "CAN1": case "CAN2": return PClock1;
            case "ADC": case "ADC1": case "ADC2": return Adc12Clock;
            case "ADC3": case "ADC4": return Adc34Clock;
            case "USB": return (Register(CFGR) & (1u << 22)) == 0 ? PllClock * 2 / 3 : PllClock;
            case "RTC": return RtcClock;
            case "MCO": return McoClock;
            default:
                this.Log(LogLevel.Warning, "Unknown STM32F1/F3 clock '{0}'", name);
                return 0;
            }
        }

        private void UpdateClocks()
        {
            SetPeripheralFrequency(nvic, HClock);
            SetPeripheralFrequency(dwt, HClock);
            for(var i = 0; i < uarts.Length; i++)
            {
                SetPeripheralFrequency(uarts[i], UartClock(i, i == 0 ? PClock2 : PClock1));
            }
            foreach(var timer in timers)
            {
                var apb2 = timer.Key == 1 || timer.Key == 8 || timer.Key == 15 ||
                    timer.Key == 16 || timer.Key == 17 || timer.Key == 20;
                var frequency = apb2 ? TimerClock2 : TimerClock1;
                if(isF3 && timer.Key == 1 && (Register(CFGR3) & (1u << 8)) != 0) frequency = PllClock * 2;
                if(isF3 && timer.Key == 8 && (Register(CFGR3) & (1u << 9)) != 0) frequency = PllClock * 2;
                SetTimerFrequency(timer.Value, frequency);
            }
        }

        private void SetTimerFrequency(IPeripheral peripheral, ulong frequency)
        {
            if(!SetPeripheralFrequency(peripheral, frequency) || peripheral == null) return;
            var field = peripheral.GetType().GetField(
                "ccTimers", BindingFlags.NonPublic | BindingFlags.Instance);
            var compareTimers = field == null ? null : field.GetValue(peripheral) as IEnumerable;
            if(compareTimers == null) return;
            foreach(var timer in compareTimers)
            {
                var target = timer as IHasFrequency;
                if(target != null) target.Frequency = frequency;
            }
        }

        private bool SetPeripheralFrequency(IPeripheral peripheral, ulong frequency)
        {
            if(peripheral == null || frequency == 0) return false;
            ulong previous;
            if(appliedFrequencies.TryGetValue(peripheral, out previous) && previous == frequency) return false;
            try
            {
                var target = peripheral as IHasFrequency;
                if(target != null) target.Frequency = frequency;
                else
                {
                    var field = peripheral.GetType().GetField(
                        "frequency", BindingFlags.NonPublic | BindingFlags.Instance);
                    if(field == null) throw new MissingFieldException(peripheral.GetType().Name, "frequency");
                    field.SetValue(peripheral, Convert.ChangeType(frequency, field.FieldType));
                }
                appliedFrequencies[peripheral] = frequency;
                return true;
            }
            catch(Exception e)
            {
                if(frequencyErrors.Add(peripheral.GetType()))
                {
                    this.Log(LogLevel.Error, "Could not apply {0}Hz clock to {1}: {2}",
                        frequency, peripheral.GetType().Name, e.Message);
                }
                return false;
            }
        }

        private ulong HsiClock => (Register(CR) & HSION) != 0 ? HsiFrequency : 0;
        private ulong HseClock => (Register(CR) & HSEON) != 0 ? hseFrequency : 0;
        private ulong LseClock => (Register(BDCR) & LSEON) != 0 ? lseFrequency : 0;
        private ulong LsiClock => (Register(CSR) & LSION) != 0 ? LsiFrequency : 0;

        private ulong PllClock
        {
            get
            {
                if((Register(CR) & PLLON) == 0) return 0;
                var config = Register(CFGR);
                ulong input;
                if((config & PLLSRC) == 0) input = HsiClock / 2;
                else if(isF105)
                {
                    var predivSource = (Register(CFGR2) & (1u << 16)) == 0 ? HseClock : Pll2Clock;
                    input = predivSource / (Field(Register(CFGR2), 0, 4) + 1);
                }
                else if(isF3) input = HseClock / (Field(Register(CFGR2), 0, 4) + 1);
                else input = HseClock / (((config & (1u << 17)) == 0) ? 1UL : 2UL);
                return input * PllMultiplier(Field(config, 18, 4));
            }
        }

        private ulong Pll2Clock => AuxiliaryPllClock(PLL2ON, 8);
        private ulong Pll3Clock => AuxiliaryPllClock(PLL3ON, 12);

        private ulong AuxiliaryPllClock(uint enable, int multiplierShift)
        {
            if(!isF105 || (Register(CR) & enable) == 0) return 0;
            var input = HseClock / (Field(Register(CFGR2), 4, 4) + 1);
            return input * AuxiliaryPllMultiplier(Field(Register(CFGR2), multiplierShift, 4));
        }

        private ulong SystemClock => Select(Field(Register(CFGR), 0, 2), HsiClock, HseClock, PllClock);
        private ulong HClock => SystemClock / AhbDivider(Field(Register(CFGR), 4, 4));
        private ulong PClock1 => HClock / ApbDivider(Field(Register(CFGR), 8, 3));
        private ulong PClock2 => HClock / ApbDivider(Field(Register(CFGR), 11, 3));
        private ulong TimerClock1 => PClock1 * (Field(Register(CFGR), 8, 3) < 4 ? 1UL : 2UL);
        private ulong TimerClock2 => PClock2 * (Field(Register(CFGR), 11, 3) < 4 ? 1UL : 2UL);

        private ulong UartClock(int index, ulong pclk)
        {
            if(!isF3) return pclk;
            var shift = index == 0 ? 0 : 14 + index * 2;
            return Select(Field(Register(CFGR3), shift, 2), pclk, SystemClock, LseClock, HsiClock);
        }

        private ulong I2cClock(int index)
        {
            if(!isF3 || index > 1) return PClock1;
            return (Register(CFGR3) & (1u << (4 + index))) == 0 ? HsiClock : SystemClock;
        }

        private ulong Adc12Clock => isF3 ? F3AdcClock(Field(Register(CFGR2), 4, 5)) :
            PClock2 / (2 * (Field(Register(CFGR), 14, 2) + 1));
        private ulong Adc34Clock => isF3 ? F3AdcClock(Field(Register(CFGR2), 9, 5)) : 0;

        private ulong F3AdcClock(uint encoding)
        {
            if(encoding < 16) return 0;
            var divisors = new uint[] { 1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256 };
            var index = encoding - 16;
            return index < divisors.Length ? PllClock / divisors[index] : 0;
        }

        private ulong RtcClock => Select(Field(Register(BDCR), 8, 2), 0,
            LseClock, LsiClock, HseClock / (isF3 ? 32UL : 128UL));

        private ulong McoClock
        {
            get
            {
                return Select(Field(Register(CFGR), 24, isF105 ? 4 : 3), 0, 0,
                    LsiClock, LseClock, SystemClock, HsiClock, HseClock, PllClock / 2,
                    Pll2Clock, Pll3Clock / 2, 0, Pll3Clock);
            }
        }

        private uint Register(long offset)
        {
            uint value;
            registers.TryGetValue(offset, out value);
            return value;
        }

        private static uint Field(uint value, int shift, int width)
        {
            return (value >> shift) & ((1u << width) - 1);
        }

        private static uint PllMultiplier(uint encoding)
        {
            return encoding == 15 ? 16u : encoding + 2;
        }

        private static uint AuxiliaryPllMultiplier(uint encoding)
        {
            if(encoding == 14) return 16;
            if(encoding == 15) return 20;
            return encoding + 2;
        }

        private static uint AhbDivider(uint encoding)
        {
            if(encoding < 8) return 1;
            switch(encoding)
            {
            case 8: return 2; case 9: return 4; case 10: return 8; case 11: return 16;
            case 12: return 64; case 13: return 128; case 14: return 256; default: return 512;
            }
        }

        private static uint ApbDivider(uint encoding)
        {
            return encoding < 4 ? 1u : 1u << ((int)encoding - 3);
        }

        private static ulong Select(uint selection, params ulong[] clocks)
        {
            return selection < clocks.Length ? clocks[selection] : 0;
        }

        private static uint ReadyBits(uint cr)
        {
            uint ready = 0;
            if((cr & HSION) != 0) ready |= HSIRDY;
            if((cr & HSEON) != 0) ready |= HSERDY;
            if((cr & PLLON) != 0) ready |= PLLRDY;
            if((cr & PLL2ON) != 0) ready |= PLL2RDY;
            if((cr & PLL3ON) != 0) ready |= PLL3RDY;
            return ready;
        }

        private readonly IPeripheral nvic;
        private readonly IPeripheral dwt;
        private readonly IPeripheral[] uarts;
        private readonly Dictionary<int, IPeripheral> timers;
        private readonly Dictionary<long, uint> registers;
        private readonly Dictionary<IPeripheral, ulong> appliedFrequencies;
        private readonly HashSet<Type> frequencyErrors;
        private uint hseFrequency;
        private readonly uint lseFrequency;
        private bool isF3;
        private bool isF105;

        private const long CR = 0x00;
        private const long CFGR = 0x04;
        private const long CIR = 0x08;
        private const long BDCR = 0x20;
        private const long CSR = 0x24;
        private const long CFGR2 = 0x2C;
        private const long CFGR3 = 0x30;
        private const uint HSION = 1u << 0;
        private const uint HSIRDY = 1u << 1;
        private const uint HSEON = 1u << 16;
        private const uint HSERDY = 1u << 17;
        private const uint PLLON = 1u << 24;
        private const uint PLLRDY = 1u << 25;
        private const uint PLL2ON = 1u << 26;
        private const uint PLL2RDY = 1u << 27;
        private const uint PLL3ON = 1u << 28;
        private const uint PLL3RDY = 1u << 29;
        private const uint PLLSRC = 1u << 16;
        private const uint LSEON = 1u << 0;
        private const uint LSERDY = 1u << 1;
        private const uint LSION = 1u << 0;
        private const uint LSIRDY = 1u << 1;
        private const uint RMVF = 1u << 24;
        private const uint SW_MASK = 0x3;
        private const uint SWS_MASK = 0xC;
        private const uint CR_READ_ONLY_MASK = HSIRDY | HSERDY | PLLRDY | PLL2RDY | PLL3RDY;
        private const uint PORRSTF = 1u << 27;
        private const uint PINRSTF = 1u << 26;
        private const uint BORRSTF = 1u << 25;
        private const uint RESET_FLAGS = 0xFE000000;
        private const ulong HsiFrequency = 8000000;
        private const ulong LsiFrequency = 40000;
        private const uint DefaultHseFrequency = 8000000;
        private const uint DefaultLseFrequency = 32768;
    }
}
