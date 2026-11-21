// Register-driven STM32L4/G4 clock controller. It models the complete clock
// tree used by ArduPilot AP_Periph targets and applies changing bus clocks to
// Renode peripherals whose timing depends on them.
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
    public class AP_STM32L4G4_RCC : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32L4G4_RCC(
            IPeripheral nvic = null, IPeripheral dwt = null,
            IPeripheral usart1 = null, IPeripheral usart2 = null,
            IPeripheral usart3 = null, IPeripheral uart4 = null,
            IPeripheral uart5 = null, IPeripheral lpuart1 = null,
            IPeripheral timer1 = null, IPeripheral timer2 = null,
            IPeripheral timer3 = null, IPeripheral timer4 = null,
            IPeripheral timer5 = null, IPeripheral timer6 = null,
            IPeripheral timer7 = null, IPeripheral timer8 = null,
            IPeripheral timer15 = null, IPeripheral timer16 = null,
            IPeripheral timer17 = null, IPeripheral timer20 = null,
            uint hseFrequency = DefaultHseFrequency,
            uint lseFrequency = DefaultLseFrequency,
            bool isG4 = false)
        {
            this.nvic = nvic;
            this.dwt = dwt;
            uarts = new[] { usart1, usart2, usart3, uart4, uart5, lpuart1 };
            timers = new Dictionary<int, IPeripheral> {
                { 1, timer1 }, { 2, timer2 }, { 3, timer3 }, { 4, timer4 },
                { 5, timer5 }, { 6, timer6 }, { 7, timer7 }, { 8, timer8 },
                { 15, timer15 }, { 16, timer16 }, { 17, timer17 }, { 20, timer20 },
            };
            this.hseFrequency = hseFrequency;
            this.lseFrequency = lseFrequency;
            this.isG4 = isG4;
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

        public bool IsG4
        {
            get { return isG4; }
            set { isG4 = value; Reset(); }
        }

        public void Reset()
        {
            registers.Clear();
            if(isG4)
            {
                registers[CR] = HSION;
                registers[CFGR] = 1;
            }
            else
            {
                registers[CR] = MSION;
                registers[CSR] = 6u << 8;
            }
            registers[CSR] = Register(CSR) | BORRSTF | PINRSTF;
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
            case CRRCR: return value | ((value & HSI48ON) << 1);
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
            case CRRCR:
                registers[CRRCR] = value & ~HSI48RDY;
                break;
            case CICR:
                registers[CIFR] = Register(CIFR) & ~value;
                break;
            default:
                registers[offset] = value;
                break;
            }
            if(offset == CR || offset == ICSCR || offset == CFGR ||
                offset == PLLCFGR || offset == PLLSAI1CFGR || offset == PLLSAI2CFGR ||
                offset == CCIPR || offset == CCIPR2 || offset == BDCR ||
                offset == CSR || offset == CRRCR)
            {
                UpdateClocks();
            }
        }

        public ulong GetClockFrequency(string name)
        {
            if(String.IsNullOrEmpty(name)) return 0;
            switch(name.Replace("_", "").ToUpperInvariant())
            {
            case "MSI": return MsiClock;
            case "HSI": case "HSI16": return HsiClock;
            case "HSI48": return Hsi48Clock;
            case "HSE": return HseClock;
            case "LSE": return LseClock;
            case "LSI": return LsiClock;
            case "PLLP": return MainPllClock(PllOutput.P);
            case "PLLQ": return MainPllClock(PllOutput.Q);
            case "PLLR": case "PLL": return MainPllClock(PllOutput.R);
            case "PLLSAI1P": return AuxiliaryPllClock(PLLSAI1CFGR, PLLSAI1ON, PllOutput.P);
            case "PLLSAI1Q": return AuxiliaryPllClock(PLLSAI1CFGR, PLLSAI1ON, PllOutput.Q);
            case "PLLSAI1R": return AuxiliaryPllClock(PLLSAI1CFGR, PLLSAI1ON, PllOutput.R);
            case "PLLSAI2P": return AuxiliaryPllClock(PLLSAI2CFGR, PLLSAI2ON, PllOutput.P);
            case "PLLSAI2Q": return AuxiliaryPllClock(PLLSAI2CFGR, PLLSAI2ON, PllOutput.Q);
            case "PLLSAI2R": return AuxiliaryPllClock(PLLSAI2CFGR, PLLSAI2ON, PllOutput.R);
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
            case "LPUART1": return UartClock(5, PClock1);
            case "SPI1": case "SPI4": return PClock2;
            case "SPI2": case "SPI3": return PClock1;
            case "I2C1": return I2cClock(0);
            case "I2C2": return I2cClock(1);
            case "I2C3": return I2cClock(2);
            case "I2C4": return I2cClock(3);
            case "CAN": case "CAN1": return PClock1;
            case "FDCAN": case "FDCAN1": case "FDCAN2": return FdcanClock;
            case "ADC": case "ADC1": case "ADC2": return AdcClock(false);
            case "ADC3": case "ADC4": case "ADC5": return AdcClock(true);
            case "USB": case "RNG": case "CLK48": case "CLOCK48": return Clock48;
            case "QSPI": return QspiClock;
            case "SAI1": return SaiClock(false);
            case "SAI2": return SaiClock(true);
            case "I2S": case "I2S23": return I2sClock;
            case "LPTIM1": return LpTimerClock(0);
            case "LPTIM2": return LpTimerClock(1);
            case "SWPMI1": return isG4 ? 0 :
                    ((Register(CCIPR) & (1u << 30)) == 0 ? PClock1 : HsiClock);
            case "DFSDM": return isG4 ? 0 :
                    ((Register(CCIPR) & (1u << 31)) == 0 ? PClock2 : SystemClock);
            case "RTC": return RtcClock;
            case "MCO": return McoClock;
            default:
                this.Log(LogLevel.Warning, "Unknown STM32L4/G4 clock '{0}'", name);
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
                SetTimerFrequency(timer.Value, apb2 ? TimerClock2 : TimerClock1);
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

        private ulong MsiClock
        {
            get
            {
                if(isG4 || (Register(CR) & MSION) == 0) return 0;
                var range = (Register(CR) & MSIRGSEL) != 0 ?
                    Field(Register(CR), 4, 4) : Field(Register(CSR), 8, 4);
                var clocks = new ulong[] {
                    100000, 200000, 400000, 800000, 1000000, 2000000,
                    4000000, 8000000, 16000000, 24000000, 32000000, 48000000,
                };
                return range < clocks.Length ? clocks[range] : 0;
            }
        }

        private ulong HsiClock => (Register(CR) & HSION) != 0 ? HsiFrequency : 0;
        private ulong Hsi48Clock => (Register(CRRCR) & HSI48ON) != 0 ? Hsi48Frequency : 0;
        private ulong HseClock => (Register(CR) & HSEON) != 0 ? hseFrequency : 0;
        private ulong LseClock => (Register(BDCR) & LSEON) != 0 ? lseFrequency : 0;
        private ulong LsiClock => (Register(CSR) & LSION) != 0 ? LsiFrequency : 0;

        private ulong MainPllClock(PllOutput output)
        {
            if((Register(CR) & PLLON) == 0) return 0;
            return PllOutputClock(Register(PLLCFGR), output);
        }

        private ulong AuxiliaryPllClock(long configOffset, uint enable, PllOutput output)
        {
            if(isG4 || (Register(CR) & enable) == 0) return 0;
            return PllOutputClock(Register(configOffset), output);
        }

        private ulong PllOutputClock(uint config, PllOutput output)
        {
            var input = PllInputClock;
            if(input == 0) return 0;
            var m = Field(Register(PLLCFGR), 4, isG4 ? 4 : 3) + 1;
            var vco = input / m * Field(config, 8, 7);
            switch(output)
            {
            case PllOutput.P:
                if((config & (1u << 16)) == 0) return 0;
                var pdiv = Field(config, 27, 5);
                return vco / (pdiv == 0 ? ((config & (1u << 17)) == 0 ? 7UL : 17UL) : pdiv);
            case PllOutput.Q:
                return (config & (1u << 20)) == 0 ? 0 : vco / PllQrDivider(Field(config, 21, 2));
            default:
                return (config & (1u << 24)) == 0 ? 0 : vco / PllQrDivider(Field(config, 25, 2));
            }
        }

        private ulong PllInputClock
        {
            get
            {
                var source = Field(Register(PLLCFGR), 0, 2);
                if(isG4) return Select(source, 0, 0, HsiClock, HseClock);
                return Select(source, 0, MsiClock, HsiClock, HseClock);
            }
        }

        private ulong SystemClock
        {
            get
            {
                var selection = Field(Register(CFGR), 0, 2);
                if(isG4) return Select(selection, 0, HsiClock, HseClock, MainPllClock(PllOutput.R));
                return Select(selection, MsiClock, HsiClock, HseClock, MainPllClock(PllOutput.R));
            }
        }

        private ulong HClock => SystemClock / AhbDivider(Field(Register(CFGR), 4, 4));
        private ulong PClock1 => HClock / ApbDivider(Field(Register(CFGR), 8, 3));
        private ulong PClock2 => HClock / ApbDivider(Field(Register(CFGR), 11, 3));
        private ulong TimerClock1 => PClock1 * (Field(Register(CFGR), 8, 3) < 4 ? 1UL : 2UL);
        private ulong TimerClock2 => PClock2 * (Field(Register(CFGR), 11, 3) < 4 ? 1UL : 2UL);

        private ulong UartClock(int index, ulong pclk)
        {
            return Select(Field(Register(CCIPR), index * 2, 2), pclk, SystemClock, HsiClock, LseClock);
        }

        private ulong I2cClock(int index)
        {
            var selection = index == 3 ? Field(Register(CCIPR2), 0, 2) :
                Field(Register(CCIPR), 12 + index * 2, 2);
            return Select(selection, PClock1, SystemClock, HsiClock, 0);
        }

        private ulong FdcanClock
        {
            get
            {
                if(!isG4) return PClock1;
                return Select(Field(Register(CCIPR), 24, 2), HseClock,
                    MainPllClock(PllOutput.Q), PClock1, 0);
            }
        }

        private ulong Clock48
        {
            get
            {
                var selection = Field(Register(CCIPR), 26, 2);
                if(isG4) return Select(selection, Hsi48Clock, 0, MainPllClock(PllOutput.Q), 0);
                return Select(selection, Hsi48Clock,
                    AuxiliaryPllClock(PLLSAI1CFGR, PLLSAI1ON, PllOutput.Q),
                    MainPllClock(PllOutput.Q), MsiClock);
            }
        }

        private ulong AdcClock(bool secondGroup)
        {
            var selection = Field(Register(CCIPR), isG4 && secondGroup ? 30 : 28, 2);
            if(isG4) return Select(selection, 0, MainPllClock(PllOutput.P), SystemClock, 0);
            return Select(selection, 0,
                AuxiliaryPllClock(PLLSAI1CFGR, PLLSAI1ON, PllOutput.R),
                AuxiliaryPllClock(PLLSAI2CFGR, PLLSAI2ON, PllOutput.R), SystemClock);
        }

        private ulong SaiClock(bool second)
        {
            if(isG4) return Select(Field(Register(CCIPR), 20, 2), SystemClock,
                MainPllClock(PllOutput.Q), 0, HsiClock);
            return Select(Field(Register(CCIPR), second ? 24 : 22, 2),
                AuxiliaryPllClock(PLLSAI1CFGR, PLLSAI1ON, PllOutput.P),
                AuxiliaryPllClock(PLLSAI2CFGR, PLLSAI2ON, PllOutput.P),
                MainPllClock(PllOutput.P), 0);
        }

        private ulong I2sClock => isG4 ? Select(Field(Register(CCIPR), 22, 2),
            SystemClock, MainPllClock(PllOutput.Q), 0, HsiClock) : 0;

        private ulong QspiClock => isG4 ? Select(Field(Register(CCIPR2), 20, 2),
            SystemClock, HsiClock, MainPllClock(PllOutput.Q), 0) : HClock;

        private ulong LpTimerClock(int index)
        {
            if(isG4 && index != 0) return 0;
            return Select(Field(Register(CCIPR), 18 + index * 2, 2),
                PClock1, LsiClock, HsiClock, LseClock);
        }

        private ulong RtcClock => Select(Field(Register(BDCR), 8, 2),
            0, LseClock, LsiClock, HseClock / 32);

        private ulong McoClock
        {
            get
            {
                var clock = Select(Field(Register(CFGR), 24, 4), 0, SystemClock,
                    MsiClock, HsiClock, HseClock, MainPllClock(PllOutput.R),
                    LsiClock, LseClock, Hsi48Clock);
                var prescaler = Field(Register(CFGR), 28, 3);
                return clock / (prescaler == 0 ? 1UL : 1UL << (int)prescaler);
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

        private static uint PllQrDivider(uint encoding)
        {
            return 2u * (encoding + 1);
        }

        private static ulong Select(uint selection, params ulong[] clocks)
        {
            return selection < clocks.Length ? clocks[selection] : 0;
        }

        private uint ReadyBits(uint cr)
        {
            uint ready = 0;
            if(!isG4 && (cr & MSION) != 0) ready |= MSIRDY;
            if((cr & HSION) != 0) ready |= HSIRDY;
            if((cr & HSEON) != 0) ready |= HSERDY;
            if((cr & PLLON) != 0) ready |= PLLRDY;
            if(!isG4 && (cr & PLLSAI1ON) != 0) ready |= PLLSAI1RDY;
            if(!isG4 && (cr & PLLSAI2ON) != 0) ready |= PLLSAI2RDY;
            return ready;
        }

        private enum PllOutput
        {
            P,
            Q,
            R,
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
        private bool isG4;

        private const long CR = 0x00;
        private const long ICSCR = 0x04;
        private const long CFGR = 0x08;
        private const long PLLCFGR = 0x0C;
        private const long PLLSAI1CFGR = 0x10;
        private const long PLLSAI2CFGR = 0x14;
        private const long CIFR = 0x1C;
        private const long CICR = 0x20;
        private const long CCIPR = 0x88;
        private const long BDCR = 0x90;
        private const long CSR = 0x94;
        private const long CRRCR = 0x98;
        private const long CCIPR2 = 0x9C;
        private const uint MSION = 1u << 0;
        private const uint MSIRDY = 1u << 1;
        private const uint MSIRGSEL = 1u << 3;
        private const uint HSION = 1u << 8;
        private const uint HSIRDY = 1u << 10;
        private const uint HSEON = 1u << 16;
        private const uint HSERDY = 1u << 17;
        private const uint PLLON = 1u << 24;
        private const uint PLLRDY = 1u << 25;
        private const uint PLLSAI1ON = 1u << 26;
        private const uint PLLSAI1RDY = 1u << 27;
        private const uint PLLSAI2ON = 1u << 28;
        private const uint PLLSAI2RDY = 1u << 29;
        private const uint LSEON = 1u << 0;
        private const uint LSERDY = 1u << 1;
        private const uint LSION = 1u << 0;
        private const uint LSIRDY = 1u << 1;
        private const uint HSI48ON = 1u << 0;
        private const uint HSI48RDY = 1u << 1;
        private const uint RMVF = 1u << 23;
        private const uint SW_MASK = 0x3;
        private const uint SWS_MASK = 0xC;
        private const uint CR_READ_ONLY_MASK = MSIRDY | HSIRDY | HSERDY | PLLRDY |
            PLLSAI1RDY | PLLSAI2RDY;
        private const uint BORRSTF = 1u << 27;
        private const uint PINRSTF = 1u << 26;
        private const uint RESET_FLAGS = 0xFF000000;
        private const ulong HsiFrequency = 16000000;
        private const ulong Hsi48Frequency = 48000000;
        private const ulong LsiFrequency = 32000;
        private const uint DefaultHseFrequency = 8000000;
        private const uint DefaultLseFrequency = 32768;
    }
}
