// STM32F4/F7 reset and clock controller.  The two families share the main
// RCC register layout; F7 adds peripheral muxes in DCKCFGR2.  ChibiOS writes
// the complete tree at boot, and this model propagates the resulting clocks
// into timing-aware Renode peripherals.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AP_STM32F4_RCC : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32F4_RCC(
            IMachine machine, IPeripheral rtcPeripheral = null,
            IPeripheral nvic = null, IPeripheral dwt = null,
            IPeripheral usart1 = null, IPeripheral usart2 = null,
            IPeripheral usart3 = null, IPeripheral uart4 = null,
            IPeripheral uart5 = null, IPeripheral usart6 = null,
            IPeripheral uart7 = null, IPeripheral uart8 = null,
            IPeripheral timer1 = null, IPeripheral timer2 = null,
            IPeripheral timer3 = null, IPeripheral timer4 = null,
            IPeripheral timer5 = null, IPeripheral timer6 = null,
            IPeripheral timer7 = null, IPeripheral timer8 = null,
            IPeripheral timer9 = null, IPeripheral timer10 = null,
            IPeripheral timer11 = null, IPeripheral timer12 = null,
            IPeripheral timer13 = null, IPeripheral timer14 = null,
            uint hseFrequency = DefaultHseFrequency,
            uint lseFrequency = DefaultLseFrequency, bool isF7 = false)
        {
            this.machine = machine;
            this.rtcPeripheral = rtcPeripheral;
            this.nvic = nvic;
            this.dwt = dwt;
            uarts = new[] { usart1, usart2, usart3, uart4, uart5, usart6, uart7, uart8 };
            timers = new[] {
                timer1, timer2, timer3, timer4, timer5, timer6, timer7,
                timer8, timer9, timer10, timer11, timer12, timer13, timer14,
            };
            this.hseFrequency = hseFrequency;
            this.lseFrequency = lseFrequency;
            this.isF7 = isF7;
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

        public uint LseFrequency
        {
            get { return lseFrequency; }
            set { lseFrequency = value; UpdateClocks(); }
        }

        public bool IsF7
        {
            get { return isF7; }
            set { isF7 = value; UpdateClocks(); }
        }

        public IPeripheral Dwt { get { return dwt; } set { dwt = value; UpdateClocks(); } }
        public IPeripheral Uart4 { get { return uarts[3]; } set { uarts[3] = value; UpdateClocks(); } }
        public IPeripheral Uart5 { get { return uarts[4]; } set { uarts[4] = value; UpdateClocks(); } }
        public IPeripheral Uart7 { get { return uarts[6]; } set { uarts[6] = value; UpdateClocks(); } }
        public IPeripheral Uart8 { get { return uarts[7]; } set { uarts[7] = value; UpdateClocks(); } }
        public IPeripheral Timer12 { get { return timers[11]; } set { timers[11] = value; UpdateClocks(); } }

        public void Reset()
        {
            registers.Clear();
            registers[CR] = 0x83;
            registers[PLLCFGR] = 0x24003010;
            registers[PLLI2SCFGR] = 0x20003000;
            registers[PLLSAICFGR] = 0x24003000;
            registers[CSR] = PORRSTF | PINRSTF | BORRSTF;
            appliedFrequencies.Clear();
            UpdateClocks();
        }

        public uint ReadDoubleWord(long offset)
        {
            var value = Register(offset);
            switch(offset)
            {
            case CR:
                return value | ReadyBits(value);
            case CFGR:
                return (value & ~SWS_MASK) | ((value & SW_MASK) << 2);
            case BDCR:
                return value | ((value & LSEON) << 1);
            case CSR:
                return value | ((value & LSION) << 1);
            default:
                return value;
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
                if(rtcPeripheral != null)
                {
                    if((value & RTCEN) != 0) machine.SystemBus.EnablePeripheral(rtcPeripheral);
                    else machine.SystemBus.DisablePeripheral(rtcPeripheral);
                }
                break;
            case CSR:
                if((value & RMVF) != 0)
                {
                    value &= ~RESET_FLAGS;
                }
                registers[CSR] = value & ~(LSIRDY | RMVF);
                break;
            case CIR:
                // Oscillators stabilize immediately, so no clock interrupt is pending.
                registers[CIR] = value & 0x000000FF;
                break;
            default:
                registers[offset] = value;
                break;
            }
            if(IsClockConfigurationRegister(offset))
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
            case "PLLP": return PllOutput(PLLCFGR, PllOutputKind.P);
            case "PLLQ": return PllOutput(PLLCFGR, PllOutputKind.Q);
            case "PLLR": return PllOutput(PLLCFGR, PllOutputKind.R);
            case "PLLI2SP": return PllOutput(PLLI2SCFGR, PllOutputKind.P);
            case "PLLI2SQ": return PllOutput(PLLI2SCFGR, PllOutputKind.Q);
            case "PLLI2SR": return PllOutput(PLLI2SCFGR, PllOutputKind.R);
            case "PLLSAIP": return PllOutput(PLLSAICFGR, PllOutputKind.P);
            case "PLLSAIQ": return PllOutput(PLLSAICFGR, PllOutputKind.Q);
            case "PLLSAIR": return PllOutput(PLLSAICFGR, PllOutputKind.R);
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
            case "USART6": return UartClock(5, PClock2);
            case "UART7": return UartClock(6, PClock1);
            case "UART8": return UartClock(7, PClock1);
            case "SPI1": case "SPI4": case "SPI5": case "SPI6": return PClock2;
            case "SPI2": case "SPI3": return PClock1;
            case "I2C1": return I2cClock(0);
            case "I2C2": return I2cClock(1);
            case "I2C3": return I2cClock(2);
            case "I2C4": return I2cClock(3);
            case "CAN": case "CAN1": case "CAN2": return PClock1;
            case "CLOCK48": case "USB": case "RNG": return Clock48;
            case "SDMMC1": case "SDIO": return SdmmcClock(28);
            case "SDMMC2": return SdmmcClock(29);
            case "RTC": return RtcClock;
            case "MCO1": return Mco1Clock;
            case "MCO2": return Mco2Clock;
            case "SAI1": return SaiClock(20);
            case "SAI2": return SaiClock(22);
            case "I2S": return (Register(CFGR) & (1u << 23)) == 0 ?
                    PllOutput(PLLI2SCFGR, PllOutputKind.R) : 0;
            default:
                this.Log(LogLevel.Warning, "Unknown STM32F4/F7 clock '{0}'", name);
                return 0;
            }
        }

        private void UpdateClocks()
        {
            SetPeripheralFrequency(nvic, HClock);
            SetPeripheralFrequency(dwt, HClock);
            for(var i = 0; i < uarts.Length; i++)
            {
                var pclk = i == 0 || i == 5 ? PClock2 : PClock1;
                SetPeripheralFrequency(uarts[i], UartClock(i, pclk));
            }
            for(var i = 0; i < timers.Length; i++)
            {
                // TIM1, 8-11 are on APB2.  The array is TIM1 through TIM14.
                var timerNumber = i + 1;
                var apb2 = timerNumber == 1 || (timerNumber >= 8 && timerNumber <= 11);
                SetTimerFrequency(timers[i], apb2 ? TimerClock2 : TimerClock1);
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
                if(target != null)
                {
                    target.Frequency = frequency;
                }
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

        private ulong SystemClock => Select(Field(Register(CFGR), 0, 2),
            HsiClock, HseClock, PllOutput(PLLCFGR, PllOutputKind.P));
        private ulong HClock => Divide(SystemClock, AhbDivider(Field(Register(CFGR), 4, 4)));
        private ulong PClock1 => Divide(HClock, ApbDivider(Field(Register(CFGR), 10, 3)));
        private ulong PClock2 => Divide(HClock, ApbDivider(Field(Register(CFGR), 13, 3)));
        private ulong TimerClock1 => TimerClock(PClock1, Field(Register(CFGR), 10, 3));
        private ulong TimerClock2 => TimerClock(PClock2, Field(Register(CFGR), 13, 3));

        private ulong TimerClock(ulong pclk, uint prescaler)
        {
            if(prescaler < 4) return pclk;
            if((Register(DCKCFGR1) & TIMPRE) == 0) return pclk * 2;
            return prescaler == 4 ? pclk * 2 : pclk * 4;
        }

        private ulong UartClock(int index, ulong pclk)
        {
            if(!isF7) return pclk;
            return Select(Field(Register(DCKCFGR2), index * 2, 2),
                pclk, SystemClock, HsiClock, LseClock);
        }

        private ulong I2cClock(int index)
        {
            if(!isF7 || index == 3) return PClock1;
            return Select(Field(Register(DCKCFGR2), 16 + index * 2, 2),
                PClock1, SystemClock, HsiClock, 0);
        }

        private ulong PllOutput(long register, PllOutputKind output)
        {
            uint enable;
            if(register == PLLCFGR) enable = PLLON;
            else if(register == PLLI2SCFGR) enable = PLLI2SON;
            else enable = PLLSAION;
            if((Register(CR) & enable) == 0) return 0;
            var config = Register(register);
            var source = (Register(PLLCFGR) & PLLSRC) == 0 ? HsiClock : HseClock;
            var m = Field(Register(PLLCFGR), 0, 6);
            var n = Field(config, 6, 9);
            uint divisor;
            switch(output)
            {
            case PllOutputKind.P: divisor = 2 * (Field(config, 16, 2) + 1); break;
            case PllOutputKind.Q: divisor = Field(config, 24, 4); break;
            default: divisor = Field(config, 28, 3); break;
            }
            return source == 0 || m == 0 || n == 0 || divisor == 0 ? 0 : source * n / m / divisor;
        }

        private ulong Clock48 => isF7 && (Register(DCKCFGR2) & CK48MSEL) != 0 ?
            PllOutput(PLLSAICFGR, PllOutputKind.P) : PllOutput(PLLCFGR, PllOutputKind.Q);

        private ulong SdmmcClock(int selectionBit)
        {
            if(!isF7) return Clock48;
            return (Register(DCKCFGR2) & (1u << selectionBit)) == 0 ? Clock48 : SystemClock;
        }

        private ulong RtcClock
        {
            get
            {
                var divider = Field(Register(CFGR), 16, 5);
                var hseRtc = divider == 0 ? 0 : HseClock / divider;
                return Select(Field(Register(BDCR), 8, 2), 0, LseClock, LsiClock, hseRtc);
            }
        }

        private ulong Mco1Clock
        {
            get
            {
                var source = Select(Field(Register(CFGR), 21, 2), HsiClock, LseClock, HseClock,
                    PllOutput(PLLCFGR, PllOutputKind.P));
                var divider = McoDivider(Field(Register(CFGR), 24, 3));
                return source / divider;
            }
        }

        private ulong Mco2Clock
        {
            get
            {
                var source = Select(Field(Register(CFGR), 30, 2), SystemClock,
                    PllOutput(PLLI2SCFGR, PllOutputKind.R), HseClock,
                    PllOutput(PLLCFGR, PllOutputKind.P));
                var divider = McoDivider(Field(Register(CFGR), 27, 3));
                return source / divider;
            }
        }

        private ulong SaiClock(int shift)
        {
            return Select(Field(Register(DCKCFGR1), shift, 2),
                PllOutput(PLLSAICFGR, PllOutputKind.Q),
                PllOutput(PLLI2SCFGR, PllOutputKind.Q), 0);
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
            case 8: return 2;
            case 9: return 4;
            case 10: return 8;
            case 11: return 16;
            case 12: return 64;
            case 13: return 128;
            case 14: return 256;
            default: return 512;
            }
        }

        private static uint ApbDivider(uint encoding)
        {
            return encoding < 4 ? 1u : 1u << ((int)encoding - 3);
        }

        private static uint McoDivider(uint encoding)
        {
            return encoding < 4 ? 1u : encoding - 2;
        }

        private static ulong Divide(ulong clock, uint divisor)
        {
            return divisor == 0 ? 0 : clock / divisor;
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
            if((cr & PLLI2SON) != 0) ready |= PLLI2SRDY;
            if((cr & PLLSAION) != 0) ready |= PLLSAIRDY;
            return ready;
        }

        private static bool IsClockConfigurationRegister(long offset)
        {
            return offset == CR || offset == PLLCFGR || offset == CFGR ||
                offset == PLLI2SCFGR || offset == PLLSAICFGR ||
                offset == DCKCFGR1 || offset == DCKCFGR2 ||
                offset == BDCR || offset == CSR;
        }

        private readonly IMachine machine;
        private readonly IPeripheral rtcPeripheral;
        private readonly IPeripheral nvic;
        private IPeripheral dwt;
        private readonly IPeripheral[] uarts;
        private readonly IPeripheral[] timers;
        private readonly Dictionary<long, uint> registers;
        private readonly Dictionary<IPeripheral, ulong> appliedFrequencies;
        private readonly HashSet<Type> frequencyErrors;
        private uint hseFrequency;
        private uint lseFrequency;
        private bool isF7;

        private enum PllOutputKind { P, Q, R }

        private const long CR = 0x00;
        private const long PLLCFGR = 0x04;
        private const long CFGR = 0x08;
        private const long CIR = 0x0C;
        private const long BDCR = 0x70;
        private const long CSR = 0x74;
        private const long PLLI2SCFGR = 0x84;
        private const long PLLSAICFGR = 0x88;
        private const long DCKCFGR1 = 0x8C;
        private const long DCKCFGR2 = 0x90;

        private const uint HSION = 1u << 0;
        private const uint HSIRDY = 1u << 1;
        private const uint HSEON = 1u << 16;
        private const uint HSERDY = 1u << 17;
        private const uint PLLON = 1u << 24;
        private const uint PLLRDY = 1u << 25;
        private const uint PLLI2SON = 1u << 26;
        private const uint PLLI2SRDY = 1u << 27;
        private const uint PLLSAION = 1u << 28;
        private const uint PLLSAIRDY = 1u << 29;
        private const uint PLLSRC = 1u << 22;
        private const uint TIMPRE = 1u << 24;
        private const uint CK48MSEL = 1u << 27;
        private const uint LSEON = 1u << 0;
        private const uint LSERDY = 1u << 1;
        private const uint RTCEN = 1u << 15;
        private const uint LSION = 1u << 0;
        private const uint LSIRDY = 1u << 1;
        private const uint RMVF = 1u << 24;
        private const uint SW_MASK = 0x3;
        private const uint SWS_MASK = 0xC;
        private const uint CR_READ_ONLY_MASK = HSIRDY | HSERDY | PLLRDY | PLLI2SRDY | PLLSAIRDY;
        private const uint PORRSTF = 1u << 27;
        private const uint PINRSTF = 1u << 26;
        private const uint BORRSTF = 1u << 25;
        private const uint RESET_FLAGS = 0xFE000000;

        private const ulong HsiFrequency = 16000000;
        private const ulong LsiFrequency = 32000;
        private const uint DefaultHseFrequency = 8000000;
        private const uint DefaultLseFrequency = 32768;
    }
}
