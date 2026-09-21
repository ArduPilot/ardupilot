// STM32H743/H757 reset and clock controller.  ChibiOS programs the complete
// clock tree at boot, so fixed frequencies in the platform description are
// wrong for boards which use a different crystal or system clock.  This model
// implements the oscillator, PLL, domain-prescaler and kernel-clock mux
// registers and propagates their derived clocks into timing-aware Renode
// peripherals.
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
    public class AP_STM32H7_RCC : IDoubleWordPeripheral, IKnownSize
    {
        public AP_STM32H7_RCC(
            IPeripheral nvic = null, IPeripheral dwt = null,
            IPeripheral usart1 = null, IPeripheral usart2 = null,
            IPeripheral usart3 = null, IPeripheral uart4 = null,
            IPeripheral uart5 = null, IPeripheral usart6 = null,
            IPeripheral uart7 = null, IPeripheral uart8 = null,
            IPeripheral lpuart1 = null,
            IPeripheral timer1 = null, IPeripheral timer2 = null,
            IPeripheral timer3 = null, IPeripheral timer4 = null,
            IPeripheral timer5 = null, IPeripheral timer8 = null,
            IPeripheral timer12 = null, IPeripheral timer13 = null,
            IPeripheral timer14 = null, IPeripheral timer15 = null,
            uint hseFrequency = DefaultHseFrequency,
            uint lseFrequency = DefaultLseFrequency)
        {
            registers = new Dictionary<long, uint>();
            appliedFrequencies = new Dictionary<IPeripheral, ulong>();
            frequencyErrors = new HashSet<Type>();
            this.nvic = nvic;
            this.dwt = dwt;
            this.usart1 = usart1;
            this.usart2 = usart2;
            this.usart3 = usart3;
            this.uart4 = uart4;
            this.uart5 = uart5;
            this.usart6 = usart6;
            this.uart7 = uart7;
            this.uart8 = uart8;
            this.lpuart1 = lpuart1;
            this.timer1 = timer1;
            this.timer2 = timer2;
            this.timer3 = timer3;
            this.timer4 = timer4;
            this.timer5 = timer5;
            this.timer8 = timer8;
            this.timer12 = timer12;
            this.timer13 = timer13;
            this.timer14 = timer14;
            this.timer15 = timer15;
            this.hseFrequency = hseFrequency;
            this.lseFrequency = lseFrequency;
            Reset();
        }

        public long Size => 0x400;

        public uint HseFrequency
        {
            get { return hseFrequency; }
            set
            {
                hseFrequency = value;
                UpdateClocks();
            }
        }

        public uint LseFrequency
        {
            get { return lseFrequency; }
            set
            {
                lseFrequency = value;
                UpdateClocks();
            }
        }

        public IPeripheral Nvic { get { return nvic; } set { nvic = value; UpdateClocks(); } }
        public IPeripheral Dwt { get { return dwt; } set { dwt = value; UpdateClocks(); } }

        public IPeripheral Usart1 { get { return usart1; } set { usart1 = value; UpdateClocks(); } }
        public IPeripheral Usart2 { get { return usart2; } set { usart2 = value; UpdateClocks(); } }
        public IPeripheral Usart3 { get { return usart3; } set { usart3 = value; UpdateClocks(); } }
        public IPeripheral Uart4 { get { return uart4; } set { uart4 = value; UpdateClocks(); } }
        public IPeripheral Uart5 { get { return uart5; } set { uart5 = value; UpdateClocks(); } }
        public IPeripheral Usart6 { get { return usart6; } set { usart6 = value; UpdateClocks(); } }
        public IPeripheral Uart7 { get { return uart7; } set { uart7 = value; UpdateClocks(); } }
        public IPeripheral Uart8 { get { return uart8; } set { uart8 = value; UpdateClocks(); } }
        public IPeripheral Lpuart1 { get { return lpuart1; } set { lpuart1 = value; UpdateClocks(); } }

        public IPeripheral Timer1 { get { return timer1; } set { timer1 = value; UpdateClocks(); } }
        public IPeripheral Timer2 { get { return timer2; } set { timer2 = value; UpdateClocks(); } }
        public IPeripheral Timer3 { get { return timer3; } set { timer3 = value; UpdateClocks(); } }
        public IPeripheral Timer4 { get { return timer4; } set { timer4 = value; UpdateClocks(); } }
        public IPeripheral Timer5 { get { return timer5; } set { timer5 = value; UpdateClocks(); } }
        public IPeripheral Timer8 { get { return timer8; } set { timer8 = value; UpdateClocks(); } }
        public IPeripheral Timer12 { get { return timer12; } set { timer12 = value; UpdateClocks(); } }
        public IPeripheral Timer13 { get { return timer13; } set { timer13 = value; UpdateClocks(); } }
        public IPeripheral Timer14 { get { return timer14; } set { timer14 = value; UpdateClocks(); } }
        public IPeripheral Timer15 { get { return timer15; } set { timer15 = value; UpdateClocks(); } }

        public void Reset()
        {
            registers.Clear();
            registers[CR] = HSION;
            registers[RSR] = PORRSTF | PINRSTF | BORRSTF;
            appliedFrequencies.Clear();
            UpdateClocks();
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
                // The emulated sources start immediately, so SWS follows SW.
                return (value & ~SWS_MASK) | ((value & SW_MASK) << SWS_SHIFT);
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
                UpdateClocks();
                return;
            case CFGR:
                registers[CFGR] = value & ~SWS_MASK;
                UpdateClocks();
                return;
            case BDCR:
                registers[BDCR] = value & ~LSERDY;
                UpdateClocks();
                return;
            case CSR:
                registers[CSR] = value & ~LSIRDY;
                UpdateClocks();
                return;
            case RSR:
                // RMVF clears reset-cause flags, but is not itself retained.
                if((value & RMVF) != 0)
                {
                    registers[RSR] = 0;
                }
                return;
            case CICR:
                // Clock interrupts are instantaneous in this model.
                registers[CIFR] = 0;
                registers[CICR] = value;
                return;
            default:
                registers[offset] = value;
                if(IsClockConfigurationRegister(offset))
                {
                    UpdateClocks();
                }
                return;
            }
        }

        // This is intentionally monitor-visible.  It makes it possible to
        // verify the firmware-programmed clock tree without duplicating the
        // calculations in board-generation or test scripts.
        public ulong GetClockFrequency(string name)
        {
            if(String.IsNullOrEmpty(name))
            {
                return 0;
            }
            switch(name.Replace("_", "").ToUpperInvariant())
            {
            case "HSI": return HsiClock;
            case "CSI": return CsiClock;
            case "HSE": return HseClock;
            case "HSI48": return Hsi48Clock;
            case "LSE": return LseClock;
            case "LSI": return LsiClock;
            case "PLL1P": return PllOutput(1, PllOutputKind.P);
            case "PLL1Q": return PllOutput(1, PllOutputKind.Q);
            case "PLL1R": return PllOutput(1, PllOutputKind.R);
            case "PLL2P": return PllOutput(2, PllOutputKind.P);
            case "PLL2Q": return PllOutput(2, PllOutputKind.Q);
            case "PLL2R": return PllOutput(2, PllOutputKind.R);
            case "PLL3P": return PllOutput(3, PllOutputKind.P);
            case "PLL3Q": return PllOutput(3, PllOutputKind.Q);
            case "PLL3R": return PllOutput(3, PllOutputKind.R);
            case "SYS": case "SYSCK": case "SYSCLK": return SystemClock;
            case "CORE": case "CORECK": return CoreClock;
            case "HCLK": return HClock;
            case "PCLK1": return PClock1;
            case "PCLK2": return PClock2;
            case "PCLK3": return PClock3;
            case "PCLK4": return PClock4;
            case "TIMCLK1": return TimerClock1;
            case "TIMCLK2": return TimerClock2;
            case "PER": case "PERCK": return PeripheralClock;
            case "USART1": case "USART6": return Usart16Clock;
            case "USART2": case "USART3": case "UART4": case "UART5":
            case "UART7": case "UART8": return Usart234578Clock;
            case "LPUART1": return Lpuart1Clock;
            case "SPI1": case "SPI2": case "SPI3": return Spi123Clock;
            case "SPI4": case "SPI5": return Spi45Clock;
            case "SPI6": return Spi6Clock;
            case "I2C1": case "I2C2": case "I2C3": return I2c123Clock;
            case "I2C4": return I2c4Clock;
            case "FDCAN": case "FDCAN1": case "FDCAN2": return FdcanClock;
            case "SDMMC": case "SDMMC1": case "SDMMC2": return SdmmcClock;
            case "RNG": return RngClock;
            case "ADC": return AdcClock;
            case "USB": return UsbClock;
            case "FMC": return FmcClock;
            case "QSPI": return QspiClock;
            case "LPTIM1": return Lptim1Clock;
            case "LPTIM2": return Lptim2Clock;
            case "LPTIM3": case "LPTIM4": case "LPTIM5": return Lptim345Clock;
            case "RTC": return RtcClock;
            case "MCO1": return Mco1Clock;
            case "MCO2": return Mco2Clock;
            case "SAI1": return SaiClock(Field(Register(D2CCIP1R), 0, 3));
            case "SAI2": case "SAI3": return SaiClock(Field(Register(D2CCIP1R), 6, 3));
            case "SAI4A": return SaiClock(Field(Register(D3CCIPR), 21, 3));
            case "SAI4B": return SaiClock(Field(Register(D3CCIPR), 24, 3));
            case "SPDIF": return SpdifClock;
            case "DFSDM1": return (Register(D2CCIP1R) & (1u << 24)) == 0 ? PClock2 : SystemClock;
            case "SWP": return (Register(D2CCIP1R) & (1u << 31)) == 0 ? PClock1 : HsiClock;
            case "CEC": return CecClock;
            default:
                this.Log(LogLevel.Warning, "Unknown STM32H7 clock '{0}'", name);
                return 0;
            }
        }

        private void UpdateClocks()
        {
            SetPeripheralFrequency(nvic, CoreClock);
            SetPeripheralFrequency(dwt, CoreClock);

            SetPeripheralFrequency(usart1, Usart16Clock);
            SetPeripheralFrequency(usart6, Usart16Clock);
            SetPeripheralFrequency(usart2, Usart234578Clock);
            SetPeripheralFrequency(usart3, Usart234578Clock);
            SetPeripheralFrequency(uart4, Usart234578Clock);
            SetPeripheralFrequency(uart5, Usart234578Clock);
            SetPeripheralFrequency(uart7, Usart234578Clock);
            SetPeripheralFrequency(uart8, Usart234578Clock);
            SetPeripheralFrequency(lpuart1, Lpuart1Clock);

            SetTimerFrequency(timer1, TimerClock2);
            SetTimerFrequency(timer8, TimerClock2);
            SetTimerFrequency(timer15, TimerClock2);
            SetTimerFrequency(timer2, TimerClock1);
            SetTimerFrequency(timer3, TimerClock1);
            SetTimerFrequency(timer4, TimerClock1);
            SetTimerFrequency(timer5, TimerClock1);
            SetTimerFrequency(timer12, TimerClock1);
            SetTimerFrequency(timer13, TimerClock1);
            SetTimerFrequency(timer14, TimerClock1);
        }

        private void SetTimerFrequency(IPeripheral peripheral, ulong frequency)
        {
            if(!SetPeripheralFrequency(peripheral, frequency) || peripheral == null)
            {
                return;
            }

            // STM32_Timer owns four auxiliary compare timers.  Its inherited
            // Frequency property updates the main counter only.
            var field = peripheral.GetType().GetField(
                "ccTimers", BindingFlags.NonPublic | BindingFlags.Instance);
            var timers = field == null ? null : field.GetValue(peripheral) as IEnumerable;
            if(timers == null)
            {
                return;
            }
            foreach(var timer in timers)
            {
                var frequencyTarget = timer as IHasFrequency;
                if(frequencyTarget != null)
                {
                    frequencyTarget.Frequency = frequency;
                }
            }
        }

        private bool SetPeripheralFrequency(IPeripheral peripheral, ulong frequency)
        {
            if(peripheral == null || frequency == 0)
            {
                return false;
            }
            ulong previous;
            if(appliedFrequencies.TryGetValue(peripheral, out previous) && previous == frequency)
            {
                return false;
            }

            try
            {
                var frequencyTarget = peripheral as IHasFrequency;
                if(frequencyTarget != null)
                {
                    frequencyTarget.Frequency = frequency;
                }
                else
                {
                    // STM32F7_USART predates IHasFrequency and keeps its input
                    // clock in a private readonly uint.  BaudRate and receiver
                    // timing read the field dynamically, so updating it is
                    // sufficient and avoids carrying a fork of Renode.
                    var field = peripheral.GetType().GetField(
                        "frequency", BindingFlags.NonPublic | BindingFlags.Instance);
                    if(field == null)
                    {
                        throw new MissingFieldException(peripheral.GetType().Name, "frequency");
                    }
                    field.SetValue(peripheral, Convert.ChangeType(frequency, field.FieldType));
                }
                appliedFrequencies[peripheral] = frequency;
                return true;
            }
            catch(Exception e)
            {
                if(frequencyErrors.Add(peripheral.GetType()))
                {
                    this.Log(LogLevel.Error,
                        "Could not apply {0}Hz clock to {1}: {2}",
                        frequency, peripheral.GetType().Name, e.Message);
                }
                return false;
            }
        }

        private ulong HsiClock => ClockEnabled(HSION) ?
            HsiFrequency >> (int)Field(Register(CR), 3, 2) : 0;
        private ulong CsiClock => ClockEnabled(CSION) ? CsiFrequency : 0;
        private ulong Hsi48Clock => ClockEnabled(HSI48ON) ? Hsi48Frequency : 0;
        private ulong HseClock => ClockEnabled(HSEON) ? hseFrequency : 0;
        private ulong LseClock => (Register(BDCR) & LSEON) != 0 ? lseFrequency : 0;
        private ulong LsiClock => (Register(CSR) & LSION) != 0 ? LsiFrequency : 0;

        private ulong SystemClock
        {
            get
            {
                switch(Field(Register(CFGR), 0, 3))
                {
                case 0: return HsiClock;
                case 1: return CsiClock;
                case 2: return HseClock;
                case 3: return PllOutput(1, PllOutputKind.P);
                default: return 0;
                }
            }
        }

        private ulong CoreClock => Divide(SystemClock, AhbDivider(Field(Register(D1CFGR), 8, 4)));
        private ulong HClock => Divide(CoreClock, AhbDivider(Field(Register(D1CFGR), 0, 4)));
        private ulong PClock1 => Divide(HClock, ApbDivider(Field(Register(D2CFGR), 4, 3)));
        private ulong PClock2 => Divide(HClock, ApbDivider(Field(Register(D2CFGR), 8, 3)));
        private ulong PClock3 => Divide(HClock, ApbDivider(Field(Register(D1CFGR), 4, 3)));
        private ulong PClock4 => Divide(HClock, ApbDivider(Field(Register(D3CFGR), 4, 3)));

        private ulong TimerClock1 => TimerClock(PClock1, Field(Register(D2CFGR), 4, 3));
        private ulong TimerClock2 => TimerClock(PClock2, Field(Register(D2CFGR), 8, 3));

        private ulong TimerClock(ulong pclk, uint prescalerEncoding)
        {
            if(prescalerEncoding < 4)
            {
                return pclk;
            }
            // TIMPRE=0 always doubles a divided APB clock.  TIMPRE=1 uses
            // 2x for /2 and 4x for the larger APB divisors.
            var multiplier = (Register(CFGR) & TIMPRE) == 0 || prescalerEncoding == 4 ? 2UL : 4UL;
            return pclk * multiplier;
        }

        private ulong PeripheralClock => Select(Field(Register(D1CCIPR), 28, 2),
            HsiClock, CsiClock, HseClock);

        private ulong Usart16Clock => UartClock(Field(Register(D2CCIP2R), 3, 3), PClock2);
        private ulong Usart234578Clock => UartClock(Field(Register(D2CCIP2R), 0, 3), PClock1);
        private ulong Lpuart1Clock => UartClock(Field(Register(D3CCIPR), 0, 3), PClock4);

        private ulong UartClock(uint selection, ulong pclk)
        {
            return Select(selection, pclk, PllOutput(2, PllOutputKind.Q),
                PllOutput(3, PllOutputKind.Q), HsiClock, CsiClock, LseClock);
        }

        private ulong Spi123Clock => Select(Field(Register(D2CCIP1R), 12, 3),
            PllOutput(1, PllOutputKind.Q), PllOutput(2, PllOutputKind.P),
            PllOutput(3, PllOutputKind.P), 0, PeripheralClock);
        private ulong Spi45Clock => Select(Field(Register(D2CCIP1R), 16, 3),
            PClock2, PllOutput(2, PllOutputKind.Q), PllOutput(3, PllOutputKind.Q),
            HsiClock, CsiClock, HseClock);
        private ulong Spi6Clock => Select(Field(Register(D3CCIPR), 28, 3),
            PClock4, PllOutput(2, PllOutputKind.Q), PllOutput(3, PllOutputKind.Q),
            HsiClock, CsiClock, HseClock);

        private ulong I2c123Clock => Select(Field(Register(D2CCIP2R), 12, 2),
            PClock1, PllOutput(3, PllOutputKind.R), HsiClock, CsiClock);
        private ulong I2c4Clock => Select(Field(Register(D3CCIPR), 8, 2),
            PClock4, PllOutput(3, PllOutputKind.R), HsiClock, CsiClock);
        private ulong FdcanClock => Select(Field(Register(D2CCIP1R), 28, 2),
            HseClock, PllOutput(1, PllOutputKind.Q), PllOutput(2, PllOutputKind.Q));
        private ulong SdmmcClock => Select(Field(Register(D1CCIPR), 16, 1),
            PllOutput(1, PllOutputKind.Q), PllOutput(2, PllOutputKind.R));
        private ulong RngClock => Select(Field(Register(D2CCIP2R), 8, 2),
            Hsi48Clock, PllOutput(1, PllOutputKind.Q), LseClock, LsiClock);
        private ulong AdcClock => Select(Field(Register(D3CCIPR), 16, 2),
            PllOutput(2, PllOutputKind.P), PllOutput(3, PllOutputKind.R), PeripheralClock, 0);
        private ulong UsbClock => Select(Field(Register(D2CCIP2R), 20, 2),
            0, PllOutput(1, PllOutputKind.Q), PllOutput(3, PllOutputKind.Q), Hsi48Clock);
        private ulong FmcClock => HighSpeedPeripheralClock(Field(Register(D1CCIPR), 0, 2));
        private ulong QspiClock => HighSpeedPeripheralClock(Field(Register(D1CCIPR), 4, 2));

        private ulong HighSpeedPeripheralClock(uint selection)
        {
            return Select(selection, HClock, PllOutput(1, PllOutputKind.Q),
                PllOutput(2, PllOutputKind.R), PeripheralClock);
        }

        private ulong Lptim1Clock => LowPowerTimerClock(
            Field(Register(D2CCIP2R), 28, 3), PClock1, false);
        private ulong Lptim2Clock => LowPowerTimerClock(
            Field(Register(D3CCIPR), 10, 3), PClock4, true);
        private ulong Lptim345Clock => LowPowerTimerClock(
            Field(Register(D3CCIPR), 13, 3), PClock4, true);

        private ulong RtcClock
        {
            get
            {
                var hseDivider = Field(Register(CFGR), 8, 6);
                var hseRtc = hseDivider == 0 ? 0 : HseClock / hseDivider;
                return Select(Field(Register(BDCR), 8, 2), 0, LseClock, LsiClock, hseRtc);
            }
        }

        private ulong Mco1Clock
        {
            get
            {
                var divider = Field(Register(CFGR), 18, 4);
                var source = Select(Field(Register(CFGR), 22, 3), HsiClock,
                    LseClock, HseClock, PllOutput(1, PllOutputKind.Q), Hsi48Clock);
                return divider == 0 ? 0 : source / divider;
            }
        }

        private ulong Mco2Clock
        {
            get
            {
                var divider = Field(Register(CFGR), 25, 4);
                var source = Select(Field(Register(CFGR), 29, 3), SystemClock,
                    PllOutput(2, PllOutputKind.P), HseClock,
                    PllOutput(1, PllOutputKind.P), CsiClock, LsiClock);
                return divider == 0 ? 0 : source / divider;
            }
        }

        private ulong SaiClock(uint selection)
        {
            return Select(selection, PllOutput(1, PllOutputKind.Q),
                PllOutput(2, PllOutputKind.P), PllOutput(3, PllOutputKind.P),
                0, PeripheralClock);
        }

        private ulong SpdifClock => Select(Field(Register(D2CCIP1R), 20, 2),
            PllOutput(1, PllOutputKind.Q), PllOutput(2, PllOutputKind.R),
            PllOutput(3, PllOutputKind.R), HsiClock);
        private ulong CecClock => Select(Field(Register(D2CCIP2R), 22, 2),
            LseClock, LsiClock, CsiClock, 0);

        private ulong LowPowerTimerClock(uint selection, ulong pclk, bool pll3UsesP)
        {
            return Select(selection, pclk, PllOutput(2, PllOutputKind.P),
                PllOutput(3, pll3UsesP ? PllOutputKind.P : PllOutputKind.R),
                LseClock, LsiClock, PeripheralClock);
        }

        private ulong PllOutput(int pll, PllOutputKind output)
        {
            var onBit = PLL1ON << ((pll - 1) * 2);
            if((Register(CR) & onBit) == 0)
            {
                return 0;
            }

            var outputEnableBit = 16 + (pll - 1) * 3 + (int)output;
            if((Register(PLLCFGR) & (1u << outputEnableBit)) == 0)
            {
                return 0;
            }

            var source = Select(Field(Register(PLLCKSELR), 0, 2),
                HsiClock, CsiClock, HseClock, 0);
            var mShift = 4 + (pll - 1) * 8;
            var m = Field(Register(PLLCKSELR), mShift, 6);
            if(source == 0 || m == 0)
            {
                return 0;
            }

            var divr = Register(PLL1DIVR + (pll - 1) * 8);
            var fracr = Register(PLL1FRACR + (pll - 1) * 8);
            var n = Field(divr, 0, 9) + 1;
            var fracEnabled = (Register(PLLCFGR) & (1u << ((pll - 1) * 4))) != 0;
            var frac = fracEnabled ? Field(fracr, 3, 13) : 0;
            var dividerShift = output == PllOutputKind.P ? 9 :
                output == PllOutputKind.Q ? 16 : 24;
            var outputDivider = Field(divr, dividerShift, 7) + 1;
            var multiplier = (double)n + (double)frac / 8192.0;
            return (ulong)Math.Round((double)source * multiplier / m / outputDivider);
        }

        private bool ClockEnabled(uint bit)
        {
            return (Register(CR) & bit) != 0;
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
            if(encoding < 4) return 1;
            return 1u << ((int)encoding - 3);
        }

        private static ulong Divide(ulong clock, uint divisor)
        {
            return divisor == 0 ? 0 : clock / divisor;
        }

        private static ulong Select(uint selection, params ulong[] clocks)
        {
            return selection < clocks.Length ? clocks[selection] : 0;
        }

        private static bool IsClockConfigurationRegister(long offset)
        {
            return offset == CR || offset == CFGR || offset == D1CFGR ||
                offset == D2CFGR || offset == D3CFGR || offset == PLLCKSELR ||
                offset == PLLCFGR || (offset >= PLL1DIVR && offset <= PLL3FRACR) ||
                offset == D1CCIPR || offset == D2CCIP1R || offset == D2CCIP2R ||
                offset == D3CCIPR || offset == BDCR || offset == CSR;
        }

        private static uint ReadyBits(uint cr)
        {
            uint ready = D1CKRDY | D2CKRDY;
            if((cr & HSION) != 0) ready |= HSIRDY | HSIDIVF;
            if((cr & CSION) != 0) ready |= CSIRDY;
            if((cr & HSI48ON) != 0) ready |= HSI48RDY;
            if((cr & HSEON) != 0) ready |= HSERDY;
            if((cr & PLL1ON) != 0) ready |= PLL1RDY;
            if((cr & PLL2ON) != 0) ready |= PLL2RDY;
            if((cr & PLL3ON) != 0) ready |= PLL3RDY;
            return ready;
        }

        private readonly Dictionary<long, uint> registers;
        private readonly Dictionary<IPeripheral, ulong> appliedFrequencies;
        private readonly HashSet<Type> frequencyErrors;

        private uint hseFrequency;
        private uint lseFrequency;
        private IPeripheral nvic;
        private IPeripheral dwt;
        private IPeripheral usart1;
        private IPeripheral usart2;
        private IPeripheral usart3;
        private IPeripheral uart4;
        private IPeripheral uart5;
        private IPeripheral usart6;
        private IPeripheral uart7;
        private IPeripheral uart8;
        private IPeripheral lpuart1;
        private IPeripheral timer1;
        private IPeripheral timer2;
        private IPeripheral timer3;
        private IPeripheral timer4;
        private IPeripheral timer5;
        private IPeripheral timer8;
        private IPeripheral timer12;
        private IPeripheral timer13;
        private IPeripheral timer14;
        private IPeripheral timer15;

        private enum PllOutputKind
        {
            P,
            Q,
            R,
        }

        private const long CR = 0x00;
        private const long CFGR = 0x10;
        private const long D1CFGR = 0x18;
        private const long D2CFGR = 0x1C;
        private const long D3CFGR = 0x20;
        private const long PLLCKSELR = 0x28;
        private const long PLLCFGR = 0x2C;
        private const long PLL1DIVR = 0x30;
        private const long PLL1FRACR = 0x34;
        private const long PLL3FRACR = 0x44;
        private const long D1CCIPR = 0x4C;
        private const long D2CCIP1R = 0x50;
        private const long D2CCIP2R = 0x54;
        private const long D3CCIPR = 0x58;
        private const long CIFR = 0x64;
        private const long CICR = 0x68;
        private const long BDCR = 0x70;
        private const long CSR = 0x74;
        private const long RSR = 0xD0;

        private const uint HSION = 1u << 0;
        private const uint HSIRDY = 1u << 2;
        private const uint HSIDIVF = 1u << 5;
        private const uint CSION = 1u << 7;
        private const uint CSIRDY = 1u << 8;
        private const uint HSI48ON = 1u << 12;
        private const uint HSI48RDY = 1u << 13;
        private const uint D1CKRDY = 1u << 14;
        private const uint D2CKRDY = 1u << 15;
        private const uint HSEON = 1u << 16;
        private const uint HSERDY = 1u << 17;
        private const uint PLL1ON = 1u << 24;
        private const uint PLL1RDY = 1u << 25;
        private const uint PLL2ON = 1u << 26;
        private const uint PLL2RDY = 1u << 27;
        private const uint PLL3ON = 1u << 28;
        private const uint PLL3RDY = 1u << 29;
        private const uint TIMPRE = 1u << 15;
        private const uint LSEON = 1u << 0;
        private const uint LSERDY = 1u << 1;
        private const uint LSION = 1u << 0;
        private const uint LSIRDY = 1u << 1;
        private const uint RMVF = 1u << 16;
        private const uint SW_MASK = 0x7;
        private const int SWS_SHIFT = 3;
        private const uint SWS_MASK = 0x7 << SWS_SHIFT;
        private const uint CR_READ_ONLY_MASK = HSIRDY | HSIDIVF | CSIRDY |
            HSI48RDY | D1CKRDY | D2CKRDY | HSERDY | PLL1RDY |
            PLL2RDY | PLL3RDY;
        private const uint PORRSTF = 1u << 23;
        private const uint PINRSTF = 1u << 22;
        private const uint BORRSTF = 1u << 21;

        private const ulong HsiFrequency = 64000000;
        private const ulong CsiFrequency = 4000000;
        private const ulong Hsi48Frequency = 48000000;
        private const ulong LsiFrequency = 32000;
        private const uint DefaultHseFrequency = 24000000;
        private const uint DefaultLseFrequency = 32768;
    }
}
