/*
    ChibiOS - Copyright (C) 2006..2025 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * RP2350_MCUCONF drivers configuration.
 * IRQ priorities: 15...0 Lowest...Highest (4 bits on Cortex-M33).
 */

#define RP2350_MCUCONF

/* Enable DMA support - required for rp_dma.c code to be compiled in */
#if !defined(RP_DMA_REQUIRED)
#define RP_DMA_REQUIRED
#endif  // !defined(RP_DMA_REQUIRED)

/*
 * HAL driver system settings.
 */
#define RP_NO_INIT                          FALSE
#ifndef RP_CORE1_START
#define RP_CORE1_START                      TRUE
#endif
// SRAM9 copy, filled before halInit() starts core1
#define RP_CORE1_VECTORS_TABLE              rp2350_c1_vectors
#define RP_CORE1_ENTRY_POINT                _crt0_c1_entry
#define RP_CORE1_STACK_END                  __c1_main_stack_end__

/*
 * Flight firmware settings every RP2350 board shares, so a board's hwdef.dat
 * does not repeat them; a board can still set any of them. The bootloader
 * runs the stock 150 MHz clock with the flash timing in its hwdef-bl.dat.
 */
#if !defined(HAL_BOOTLOADER_BUILD)

/*
 * CPU clock: 200 MHz (1200 MHz VCO, FBDIV 100, POSTDIV 6x1), a 1.33x
 * overclock of the RP2350 datasheet's 150 MHz clk_sys/clk_peri limit, chosen
 * for the 100 MHz flash clock it divides to. The on-chip VREG is set to
 * 1.15 V (VSEL=0x0c) before the PLL switch in rp2350_vreg_init(). The
 * datasheet does not qualify 200 MHz across process, voltage and temperature.
 * Bring-up ran at 375 MHz at 1.30 V, then at 225 MHz with a 75 MHz flash
 * clock. The bidirectional DShot and OSD programs want a 75 MHz PIO clock,
 * which 200 MHz reaches with a fractional divider. A board declares the rate
 * with MCU_CLOCKRATE_MHZ, and system.cpp checks that the two agree.
 */
#ifndef RP_PLL_SYS_VCO_FREQ
#define RP_PLL_SYS_VCO_FREQ                 1200000000
#endif
#ifndef RP_PLL_SYS_POSTDIV1
#define RP_PLL_SYS_POSTDIV1                 6
#endif
#ifndef RP_PLL_SYS_POSTDIV2
#define RP_PLL_SYS_POSTDIV2                 1
#endif
// on-chip VREG voltage select: 0x0c = 1.15 V (datasheet Table 481, VREG.VSEL)
#ifndef RP_VREG_VSEL
#define RP_VREG_VSEL                        12
#endif

/*
 * QMI flash timing, measured on RPI_UAVFC, whose flash is a Winbond W25Q32JV
 * (JEDEC ID EF4016, read over SWD): 133 MHz, tCLQV 6 ns, tCLQX 1.5 ns. The
 * QMI samples (CLKDIV + RXDELAY) / (2 x sysclk) after the SCK falling edge,
 * but the board's round trip delays the data, so the read window has to be
 * measured. At 200 MHz with CLKDIV 2 (SCK 100 MHz), two reads of the 1.46 MB
 * app per setting with both cores halted: RXDELAY 0 (5 ns) and 5-7
 * (17.5-22.5 ns) read nothing right, 1-4 (7.5-15 ns) read every byte right.
 * RXDELAY 2 (10 ns) has a passing setting either side. It is also applied
 * before the PLL switch, while the core runs from flash on the ring
 * oscillator; there RXDELAY above CLKDIV would sample the next bit, and
 * RXDELAY 2 samples on the edge, inside the flash's hold time. A board with
 * another flash sweeps its own window (rp2350/DEVELOPMENT.md, "Clock and
 * flash timing") and sets these in its hwdef.dat if it differs.
 */
#ifndef RP_QMI_CLKDIV
#define RP_QMI_CLKDIV                       2
#endif
#ifndef RP_QMI_RXDELAY
#define RP_QMI_RXDELAY                      2
#endif
// system clocks: 3 is 15 ns at 200 MHz, against the W25Q32JV's 10 ns /CS
// deselect time between reads (tSHSL1). The bootrom leaves 7 (35 ns).
#ifndef RP_QMI_MIN_DESELECT
#define RP_QMI_MIN_DESELECT                 3
#endif
/*
 * Continuous read: mode byte 0xA0 (M5-4 = 10) leaves the flash expecting the
 * next read's address, so a quad XIP cache miss takes 28 flash clocks instead
 * of 36. Revision G of the W25Q32JV datasheet documents this; revision I no
 * longer does, so the port checks the flash reads back correctly and keeps
 * the ordinary format if not.
 */
#ifndef RP_QMI_CONT_READ_MODE
#define RP_QMI_CONT_READ_MODE               0xA0
#endif

// bounds-check flash programming and block erases against the board's part
#ifndef RP_FLASH_SIZE
#define RP_FLASH_SIZE                       (BOARD_FLASH_SIZE * 1024U)
#endif

// the USB CDC console and MAVLink link
#ifndef RP_USB_USE_USB1
#define RP_USB_USE_USB1                     TRUE
#endif

// battery monitoring; the ADC driver needs a peripheral once HAL_USE_ADC is on
#ifndef RP_ADC_USE_ADC1
#define RP_ADC_USE_ADC1                     TRUE
#endif

/*
 * The PC sampler owns TIMER0 ALARM2 (core0) and ALARM3 (core1) with its own
 * zero-latency vectors; suppress the ChibiOS ST-LLD handlers for those two
 * alarms (the SMP scheduler only uses ALARM0/ALARM1) so there is no clash.
 */
#ifndef ST_TIMER_ALARM2_SUPPRESS_ISR
#define ST_TIMER_ALARM2_SUPPRESS_ISR
#endif
#ifndef ST_TIMER_ALARM3_SUPPRESS_ISR
#define ST_TIMER_ALARM3_SUPPRESS_ISR
#endif

#endif  // !defined(HAL_BOOTLOADER_BUILD)

/*
 * IRQ system settings.
 */
#define RP_IRQ_SYSTICK_PRIORITY             2
#define RP_IRQ_TIMER0_ALARM0_PRIORITY       2
#define RP_IRQ_TIMER0_ALARM1_PRIORITY       2
#define RP_IRQ_TIMER0_ALARM2_PRIORITY       2
#define RP_IRQ_TIMER0_ALARM3_PRIORITY       2
#define RP_IRQ_UART0_PRIORITY               3
#define RP_IRQ_UART1_PRIORITY               3
#define RP_IRQ_USB0_PRIORITY                3
/*
 * GPIO edge callbacks are used for RC input on Pico2.
 * Keeping the bank IRQ below USB avoids RC pulse bursts starving CDC completion handling.
 */
#define RP_IO_IRQ_BANK0_PRIORITY            10
#define RP_IRQ_SPI0_PRIORITY                2
#define RP_IRQ_SPI1_PRIORITY                2

/*
 * SIO driver system settings.
 */
#ifndef RP_SIO_USE_UART0
#define RP_SIO_USE_UART0                    TRUE
#endif
#ifndef RP_SIO_USE_UART1
#define RP_SIO_USE_UART1                    FALSE
#endif

/*
 * SPI driver system settings.
 */
#ifndef RP_SPI_USE_SPI0
#define RP_SPI_USE_SPI0                     FALSE
#endif
#ifndef RP_SPI_USE_SPI1
#define RP_SPI_USE_SPI1                     FALSE
#endif
#ifndef RP_SPI_SPI0_RX_DMA_CHANNEL
#define RP_SPI_SPI0_RX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#endif
#ifndef RP_SPI_SPI0_TX_DMA_CHANNEL
#define RP_SPI_SPI0_TX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#endif
#ifndef RP_SPI_SPI1_RX_DMA_CHANNEL
#define RP_SPI_SPI1_RX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#endif
#ifndef RP_SPI_SPI1_TX_DMA_CHANNEL
#define RP_SPI_SPI1_TX_DMA_CHANNEL          RP_DMA_CHANNEL_ID_ANY
#endif
#ifndef RP_SPI_SPI0_DMA_PRIORITY
#define RP_SPI_SPI0_DMA_PRIORITY            1
#endif
#ifndef RP_SPI_SPI1_DMA_PRIORITY
#define RP_SPI_SPI1_DMA_PRIORITY            1
#endif
#define RP_SPI_DMA_ERROR_HOOK(spip)

/*
 * I2C driver system settings.
 */
#ifndef RP_I2C_USE_I2C0
#define RP_I2C_USE_I2C0                     FALSE
#endif
#ifndef RP_I2C_USE_I2C1
#define RP_I2C_USE_I2C1                     FALSE
#endif
#ifndef RP_IRQ_I2C0_PRIORITY
#define RP_IRQ_I2C0_PRIORITY                2
#endif
#ifndef RP_IRQ_I2C1_PRIORITY
#define RP_IRQ_I2C1_PRIORITY                2
#endif

/*
 * ADC driver system settings.
 */
#ifndef RP_ADC_USE_ADC1
#define RP_ADC_USE_ADC1                     FALSE
#endif
#ifndef RP_ADC_ADC1_DMA_PRIORITY
#define RP_ADC_ADC1_DMA_PRIORITY            0
#endif
#ifndef RP_ADC_ADC1_DMA_IRQ_PRIORITY
#define RP_ADC_ADC1_DMA_IRQ_PRIORITY        3
#endif

#endif /* MCUCONF_H */
