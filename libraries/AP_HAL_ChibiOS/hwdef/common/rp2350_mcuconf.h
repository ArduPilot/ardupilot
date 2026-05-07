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
#endif

/*
 * HAL driver system settings.
 */
#define RP_NO_INIT                          FALSE
#ifndef RP_CORE1_START
#define RP_CORE1_START                      TRUE
#endif
#define RP_CORE1_VECTORS_TABLE              _vectors
#define RP_CORE1_ENTRY_POINT                _crt0_c1_entry
#define RP_CORE1_STACK_END                  __c1_main_stack_end__

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
