/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

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

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for STMicroelectronics STM32F469I-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F469I_DISCOVERY
#define BOARD_NAME                  "STMicroelectronics STM32F469I-Discovery"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#undef STM32F469xx
#define STM32F469xx

/*
 * IO pins assignments.
 */
#define GPIOA_BUTTON                0U
#define GPIOA_ARD_D3                1U
#define GPIOA_TIM5_CH2              1U
#define GPIOA_ARD_D5                2U
#define GPIOA_TIM5_CH3              2U
#define GPIOA_LCD_BL_CTRL           3U
#define GPIOA_TIM5_CH4              3U
#define GPIOA_ARD_A5                4U
#define GPIOA_EXT_7                 5U
#define GPIOA_SPI1_SCK              5U
#define GPIOA_ARD_D6                6U
#define GPIOA_TIM3_CH1              6U
#define GPIOA_ARD_D9                7U
#define GPIOA_TIM3_CH2              7U
#define GPIOA_EXT_3                 8U
#define GPIOA_MCO1                  8U
#define GPIOA_VBUS_FS1              9U
#define GPIOA_USB_FS1_ID            10U
#define GPIOA_USB_FS1_N             11U
#define GPIOA_USB_FS1_P             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_EXT_11                15U
#define GPIOA_SPI1_NSS              15U

#define GPIOB_EXT_RESET             0U
#define GPIOB_ARD_A0                1U
#define GPIOB_OTG_FS1_POWERSWITCHON 2U
#define GPIOB_I2S3_CK               3U
#define GPIOB_SPI3_SCK              3U
#define GPIOB_EXT_5                 4U
#define GPIOB_SPI1_MISO             4U
#define GPIOB_EXT_9                 5U
#define GPIOB_SPI1_MOSI             5U
#define GPIOB_QSPI_BK1_NCS          6U
#define GPIOB_QSPI_FS1_OVERCURRENT  7U
#define GPIOB_I2C1_SCL              8U
#define GPIOB_I2C1_SDA              9U
#define GPIOB_STLK_RX               10U
#define GPIOB_STLK_TX               11U
#define GPIOB_EXT_12                12U
#define GPIOB_CAN2_RX               12U
#define GPIOB_EXT_10                13U
#define GPIOB_CAN2_TX               13U
#define GPIOB_ARD_D12               14U
#define GPIOB_SPI2_MISO             14U
#define GPIOB_ARD_D11               15U
#define GPIOB_SPI2_MOSI             15U

#define GPIOC_SDNWE                 0U
#define GPIOC_EXT_14                1U
#define GPIOC_ADC123_IN11           1U
#define GPIOC_ARD_A1                2U
#define GPIOC_ARD_A2                3U
#define GPIOC_ARD_A3                4U
#define GPIOC_ARD_A4                5U
#define GPIOC_EXT_6                 6U
#define GPIOC_USART6_TX             6U
#define GPIOC_EXT_8                 7U
#define GPIOC_USART6_RX             7U
#define GPIOC_USD_D0                8U
#define GPIOC_SDIO_D0               8U
#define GPIOC_USD_D1                9U
#define GPIOC_SDIO_D1               9U
#define GPIOC_USD_D2                10U
#define GPIOC_SDIO_D2               10U
#define GPIOC_USD_D3                11U
#define GPIOC_SDIO_D3               11U
#define GPIOC_USD_CLK               12U
#define GPIOC_SDIO_CK               12U
#define GPIOC_EXT_13                13U
#define GPIOC_ANTI_TAMP1            13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_USD_CMD               2U
#define GPIOD_SDIO_CMD              2U
#define GPIOD_ARD_D13               3U
#define GPIOD_SPI2_SCK              3U
#define GPIOD_LED2                  4U
#define GPIOD_LED3                  5U
#define GPIOD_MIC_DATA              6U
#define GPIOD_SAI1_SD_A             6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_PIN11                 11U
#define GPIOD_PIN12                 12U
#define GPIOD_MIC_CK                13U
#define GPIOD_TIM4_CH2              13U
#define GPIOD_PIN14                 14U
#define GPIOD_PIN15                 15U

#define GPIOE_FMC_NBL0              0U
#define GPIOE_FMC_NBL1              1U
#define GPIOE_AUDIO_RST             2U
#define GPIOE_SPKR_HP               3U
#define GPIOE_SAI1_FSA              4U
#define GPIOE_SAI1_SCKA             5U
#define GPIOE_SAI1_SDA              6U
#define GPIOE_PIN7                  7U
#define GPIOE_PIN8                  8U
#define GPIOE_PIN9                  9U
#define GPIOE_PIN10                 10U
#define GPIOE_PIN11                 11U
#define GPIOE_PIN12                 12U
#define GPIOE_PIN13                 13U
#define GPIOE_PIN14                 14U
#define GPIOE_PIN15                 15U

#define GPIOF_PIN0                  0U
#define GPIOF_PIN1                  1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_QSPI_BK1_IO3          6U
#define GPIOF_QSPI_BK1_IO2          7U
#define GPIOF_QSPI_BK1_IO0          8U
#define GPIOF_QSPI_BK1_IO1          9U
#define GPIOF_QSPI_CLK              10U
#define GPIOF_SDNRAS                11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_USD_DETECT            2U
#define GPIOG_PIN3                  3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_LED1                  6U
#define GPIOG_SAI1_MCLKA            7U
#define GPIOG_SDCLK                 8U
#define GPIOG_ARD_D0                9U
#define GPIOG_ARD_D8                10U
#define GPIOG_ARD_D7                11U
#define GPIOG_ARD_D4                12U
#define GPIOG_ARD_D2                13U
#define GPIOG_ARD_D1                14U
#define GPIOG_SDNCAS                15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_SDCKE0                2U
#define GPIOH_SDNE0                 3U
#define GPIOH_I2C2_SCL              4U
#define GPIOH_I2C2_SDA              5U
#define GPIOH_ARD_D10               6U
#define GPIOH_SPI2_NSS              6U
#define GPIOH_LCD_RESET             7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_FMC_NBL2              4U
#define GPIOI_FMC_NBL3              5U
#define GPIOI_PIN6                  6U
#define GPIOI_PIN7                  7U
#define GPIOI_PIN8                  8U
#define GPIOI_PIN9                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

#define GPIOJ_PIN0                  0U
#define GPIOJ_PIN1                  1U
#define GPIOJ_DSI_TE                2U
#define GPIOJ_PIN3                  3U
#define GPIOJ_PIN4                  4U
#define GPIOJ_LCD_INT               5U
#define GPIOJ_PIN6                  6U
#define GPIOJ_PIN7                  7U
#define GPIOJ_PIN8                  8U
#define GPIOJ_PIN9                  9U
#define GPIOJ_PIN10                 10U
#define GPIOJ_PIN11                 11U
#define GPIOJ_PIN12                 12U
#define GPIOJ_PIN13                 13U
#define GPIOJ_PIN14                 14U
#define GPIOJ_PIN15                 15U

#define GPIOK_PIN0                  0U
#define GPIOK_PIN1                  1U
#define GPIOK_PIN2                  2U
#define GPIOK_LED4                  3U
#define GPIOK_PIN4                  4U
#define GPIOK_PIN5                  5U
#define GPIOK_PIN6                  6U
#define GPIOK_PIN7                  7U
#define GPIOK_PIN8                  8U
#define GPIOK_PIN9                  9U
#define GPIOK_PIN10                 10U
#define GPIOK_PIN11                 11U
#define GPIOK_PIN12                 12U
#define GPIOK_PIN13                 13U
#define GPIOK_PIN14                 14U
#define GPIOK_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_BUTTON                 PAL_LINE(GPIOA, 0U)
#define LINE_ARD_D3                 PAL_LINE(GPIOA, 1U)
#define LINE_TIM5_CH2               PAL_LINE(GPIOA, 1U)
#define LINE_ARD_D5                 PAL_LINE(GPIOA, 2U)
#define LINE_TIM5_CH3               PAL_LINE(GPIOA, 2U)
#define LINE_LCD_BL_CTRL            PAL_LINE(GPIOA, 3U)
#define LINE_TIM5_CH4               PAL_LINE(GPIOA, 3U)
#define LINE_ARD_A5                 PAL_LINE(GPIOA, 4U)
#define LINE_EXT_7                  PAL_LINE(GPIOA, 5U)
#define LINE_SPI1_SCK               PAL_LINE(GPIOA, 5U)
#define LINE_ARD_D6                 PAL_LINE(GPIOA, 6U)
#define LINE_TIM3_CH1               PAL_LINE(GPIOA, 6U)
#define LINE_ARD_D9                 PAL_LINE(GPIOA, 7U)
#define LINE_TIM3_CH2               PAL_LINE(GPIOA, 7U)
#define LINE_EXT_3                  PAL_LINE(GPIOA, 8U)
#define LINE_MCO1                   PAL_LINE(GPIOA, 8U)
#define LINE_VBUS_FS1               PAL_LINE(GPIOA, 9U)
#define LINE_USB_FS1_ID             PAL_LINE(GPIOA, 10U)
#define LINE_USB_FS1_N              PAL_LINE(GPIOA, 11U)
#define LINE_USB_FS1_P              PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_EXT_11                 PAL_LINE(GPIOA, 15U)
#define LINE_SPI1_NSS               PAL_LINE(GPIOA, 15U)
#define LINE_EXT_RESET              PAL_LINE(GPIOB, 0U)
#define LINE_ARD_A0                 PAL_LINE(GPIOB, 1U)
#define LINE_OTG_FS1_POWERSWITCHON  PAL_LINE(GPIOB, 2U)
#define LINE_I2S3_CK                PAL_LINE(GPIOB, 3U)
#define LINE_SPI3_SCK               PAL_LINE(GPIOB, 3U)
#define LINE_EXT_5                  PAL_LINE(GPIOB, 4U)
#define LINE_SPI1_MISO              PAL_LINE(GPIOB, 4U)
#define LINE_EXT_9                  PAL_LINE(GPIOB, 5U)
#define LINE_SPI1_MOSI              PAL_LINE(GPIOB, 5U)
#define LINE_QSPI_BK1_NCS           PAL_LINE(GPIOB, 6U)
#define LINE_QSPI_FS1_OVERCURRENT   PAL_LINE(GPIOB, 7U)
#define LINE_I2C1_SCL               PAL_LINE(GPIOB, 8U)
#define LINE_I2C1_SDA               PAL_LINE(GPIOB, 9U)
#define LINE_STLK_RX                PAL_LINE(GPIOB, 10U)
#define LINE_STLK_TX                PAL_LINE(GPIOB, 11U)
#define LINE_EXT_12                 PAL_LINE(GPIOB, 12U)
#define LINE_CAN2_RX                PAL_LINE(GPIOB, 12U)
#define LINE_EXT_10                 PAL_LINE(GPIOB, 13U)
#define LINE_CAN2_TX                PAL_LINE(GPIOB, 13U)
#define LINE_ARD_D12                PAL_LINE(GPIOB, 14U)
#define LINE_SPI2_MISO              PAL_LINE(GPIOB, 14U)
#define LINE_ARD_D11                PAL_LINE(GPIOB, 15U)
#define LINE_SPI2_MOSI              PAL_LINE(GPIOB, 15U)
#define LINE_SDNWE                  PAL_LINE(GPIOC, 0U)
#define LINE_EXT_14                 PAL_LINE(GPIOC, 1U)
#define LINE_ADC123_IN11            PAL_LINE(GPIOC, 1U)
#define LINE_ARD_A1                 PAL_LINE(GPIOC, 2U)
#define LINE_ARD_A2                 PAL_LINE(GPIOC, 3U)
#define LINE_ARD_A3                 PAL_LINE(GPIOC, 4U)
#define LINE_ARD_A4                 PAL_LINE(GPIOC, 5U)
#define LINE_EXT_6                  PAL_LINE(GPIOC, 6U)
#define LINE_USART6_TX              PAL_LINE(GPIOC, 6U)
#define LINE_EXT_8                  PAL_LINE(GPIOC, 7U)
#define LINE_USART6_RX              PAL_LINE(GPIOC, 7U)
#define LINE_USD_D0                 PAL_LINE(GPIOC, 8U)
#define LINE_SDIO_D0                PAL_LINE(GPIOC, 8U)
#define LINE_USD_D1                 PAL_LINE(GPIOC, 9U)
#define LINE_SDIO_D1                PAL_LINE(GPIOC, 9U)
#define LINE_USD_D2                 PAL_LINE(GPIOC, 10U)
#define LINE_SDIO_D2                PAL_LINE(GPIOC, 10U)
#define LINE_USD_D3                 PAL_LINE(GPIOC, 11U)
#define LINE_SDIO_D3                PAL_LINE(GPIOC, 11U)
#define LINE_USD_CLK                PAL_LINE(GPIOC, 12U)
#define LINE_SDIO_CK                PAL_LINE(GPIOC, 12U)
#define LINE_EXT_13                 PAL_LINE(GPIOC, 13U)
#define LINE_ANTI_TAMP1             PAL_LINE(GPIOC, 13U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)
#define LINE_USD_CMD                PAL_LINE(GPIOD, 2U)
#define LINE_SDIO_CMD               PAL_LINE(GPIOD, 2U)
#define LINE_ARD_D13                PAL_LINE(GPIOD, 3U)
#define LINE_SPI2_SCK               PAL_LINE(GPIOD, 3U)
#define LINE_LED2                   PAL_LINE(GPIOD, 4U)
#define LINE_LED3                   PAL_LINE(GPIOD, 5U)
#define LINE_MIC_DATA               PAL_LINE(GPIOD, 6U)
#define LINE_SAI1_SD_A              PAL_LINE(GPIOD, 6U)
#define LINE_MIC_CK                 PAL_LINE(GPIOD, 13U)
#define LINE_TIM4_CH2               PAL_LINE(GPIOD, 13U)
#define LINE_FMC_NBL0               PAL_LINE(GPIOE, 0U)
#define LINE_FMC_NBL1               PAL_LINE(GPIOE, 1U)
#define LINE_AUDIO_RST              PAL_LINE(GPIOE, 2U)
#define LINE_SPKR_HP                PAL_LINE(GPIOE, 3U)
#define LINE_SAI1_FSA               PAL_LINE(GPIOE, 4U)
#define LINE_SAI1_SCKA              PAL_LINE(GPIOE, 5U)
#define LINE_SAI1_SDA               PAL_LINE(GPIOE, 6U)
#define LINE_QSPI_BK1_IO3           PAL_LINE(GPIOF, 6U)
#define LINE_QSPI_BK1_IO2           PAL_LINE(GPIOF, 7U)
#define LINE_QSPI_BK1_IO0           PAL_LINE(GPIOF, 8U)
#define LINE_QSPI_BK1_IO1           PAL_LINE(GPIOF, 9U)
#define LINE_QSPI_CLK               PAL_LINE(GPIOF, 10U)
#define LINE_SDNRAS                 PAL_LINE(GPIOF, 11U)
#define LINE_USD_DETECT             PAL_LINE(GPIOG, 2U)
#define LINE_LED1                   PAL_LINE(GPIOG, 6U)
#define LINE_SAI1_MCLKA             PAL_LINE(GPIOG, 7U)
#define LINE_SDCLK                  PAL_LINE(GPIOG, 8U)
#define LINE_ARD_D0                 PAL_LINE(GPIOG, 9U)
#define LINE_ARD_D8                 PAL_LINE(GPIOG, 10U)
#define LINE_ARD_D7                 PAL_LINE(GPIOG, 11U)
#define LINE_ARD_D4                 PAL_LINE(GPIOG, 12U)
#define LINE_ARD_D2                 PAL_LINE(GPIOG, 13U)
#define LINE_ARD_D1                 PAL_LINE(GPIOG, 14U)
#define LINE_SDNCAS                 PAL_LINE(GPIOG, 15U)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)
#define LINE_SDCKE0                 PAL_LINE(GPIOH, 2U)
#define LINE_SDNE0                  PAL_LINE(GPIOH, 3U)
#define LINE_I2C2_SCL               PAL_LINE(GPIOH, 4U)
#define LINE_I2C2_SDA               PAL_LINE(GPIOH, 5U)
#define LINE_ARD_D10                PAL_LINE(GPIOH, 6U)
#define LINE_SPI2_NSS               PAL_LINE(GPIOH, 6U)
#define LINE_LCD_RESET              PAL_LINE(GPIOH, 7U)
#define LINE_FMC_NBL2               PAL_LINE(GPIOI, 4U)
#define LINE_FMC_NBL3               PAL_LINE(GPIOI, 5U)
#define LINE_DSI_TE                 PAL_LINE(GPIOJ, 2U)
#define LINE_LCD_INT                PAL_LINE(GPIOJ, 5U)
#define LINE_LED4                   PAL_LINE(GPIOK, 3U)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - BUTTON                    (input floating).
 * PA1  - ARD_D3 TIM5_CH2           (alternate 2).
 * PA2  - ARD_D5 TIM5_CH3           (alternate 2).
 * PA3  - LCD_BL_CTRL TIM5_CH4      (alternate 2).
 * PA4  - ARD_A5                    (analog).
 * PA5  - EXT_7 SPI1_SCK            (alternate 5).
 * PA6  - ARD_D6 TIM3_CH1           (alternate 2).
 * PA7  - ARD_D9 TIM3_CH2           (alternate 2).
 * PA8  - EXT_3 MCO1                (alternate 0).
 * PA9  - VBUS_FS1                  (analog).
 * PA10 - USB_FS1_ID                (alternate 10).
 * PA11 - USB_FS1_N                 (alternate 10).
 * PA12 - USB_FS1_P                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - EXT_11 SPI1_NSS           (output pushpull maximum).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_BUTTON) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_ARD_D3) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_ARD_D5) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_BL_CTRL) |\
                                     PIN_MODE_ANALOG(GPIOA_ARD_A5) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_EXT_7) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_ARD_D6) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_ARD_D9) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_EXT_3) |      \
                                     PIN_MODE_ANALOG(GPIOA_VBUS_FS1) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_FS1_ID) | \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_FS1_N) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_FS1_P) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_OUTPUT(GPIOA_EXT_11))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_BUTTON) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_D3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_D5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_BL_CTRL) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_A5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_EXT_7) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_D6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_D9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_EXT_3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_VBUS_FS1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_FS1_ID) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_FS1_N) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_FS1_P) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_EXT_11))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_BUTTON) |        \
                                     PIN_OSPEED_HIGH(GPIOA_ARD_D3) |        \
                                     PIN_OSPEED_HIGH(GPIOA_ARD_D5) |        \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_BL_CTRL) |   \
                                     PIN_OSPEED_HIGH(GPIOA_ARD_A5) |        \
                                     PIN_OSPEED_HIGH(GPIOA_EXT_7) |         \
                                     PIN_OSPEED_HIGH(GPIOA_ARD_D6) |        \
                                     PIN_OSPEED_HIGH(GPIOA_ARD_D9) |        \
                                     PIN_OSPEED_HIGH(GPIOA_EXT_3) |         \
                                     PIN_OSPEED_HIGH(GPIOA_VBUS_FS1) |      \
                                     PIN_OSPEED_HIGH(GPIOA_USB_FS1_ID) |    \
                                     PIN_OSPEED_HIGH(GPIOA_USB_FS1_N) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USB_FS1_P) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_EXT_11))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_BUTTON) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_ARD_D3) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_ARD_D5) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_BL_CTRL) |\
                                     PIN_PUPDR_FLOATING(GPIOA_ARD_A5) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_EXT_7) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_ARD_D6) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_ARD_D9) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_EXT_3) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_VBUS_FS1) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_FS1_ID) | \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_FS1_N) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_FS1_P) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_EXT_11))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_BUTTON) |           \
                                     PIN_ODR_HIGH(GPIOA_ARD_D3) |           \
                                     PIN_ODR_HIGH(GPIOA_ARD_D5) |           \
                                     PIN_ODR_HIGH(GPIOA_LCD_BL_CTRL) |      \
                                     PIN_ODR_HIGH(GPIOA_ARD_A5) |           \
                                     PIN_ODR_HIGH(GPIOA_EXT_7) |            \
                                     PIN_ODR_HIGH(GPIOA_ARD_D6) |           \
                                     PIN_ODR_HIGH(GPIOA_ARD_D9) |           \
                                     PIN_ODR_HIGH(GPIOA_EXT_3) |            \
                                     PIN_ODR_HIGH(GPIOA_VBUS_FS1) |         \
                                     PIN_ODR_HIGH(GPIOA_USB_FS1_ID) |       \
                                     PIN_ODR_HIGH(GPIOA_USB_FS1_N) |        \
                                     PIN_ODR_HIGH(GPIOA_USB_FS1_P) |        \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_EXT_11))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_BUTTON, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_ARD_D3, 2U) |        \
                                     PIN_AFIO_AF(GPIOA_ARD_D5, 2U) |        \
                                     PIN_AFIO_AF(GPIOA_LCD_BL_CTRL, 2U) |   \
                                     PIN_AFIO_AF(GPIOA_ARD_A5, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_EXT_7, 5U) |         \
                                     PIN_AFIO_AF(GPIOA_ARD_D6, 2U) |        \
                                     PIN_AFIO_AF(GPIOA_ARD_D9, 2U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_EXT_3, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_VBUS_FS1, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_USB_FS1_ID, 10U) |   \
                                     PIN_AFIO_AF(GPIOA_USB_FS1_N, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_USB_FS1_P, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_EXT_11, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - EXT_RESET                 (output pushpull maximum).
 * PB1  - ARD_A0                    (analog).
 * PB2  - OTG_FS1_POWERSWITCHON     (output pushpull maximum).
 * PB3  - I2S3_CK SPI3_SCK          (alternate 6).
 * PB4  - EXT_5 SPI1_MISO           (alternate 5).
 * PB5  - EXT_9 SPI1_MOSI           (alternate 5).
 * PB6  - QSPI_BK1_NCS              (alternate 10).
 * PB7  - QSPI_FS1_OVERCURRENT      (input floating).
 * PB8  - I2C1_SCL                  (alternate 4).
 * PB9  - I2C1_SDA                  (alternate 4).
 * PB10 - STLK_RX                   (alternate 7).
 * PB11 - STLK_TX                   (alternate 7).
 * PB12 - EXT_12 CAN2_RX            (alternate 9).
 * PB13 - EXT_10 CAN2_TX            (alternate 9).
 * PB14 - ARD_D12 SPI2_MISO         (alternate 5).
 * PB15 - ARD_D11 SPI2_MOSI         (alternate 5).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_EXT_RESET) |     \
                                     PIN_MODE_ANALOG(GPIOB_ARD_A0) |        \
                                     PIN_MODE_OUTPUT(GPIOB_OTG_FS1_POWERSWITCHON) |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2S3_CK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_EXT_5) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_EXT_9) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_QSPI_BK1_NCS) |\
                                     PIN_MODE_INPUT(GPIOB_QSPI_FS1_OVERCURRENT) |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_STLK_RX) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_STLK_TX) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_EXT_12) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_EXT_10) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_ARD_D12) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_ARD_D11))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_EXT_RESET) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_A0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OTG_FS1_POWERSWITCHON) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2S3_CK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_EXT_5) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_EXT_9) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_QSPI_BK1_NCS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_QSPI_FS1_OVERCURRENT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2C1_SCL) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2C1_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_STLK_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_STLK_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_EXT_12) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_EXT_10) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D12) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D11))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_EXT_RESET) |     \
                                     PIN_OSPEED_HIGH(GPIOB_ARD_A0) |        \
                                     PIN_OSPEED_HIGH(GPIOB_OTG_FS1_POWERSWITCHON) |\
                                     PIN_OSPEED_HIGH(GPIOB_I2S3_CK) |       \
                                     PIN_OSPEED_HIGH(GPIOB_EXT_5) |         \
                                     PIN_OSPEED_HIGH(GPIOB_EXT_9) |         \
                                     PIN_OSPEED_HIGH(GPIOB_QSPI_BK1_NCS) |  \
                                     PIN_OSPEED_HIGH(GPIOB_QSPI_FS1_OVERCURRENT) |\
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL) |      \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOB_STLK_RX) |       \
                                     PIN_OSPEED_HIGH(GPIOB_STLK_TX) |       \
                                     PIN_OSPEED_HIGH(GPIOB_EXT_12) |        \
                                     PIN_OSPEED_HIGH(GPIOB_EXT_10) |        \
                                     PIN_OSPEED_HIGH(GPIOB_ARD_D12) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ARD_D11))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_EXT_RESET) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_ARD_A0) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_OTG_FS1_POWERSWITCHON) |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2S3_CK) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_EXT_5) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_EXT_9) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_QSPI_BK1_NCS) |\
                                     PIN_PUPDR_FLOATING(GPIOB_QSPI_FS1_OVERCURRENT) |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_STLK_RX) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_STLK_TX) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_EXT_12) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_EXT_10) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_ARD_D12) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_ARD_D11))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_EXT_RESET) |        \
                                     PIN_ODR_HIGH(GPIOB_ARD_A0) |           \
                                     PIN_ODR_HIGH(GPIOB_OTG_FS1_POWERSWITCHON) |\
                                     PIN_ODR_HIGH(GPIOB_I2S3_CK) |          \
                                     PIN_ODR_HIGH(GPIOB_EXT_5) |            \
                                     PIN_ODR_HIGH(GPIOB_EXT_9) |            \
                                     PIN_ODR_HIGH(GPIOB_QSPI_BK1_NCS) |     \
                                     PIN_ODR_HIGH(GPIOB_QSPI_FS1_OVERCURRENT) |\
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |         \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |         \
                                     PIN_ODR_HIGH(GPIOB_STLK_RX) |          \
                                     PIN_ODR_HIGH(GPIOB_STLK_TX) |          \
                                     PIN_ODR_HIGH(GPIOB_EXT_12) |           \
                                     PIN_ODR_HIGH(GPIOB_EXT_10) |           \
                                     PIN_ODR_HIGH(GPIOB_ARD_D12) |          \
                                     PIN_ODR_HIGH(GPIOB_ARD_D11))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_EXT_RESET, 0U) |     \
                                     PIN_AFIO_AF(GPIOB_ARD_A0, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_OTG_FS1_POWERSWITCHON, 0U) |\
                                     PIN_AFIO_AF(GPIOB_I2S3_CK, 6U) |       \
                                     PIN_AFIO_AF(GPIOB_EXT_5, 5U) |         \
                                     PIN_AFIO_AF(GPIOB_EXT_9, 5U) |         \
                                     PIN_AFIO_AF(GPIOB_QSPI_BK1_NCS, 10U) | \
                                     PIN_AFIO_AF(GPIOB_QSPI_FS1_OVERCURRENT, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_I2C1_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 4U) |      \
                                     PIN_AFIO_AF(GPIOB_STLK_RX, 7U) |       \
                                     PIN_AFIO_AF(GPIOB_STLK_TX, 7U) |       \
                                     PIN_AFIO_AF(GPIOB_EXT_12, 9U) |        \
                                     PIN_AFIO_AF(GPIOB_EXT_10, 9U) |        \
                                     PIN_AFIO_AF(GPIOB_ARD_D12, 5U) |       \
                                     PIN_AFIO_AF(GPIOB_ARD_D11, 5U))

/*
 * GPIOC setup:
 *
 * PC0  - SDNWE                     (alternate 12).
 * PC1  - EXT_14 ADC123_IN11        (analog).
 * PC2  - ARD_A1                    (analog).
 * PC3  - ARD_A2                    (analog).
 * PC4  - ARD_A3                    (analog).
 * PC5  - ARD_A4                    (analog).
 * PC6  - EXT_6 USART6_TX           (alternate 8).
 * PC7  - EXT_8 USART6_RX           (alternate 8).
 * PC8  - USD_D0 SDIO_D0            (alternate 12).
 * PC9  - USD_D1 SDIO_D1            (alternate 12).
 * PC10 - USD_D2 SDIO_D2            (alternate 12).
 * PC11 - USD_D3 SDIO_D3            (alternate 12).
 * PC12 - USD_CLK SDIO_CK           (alternate 12).
 * PC13 - EXT_13 ANTI_TAMP1         (input pullup).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ALTERNATE(GPIOC_SDNWE) |      \
                                     PIN_MODE_ANALOG(GPIOC_EXT_14) |        \
                                     PIN_MODE_ANALOG(GPIOC_ARD_A1) |        \
                                     PIN_MODE_ANALOG(GPIOC_ARD_A2) |        \
                                     PIN_MODE_ANALOG(GPIOC_ARD_A3) |        \
                                     PIN_MODE_ANALOG(GPIOC_ARD_A4) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_EXT_6) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_EXT_8) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_USD_D0) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_USD_D1) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_USD_D2) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_USD_D3) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_USD_CLK) |    \
                                     PIN_MODE_INPUT(GPIOC_EXT_13) |         \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_SDNWE) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_EXT_14) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_A1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_A2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_A3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_A4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_EXT_6) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_EXT_8) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USD_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USD_D1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USD_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USD_D3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USD_CLK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_EXT_13) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_SDNWE) |         \
                                     PIN_OSPEED_HIGH(GPIOC_EXT_14) |        \
                                     PIN_OSPEED_HIGH(GPIOC_ARD_A1) |        \
                                     PIN_OSPEED_HIGH(GPIOC_ARD_A2) |        \
                                     PIN_OSPEED_HIGH(GPIOC_ARD_A3) |        \
                                     PIN_OSPEED_HIGH(GPIOC_ARD_A4) |        \
                                     PIN_OSPEED_HIGH(GPIOC_EXT_6) |         \
                                     PIN_OSPEED_HIGH(GPIOC_EXT_8) |         \
                                     PIN_OSPEED_HIGH(GPIOC_USD_D0) |        \
                                     PIN_OSPEED_HIGH(GPIOC_USD_D1) |        \
                                     PIN_OSPEED_HIGH(GPIOC_USD_D2) |        \
                                     PIN_OSPEED_HIGH(GPIOC_USD_D3) |        \
                                     PIN_OSPEED_HIGH(GPIOC_USD_CLK) |       \
                                     PIN_OSPEED_HIGH(GPIOC_EXT_13) |        \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_IN) |      \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_SDNWE) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_EXT_14) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_ARD_A1) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_ARD_A2) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_ARD_A3) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_ARD_A4) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_EXT_6) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_EXT_8) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_USD_D0) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_USD_D1) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_USD_D2) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_USD_D3) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_USD_CLK) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_EXT_13) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_SDNWE) |            \
                                     PIN_ODR_HIGH(GPIOC_EXT_14) |           \
                                     PIN_ODR_HIGH(GPIOC_ARD_A1) |           \
                                     PIN_ODR_HIGH(GPIOC_ARD_A2) |           \
                                     PIN_ODR_HIGH(GPIOC_ARD_A3) |           \
                                     PIN_ODR_HIGH(GPIOC_ARD_A4) |           \
                                     PIN_ODR_HIGH(GPIOC_EXT_6) |            \
                                     PIN_ODR_HIGH(GPIOC_EXT_8) |            \
                                     PIN_ODR_HIGH(GPIOC_USD_D0) |           \
                                     PIN_ODR_HIGH(GPIOC_USD_D1) |           \
                                     PIN_ODR_HIGH(GPIOC_USD_D2) |           \
                                     PIN_ODR_HIGH(GPIOC_USD_D3) |           \
                                     PIN_ODR_HIGH(GPIOC_USD_CLK) |          \
                                     PIN_ODR_HIGH(GPIOC_EXT_13) |           \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_SDNWE, 12U) |        \
                                     PIN_AFIO_AF(GPIOC_EXT_14, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_ARD_A1, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_ARD_A2, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_ARD_A3, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_ARD_A4, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_EXT_6, 8U) |         \
                                     PIN_AFIO_AF(GPIOC_EXT_8, 8U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_USD_D0, 12U) |       \
                                     PIN_AFIO_AF(GPIOC_USD_D1, 12U) |       \
                                     PIN_AFIO_AF(GPIOC_USD_D2, 12U) |       \
                                     PIN_AFIO_AF(GPIOC_USD_D3, 12U) |       \
                                     PIN_AFIO_AF(GPIOC_USD_CLK, 12U) |      \
                                     PIN_AFIO_AF(GPIOC_EXT_13, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pullup).
 * PD1  - PIN1                      (input pullup).
 * PD2  - USD_CMD SDIO_CMD          (alternate 12).
 * PD3  - ARD_D13 SPI2_SCK          (alternate 5).
 * PD4  - LED2                      (output pushpull maximum).
 * PD5  - LED3                      (output pushpull maximum).
 * PD6  - MIC_DATA SAI1_SD_A        (alternate 6).
 * PD7  - PIN7                      (input pullup).
 * PD8  - PIN8                      (input pullup).
 * PD9  - PIN9                      (input pullup).
 * PD10 - PIN10                     (input pullup).
 * PD11 - PIN11                     (input pullup).
 * PD12 - PIN12                     (input pullup).
 * PD13 - MIC_CK TIM4_CH2           (alternate 2).
 * PD14 - PIN14                     (input pullup).
 * PD15 - PIN15                     (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |           \
                                     PIN_MODE_ALTERNATE(GPIOD_USD_CMD) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_ARD_D13) |    \
                                     PIN_MODE_OUTPUT(GPIOD_LED2) |          \
                                     PIN_MODE_OUTPUT(GPIOD_LED3) |          \
                                     PIN_MODE_ALTERNATE(GPIOD_MIC_DATA) |   \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN12) |          \
                                     PIN_MODE_ALTERNATE(GPIOD_MIC_CK) |     \
                                     PIN_MODE_INPUT(GPIOD_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USD_CMD) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_ARD_D13) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LED2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LED3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MIC_DATA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MIC_CK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOD_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN1) |       \
                                     PIN_OSPEED_HIGH(GPIOD_USD_CMD) |       \
                                     PIN_OSPEED_HIGH(GPIOD_ARD_D13) |       \
                                     PIN_OSPEED_HIGH(GPIOD_LED2) |          \
                                     PIN_OSPEED_HIGH(GPIOD_LED3) |          \
                                     PIN_OSPEED_HIGH(GPIOD_MIC_DATA) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN12) |      \
                                     PIN_OSPEED_HIGH(GPIOD_MIC_CK) |        \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_USD_CMD) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_ARD_D13) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_LED2) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_LED3) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_MIC_DATA) |   \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOD_MIC_CK) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOD_USD_CMD) |          \
                                     PIN_ODR_HIGH(GPIOD_ARD_D13) |          \
                                     PIN_ODR_HIGH(GPIOD_LED2) |             \
                                     PIN_ODR_HIGH(GPIOD_LED3) |             \
                                     PIN_ODR_HIGH(GPIOD_MIC_DATA) |         \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOD_MIC_CK) |           \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_USD_CMD, 12U) |      \
                                     PIN_AFIO_AF(GPIOD_ARD_D13, 5U) |       \
                                     PIN_AFIO_AF(GPIOD_LED2, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_LED3, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_MIC_DATA, 6U) |      \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_MIC_CK, 2U) |        \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - FMC_NBL0                  (alternate 12).
 * PE1  - FMC_NBL1                  (alternate 12).
 * PE2  - AUDIO_RST                 (alternate 6).
 * PE3  - SPKR_HP                   (alternate 6).
 * PE4  - SAI1_FSA                  (alternate 6).
 * PE5  - SAI1_SCKA                 (alternate 6).
 * PE6  - SAI1_SDA                  (alternate 6).
 * PE7  - PIN7                      (input pullup).
 * PE8  - PIN8                      (input pullup).
 * PE9  - PIN9                      (input pullup).
 * PE10 - PIN10                     (input pullup).
 * PE11 - PIN11                     (input pullup).
 * PE12 - PIN12                     (input pullup).
 * PE13 - PIN13                     (input pullup).
 * PE14 - PIN14                     (input pullup).
 * PE15 - PIN15                     (input pullup).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_FMC_NBL0) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_NBL1) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_AUDIO_RST) |  \
                                     PIN_MODE_ALTERNATE(GPIOE_SPKR_HP) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_FSA) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_SCKA) |  \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_SDA) |   \
                                     PIN_MODE_INPUT(GPIOE_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_FMC_NBL0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_NBL1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_AUDIO_RST) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPKR_HP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_FSA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SCKA) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_FMC_NBL0) |      \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_NBL1) |      \
                                     PIN_OSPEED_HIGH(GPIOE_AUDIO_RST) |     \
                                     PIN_OSPEED_HIGH(GPIOE_SPKR_HP) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_FSA) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_SCKA) |     \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_SDA) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_FMC_NBL0) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_NBL1) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_AUDIO_RST) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_SPKR_HP) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_FSA) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_SCKA) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_SDA) |   \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_FMC_NBL0) |         \
                                     PIN_ODR_HIGH(GPIOE_FMC_NBL1) |         \
                                     PIN_ODR_HIGH(GPIOE_AUDIO_RST) |        \
                                     PIN_ODR_HIGH(GPIOE_SPKR_HP) |          \
                                     PIN_ODR_HIGH(GPIOE_SAI1_FSA) |         \
                                     PIN_ODR_HIGH(GPIOE_SAI1_SCKA) |        \
                                     PIN_ODR_HIGH(GPIOE_SAI1_SDA) |         \
                                     PIN_ODR_HIGH(GPIOE_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_FMC_NBL0, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_FMC_NBL1, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_AUDIO_RST, 6U) |     \
                                     PIN_AFIO_AF(GPIOE_SPKR_HP, 6U) |       \
                                     PIN_AFIO_AF(GPIOE_SAI1_FSA, 6U) |      \
                                     PIN_AFIO_AF(GPIOE_SAI1_SCKA, 6U) |     \
                                     PIN_AFIO_AF(GPIOE_SAI1_SDA, 6U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0U))

/*
 * GPIOF setup:
 *
 * PF0  - PIN0                      (input pullup).
 * PF1  - PIN1                      (input pullup).
 * PF2  - PIN2                      (input pullup).
 * PF3  - PIN3                      (input pullup).
 * PF4  - PIN4                      (input pullup).
 * PF5  - PIN5                      (input pullup).
 * PF6  - QSPI_BK1_IO3              (alternate 9).
 * PF7  - QSPI_BK1_IO2              (alternate 9).
 * PF8  - QSPI_BK1_IO0              (alternate 9).
 * PF9  - QSPI_BK1_IO1              (alternate 9).
 * PF10 - QSPI_CLK                  (alternate 9).
 * PF11 - SDNRAS                    (alternate 12).
 * PF12 - PIN12                     (input pullup).
 * PF13 - PIN13                     (input pullup).
 * PF14 - PIN14                     (input pullup).
 * PF15 - PIN15                     (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |           \
                                     PIN_MODE_ALTERNATE(GPIOF_QSPI_BK1_IO3) |\
                                     PIN_MODE_ALTERNATE(GPIOF_QSPI_BK1_IO2) |\
                                     PIN_MODE_ALTERNATE(GPIOF_QSPI_BK1_IO0) |\
                                     PIN_MODE_ALTERNATE(GPIOF_QSPI_BK1_IO1) |\
                                     PIN_MODE_ALTERNATE(GPIOF_QSPI_CLK) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDNRAS) |     \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_QSPI_BK1_IO3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_QSPI_BK1_IO2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_QSPI_BK1_IO0) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_QSPI_BK1_IO1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_QSPI_CLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDNRAS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOF_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN5) |       \
                                     PIN_OSPEED_HIGH(GPIOF_QSPI_BK1_IO3) |  \
                                     PIN_OSPEED_HIGH(GPIOF_QSPI_BK1_IO2) |  \
                                     PIN_OSPEED_HIGH(GPIOF_QSPI_BK1_IO0) |  \
                                     PIN_OSPEED_HIGH(GPIOF_QSPI_BK1_IO1) |  \
                                     PIN_OSPEED_HIGH(GPIOF_QSPI_CLK) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDNRAS) |        \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLUP(GPIOF_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_QSPI_BK1_IO3) |\
                                     PIN_PUPDR_FLOATING(GPIOF_QSPI_BK1_IO2) |\
                                     PIN_PUPDR_FLOATING(GPIOF_QSPI_BK1_IO0) |\
                                     PIN_PUPDR_FLOATING(GPIOF_QSPI_BK1_IO1) |\
                                     PIN_PUPDR_FLOATING(GPIOF_QSPI_CLK) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDNRAS) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_QSPI_BK1_IO3) |     \
                                     PIN_ODR_HIGH(GPIOF_QSPI_BK1_IO2) |     \
                                     PIN_ODR_HIGH(GPIOF_QSPI_BK1_IO0) |     \
                                     PIN_ODR_HIGH(GPIOF_QSPI_BK1_IO1) |     \
                                     PIN_ODR_HIGH(GPIOF_QSPI_CLK) |         \
                                     PIN_ODR_HIGH(GPIOF_SDNRAS) |           \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_QSPI_BK1_IO3, 9U) |  \
                                     PIN_AFIO_AF(GPIOF_QSPI_BK1_IO2, 9U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_QSPI_BK1_IO0, 9U) |  \
                                     PIN_AFIO_AF(GPIOF_QSPI_BK1_IO1, 9U) |  \
                                     PIN_AFIO_AF(GPIOF_QSPI_CLK, 9U) |      \
                                     PIN_AFIO_AF(GPIOF_SDNRAS, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - PIN0                      (input pullup).
 * PG1  - PIN1                      (input pullup).
 * PG2  - USD_DETECT                (input pullup).
 * PG3  - PIN3                      (input pullup).
 * PG4  - PIN4                      (input pullup).
 * PG5  - PIN5                      (input pullup).
 * PG6  - LED1                      (output pushpull maximum).
 * PG7  - SAI1_MCLKA                (alternate 6).
 * PG8  - SDCLK                     (alternate 12).
 * PG9  - ARD_D0                    (input pullup).
 * PG10 - ARD_D8                    (input pullup).
 * PG11 - ARD_D7                    (input pullup).
 * PG12 - ARD_D4                    (input pullup).
 * PG13 - ARD_D2                    (input pullup).
 * PG14 - ARD_D1                    (input pullup).
 * PG15 - SDNCAS                    (alternate 12).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOG_USD_DETECT) |     \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN5) |           \
                                     PIN_MODE_OUTPUT(GPIOG_LED1) |          \
                                     PIN_MODE_ALTERNATE(GPIOG_SAI1_MCLKA) | \
                                     PIN_MODE_ALTERNATE(GPIOG_SDCLK) |      \
                                     PIN_MODE_INPUT(GPIOG_ARD_D0) |         \
                                     PIN_MODE_INPUT(GPIOG_ARD_D8) |         \
                                     PIN_MODE_INPUT(GPIOG_ARD_D7) |         \
                                     PIN_MODE_INPUT(GPIOG_ARD_D4) |         \
                                     PIN_MODE_INPUT(GPIOG_ARD_D2) |         \
                                     PIN_MODE_INPUT(GPIOG_ARD_D1) |         \
                                     PIN_MODE_ALTERNATE(GPIOG_SDNCAS))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_USD_DETECT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_LED1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SAI1_MCLKA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SDCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SDNCAS))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOG_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN1) |       \
                                     PIN_OSPEED_HIGH(GPIOG_USD_DETECT) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN5) |       \
                                     PIN_OSPEED_HIGH(GPIOG_LED1) |          \
                                     PIN_OSPEED_HIGH(GPIOG_SAI1_MCLKA) |    \
                                     PIN_OSPEED_HIGH(GPIOG_SDCLK) |         \
                                     PIN_OSPEED_VERYLOW(GPIOG_ARD_D0) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_ARD_D8) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_ARD_D7) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_ARD_D4) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_ARD_D2) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_ARD_D1) |     \
                                     PIN_OSPEED_HIGH(GPIOG_SDNCAS))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLUP(GPIOG_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_USD_DETECT) |   \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_LED1) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_SAI1_MCLKA) | \
                                     PIN_PUPDR_FLOATING(GPIOG_SDCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D0) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D8) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D7) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D4) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D2) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D1) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_SDNCAS))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOG_USD_DETECT) |       \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOG_LED1) |             \
                                     PIN_ODR_HIGH(GPIOG_SAI1_MCLKA) |       \
                                     PIN_ODR_HIGH(GPIOG_SDCLK) |            \
                                     PIN_ODR_HIGH(GPIOG_ARD_D0) |           \
                                     PIN_ODR_HIGH(GPIOG_ARD_D8) |           \
                                     PIN_ODR_HIGH(GPIOG_ARD_D7) |           \
                                     PIN_ODR_HIGH(GPIOG_ARD_D4) |           \
                                     PIN_ODR_HIGH(GPIOG_ARD_D2) |           \
                                     PIN_ODR_HIGH(GPIOG_ARD_D1) |           \
                                     PIN_ODR_HIGH(GPIOG_SDNCAS))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_USD_DETECT, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_LED1, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_SAI1_MCLKA, 6U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_SDCLK, 12U) |        \
                                     PIN_AFIO_AF(GPIOG_ARD_D0, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_ARD_D8, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_ARD_D7, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_ARD_D4, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_ARD_D2, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_ARD_D1, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_SDNCAS, 12U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - SDCKE0                    (alternate 12).
 * PH3  - SDNE0                     (alternate 12).
 * PH4  - I2C2_SCL                  (alternate 4).
 * PH5  - I2C2_SDA                  (alternate 4).
 * PH6  - ARD_D10 SPI2_NSS          (output pushpull maximum).
 * PH7  - LCD_RESET                 (output pushpull minimum).
 * PH8  - PIN8                      (input pullup).
 * PH9  - PIN9                      (input pullup).
 * PH10 - PIN10                     (input pullup).
 * PH11 - PIN11                     (input pullup).
 * PH12 - PIN12                     (input pullup).
 * PH13 - PIN13                     (input pullup).
 * PH14 - PIN14                     (input pullup).
 * PH15 - PIN15                     (input pullup).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_ALTERNATE(GPIOH_SDCKE0) |     \
                                     PIN_MODE_ALTERNATE(GPIOH_SDNE0) |      \
                                     PIN_MODE_ALTERNATE(GPIOH_I2C2_SCL) |   \
                                     PIN_MODE_ALTERNATE(GPIOH_I2C2_SDA) |   \
                                     PIN_MODE_OUTPUT(GPIOH_ARD_D10) |       \
                                     PIN_MODE_OUTPUT(GPIOH_LCD_RESET) |     \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_SDCKE0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_SDNE0) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_I2C2_SCL) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOH_I2C2_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOH_ARD_D10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_RESET) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_HIGH(GPIOH_SDCKE0) |        \
                                     PIN_OSPEED_HIGH(GPIOH_SDNE0) |         \
                                     PIN_OSPEED_HIGH(GPIOH_I2C2_SCL) |      \
                                     PIN_OSPEED_HIGH(GPIOH_I2C2_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOH_ARD_D10) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_LCD_RESET) |  \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_SDCKE0) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_SDNE0) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_I2C2_SCL) |   \
                                     PIN_PUPDR_FLOATING(GPIOH_I2C2_SDA) |   \
                                     PIN_PUPDR_PULLUP(GPIOH_ARD_D10) |      \
                                     PIN_PUPDR_PULLUP(GPIOH_LCD_RESET) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_SDCKE0) |           \
                                     PIN_ODR_HIGH(GPIOH_SDNE0) |            \
                                     PIN_ODR_HIGH(GPIOH_I2C2_SCL) |         \
                                     PIN_ODR_HIGH(GPIOH_I2C2_SDA) |         \
                                     PIN_ODR_HIGH(GPIOH_ARD_D10) |          \
                                     PIN_ODR_HIGH(GPIOH_LCD_RESET) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_SDCKE0, 12U) |       \
                                     PIN_AFIO_AF(GPIOH_SDNE0, 12U) |        \
                                     PIN_AFIO_AF(GPIOH_I2C2_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOH_I2C2_SDA, 4U) |      \
                                     PIN_AFIO_AF(GPIOH_ARD_D10, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_LCD_RESET, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input pullup).
 * PI1  - PIN1                      (input pullup).
 * PI2  - PIN2                      (input pullup).
 * PI3  - PIN3                      (input pullup).
 * PI4  - FMC_NBL2                  (alternate 12).
 * PI5  - FMC_NBL3                  (alternate 12).
 * PI6  - PIN6                      (input pullup).
 * PI7  - PIN7                      (input pullup).
 * PI8  - PIN8                      (input pullup).
 * PI9  - PIN9                      (input pullup).
 * PI10 - PIN10                     (input pullup).
 * PI11 - PIN11                     (input pullup).
 * PI12 - PIN12                     (input pullup).
 * PI13 - PIN13                     (input pullup).
 * PI14 - PIN14                     (input pullup).
 * PI15 - PIN15                     (input pullup).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |           \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_NBL2) |   \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_NBL3) |   \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_NBL2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_NBL3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOI_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN3) |       \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_NBL2) |      \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_NBL3) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_NBL2) |   \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_NBL3) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOI_FMC_NBL2) |         \
                                     PIN_ODR_HIGH(GPIOI_FMC_NBL3) |         \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_FMC_NBL2, 12U) |     \
                                     PIN_AFIO_AF(GPIOI_FMC_NBL3, 12U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))

/*
 * GPIOJ setup:
 *
 * PJ0  - PIN0                      (input pullup).
 * PJ1  - PIN1                      (input pullup).
 * PJ2  - DSI_TE                    (alternate 13).
 * PJ3  - PIN3                      (input pullup).
 * PJ4  - PIN4                      (input pullup).
 * PJ5  - LCD_INT                   (input pullup).
 * PJ6  - PIN6                      (input pullup).
 * PJ7  - PIN7                      (input pullup).
 * PJ8  - PIN8                      (input pullup).
 * PJ9  - PIN9                      (input pullup).
 * PJ10 - PIN10                     (input pullup).
 * PJ11 - PIN11                     (input pullup).
 * PJ12 - PIN12                     (input pullup).
 * PJ13 - PIN13                     (input pullup).
 * PJ14 - PIN14                     (input pullup).
 * PJ15 - PIN15                     (input pullup).
 */
#define VAL_GPIOJ_MODER             (PIN_MODE_INPUT(GPIOJ_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN1) |           \
                                     PIN_MODE_ALTERNATE(GPIOJ_DSI_TE) |     \
                                     PIN_MODE_INPUT(GPIOJ_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOJ_LCD_INT) |        \
                                     PIN_MODE_INPUT(GPIOJ_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN15))
#define VAL_GPIOJ_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOJ_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_DSI_TE) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_INT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN15))
#define VAL_GPIOJ_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOJ_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN1) |       \
                                     PIN_OSPEED_HIGH(GPIOJ_DSI_TE) |        \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_INT) |    \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN15))
#define VAL_GPIOJ_PUPDR             (PIN_PUPDR_PULLUP(GPIOJ_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN1) |         \
                                     PIN_PUPDR_FLOATING(GPIOJ_DSI_TE) |     \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_LCD_INT) |      \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN15))
#define VAL_GPIOJ_ODR               (PIN_ODR_HIGH(GPIOJ_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOJ_DSI_TE) |           \
                                     PIN_ODR_HIGH(GPIOJ_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOJ_LCD_INT) |          \
                                     PIN_ODR_HIGH(GPIOJ_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN15))
#define VAL_GPIOJ_AFRL              (PIN_AFIO_AF(GPIOJ_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_DSI_TE, 13U) |       \
                                     PIN_AFIO_AF(GPIOJ_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_LCD_INT, 0U) |       \
                                     PIN_AFIO_AF(GPIOJ_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN7, 0U))
#define VAL_GPIOJ_AFRH              (PIN_AFIO_AF(GPIOJ_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN15, 0U))

/*
 * GPIOK setup:
 *
 * PK0  - PIN0                      (input pullup).
 * PK1  - PIN1                      (input pullup).
 * PK2  - PIN2                      (input pullup).
 * PK3  - LED4                      (output pushpull maximum).
 * PK4  - PIN4                      (input pullup).
 * PK5  - PIN5                      (input pullup).
 * PK6  - PIN6                      (input pullup).
 * PK7  - PIN7                      (input pullup).
 * PK8  - PIN8                      (input pullup).
 * PK9  - PIN9                      (input pullup).
 * PK10 - PIN10                     (input pullup).
 * PK11 - PIN11                     (input pullup).
 * PK12 - PIN12                     (input pullup).
 * PK13 - PIN13                     (input pullup).
 * PK14 - PIN14                     (input pullup).
 * PK15 - PIN15                     (input pullup).
 */
#define VAL_GPIOK_MODER             (PIN_MODE_INPUT(GPIOK_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN2) |           \
                                     PIN_MODE_OUTPUT(GPIOK_LED4) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN15))
#define VAL_GPIOK_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOK_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LED4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN15))
#define VAL_GPIOK_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOK_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN2) |       \
                                     PIN_OSPEED_HIGH(GPIOK_LED4) |          \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN15))
#define VAL_GPIOK_PUPDR             (PIN_PUPDR_PULLUP(GPIOK_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOK_LED4) |       \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN15))
#define VAL_GPIOK_ODR               (PIN_ODR_HIGH(GPIOK_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOK_LED4) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN15))
#define VAL_GPIOK_AFRL              (PIN_AFIO_AF(GPIOK_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_LED4, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN7, 0U))
#define VAL_GPIOK_AFRH              (PIN_AFIO_AF(GPIOK_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN15, 0U))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
