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
 * Setup for STMicroelectronics STM32L053-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32L053_DISCOVERY
#define BOARD_NAME                  "STMicroelectronics STM32L053-Discovery"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#define STM32_LSEDRV                (3U << 11U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

#define STM32_HSE_BYPASS

/*
 * MCU type as defined in the ST header.
 */
#undef STM32L053xx
#define STM32L053xx

/*
 * IO pins assignments.
 */
#define GPIOA_BUTTON                0U
#define GPIOA_MFX_WAKEUP            1U
#define GPIOA_TS_G1_IO3             2U
#define GPIOA_TS_G1_IO4             3U
#define GPIOA_PIN4                  4U
#define GPIOA_LED_RED               5U
#define GPIOA_TS_G2_IO3             6U
#define GPIOA_TS_G2_IO4             7U
#define GPIOA_EPD1_BUSY             8U
#define GPIOA_USART1_TX             9U
#define GPIOA_USART1_RX             10U
#define GPIOA_USB1_DM               11U
#define GPIOA_USB1_DP               12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_EPD1_CS               15U

#define GPIOB_TS_G3_IO2             0U
#define GPIOB_TS_G3_IO3             1U
#define GPIOB_EPD1_RESET            2U
#define GPIOB_EPD1_SCK              3U
#define GPIOB_LED_GREEN             4U
#define GPIOB_EPD1_MOSI             5U
#define GPIOB_NFC_IRQINN            6U
#define GPIOB_NFC_IRQOUTN           7U
#define GPIOB_MFX_I2C1_SCL          8U
#define GPIOB_MFX_I2C1_SDA          9U
#define GPIOB_EPD1_PWR_ENN          10U
#define GPIOB_EPD1_D_C              11U
#define GPIOB_NFC_NSS               12U
#define GPIOB_NFC_SCK               13U
#define GPIOB_NFC_MISO              14U
#define GPIOB_NFC_MOSI              15U

#define GPIOC_PIN0                  0U
#define GPIOC_PIN1                  1U
#define GPIOC_PIN2                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_PIN4                  4U
#define GPIOC_PIN5                  5U
#define GPIOC_PIN6                  6U
#define GPIOC_PIN7                  7U
#define GPIOC_PIN8                  8U
#define GPIOC_PIN9                  9U
#define GPIOC_PIN10                 10U
#define GPIOC_PIN11                 11U
#define GPIOC_PIN12                 12U
#define GPIOC_MFX_IRQ_OUT           13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_PIN2                  2U
#define GPIOD_PIN3                  3U
#define GPIOD_PIN4                  4U
#define GPIOD_PIN5                  5U
#define GPIOD_PIN6                  6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_PIN11                 11U
#define GPIOD_PIN12                 12U
#define GPIOD_PIN13                 13U
#define GPIOD_PIN14                 14U
#define GPIOD_PIN15                 15U

#define GPIOH_PIN0                  0U
#define GPIOH_PIN1                  1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_BUTTON                 PAL_LINE(GPIOA, 0U)
#define LINE_MFX_WAKEUP             PAL_LINE(GPIOA, 1U)
#define LINE_TS_G1_IO3              PAL_LINE(GPIOA, 2U)
#define LINE_TS_G1_IO4              PAL_LINE(GPIOA, 3U)
#define LINE_LED_RED                PAL_LINE(GPIOA, 5U)
#define LINE_TS_G2_IO3              PAL_LINE(GPIOA, 6U)
#define LINE_TS_G2_IO4              PAL_LINE(GPIOA, 7U)
#define LINE_EPD1_BUSY              PAL_LINE(GPIOA, 8U)
#define LINE_USART1_TX              PAL_LINE(GPIOA, 9U)
#define LINE_USART1_RX              PAL_LINE(GPIOA, 10U)
#define LINE_USB1_DM                PAL_LINE(GPIOA, 11U)
#define LINE_USB1_DP                PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_EPD1_CS                PAL_LINE(GPIOA, 15U)
#define LINE_TS_G3_IO2              PAL_LINE(GPIOB, 0U)
#define LINE_TS_G3_IO3              PAL_LINE(GPIOB, 1U)
#define LINE_EPD1_RESET             PAL_LINE(GPIOB, 2U)
#define LINE_EPD1_SCK               PAL_LINE(GPIOB, 3U)
#define LINE_LED_GREEN              PAL_LINE(GPIOB, 4U)
#define LINE_EPD1_MOSI              PAL_LINE(GPIOB, 5U)
#define LINE_NFC_IRQINN             PAL_LINE(GPIOB, 6U)
#define LINE_NFC_IRQOUTN            PAL_LINE(GPIOB, 7U)
#define LINE_MFX_I2C1_SCL           PAL_LINE(GPIOB, 8U)
#define LINE_MFX_I2C1_SDA           PAL_LINE(GPIOB, 9U)
#define LINE_EPD1_PWR_ENN           PAL_LINE(GPIOB, 10U)
#define LINE_EPD1_D_C               PAL_LINE(GPIOB, 11U)
#define LINE_NFC_NSS                PAL_LINE(GPIOB, 12U)
#define LINE_NFC_SCK                PAL_LINE(GPIOB, 13U)
#define LINE_NFC_MISO               PAL_LINE(GPIOB, 14U)
#define LINE_NFC_MOSI               PAL_LINE(GPIOB, 15U)
#define LINE_MFX_IRQ_OUT            PAL_LINE(GPIOC, 13U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)

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
 * PA1  - MFX_WAKEUP                (alternate 0).
 * PA2  - TS_G1_IO3                 (alternate 3).
 * PA3  - TS_G1_IO4                 (alternate 3).
 * PA4  - PIN4                      (input pullup).
 * PA5  - LED_RED                   (output pushpull maximum).
 * PA6  - TS_G2_IO3                 (alternate 3).
 * PA7  - TS_G2_IO4                 (alternate 3).
 * PA8  - EPD1_BUSY                 (input floating).
 * PA9  - USART1_TX                 (alternate 4).
 * PA10 - USART1_RX                 (alternate 4).
 * PA11 - USB1_DM                   (input floating).
 * PA12 - USB1_DP                   (input floating).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - EPD1_CS                   (output pushpull maximum).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_BUTTON) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_MFX_WAKEUP) | \
                                     PIN_MODE_ALTERNATE(GPIOA_TS_G1_IO3) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_TS_G1_IO4) |  \
                                     PIN_MODE_INPUT(GPIOA_PIN4) |           \
                                     PIN_MODE_OUTPUT(GPIOA_LED_RED) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_TS_G2_IO3) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_TS_G2_IO4) |  \
                                     PIN_MODE_INPUT(GPIOA_EPD1_BUSY) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_USART1_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USART1_RX) |  \
                                     PIN_MODE_INPUT(GPIOA_USB1_DM) |        \
                                     PIN_MODE_INPUT(GPIOA_USB1_DP) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_OUTPUT(GPIOA_EPD1_CS))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_BUTTON) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MFX_WAKEUP) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TS_G1_IO3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TS_G1_IO4) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_RED) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TS_G2_IO3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TS_G2_IO4) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_EPD1_BUSY) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART1_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART1_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB1_DM) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB1_DP) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_EPD1_CS))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_BUTTON) |        \
                                     PIN_OSPEED_HIGH(GPIOA_MFX_WAKEUP) |    \
                                     PIN_OSPEED_HIGH(GPIOA_TS_G1_IO3) |     \
                                     PIN_OSPEED_HIGH(GPIOA_TS_G1_IO4) |     \
                                     PIN_OSPEED_HIGH(GPIOA_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOA_LED_RED) |       \
                                     PIN_OSPEED_HIGH(GPIOA_TS_G2_IO3) |     \
                                     PIN_OSPEED_HIGH(GPIOA_TS_G2_IO4) |     \
                                     PIN_OSPEED_HIGH(GPIOA_EPD1_BUSY) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USART1_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USART1_RX) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USB1_DM) |       \
                                     PIN_OSPEED_HIGH(GPIOA_USB1_DP) |       \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_EPD1_CS))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_BUTTON) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_MFX_WAKEUP) | \
                                     PIN_PUPDR_FLOATING(GPIOA_TS_G1_IO3) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_TS_G1_IO4) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_RED) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_TS_G2_IO3) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_TS_G2_IO4) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_EPD1_BUSY) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USART1_TX) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USART1_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB1_DM) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_USB1_DP) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOA_EPD1_CS))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_BUTTON) |           \
                                     PIN_ODR_HIGH(GPIOA_MFX_WAKEUP) |       \
                                     PIN_ODR_HIGH(GPIOA_TS_G1_IO3) |        \
                                     PIN_ODR_HIGH(GPIOA_TS_G1_IO4) |        \
                                     PIN_ODR_HIGH(GPIOA_PIN4) |             \
                                     PIN_ODR_LOW(GPIOA_LED_RED) |           \
                                     PIN_ODR_HIGH(GPIOA_TS_G2_IO3) |        \
                                     PIN_ODR_HIGH(GPIOA_TS_G2_IO4) |        \
                                     PIN_ODR_HIGH(GPIOA_EPD1_BUSY) |        \
                                     PIN_ODR_HIGH(GPIOA_USART1_TX) |        \
                                     PIN_ODR_HIGH(GPIOA_USART1_RX) |        \
                                     PIN_ODR_HIGH(GPIOA_USB1_DM) |          \
                                     PIN_ODR_HIGH(GPIOA_USB1_DP) |          \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_EPD1_CS))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_BUTTON, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_MFX_WAKEUP, 0U) |    \
                                     PIN_AFIO_AF(GPIOA_TS_G1_IO3, 3U) |     \
                                     PIN_AFIO_AF(GPIOA_TS_G1_IO4, 3U) |     \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_LED_RED, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_TS_G2_IO3, 3U) |     \
                                     PIN_AFIO_AF(GPIOA_TS_G2_IO4, 3U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_EPD1_BUSY, 0U) |     \
                                     PIN_AFIO_AF(GPIOA_USART1_TX, 4U) |     \
                                     PIN_AFIO_AF(GPIOA_USART1_RX, 4U) |     \
                                     PIN_AFIO_AF(GPIOA_USB1_DM, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_USB1_DP, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_EPD1_CS, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - TS_G3_IO2                 (alternate 3).
 * PB1  - TS_G3_IO3                 (alternate 3).
 * PB2  - EPD1_RESET                (output pushpull maximum).
 * PB3  - EPD1_SCK                  (alternate 0).
 * PB4  - LED_GREEN                 (output pushpull maximum).
 * PB5  - EPD1_MOSI                 (alternate 0).
 * PB6  - NFC_IRQINN                (input pulldown).
 * PB7  - NFC_IRQOUTN               (output pushpull maximum).
 * PB8  - MFX_I2C1_SCL              (alternate 4).
 * PB9  - MFX_I2C1_SDA              (alternate 4).
 * PB10 - EPD1_PWR_ENN              (output pushpull maximum).
 * PB11 - EPD1_D_C                  (output pushpull maximum).
 * PB12 - NFC_NSS                   (output pushpull maximum).
 * PB13 - NFC_SCK                   (alternate 0).
 * PB14 - NFC_MISO                  (alternate 0).
 * PB15 - NFC_MOSI                  (alternate 0).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(GPIOB_TS_G3_IO2) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_TS_G3_IO3) |  \
                                     PIN_MODE_OUTPUT(GPIOB_EPD1_RESET) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_EPD1_SCK) |   \
                                     PIN_MODE_OUTPUT(GPIOB_LED_GREEN) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_EPD1_MOSI) |  \
                                     PIN_MODE_INPUT(GPIOB_NFC_IRQINN) |     \
                                     PIN_MODE_OUTPUT(GPIOB_NFC_IRQOUTN) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_MFX_I2C1_SCL) |\
                                     PIN_MODE_ALTERNATE(GPIOB_MFX_I2C1_SDA) |\
                                     PIN_MODE_OUTPUT(GPIOB_EPD1_PWR_ENN) |  \
                                     PIN_MODE_OUTPUT(GPIOB_EPD1_D_C) |      \
                                     PIN_MODE_OUTPUT(GPIOB_NFC_NSS) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_NFC_SCK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_NFC_MISO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_NFC_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_TS_G3_IO2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_TS_G3_IO3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_EPD1_RESET) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_EPD1_SCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_GREEN) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_EPD1_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NFC_IRQINN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NFC_IRQOUTN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_MFX_I2C1_SCL) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_MFX_I2C1_SDA) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_EPD1_PWR_ENN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_EPD1_D_C) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NFC_NSS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NFC_SCK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NFC_MISO) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NFC_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_TS_G3_IO2) |     \
                                     PIN_OSPEED_HIGH(GPIOB_TS_G3_IO3) |     \
                                     PIN_OSPEED_HIGH(GPIOB_EPD1_RESET) |    \
                                     PIN_OSPEED_HIGH(GPIOB_EPD1_SCK) |      \
                                     PIN_OSPEED_HIGH(GPIOB_LED_GREEN) |     \
                                     PIN_OSPEED_HIGH(GPIOB_EPD1_MOSI) |     \
                                     PIN_OSPEED_HIGH(GPIOB_NFC_IRQINN) |    \
                                     PIN_OSPEED_HIGH(GPIOB_NFC_IRQOUTN) |   \
                                     PIN_OSPEED_HIGH(GPIOB_MFX_I2C1_SCL) |  \
                                     PIN_OSPEED_HIGH(GPIOB_MFX_I2C1_SDA) |  \
                                     PIN_OSPEED_HIGH(GPIOB_EPD1_PWR_ENN) |  \
                                     PIN_OSPEED_HIGH(GPIOB_EPD1_D_C) |      \
                                     PIN_OSPEED_HIGH(GPIOB_NFC_NSS) |       \
                                     PIN_OSPEED_HIGH(GPIOB_NFC_SCK) |       \
                                     PIN_OSPEED_HIGH(GPIOB_NFC_MISO) |      \
                                     PIN_OSPEED_HIGH(GPIOB_NFC_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOB_TS_G3_IO2) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_TS_G3_IO3) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_EPD1_RESET) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOB_EPD1_SCK) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_GREEN) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_EPD1_MOSI) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_NFC_IRQINN) | \
                                     PIN_PUPDR_PULLDOWN(GPIOB_NFC_IRQOUTN) |\
                                     PIN_PUPDR_FLOATING(GPIOB_MFX_I2C1_SCL) |\
                                     PIN_PUPDR_FLOATING(GPIOB_MFX_I2C1_SDA) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_EPD1_PWR_ENN) |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_EPD1_D_C) |   \
                                     PIN_PUPDR_PULLUP(GPIOB_NFC_NSS) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_NFC_SCK) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_NFC_MISO) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_NFC_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_TS_G3_IO2) |        \
                                     PIN_ODR_HIGH(GPIOB_TS_G3_IO3) |        \
                                     PIN_ODR_HIGH(GPIOB_EPD1_RESET) |       \
                                     PIN_ODR_HIGH(GPIOB_EPD1_SCK) |         \
                                     PIN_ODR_LOW(GPIOB_LED_GREEN) |         \
                                     PIN_ODR_HIGH(GPIOB_EPD1_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOB_NFC_IRQINN) |       \
                                     PIN_ODR_LOW(GPIOB_NFC_IRQOUTN) |       \
                                     PIN_ODR_HIGH(GPIOB_MFX_I2C1_SCL) |     \
                                     PIN_ODR_HIGH(GPIOB_MFX_I2C1_SDA) |     \
                                     PIN_ODR_LOW(GPIOB_EPD1_PWR_ENN) |      \
                                     PIN_ODR_LOW(GPIOB_EPD1_D_C) |          \
                                     PIN_ODR_HIGH(GPIOB_NFC_NSS) |          \
                                     PIN_ODR_HIGH(GPIOB_NFC_SCK) |          \
                                     PIN_ODR_HIGH(GPIOB_NFC_MISO) |         \
                                     PIN_ODR_HIGH(GPIOB_NFC_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_TS_G3_IO2, 3U) |     \
                                     PIN_AFIO_AF(GPIOB_TS_G3_IO3, 3U) |     \
                                     PIN_AFIO_AF(GPIOB_EPD1_RESET, 0U) |    \
                                     PIN_AFIO_AF(GPIOB_EPD1_SCK, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_LED_GREEN, 0U) |     \
                                     PIN_AFIO_AF(GPIOB_EPD1_MOSI, 0U) |     \
                                     PIN_AFIO_AF(GPIOB_NFC_IRQINN, 0U) |    \
                                     PIN_AFIO_AF(GPIOB_NFC_IRQOUTN, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_MFX_I2C1_SCL, 4U) |  \
                                     PIN_AFIO_AF(GPIOB_MFX_I2C1_SDA, 4U) |  \
                                     PIN_AFIO_AF(GPIOB_EPD1_PWR_ENN, 0U) |  \
                                     PIN_AFIO_AF(GPIOB_EPD1_D_C, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_NFC_NSS, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_NFC_SCK, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_NFC_MISO, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_NFC_MOSI, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0                      (input pulldown).
 * PC1  - PIN1                      (input pulldown).
 * PC2  - PIN2                      (input pulldown).
 * PC3  - PIN3                      (input pulldown).
 * PC4  - PIN4                      (input pulldown).
 * PC5  - PIN5                      (input pulldown).
 * PC6  - PIN6                      (input pulldown).
 * PC7  - PIN7                      (input pulldown).
 * PC8  - PIN8                      (input pulldown).
 * PC9  - PIN9                      (input pulldown).
 * PC10 - PIN10                     (input pulldown).
 * PC11 - PIN11                     (input pulldown).
 * PC12 - PIN12                     (input pulldown).
 * PC13 - MFX_IRQ_OUT               (output pushpull maximum).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOC_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN12) |          \
                                     PIN_MODE_OUTPUT(GPIOC_MFX_IRQ_OUT) |   \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MFX_IRQ_OUT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOC_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOC_MFX_IRQ_OUT) |   \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_IN) |      \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOC_PIN0) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN1) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN2) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN3) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN4) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN5) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN6) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN7) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN8) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN9) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN10) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN11) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_PIN12) |      \
                                     PIN_PUPDR_PULLUP(GPIOC_MFX_IRQ_OUT) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOC_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOC_MFX_IRQ_OUT) |      \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN7, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_MFX_IRQ_OUT, 0U) |   \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pulldown).
 * PD1  - PIN1                      (input pulldown).
 * PD2  - PIN2                      (input pulldown).
 * PD3  - PIN3                      (input pulldown).
 * PD4  - PIN4                      (input pulldown).
 * PD5  - PIN5                      (input pulldown).
 * PD6  - PIN6                      (input pulldown).
 * PD7  - PIN7                      (input pulldown).
 * PD8  - PIN8                      (input pulldown).
 * PD9  - PIN9                      (input pulldown).
 * PD10 - PIN10                     (input pulldown).
 * PD11 - PIN11                     (input pulldown).
 * PD12 - PIN12                     (input pulldown).
 * PD13 - PIN13                     (input pulldown).
 * PD14 - PIN14                     (input pulldown).
 * PD15 - PIN15                     (input pulldown).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOD_PIN0) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN1) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN2) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN3) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN4) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN5) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN6) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN7) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN8) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN9) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN10) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN11) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN12) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN13) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN14) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0U))

/*
 * GPIOH setup:
 *
 * PH0  - PIN0                      (input pulldown).
 * PH1  - PIN1                      (input pulldown).
 * PH2  - PIN2                      (input pulldown).
 * PH3  - PIN3                      (input pulldown).
 * PH4  - PIN4                      (input pulldown).
 * PH5  - PIN5                      (input pulldown).
 * PH6  - PIN6                      (input pulldown).
 * PH7  - PIN7                      (input pulldown).
 * PH8  - PIN8                      (input pulldown).
 * PH9  - PIN9                      (input pulldown).
 * PH10 - PIN10                     (input pulldown).
 * PH11 - PIN11                     (input pulldown).
 * PH12 - PIN12                     (input pulldown).
 * PH13 - PIN13                     (input pulldown).
 * PH14 - PIN14                     (input pulldown).
 * PH15 - PIN15                     (input pulldown).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOH_PIN0) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN1) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN2) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN3) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN4) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN5) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN6) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN7) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN8) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN9) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN10) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN11) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN12) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN13) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN14) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

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
