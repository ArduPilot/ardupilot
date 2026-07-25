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
 * Setup for STMicroelectronics STM32L476-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32L476_DISCOVERY
#define BOARD_NAME                  "STMicroelectronics STM32L476-Discovery"

/*
 * Board oscillators-related settings.
 * NOTE: HSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                0U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#undef STM32L476xx
#define STM32L476xx

/*
 * IO pins assignments.
 */
#define GPIOA_JOY_CENTER            0U
#define GPIOA_JOY_LEFT              1U
#define GPIOA_JOY_RIGHT             2U
#define GPIOA_JOY_UP                3U
#define GPIOA_MFX_WAKEUP            4U
#define GPIOA_JOY_DOWN              5U
#define GPIOA_LCD_SEG23             6U
#define GPIOA_LCD_SEG0              7U
#define GPIOA_LCD_COM0              8U
#define GPIOA_LCD_COM1              9U
#define GPIOA_LCD_COM2              10U
#define GPIOA_OTG_FS_DM             11U
#define GPIOA_OTG_FS_DP             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15                 15U

#define GPIOB_LCD_SEG21             0U
#define GPIOB_LCD_SEG2              1U
#define GPIOB_LED_RED               2U
#define GPIOB_SWO                   3U
#define GPIOB_REG_ON_3V3            3U
#define GPIOB_LCD_SEG11             4U
#define GPIOB_LCD_SEG12             5U
#define GPIOB_I2C1_SCL              6U
#define GPIOB_I2C1_SDA              7U
#define GPIOB_GYRO_INT2             8U
#define GPIOB_LCD_COM3              9U
#define GPIOB_MFX_I2C_SCL           10U
#define GPIOB_I2C2_SCL              10U
#define GPIOB_MFX_I2C_SDA           11U
#define GPIOB_I2C2_SDA              11U
#define GPIOB_LCD_SEG20             12U
#define GPIOB_LCD_SEG3              13U
#define GPIOB_LCD_SEG19             14U
#define GPIOB_LCD_SEG4              15U

#define GPIOC_MAG_CS                0U
#define GPIOC_MAG_INT               1U
#define GPIOC_MAG_DRDY              2U
#define GPIOC_LCD_VLCD              3U
#define GPIOC_LCD_SEG22             4U
#define GPIOC_LCD_SEG1              5U
#define GPIOC_LCD_SEG14             6U
#define GPIOC_LCD_SEG9              7U
#define GPIOC_LCD_SEG13             8U
#define GPIOC_OTG_FS_PSON           9U
#define GPIOC_OTG_FS_OVCURR         10U
#define GPIOC_OTG_FS_VBUS           11U
#define GPIOC_OTG_FS_ID             12U
#define GPIOC_MFX_IRQ_OUT           13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_EXT_RST               0U
#define GPIOD_MEMS_SCK              1U
#define GPIOD_GYRO_INT1             2U
#define GPIOD_MEMS_MISO             3U
#define GPIOD_MEMS_MOSI             4U
#define GPIOD_USART_TX              5U
#define GPIOD_USART_RX              6U
#define GPIOD_GYRO_CS               7U
#define GPIOD_LCD_SEG18             8U
#define GPIOD_LCD_SEG5              9U
#define GPIOD_LCD_SEG17             10U
#define GPIOD_LCD_SEG6              11U
#define GPIOD_LCD_SEG16             12U
#define GPIOD_LCD_SEG7              13U
#define GPIOD_LCD_SEG15             14U
#define GPIOD_LCD_SEG8              15U

#define GPIOE_XL_CS                 0U
#define GPIOE_XL_INT                1U
#define GPIOE_SAI1_MCK              2U
#define GPIOE_AUDIO_RST             3U
#define GPIOE_SAI1_FS               4U
#define GPIOE_SAI1_SCK              5U
#define GPIOE_SAI1_SD               6U
#define GPIOE_AUDIO_DIN             7U
#define GPIOE_LED_GREEN             8U
#define GPIOE_AUDIO_CLK             9U
#define GPIOE_QSPI_CLK              10U
#define GPIOE_QSPI_CS               11U
#define GPIOE_QSPI_D0               12U
#define GPIOE_QSPI_D1               13U
#define GPIOE_QSPI_D2               14U
#define GPIOE_QSPI_D3               15U

#define GPIOF_PIN0                  0U
#define GPIOF_PIN1                  1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_PIN2                  2U
#define GPIOG_PIN3                  3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_PIN6                  6U
#define GPIOG_PIN7                  7U
#define GPIOG_PIN8                  8U
#define GPIOG_PIN9                  9U
#define GPIOG_PIN10                 10U
#define GPIOG_PIN11                 11U
#define GPIOG_PIN12                 12U
#define GPIOG_PIN13                 13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
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
#define LINE_JOY_CENTER             PAL_LINE(GPIOA, 0U)
#define LINE_JOY_LEFT               PAL_LINE(GPIOA, 1U)
#define LINE_JOY_RIGHT              PAL_LINE(GPIOA, 2U)
#define LINE_JOY_UP                 PAL_LINE(GPIOA, 3U)
#define LINE_MFX_WAKEUP             PAL_LINE(GPIOA, 4U)
#define LINE_JOY_DOWN               PAL_LINE(GPIOA, 5U)
#define LINE_LCD_SEG23              PAL_LINE(GPIOA, 6U)
#define LINE_LCD_SEG0               PAL_LINE(GPIOA, 7U)
#define LINE_LCD_COM0               PAL_LINE(GPIOA, 8U)
#define LINE_LCD_COM1               PAL_LINE(GPIOA, 9U)
#define LINE_LCD_COM2               PAL_LINE(GPIOA, 10U)
#define LINE_OTG_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP              PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_LCD_SEG21              PAL_LINE(GPIOB, 0U)
#define LINE_LCD_SEG2               PAL_LINE(GPIOB, 1U)
#define LINE_LED_RED                PAL_LINE(GPIOB, 2U)
#define LINE_SWO                    PAL_LINE(GPIOB, 3U)
#define LINE_REG_ON_3V3             PAL_LINE(GPIOB, 3U)
#define LINE_LCD_SEG11              PAL_LINE(GPIOB, 4U)
#define LINE_LCD_SEG12              PAL_LINE(GPIOB, 5U)
#define LINE_I2C1_SCL               PAL_LINE(GPIOB, 6U)
#define LINE_I2C1_SDA               PAL_LINE(GPIOB, 7U)
#define LINE_GYRO_INT2              PAL_LINE(GPIOB, 8U)
#define LINE_LCD_COM3               PAL_LINE(GPIOB, 9U)
#define LINE_MFX_I2C_SCL            PAL_LINE(GPIOB, 10U)
#define LINE_I2C2_SCL               PAL_LINE(GPIOB, 10U)
#define LINE_MFX_I2C_SDA            PAL_LINE(GPIOB, 11U)
#define LINE_I2C2_SDA               PAL_LINE(GPIOB, 11U)
#define LINE_LCD_SEG20              PAL_LINE(GPIOB, 12U)
#define LINE_LCD_SEG3               PAL_LINE(GPIOB, 13U)
#define LINE_LCD_SEG19              PAL_LINE(GPIOB, 14U)
#define LINE_LCD_SEG4               PAL_LINE(GPIOB, 15U)
#define LINE_MAG_CS                 PAL_LINE(GPIOC, 0U)
#define LINE_MAG_INT                PAL_LINE(GPIOC, 1U)
#define LINE_MAG_DRDY               PAL_LINE(GPIOC, 2U)
#define LINE_LCD_VLCD               PAL_LINE(GPIOC, 3U)
#define LINE_LCD_SEG22              PAL_LINE(GPIOC, 4U)
#define LINE_LCD_SEG1               PAL_LINE(GPIOC, 5U)
#define LINE_LCD_SEG14              PAL_LINE(GPIOC, 6U)
#define LINE_LCD_SEG9               PAL_LINE(GPIOC, 7U)
#define LINE_LCD_SEG13              PAL_LINE(GPIOC, 8U)
#define LINE_OTG_FS_PSON            PAL_LINE(GPIOC, 9U)
#define LINE_OTG_FS_OVCURR          PAL_LINE(GPIOC, 10U)
#define LINE_OTG_FS_VBUS            PAL_LINE(GPIOC, 11U)
#define LINE_OTG_FS_ID              PAL_LINE(GPIOC, 12U)
#define LINE_MFX_IRQ_OUT            PAL_LINE(GPIOC, 13U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)
#define LINE_EXT_RST                PAL_LINE(GPIOD, 0U)
#define LINE_MEMS_SCK               PAL_LINE(GPIOD, 1U)
#define LINE_GYRO_INT1              PAL_LINE(GPIOD, 2U)
#define LINE_MEMS_MISO              PAL_LINE(GPIOD, 3U)
#define LINE_MEMS_MOSI              PAL_LINE(GPIOD, 4U)
#define LINE_USART_TX               PAL_LINE(GPIOD, 5U)
#define LINE_USART_RX               PAL_LINE(GPIOD, 6U)
#define LINE_GYRO_CS                PAL_LINE(GPIOD, 7U)
#define LINE_LCD_SEG18              PAL_LINE(GPIOD, 8U)
#define LINE_LCD_SEG5               PAL_LINE(GPIOD, 9U)
#define LINE_LCD_SEG17              PAL_LINE(GPIOD, 10U)
#define LINE_LCD_SEG6               PAL_LINE(GPIOD, 11U)
#define LINE_LCD_SEG16              PAL_LINE(GPIOD, 12U)
#define LINE_LCD_SEG7               PAL_LINE(GPIOD, 13U)
#define LINE_LCD_SEG15              PAL_LINE(GPIOD, 14U)
#define LINE_LCD_SEG8               PAL_LINE(GPIOD, 15U)
#define LINE_XL_CS                  PAL_LINE(GPIOE, 0U)
#define LINE_XL_INT                 PAL_LINE(GPIOE, 1U)
#define LINE_SAI1_MCK               PAL_LINE(GPIOE, 2U)
#define LINE_AUDIO_RST              PAL_LINE(GPIOE, 3U)
#define LINE_SAI1_FS                PAL_LINE(GPIOE, 4U)
#define LINE_SAI1_SCK               PAL_LINE(GPIOE, 5U)
#define LINE_SAI1_SD                PAL_LINE(GPIOE, 6U)
#define LINE_AUDIO_DIN              PAL_LINE(GPIOE, 7U)
#define LINE_LED_GREEN              PAL_LINE(GPIOE, 8U)
#define LINE_AUDIO_CLK              PAL_LINE(GPIOE, 9U)
#define LINE_QSPI_CLK               PAL_LINE(GPIOE, 10U)
#define LINE_QSPI_CS                PAL_LINE(GPIOE, 11U)
#define LINE_QSPI_D0                PAL_LINE(GPIOE, 12U)
#define LINE_QSPI_D1                PAL_LINE(GPIOE, 13U)
#define LINE_QSPI_D2                PAL_LINE(GPIOE, 14U)
#define LINE_QSPI_D3                PAL_LINE(GPIOE, 15U)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)

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
#define PIN_ASCR_DISABLED(n)        (0U << (n))
#define PIN_ASCR_ENABLED(n)         (1U << (n))
#define PIN_LOCKR_DISABLED(n)       (0U << (n))
#define PIN_LOCKR_ENABLED(n)        (1U << (n))

/*
 * GPIOA setup:
 *
 * PA0  - JOY_CENTER                (input pulldown).
 * PA1  - JOY_LEFT                  (input pulldown).
 * PA2  - JOY_RIGHT                 (input pulldown).
 * PA3  - JOY_UP                    (input pulldown).
 * PA4  - MFX_WAKEUP                (output pushpull maximum).
 * PA5  - JOY_DOWN                  (input pulldown).
 * PA6  - LCD_SEG23                 (alternate 11).
 * PA7  - LCD_SEG0                  (alternate 11).
 * PA8  - LCD_COM0                  (alternate 11).
 * PA9  - LCD_COM1                  (alternate 11).
 * PA10 - LCD_COM2                  (alternate 11).
 * PA11 - OTG_FS_DM                 (alternate 10).
 * PA12 - OTG_FS_DP                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - PIN15                     (analog).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_JOY_CENTER) |     \
                                     PIN_MODE_INPUT(GPIOA_JOY_LEFT) |       \
                                     PIN_MODE_INPUT(GPIOA_JOY_RIGHT) |      \
                                     PIN_MODE_INPUT(GPIOA_JOY_UP) |         \
                                     PIN_MODE_OUTPUT(GPIOA_MFX_WAKEUP) |    \
                                     PIN_MODE_INPUT(GPIOA_JOY_DOWN) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_SEG23) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_SEG0) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_COM0) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_COM1) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_COM2) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_ANALOG(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_JOY_CENTER) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JOY_LEFT) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JOY_RIGHT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JOY_UP) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MFX_WAKEUP) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JOY_DOWN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_SEG23) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_SEG0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_COM0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_COM1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_COM2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_JOY_CENTER) |    \
                                     PIN_OSPEED_HIGH(GPIOA_JOY_LEFT) |      \
                                     PIN_OSPEED_HIGH(GPIOA_JOY_RIGHT) |     \
                                     PIN_OSPEED_HIGH(GPIOA_JOY_UP) |        \
                                     PIN_OSPEED_HIGH(GPIOA_MFX_WAKEUP) |    \
                                     PIN_OSPEED_HIGH(GPIOA_JOY_DOWN) |      \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_SEG23) |     \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_SEG0) |      \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_COM0) |      \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_COM1) |      \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_COM2) |      \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DM) |     \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DP) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOA_JOY_CENTER) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JOY_LEFT) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JOY_RIGHT) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JOY_UP) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_MFX_WAKEUP) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JOY_DOWN) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_SEG23) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_SEG0) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_COM0) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_COM1) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_COM2) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_JOY_CENTER) |       \
                                     PIN_ODR_HIGH(GPIOA_JOY_LEFT) |         \
                                     PIN_ODR_HIGH(GPIOA_JOY_RIGHT) |        \
                                     PIN_ODR_HIGH(GPIOA_JOY_UP) |           \
                                     PIN_ODR_HIGH(GPIOA_MFX_WAKEUP) |       \
                                     PIN_ODR_HIGH(GPIOA_JOY_DOWN) |         \
                                     PIN_ODR_HIGH(GPIOA_LCD_SEG23) |        \
                                     PIN_ODR_HIGH(GPIOA_LCD_SEG0) |         \
                                     PIN_ODR_HIGH(GPIOA_LCD_COM0) |         \
                                     PIN_ODR_HIGH(GPIOA_LCD_COM1) |         \
                                     PIN_ODR_HIGH(GPIOA_LCD_COM2) |         \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |        \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |        \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_JOY_CENTER, 0U) |    \
                                     PIN_AFIO_AF(GPIOA_JOY_LEFT, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_JOY_RIGHT, 0U) |     \
                                     PIN_AFIO_AF(GPIOA_JOY_UP, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_MFX_WAKEUP, 0U) |    \
                                     PIN_AFIO_AF(GPIOA_JOY_DOWN, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_LCD_SEG23, 11U) |    \
                                     PIN_AFIO_AF(GPIOA_LCD_SEG0, 11U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_LCD_COM0, 11U) |     \
                                     PIN_AFIO_AF(GPIOA_LCD_COM1, 11U) |     \
                                     PIN_AFIO_AF(GPIOA_LCD_COM2, 11U) |     \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0U))
#define VAL_GPIOA_ASCR              (PIN_ASCR_DISABLED(GPIOA_JOY_CENTER) |  \
                                     PIN_ASCR_DISABLED(GPIOA_JOY_LEFT) |    \
                                     PIN_ASCR_DISABLED(GPIOA_JOY_RIGHT) |   \
                                     PIN_ASCR_DISABLED(GPIOA_JOY_UP) |      \
                                     PIN_ASCR_DISABLED(GPIOA_MFX_WAKEUP) |  \
                                     PIN_ASCR_DISABLED(GPIOA_JOY_DOWN) |    \
                                     PIN_ASCR_DISABLED(GPIOA_LCD_SEG23) |   \
                                     PIN_ASCR_DISABLED(GPIOA_LCD_SEG0) |    \
                                     PIN_ASCR_DISABLED(GPIOA_LCD_COM0) |    \
                                     PIN_ASCR_DISABLED(GPIOA_LCD_COM1) |    \
                                     PIN_ASCR_DISABLED(GPIOA_LCD_COM2) |    \
                                     PIN_ASCR_DISABLED(GPIOA_OTG_FS_DM) |   \
                                     PIN_ASCR_DISABLED(GPIOA_OTG_FS_DP) |   \
                                     PIN_ASCR_DISABLED(GPIOA_SWDIO) |       \
                                     PIN_ASCR_DISABLED(GPIOA_SWCLK) |       \
                                     PIN_ASCR_DISABLED(GPIOA_PIN15))
#define VAL_GPIOA_LOCKR             (PIN_LOCKR_DISABLED(GPIOA_JOY_CENTER) | \
                                     PIN_LOCKR_DISABLED(GPIOA_JOY_LEFT) |   \
                                     PIN_LOCKR_DISABLED(GPIOA_JOY_RIGHT) |  \
                                     PIN_LOCKR_DISABLED(GPIOA_JOY_UP) |     \
                                     PIN_LOCKR_DISABLED(GPIOA_MFX_WAKEUP) | \
                                     PIN_LOCKR_DISABLED(GPIOA_JOY_DOWN) |   \
                                     PIN_LOCKR_DISABLED(GPIOA_LCD_SEG23) |  \
                                     PIN_LOCKR_DISABLED(GPIOA_LCD_SEG0) |   \
                                     PIN_LOCKR_DISABLED(GPIOA_LCD_COM0) |   \
                                     PIN_LOCKR_DISABLED(GPIOA_LCD_COM1) |   \
                                     PIN_LOCKR_DISABLED(GPIOA_LCD_COM2) |   \
                                     PIN_LOCKR_DISABLED(GPIOA_OTG_FS_DM) |  \
                                     PIN_LOCKR_DISABLED(GPIOA_OTG_FS_DP) |  \
                                     PIN_LOCKR_DISABLED(GPIOA_SWDIO) |      \
                                     PIN_LOCKR_DISABLED(GPIOA_SWCLK) |      \
                                     PIN_LOCKR_DISABLED(GPIOA_PIN15))

/*
 * GPIOB setup:
 *
 * PB0  - LCD_SEG21                 (alternate 11).
 * PB1  - LCD_SEG2                  (alternate 11).
 * PB2  - LED_RED                   (output pushpull maximum).
 * PB3  - SWO REG_ON_3V3            (analog).
 * PB4  - LCD_SEG11                 (alternate 11).
 * PB5  - LCD_SEG12                 (alternate 11).
 * PB6  - I2C1_SCL                  (alternate 4).
 * PB7  - I2C1_SDA                  (alternate 4).
 * PB8  - GYRO_INT2                 (input pulldown).
 * PB9  - LCD_COM3                  (alternate 11).
 * PB10 - MFX_I2C_SCL I2C2_SCL      (alternate 4).
 * PB11 - MFX_I2C_SDA I2C2_SDA      (alternate 4).
 * PB12 - LCD_SEG20                 (alternate 11).
 * PB13 - LCD_SEG3                  (alternate 11).
 * PB14 - LCD_SEG19                 (alternate 11).
 * PB15 - LCD_SEG4                  (alternate 11).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(GPIOB_LCD_SEG21) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_SEG2) |   \
                                     PIN_MODE_OUTPUT(GPIOB_LED_RED) |       \
                                     PIN_MODE_ANALOG(GPIOB_SWO) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_SEG11) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_SEG12) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) |   \
                                     PIN_MODE_INPUT(GPIOB_GYRO_INT2) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_COM3) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_MFX_I2C_SCL) |\
                                     PIN_MODE_ALTERNATE(GPIOB_MFX_I2C_SDA) |\
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_SEG20) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_SEG3) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_SEG19) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_SEG4))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_LCD_SEG21) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_SEG2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_RED) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_SEG11) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_SEG12) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2C1_SCL) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2C1_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GYRO_INT2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_COM3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MFX_I2C_SCL) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_MFX_I2C_SDA) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_SEG20) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_SEG3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_SEG19) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_SEG4))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_LCD_SEG21) |     \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_SEG2) |      \
                                     PIN_OSPEED_HIGH(GPIOB_LED_RED) |       \
                                     PIN_OSPEED_HIGH(GPIOB_SWO) |           \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_SEG11) |     \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_SEG12) |     \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCL) |      \
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOB_GYRO_INT2) |     \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_COM3) |      \
                                     PIN_OSPEED_HIGH(GPIOB_MFX_I2C_SCL) |   \
                                     PIN_OSPEED_HIGH(GPIOB_MFX_I2C_SDA) |   \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_SEG20) |     \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_SEG3) |      \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_SEG19) |     \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_SEG4))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_LCD_SEG21) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_SEG2) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_RED) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_SWO) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_SEG11) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_SEG12) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOB_GYRO_INT2) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_COM3) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_MFX_I2C_SCL) |\
                                     PIN_PUPDR_FLOATING(GPIOB_MFX_I2C_SDA) |\
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_SEG20) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_SEG3) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_SEG19) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_SEG4))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_LCD_SEG21) |        \
                                     PIN_ODR_HIGH(GPIOB_LCD_SEG2) |         \
                                     PIN_ODR_LOW(GPIOB_LED_RED) |           \
                                     PIN_ODR_HIGH(GPIOB_SWO) |              \
                                     PIN_ODR_HIGH(GPIOB_LCD_SEG11) |        \
                                     PIN_ODR_HIGH(GPIOB_LCD_SEG12) |        \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |         \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |         \
                                     PIN_ODR_HIGH(GPIOB_GYRO_INT2) |        \
                                     PIN_ODR_HIGH(GPIOB_LCD_COM3) |         \
                                     PIN_ODR_HIGH(GPIOB_MFX_I2C_SCL) |      \
                                     PIN_ODR_HIGH(GPIOB_MFX_I2C_SDA) |      \
                                     PIN_ODR_HIGH(GPIOB_LCD_SEG20) |        \
                                     PIN_ODR_HIGH(GPIOB_LCD_SEG3) |         \
                                     PIN_ODR_HIGH(GPIOB_LCD_SEG19) |        \
                                     PIN_ODR_HIGH(GPIOB_LCD_SEG4))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_LCD_SEG21, 11U) |    \
                                     PIN_AFIO_AF(GPIOB_LCD_SEG2, 11U) |     \
                                     PIN_AFIO_AF(GPIOB_LED_RED, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_SWO, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_LCD_SEG11, 11U) |    \
                                     PIN_AFIO_AF(GPIOB_LCD_SEG12, 11U) |    \
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 4U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_GYRO_INT2, 0U) |     \
                                     PIN_AFIO_AF(GPIOB_LCD_COM3, 11U) |     \
                                     PIN_AFIO_AF(GPIOB_MFX_I2C_SCL, 4U) |   \
                                     PIN_AFIO_AF(GPIOB_MFX_I2C_SDA, 4U) |   \
                                     PIN_AFIO_AF(GPIOB_LCD_SEG20, 11U) |    \
                                     PIN_AFIO_AF(GPIOB_LCD_SEG3, 11U) |     \
                                     PIN_AFIO_AF(GPIOB_LCD_SEG19, 11U) |    \
                                     PIN_AFIO_AF(GPIOB_LCD_SEG4, 11U))
#define VAL_GPIOB_ASCR              (PIN_ASCR_DISABLED(GPIOB_LCD_SEG21) |   \
                                     PIN_ASCR_DISABLED(GPIOB_LCD_SEG2) |    \
                                     PIN_ASCR_DISABLED(GPIOB_LED_RED) |     \
                                     PIN_ASCR_ENABLED(GPIOB_SWO) |          \
                                     PIN_ASCR_DISABLED(GPIOB_LCD_SEG11) |   \
                                     PIN_ASCR_DISABLED(GPIOB_LCD_SEG12) |   \
                                     PIN_ASCR_DISABLED(GPIOB_I2C1_SCL) |    \
                                     PIN_ASCR_DISABLED(GPIOB_I2C1_SDA) |    \
                                     PIN_ASCR_DISABLED(GPIOB_GYRO_INT2) |   \
                                     PIN_ASCR_DISABLED(GPIOB_LCD_COM3) |    \
                                     PIN_ASCR_DISABLED(GPIOB_MFX_I2C_SCL) | \
                                     PIN_ASCR_DISABLED(GPIOB_MFX_I2C_SDA) | \
                                     PIN_ASCR_DISABLED(GPIOB_LCD_SEG20) |   \
                                     PIN_ASCR_DISABLED(GPIOB_LCD_SEG3) |    \
                                     PIN_ASCR_DISABLED(GPIOB_LCD_SEG19) |   \
                                     PIN_ASCR_DISABLED(GPIOB_LCD_SEG4))
#define VAL_GPIOB_LOCKR             (PIN_LOCKR_DISABLED(GPIOB_LCD_SEG21) |  \
                                     PIN_LOCKR_DISABLED(GPIOB_LCD_SEG2) |   \
                                     PIN_LOCKR_DISABLED(GPIOB_LED_RED) |    \
                                     PIN_LOCKR_DISABLED(GPIOB_SWO) |        \
                                     PIN_LOCKR_DISABLED(GPIOB_LCD_SEG11) |  \
                                     PIN_LOCKR_DISABLED(GPIOB_LCD_SEG12) |  \
                                     PIN_LOCKR_DISABLED(GPIOB_I2C1_SCL) |   \
                                     PIN_LOCKR_DISABLED(GPIOB_I2C1_SDA) |   \
                                     PIN_LOCKR_DISABLED(GPIOB_GYRO_INT2) |  \
                                     PIN_LOCKR_DISABLED(GPIOB_LCD_COM3) |   \
                                     PIN_LOCKR_DISABLED(GPIOB_MFX_I2C_SCL) |\
                                     PIN_LOCKR_DISABLED(GPIOB_MFX_I2C_SDA) |\
                                     PIN_LOCKR_DISABLED(GPIOB_LCD_SEG20) |  \
                                     PIN_LOCKR_DISABLED(GPIOB_LCD_SEG3) |   \
                                     PIN_LOCKR_DISABLED(GPIOB_LCD_SEG19) |  \
                                     PIN_LOCKR_DISABLED(GPIOB_LCD_SEG4))

/*
 * GPIOC setup:
 *
 * PC0  - MAG_CS                    (output pushpull maximum).
 * PC1  - MAG_INT                   (input pulldown).
 * PC2  - MAG_DRDY                  (input pulldown).
 * PC3  - LCD_VLCD                  (alternate 11).
 * PC4  - LCD_SEG22                 (alternate 11).
 * PC5  - LCD_SEG1                  (alternate 11).
 * PC6  - LCD_SEG14                 (alternate 11).
 * PC7  - LCD_SEG9                  (alternate 11).
 * PC8  - LCD_SEG13                 (alternate 11).
 * PC9  - OTG_FS_PSON               (output pushpull maximum).
 * PC10 - OTG_FS_OVCURR             (input floating).
 * PC11 - OTG_FS_VBUS               (input floating).
 * PC12 - OTG_FS_ID                 (alternate 10).
 * PC13 - MFX_IRQ_OUT               (input pulldown).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_MAG_CS) |        \
                                     PIN_MODE_INPUT(GPIOC_MAG_INT) |        \
                                     PIN_MODE_INPUT(GPIOC_MAG_DRDY) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_LCD_VLCD) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_LCD_SEG22) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_LCD_SEG1) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_LCD_SEG14) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_LCD_SEG9) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_LCD_SEG13) |  \
                                     PIN_MODE_OUTPUT(GPIOC_OTG_FS_PSON) |   \
                                     PIN_MODE_INPUT(GPIOC_OTG_FS_OVCURR) |  \
                                     PIN_MODE_INPUT(GPIOC_OTG_FS_VBUS) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_OTG_FS_ID) |  \
                                     PIN_MODE_INPUT(GPIOC_MFX_IRQ_OUT) |    \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_MAG_CS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MAG_INT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MAG_DRDY) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LCD_VLCD) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LCD_SEG22) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LCD_SEG1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LCD_SEG14) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LCD_SEG9) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LCD_SEG13) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OTG_FS_PSON) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OTG_FS_OVCURR) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OTG_FS_VBUS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OTG_FS_ID) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MFX_IRQ_OUT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_MAG_CS) |        \
                                     PIN_OSPEED_HIGH(GPIOC_MAG_INT) |       \
                                     PIN_OSPEED_HIGH(GPIOC_MAG_DRDY) |      \
                                     PIN_OSPEED_HIGH(GPIOC_LCD_VLCD) |      \
                                     PIN_OSPEED_HIGH(GPIOC_LCD_SEG22) |     \
                                     PIN_OSPEED_HIGH(GPIOC_LCD_SEG1) |      \
                                     PIN_OSPEED_HIGH(GPIOC_LCD_SEG14) |     \
                                     PIN_OSPEED_HIGH(GPIOC_LCD_SEG9) |      \
                                     PIN_OSPEED_HIGH(GPIOC_LCD_SEG13) |     \
                                     PIN_OSPEED_HIGH(GPIOC_OTG_FS_PSON) |   \
                                     PIN_OSPEED_HIGH(GPIOC_OTG_FS_OVCURR) | \
                                     PIN_OSPEED_HIGH(GPIOC_OTG_FS_VBUS) |   \
                                     PIN_OSPEED_HIGH(GPIOC_OTG_FS_ID) |     \
                                     PIN_OSPEED_HIGH(GPIOC_MFX_IRQ_OUT) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_IN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(GPIOC_MAG_CS) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOC_MAG_INT) |    \
                                     PIN_PUPDR_PULLDOWN(GPIOC_MAG_DRDY) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_LCD_VLCD) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_LCD_SEG22) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_LCD_SEG1) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_LCD_SEG14) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_LCD_SEG9) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_LCD_SEG13) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_OTG_FS_PSON) |\
                                     PIN_PUPDR_FLOATING(GPIOC_OTG_FS_OVCURR) |\
                                     PIN_PUPDR_FLOATING(GPIOC_OTG_FS_VBUS) |\
                                     PIN_PUPDR_FLOATING(GPIOC_OTG_FS_ID) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOC_MFX_IRQ_OUT) |\
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_MAG_CS) |           \
                                     PIN_ODR_HIGH(GPIOC_MAG_INT) |          \
                                     PIN_ODR_HIGH(GPIOC_MAG_DRDY) |         \
                                     PIN_ODR_HIGH(GPIOC_LCD_VLCD) |         \
                                     PIN_ODR_HIGH(GPIOC_LCD_SEG22) |        \
                                     PIN_ODR_HIGH(GPIOC_LCD_SEG1) |         \
                                     PIN_ODR_HIGH(GPIOC_LCD_SEG14) |        \
                                     PIN_ODR_HIGH(GPIOC_LCD_SEG9) |         \
                                     PIN_ODR_HIGH(GPIOC_LCD_SEG13) |        \
                                     PIN_ODR_HIGH(GPIOC_OTG_FS_PSON) |      \
                                     PIN_ODR_HIGH(GPIOC_OTG_FS_OVCURR) |    \
                                     PIN_ODR_HIGH(GPIOC_OTG_FS_VBUS) |      \
                                     PIN_ODR_HIGH(GPIOC_OTG_FS_ID) |        \
                                     PIN_ODR_HIGH(GPIOC_MFX_IRQ_OUT) |      \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_MAG_CS, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_MAG_INT, 0U) |       \
                                     PIN_AFIO_AF(GPIOC_MAG_DRDY, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_LCD_VLCD, 11U) |     \
                                     PIN_AFIO_AF(GPIOC_LCD_SEG22, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_LCD_SEG1, 11U) |     \
                                     PIN_AFIO_AF(GPIOC_LCD_SEG14, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_LCD_SEG9, 11U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_LCD_SEG13, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_OTG_FS_PSON, 0U) |   \
                                     PIN_AFIO_AF(GPIOC_OTG_FS_OVCURR, 0U) | \
                                     PIN_AFIO_AF(GPIOC_OTG_FS_VBUS, 0U) |   \
                                     PIN_AFIO_AF(GPIOC_OTG_FS_ID, 10U) |    \
                                     PIN_AFIO_AF(GPIOC_MFX_IRQ_OUT, 0U) |   \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))
#define VAL_GPIOC_ASCR              (PIN_ASCR_DISABLED(GPIOC_MAG_CS) |      \
                                     PIN_ASCR_DISABLED(GPIOC_MAG_INT) |     \
                                     PIN_ASCR_DISABLED(GPIOC_MAG_DRDY) |    \
                                     PIN_ASCR_DISABLED(GPIOC_LCD_VLCD) |    \
                                     PIN_ASCR_DISABLED(GPIOC_LCD_SEG22) |   \
                                     PIN_ASCR_DISABLED(GPIOC_LCD_SEG1) |    \
                                     PIN_ASCR_DISABLED(GPIOC_LCD_SEG14) |   \
                                     PIN_ASCR_DISABLED(GPIOC_LCD_SEG9) |    \
                                     PIN_ASCR_DISABLED(GPIOC_LCD_SEG13) |   \
                                     PIN_ASCR_DISABLED(GPIOC_OTG_FS_PSON) | \
                                     PIN_ASCR_DISABLED(GPIOC_OTG_FS_OVCURR) |\
                                     PIN_ASCR_DISABLED(GPIOC_OTG_FS_VBUS) | \
                                     PIN_ASCR_DISABLED(GPIOC_OTG_FS_ID) |   \
                                     PIN_ASCR_DISABLED(GPIOC_MFX_IRQ_OUT) | \
                                     PIN_ASCR_DISABLED(GPIOC_OSC32_IN) |    \
                                     PIN_ASCR_DISABLED(GPIOC_OSC32_OUT))
#define VAL_GPIOC_LOCKR             (PIN_LOCKR_DISABLED(GPIOC_MAG_CS) |     \
                                     PIN_LOCKR_DISABLED(GPIOC_MAG_INT) |    \
                                     PIN_LOCKR_DISABLED(GPIOC_MAG_DRDY) |   \
                                     PIN_LOCKR_DISABLED(GPIOC_LCD_VLCD) |   \
                                     PIN_LOCKR_DISABLED(GPIOC_LCD_SEG22) |  \
                                     PIN_LOCKR_DISABLED(GPIOC_LCD_SEG1) |   \
                                     PIN_LOCKR_DISABLED(GPIOC_LCD_SEG14) |  \
                                     PIN_LOCKR_DISABLED(GPIOC_LCD_SEG9) |   \
                                     PIN_LOCKR_DISABLED(GPIOC_LCD_SEG13) |  \
                                     PIN_LOCKR_DISABLED(GPIOC_OTG_FS_PSON) |\
                                     PIN_LOCKR_DISABLED(GPIOC_OTG_FS_OVCURR) |\
                                     PIN_LOCKR_DISABLED(GPIOC_OTG_FS_VBUS) |\
                                     PIN_LOCKR_DISABLED(GPIOC_OTG_FS_ID) |  \
                                     PIN_LOCKR_DISABLED(GPIOC_MFX_IRQ_OUT) |\
                                     PIN_LOCKR_DISABLED(GPIOC_OSC32_IN) |   \
                                     PIN_LOCKR_DISABLED(GPIOC_OSC32_OUT))

/*
 * GPIOD setup:
 *
 * PD0  - EXT_RST                   (output pushpull maximum).
 * PD1  - MEMS_SCK                  (alternate 5).
 * PD2  - GYRO_INT1                 (input pulldown).
 * PD3  - MEMS_MISO                 (alternate 5).
 * PD4  - MEMS_MOSI                 (alternate 5).
 * PD5  - USART_TX                  (alternate 7).
 * PD6  - USART_RX                  (alternate 7).
 * PD7  - GYRO_CS                   (output pushpull maximum).
 * PD8  - LCD_SEG18                 (alternate 11).
 * PD9  - LCD_SEG5                  (alternate 11).
 * PD10 - LCD_SEG17                 (alternate 11).
 * PD11 - LCD_SEG6                  (alternate 11).
 * PD12 - LCD_SEG16                 (alternate 11).
 * PD13 - LCD_SEG7                  (alternate 11).
 * PD14 - LCD_SEG15                 (alternate 11).
 * PD15 - LCD_SEG8                  (alternate 11).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_OUTPUT(GPIOD_EXT_RST) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_MEMS_SCK) |   \
                                     PIN_MODE_INPUT(GPIOD_GYRO_INT1) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_MEMS_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_MEMS_MOSI) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_USART_TX) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_USART_RX) |   \
                                     PIN_MODE_OUTPUT(GPIOD_GYRO_CS) |       \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_SEG18) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_SEG5) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_SEG17) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_SEG6) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_SEG16) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_SEG7) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_SEG15) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_SEG8))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_EXT_RST) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEMS_SCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_GYRO_INT1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEMS_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_MEMS_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART_RX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_GYRO_CS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_SEG18) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_SEG5) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_SEG17) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_SEG6) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_SEG16) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_SEG7) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_SEG15) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_SEG8))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_EXT_RST) |       \
                                     PIN_OSPEED_HIGH(GPIOD_MEMS_SCK) |      \
                                     PIN_OSPEED_HIGH(GPIOD_GYRO_INT1) |     \
                                     PIN_OSPEED_HIGH(GPIOD_MEMS_MISO) |     \
                                     PIN_OSPEED_HIGH(GPIOD_MEMS_MOSI) |     \
                                     PIN_OSPEED_HIGH(GPIOD_USART_TX) |      \
                                     PIN_OSPEED_HIGH(GPIOD_USART_RX) |      \
                                     PIN_OSPEED_HIGH(GPIOD_GYRO_CS) |       \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_SEG18) |     \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_SEG5) |      \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_SEG17) |     \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_SEG6) |      \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_SEG16) |     \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_SEG7) |      \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_SEG15) |     \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_SEG8))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOD_EXT_RST) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_MEMS_SCK) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOD_GYRO_INT1) |  \
                                     PIN_PUPDR_PULLUP(GPIOD_MEMS_MISO) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_MEMS_MOSI) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_USART_TX) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_USART_RX) |   \
                                     PIN_PUPDR_PULLUP(GPIOD_GYRO_CS) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_SEG18) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_SEG5) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_SEG17) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_SEG6) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_SEG16) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_SEG7) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_SEG15) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_SEG8))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_EXT_RST) |           \
                                     PIN_ODR_HIGH(GPIOD_MEMS_SCK) |         \
                                     PIN_ODR_HIGH(GPIOD_GYRO_INT1) |        \
                                     PIN_ODR_HIGH(GPIOD_MEMS_MISO) |        \
                                     PIN_ODR_HIGH(GPIOD_MEMS_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOD_USART_TX) |         \
                                     PIN_ODR_HIGH(GPIOD_USART_RX) |         \
                                     PIN_ODR_HIGH(GPIOD_GYRO_CS) |          \
                                     PIN_ODR_HIGH(GPIOD_LCD_SEG18) |        \
                                     PIN_ODR_HIGH(GPIOD_LCD_SEG5) |         \
                                     PIN_ODR_HIGH(GPIOD_LCD_SEG17) |        \
                                     PIN_ODR_HIGH(GPIOD_LCD_SEG6) |         \
                                     PIN_ODR_HIGH(GPIOD_LCD_SEG16) |        \
                                     PIN_ODR_HIGH(GPIOD_LCD_SEG7) |         \
                                     PIN_ODR_HIGH(GPIOD_LCD_SEG15) |        \
                                     PIN_ODR_HIGH(GPIOD_LCD_SEG8))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_EXT_RST, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_MEMS_SCK, 5U) |      \
                                     PIN_AFIO_AF(GPIOD_GYRO_INT1, 0U) |     \
                                     PIN_AFIO_AF(GPIOD_MEMS_MISO, 5U) |     \
                                     PIN_AFIO_AF(GPIOD_MEMS_MOSI, 5U) |     \
                                     PIN_AFIO_AF(GPIOD_USART_TX, 7U) |      \
                                     PIN_AFIO_AF(GPIOD_USART_RX, 7U) |      \
                                     PIN_AFIO_AF(GPIOD_GYRO_CS, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_LCD_SEG18, 11U) |    \
                                     PIN_AFIO_AF(GPIOD_LCD_SEG5, 11U) |     \
                                     PIN_AFIO_AF(GPIOD_LCD_SEG17, 11U) |    \
                                     PIN_AFIO_AF(GPIOD_LCD_SEG6, 11U) |     \
                                     PIN_AFIO_AF(GPIOD_LCD_SEG16, 11U) |    \
                                     PIN_AFIO_AF(GPIOD_LCD_SEG7, 11U) |     \
                                     PIN_AFIO_AF(GPIOD_LCD_SEG15, 11U) |    \
                                     PIN_AFIO_AF(GPIOD_LCD_SEG8, 11U))
#define VAL_GPIOD_ASCR              (PIN_ASCR_DISABLED(GPIOD_EXT_RST) |     \
                                     PIN_ASCR_DISABLED(GPIOD_MEMS_SCK) |    \
                                     PIN_ASCR_DISABLED(GPIOD_GYRO_INT1) |   \
                                     PIN_ASCR_DISABLED(GPIOD_MEMS_MISO) |   \
                                     PIN_ASCR_DISABLED(GPIOD_MEMS_MOSI) |   \
                                     PIN_ASCR_DISABLED(GPIOD_USART_TX) |    \
                                     PIN_ASCR_DISABLED(GPIOD_USART_RX) |    \
                                     PIN_ASCR_DISABLED(GPIOD_GYRO_CS) |     \
                                     PIN_ASCR_DISABLED(GPIOD_LCD_SEG18) |   \
                                     PIN_ASCR_DISABLED(GPIOD_LCD_SEG5) |    \
                                     PIN_ASCR_DISABLED(GPIOD_LCD_SEG17) |   \
                                     PIN_ASCR_DISABLED(GPIOD_LCD_SEG6) |    \
                                     PIN_ASCR_DISABLED(GPIOD_LCD_SEG16) |   \
                                     PIN_ASCR_DISABLED(GPIOD_LCD_SEG7) |    \
                                     PIN_ASCR_DISABLED(GPIOD_LCD_SEG15) |   \
                                     PIN_ASCR_DISABLED(GPIOD_LCD_SEG8))
#define VAL_GPIOD_LOCKR             (PIN_LOCKR_DISABLED(GPIOD_EXT_RST) |    \
                                     PIN_LOCKR_DISABLED(GPIOD_MEMS_SCK) |   \
                                     PIN_LOCKR_DISABLED(GPIOD_GYRO_INT1) |  \
                                     PIN_LOCKR_DISABLED(GPIOD_MEMS_MISO) |  \
                                     PIN_LOCKR_DISABLED(GPIOD_MEMS_MOSI) |  \
                                     PIN_LOCKR_DISABLED(GPIOD_USART_TX) |   \
                                     PIN_LOCKR_DISABLED(GPIOD_USART_RX) |   \
                                     PIN_LOCKR_DISABLED(GPIOD_GYRO_CS) |    \
                                     PIN_LOCKR_DISABLED(GPIOD_LCD_SEG18) |  \
                                     PIN_LOCKR_DISABLED(GPIOD_LCD_SEG5) |   \
                                     PIN_LOCKR_DISABLED(GPIOD_LCD_SEG17) |  \
                                     PIN_LOCKR_DISABLED(GPIOD_LCD_SEG6) |   \
                                     PIN_LOCKR_DISABLED(GPIOD_LCD_SEG16) |  \
                                     PIN_LOCKR_DISABLED(GPIOD_LCD_SEG7) |   \
                                     PIN_LOCKR_DISABLED(GPIOD_LCD_SEG15) |  \
                                     PIN_LOCKR_DISABLED(GPIOD_LCD_SEG8))

/*
 * GPIOE setup:
 *
 * PE0  - XL_CS                     (output pushpull maximum).
 * PE1  - XL_INT                    (input pulldown).
 * PE2  - SAI1_MCK                  (alternate 13).
 * PE3  - AUDIO_RST                 (output pushpull maximum).
 * PE4  - SAI1_FS                   (alternate 13).
 * PE5  - SAI1_SCK                  (alternate 13).
 * PE6  - SAI1_SD                   (alternate 13).
 * PE7  - AUDIO_DIN                 (alternate 6).
 * PE8  - LED_GREEN                 (output pushpull maximum).
 * PE9  - AUDIO_CLK                 (alternate 6).
 * PE10 - QSPI_CLK                  (alternate 10).
 * PE11 - QSPI_CS                   (alternate 10).
 * PE12 - QSPI_D0                   (alternate 10).
 * PE13 - QSPI_D1                   (alternate 10).
 * PE14 - QSPI_D2                   (alternate 10).
 * PE15 - QSPI_D3                   (alternate 10).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_OUTPUT(GPIOE_XL_CS) |         \
                                     PIN_MODE_INPUT(GPIOE_XL_INT) |         \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_MCK) |   \
                                     PIN_MODE_OUTPUT(GPIOE_AUDIO_RST) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_FS) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_SCK) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_SD) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_AUDIO_DIN) |  \
                                     PIN_MODE_OUTPUT(GPIOE_LED_GREEN) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_AUDIO_CLK) |  \
                                     PIN_MODE_ALTERNATE(GPIOE_QSPI_CLK) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_QSPI_CS) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_QSPI_D0) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_QSPI_D1) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_QSPI_D2) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_QSPI_D3))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_XL_CS) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_XL_INT) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_MCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_AUDIO_RST) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_FS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SD) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_AUDIO_DIN) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_LED_GREEN) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_AUDIO_CLK) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_QSPI_CLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_QSPI_CS) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_QSPI_D0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_QSPI_D1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_QSPI_D2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_QSPI_D3))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_XL_CS) |         \
                                     PIN_OSPEED_HIGH(GPIOE_XL_INT) |        \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_MCK) |      \
                                     PIN_OSPEED_HIGH(GPIOE_AUDIO_RST) |     \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_FS) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_SCK) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_SD) |       \
                                     PIN_OSPEED_HIGH(GPIOE_AUDIO_DIN) |     \
                                     PIN_OSPEED_HIGH(GPIOE_LED_GREEN) |     \
                                     PIN_OSPEED_HIGH(GPIOE_AUDIO_CLK) |     \
                                     PIN_OSPEED_HIGH(GPIOE_QSPI_CLK) |      \
                                     PIN_OSPEED_HIGH(GPIOE_QSPI_CS) |       \
                                     PIN_OSPEED_HIGH(GPIOE_QSPI_D0) |       \
                                     PIN_OSPEED_HIGH(GPIOE_QSPI_D1) |       \
                                     PIN_OSPEED_HIGH(GPIOE_QSPI_D2) |       \
                                     PIN_OSPEED_HIGH(GPIOE_QSPI_D3))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_XL_CS) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOE_XL_INT) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_MCK) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_AUDIO_RST) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_FS) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_SCK) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_SD) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_AUDIO_DIN) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_LED_GREEN) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_AUDIO_CLK) |  \
                                     PIN_PUPDR_PULLUP(GPIOE_QSPI_CLK) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_QSPI_CS) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_QSPI_D0) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_QSPI_D1) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_QSPI_D2) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_QSPI_D3))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_XL_CS) |            \
                                     PIN_ODR_HIGH(GPIOE_XL_INT) |           \
                                     PIN_ODR_HIGH(GPIOE_SAI1_MCK) |         \
                                     PIN_ODR_HIGH(GPIOE_AUDIO_RST) |        \
                                     PIN_ODR_HIGH(GPIOE_SAI1_FS) |          \
                                     PIN_ODR_HIGH(GPIOE_SAI1_SCK) |         \
                                     PIN_ODR_HIGH(GPIOE_SAI1_SD) |          \
                                     PIN_ODR_HIGH(GPIOE_AUDIO_DIN) |        \
                                     PIN_ODR_LOW(GPIOE_LED_GREEN) |         \
                                     PIN_ODR_HIGH(GPIOE_AUDIO_CLK) |        \
                                     PIN_ODR_HIGH(GPIOE_QSPI_CLK) |         \
                                     PIN_ODR_HIGH(GPIOE_QSPI_CS) |          \
                                     PIN_ODR_HIGH(GPIOE_QSPI_D0) |          \
                                     PIN_ODR_HIGH(GPIOE_QSPI_D1) |          \
                                     PIN_ODR_HIGH(GPIOE_QSPI_D2) |          \
                                     PIN_ODR_HIGH(GPIOE_QSPI_D3))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_XL_CS, 0U) |         \
                                     PIN_AFIO_AF(GPIOE_XL_INT, 0U) |        \
                                     PIN_AFIO_AF(GPIOE_SAI1_MCK, 13U) |     \
                                     PIN_AFIO_AF(GPIOE_AUDIO_RST, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_SAI1_FS, 13U) |      \
                                     PIN_AFIO_AF(GPIOE_SAI1_SCK, 13U) |     \
                                     PIN_AFIO_AF(GPIOE_SAI1_SD, 13U) |      \
                                     PIN_AFIO_AF(GPIOE_AUDIO_DIN, 6U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_LED_GREEN, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_AUDIO_CLK, 6U) |     \
                                     PIN_AFIO_AF(GPIOE_QSPI_CLK, 10U) |     \
                                     PIN_AFIO_AF(GPIOE_QSPI_CS, 10U) |      \
                                     PIN_AFIO_AF(GPIOE_QSPI_D0, 10U) |      \
                                     PIN_AFIO_AF(GPIOE_QSPI_D1, 10U) |      \
                                     PIN_AFIO_AF(GPIOE_QSPI_D2, 10U) |      \
                                     PIN_AFIO_AF(GPIOE_QSPI_D3, 10U))
#define VAL_GPIOE_ASCR              (PIN_ASCR_DISABLED(GPIOE_XL_CS) |       \
                                     PIN_ASCR_DISABLED(GPIOE_XL_INT) |      \
                                     PIN_ASCR_DISABLED(GPIOE_SAI1_MCK) |    \
                                     PIN_ASCR_DISABLED(GPIOE_AUDIO_RST) |   \
                                     PIN_ASCR_DISABLED(GPIOE_SAI1_FS) |     \
                                     PIN_ASCR_DISABLED(GPIOE_SAI1_SCK) |    \
                                     PIN_ASCR_DISABLED(GPIOE_SAI1_SD) |     \
                                     PIN_ASCR_DISABLED(GPIOE_AUDIO_DIN) |   \
                                     PIN_ASCR_DISABLED(GPIOE_LED_GREEN) |   \
                                     PIN_ASCR_DISABLED(GPIOE_AUDIO_CLK) |   \
                                     PIN_ASCR_DISABLED(GPIOE_QSPI_CLK) |    \
                                     PIN_ASCR_DISABLED(GPIOE_QSPI_CS) |     \
                                     PIN_ASCR_DISABLED(GPIOE_QSPI_D0) |     \
                                     PIN_ASCR_DISABLED(GPIOE_QSPI_D1) |     \
                                     PIN_ASCR_DISABLED(GPIOE_QSPI_D2) |     \
                                     PIN_ASCR_DISABLED(GPIOE_QSPI_D3))
#define VAL_GPIOE_LOCKR             (PIN_LOCKR_DISABLED(GPIOE_XL_CS) |      \
                                     PIN_LOCKR_DISABLED(GPIOE_XL_INT) |     \
                                     PIN_LOCKR_DISABLED(GPIOE_SAI1_MCK) |   \
                                     PIN_LOCKR_DISABLED(GPIOE_AUDIO_RST) |  \
                                     PIN_LOCKR_DISABLED(GPIOE_SAI1_FS) |    \
                                     PIN_LOCKR_DISABLED(GPIOE_SAI1_SCK) |   \
                                     PIN_LOCKR_DISABLED(GPIOE_SAI1_SD) |    \
                                     PIN_LOCKR_DISABLED(GPIOE_AUDIO_DIN) |  \
                                     PIN_LOCKR_DISABLED(GPIOE_LED_GREEN) |  \
                                     PIN_LOCKR_DISABLED(GPIOE_AUDIO_CLK) |  \
                                     PIN_LOCKR_DISABLED(GPIOE_QSPI_CLK) |   \
                                     PIN_LOCKR_DISABLED(GPIOE_QSPI_CS) |    \
                                     PIN_LOCKR_DISABLED(GPIOE_QSPI_D0) |    \
                                     PIN_LOCKR_DISABLED(GPIOE_QSPI_D1) |    \
                                     PIN_LOCKR_DISABLED(GPIOE_QSPI_D2) |    \
                                     PIN_LOCKR_DISABLED(GPIOE_QSPI_D3))

/*
 * GPIOF setup:
 *
 * PF0  - PIN0                      (analog).
 * PF1  - PIN1                      (analog).
 * PF2  - PIN2                      (analog).
 * PF3  - PIN3                      (analog).
 * PF4  - PIN4                      (analog).
 * PF5  - PIN5                      (analog).
 * PF6  - PIN6                      (analog).
 * PF7  - PIN7                      (analog).
 * PF8  - PIN8                      (analog).
 * PF9  - PIN9                      (analog).
 * PF10 - PIN10                     (analog).
 * PF11 - PIN11                     (analog).
 * PF12 - PIN12                     (analog).
 * PF13 - PIN13                     (analog).
 * PF14 - PIN14                     (analog).
 * PF15 - PIN15                     (analog).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ANALOG(GPIOF_PIN0) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN1) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN2) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN3) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN4) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN5) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN6) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN7) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN8) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN9) |          \
                                     PIN_MODE_ANALOG(GPIOF_PIN10) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN11) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN12) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN13) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN14) |         \
                                     PIN_MODE_ANALOG(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |            \
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
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))
#define VAL_GPIOF_ASCR              (PIN_ASCR_DISABLED(GPIOF_PIN0) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN1) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN2) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN3) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN4) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN5) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN6) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN7) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN8) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN9) |        \
                                     PIN_ASCR_DISABLED(GPIOF_PIN10) |       \
                                     PIN_ASCR_DISABLED(GPIOF_PIN11) |       \
                                     PIN_ASCR_DISABLED(GPIOF_PIN12) |       \
                                     PIN_ASCR_DISABLED(GPIOF_PIN13) |       \
                                     PIN_ASCR_DISABLED(GPIOF_PIN14) |       \
                                     PIN_ASCR_DISABLED(GPIOF_PIN15))
#define VAL_GPIOF_LOCKR             (PIN_LOCKR_DISABLED(GPIOF_PIN0) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN1) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN2) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN3) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN4) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN5) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN6) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN7) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN8) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN9) |       \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN10) |      \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN11) |      \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN12) |      \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN13) |      \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN14) |      \
                                     PIN_LOCKR_DISABLED(GPIOF_PIN15))

/*
 * GPIOG setup:
 *
 * PG0  - PIN0                      (analog).
 * PG1  - PIN1                      (analog).
 * PG2  - PIN2                      (analog).
 * PG3  - PIN3                      (analog).
 * PG4  - PIN4                      (analog).
 * PG5  - PIN5                      (analog).
 * PG6  - PIN6                      (analog).
 * PG7  - PIN7                      (analog).
 * PG8  - PIN8                      (analog).
 * PG9  - PIN9                      (analog).
 * PG10 - PIN10                     (analog).
 * PG11 - PIN11                     (analog).
 * PG12 - PIN12                     (analog).
 * PG13 - PIN13                     (analog).
 * PG14 - PIN14                     (analog).
 * PG15 - PIN15                     (analog).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_ANALOG(GPIOG_PIN0) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN1) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN2) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN3) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN4) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN5) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN6) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN7) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN8) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN9) |          \
                                     PIN_MODE_ANALOG(GPIOG_PIN10) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN11) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN12) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN13) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN14) |         \
                                     PIN_MODE_ANALOG(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOG_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))
#define VAL_GPIOG_ASCR              (PIN_ASCR_DISABLED(GPIOG_PIN0) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN1) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN2) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN3) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN4) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN5) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN6) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN7) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN8) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN9) |        \
                                     PIN_ASCR_DISABLED(GPIOG_PIN10) |       \
                                     PIN_ASCR_DISABLED(GPIOG_PIN11) |       \
                                     PIN_ASCR_DISABLED(GPIOG_PIN12) |       \
                                     PIN_ASCR_DISABLED(GPIOG_PIN13) |       \
                                     PIN_ASCR_DISABLED(GPIOG_PIN14) |       \
                                     PIN_ASCR_DISABLED(GPIOG_PIN15))
#define VAL_GPIOG_LOCKR             (PIN_LOCKR_DISABLED(GPIOG_PIN0) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN1) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN2) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN3) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN4) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN5) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN6) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN7) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN8) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN9) |       \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN10) |      \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN11) |      \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN12) |      \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN13) |      \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN14) |      \
                                     PIN_LOCKR_DISABLED(GPIOG_PIN15))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - PIN2                      (analog).
 * PH3  - PIN3                      (analog).
 * PH4  - PIN4                      (analog).
 * PH5  - PIN5                      (analog).
 * PH6  - PIN6                      (analog).
 * PH7  - PIN7                      (analog).
 * PH8  - PIN8                      (analog).
 * PH9  - PIN9                      (analog).
 * PH10 - PIN10                     (analog).
 * PH11 - PIN11                     (analog).
 * PH12 - PIN12                     (analog).
 * PH13 - PIN13                     (analog).
 * PH14 - PIN14                     (analog).
 * PH15 - PIN15                     (analog).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_ANALOG(GPIOH_PIN2) |          \
                                     PIN_MODE_ANALOG(GPIOH_PIN3) |          \
                                     PIN_MODE_ANALOG(GPIOH_PIN4) |          \
                                     PIN_MODE_ANALOG(GPIOH_PIN5) |          \
                                     PIN_MODE_ANALOG(GPIOH_PIN6) |          \
                                     PIN_MODE_ANALOG(GPIOH_PIN7) |          \
                                     PIN_MODE_ANALOG(GPIOH_PIN8) |          \
                                     PIN_MODE_ANALOG(GPIOH_PIN9) |          \
                                     PIN_MODE_ANALOG(GPIOH_PIN10) |         \
                                     PIN_MODE_ANALOG(GPIOH_PIN11) |         \
                                     PIN_MODE_ANALOG(GPIOH_PIN12) |         \
                                     PIN_MODE_ANALOG(GPIOH_PIN13) |         \
                                     PIN_MODE_ANALOG(GPIOH_PIN14) |         \
                                     PIN_MODE_ANALOG(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
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
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN7) |       \
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
                                     PIN_PUPDR_FLOATING(GPIOH_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
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
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
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
#define VAL_GPIOH_ASCR              (PIN_ASCR_DISABLED(GPIOH_OSC_IN) |      \
                                     PIN_ASCR_DISABLED(GPIOH_OSC_OUT) |     \
                                     PIN_ASCR_DISABLED(GPIOH_PIN2) |        \
                                     PIN_ASCR_DISABLED(GPIOH_PIN3) |        \
                                     PIN_ASCR_DISABLED(GPIOH_PIN4) |        \
                                     PIN_ASCR_DISABLED(GPIOH_PIN5) |        \
                                     PIN_ASCR_DISABLED(GPIOH_PIN6) |        \
                                     PIN_ASCR_DISABLED(GPIOH_PIN7) |        \
                                     PIN_ASCR_DISABLED(GPIOH_PIN8) |        \
                                     PIN_ASCR_DISABLED(GPIOH_PIN9) |        \
                                     PIN_ASCR_DISABLED(GPIOH_PIN10) |       \
                                     PIN_ASCR_DISABLED(GPIOH_PIN11) |       \
                                     PIN_ASCR_DISABLED(GPIOH_PIN12) |       \
                                     PIN_ASCR_DISABLED(GPIOH_PIN13) |       \
                                     PIN_ASCR_DISABLED(GPIOH_PIN14) |       \
                                     PIN_ASCR_DISABLED(GPIOH_PIN15))
#define VAL_GPIOH_LOCKR             (PIN_LOCKR_DISABLED(GPIOH_OSC_IN) |     \
                                     PIN_LOCKR_DISABLED(GPIOH_OSC_OUT) |    \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN2) |       \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN3) |       \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN4) |       \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN5) |       \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN6) |       \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN7) |       \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN8) |       \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN9) |       \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN10) |      \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN11) |      \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN12) |      \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN13) |      \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN14) |      \
                                     PIN_LOCKR_DISABLED(GPIOH_PIN15))

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
