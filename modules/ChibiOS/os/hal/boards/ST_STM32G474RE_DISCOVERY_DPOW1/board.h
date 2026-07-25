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
 * Setup for STMicroelectronics STM32G747RE-Discovery-DPOW1 board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32G474RE_DISCOVERY_DPOW1
#define BOARD_NAME                  "STMicroelectronics STM32G747RE-Discovery-DPOW1"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 * NOTE: HSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
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
#undef STM32G474xx
#define STM32G474xx

/*
 * IO pins assignments.
 */
#define GPIOA_BUCK_GREEN_SENSE      0U
#define GPIOA_BUCKBOOST_VIN         1U
#define GPIOA_BUBKBOOST_I_IN_AVG    2U
#define GPIOA_BUCKBOOST_VOUT        3U
#define GPIOA_PIN4                  4U
#define GPIOA_PIN5                  5U
#define GPIOA_PIN6                  6U
#define GPIOA_BUCK_RED_SENSE        7U
#define GPIOA_BUCK_BLUE_DRIVE       8U
#define GPIOA_USBPD_DBCC1           9U
#define GPIOA_USBPD_DBCC2           10U
#define GPIOA_USBPD_FS_DM           11U
#define GPIOA_USBPD_FS_DP           12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_LED_DOWN_BLUE         15U

#define GPIOB_BUCK_BLUE_SENSE       0U
#define GPIOB_LED_LEFT_ORANGE       1U
#define GPIOB_JOY_DOWN              2U
#define GPIOB_TRACESWO              3U
#define GPIOB_USBPD_CC2             4U
#define GPIOB_LED_UP_RED            5U
#define GPIOB_USBPD_CC1             6U
#define GPIOB_LED_RIGHT_GREEN       7U
#define GPIOB_BOOT0                 8U
#define GPIOB_PIN9                  9U
#define GPIOB_JOY_UP                10U
#define GPIOB_BUBKBOOST_I_IN_SENSE  11U
#define GPIOB_BUCKBOOST_P1_DRIVE    12U
#define GPIOB_BUCKBOOST_N1_DRIVE    13U
#define GPIOB_BUCKBOOST_N2_DRIVE    14U
#define GPIOB_BUCKBOOST_P2_DRIVE    15U

#define GPIOC_PIN0                  0U
#define GPIOC_PIN1                  1U
#define GPIOC_USBPD_VIN             2U
#define GPIOC_BUCKBOOST_USBPD_EN    3U
#define GPIOC_JOY_LEFT              4U
#define GPIOC_JOY_RIGHT             5U
#define GPIOC_BUCK_RED_DRIVE        6U
#define GPIOC_RC_SINUS              7U
#define GPIOC_BUCK_GREEN_DRIVE      8U
#define GPIOC_PIN9                  9U
#define GPIOC_USART3_TX             10U
#define GPIOC_USART3_RX             11U
#define GPIOC_P5V_USBPD_1A_PROTECT  12U
#define GPIOC_JOY_SELECT            13U
#define GPIOC_BUCKBOOST_LOAD_50     14U
#define GPIOC_BUCKBOOST_LOAD_100    15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_USBPD_550mA_PROTECT   2U
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

#define GPIOE_PIN0                  0U
#define GPIOE_PIN1                  1U
#define GPIOE_PIN2                  2U
#define GPIOE_PIN3                  3U
#define GPIOE_PIN4                  4U
#define GPIOE_PIN5                  5U
#define GPIOE_PIN6                  6U
#define GPIOE_PIN7                  7U
#define GPIOE_PIN8                  8U
#define GPIOE_PIN9                  9U
#define GPIOE_PIN10                 10U
#define GPIOE_PIN11                 11U
#define GPIOE_PIN12                 12U
#define GPIOE_PIN13                 13U
#define GPIOE_PIN14                 14U
#define GPIOE_PIN15                 15U

#define GPIOF_OSC_IN                0U
#define GPIOF_OSC_OUT               1U
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

/*
 * IO lines assignments.
 */
#define LINE_BUCK_GREEN_SENSE       PAL_LINE(GPIOA, 0U)
#define LINE_BUCKBOOST_VIN          PAL_LINE(GPIOA, 1U)
#define LINE_BUBKBOOST_I_IN_AVG     PAL_LINE(GPIOA, 2U)
#define LINE_BUCKBOOST_VOUT         PAL_LINE(GPIOA, 3U)
#define LINE_BUCK_RED_SENSE         PAL_LINE(GPIOA, 7U)
#define LINE_BUCK_BLUE_DRIVE        PAL_LINE(GPIOA, 8U)
#define LINE_USBPD_DBCC1            PAL_LINE(GPIOA, 9U)
#define LINE_USBPD_DBCC2            PAL_LINE(GPIOA, 10U)
#define LINE_USBPD_FS_DM            PAL_LINE(GPIOA, 11U)
#define LINE_USBPD_FS_DP            PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_LED_DOWN_BLUE          PAL_LINE(GPIOA, 15U)
#define LINE_BUCK_BLUE_SENSE        PAL_LINE(GPIOB, 0U)
#define LINE_LED_LEFT_ORANGE        PAL_LINE(GPIOB, 1U)
#define LINE_JOY_DOWN               PAL_LINE(GPIOB, 2U)
#define LINE_TRACESWO               PAL_LINE(GPIOB, 3U)
#define LINE_USBPD_CC2              PAL_LINE(GPIOB, 4U)
#define LINE_LED_UP_RED             PAL_LINE(GPIOB, 5U)
#define LINE_USBPD_CC1              PAL_LINE(GPIOB, 6U)
#define LINE_LED_RIGHT_GREEN        PAL_LINE(GPIOB, 7U)
#define LINE_BOOT0                  PAL_LINE(GPIOB, 8U)
#define LINE_JOY_UP                 PAL_LINE(GPIOB, 10U)
#define LINE_BUBKBOOST_I_IN_SENSE   PAL_LINE(GPIOB, 11U)
#define LINE_BUCKBOOST_P1_DRIVE     PAL_LINE(GPIOB, 12U)
#define LINE_BUCKBOOST_N1_DRIVE     PAL_LINE(GPIOB, 13U)
#define LINE_BUCKBOOST_N2_DRIVE     PAL_LINE(GPIOB, 14U)
#define LINE_BUCKBOOST_P2_DRIVE     PAL_LINE(GPIOB, 15U)
#define LINE_USBPD_VIN              PAL_LINE(GPIOC, 2U)
#define LINE_BUCKBOOST_USBPD_EN     PAL_LINE(GPIOC, 3U)
#define LINE_JOY_LEFT               PAL_LINE(GPIOC, 4U)
#define LINE_JOY_RIGHT              PAL_LINE(GPIOC, 5U)
#define LINE_BUCK_RED_DRIVE         PAL_LINE(GPIOC, 6U)
#define LINE_RC_SINUS               PAL_LINE(GPIOC, 7U)
#define LINE_BUCK_GREEN_DRIVE       PAL_LINE(GPIOC, 8U)
#define LINE_USART3_TX              PAL_LINE(GPIOC, 10U)
#define LINE_USART3_RX              PAL_LINE(GPIOC, 11U)
#define LINE_P5V_USBPD_1A_PROTECT   PAL_LINE(GPIOC, 12U)
#define LINE_JOY_SELECT             PAL_LINE(GPIOC, 13U)
#define LINE_BUCKBOOST_LOAD_50      PAL_LINE(GPIOC, 14U)
#define LINE_BUCKBOOST_LOAD_100     PAL_LINE(GPIOC, 15U)
#define LINE_USBPD_550mA_PROTECT    PAL_LINE(GPIOD, 2U)
#define LINE_OSC_IN                 PAL_LINE(GPIOF, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOF, 1U)

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
#define PIN_LOCKR_DISABLED(n)       (0U << (n))
#define PIN_LOCKR_ENABLED(n)        (1U << (n))

/*
 * GPIOA setup:
 *
 * PA0  - BUCK_GREEN_SENSE          (analog).
 * PA1  - BUCKBOOST_VIN             (analog).
 * PA2  - BUBKBOOST_I_IN_AVG        (analog).
 * PA3  - BUCKBOOST_VOUT            (analog).
 * PA4  - PIN4                      (analog).
 * PA5  - PIN5                      (analog).
 * PA6  - PIN6                      (analog).
 * PA7  - BUCK_RED_SENSE            (analog).
 * PA8  - BUCK_BLUE_DRIVE           (alternate 13).
 * PA9  - USBPD_DBCC1               (input pullup).
 * PA10 - USBPD_DBCC2               (input pullup).
 * PA11 - USBPD_FS_DM               (analog).
 * PA12 - USBPD_FS_DP               (analog).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - LED_DOWN_BLUE             (output pushpull maximum).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_BUCK_GREEN_SENSE) |\
                                     PIN_MODE_ANALOG(GPIOA_BUCKBOOST_VIN) | \
                                     PIN_MODE_ANALOG(GPIOA_BUBKBOOST_I_IN_AVG) |\
                                     PIN_MODE_ANALOG(GPIOA_BUCKBOOST_VOUT) |\
                                     PIN_MODE_ANALOG(GPIOA_PIN4) |          \
                                     PIN_MODE_ANALOG(GPIOA_PIN5) |          \
                                     PIN_MODE_ANALOG(GPIOA_PIN6) |          \
                                     PIN_MODE_ANALOG(GPIOA_BUCK_RED_SENSE) |\
                                     PIN_MODE_ALTERNATE(GPIOA_BUCK_BLUE_DRIVE) |\
                                     PIN_MODE_INPUT(GPIOA_USBPD_DBCC1) |    \
                                     PIN_MODE_INPUT(GPIOA_USBPD_DBCC2) |    \
                                     PIN_MODE_ANALOG(GPIOA_USBPD_FS_DM) |   \
                                     PIN_MODE_ANALOG(GPIOA_USBPD_FS_DP) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_OUTPUT(GPIOA_LED_DOWN_BLUE))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_BUCK_GREEN_SENSE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BUCKBOOST_VIN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BUBKBOOST_I_IN_AVG) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BUCKBOOST_VOUT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_BUCK_RED_SENSE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_BUCK_BLUE_DRIVE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USBPD_DBCC1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USBPD_DBCC2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USBPD_FS_DM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USBPD_FS_DP) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_DOWN_BLUE))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_BUCK_GREEN_SENSE) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_BUCKBOOST_VIN) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_BUBKBOOST_I_IN_AVG) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_BUCKBOOST_VOUT) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_BUCK_RED_SENSE) |\
                                     PIN_OSPEED_HIGH(GPIOA_BUCK_BLUE_DRIVE) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_USBPD_DBCC1) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_USBPD_DBCC2) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_USBPD_FS_DM) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_USBPD_FS_DP) |\
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_LED_DOWN_BLUE))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_BUCK_GREEN_SENSE) |\
                                     PIN_PUPDR_FLOATING(GPIOA_BUCKBOOST_VIN) |\
                                     PIN_PUPDR_FLOATING(GPIOA_BUBKBOOST_I_IN_AVG) |\
                                     PIN_PUPDR_FLOATING(GPIOA_BUCKBOOST_VOUT) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_BUCK_RED_SENSE) |\
                                     PIN_PUPDR_FLOATING(GPIOA_BUCK_BLUE_DRIVE) |\
                                     PIN_PUPDR_PULLUP(GPIOA_USBPD_DBCC1) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_USBPD_DBCC2) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USBPD_FS_DM) |\
                                     PIN_PUPDR_FLOATING(GPIOA_USBPD_FS_DP) |\
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_LED_DOWN_BLUE))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_BUCK_GREEN_SENSE) |  \
                                     PIN_ODR_LOW(GPIOA_BUCKBOOST_VIN) |     \
                                     PIN_ODR_LOW(GPIOA_BUBKBOOST_I_IN_AVG) |\
                                     PIN_ODR_LOW(GPIOA_BUCKBOOST_VOUT) |    \
                                     PIN_ODR_LOW(GPIOA_PIN4) |              \
                                     PIN_ODR_LOW(GPIOA_PIN5) |              \
                                     PIN_ODR_LOW(GPIOA_PIN6) |              \
                                     PIN_ODR_LOW(GPIOA_BUCK_RED_SENSE) |    \
                                     PIN_ODR_LOW(GPIOA_BUCK_BLUE_DRIVE) |   \
                                     PIN_ODR_LOW(GPIOA_USBPD_DBCC1) |       \
                                     PIN_ODR_LOW(GPIOA_USBPD_DBCC2) |       \
                                     PIN_ODR_LOW(GPIOA_USBPD_FS_DM) |       \
                                     PIN_ODR_LOW(GPIOA_USBPD_FS_DP) |       \
                                     PIN_ODR_LOW(GPIOA_SWDIO) |             \
                                     PIN_ODR_LOW(GPIOA_SWCLK) |             \
                                     PIN_ODR_LOW(GPIOA_LED_DOWN_BLUE))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_BUCK_GREEN_SENSE, 0U) |\
                                     PIN_AFIO_AF(GPIOA_BUCKBOOST_VIN, 0U) | \
                                     PIN_AFIO_AF(GPIOA_BUBKBOOST_I_IN_AVG, 0U) |\
                                     PIN_AFIO_AF(GPIOA_BUCKBOOST_VOUT, 0U) |\
                                     PIN_AFIO_AF(GPIOA_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_BUCK_RED_SENSE, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_BUCK_BLUE_DRIVE, 13U) |\
                                     PIN_AFIO_AF(GPIOA_USBPD_DBCC1, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_USBPD_DBCC2, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_USBPD_FS_DM, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_USBPD_FS_DP, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_LED_DOWN_BLUE, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - BUCK_BLUE_SENSE           (analog).
 * PB1  - LED_LEFT_ORANGE           (output pushpull maximum).
 * PB2  - JOY_DOWN                  (input pullup).
 * PB3  - TRACESWO                  (alternate 0).
 * PB4  - USBPD_CC2                 (alternate 12).
 * PB5  - LED_UP_RED                (output pushpull maximum).
 * PB6  - USBPD_CC1                 (alternate 12).
 * PB7  - LED_RIGHT_GREEN           (output pushpull maximum).
 * PB8  - BOOT0                     (analog).
 * PB9  - PIN9                      (analog).
 * PB10 - JOY_UP                    (input pullup).
 * PB11 - BUBKBOOST_I_IN_SENSE      (analog).
 * PB12 - BUCKBOOST_P1_DRIVE        (alternate 13).
 * PB13 - BUCKBOOST_N1_DRIVE        (alternate 13).
 * PB14 - BUCKBOOST_N2_DRIVE        (alternate 13).
 * PB15 - BUCKBOOST_P2_DRIVE        (alternate 13).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ANALOG(GPIOB_BUCK_BLUE_SENSE) |\
                                     PIN_MODE_OUTPUT(GPIOB_LED_LEFT_ORANGE) |\
                                     PIN_MODE_INPUT(GPIOB_JOY_DOWN) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_TRACESWO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_USBPD_CC2) |  \
                                     PIN_MODE_OUTPUT(GPIOB_LED_UP_RED) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_USBPD_CC1) |  \
                                     PIN_MODE_OUTPUT(GPIOB_LED_RIGHT_GREEN) |\
                                     PIN_MODE_ANALOG(GPIOB_BOOT0) |         \
                                     PIN_MODE_ANALOG(GPIOB_PIN9) |          \
                                     PIN_MODE_INPUT(GPIOB_JOY_UP) |         \
                                     PIN_MODE_ANALOG(GPIOB_BUBKBOOST_I_IN_SENSE) |\
                                     PIN_MODE_ALTERNATE(GPIOB_BUCKBOOST_P1_DRIVE) |\
                                     PIN_MODE_ALTERNATE(GPIOB_BUCKBOOST_N1_DRIVE) |\
                                     PIN_MODE_ALTERNATE(GPIOB_BUCKBOOST_N2_DRIVE) |\
                                     PIN_MODE_ALTERNATE(GPIOB_BUCKBOOST_P2_DRIVE))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_BUCK_BLUE_SENSE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_LEFT_ORANGE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_JOY_DOWN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_TRACESWO) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USBPD_CC2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_UP_RED) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USBPD_CC1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_RIGHT_GREEN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_BOOT0) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JOY_UP) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BUBKBOOST_I_IN_SENSE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_BUCKBOOST_P1_DRIVE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_BUCKBOOST_N1_DRIVE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_BUCKBOOST_N2_DRIVE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_BUCKBOOST_P2_DRIVE))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOB_BUCK_BLUE_SENSE) |\
                                     PIN_OSPEED_HIGH(GPIOB_LED_LEFT_ORANGE) |\
                                     PIN_OSPEED_VERYLOW(GPIOB_JOY_DOWN) |   \
                                     PIN_OSPEED_HIGH(GPIOB_TRACESWO) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_USBPD_CC2) |  \
                                     PIN_OSPEED_HIGH(GPIOB_LED_UP_RED) |    \
                                     PIN_OSPEED_VERYLOW(GPIOB_USBPD_CC1) |  \
                                     PIN_OSPEED_HIGH(GPIOB_LED_RIGHT_GREEN) |\
                                     PIN_OSPEED_VERYLOW(GPIOB_BOOT0) |      \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOB_JOY_UP) |     \
                                     PIN_OSPEED_VERYLOW(GPIOB_BUBKBOOST_I_IN_SENSE) |\
                                     PIN_OSPEED_HIGH(GPIOB_BUCKBOOST_P1_DRIVE) |\
                                     PIN_OSPEED_HIGH(GPIOB_BUCKBOOST_N1_DRIVE) |\
                                     PIN_OSPEED_HIGH(GPIOB_BUCKBOOST_N2_DRIVE) |\
                                     PIN_OSPEED_HIGH(GPIOB_BUCKBOOST_P2_DRIVE))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_BUCK_BLUE_SENSE) |\
                                     PIN_PUPDR_FLOATING(GPIOB_LED_LEFT_ORANGE) |\
                                     PIN_PUPDR_PULLUP(GPIOB_JOY_DOWN) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_TRACESWO) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_USBPD_CC2) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_UP_RED) | \
                                     PIN_PUPDR_FLOATING(GPIOB_USBPD_CC1) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_RIGHT_GREEN) |\
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT0) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN9) |       \
                                     PIN_PUPDR_PULLUP(GPIOB_JOY_UP) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_BUBKBOOST_I_IN_SENSE) |\
                                     PIN_PUPDR_FLOATING(GPIOB_BUCKBOOST_P1_DRIVE) |\
                                     PIN_PUPDR_FLOATING(GPIOB_BUCKBOOST_N1_DRIVE) |\
                                     PIN_PUPDR_FLOATING(GPIOB_BUCKBOOST_N2_DRIVE) |\
                                     PIN_PUPDR_FLOATING(GPIOB_BUCKBOOST_P2_DRIVE))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_BUCK_BLUE_SENSE) |   \
                                     PIN_ODR_LOW(GPIOB_LED_LEFT_ORANGE) |   \
                                     PIN_ODR_LOW(GPIOB_JOY_DOWN) |          \
                                     PIN_ODR_LOW(GPIOB_TRACESWO) |          \
                                     PIN_ODR_LOW(GPIOB_USBPD_CC2) |         \
                                     PIN_ODR_LOW(GPIOB_LED_UP_RED) |        \
                                     PIN_ODR_LOW(GPIOB_USBPD_CC1) |         \
                                     PIN_ODR_LOW(GPIOB_LED_RIGHT_GREEN) |   \
                                     PIN_ODR_LOW(GPIOB_BOOT0) |             \
                                     PIN_ODR_LOW(GPIOB_PIN9) |              \
                                     PIN_ODR_LOW(GPIOB_JOY_UP) |            \
                                     PIN_ODR_LOW(GPIOB_BUBKBOOST_I_IN_SENSE) |\
                                     PIN_ODR_LOW(GPIOB_BUCKBOOST_P1_DRIVE) |\
                                     PIN_ODR_LOW(GPIOB_BUCKBOOST_N1_DRIVE) |\
                                     PIN_ODR_LOW(GPIOB_BUCKBOOST_N2_DRIVE) |\
                                     PIN_ODR_LOW(GPIOB_BUCKBOOST_P2_DRIVE))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_BUCK_BLUE_SENSE, 0U) |\
                                     PIN_AFIO_AF(GPIOB_LED_LEFT_ORANGE, 0U) |\
                                     PIN_AFIO_AF(GPIOB_JOY_DOWN, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_TRACESWO, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_USBPD_CC2, 12U) |    \
                                     PIN_AFIO_AF(GPIOB_LED_UP_RED, 0U) |    \
                                     PIN_AFIO_AF(GPIOB_USBPD_CC1, 12U) |    \
                                     PIN_AFIO_AF(GPIOB_LED_RIGHT_GREEN, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_BOOT0, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_JOY_UP, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_BUBKBOOST_I_IN_SENSE, 0U) |\
                                     PIN_AFIO_AF(GPIOB_BUCKBOOST_P1_DRIVE, 13U) |\
                                     PIN_AFIO_AF(GPIOB_BUCKBOOST_N1_DRIVE, 13U) |\
                                     PIN_AFIO_AF(GPIOB_BUCKBOOST_N2_DRIVE, 13U) |\
                                     PIN_AFIO_AF(GPIOB_BUCKBOOST_P2_DRIVE, 13U))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0                      (analog).
 * PC1  - PIN1                      (analog).
 * PC2  - USBPD_VIN                 (analog).
 * PC3  - BUCKBOOST_USBPD_EN        (output pushpull maximum).
 * PC4  - JOY_LEFT                  (input pullup).
 * PC5  - JOY_RIGHT                 (input pullup).
 * PC6  - BUCK_RED_DRIVE            (alternate 13).
 * PC7  - RC_SINUS                  (alternate 13).
 * PC8  - BUCK_GREEN_DRIVE          (alternate 3).
 * PC9  - PIN9                      (analog).
 * PC10 - USART3_TX                 (alternate 7).
 * PC11 - USART3_RX                 (alternate 7).
 * PC12 - P5V_USBPD_1A_PROTECT      (output pushpull maximum).
 * PC13 - JOY_SELECT                (input pullup).
 * PC14 - BUCKBOOST_LOAD_50         (output pushpull maximum).
 * PC15 - BUCKBOOST_LOAD_100        (output pushpull maximum).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_PIN0) |          \
                                     PIN_MODE_ANALOG(GPIOC_PIN1) |          \
                                     PIN_MODE_ANALOG(GPIOC_USBPD_VIN) |     \
                                     PIN_MODE_OUTPUT(GPIOC_BUCKBOOST_USBPD_EN) |\
                                     PIN_MODE_INPUT(GPIOC_JOY_LEFT) |       \
                                     PIN_MODE_INPUT(GPIOC_JOY_RIGHT) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_BUCK_RED_DRIVE) |\
                                     PIN_MODE_ALTERNATE(GPIOC_RC_SINUS) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_BUCK_GREEN_DRIVE) |\
                                     PIN_MODE_ANALOG(GPIOC_PIN9) |          \
                                     PIN_MODE_ALTERNATE(GPIOC_USART3_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_USART3_RX) |  \
                                     PIN_MODE_OUTPUT(GPIOC_P5V_USBPD_1A_PROTECT) |\
                                     PIN_MODE_INPUT(GPIOC_JOY_SELECT) |     \
                                     PIN_MODE_OUTPUT(GPIOC_BUCKBOOST_LOAD_50) |\
                                     PIN_MODE_OUTPUT(GPIOC_BUCKBOOST_LOAD_100))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USBPD_VIN) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_BUCKBOOST_USBPD_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_JOY_LEFT) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_JOY_RIGHT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_BUCK_RED_DRIVE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_RC_SINUS) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_BUCK_GREEN_DRIVE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART3_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART3_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_P5V_USBPD_1A_PROTECT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_JOY_SELECT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_BUCKBOOST_LOAD_50) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_BUCKBOOST_LOAD_100))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_USBPD_VIN) |  \
                                     PIN_OSPEED_HIGH(GPIOC_BUCKBOOST_USBPD_EN) |\
                                     PIN_OSPEED_VERYLOW(GPIOC_JOY_LEFT) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_JOY_RIGHT) |  \
                                     PIN_OSPEED_HIGH(GPIOC_BUCK_RED_DRIVE) |\
                                     PIN_OSPEED_HIGH(GPIOC_RC_SINUS) |      \
                                     PIN_OSPEED_HIGH(GPIOC_BUCK_GREEN_DRIVE) |\
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN9) |       \
                                     PIN_OSPEED_HIGH(GPIOC_USART3_TX) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_USART3_RX) |  \
                                     PIN_OSPEED_HIGH(GPIOC_P5V_USBPD_1A_PROTECT) |\
                                     PIN_OSPEED_VERYLOW(GPIOC_JOY_SELECT) | \
                                     PIN_OSPEED_HIGH(GPIOC_BUCKBOOST_LOAD_50) |\
                                     PIN_OSPEED_HIGH(GPIOC_BUCKBOOST_LOAD_100))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_USBPD_VIN) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_BUCKBOOST_USBPD_EN) |\
                                     PIN_PUPDR_PULLUP(GPIOC_JOY_LEFT) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_JOY_RIGHT) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_BUCK_RED_DRIVE) |\
                                     PIN_PUPDR_FLOATING(GPIOC_RC_SINUS) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_BUCK_GREEN_DRIVE) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_USART3_TX) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_USART3_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_P5V_USBPD_1A_PROTECT) |\
                                     PIN_PUPDR_PULLUP(GPIOC_JOY_SELECT) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_BUCKBOOST_LOAD_50) |\
                                     PIN_PUPDR_FLOATING(GPIOC_BUCKBOOST_LOAD_100))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_PIN0) |              \
                                     PIN_ODR_LOW(GPIOC_PIN1) |              \
                                     PIN_ODR_LOW(GPIOC_USBPD_VIN) |         \
                                     PIN_ODR_LOW(GPIOC_BUCKBOOST_USBPD_EN) |\
                                     PIN_ODR_LOW(GPIOC_JOY_LEFT) |          \
                                     PIN_ODR_LOW(GPIOC_JOY_RIGHT) |         \
                                     PIN_ODR_LOW(GPIOC_BUCK_RED_DRIVE) |    \
                                     PIN_ODR_LOW(GPIOC_RC_SINUS) |          \
                                     PIN_ODR_LOW(GPIOC_BUCK_GREEN_DRIVE) |  \
                                     PIN_ODR_LOW(GPIOC_PIN9) |              \
                                     PIN_ODR_LOW(GPIOC_USART3_TX) |         \
                                     PIN_ODR_LOW(GPIOC_USART3_RX) |         \
                                     PIN_ODR_LOW(GPIOC_P5V_USBPD_1A_PROTECT) |\
                                     PIN_ODR_LOW(GPIOC_JOY_SELECT) |        \
                                     PIN_ODR_LOW(GPIOC_BUCKBOOST_LOAD_50) | \
                                     PIN_ODR_LOW(GPIOC_BUCKBOOST_LOAD_100))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_USBPD_VIN, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_BUCKBOOST_USBPD_EN, 0U) |\
                                     PIN_AFIO_AF(GPIOC_JOY_LEFT, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_JOY_RIGHT, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_BUCK_RED_DRIVE, 13U) |\
                                     PIN_AFIO_AF(GPIOC_RC_SINUS, 13U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_BUCK_GREEN_DRIVE, 3U) |\
                                     PIN_AFIO_AF(GPIOC_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_USART3_TX, 7U) |     \
                                     PIN_AFIO_AF(GPIOC_USART3_RX, 7U) |     \
                                     PIN_AFIO_AF(GPIOC_P5V_USBPD_1A_PROTECT, 0U) |\
                                     PIN_AFIO_AF(GPIOC_JOY_SELECT, 0U) |    \
                                     PIN_AFIO_AF(GPIOC_BUCKBOOST_LOAD_50, 0U) |\
                                     PIN_AFIO_AF(GPIOC_BUCKBOOST_LOAD_100, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (analog).
 * PD1  - PIN1                      (analog).
 * PD2  - USBPD_550mA_PROTECT       (output pushpull maximum).
 * PD3  - PIN3                      (analog).
 * PD4  - PIN4                      (analog).
 * PD5  - PIN5                      (analog).
 * PD6  - PIN6                      (analog).
 * PD7  - PIN7                      (analog).
 * PD8  - PIN8                      (analog).
 * PD9  - PIN9                      (analog).
 * PD10 - PIN10                     (analog).
 * PD11 - PIN11                     (analog).
 * PD12 - PIN12                     (analog).
 * PD13 - PIN13                     (analog).
 * PD14 - PIN14                     (analog).
 * PD15 - PIN15                     (analog).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ANALOG(GPIOD_PIN0) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN1) |          \
                                     PIN_MODE_OUTPUT(GPIOD_USBPD_550mA_PROTECT) |\
                                     PIN_MODE_ANALOG(GPIOD_PIN3) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN4) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN5) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN6) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN7) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN8) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN9) |          \
                                     PIN_MODE_ANALOG(GPIOD_PIN10) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN11) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN12) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN13) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN14) |         \
                                     PIN_MODE_ANALOG(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USBPD_550mA_PROTECT) |\
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
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOD_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN1) |       \
                                     PIN_OSPEED_HIGH(GPIOD_USBPD_550mA_PROTECT) |\
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_USBPD_550mA_PROTECT) |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_PIN0) |              \
                                     PIN_ODR_LOW(GPIOD_PIN1) |              \
                                     PIN_ODR_LOW(GPIOD_USBPD_550mA_PROTECT) |\
                                     PIN_ODR_LOW(GPIOD_PIN3) |              \
                                     PIN_ODR_LOW(GPIOD_PIN4) |              \
                                     PIN_ODR_LOW(GPIOD_PIN5) |              \
                                     PIN_ODR_LOW(GPIOD_PIN6) |              \
                                     PIN_ODR_LOW(GPIOD_PIN7) |              \
                                     PIN_ODR_LOW(GPIOD_PIN8) |              \
                                     PIN_ODR_LOW(GPIOD_PIN9) |              \
                                     PIN_ODR_LOW(GPIOD_PIN10) |             \
                                     PIN_ODR_LOW(GPIOD_PIN11) |             \
                                     PIN_ODR_LOW(GPIOD_PIN12) |             \
                                     PIN_ODR_LOW(GPIOD_PIN13) |             \
                                     PIN_ODR_LOW(GPIOD_PIN14) |             \
                                     PIN_ODR_LOW(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_USBPD_550mA_PROTECT, 0U) |\
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
 * GPIOE setup:
 *
 * PE0  - PIN0                      (analog).
 * PE1  - PIN1                      (analog).
 * PE2  - PIN2                      (analog).
 * PE3  - PIN3                      (analog).
 * PE4  - PIN4                      (analog).
 * PE5  - PIN5                      (analog).
 * PE6  - PIN6                      (analog).
 * PE7  - PIN7                      (analog).
 * PE8  - PIN8                      (analog).
 * PE9  - PIN9                      (analog).
 * PE10 - PIN10                     (analog).
 * PE11 - PIN11                     (analog).
 * PE12 - PIN12                     (analog).
 * PE13 - PIN13                     (analog).
 * PE14 - PIN14                     (analog).
 * PE15 - PIN15                     (analog).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ANALOG(GPIOE_PIN0) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN1) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN2) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN3) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN4) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN5) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN6) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN7) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN8) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN9) |          \
                                     PIN_MODE_ANALOG(GPIOE_PIN10) |         \
                                     PIN_MODE_ANALOG(GPIOE_PIN11) |         \
                                     PIN_MODE_ANALOG(GPIOE_PIN12) |         \
                                     PIN_MODE_ANALOG(GPIOE_PIN13) |         \
                                     PIN_MODE_ANALOG(GPIOE_PIN14) |         \
                                     PIN_MODE_ANALOG(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOE_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_LOW(GPIOE_PIN0) |              \
                                     PIN_ODR_LOW(GPIOE_PIN1) |              \
                                     PIN_ODR_LOW(GPIOE_PIN2) |              \
                                     PIN_ODR_LOW(GPIOE_PIN3) |              \
                                     PIN_ODR_LOW(GPIOE_PIN4) |              \
                                     PIN_ODR_LOW(GPIOE_PIN5) |              \
                                     PIN_ODR_LOW(GPIOE_PIN6) |              \
                                     PIN_ODR_LOW(GPIOE_PIN7) |              \
                                     PIN_ODR_LOW(GPIOE_PIN8) |              \
                                     PIN_ODR_LOW(GPIOE_PIN9) |              \
                                     PIN_ODR_LOW(GPIOE_PIN10) |             \
                                     PIN_ODR_LOW(GPIOE_PIN11) |             \
                                     PIN_ODR_LOW(GPIOE_PIN12) |             \
                                     PIN_ODR_LOW(GPIOE_PIN13) |             \
                                     PIN_ODR_LOW(GPIOE_PIN14) |             \
                                     PIN_ODR_LOW(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0U) |          \
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
 * PF0  - OSC_IN                    (analog).
 * PF1  - OSC_OUT                   (analog).
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
#define VAL_GPIOF_MODER             (PIN_MODE_ANALOG(GPIOF_OSC_IN) |        \
                                     PIN_MODE_ANALOG(GPIOF_OSC_OUT) |       \
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
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSC_OUT) |    \
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
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOF_OSC_IN) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_OSC_OUT) |    \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_OSC_OUT) |    \
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
#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_OSC_IN) |            \
                                     PIN_ODR_LOW(GPIOF_OSC_OUT) |           \
                                     PIN_ODR_LOW(GPIOF_PIN2) |              \
                                     PIN_ODR_LOW(GPIOF_PIN3) |              \
                                     PIN_ODR_LOW(GPIOF_PIN4) |              \
                                     PIN_ODR_LOW(GPIOF_PIN5) |              \
                                     PIN_ODR_LOW(GPIOF_PIN6) |              \
                                     PIN_ODR_LOW(GPIOF_PIN7) |              \
                                     PIN_ODR_LOW(GPIOF_PIN8) |              \
                                     PIN_ODR_LOW(GPIOF_PIN9) |              \
                                     PIN_ODR_LOW(GPIOF_PIN10) |             \
                                     PIN_ODR_LOW(GPIOF_PIN11) |             \
                                     PIN_ODR_LOW(GPIOF_PIN12) |             \
                                     PIN_ODR_LOW(GPIOF_PIN13) |             \
                                     PIN_ODR_LOW(GPIOF_PIN14) |             \
                                     PIN_ODR_LOW(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_OSC_OUT, 0U) |       \
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
#define VAL_GPIOG_ODR               (PIN_ODR_LOW(GPIOG_PIN0) |              \
                                     PIN_ODR_LOW(GPIOG_PIN1) |              \
                                     PIN_ODR_LOW(GPIOG_PIN2) |              \
                                     PIN_ODR_LOW(GPIOG_PIN3) |              \
                                     PIN_ODR_LOW(GPIOG_PIN4) |              \
                                     PIN_ODR_LOW(GPIOG_PIN5) |              \
                                     PIN_ODR_LOW(GPIOG_PIN6) |              \
                                     PIN_ODR_LOW(GPIOG_PIN7) |              \
                                     PIN_ODR_LOW(GPIOG_PIN8) |              \
                                     PIN_ODR_LOW(GPIOG_PIN9) |              \
                                     PIN_ODR_LOW(GPIOG_PIN10) |             \
                                     PIN_ODR_LOW(GPIOG_PIN11) |             \
                                     PIN_ODR_LOW(GPIOG_PIN12) |             \
                                     PIN_ODR_LOW(GPIOG_PIN13) |             \
                                     PIN_ODR_LOW(GPIOG_PIN14) |             \
                                     PIN_ODR_LOW(GPIOG_PIN15))
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
