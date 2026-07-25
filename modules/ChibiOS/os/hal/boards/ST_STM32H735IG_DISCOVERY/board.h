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
 * Setup for STMicroelectronics STM32H735IG_DISCOVERY board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32H735IG_DISCOVERY
#define BOARD_NAME                  "STMicroelectronics STM32H735IG_DISCOVERY"

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID                MII_LAN8742A_ID
#define BOARD_PHY_RMII

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                25000000U
#endif

#define STM32_HSE_BYPASS

/*
 * MCU type as defined in the ST header.
 */
#undef STM32H735xx
#define STM32H735xx

/*
 * IO pins assignments.
 */
#define GPIOA_ARD_D3                0U
#define GPIOA_RMII_REF_CLK          1U
#define GPIOA_RMII_MDIO             2U
#define GPIOA_LCD_B5                3U
#define GPIOA_LCD_VSYNC             4U
#define GPIOA_STMOD_13              5U
#define GPIOA_LCD_G2                6U
#define GPIOA_RMII_CRS_DV           7U
#define GPIOA_LCD_B3                8U
#define GPIOA_USB_FS_VBUS           9U
#define GPIOA_USB_FS_ID             10U
#define GPIOA_USB_FS_DM             11U
#define GPIOA_USB_FS_DP             12U
#define GPIOA_JTMS                  13U
#define GPIOA_JTCK                  14U
#define GPIOA_JTDI                  15U

#define GPIOB_LCD_G1                0U
#define GPIOB_LCD_G0                1U
#define GPIOB_OSPI1_DQS             2U
#define GPIOB_JTDO_TRACESWO         3U
#define GPIOB_NJTRST                4U
#define GPIOB_FDCAN2_RX             5U
#define GPIOB_FDCAN2_TX             6U
#define GPIOB_ARD_D9                7U
#define GPIOB_LCD_B6                8U
#define GPIOB_LCD_B7                9U
#define GPIOB_MII_RX_ER             10U
#define GPIOB_MII_TX_EN             11U
#define GPIOB_RMII_TXD0             12U
#define GPIOB_RMII_TXD1             13U
#define GPIOB_ARD_D1                14U
#define GPIOB_ARD_D0                15U

#define GPIOC_ARD_A0                0U
#define GPIOC_RMII_MDC              1U
#define GPIOC_LED2                  2U
#define GPIOC_LED_RED               2U
#define GPIOC_LED1                  3U
#define GPIOC_LED_GREEN             3U
#define GPIOC_RMII_RXD0             4U
#define GPIOC_RMII_RXD1             5U
#define GPIOC_LCD_HSYNC             6U
#define GPIOC_LCD_G6                7U
#define GPIOC_SDIO1_D0              8U
#define GPIOC_SDIO1_D1              9U
#define GPIOC_SDIO1_D2              10U
#define GPIOC_SDIO1_D3              11U
#define GPIOC_SDIO1_CK              12U
#define GPIOC_BUTTON                13U
#define GPIOC_WAKEUP                13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_LCD_B1                0U
#define GPIOD_DETECTN               1U
#define GPIOD_SDIO1_CMD             2U
#define GPIOD_LCD_G7                3U
#define GPIOD_OSPI1_IO4             4U
#define GPIOD_OSPI1_IO5             5U
#define GPIOD_LCD_B2                6U
#define GPIOD_OSPI1_IO7             7U
#define GPIOD_USART3_RX             8U
#define GPIOD_STLK_RX               8U
#define GPIOD_USART3_TX             9U
#define GPIOD_STLK_TX               9U
#define GPIOD_LCD_DISP              10U
#define GPIOD_OSPI1_IO0             11U
#define GPIOD_OSPI1_IO1             12U
#define GPIOD_OSPI1_IO3             13U
#define GPIOD_STMOD_14              14U
#define GPIOD_ARD_D6                15U

#define GPIOE_LCD_R0                0U
#define GPIOE_LCD_R6                1U
#define GPIOE_OSPI1_IO2             2U
#define GPIOE_ARD_D8                3U
#define GPIOE_PIN4                  4U
#define GPIOE_PIN5                  5U
#define GPIOE_PIN6                  6U
#define GPIOE_STMOD_17              7U
#define GPIOE_AUDIO_INT             8U
#define GPIOE_STMOD_18              9U
#define GPIOE_STMOD_19              10U
#define GPIOE_LCD_G3                11U
#define GPIOE_LCD_B4                12U
#define GPIOE_LCD_DE                13U
#define GPIOE_ARD_D5                14U
#define GPIOE_LCD_R7                15U

#define GPIOF_OSPI2_IO0             0U
#define GPIOF_OSPI2_IO1             1U
#define GPIOF_OSPI2_IO2             2U
#define GPIOF_OSPI2_IO3             3U
#define GPIOF_OSPI2_CLK             4U
#define GPIOF_SD_DETECT             5U
#define GPIOF_STMOD_1               6U
#define GPIOF_STMOD_2               7U
#define GPIOF_STMOD_3               8U
#define GPIOF_STMOD_4               9U
#define GPIOF_OSPI1_CLK             10U
#define GPIOF_STMOD_8               11U
#define GPIOF_OSPI2_DQS             12U
#define GPIOF_STMOD_20              13U
#define GPIOF_STMOD_7               14U
#define GPIOF_STMOD_10              15U

#define GPIOG_OSPI2_IO4             0U
#define GPIOG_OSPI2_IO5             1U
#define GPIOG_CTP_INT               2U
#define GPIOG_ARD_D2                3U
#define GPIOG_ARD_D4                4U
#define GPIOG_ARD_D7                5U
#define GPIOG_OSPI1_NCS             6U
#define GPIOG_LCD_CLK               7U
#define GPIOG_MEMS_LED              8U
#define GPIOG_OSPI1_IO6             9U
#define GPIOG_OSPI2_IO6             10U
#define GPIOG_OSPI2_IO7             11U
#define GPIOG_OSPI2_NCS             12U
#define GPIOG_USB_FS_OVCR           13U
#define GPIOG_LCD_B0                14U
#define GPIOG_LCD_BL_CTRL           15U

#define GPIOH_OSC_IN                0U
#define GPIOH_STMOD_12              1U
#define GPIOH_ARD_A1                2U
#define GPIOH_LCD_R1                3U
#define GPIOH_LCD_G5                4U
#define GPIOH_USB_FS_PWR_EN         5U
#define GPIOH_LCD_RST               6U
#define GPIOH_STMOD_9               7U
#define GPIOH_LCD_R2                8U
#define GPIOH_LCD_R3                9U
#define GPIOH_LCD_R4                10U
#define GPIOH_LCD_R5                11U
#define GPIOH_STMOD_11              12U
#define GPIOH_FDCAN1_TX             13U
#define GPIOH_FDCAN1_RX             14U
#define GPIOH_LCD_G4                15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
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
#define GPIOJ_PIN2                  2U
#define GPIOJ_PIN3                  3U
#define GPIOJ_PIN4                  4U
#define GPIOJ_PIN5                  5U
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
#define GPIOK_PIN3                  3U
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
#define LINE_ARD_D3                 PAL_LINE(GPIOA, 0U)
#define LINE_RMII_REF_CLK           PAL_LINE(GPIOA, 1U)
#define LINE_RMII_MDIO              PAL_LINE(GPIOA, 2U)
#define LINE_LCD_B5                 PAL_LINE(GPIOA, 3U)
#define LINE_LCD_VSYNC              PAL_LINE(GPIOA, 4U)
#define LINE_STMOD_13               PAL_LINE(GPIOA, 5U)
#define LINE_LCD_G2                 PAL_LINE(GPIOA, 6U)
#define LINE_RMII_CRS_DV            PAL_LINE(GPIOA, 7U)
#define LINE_LCD_B3                 PAL_LINE(GPIOA, 8U)
#define LINE_USB_FS_VBUS            PAL_LINE(GPIOA, 9U)
#define LINE_USB_FS_ID              PAL_LINE(GPIOA, 10U)
#define LINE_USB_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_USB_FS_DP              PAL_LINE(GPIOA, 12U)
#define LINE_JTMS                   PAL_LINE(GPIOA, 13U)
#define LINE_JTCK                   PAL_LINE(GPIOA, 14U)
#define LINE_JTDI                   PAL_LINE(GPIOA, 15U)
#define LINE_LCD_G1                 PAL_LINE(GPIOB, 0U)
#define LINE_LCD_G0                 PAL_LINE(GPIOB, 1U)
#define LINE_OSPI1_DQS              PAL_LINE(GPIOB, 2U)
#define LINE_JTDO_TRACESWO          PAL_LINE(GPIOB, 3U)
#define LINE_NJTRST                 PAL_LINE(GPIOB, 4U)
#define LINE_FDCAN2_RX              PAL_LINE(GPIOB, 5U)
#define LINE_FDCAN2_TX              PAL_LINE(GPIOB, 6U)
#define LINE_ARD_D9                 PAL_LINE(GPIOB, 7U)
#define LINE_LCD_B6                 PAL_LINE(GPIOB, 8U)
#define LINE_LCD_B7                 PAL_LINE(GPIOB, 9U)
#define LINE_MII_RX_ER              PAL_LINE(GPIOB, 10U)
#define LINE_MII_TX_EN              PAL_LINE(GPIOB, 11U)
#define LINE_RMII_TXD0              PAL_LINE(GPIOB, 12U)
#define LINE_RMII_TXD1              PAL_LINE(GPIOB, 13U)
#define LINE_ARD_D1                 PAL_LINE(GPIOB, 14U)
#define LINE_ARD_D0                 PAL_LINE(GPIOB, 15U)
#define LINE_ARD_A0                 PAL_LINE(GPIOC, 0U)
#define LINE_RMII_MDC               PAL_LINE(GPIOC, 1U)
#define LINE_LED2                   PAL_LINE(GPIOC, 2U)
#define LINE_LED_RED                PAL_LINE(GPIOC, 2U)
#define LINE_LED1                   PAL_LINE(GPIOC, 3U)
#define LINE_LED_GREEN              PAL_LINE(GPIOC, 3U)
#define LINE_RMII_RXD0              PAL_LINE(GPIOC, 4U)
#define LINE_RMII_RXD1              PAL_LINE(GPIOC, 5U)
#define LINE_LCD_HSYNC              PAL_LINE(GPIOC, 6U)
#define LINE_LCD_G6                 PAL_LINE(GPIOC, 7U)
#define LINE_SDIO1_D0               PAL_LINE(GPIOC, 8U)
#define LINE_SDIO1_D1               PAL_LINE(GPIOC, 9U)
#define LINE_SDIO1_D2               PAL_LINE(GPIOC, 10U)
#define LINE_SDIO1_D3               PAL_LINE(GPIOC, 11U)
#define LINE_SDIO1_CK               PAL_LINE(GPIOC, 12U)
#define LINE_BUTTON                 PAL_LINE(GPIOC, 13U)
#define LINE_WAKEUP                 PAL_LINE(GPIOC, 13U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)
#define LINE_LCD_B1                 PAL_LINE(GPIOD, 0U)
#define LINE_DETECTN                PAL_LINE(GPIOD, 1U)
#define LINE_SDIO1_CMD              PAL_LINE(GPIOD, 2U)
#define LINE_LCD_G7                 PAL_LINE(GPIOD, 3U)
#define LINE_OSPI1_IO4              PAL_LINE(GPIOD, 4U)
#define LINE_OSPI1_IO5              PAL_LINE(GPIOD, 5U)
#define LINE_LCD_B2                 PAL_LINE(GPIOD, 6U)
#define LINE_OSPI1_IO7              PAL_LINE(GPIOD, 7U)
#define LINE_USART3_RX              PAL_LINE(GPIOD, 8U)
#define LINE_STLK_RX                PAL_LINE(GPIOD, 8U)
#define LINE_USART3_TX              PAL_LINE(GPIOD, 9U)
#define LINE_STLK_TX                PAL_LINE(GPIOD, 9U)
#define LINE_LCD_DISP               PAL_LINE(GPIOD, 10U)
#define LINE_OSPI1_IO0              PAL_LINE(GPIOD, 11U)
#define LINE_OSPI1_IO1              PAL_LINE(GPIOD, 12U)
#define LINE_OSPI1_IO3              PAL_LINE(GPIOD, 13U)
#define LINE_STMOD_14               PAL_LINE(GPIOD, 14U)
#define LINE_ARD_D6                 PAL_LINE(GPIOD, 15U)
#define LINE_LCD_R0                 PAL_LINE(GPIOE, 0U)
#define LINE_LCD_R6                 PAL_LINE(GPIOE, 1U)
#define LINE_OSPI1_IO2              PAL_LINE(GPIOE, 2U)
#define LINE_ARD_D8                 PAL_LINE(GPIOE, 3U)
#define LINE_STMOD_17               PAL_LINE(GPIOE, 7U)
#define LINE_AUDIO_INT              PAL_LINE(GPIOE, 8U)
#define LINE_STMOD_18               PAL_LINE(GPIOE, 9U)
#define LINE_STMOD_19               PAL_LINE(GPIOE, 10U)
#define LINE_LCD_G3                 PAL_LINE(GPIOE, 11U)
#define LINE_LCD_B4                 PAL_LINE(GPIOE, 12U)
#define LINE_LCD_DE                 PAL_LINE(GPIOE, 13U)
#define LINE_ARD_D5                 PAL_LINE(GPIOE, 14U)
#define LINE_LCD_R7                 PAL_LINE(GPIOE, 15U)
#define LINE_OSPI2_IO0              PAL_LINE(GPIOF, 0U)
#define LINE_OSPI2_IO1              PAL_LINE(GPIOF, 1U)
#define LINE_OSPI2_IO2              PAL_LINE(GPIOF, 2U)
#define LINE_OSPI2_IO3              PAL_LINE(GPIOF, 3U)
#define LINE_OSPI2_CLK              PAL_LINE(GPIOF, 4U)
#define LINE_SD_DETECT              PAL_LINE(GPIOF, 5U)
#define LINE_STMOD_1                PAL_LINE(GPIOF, 6U)
#define LINE_STMOD_2                PAL_LINE(GPIOF, 7U)
#define LINE_STMOD_3                PAL_LINE(GPIOF, 8U)
#define LINE_STMOD_4                PAL_LINE(GPIOF, 9U)
#define LINE_OSPI1_CLK              PAL_LINE(GPIOF, 10U)
#define LINE_STMOD_8                PAL_LINE(GPIOF, 11U)
#define LINE_OSPI2_DQS              PAL_LINE(GPIOF, 12U)
#define LINE_STMOD_20               PAL_LINE(GPIOF, 13U)
#define LINE_STMOD_7                PAL_LINE(GPIOF, 14U)
#define LINE_STMOD_10               PAL_LINE(GPIOF, 15U)
#define LINE_OSPI2_IO4              PAL_LINE(GPIOG, 0U)
#define LINE_OSPI2_IO5              PAL_LINE(GPIOG, 1U)
#define LINE_CTP_INT                PAL_LINE(GPIOG, 2U)
#define LINE_ARD_D2                 PAL_LINE(GPIOG, 3U)
#define LINE_ARD_D4                 PAL_LINE(GPIOG, 4U)
#define LINE_ARD_D7                 PAL_LINE(GPIOG, 5U)
#define LINE_OSPI1_NCS              PAL_LINE(GPIOG, 6U)
#define LINE_LCD_CLK                PAL_LINE(GPIOG, 7U)
#define LINE_MEMS_LED               PAL_LINE(GPIOG, 8U)
#define LINE_OSPI1_IO6              PAL_LINE(GPIOG, 9U)
#define LINE_OSPI2_IO6              PAL_LINE(GPIOG, 10U)
#define LINE_OSPI2_IO7              PAL_LINE(GPIOG, 11U)
#define LINE_OSPI2_NCS              PAL_LINE(GPIOG, 12U)
#define LINE_USB_FS_OVCR            PAL_LINE(GPIOG, 13U)
#define LINE_LCD_B0                 PAL_LINE(GPIOG, 14U)
#define LINE_LCD_BL_CTRL            PAL_LINE(GPIOG, 15U)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_STMOD_12               PAL_LINE(GPIOH, 1U)
#define LINE_ARD_A1                 PAL_LINE(GPIOH, 2U)
#define LINE_LCD_R1                 PAL_LINE(GPIOH, 3U)
#define LINE_LCD_G5                 PAL_LINE(GPIOH, 4U)
#define LINE_USB_FS_PWR_EN          PAL_LINE(GPIOH, 5U)
#define LINE_LCD_RST                PAL_LINE(GPIOH, 6U)
#define LINE_STMOD_9                PAL_LINE(GPIOH, 7U)
#define LINE_LCD_R2                 PAL_LINE(GPIOH, 8U)
#define LINE_LCD_R3                 PAL_LINE(GPIOH, 9U)
#define LINE_LCD_R4                 PAL_LINE(GPIOH, 10U)
#define LINE_LCD_R5                 PAL_LINE(GPIOH, 11U)
#define LINE_STMOD_11               PAL_LINE(GPIOH, 12U)
#define LINE_FDCAN1_TX              PAL_LINE(GPIOH, 13U)
#define LINE_FDCAN1_RX              PAL_LINE(GPIOH, 14U)
#define LINE_LCD_G4                 PAL_LINE(GPIOH, 15U)

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
 * PA0  - ARD_D3                    (input pullup).
 * PA1  - RMII_REF_CLK              (alternate 11).
 * PA2  - RMII_MDIO                 (alternate 11).
 * PA3  - LCD_B5                    (alternate 14).
 * PA4  - LCD_VSYNC                 (alternate 14).
 * PA5  - STMOD_13                  (input pullup).
 * PA6  - LCD_G2                    (alternate 14).
 * PA7  - RMII_CRS_DV               (alternate 11).
 * PA8  - LCD_B3                    (alternate 14).
 * PA9  - USB_FS_VBUS               (analog).
 * PA10 - USB_FS_ID                 (alternate 10).
 * PA11 - USB_FS_DM                 (alternate 10).
 * PA12 - USB_FS_DP                 (alternate 10).
 * PA13 - JTMS                      (alternate 0).
 * PA14 - JTCK                      (alternate 0).
 * PA15 - JTDI                      (alternate 0).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_ARD_D3) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_REF_CLK) |\
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_MDIO) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_B5) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_VSYNC) |  \
                                     PIN_MODE_INPUT(GPIOA_STMOD_13) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_G2) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_CRS_DV) |\
                                     PIN_MODE_ALTERNATE(GPIOA_LCD_B3) |     \
                                     PIN_MODE_ANALOG(GPIOA_USB_FS_VBUS) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_FS_ID) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_FS_DP) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_JTMS) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_JTCK) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_JTDI))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_ARD_D3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RMII_REF_CLK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_RMII_MDIO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_B5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_VSYNC) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_STMOD_13) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_G2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RMII_CRS_DV) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_B3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_FS_VBUS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_FS_ID) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_FS_DM) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_FS_DP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTMS) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTCK) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTDI))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_ARD_D3) |     \
                                     PIN_OSPEED_HIGH(GPIOA_RMII_REF_CLK) |  \
                                     PIN_OSPEED_HIGH(GPIOA_RMII_MDIO) |     \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_B5) |        \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_VSYNC) |     \
                                     PIN_OSPEED_VERYLOW(GPIOA_STMOD_13) |   \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_G2) |        \
                                     PIN_OSPEED_HIGH(GPIOA_RMII_CRS_DV) |   \
                                     PIN_OSPEED_HIGH(GPIOA_LCD_B3) |        \
                                     PIN_OSPEED_HIGH(GPIOA_USB_FS_VBUS) |   \
                                     PIN_OSPEED_HIGH(GPIOA_USB_FS_ID) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USB_FS_DM) |     \
                                     PIN_OSPEED_HIGH(GPIOA_USB_FS_DP) |     \
                                     PIN_OSPEED_HIGH(GPIOA_JTMS) |          \
                                     PIN_OSPEED_HIGH(GPIOA_JTCK) |          \
                                     PIN_OSPEED_HIGH(GPIOA_JTDI))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_ARD_D3) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_RMII_REF_CLK) |\
                                     PIN_PUPDR_FLOATING(GPIOA_RMII_MDIO) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_B5) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_VSYNC) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_STMOD_13) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_G2) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_RMII_CRS_DV) |\
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_B3) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_FS_VBUS) |\
                                     PIN_PUPDR_FLOATING(GPIOA_USB_FS_ID) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_FS_DM) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_FS_DP) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_JTMS) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_JTCK) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_JTDI))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_ARD_D3) |            \
                                     PIN_ODR_LOW(GPIOA_RMII_REF_CLK) |      \
                                     PIN_ODR_LOW(GPIOA_RMII_MDIO) |         \
                                     PIN_ODR_LOW(GPIOA_LCD_B5) |            \
                                     PIN_ODR_LOW(GPIOA_LCD_VSYNC) |         \
                                     PIN_ODR_HIGH(GPIOA_STMOD_13) |         \
                                     PIN_ODR_LOW(GPIOA_LCD_G2) |            \
                                     PIN_ODR_LOW(GPIOA_RMII_CRS_DV) |       \
                                     PIN_ODR_LOW(GPIOA_LCD_B3) |            \
                                     PIN_ODR_LOW(GPIOA_USB_FS_VBUS) |       \
                                     PIN_ODR_LOW(GPIOA_USB_FS_ID) |         \
                                     PIN_ODR_LOW(GPIOA_USB_FS_DM) |         \
                                     PIN_ODR_LOW(GPIOA_USB_FS_DP) |         \
                                     PIN_ODR_LOW(GPIOA_JTMS) |              \
                                     PIN_ODR_LOW(GPIOA_JTCK) |              \
                                     PIN_ODR_LOW(GPIOA_JTDI))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_ARD_D3, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_RMII_REF_CLK, 11U) | \
                                     PIN_AFIO_AF(GPIOA_RMII_MDIO, 11U) |    \
                                     PIN_AFIO_AF(GPIOA_LCD_B5, 14U) |       \
                                     PIN_AFIO_AF(GPIOA_LCD_VSYNC, 14U) |    \
                                     PIN_AFIO_AF(GPIOA_STMOD_13, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_LCD_G2, 14U) |       \
                                     PIN_AFIO_AF(GPIOA_RMII_CRS_DV, 11U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_LCD_B3, 14U) |       \
                                     PIN_AFIO_AF(GPIOA_USB_FS_VBUS, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_USB_FS_ID, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_USB_FS_DM, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_USB_FS_DP, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_JTMS, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_JTCK, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_JTDI, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - LCD_G1                    (alternate 14).
 * PB1  - LCD_G0                    (alternate 14).
 * PB2  - OSPI1_DQS                 (alternate 10).
 * PB3  - JTDO_TRACESWO             (alternate 0).
 * PB4  - NJTRST                    (alternate 0).
 * PB5  - FDCAN2_RX                 (alternate 9).
 * PB6  - FDCAN2_TX                 (alternate 9).
 * PB7  - ARD_D9                    (input pullup).
 * PB8  - LCD_B6                    (alternate 14).
 * PB9  - LCD_B7                    (alternate 14).
 * PB10 - MII_RX_ER                 (alternate 11).
 * PB11 - MII_TX_EN                 (alternate 11).
 * PB12 - RMII_TXD0                 (alternate 11).
 * PB13 - RMII_TXD1                 (alternate 11).
 * PB14 - ARD_D1                    (input pullup).
 * PB15 - ARD_D0                    (input pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(GPIOB_LCD_G1) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_G0) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_OSPI1_DQS) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_JTDO_TRACESWO) |\
                                     PIN_MODE_ALTERNATE(GPIOB_NJTRST) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_FDCAN2_RX) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_FDCAN2_TX) |  \
                                     PIN_MODE_INPUT(GPIOB_ARD_D9) |         \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_B6) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_LCD_B7) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_MII_RX_ER) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_MII_TX_EN) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_RMII_TXD0) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_RMII_TXD1) |  \
                                     PIN_MODE_INPUT(GPIOB_ARD_D1) |         \
                                     PIN_MODE_INPUT(GPIOB_ARD_D0))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_LCD_G1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_G0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OSPI1_DQS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTDO_TRACESWO) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_NJTRST) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_FDCAN2_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_FDCAN2_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_B6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_B7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MII_RX_ER) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MII_TX_EN) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_RMII_TXD0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_RMII_TXD1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D0))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_LCD_G1) |        \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_G0) |        \
                                     PIN_OSPEED_HIGH(GPIOB_OSPI1_DQS) |     \
                                     PIN_OSPEED_HIGH(GPIOB_JTDO_TRACESWO) | \
                                     PIN_OSPEED_HIGH(GPIOB_NJTRST) |        \
                                     PIN_OSPEED_HIGH(GPIOB_FDCAN2_RX) |     \
                                     PIN_OSPEED_HIGH(GPIOB_FDCAN2_TX) |     \
                                     PIN_OSPEED_VERYLOW(GPIOB_ARD_D9) |     \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_B6) |        \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_B7) |        \
                                     PIN_OSPEED_HIGH(GPIOB_MII_RX_ER) |     \
                                     PIN_OSPEED_HIGH(GPIOB_MII_TX_EN) |     \
                                     PIN_OSPEED_HIGH(GPIOB_RMII_TXD0) |     \
                                     PIN_OSPEED_HIGH(GPIOB_RMII_TXD1) |     \
                                     PIN_OSPEED_VERYLOW(GPIOB_ARD_D1) |     \
                                     PIN_OSPEED_VERYLOW(GPIOB_ARD_D0))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_LCD_G1) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_G0) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_OSPI1_DQS) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_JTDO_TRACESWO) |\
                                     PIN_PUPDR_PULLUP(GPIOB_NJTRST) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_FDCAN2_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_FDCAN2_TX) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_ARD_D9) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_B6) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_B7) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_MII_RX_ER) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_MII_TX_EN) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_RMII_TXD0) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_RMII_TXD1) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_ARD_D1) |       \
                                     PIN_PUPDR_PULLUP(GPIOB_ARD_D0))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_LCD_G1) |            \
                                     PIN_ODR_LOW(GPIOB_LCD_G0) |            \
                                     PIN_ODR_LOW(GPIOB_OSPI1_DQS) |         \
                                     PIN_ODR_LOW(GPIOB_JTDO_TRACESWO) |     \
                                     PIN_ODR_LOW(GPIOB_NJTRST) |            \
                                     PIN_ODR_LOW(GPIOB_FDCAN2_RX) |         \
                                     PIN_ODR_LOW(GPIOB_FDCAN2_TX) |         \
                                     PIN_ODR_LOW(GPIOB_ARD_D9) |            \
                                     PIN_ODR_LOW(GPIOB_LCD_B6) |            \
                                     PIN_ODR_LOW(GPIOB_LCD_B7) |            \
                                     PIN_ODR_HIGH(GPIOB_MII_RX_ER) |        \
                                     PIN_ODR_HIGH(GPIOB_MII_TX_EN) |        \
                                     PIN_ODR_HIGH(GPIOB_RMII_TXD0) |        \
                                     PIN_ODR_HIGH(GPIOB_RMII_TXD1) |        \
                                     PIN_ODR_LOW(GPIOB_ARD_D1) |            \
                                     PIN_ODR_LOW(GPIOB_ARD_D0))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_LCD_G1, 14U) |       \
                                     PIN_AFIO_AF(GPIOB_LCD_G0, 14U) |       \
                                     PIN_AFIO_AF(GPIOB_OSPI1_DQS, 10U) |    \
                                     PIN_AFIO_AF(GPIOB_JTDO_TRACESWO, 0U) | \
                                     PIN_AFIO_AF(GPIOB_NJTRST, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_FDCAN2_RX, 9U) |     \
                                     PIN_AFIO_AF(GPIOB_FDCAN2_TX, 9U) |     \
                                     PIN_AFIO_AF(GPIOB_ARD_D9, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_LCD_B6, 14U) |       \
                                     PIN_AFIO_AF(GPIOB_LCD_B7, 14U) |       \
                                     PIN_AFIO_AF(GPIOB_MII_RX_ER, 11U) |    \
                                     PIN_AFIO_AF(GPIOB_MII_TX_EN, 11U) |    \
                                     PIN_AFIO_AF(GPIOB_RMII_TXD0, 11U) |    \
                                     PIN_AFIO_AF(GPIOB_RMII_TXD1, 11U) |    \
                                     PIN_AFIO_AF(GPIOB_ARD_D1, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_ARD_D0, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - ARD_A0                    (input pullup).
 * PC1  - RMII_MDC                  (alternate 11).
 * PC2  - LED2 LED_RED              (output opendrain maximum).
 * PC3  - LED1 LED_GREEN            (output opendrain maximum).
 * PC4  - RMII_RXD0                 (alternate 11).
 * PC5  - RMII_RXD1                 (alternate 11).
 * PC6  - LCD_HSYNC                 (alternate 14).
 * PC7  - LCD_G6                    (alternate 14).
 * PC8  - SDIO1_D0                  (alternate 12).
 * PC9  - SDIO1_D1                  (alternate 12).
 * PC10 - SDIO1_D2                  (alternate 12).
 * PC11 - SDIO1_D3                  (alternate 12).
 * PC12 - SDIO1_CK                  (alternate 12).
 * PC13 - BUTTON WAKEUP             (input floating).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_ARD_A0) |         \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_MDC) |   \
                                     PIN_MODE_OUTPUT(GPIOC_LED2) |          \
                                     PIN_MODE_OUTPUT(GPIOC_LED1) |          \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_RXD0) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_RXD1) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_LCD_HSYNC) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_LCD_G6) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D0) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D1) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D2) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D3) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_CK) |   \
                                     PIN_MODE_INPUT(GPIOC_BUTTON) |         \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_ARD_A0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_RMII_MDC) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED2) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOC_LED1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_RMII_RXD0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_RMII_RXD1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LCD_HSYNC) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LCD_G6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_CK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_BUTTON) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_ARD_A0) |     \
                                     PIN_OSPEED_HIGH(GPIOC_RMII_MDC) |      \
                                     PIN_OSPEED_HIGH(GPIOC_LED2) |          \
                                     PIN_OSPEED_HIGH(GPIOC_LED1) |          \
                                     PIN_OSPEED_HIGH(GPIOC_RMII_RXD0) |     \
                                     PIN_OSPEED_HIGH(GPIOC_RMII_RXD1) |     \
                                     PIN_OSPEED_HIGH(GPIOC_LCD_HSYNC) |     \
                                     PIN_OSPEED_HIGH(GPIOC_LCD_G6) |        \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D0) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D1) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D2) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D3) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_CK) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_BUTTON) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_IN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(GPIOC_ARD_A0) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_MDC) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_LED2) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_LED1) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_RXD0) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_RXD1) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_LCD_HSYNC) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_LCD_G6) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D0) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D1) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D2) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D3) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_CK) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_BUTTON) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_ARD_A0) |            \
                                     PIN_ODR_HIGH(GPIOC_RMII_MDC) |         \
                                     PIN_ODR_HIGH(GPIOC_LED2) |             \
                                     PIN_ODR_HIGH(GPIOC_LED1) |             \
                                     PIN_ODR_HIGH(GPIOC_RMII_RXD0) |        \
                                     PIN_ODR_HIGH(GPIOC_RMII_RXD1) |        \
                                     PIN_ODR_LOW(GPIOC_LCD_HSYNC) |         \
                                     PIN_ODR_LOW(GPIOC_LCD_G6) |            \
                                     PIN_ODR_LOW(GPIOC_SDIO1_D0) |          \
                                     PIN_ODR_LOW(GPIOC_SDIO1_D1) |          \
                                     PIN_ODR_LOW(GPIOC_SDIO1_D2) |          \
                                     PIN_ODR_LOW(GPIOC_SDIO1_D3) |          \
                                     PIN_ODR_LOW(GPIOC_SDIO1_CK) |          \
                                     PIN_ODR_HIGH(GPIOC_BUTTON) |           \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_ARD_A0, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_RMII_MDC, 11U) |     \
                                     PIN_AFIO_AF(GPIOC_LED2, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_LED1, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_RMII_RXD0, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_RMII_RXD1, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_LCD_HSYNC, 14U) |    \
                                     PIN_AFIO_AF(GPIOC_LCD_G6, 14U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SDIO1_D0, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SDIO1_D1, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SDIO1_D2, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SDIO1_D3, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SDIO1_CK, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_BUTTON, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - LCD_B1                    (alternate 14).
 * PD1  - DETECTN                   (input pullup).
 * PD2  - SDIO1_CMD                 (alternate 12).
 * PD3  - LCD_G7                    (alternate 14).
 * PD4  - OSPI1_IO4                 (alternate 10).
 * PD5  - OSPI1_IO5                 (alternate 10).
 * PD6  - LCD_B2                    (alternate 14).
 * PD7  - OSPI1_IO7                 (alternate 10).
 * PD8  - USART3_RX STLK_RX         (alternate 7).
 * PD9  - USART3_TX STLK_TX         (alternate 7).
 * PD10 - LCD_DISP                  (output pushpull minimum).
 * PD11 - OSPI1_IO0                 (alternate 9).
 * PD12 - OSPI1_IO1                 (alternate 9).
 * PD13 - OSPI1_IO3                 (alternate 9).
 * PD14 - STMOD_14                  (input pullup).
 * PD15 - ARD_D6                    (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_LCD_B1) |     \
                                     PIN_MODE_INPUT(GPIOD_DETECTN) |        \
                                     PIN_MODE_ALTERNATE(GPIOD_SDIO1_CMD) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_G7) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_OSPI1_IO4) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_OSPI1_IO5) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_LCD_B2) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_OSPI1_IO7) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_USART3_RX) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_USART3_TX) |  \
                                     PIN_MODE_OUTPUT(GPIOD_LCD_DISP) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_OSPI1_IO0) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_OSPI1_IO1) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_OSPI1_IO3) |  \
                                     PIN_MODE_INPUT(GPIOD_STMOD_14) |       \
                                     PIN_MODE_INPUT(GPIOD_ARD_D6))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_LCD_B1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DETECTN) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDIO1_CMD) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_G7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_OSPI1_IO4) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_OSPI1_IO5) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_B2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_OSPI1_IO7) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART3_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_USART3_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_DISP) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_OSPI1_IO0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_OSPI1_IO1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_OSPI1_IO3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_STMOD_14) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_ARD_D6))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_LCD_B1) |        \
                                     PIN_OSPEED_VERYLOW(GPIOD_DETECTN) |    \
                                     PIN_OSPEED_HIGH(GPIOD_SDIO1_CMD) |     \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_G7) |        \
                                     PIN_OSPEED_HIGH(GPIOD_OSPI1_IO4) |     \
                                     PIN_OSPEED_HIGH(GPIOD_OSPI1_IO5) |     \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_B2) |        \
                                     PIN_OSPEED_HIGH(GPIOD_OSPI1_IO7) |     \
                                     PIN_OSPEED_HIGH(GPIOD_USART3_RX) |     \
                                     PIN_OSPEED_HIGH(GPIOD_USART3_TX) |     \
                                     PIN_OSPEED_VERYLOW(GPIOD_LCD_DISP) |   \
                                     PIN_OSPEED_HIGH(GPIOD_OSPI1_IO0) |     \
                                     PIN_OSPEED_HIGH(GPIOD_OSPI1_IO1) |     \
                                     PIN_OSPEED_HIGH(GPIOD_OSPI1_IO3) |     \
                                     PIN_OSPEED_VERYLOW(GPIOD_STMOD_14) |   \
                                     PIN_OSPEED_VERYLOW(GPIOD_ARD_D6))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_LCD_B1) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_DETECTN) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_SDIO1_CMD) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_G7) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_OSPI1_IO4) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_OSPI1_IO5) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_B2) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_OSPI1_IO7) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_USART3_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_USART3_TX) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_LCD_DISP) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_OSPI1_IO0) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_OSPI1_IO1) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_OSPI1_IO3) |  \
                                     PIN_PUPDR_PULLUP(GPIOD_STMOD_14) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_ARD_D6))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_LCD_B1) |            \
                                     PIN_ODR_HIGH(GPIOD_DETECTN) |          \
                                     PIN_ODR_LOW(GPIOD_SDIO1_CMD) |         \
                                     PIN_ODR_LOW(GPIOD_LCD_G7) |            \
                                     PIN_ODR_LOW(GPIOD_OSPI1_IO4) |         \
                                     PIN_ODR_LOW(GPIOD_OSPI1_IO5) |         \
                                     PIN_ODR_LOW(GPIOD_LCD_B2) |            \
                                     PIN_ODR_LOW(GPIOD_OSPI1_IO7) |         \
                                     PIN_ODR_HIGH(GPIOD_USART3_RX) |        \
                                     PIN_ODR_HIGH(GPIOD_USART3_TX) |        \
                                     PIN_ODR_LOW(GPIOD_LCD_DISP) |          \
                                     PIN_ODR_LOW(GPIOD_OSPI1_IO0) |         \
                                     PIN_ODR_LOW(GPIOD_OSPI1_IO1) |         \
                                     PIN_ODR_LOW(GPIOD_OSPI1_IO3) |         \
                                     PIN_ODR_HIGH(GPIOD_STMOD_14) |         \
                                     PIN_ODR_LOW(GPIOD_ARD_D6))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_LCD_B1, 14U) |       \
                                     PIN_AFIO_AF(GPIOD_DETECTN, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_SDIO1_CMD, 12U) |    \
                                     PIN_AFIO_AF(GPIOD_LCD_G7, 14U) |       \
                                     PIN_AFIO_AF(GPIOD_OSPI1_IO4, 10U) |    \
                                     PIN_AFIO_AF(GPIOD_OSPI1_IO5, 10U) |    \
                                     PIN_AFIO_AF(GPIOD_LCD_B2, 14U) |       \
                                     PIN_AFIO_AF(GPIOD_OSPI1_IO7, 10U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_USART3_RX, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_USART3_TX, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_LCD_DISP, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_OSPI1_IO0, 9U) |     \
                                     PIN_AFIO_AF(GPIOD_OSPI1_IO1, 9U) |     \
                                     PIN_AFIO_AF(GPIOD_OSPI1_IO3, 9U) |     \
                                     PIN_AFIO_AF(GPIOD_STMOD_14, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_ARD_D6, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - LCD_R0                    (alternate 14).
 * PE1  - LCD_R6                    (alternate 14).
 * PE2  - OSPI1_IO2                 (alternate 9).
 * PE3  - ARD_D8                    (input pullup).
 * PE4  - PIN4                      (input pullup).
 * PE5  - PIN5                      (input pullup).
 * PE6  - PIN6                      (input pullup).
 * PE7  - STMOD_17                  (input pullup).
 * PE8  - AUDIO_INT                 (input pullup).
 * PE9  - STMOD_18                  (input pullup).
 * PE10 - STMOD_19                  (input pullup).
 * PE11 - LCD_G3                    (alternate 14).
 * PE12 - LCD_B4                    (alternate 14).
 * PE13 - LCD_DE                    (alternate 14).
 * PE14 - ARD_D5                    (input pullup).
 * PE15 - LCD_R7                    (alternate 14).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_LCD_R0) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_LCD_R6) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_OSPI1_IO2) |  \
                                     PIN_MODE_INPUT(GPIOE_ARD_D8) |         \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOE_STMOD_17) |       \
                                     PIN_MODE_INPUT(GPIOE_AUDIO_INT) |      \
                                     PIN_MODE_INPUT(GPIOE_STMOD_18) |       \
                                     PIN_MODE_INPUT(GPIOE_STMOD_19) |       \
                                     PIN_MODE_ALTERNATE(GPIOE_LCD_G3) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_LCD_B4) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_LCD_DE) |     \
                                     PIN_MODE_INPUT(GPIOE_ARD_D5) |         \
                                     PIN_MODE_ALTERNATE(GPIOE_LCD_R7))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_LCD_R0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_LCD_R6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_OSPI1_IO2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_ARD_D8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_STMOD_17) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_AUDIO_INT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_STMOD_18) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_STMOD_19) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_LCD_G3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_LCD_B4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_LCD_DE) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_ARD_D5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_LCD_R7))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_LCD_R0) |        \
                                     PIN_OSPEED_HIGH(GPIOE_LCD_R6) |        \
                                     PIN_OSPEED_HIGH(GPIOE_OSPI1_IO2) |     \
                                     PIN_OSPEED_VERYLOW(GPIOE_ARD_D8) |     \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_STMOD_17) |   \
                                     PIN_OSPEED_VERYLOW(GPIOE_AUDIO_INT) |  \
                                     PIN_OSPEED_VERYLOW(GPIOE_STMOD_18) |   \
                                     PIN_OSPEED_VERYLOW(GPIOE_STMOD_19) |   \
                                     PIN_OSPEED_HIGH(GPIOE_LCD_G3) |        \
                                     PIN_OSPEED_HIGH(GPIOE_LCD_B4) |        \
                                     PIN_OSPEED_HIGH(GPIOE_LCD_DE) |        \
                                     PIN_OSPEED_VERYLOW(GPIOE_ARD_D5) |     \
                                     PIN_OSPEED_HIGH(GPIOE_LCD_R7))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_LCD_R0) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_LCD_R6) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_OSPI1_IO2) |  \
                                     PIN_PUPDR_PULLUP(GPIOE_ARD_D8) |       \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_STMOD_17) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_AUDIO_INT) |    \
                                     PIN_PUPDR_PULLUP(GPIOE_STMOD_18) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_STMOD_19) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_LCD_G3) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_LCD_B4) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_LCD_DE) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_ARD_D5) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_LCD_R7))
#define VAL_GPIOE_ODR               (PIN_ODR_LOW(GPIOE_LCD_R0) |            \
                                     PIN_ODR_LOW(GPIOE_LCD_R6) |            \
                                     PIN_ODR_LOW(GPIOE_OSPI1_IO2) |         \
                                     PIN_ODR_LOW(GPIOE_ARD_D8) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOE_STMOD_17) |         \
                                     PIN_ODR_HIGH(GPIOE_AUDIO_INT) |        \
                                     PIN_ODR_HIGH(GPIOE_STMOD_18) |         \
                                     PIN_ODR_HIGH(GPIOE_STMOD_19) |         \
                                     PIN_ODR_LOW(GPIOE_LCD_G3) |            \
                                     PIN_ODR_LOW(GPIOE_LCD_B4) |            \
                                     PIN_ODR_LOW(GPIOE_LCD_DE) |            \
                                     PIN_ODR_LOW(GPIOE_ARD_D5) |            \
                                     PIN_ODR_LOW(GPIOE_LCD_R7))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_LCD_R0, 14U) |       \
                                     PIN_AFIO_AF(GPIOE_LCD_R6, 14U) |       \
                                     PIN_AFIO_AF(GPIOE_OSPI1_IO2, 9U) |     \
                                     PIN_AFIO_AF(GPIOE_ARD_D8, 0U) |        \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_STMOD_17, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_AUDIO_INT, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_STMOD_18, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_STMOD_19, 0U) |      \
                                     PIN_AFIO_AF(GPIOE_LCD_G3, 14U) |       \
                                     PIN_AFIO_AF(GPIOE_LCD_B4, 14U) |       \
                                     PIN_AFIO_AF(GPIOE_LCD_DE, 14U) |       \
                                     PIN_AFIO_AF(GPIOE_ARD_D5, 0U) |        \
                                     PIN_AFIO_AF(GPIOE_LCD_R7, 14U))

/*
 * GPIOF setup:
 *
 * PF0  - OSPI2_IO0                 (alternate 9).
 * PF1  - OSPI2_IO1                 (alternate 9).
 * PF2  - OSPI2_IO2                 (alternate 9).
 * PF3  - OSPI2_IO3                 (alternate 9).
 * PF4  - OSPI2_CLK                 (alternate 9).
 * PF5  - SD_DETECT                 (input pullup).
 * PF6  - STMOD_1                   (input pullup).
 * PF7  - STMOD_2                   (input pullup).
 * PF8  - STMOD_3                   (input pullup).
 * PF9  - STMOD_4                   (input pullup).
 * PF10 - OSPI1_CLK                 (alternate 9).
 * PF11 - STMOD_8                   (input pullup).
 * PF12 - OSPI2_DQS                 (alternate 9).
 * PF13 - STMOD_20                  (input pullup).
 * PF14 - STMOD_7                   (input pullup).
 * PF15 - STMOD_10                  (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_OSPI2_IO0) |  \
                                     PIN_MODE_ALTERNATE(GPIOF_OSPI2_IO1) |  \
                                     PIN_MODE_ALTERNATE(GPIOF_OSPI2_IO2) |  \
                                     PIN_MODE_ALTERNATE(GPIOF_OSPI2_IO3) |  \
                                     PIN_MODE_ALTERNATE(GPIOF_OSPI2_CLK) |  \
                                     PIN_MODE_INPUT(GPIOF_SD_DETECT) |      \
                                     PIN_MODE_INPUT(GPIOF_STMOD_1) |        \
                                     PIN_MODE_INPUT(GPIOF_STMOD_2) |        \
                                     PIN_MODE_INPUT(GPIOF_STMOD_3) |        \
                                     PIN_MODE_INPUT(GPIOF_STMOD_4) |        \
                                     PIN_MODE_ALTERNATE(GPIOF_OSPI1_CLK) |  \
                                     PIN_MODE_INPUT(GPIOF_STMOD_8) |        \
                                     PIN_MODE_ALTERNATE(GPIOF_OSPI2_DQS) |  \
                                     PIN_MODE_INPUT(GPIOF_STMOD_20) |       \
                                     PIN_MODE_INPUT(GPIOF_STMOD_7) |        \
                                     PIN_MODE_INPUT(GPIOF_STMOD_10))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_OSPI2_IO0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSPI2_IO1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSPI2_IO2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSPI2_IO3) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSPI2_CLK) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SD_DETECT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STMOD_1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STMOD_2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STMOD_3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STMOD_4) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSPI1_CLK) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STMOD_8) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSPI2_DQS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STMOD_20) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STMOD_7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_STMOD_10))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_OSPI2_IO0) |     \
                                     PIN_OSPEED_HIGH(GPIOF_OSPI2_IO1) |     \
                                     PIN_OSPEED_HIGH(GPIOF_OSPI2_IO2) |     \
                                     PIN_OSPEED_HIGH(GPIOF_OSPI2_IO3) |     \
                                     PIN_OSPEED_HIGH(GPIOF_OSPI2_CLK) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_SD_DETECT) |  \
                                     PIN_OSPEED_VERYLOW(GPIOF_STMOD_1) |    \
                                     PIN_OSPEED_VERYLOW(GPIOF_STMOD_2) |    \
                                     PIN_OSPEED_VERYLOW(GPIOF_STMOD_3) |    \
                                     PIN_OSPEED_VERYLOW(GPIOF_STMOD_4) |    \
                                     PIN_OSPEED_HIGH(GPIOF_OSPI1_CLK) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_STMOD_8) |    \
                                     PIN_OSPEED_HIGH(GPIOF_OSPI2_DQS) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_STMOD_20) |   \
                                     PIN_OSPEED_VERYLOW(GPIOF_STMOD_7) |    \
                                     PIN_OSPEED_VERYLOW(GPIOF_STMOD_10))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_OSPI2_IO0) |  \
                                     PIN_PUPDR_FLOATING(GPIOF_OSPI2_IO1) |  \
                                     PIN_PUPDR_FLOATING(GPIOF_OSPI2_IO2) |  \
                                     PIN_PUPDR_FLOATING(GPIOF_OSPI2_IO3) |  \
                                     PIN_PUPDR_FLOATING(GPIOF_OSPI2_CLK) |  \
                                     PIN_PUPDR_PULLUP(GPIOF_SD_DETECT) |    \
                                     PIN_PUPDR_PULLUP(GPIOF_STMOD_1) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_STMOD_2) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_STMOD_3) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_STMOD_4) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_OSPI1_CLK) |  \
                                     PIN_PUPDR_PULLUP(GPIOF_STMOD_8) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_OSPI2_DQS) |  \
                                     PIN_PUPDR_PULLUP(GPIOF_STMOD_20) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_STMOD_7) |      \
                                     PIN_PUPDR_PULLUP(GPIOF_STMOD_10))
#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_OSPI2_IO0) |         \
                                     PIN_ODR_LOW(GPIOF_OSPI2_IO1) |         \
                                     PIN_ODR_LOW(GPIOF_OSPI2_IO2) |         \
                                     PIN_ODR_LOW(GPIOF_OSPI2_IO3) |         \
                                     PIN_ODR_LOW(GPIOF_OSPI2_CLK) |         \
                                     PIN_ODR_LOW(GPIOF_SD_DETECT) |         \
                                     PIN_ODR_HIGH(GPIOF_STMOD_1) |          \
                                     PIN_ODR_HIGH(GPIOF_STMOD_2) |          \
                                     PIN_ODR_HIGH(GPIOF_STMOD_3) |          \
                                     PIN_ODR_HIGH(GPIOF_STMOD_4) |          \
                                     PIN_ODR_LOW(GPIOF_OSPI1_CLK) |         \
                                     PIN_ODR_HIGH(GPIOF_STMOD_8) |          \
                                     PIN_ODR_LOW(GPIOF_OSPI2_DQS) |         \
                                     PIN_ODR_HIGH(GPIOF_STMOD_20) |         \
                                     PIN_ODR_HIGH(GPIOF_STMOD_7) |          \
                                     PIN_ODR_HIGH(GPIOF_STMOD_10))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSPI2_IO0, 9U) |     \
                                     PIN_AFIO_AF(GPIOF_OSPI2_IO1, 9U) |     \
                                     PIN_AFIO_AF(GPIOF_OSPI2_IO2, 9U) |     \
                                     PIN_AFIO_AF(GPIOF_OSPI2_IO3, 9U) |     \
                                     PIN_AFIO_AF(GPIOF_OSPI2_CLK, 9U) |     \
                                     PIN_AFIO_AF(GPIOF_SD_DETECT, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_STMOD_1, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_STMOD_2, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_STMOD_3, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_STMOD_4, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_OSPI1_CLK, 9U) |     \
                                     PIN_AFIO_AF(GPIOF_STMOD_8, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_OSPI2_DQS, 9U) |     \
                                     PIN_AFIO_AF(GPIOF_STMOD_20, 0U) |      \
                                     PIN_AFIO_AF(GPIOF_STMOD_7, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_STMOD_10, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - OSPI2_IO4                 (alternate 9).
 * PG1  - OSPI2_IO5                 (alternate 9).
 * PG2  - CTP_INT                   (input pullup).
 * PG3  - ARD_D2                    (input pullup).
 * PG4  - ARD_D4                    (input pullup).
 * PG5  - ARD_D7                    (input pullup).
 * PG6  - OSPI1_NCS                 (alternate 10).
 * PG7  - LCD_CLK                   (alternate 14).
 * PG8  - MEMS_LED                  (input pullup).
 * PG9  - OSPI1_IO6                 (alternate 9).
 * PG10 - OSPI2_IO6                 (alternate 3).
 * PG11 - OSPI2_IO7                 (alternate 9).
 * PG12 - OSPI2_NCS                 (alternate 3).
 * PG13 - USB_FS_OVCR               (input pullup).
 * PG14 - LCD_B0                    (alternate 14).
 * PG15 - LCD_BL_CTRL               (output pushpull minimum).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_ALTERNATE(GPIOG_OSPI2_IO4) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_OSPI2_IO5) |  \
                                     PIN_MODE_INPUT(GPIOG_CTP_INT) |        \
                                     PIN_MODE_INPUT(GPIOG_ARD_D2) |         \
                                     PIN_MODE_INPUT(GPIOG_ARD_D4) |         \
                                     PIN_MODE_INPUT(GPIOG_ARD_D7) |         \
                                     PIN_MODE_ALTERNATE(GPIOG_OSPI1_NCS) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_LCD_CLK) |    \
                                     PIN_MODE_INPUT(GPIOG_MEMS_LED) |       \
                                     PIN_MODE_ALTERNATE(GPIOG_OSPI1_IO6) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_OSPI2_IO6) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_OSPI2_IO7) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_OSPI2_NCS) |  \
                                     PIN_MODE_INPUT(GPIOG_USB_FS_OVCR) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_LCD_B0) |     \
                                     PIN_MODE_OUTPUT(GPIOG_LCD_BL_CTRL))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_OSPI2_IO4) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_OSPI2_IO5) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_CTP_INT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_OSPI1_NCS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_LCD_CLK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MEMS_LED) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_OSPI1_IO6) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_OSPI2_IO6) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_OSPI2_IO7) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_OSPI2_NCS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_USB_FS_OVCR) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_LCD_B0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_LCD_BL_CTRL))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_OSPI2_IO4) |     \
                                     PIN_OSPEED_HIGH(GPIOG_OSPI2_IO5) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_CTP_INT) |    \
                                     PIN_OSPEED_VERYLOW(GPIOG_ARD_D2) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_ARD_D4) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_ARD_D7) |     \
                                     PIN_OSPEED_HIGH(GPIOG_OSPI1_NCS) |     \
                                     PIN_OSPEED_HIGH(GPIOG_LCD_CLK) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_MEMS_LED) |   \
                                     PIN_OSPEED_HIGH(GPIOG_OSPI1_IO6) |     \
                                     PIN_OSPEED_HIGH(GPIOG_OSPI2_IO6) |     \
                                     PIN_OSPEED_HIGH(GPIOG_OSPI2_IO7) |     \
                                     PIN_OSPEED_HIGH(GPIOG_OSPI2_NCS) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_USB_FS_OVCR) |\
                                     PIN_OSPEED_HIGH(GPIOG_LCD_B0) |        \
                                     PIN_OSPEED_VERYLOW(GPIOG_LCD_BL_CTRL))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_OSPI2_IO4) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_OSPI2_IO5) |  \
                                     PIN_PUPDR_PULLUP(GPIOG_CTP_INT) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D2) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D4) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D7) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_OSPI1_NCS) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_LCD_CLK) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_MEMS_LED) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_OSPI1_IO6) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_OSPI2_IO6) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_OSPI2_IO7) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_OSPI2_NCS) |  \
                                     PIN_PUPDR_PULLUP(GPIOG_USB_FS_OVCR) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_LCD_B0) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_LCD_BL_CTRL))
#define VAL_GPIOG_ODR               (PIN_ODR_LOW(GPIOG_OSPI2_IO4) |         \
                                     PIN_ODR_LOW(GPIOG_OSPI2_IO5) |         \
                                     PIN_ODR_HIGH(GPIOG_CTP_INT) |          \
                                     PIN_ODR_LOW(GPIOG_ARD_D2) |            \
                                     PIN_ODR_LOW(GPIOG_ARD_D4) |            \
                                     PIN_ODR_LOW(GPIOG_ARD_D7) |            \
                                     PIN_ODR_LOW(GPIOG_OSPI1_NCS) |         \
                                     PIN_ODR_LOW(GPIOG_LCD_CLK) |           \
                                     PIN_ODR_HIGH(GPIOG_MEMS_LED) |         \
                                     PIN_ODR_LOW(GPIOG_OSPI1_IO6) |         \
                                     PIN_ODR_LOW(GPIOG_OSPI2_IO6) |         \
                                     PIN_ODR_LOW(GPIOG_OSPI2_IO7) |         \
                                     PIN_ODR_LOW(GPIOG_OSPI2_NCS) |         \
                                     PIN_ODR_HIGH(GPIOG_USB_FS_OVCR) |      \
                                     PIN_ODR_LOW(GPIOG_LCD_B0) |            \
                                     PIN_ODR_LOW(GPIOG_LCD_BL_CTRL))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_OSPI2_IO4, 9U) |     \
                                     PIN_AFIO_AF(GPIOG_OSPI2_IO5, 9U) |     \
                                     PIN_AFIO_AF(GPIOG_CTP_INT, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_ARD_D2, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_ARD_D4, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_ARD_D7, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_OSPI1_NCS, 10U) |    \
                                     PIN_AFIO_AF(GPIOG_LCD_CLK, 14U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_MEMS_LED, 0U) |      \
                                     PIN_AFIO_AF(GPIOG_OSPI1_IO6, 9U) |     \
                                     PIN_AFIO_AF(GPIOG_OSPI2_IO6, 3U) |     \
                                     PIN_AFIO_AF(GPIOG_OSPI2_IO7, 9U) |     \
                                     PIN_AFIO_AF(GPIOG_OSPI2_NCS, 3U) |     \
                                     PIN_AFIO_AF(GPIOG_USB_FS_OVCR, 0U) |   \
                                     PIN_AFIO_AF(GPIOG_LCD_B0, 14U) |       \
                                     PIN_AFIO_AF(GPIOG_LCD_BL_CTRL, 0U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - STMOD_12                  (input pullup).
 * PH2  - ARD_A1                    (input pullup).
 * PH3  - LCD_R1                    (alternate 14).
 * PH4  - LCD_G5                    (alternate 14).
 * PH5  - USB_FS_PWR_EN             (output pushpull minimum).
 * PH6  - LCD_RST                   (output pushpull minimum).
 * PH7  - STMOD_9                   (input pullup).
 * PH8  - LCD_R2                    (alternate 14).
 * PH9  - LCD_R3                    (alternate 14).
 * PH10 - LCD_R4                    (alternate 14).
 * PH11 - LCD_R5                    (alternate 14).
 * PH12 - STMOD_11                  (input pullup).
 * PH13 - FDCAN1_TX                 (alternate 9).
 * PH14 - FDCAN1_RX                 (alternate 9).
 * PH15 - LCD_G4                    (alternate 14).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_STMOD_12) |       \
                                     PIN_MODE_INPUT(GPIOH_ARD_A1) |         \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_R1) |     \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_G5) |     \
                                     PIN_MODE_OUTPUT(GPIOH_USB_FS_PWR_EN) | \
                                     PIN_MODE_OUTPUT(GPIOH_LCD_RST) |       \
                                     PIN_MODE_INPUT(GPIOH_STMOD_9) |        \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_R2) |     \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_R3) |     \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_R4) |     \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_R5) |     \
                                     PIN_MODE_INPUT(GPIOH_STMOD_11) |       \
                                     PIN_MODE_ALTERNATE(GPIOH_FDCAN1_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOH_FDCAN1_RX) |  \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_G4))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_STMOD_12) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOH_ARD_A1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_R1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_G5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_USB_FS_PWR_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_RST) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_STMOD_9) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_R2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_R3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_R4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_R5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_STMOD_11) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FDCAN1_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FDCAN1_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_G4))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_VERYLOW(GPIOH_STMOD_12) |   \
                                     PIN_OSPEED_VERYLOW(GPIOH_ARD_A1) |     \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_R1) |        \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_G5) |        \
                                     PIN_OSPEED_VERYLOW(GPIOH_USB_FS_PWR_EN) |\
                                     PIN_OSPEED_VERYLOW(GPIOH_LCD_RST) |    \
                                     PIN_OSPEED_VERYLOW(GPIOH_STMOD_9) |    \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_R2) |        \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_R3) |        \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_R4) |        \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_R5) |        \
                                     PIN_OSPEED_VERYLOW(GPIOH_STMOD_11) |   \
                                     PIN_OSPEED_HIGH(GPIOH_FDCAN1_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOH_FDCAN1_RX) |     \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_G4))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_PULLUP(GPIOH_STMOD_12) |     \
                                     PIN_PUPDR_PULLUP(GPIOH_ARD_A1) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_R1) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_G5) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_USB_FS_PWR_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_RST) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_STMOD_9) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_R2) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_R3) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_R4) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_R5) |     \
                                     PIN_PUPDR_PULLUP(GPIOH_STMOD_11) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_FDCAN1_TX) |  \
                                     PIN_PUPDR_FLOATING(GPIOH_FDCAN1_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_G4))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_STMOD_12) |         \
                                     PIN_ODR_LOW(GPIOH_ARD_A1) |            \
                                     PIN_ODR_LOW(GPIOH_LCD_R1) |            \
                                     PIN_ODR_LOW(GPIOH_LCD_G5) |            \
                                     PIN_ODR_LOW(GPIOH_USB_FS_PWR_EN) |     \
                                     PIN_ODR_LOW(GPIOH_LCD_RST) |           \
                                     PIN_ODR_HIGH(GPIOH_STMOD_9) |          \
                                     PIN_ODR_LOW(GPIOH_LCD_R2) |            \
                                     PIN_ODR_LOW(GPIOH_LCD_R3) |            \
                                     PIN_ODR_LOW(GPIOH_LCD_R4) |            \
                                     PIN_ODR_LOW(GPIOH_LCD_R5) |            \
                                     PIN_ODR_HIGH(GPIOH_STMOD_11) |         \
                                     PIN_ODR_LOW(GPIOH_FDCAN1_TX) |         \
                                     PIN_ODR_LOW(GPIOH_FDCAN1_RX) |         \
                                     PIN_ODR_LOW(GPIOH_LCD_G4))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_STMOD_12, 0U) |      \
                                     PIN_AFIO_AF(GPIOH_ARD_A1, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_LCD_R1, 14U) |       \
                                     PIN_AFIO_AF(GPIOH_LCD_G5, 14U) |       \
                                     PIN_AFIO_AF(GPIOH_USB_FS_PWR_EN, 0U) | \
                                     PIN_AFIO_AF(GPIOH_LCD_RST, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_STMOD_9, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_LCD_R2, 14U) |       \
                                     PIN_AFIO_AF(GPIOH_LCD_R3, 14U) |       \
                                     PIN_AFIO_AF(GPIOH_LCD_R4, 14U) |       \
                                     PIN_AFIO_AF(GPIOH_LCD_R5, 14U) |       \
                                     PIN_AFIO_AF(GPIOH_STMOD_11, 0U) |      \
                                     PIN_AFIO_AF(GPIOH_FDCAN1_TX, 9U) |     \
                                     PIN_AFIO_AF(GPIOH_FDCAN1_RX, 9U) |     \
                                     PIN_AFIO_AF(GPIOH_LCD_G4, 14U))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input pullup).
 * PI1  - PIN1                      (input pullup).
 * PI2  - PIN2                      (input pullup).
 * PI3  - PIN3                      (input pullup).
 * PI4  - PIN4                      (input pullup).
 * PI5  - PIN5                      (input pullup).
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
                                     PIN_MODE_INPUT(GPIOI_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |           \
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
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |       \
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
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN5) |       \
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
                                     PIN_PUPDR_PULLUP(GPIOI_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN5) |         \
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
                                     PIN_ODR_HIGH(GPIOI_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |             \
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
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U) |          \
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
 * PJ2  - PIN2                      (input pullup).
 * PJ3  - PIN3                      (input pullup).
 * PJ4  - PIN4                      (input pullup).
 * PJ5  - PIN5                      (input pullup).
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
                                     PIN_MODE_INPUT(GPIOJ_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN5) |           \
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
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN5) |       \
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
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN5) |       \
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
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN5) |         \
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
                                     PIN_ODR_HIGH(GPIOJ_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN5) |             \
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
                                     PIN_AFIO_AF(GPIOJ_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN5, 0U) |          \
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
 * PK3  - PIN3                      (input pullup).
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
                                     PIN_MODE_INPUT(GPIOK_PIN3) |           \
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
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN3) |       \
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
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN3) |       \
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
                                     PIN_PUPDR_PULLUP(GPIOK_PIN3) |         \
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
                                     PIN_ODR_HIGH(GPIOK_PIN3) |             \
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
                                     PIN_AFIO_AF(GPIOK_PIN3, 0U) |          \
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
