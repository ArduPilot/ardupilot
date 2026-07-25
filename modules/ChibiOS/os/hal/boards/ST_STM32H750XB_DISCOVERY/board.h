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
 * Setup for STMicroelectronics STM32H750XB_DISCOVERY board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32H750XB_DISCOVERY
#define BOARD_NAME                  "STMicroelectronics STM32H750XB_DISCOVERY"

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID                MII_LAN8740A_ID

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
#undef STM32H750xx
#define STM32H750xx

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0U
#define GPIOA_MII_RX_CLK            1U
#define GPIOA_MII_MDIO              2U
#define GPIOA_PIN3                  3U
#define GPIOA_PIN4                  4U
#define GPIOA_PIN5                  5U
#define GPIOA_PIN6                  6U
#define GPIOA_MII_RX_DV             7U
#define GPIOA_PIN8                  8U
#define GPIOA_VBUS_FS2              9U
#define GPIOA_USB_OTG_FS2_ID        10U
#define GPIOA_OTG_FS_DM             11U
#define GPIOA_OTG_FS_DP             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_T_JTDI                15U

#define GPIOB_MII_RXD2              0U
#define GPIOB_MII_RXD3              1U
#define GPIOB_MII_TX_ER_NINT        2U
#define GPIOB_SWO                   3U
#define GPIOB_PIN4                  4U
#define GPIOB_FDCAN2_RX             5U
#define GPIOB_PIN6                  6U
#define GPIOB_PIN7                  7U
#define GPIOB_SDIO1_D4              8U
#define GPIOB_SDIO1_D5              9U
#define GPIOB_VCP_TX                10U
#define GPIOB_VCP_RX                11U
#define GPIOB_LCD_RST               12U
#define GPIOB_FDCAN2_TX             13U
#define GPIOB_PIN14                 14U
#define GPIOB_PIN15                 15U

#define GPIOC_PIN0                  0U
#define GPIOC_MII_MDC               1U
#define GPIOC_MII_TXD2              2U
#define GPIOC_MII_TX_CLK            3U
#define GPIOC_MII_RXD0              4U
#define GPIOC_MII_RXD1              5U
#define GPIOC_SDIO1_D6              6U
#define GPIOC_SDIO1_D7              7U
#define GPIOC_SDIO1_D0              8U
#define GPIOC_SDIO1_D1              9U
#define GPIOC_SDIO1_D2              10U
#define GPIOC_SDIO1_D3              11U
#define GPIOC_SDIO1_CK              12U
#define GPIOC_BUTTON                13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_SDRAM_D2              0U
#define GPIOD_SDRAM_D3              1U
#define GPIOD_SDIO1_CMD             2U
#define GPIOD_PIN3                  3U
#define GPIOD_PIN4                  4U
#define GPIOD_PIN5                  5U
#define GPIOD_PIN6                  6U
#define GPIOD_LCD_DISP              7U
#define GPIOD_SDRAM_D13             8U
#define GPIOD_SDRAM_D14             9U
#define GPIOD_SDRAM_D15             10U
#define GPIOD_QSPI_BK1_IO0          11U
#define GPIOD_I2C4_SCL              12U
#define GPIOD_I2C4_SDA              13U
#define GPIOD_SDRAM_D0              14U
#define GPIOD_SDRAM_D1              15U

#define GPIOE_SDRAM_NBL0            0U
#define GPIOE_SDRAM_NBL1            1U
#define GPIOE_MII_TXD3              2U
#define GPIOE_PIN3                  3U
#define GPIOE_SAI4_D2               4U
#define GPIOE_SAI4_CK2              5U
#define GPIOE_PIN6                  6U
#define GPIOE_SDRAM_D4              7U
#define GPIOE_SDRAM_D5              8U
#define GPIOE_SDRAM_D6              9U
#define GPIOE_SDRAM_D7              10U
#define GPIOE_SDRAM_D8              11U
#define GPIOE_SDRAM_D9              12U
#define GPIOE_SDRAM_D10             13U
#define GPIOE_SDRAM_D11             14U
#define GPIOE_SDRAM_D12             15U

#define GPIOF_SDRAM_A0              0U
#define GPIOF_SDRAM_A1              1U
#define GPIOF_SDRAM_A2              2U
#define GPIOF_SDRAM_A3              3U
#define GPIOF_SDRAM_A4              4U
#define GPIOF_SDRAM_A5              5U
#define GPIOF_QSPI_BK1_IO3          6U
#define GPIOF_QSPI_BK1_IO2          7U
#define GPIOF_PIN8                  8U
#define GPIOF_QSPI_BK1_IO1          9U
#define GPIOF_QSPI_CLK              10U
#define GPIOF_SDRAM_SDNRAS          11U
#define GPIOF_SDRAM_A6              12U
#define GPIOF_SDRAM_A7              13U
#define GPIOF_SDRAM_A8              14U
#define GPIOF_SDRAM_A9              15U

#define GPIOG_SDRAM_A10             0U
#define GPIOG_SDRAM_A11             1U
#define GPIOG_LCD_INT               2U
#define GPIOG_PIN3                  3U
#define GPIOG_SDRAM_BA0             4U
#define GPIOG_SDRAM_BA1             5U
#define GPIOG_QSPI_BK1_NCS          6U
#define GPIOG_PIN7                  7U
#define GPIOG_SDRAM_SDCLK           8U
#define GPIOG_QSPI_BK2_IO2          9U
#define GPIOG_SAI2_SDB              10U
#define GPIOG_MII_TX_EN             11U
#define GPIOG_MII_TXD1              12U
#define GPIOG_MII_TXD0              13U
#define GPIOG_QSPI_BK2_IO3          14U
#define GPIOG_SDRAM_SDNCAS          15U

#define GPIOH_OSC_IN                0U
#define GPIOH_PIN1                  1U
#define GPIOH_QSPI_BK2_IO0          2U
#define GPIOH_QSPI_BK2_IO1          3U
#define GPIOH_PIN4                  4U
#define GPIOH_SDRAM_SDNWE           5U
#define GPIOH_SDRAM_SDNE1           6U
#define GPIOH_SDRAM_SDCKE1          7U
#define GPIOH_PIN8                  8U
#define GPIOH_LCD_R3                9U
#define GPIOH_PIN10                 10U
#define GPIOH_OTG_FS2_OVER_CURRENT  11U
#define GPIOH_PIN12                 12U
#define GPIOH_FDCAN1_TX             13U
#define GPIOH_FDCAN1_RX             14U
#define GPIOH_PIN15                 15U

#define GPIOI_LCD_G5                0U
#define GPIOI_LCD_G6                1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_SAI2_MCLKA            4U
#define GPIOI_SAI2_SCKA             5U
#define GPIOI_SAI2_SDA              6U
#define GPIOI_SAI2_FSA              7U
#define GPIOI_PIN8                  8U
#define GPIOI_LCD_VSYNC             9U
#define GPIOI_MII_RX_ER             10U
#define GPIOI_PIN11                 11U
#define GPIOI_LCD_HSYNC             12U
#define GPIOI_LED_RED               13U
#define GPIOI_LCD_CLK               14U
#define GPIOI_LCD_R0                15U

#define GPIOJ_LCD_R1                0U
#define GPIOJ_LCD_R2                1U
#define GPIOJ_LED_GREEN             2U
#define GPIOJ_LCD_R4                3U
#define GPIOJ_LCD_R5                4U
#define GPIOJ_LCD_R6                5U
#define GPIOJ_LCD_R7                6U
#define GPIOJ_LCD_G0                7U
#define GPIOJ_LCD_G1                8U
#define GPIOJ_LCD_G2                9U
#define GPIOJ_LCD_G3                10U
#define GPIOJ_LCD_G4                11U
#define GPIOJ_LCD_B0                12U
#define GPIOJ_LCD_B1                13U
#define GPIOJ_LCD_B2                14U
#define GPIOJ_LCD_B3                15U

#define GPIOK_LCD_BL                0U
#define GPIOK_PIN1                  1U
#define GPIOK_LCD_G7                2U
#define GPIOK_LCD_B4                3U
#define GPIOK_LCD_B5                4U
#define GPIOK_LCD_B6                5U
#define GPIOK_LCD_B7                6U
#define GPIOK_LCD_DE                7U
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
#define LINE_MII_RX_CLK             PAL_LINE(GPIOA, 1U)
#define LINE_MII_MDIO               PAL_LINE(GPIOA, 2U)
#define LINE_MII_RX_DV              PAL_LINE(GPIOA, 7U)
#define LINE_VBUS_FS2               PAL_LINE(GPIOA, 9U)
#define LINE_USB_OTG_FS2_ID         PAL_LINE(GPIOA, 10U)
#define LINE_OTG_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP              PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_T_JTDI                 PAL_LINE(GPIOA, 15U)
#define LINE_MII_RXD2               PAL_LINE(GPIOB, 0U)
#define LINE_MII_RXD3               PAL_LINE(GPIOB, 1U)
#define LINE_MII_TX_ER_NINT         PAL_LINE(GPIOB, 2U)
#define LINE_SWO                    PAL_LINE(GPIOB, 3U)
#define LINE_FDCAN2_RX              PAL_LINE(GPIOB, 5U)
#define LINE_SDIO1_D4               PAL_LINE(GPIOB, 8U)
#define LINE_SDIO1_D5               PAL_LINE(GPIOB, 9U)
#define LINE_VCP_TX                 PAL_LINE(GPIOB, 10U)
#define LINE_VCP_RX                 PAL_LINE(GPIOB, 11U)
#define LINE_LCD_RST                PAL_LINE(GPIOB, 12U)
#define LINE_FDCAN2_TX              PAL_LINE(GPIOB, 13U)
#define LINE_MII_MDC                PAL_LINE(GPIOC, 1U)
#define LINE_MII_TXD2               PAL_LINE(GPIOC, 2U)
#define LINE_MII_TX_CLK             PAL_LINE(GPIOC, 3U)
#define LINE_MII_RXD0               PAL_LINE(GPIOC, 4U)
#define LINE_MII_RXD1               PAL_LINE(GPIOC, 5U)
#define LINE_SDIO1_D6               PAL_LINE(GPIOC, 6U)
#define LINE_SDIO1_D7               PAL_LINE(GPIOC, 7U)
#define LINE_SDIO1_D0               PAL_LINE(GPIOC, 8U)
#define LINE_SDIO1_D1               PAL_LINE(GPIOC, 9U)
#define LINE_SDIO1_D2               PAL_LINE(GPIOC, 10U)
#define LINE_SDIO1_D3               PAL_LINE(GPIOC, 11U)
#define LINE_SDIO1_CK               PAL_LINE(GPIOC, 12U)
#define LINE_BUTTON                 PAL_LINE(GPIOC, 13U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)
#define LINE_SDRAM_D2               PAL_LINE(GPIOD, 0U)
#define LINE_SDRAM_D3               PAL_LINE(GPIOD, 1U)
#define LINE_SDIO1_CMD              PAL_LINE(GPIOD, 2U)
#define LINE_LCD_DISP               PAL_LINE(GPIOD, 7U)
#define LINE_SDRAM_D13              PAL_LINE(GPIOD, 8U)
#define LINE_SDRAM_D14              PAL_LINE(GPIOD, 9U)
#define LINE_SDRAM_D15              PAL_LINE(GPIOD, 10U)
#define LINE_QSPI_BK1_IO0           PAL_LINE(GPIOD, 11U)
#define LINE_I2C4_SCL               PAL_LINE(GPIOD, 12U)
#define LINE_I2C4_SDA               PAL_LINE(GPIOD, 13U)
#define LINE_SDRAM_D0               PAL_LINE(GPIOD, 14U)
#define LINE_SDRAM_D1               PAL_LINE(GPIOD, 15U)
#define LINE_SDRAM_NBL0             PAL_LINE(GPIOE, 0U)
#define LINE_SDRAM_NBL1             PAL_LINE(GPIOE, 1U)
#define LINE_MII_TXD3               PAL_LINE(GPIOE, 2U)
#define LINE_SAI4_D2                PAL_LINE(GPIOE, 4U)
#define LINE_SAI4_CK2               PAL_LINE(GPIOE, 5U)
#define LINE_SDRAM_D4               PAL_LINE(GPIOE, 7U)
#define LINE_SDRAM_D5               PAL_LINE(GPIOE, 8U)
#define LINE_SDRAM_D6               PAL_LINE(GPIOE, 9U)
#define LINE_SDRAM_D7               PAL_LINE(GPIOE, 10U)
#define LINE_SDRAM_D8               PAL_LINE(GPIOE, 11U)
#define LINE_SDRAM_D9               PAL_LINE(GPIOE, 12U)
#define LINE_SDRAM_D10              PAL_LINE(GPIOE, 13U)
#define LINE_SDRAM_D11              PAL_LINE(GPIOE, 14U)
#define LINE_SDRAM_D12              PAL_LINE(GPIOE, 15U)
#define LINE_SDRAM_A0               PAL_LINE(GPIOF, 0U)
#define LINE_SDRAM_A1               PAL_LINE(GPIOF, 1U)
#define LINE_SDRAM_A2               PAL_LINE(GPIOF, 2U)
#define LINE_SDRAM_A3               PAL_LINE(GPIOF, 3U)
#define LINE_SDRAM_A4               PAL_LINE(GPIOF, 4U)
#define LINE_SDRAM_A5               PAL_LINE(GPIOF, 5U)
#define LINE_QSPI_BK1_IO3           PAL_LINE(GPIOF, 6U)
#define LINE_QSPI_BK1_IO2           PAL_LINE(GPIOF, 7U)
#define LINE_QSPI_BK1_IO1           PAL_LINE(GPIOF, 9U)
#define LINE_QSPI_CLK               PAL_LINE(GPIOF, 10U)
#define LINE_SDRAM_SDNRAS           PAL_LINE(GPIOF, 11U)
#define LINE_SDRAM_A6               PAL_LINE(GPIOF, 12U)
#define LINE_SDRAM_A7               PAL_LINE(GPIOF, 13U)
#define LINE_SDRAM_A8               PAL_LINE(GPIOF, 14U)
#define LINE_SDRAM_A9               PAL_LINE(GPIOF, 15U)
#define LINE_SDRAM_A10              PAL_LINE(GPIOG, 0U)
#define LINE_SDRAM_A11              PAL_LINE(GPIOG, 1U)
#define LINE_LCD_INT                PAL_LINE(GPIOG, 2U)
#define LINE_SDRAM_BA0              PAL_LINE(GPIOG, 4U)
#define LINE_SDRAM_BA1              PAL_LINE(GPIOG, 5U)
#define LINE_QSPI_BK1_NCS           PAL_LINE(GPIOG, 6U)
#define LINE_SDRAM_SDCLK            PAL_LINE(GPIOG, 8U)
#define LINE_QSPI_BK2_IO2           PAL_LINE(GPIOG, 9U)
#define LINE_SAI2_SDB               PAL_LINE(GPIOG, 10U)
#define LINE_MII_TX_EN              PAL_LINE(GPIOG, 11U)
#define LINE_MII_TXD1               PAL_LINE(GPIOG, 12U)
#define LINE_MII_TXD0               PAL_LINE(GPIOG, 13U)
#define LINE_QSPI_BK2_IO3           PAL_LINE(GPIOG, 14U)
#define LINE_SDRAM_SDNCAS           PAL_LINE(GPIOG, 15U)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_QSPI_BK2_IO0           PAL_LINE(GPIOH, 2U)
#define LINE_QSPI_BK2_IO1           PAL_LINE(GPIOH, 3U)
#define LINE_SDRAM_SDNWE            PAL_LINE(GPIOH, 5U)
#define LINE_SDRAM_SDNE1            PAL_LINE(GPIOH, 6U)
#define LINE_SDRAM_SDCKE1           PAL_LINE(GPIOH, 7U)
#define LINE_LCD_R3                 PAL_LINE(GPIOH, 9U)
#define LINE_OTG_FS2_OVER_CURRENT   PAL_LINE(GPIOH, 11U)
#define LINE_FDCAN1_TX              PAL_LINE(GPIOH, 13U)
#define LINE_FDCAN1_RX              PAL_LINE(GPIOH, 14U)
#define LINE_LCD_G5                 PAL_LINE(GPIOI, 0U)
#define LINE_LCD_G6                 PAL_LINE(GPIOI, 1U)
#define LINE_SAI2_MCLKA             PAL_LINE(GPIOI, 4U)
#define LINE_SAI2_SCKA              PAL_LINE(GPIOI, 5U)
#define LINE_SAI2_SDA               PAL_LINE(GPIOI, 6U)
#define LINE_SAI2_FSA               PAL_LINE(GPIOI, 7U)
#define LINE_LCD_VSYNC              PAL_LINE(GPIOI, 9U)
#define LINE_MII_RX_ER              PAL_LINE(GPIOI, 10U)
#define LINE_LCD_HSYNC              PAL_LINE(GPIOI, 12U)
#define LINE_LED_RED                PAL_LINE(GPIOI, 13U)
#define LINE_LCD_CLK                PAL_LINE(GPIOI, 14U)
#define LINE_LCD_R0                 PAL_LINE(GPIOI, 15U)
#define LINE_LCD_R1                 PAL_LINE(GPIOJ, 0U)
#define LINE_LCD_R2                 PAL_LINE(GPIOJ, 1U)
#define LINE_LED_GREEN              PAL_LINE(GPIOJ, 2U)
#define LINE_LCD_R4                 PAL_LINE(GPIOJ, 3U)
#define LINE_LCD_R5                 PAL_LINE(GPIOJ, 4U)
#define LINE_LCD_R6                 PAL_LINE(GPIOJ, 5U)
#define LINE_LCD_R7                 PAL_LINE(GPIOJ, 6U)
#define LINE_LCD_G0                 PAL_LINE(GPIOJ, 7U)
#define LINE_LCD_G1                 PAL_LINE(GPIOJ, 8U)
#define LINE_LCD_G2                 PAL_LINE(GPIOJ, 9U)
#define LINE_LCD_G3                 PAL_LINE(GPIOJ, 10U)
#define LINE_LCD_G4                 PAL_LINE(GPIOJ, 11U)
#define LINE_LCD_B0                 PAL_LINE(GPIOJ, 12U)
#define LINE_LCD_B1                 PAL_LINE(GPIOJ, 13U)
#define LINE_LCD_B2                 PAL_LINE(GPIOJ, 14U)
#define LINE_LCD_B3                 PAL_LINE(GPIOJ, 15U)
#define LINE_LCD_BL                 PAL_LINE(GPIOK, 0U)
#define LINE_LCD_G7                 PAL_LINE(GPIOK, 2U)
#define LINE_LCD_B4                 PAL_LINE(GPIOK, 3U)
#define LINE_LCD_B5                 PAL_LINE(GPIOK, 4U)
#define LINE_LCD_B6                 PAL_LINE(GPIOK, 5U)
#define LINE_LCD_B7                 PAL_LINE(GPIOK, 6U)
#define LINE_LCD_DE                 PAL_LINE(GPIOK, 7U)

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
 * PA0  - PIN0                      (input floating).
 * PA1  - MII_RX_CLK                (alternate 11).
 * PA2  - MII_MDIO                  (alternate 11).
 * PA3  - PIN3                      (input floating).
 * PA4  - PIN4                      (input floating).
 * PA5  - PIN5                      (input floating).
 * PA6  - PIN6                      (input floating).
 * PA7  - MII_RX_DV                 (alternate 11).
 * PA8  - PIN8                      (input floating).
 * PA9  - VBUS_FS2                  (analog).
 * PA10 - USB_OTG_FS2_ID            (alternate 10).
 * PA11 - OTG_FS_DM                 (alternate 10).
 * PA12 - OTG_FS_DP                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - T_JTDI                    (alternate 0).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) |           \
                                     PIN_MODE_ALTERNATE(GPIOA_MII_RX_CLK) | \
                                     PIN_MODE_ALTERNATE(GPIOA_MII_MDIO) |   \
                                     PIN_MODE_INPUT(GPIOA_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN6) |           \
                                     PIN_MODE_ALTERNATE(GPIOA_MII_RX_DV) |  \
                                     PIN_MODE_INPUT(GPIOA_PIN8) |           \
                                     PIN_MODE_ANALOG(GPIOA_VBUS_FS2) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_OTG_FS2_ID) |\
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_T_JTDI))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MII_RX_CLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MII_MDIO) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MII_RX_DV) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_VBUS_FS2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_OTG_FS2_ID) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_T_JTDI))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_PIN0) |       \
                                     PIN_OSPEED_HIGH(GPIOA_MII_RX_CLK) |    \
                                     PIN_OSPEED_HIGH(GPIOA_MII_MDIO) |      \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PIN6) |       \
                                     PIN_OSPEED_HIGH(GPIOA_MII_RX_DV) |     \
                                     PIN_OSPEED_HIGH(GPIOA_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOA_VBUS_FS2) |      \
                                     PIN_OSPEED_HIGH(GPIOA_USB_OTG_FS2_ID) |\
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DM) |     \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DP) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_T_JTDI))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_MII_RX_CLK) | \
                                     PIN_PUPDR_PULLUP(GPIOA_MII_MDIO) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN6) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_MII_RX_DV) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_VBUS_FS2) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_OTG_FS2_ID) |\
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_T_JTDI))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOA_MII_RX_CLK) |       \
                                     PIN_ODR_HIGH(GPIOA_MII_MDIO) |         \
                                     PIN_ODR_HIGH(GPIOA_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOA_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOA_MII_RX_DV) |        \
                                     PIN_ODR_HIGH(GPIOA_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOA_VBUS_FS2) |         \
                                     PIN_ODR_HIGH(GPIOA_USB_OTG_FS2_ID) |   \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |        \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |        \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_T_JTDI))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_MII_RX_CLK, 11U) |   \
                                     PIN_AFIO_AF(GPIOA_MII_MDIO, 11U) |     \
                                     PIN_AFIO_AF(GPIOA_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_MII_RX_DV, 11U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_VBUS_FS2, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_USB_OTG_FS2_ID, 10U) |\
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_T_JTDI, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - MII_RXD2                  (alternate 11).
 * PB1  - MII_RXD3                  (alternate 11).
 * PB2  - MII_TX_ER_NINT            (alternate 11).
 * PB3  - SWO                       (alternate 0).
 * PB4  - PIN4                      (input floating).
 * PB5  - FDCAN2_RX                 (alternate 9).
 * PB6  - PIN6                      (input floating).
 * PB7  - PIN7                      (input floating).
 * PB8  - SDIO1_D4                  (alternate 12).
 * PB9  - SDIO1_D5                  (alternate 12).
 * PB10 - VCP_TX                    (alternate 7).
 * PB11 - VCP_RX                    (alternate 7).
 * PB12 - LCD_RST                   (output pushpull maximum).
 * PB13 - FDCAN2_TX                 (alternate 9).
 * PB14 - PIN14                     (input floating).
 * PB15 - PIN15                     (input floating).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(GPIOB_MII_RXD2) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_MII_RXD3) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_MII_TX_ER_NINT) |\
                                     PIN_MODE_ALTERNATE(GPIOB_SWO) |        \
                                     PIN_MODE_INPUT(GPIOB_PIN4) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_FDCAN2_RX) |  \
                                     PIN_MODE_INPUT(GPIOB_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN7) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_SDIO1_D4) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_SDIO1_D5) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_VCP_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_VCP_RX) |     \
                                     PIN_MODE_OUTPUT(GPIOB_LCD_RST) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_FDCAN2_TX) |  \
                                     PIN_MODE_INPUT(GPIOB_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_MII_RXD2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MII_RXD3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MII_TX_ER_NINT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_FDCAN2_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SDIO1_D4) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SDIO1_D5) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_VCP_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_VCP_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_RST) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_FDCAN2_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_MII_RXD2) |      \
                                     PIN_OSPEED_HIGH(GPIOB_MII_RXD3) |      \
                                     PIN_OSPEED_HIGH(GPIOB_MII_TX_ER_NINT) |\
                                     PIN_OSPEED_HIGH(GPIOB_SWO) |           \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN4) |       \
                                     PIN_OSPEED_HIGH(GPIOB_FDCAN2_RX) |     \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN6) |       \
                                     PIN_OSPEED_HIGH(GPIOB_PIN7) |          \
                                     PIN_OSPEED_VERYLOW(GPIOB_SDIO1_D4) |   \
                                     PIN_OSPEED_VERYLOW(GPIOB_SDIO1_D5) |   \
                                     PIN_OSPEED_HIGH(GPIOB_VCP_TX) |        \
                                     PIN_OSPEED_HIGH(GPIOB_VCP_RX) |        \
                                     PIN_OSPEED_HIGH(GPIOB_LCD_RST) |       \
                                     PIN_OSPEED_HIGH(GPIOB_FDCAN2_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOB_PIN14) |         \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_MII_RXD2) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_MII_RXD3) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_MII_TX_ER_NINT) |\
                                     PIN_PUPDR_PULLUP(GPIOB_SWO) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_FDCAN2_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_SDIO1_D4) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_SDIO1_D5) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_VCP_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_VCP_RX) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOB_LCD_RST) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_FDCAN2_TX) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_MII_RXD2) |         \
                                     PIN_ODR_HIGH(GPIOB_MII_RXD3) |         \
                                     PIN_ODR_HIGH(GPIOB_MII_TX_ER_NINT) |   \
                                     PIN_ODR_HIGH(GPIOB_SWO) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOB_FDCAN2_RX) |        \
                                     PIN_ODR_HIGH(GPIOB_PIN6) |             \
                                     PIN_ODR_LOW(GPIOB_PIN7) |              \
                                     PIN_ODR_HIGH(GPIOB_SDIO1_D4) |         \
                                     PIN_ODR_HIGH(GPIOB_SDIO1_D5) |         \
                                     PIN_ODR_HIGH(GPIOB_VCP_TX) |           \
                                     PIN_ODR_HIGH(GPIOB_VCP_RX) |           \
                                     PIN_ODR_LOW(GPIOB_LCD_RST) |           \
                                     PIN_ODR_HIGH(GPIOB_FDCAN2_TX) |        \
                                     PIN_ODR_LOW(GPIOB_PIN14) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_MII_RXD2, 11U) |     \
                                     PIN_AFIO_AF(GPIOB_MII_RXD3, 11U) |     \
                                     PIN_AFIO_AF(GPIOB_MII_TX_ER_NINT, 11U) |\
                                     PIN_AFIO_AF(GPIOB_SWO, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_FDCAN2_RX, 9U) |     \
                                     PIN_AFIO_AF(GPIOB_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN7, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_SDIO1_D4, 12U) |     \
                                     PIN_AFIO_AF(GPIOB_SDIO1_D5, 12U) |     \
                                     PIN_AFIO_AF(GPIOB_VCP_TX, 7U) |        \
                                     PIN_AFIO_AF(GPIOB_VCP_RX, 7U) |        \
                                     PIN_AFIO_AF(GPIOB_LCD_RST, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_FDCAN2_TX, 9U) |     \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0                      (input floating).
 * PC1  - MII_MDC                   (alternate 11).
 * PC2  - MII_TXD2                  (alternate 11).
 * PC3  - MII_TX_CLK                (alternate 11).
 * PC4  - MII_RXD0                  (alternate 11).
 * PC5  - MII_RXD1                  (alternate 11).
 * PC6  - SDIO1_D6                  (alternate 12).
 * PC7  - SDIO1_D7                  (alternate 12).
 * PC8  - SDIO1_D0                  (alternate 12).
 * PC9  - SDIO1_D1                  (alternate 12).
 * PC10 - SDIO1_D2                  (alternate 12).
 * PC11 - SDIO1_D3                  (alternate 12).
 * PC12 - SDIO1_CK                  (alternate 12).
 * PC13 - BUTTON                    (input floating).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (output pushpull minimum).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0) |           \
                                     PIN_MODE_ALTERNATE(GPIOC_MII_MDC) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_MII_TXD2) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_MII_TX_CLK) | \
                                     PIN_MODE_ALTERNATE(GPIOC_MII_RXD0) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_MII_RXD1) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D6) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D7) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D0) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D1) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D2) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_D3) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO1_CK) |   \
                                     PIN_MODE_INPUT(GPIOC_BUTTON) |         \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_OUTPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MII_MDC) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MII_TXD2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MII_TX_CLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MII_RXD0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MII_RXD1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D6) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D7) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_D3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO1_CK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_BUTTON) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_PIN0) |       \
                                     PIN_OSPEED_HIGH(GPIOC_MII_MDC) |       \
                                     PIN_OSPEED_HIGH(GPIOC_MII_TXD2) |      \
                                     PIN_OSPEED_HIGH(GPIOC_MII_TX_CLK) |    \
                                     PIN_OSPEED_HIGH(GPIOC_MII_RXD0) |      \
                                     PIN_OSPEED_HIGH(GPIOC_MII_RXD1) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D6) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D7) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D0) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D1) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D2) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_D3) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO1_CK) |      \
                                     PIN_OSPEED_HIGH(GPIOC_BUTTON) |        \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_IN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_MII_MDC) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_MII_TXD2) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_MII_TX_CLK) | \
                                     PIN_PUPDR_FLOATING(GPIOC_MII_RXD0) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_MII_RXD1) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D6) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D7) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D0) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D1) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D2) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_D3) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SDIO1_CK) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_BUTTON) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOC_MII_MDC) |          \
                                     PIN_ODR_HIGH(GPIOC_MII_TXD2) |         \
                                     PIN_ODR_HIGH(GPIOC_MII_TX_CLK) |       \
                                     PIN_ODR_HIGH(GPIOC_MII_RXD0) |         \
                                     PIN_ODR_HIGH(GPIOC_MII_RXD1) |         \
                                     PIN_ODR_HIGH(GPIOC_SDIO1_D6) |         \
                                     PIN_ODR_HIGH(GPIOC_SDIO1_D7) |         \
                                     PIN_ODR_HIGH(GPIOC_SDIO1_D0) |         \
                                     PIN_ODR_HIGH(GPIOC_SDIO1_D1) |         \
                                     PIN_ODR_HIGH(GPIOC_SDIO1_D2) |         \
                                     PIN_ODR_HIGH(GPIOC_SDIO1_D3) |         \
                                     PIN_ODR_HIGH(GPIOC_SDIO1_CK) |         \
                                     PIN_ODR_HIGH(GPIOC_BUTTON) |           \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_MII_MDC, 11U) |      \
                                     PIN_AFIO_AF(GPIOC_MII_TXD2, 11U) |     \
                                     PIN_AFIO_AF(GPIOC_MII_TX_CLK, 11U) |   \
                                     PIN_AFIO_AF(GPIOC_MII_RXD0, 11U) |     \
                                     PIN_AFIO_AF(GPIOC_MII_RXD1, 11U) |     \
                                     PIN_AFIO_AF(GPIOC_SDIO1_D6, 12U) |     \
                                     PIN_AFIO_AF(GPIOC_SDIO1_D7, 12U))
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
 * PD0  - SDRAM_D2                  (alternate 12).
 * PD1  - SDRAM_D3                  (alternate 12).
 * PD2  - SDIO1_CMD                 (alternate 12).
 * PD3  - PIN3                      (input floating).
 * PD4  - PIN4                      (input floating).
 * PD5  - PIN5                      (input floating).
 * PD6  - PIN6                      (input floating).
 * PD7  - LCD_DISP                  (output pushpull maximum).
 * PD8  - SDRAM_D13                 (alternate 12).
 * PD9  - SDRAM_D14                 (alternate 12).
 * PD10 - SDRAM_D15                 (alternate 12).
 * PD11 - QSPI_BK1_IO0              (alternate 9).
 * PD12 - I2C4_SCL                  (alternate 4).
 * PD13 - I2C4_SDA                  (alternate 4).
 * PD14 - SDRAM_D0                  (alternate 12).
 * PD15 - SDRAM_D1                  (alternate 12).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_SDRAM_D2) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_SDRAM_D3) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_SDIO1_CMD) |  \
                                     PIN_MODE_INPUT(GPIOD_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN6) |           \
                                     PIN_MODE_OUTPUT(GPIOD_LCD_DISP) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_SDRAM_D13) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_SDRAM_D14) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_SDRAM_D15) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_QSPI_BK1_IO0) |\
                                     PIN_MODE_ALTERNATE(GPIOD_I2C4_SCL) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_I2C4_SDA) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_SDRAM_D0) |   \
                                     PIN_MODE_ALTERNATE(GPIOD_SDRAM_D1))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_SDRAM_D2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDRAM_D3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDIO1_CMD) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LCD_DISP) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDRAM_D13) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDRAM_D14) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDRAM_D15) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_QSPI_BK1_IO0) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_I2C4_SCL) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_I2C4_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDRAM_D0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDRAM_D1))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_SDRAM_D2) |      \
                                     PIN_OSPEED_HIGH(GPIOD_SDRAM_D3) |      \
                                     PIN_OSPEED_HIGH(GPIOD_SDIO1_CMD) |     \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN6) |       \
                                     PIN_OSPEED_HIGH(GPIOD_LCD_DISP) |      \
                                     PIN_OSPEED_HIGH(GPIOD_SDRAM_D13) |     \
                                     PIN_OSPEED_HIGH(GPIOD_SDRAM_D14) |     \
                                     PIN_OSPEED_HIGH(GPIOD_SDRAM_D15) |     \
                                     PIN_OSPEED_HIGH(GPIOD_QSPI_BK1_IO0) |  \
                                     PIN_OSPEED_HIGH(GPIOD_I2C4_SCL) |      \
                                     PIN_OSPEED_HIGH(GPIOD_I2C4_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOD_SDRAM_D0) |      \
                                     PIN_OSPEED_HIGH(GPIOD_SDRAM_D1))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_SDRAM_D2) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_SDRAM_D3) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_SDIO1_CMD) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6) |       \
                                     PIN_PUPDR_PULLUP(GPIOD_LCD_DISP) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_SDRAM_D13) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_SDRAM_D14) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_SDRAM_D15) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_QSPI_BK1_IO0) |\
                                     PIN_PUPDR_FLOATING(GPIOD_I2C4_SCL) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_I2C4_SDA) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_SDRAM_D0) |   \
                                     PIN_PUPDR_FLOATING(GPIOD_SDRAM_D1))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_SDRAM_D2) |         \
                                     PIN_ODR_HIGH(GPIOD_SDRAM_D3) |         \
                                     PIN_ODR_HIGH(GPIOD_SDIO1_CMD) |        \
                                     PIN_ODR_HIGH(GPIOD_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOD_LCD_DISP) |         \
                                     PIN_ODR_HIGH(GPIOD_SDRAM_D13) |        \
                                     PIN_ODR_HIGH(GPIOD_SDRAM_D14) |        \
                                     PIN_ODR_HIGH(GPIOD_SDRAM_D15) |        \
                                     PIN_ODR_HIGH(GPIOD_QSPI_BK1_IO0) |     \
                                     PIN_ODR_HIGH(GPIOD_I2C4_SCL) |         \
                                     PIN_ODR_HIGH(GPIOD_I2C4_SDA) |         \
                                     PIN_ODR_HIGH(GPIOD_SDRAM_D0) |         \
                                     PIN_ODR_HIGH(GPIOD_SDRAM_D1))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_SDRAM_D2, 12U) |     \
                                     PIN_AFIO_AF(GPIOD_SDRAM_D3, 12U) |     \
                                     PIN_AFIO_AF(GPIOD_SDIO1_CMD, 12U) |    \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_LCD_DISP, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_SDRAM_D13, 12U) |    \
                                     PIN_AFIO_AF(GPIOD_SDRAM_D14, 12U) |    \
                                     PIN_AFIO_AF(GPIOD_SDRAM_D15, 12U) |    \
                                     PIN_AFIO_AF(GPIOD_QSPI_BK1_IO0, 9U) |  \
                                     PIN_AFIO_AF(GPIOD_I2C4_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOD_I2C4_SDA, 4U) |      \
                                     PIN_AFIO_AF(GPIOD_SDRAM_D0, 12U) |     \
                                     PIN_AFIO_AF(GPIOD_SDRAM_D1, 12U))

/*
 * GPIOE setup:
 *
 * PE0  - SDRAM_NBL0                (alternate 12).
 * PE1  - SDRAM_NBL1                (alternate 12).
 * PE2  - MII_TXD3                  (alternate 11).
 * PE3  - PIN3                      (input floating).
 * PE4  - SAI4_D2                   (alternate 10).
 * PE5  - SAI4_CK2                  (alternate 10).
 * PE6  - PIN6                      (input floating).
 * PE7  - SDRAM_D4                  (alternate 12).
 * PE8  - SDRAM_D5                  (alternate 12).
 * PE9  - SDRAM_D6                  (alternate 12).
 * PE10 - SDRAM_D7                  (alternate 12).
 * PE11 - SDRAM_D8                  (alternate 12).
 * PE12 - SDRAM_D9                  (alternate 12).
 * PE13 - SDRAM_D10                 (alternate 12).
 * PE14 - SDRAM_D11                 (alternate 12).
 * PE15 - SDRAM_D12                 (alternate 12).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_SDRAM_NBL0) | \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_NBL1) | \
                                     PIN_MODE_ALTERNATE(GPIOE_MII_TXD3) |   \
                                     PIN_MODE_INPUT(GPIOE_PIN3) |           \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI4_D2) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI4_CK2) |   \
                                     PIN_MODE_INPUT(GPIOE_PIN6) |           \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_D4) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_D5) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_D6) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_D7) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_D8) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_D9) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_D10) |  \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_D11) |  \
                                     PIN_MODE_ALTERNATE(GPIOE_SDRAM_D12))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_NBL0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_NBL1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_MII_TXD3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI4_D2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI4_CK2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_D4) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_D5) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_D6) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_D7) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_D8) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_D9) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_D10) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_D11) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SDRAM_D12))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_SDRAM_NBL0) |    \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_NBL1) |    \
                                     PIN_OSPEED_HIGH(GPIOE_MII_TXD3) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN3) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SAI4_D2) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SAI4_CK2) |      \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN6) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_D4) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_D5) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_D6) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_D7) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_D8) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_D9) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_D10) |     \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_D11) |     \
                                     PIN_OSPEED_HIGH(GPIOE_SDRAM_D12))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_SDRAM_NBL0) | \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_NBL1) | \
                                     PIN_PUPDR_FLOATING(GPIOE_MII_TXD3) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI4_D2) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI4_CK2) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_D4) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_D5) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_D6) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_D7) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_D8) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_D9) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_D10) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_D11) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_SDRAM_D12))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_SDRAM_NBL0) |       \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_NBL1) |       \
                                     PIN_ODR_HIGH(GPIOE_MII_TXD3) |         \
                                     PIN_ODR_HIGH(GPIOE_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOE_SAI4_D2) |          \
                                     PIN_ODR_HIGH(GPIOE_SAI4_CK2) |         \
                                     PIN_ODR_HIGH(GPIOE_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_D4) |         \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_D5) |         \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_D6) |         \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_D7) |         \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_D8) |         \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_D9) |         \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_D10) |        \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_D11) |        \
                                     PIN_ODR_HIGH(GPIOE_SDRAM_D12))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_SDRAM_NBL0, 12U) |   \
                                     PIN_AFIO_AF(GPIOE_SDRAM_NBL1, 12U) |   \
                                     PIN_AFIO_AF(GPIOE_MII_TXD3, 11U) |     \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_SAI4_D2, 10U) |      \
                                     PIN_AFIO_AF(GPIOE_SAI4_CK2, 10U) |     \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_SDRAM_D4, 12U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_SDRAM_D5, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_SDRAM_D6, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_SDRAM_D7, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_SDRAM_D8, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_SDRAM_D9, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_SDRAM_D10, 12U) |    \
                                     PIN_AFIO_AF(GPIOE_SDRAM_D11, 12U) |    \
                                     PIN_AFIO_AF(GPIOE_SDRAM_D12, 12U))

/*
 * GPIOF setup:
 *
 * PF0  - SDRAM_A0                  (alternate 12).
 * PF1  - SDRAM_A1                  (alternate 12).
 * PF2  - SDRAM_A2                  (alternate 12).
 * PF3  - SDRAM_A3                  (alternate 12).
 * PF4  - SDRAM_A4                  (alternate 12).
 * PF5  - SDRAM_A5                  (alternate 12).
 * PF6  - QSPI_BK1_IO3              (alternate 9).
 * PF7  - QSPI_BK1_IO2              (alternate 9).
 * PF8  - PIN8                      (input floating).
 * PF9  - QSPI_BK1_IO1              (alternate 10).
 * PF10 - QSPI_CLK                  (alternate 9).
 * PF11 - SDRAM_SDNRAS              (alternate 12).
 * PF12 - SDRAM_A6                  (alternate 12).
 * PF13 - SDRAM_A7                  (alternate 12).
 * PF14 - SDRAM_A8                  (alternate 12).
 * PF15 - SDRAM_A9                  (alternate 12).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_SDRAM_A0) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_A1) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_A2) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_A3) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_A4) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_A5) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_QSPI_BK1_IO3) |\
                                     PIN_MODE_ALTERNATE(GPIOF_QSPI_BK1_IO2) |\
                                     PIN_MODE_INPUT(GPIOF_PIN8) |           \
                                     PIN_MODE_ALTERNATE(GPIOF_QSPI_BK1_IO1) |\
                                     PIN_MODE_ALTERNATE(GPIOF_QSPI_CLK) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_SDNRAS) |\
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_A6) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_A7) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_A8) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_SDRAM_A9))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A4) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A5) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_QSPI_BK1_IO3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_QSPI_BK1_IO2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_QSPI_BK1_IO1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_QSPI_CLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_SDNRAS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A6) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A7) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A8) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOF_SDRAM_A9))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_SDRAM_A0) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_A1) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_A2) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_A3) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_A4) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_A5) |      \
                                     PIN_OSPEED_HIGH(GPIOF_QSPI_BK1_IO3) |  \
                                     PIN_OSPEED_HIGH(GPIOF_QSPI_BK1_IO2) |  \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN8) |       \
                                     PIN_OSPEED_HIGH(GPIOF_QSPI_BK1_IO1) |  \
                                     PIN_OSPEED_HIGH(GPIOF_QSPI_CLK) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_SDNRAS) |  \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_A6) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_A7) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_A8) |      \
                                     PIN_OSPEED_HIGH(GPIOF_SDRAM_A9))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_SDRAM_A0) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_A1) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_A2) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_A3) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_A4) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_A5) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_QSPI_BK1_IO3) |\
                                     PIN_PUPDR_FLOATING(GPIOF_QSPI_BK1_IO2) |\
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_QSPI_BK1_IO1) |\
                                     PIN_PUPDR_FLOATING(GPIOF_QSPI_CLK) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_SDNRAS) |\
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_A6) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_A7) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_A8) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_SDRAM_A9))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_SDRAM_A0) |         \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_A1) |         \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_A2) |         \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_A3) |         \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_A4) |         \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_A5) |         \
                                     PIN_ODR_HIGH(GPIOF_QSPI_BK1_IO3) |     \
                                     PIN_ODR_HIGH(GPIOF_QSPI_BK1_IO2) |     \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_QSPI_BK1_IO1) |     \
                                     PIN_ODR_HIGH(GPIOF_QSPI_CLK) |         \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_SDNRAS) |     \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_A6) |         \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_A7) |         \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_A8) |         \
                                     PIN_ODR_HIGH(GPIOF_SDRAM_A9))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_SDRAM_A0, 12U) |     \
                                     PIN_AFIO_AF(GPIOF_SDRAM_A1, 12U) |     \
                                     PIN_AFIO_AF(GPIOF_SDRAM_A2, 12U) |     \
                                     PIN_AFIO_AF(GPIOF_SDRAM_A3, 12U) |     \
                                     PIN_AFIO_AF(GPIOF_SDRAM_A4, 12U) |     \
                                     PIN_AFIO_AF(GPIOF_SDRAM_A5, 12U) |     \
                                     PIN_AFIO_AF(GPIOF_QSPI_BK1_IO3, 9U) |  \
                                     PIN_AFIO_AF(GPIOF_QSPI_BK1_IO2, 9U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_QSPI_BK1_IO1, 10U) | \
                                     PIN_AFIO_AF(GPIOF_QSPI_CLK, 9U) |      \
                                     PIN_AFIO_AF(GPIOF_SDRAM_SDNRAS, 12U) | \
                                     PIN_AFIO_AF(GPIOF_SDRAM_A6, 12U) |     \
                                     PIN_AFIO_AF(GPIOF_SDRAM_A7, 12U) |     \
                                     PIN_AFIO_AF(GPIOF_SDRAM_A8, 12U) |     \
                                     PIN_AFIO_AF(GPIOF_SDRAM_A9, 12U))

/*
 * GPIOG setup:
 *
 * PG0  - SDRAM_A10                 (alternate 12).
 * PG1  - SDRAM_A11                 (alternate 12).
 * PG2  - LCD_INT                   (input floating).
 * PG3  - PIN3                      (input floating).
 * PG4  - SDRAM_BA0                 (alternate 12).
 * PG5  - SDRAM_BA1                 (alternate 12).
 * PG6  - QSPI_BK1_NCS              (alternate 10).
 * PG7  - PIN7                      (input floating).
 * PG8  - SDRAM_SDCLK               (alternate 12).
 * PG9  - QSPI_BK2_IO2              (alternate 9).
 * PG10 - SAI2_SDB                  (alternate 10).
 * PG11 - MII_TX_EN                 (alternate 11).
 * PG12 - MII_TXD1                  (alternate 11).
 * PG13 - MII_TXD0                  (alternate 11).
 * PG14 - QSPI_BK2_IO3              (alternate 9).
 * PG15 - SDRAM_SDNCAS              (alternate 12).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_ALTERNATE(GPIOG_SDRAM_A10) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_SDRAM_A11) |  \
                                     PIN_MODE_INPUT(GPIOG_LCD_INT) |        \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |           \
                                     PIN_MODE_ALTERNATE(GPIOG_SDRAM_BA0) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_SDRAM_BA1) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_QSPI_BK1_NCS) |\
                                     PIN_MODE_INPUT(GPIOG_PIN7) |           \
                                     PIN_MODE_ALTERNATE(GPIOG_SDRAM_SDCLK) |\
                                     PIN_MODE_ALTERNATE(GPIOG_QSPI_BK2_IO2) |\
                                     PIN_MODE_ALTERNATE(GPIOG_SAI2_SDB) |   \
                                     PIN_MODE_ALTERNATE(GPIOG_MII_TX_EN) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_MII_TXD1) |   \
                                     PIN_MODE_ALTERNATE(GPIOG_MII_TXD0) |   \
                                     PIN_MODE_ALTERNATE(GPIOG_QSPI_BK2_IO3) |\
                                     PIN_MODE_ALTERNATE(GPIOG_SDRAM_SDNCAS))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_SDRAM_A10) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SDRAM_A11) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_LCD_INT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SDRAM_BA0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SDRAM_BA1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_QSPI_BK1_NCS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SDRAM_SDCLK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_QSPI_BK2_IO2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_SAI2_SDB) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MII_TX_EN) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MII_TXD1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MII_TXD0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_QSPI_BK2_IO3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_SDRAM_SDNCAS))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_SDRAM_A10) |     \
                                     PIN_OSPEED_HIGH(GPIOG_SDRAM_A11) |     \
                                     PIN_OSPEED_HIGH(GPIOG_LCD_INT) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN3) |       \
                                     PIN_OSPEED_HIGH(GPIOG_SDRAM_BA0) |     \
                                     PIN_OSPEED_HIGH(GPIOG_SDRAM_BA1) |     \
                                     PIN_OSPEED_HIGH(GPIOG_QSPI_BK1_NCS) |  \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN7) |       \
                                     PIN_OSPEED_HIGH(GPIOG_SDRAM_SDCLK) |   \
                                     PIN_OSPEED_HIGH(GPIOG_QSPI_BK2_IO2) |  \
                                     PIN_OSPEED_HIGH(GPIOG_SAI2_SDB) |      \
                                     PIN_OSPEED_HIGH(GPIOG_MII_TX_EN) |     \
                                     PIN_OSPEED_HIGH(GPIOG_MII_TXD1) |      \
                                     PIN_OSPEED_HIGH(GPIOG_MII_TXD0) |      \
                                     PIN_OSPEED_HIGH(GPIOG_QSPI_BK2_IO3) |  \
                                     PIN_OSPEED_HIGH(GPIOG_SDRAM_SDNCAS))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_SDRAM_A10) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_SDRAM_A11) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_LCD_INT) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_SDRAM_BA0) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_SDRAM_BA1) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_QSPI_BK1_NCS) |\
                                     PIN_PUPDR_FLOATING(GPIOG_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_SDRAM_SDCLK) |\
                                     PIN_PUPDR_FLOATING(GPIOG_QSPI_BK2_IO2) |\
                                     PIN_PUPDR_FLOATING(GPIOG_SAI2_SDB) |   \
                                     PIN_PUPDR_FLOATING(GPIOG_MII_TX_EN) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_MII_TXD1) |   \
                                     PIN_PUPDR_FLOATING(GPIOG_MII_TXD0) |   \
                                     PIN_PUPDR_FLOATING(GPIOG_QSPI_BK2_IO3) |\
                                     PIN_PUPDR_FLOATING(GPIOG_SDRAM_SDNCAS))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_SDRAM_A10) |        \
                                     PIN_ODR_HIGH(GPIOG_SDRAM_A11) |        \
                                     PIN_ODR_HIGH(GPIOG_LCD_INT) |          \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOG_SDRAM_BA0) |        \
                                     PIN_ODR_HIGH(GPIOG_SDRAM_BA1) |        \
                                     PIN_ODR_HIGH(GPIOG_QSPI_BK1_NCS) |     \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOG_SDRAM_SDCLK) |      \
                                     PIN_ODR_HIGH(GPIOG_QSPI_BK2_IO2) |     \
                                     PIN_ODR_HIGH(GPIOG_SAI2_SDB) |         \
                                     PIN_ODR_HIGH(GPIOG_MII_TX_EN) |        \
                                     PIN_ODR_HIGH(GPIOG_MII_TXD1) |         \
                                     PIN_ODR_HIGH(GPIOG_MII_TXD0) |         \
                                     PIN_ODR_HIGH(GPIOG_QSPI_BK2_IO3) |     \
                                     PIN_ODR_HIGH(GPIOG_SDRAM_SDNCAS))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_SDRAM_A10, 12U) |    \
                                     PIN_AFIO_AF(GPIOG_SDRAM_A11, 12U) |    \
                                     PIN_AFIO_AF(GPIOG_LCD_INT, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_SDRAM_BA0, 12U) |    \
                                     PIN_AFIO_AF(GPIOG_SDRAM_BA1, 12U) |    \
                                     PIN_AFIO_AF(GPIOG_QSPI_BK1_NCS, 10U) | \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_SDRAM_SDCLK, 12U) |  \
                                     PIN_AFIO_AF(GPIOG_QSPI_BK2_IO2, 9U) |  \
                                     PIN_AFIO_AF(GPIOG_SAI2_SDB, 10U) |     \
                                     PIN_AFIO_AF(GPIOG_MII_TX_EN, 11U) |    \
                                     PIN_AFIO_AF(GPIOG_MII_TXD1, 11U) |     \
                                     PIN_AFIO_AF(GPIOG_MII_TXD0, 11U) |     \
                                     PIN_AFIO_AF(GPIOG_QSPI_BK2_IO3, 9U) |  \
                                     PIN_AFIO_AF(GPIOG_SDRAM_SDNCAS, 12U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - PIN1                      (input floating).
 * PH2  - QSPI_BK2_IO0              (alternate 9).
 * PH3  - QSPI_BK2_IO1              (alternate 9).
 * PH4  - PIN4                      (input floating).
 * PH5  - SDRAM_SDNWE               (alternate 12).
 * PH6  - SDRAM_SDNE1               (alternate 12).
 * PH7  - SDRAM_SDCKE1              (alternate 12).
 * PH8  - PIN8                      (input floating).
 * PH9  - LCD_R3                    (alternate 14).
 * PH10 - PIN10                     (input floating).
 * PH11 - OTG_FS2_OVER_CURRENT      (input floating).
 * PH12 - PIN12                     (input floating).
 * PH13 - FDCAN1_TX                 (alternate 9).
 * PH14 - FDCAN1_RX                 (alternate 9).
 * PH15 - PIN15                     (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_PIN1) |           \
                                     PIN_MODE_ALTERNATE(GPIOH_QSPI_BK2_IO0) |\
                                     PIN_MODE_ALTERNATE(GPIOH_QSPI_BK2_IO1) |\
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_ALTERNATE(GPIOH_SDRAM_SDNWE) |\
                                     PIN_MODE_ALTERNATE(GPIOH_SDRAM_SDNE1) |\
                                     PIN_MODE_ALTERNATE(GPIOH_SDRAM_SDCKE1) |\
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_R3) |     \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_OTG_FS2_OVER_CURRENT) |\
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_ALTERNATE(GPIOH_FDCAN1_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOH_FDCAN1_RX) |  \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_QSPI_BK2_IO0) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_QSPI_BK2_IO1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_SDRAM_SDNWE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_SDRAM_SDNE1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_SDRAM_SDCKE1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_LCD_R3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OTG_FS2_OVER_CURRENT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FDCAN1_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FDCAN1_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOH_QSPI_BK2_IO0) |  \
                                     PIN_OSPEED_HIGH(GPIOH_QSPI_BK2_IO1) |  \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN4) |       \
                                     PIN_OSPEED_HIGH(GPIOH_SDRAM_SDNWE) |   \
                                     PIN_OSPEED_HIGH(GPIOH_SDRAM_SDNE1) |   \
                                     PIN_OSPEED_HIGH(GPIOH_SDRAM_SDCKE1) |  \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN8) |       \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_R3) |        \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_OTG_FS2_OVER_CURRENT) |\
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN12) |      \
                                     PIN_OSPEED_HIGH(GPIOH_FDCAN1_TX) |     \
                                     PIN_OSPEED_HIGH(GPIOH_FDCAN1_RX) |     \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_QSPI_BK2_IO0) |\
                                     PIN_PUPDR_FLOATING(GPIOH_QSPI_BK2_IO1) |\
                                     PIN_PUPDR_FLOATING(GPIOH_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_SDRAM_SDNWE) |\
                                     PIN_PUPDR_FLOATING(GPIOH_SDRAM_SDNE1) |\
                                     PIN_PUPDR_FLOATING(GPIOH_SDRAM_SDCKE1) |\
                                     PIN_PUPDR_FLOATING(GPIOH_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_R3) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_OTG_FS2_OVER_CURRENT) |\
                                     PIN_PUPDR_FLOATING(GPIOH_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOH_FDCAN1_TX) |  \
                                     PIN_PUPDR_FLOATING(GPIOH_FDCAN1_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOH_QSPI_BK2_IO0) |     \
                                     PIN_ODR_HIGH(GPIOH_QSPI_BK2_IO1) |     \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_SDRAM_SDNWE) |      \
                                     PIN_ODR_HIGH(GPIOH_SDRAM_SDNE1) |      \
                                     PIN_ODR_HIGH(GPIOH_SDRAM_SDCKE1) |     \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_LCD_R3) |           \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_OTG_FS2_OVER_CURRENT) |\
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_FDCAN1_TX) |        \
                                     PIN_ODR_HIGH(GPIOH_FDCAN1_RX) |        \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_QSPI_BK2_IO0, 9U) |  \
                                     PIN_AFIO_AF(GPIOH_QSPI_BK2_IO1, 9U) |  \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_SDRAM_SDNWE, 12U) |  \
                                     PIN_AFIO_AF(GPIOH_SDRAM_SDNE1, 12U) |  \
                                     PIN_AFIO_AF(GPIOH_SDRAM_SDCKE1, 12U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_LCD_R3, 14U) |       \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_OTG_FS2_OVER_CURRENT, 0U) |\
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_FDCAN1_TX, 9U) |     \
                                     PIN_AFIO_AF(GPIOH_FDCAN1_RX, 9U) |     \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - LCD_G5                    (alternate 14).
 * PI1  - LCD_G6                    (alternate 14).
 * PI2  - PIN2                      (input floating).
 * PI3  - PIN3                      (input floating).
 * PI4  - SAI2_MCLKA                (alternate 10).
 * PI5  - SAI2_SCKA                 (alternate 10).
 * PI6  - SAI2_SDA                  (alternate 10).
 * PI7  - SAI2_FSA                  (alternate 10).
 * PI8  - PIN8                      (input floating).
 * PI9  - LCD_VSYNC                 (alternate 14).
 * PI10 - MII_RX_ER                 (alternate 11).
 * PI11 - PIN11                     (input floating).
 * PI12 - LCD_HSYNC                 (alternate 14).
 * PI13 - LED_RED                   (output pushpull maximum).
 * PI14 - LCD_CLK                   (alternate 14).
 * PI15 - LCD_R0                    (alternate 14).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_ALTERNATE(GPIOI_LCD_G5) |     \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_G6) |     \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |           \
                                     PIN_MODE_ALTERNATE(GPIOI_SAI2_MCLKA) | \
                                     PIN_MODE_ALTERNATE(GPIOI_SAI2_SCKA) |  \
                                     PIN_MODE_ALTERNATE(GPIOI_SAI2_SDA) |   \
                                     PIN_MODE_ALTERNATE(GPIOI_SAI2_FSA) |   \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_VSYNC) |  \
                                     PIN_MODE_ALTERNATE(GPIOI_MII_RX_ER) |  \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_HSYNC) |  \
                                     PIN_MODE_OUTPUT(GPIOI_LED_RED) |       \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_CLK) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_R0))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_LCD_G5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_G6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_SAI2_MCLKA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_SAI2_SCKA) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOI_SAI2_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_SAI2_FSA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_VSYNC) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOI_MII_RX_ER) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_HSYNC) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LED_RED) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_CLK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_R0))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_HIGH(GPIOI_LCD_G5) |        \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_G6) |        \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN3) |       \
                                     PIN_OSPEED_HIGH(GPIOI_SAI2_MCLKA) |    \
                                     PIN_OSPEED_HIGH(GPIOI_SAI2_SCKA) |     \
                                     PIN_OSPEED_HIGH(GPIOI_SAI2_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOI_SAI2_FSA) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN8) |       \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_VSYNC) |     \
                                     PIN_OSPEED_HIGH(GPIOI_MII_RX_ER) |     \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN11) |      \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_HSYNC) |     \
                                     PIN_OSPEED_HIGH(GPIOI_LED_RED) |       \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_CLK) |       \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_R0))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(GPIOI_LCD_G5) |     \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_G6) |     \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_SAI2_MCLKA) | \
                                     PIN_PUPDR_FLOATING(GPIOI_SAI2_SCKA) |  \
                                     PIN_PUPDR_FLOATING(GPIOI_SAI2_SDA) |   \
                                     PIN_PUPDR_FLOATING(GPIOI_SAI2_FSA) |   \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_VSYNC) |  \
                                     PIN_PUPDR_FLOATING(GPIOI_MII_RX_ER) |  \
                                     PIN_PUPDR_FLOATING(GPIOI_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_HSYNC) |  \
                                     PIN_PUPDR_PULLUP(GPIOI_LED_RED) |      \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_CLK) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_R0))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_LCD_G5) |           \
                                     PIN_ODR_HIGH(GPIOI_LCD_G6) |           \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOI_SAI2_MCLKA) |       \
                                     PIN_ODR_HIGH(GPIOI_SAI2_SCKA) |        \
                                     PIN_ODR_HIGH(GPIOI_SAI2_SDA) |         \
                                     PIN_ODR_HIGH(GPIOI_SAI2_FSA) |         \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_LCD_VSYNC) |        \
                                     PIN_ODR_HIGH(GPIOI_MII_RX_ER) |        \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOI_LCD_HSYNC) |        \
                                     PIN_ODR_HIGH(GPIOI_LED_RED) |          \
                                     PIN_ODR_HIGH(GPIOI_LCD_CLK) |          \
                                     PIN_ODR_HIGH(GPIOI_LCD_R0))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_LCD_G5, 14U) |       \
                                     PIN_AFIO_AF(GPIOI_LCD_G6, 14U) |       \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_SAI2_MCLKA, 10U) |   \
                                     PIN_AFIO_AF(GPIOI_SAI2_SCKA, 10U) |    \
                                     PIN_AFIO_AF(GPIOI_SAI2_SDA, 10U) |     \
                                     PIN_AFIO_AF(GPIOI_SAI2_FSA, 10U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_LCD_VSYNC, 14U) |    \
                                     PIN_AFIO_AF(GPIOI_MII_RX_ER, 11U) |    \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_LCD_HSYNC, 14U) |    \
                                     PIN_AFIO_AF(GPIOI_LED_RED, 0U) |       \
                                     PIN_AFIO_AF(GPIOI_LCD_CLK, 14U) |      \
                                     PIN_AFIO_AF(GPIOI_LCD_R0, 14U))

/*
 * GPIOJ setup:
 *
 * PJ0  - LCD_R1                    (alternate 14).
 * PJ1  - LCD_R2                    (alternate 14).
 * PJ2  - LED_GREEN                 (output pushpull maximum).
 * PJ3  - LCD_R4                    (alternate 14).
 * PJ4  - LCD_R5                    (alternate 14).
 * PJ5  - LCD_R6                    (alternate 14).
 * PJ6  - LCD_R7                    (alternate 14).
 * PJ7  - LCD_G0                    (alternate 14).
 * PJ8  - LCD_G1                    (alternate 14).
 * PJ9  - LCD_G2                    (alternate 14).
 * PJ10 - LCD_G3                    (alternate 14).
 * PJ11 - LCD_G4                    (alternate 14).
 * PJ12 - LCD_B0                    (alternate 14).
 * PJ13 - LCD_B1                    (alternate 14).
 * PJ14 - LCD_B2                    (alternate 14).
 * PJ15 - LCD_B3                    (alternate 14).
 */
#define VAL_GPIOJ_MODER             (PIN_MODE_ALTERNATE(GPIOJ_LCD_R1) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R2) |     \
                                     PIN_MODE_OUTPUT(GPIOJ_LED_GREEN) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R4) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R5) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R6) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R7) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G0) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G1) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G2) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G3) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G4) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_B0) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_B1) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_B2) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_B3))
#define VAL_GPIOJ_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LED_GREEN) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_B0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_B1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_B2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_B3))
#define VAL_GPIOJ_OSPEEDR           (PIN_OSPEED_HIGH(GPIOJ_LCD_R1) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_R2) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LED_GREEN) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_R4) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_R5) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_R6) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_R7) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_G0) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_G1) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_G2) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_G3) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_G4) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_B0) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_B1) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_B2) |        \
                                     PIN_OSPEED_HIGH(GPIOJ_LCD_B3))
#define VAL_GPIOJ_PUPDR             (PIN_PUPDR_FLOATING(GPIOJ_LCD_R1) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R2) |     \
                                     PIN_PUPDR_PULLUP(GPIOJ_LED_GREEN) |    \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R4) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R5) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R6) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R7) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_G0) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_G1) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_G2) |     \
                                     PIN_PUPDR_PULLUP(GPIOJ_LCD_G3) |       \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_G4) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_B0) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_B1) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_B2) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_B3))
#define VAL_GPIOJ_ODR               (PIN_ODR_HIGH(GPIOJ_LCD_R1) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R2) |           \
                                     PIN_ODR_HIGH(GPIOJ_LED_GREEN) |        \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R4) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R5) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R6) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R7) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G0) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G1) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G2) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G3) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G4) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_B0) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_B1) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_B2) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_B3))
#define VAL_GPIOJ_AFRL              (PIN_AFIO_AF(GPIOJ_LCD_R1, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R2, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LED_GREEN, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_LCD_R4, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R5, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R6, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R7, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_G0, 14U))
#define VAL_GPIOJ_AFRH              (PIN_AFIO_AF(GPIOJ_LCD_G1, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_G2, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_G3, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_G4, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_B0, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_B1, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_B2, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_B3, 14U))

/*
 * GPIOK setup:
 *
 * PK0  - LCD_BL                    (output pushpull minimum).
 * PK1  - PIN1                      (input floating).
 * PK2  - LCD_G7                    (alternate 14).
 * PK3  - LCD_B4                    (alternate 14).
 * PK4  - LCD_B5                    (alternate 14).
 * PK5  - LCD_B6                    (alternate 14).
 * PK6  - LCD_B7                    (alternate 14).
 * PK7  - LCD_DE                    (alternate 14).
 * PK8  - PIN8                      (input pullup).
 * PK9  - PIN9                      (input pullup).
 * PK10 - PIN10                     (input pullup).
 * PK11 - PIN11                     (input pullup).
 * PK12 - PIN12                     (input pullup).
 * PK13 - PIN13                     (input pullup).
 * PK14 - PIN14                     (input pullup).
 * PK15 - PIN15                     (input pullup).
 */
#define VAL_GPIOK_MODER             (PIN_MODE_OUTPUT(GPIOK_LCD_BL) |        \
                                     PIN_MODE_INPUT(GPIOK_PIN1) |           \
                                     PIN_MODE_ALTERNATE(GPIOK_LCD_G7) |     \
                                     PIN_MODE_ALTERNATE(GPIOK_LCD_B4) |     \
                                     PIN_MODE_ALTERNATE(GPIOK_LCD_B5) |     \
                                     PIN_MODE_ALTERNATE(GPIOK_LCD_B6) |     \
                                     PIN_MODE_ALTERNATE(GPIOK_LCD_B7) |     \
                                     PIN_MODE_ALTERNATE(GPIOK_LCD_DE) |     \
                                     PIN_MODE_INPUT(GPIOK_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN15))
#define VAL_GPIOK_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOK_LCD_BL) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LCD_G7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LCD_B4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LCD_B5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LCD_B6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LCD_B7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LCD_DE) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN15))
#define VAL_GPIOK_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOK_LCD_BL) |     \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN1) |       \
                                     PIN_OSPEED_HIGH(GPIOK_LCD_G7) |        \
                                     PIN_OSPEED_HIGH(GPIOK_LCD_B4) |        \
                                     PIN_OSPEED_HIGH(GPIOK_LCD_B5) |        \
                                     PIN_OSPEED_HIGH(GPIOK_LCD_B6) |        \
                                     PIN_OSPEED_HIGH(GPIOK_LCD_B7) |        \
                                     PIN_OSPEED_HIGH(GPIOK_LCD_DE) |        \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN15))
#define VAL_GPIOK_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOK_LCD_BL) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_G7) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_B4) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_B5) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_B6) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_B7) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_DE) |     \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN15))
#define VAL_GPIOK_ODR               (PIN_ODR_LOW(GPIOK_LCD_BL) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOK_LCD_G7) |           \
                                     PIN_ODR_HIGH(GPIOK_LCD_B4) |           \
                                     PIN_ODR_HIGH(GPIOK_LCD_B5) |           \
                                     PIN_ODR_HIGH(GPIOK_LCD_B6) |           \
                                     PIN_ODR_HIGH(GPIOK_LCD_B7) |           \
                                     PIN_ODR_HIGH(GPIOK_LCD_DE) |           \
                                     PIN_ODR_HIGH(GPIOK_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN15))
#define VAL_GPIOK_AFRL              (PIN_AFIO_AF(GPIOK_LCD_BL, 0U) |        \
                                     PIN_AFIO_AF(GPIOK_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_LCD_G7, 14U) |       \
                                     PIN_AFIO_AF(GPIOK_LCD_B4, 14U) |       \
                                     PIN_AFIO_AF(GPIOK_LCD_B5, 14U) |       \
                                     PIN_AFIO_AF(GPIOK_LCD_B6, 14U) |       \
                                     PIN_AFIO_AF(GPIOK_LCD_B7, 14U) |       \
                                     PIN_AFIO_AF(GPIOK_LCD_DE, 14U))
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
