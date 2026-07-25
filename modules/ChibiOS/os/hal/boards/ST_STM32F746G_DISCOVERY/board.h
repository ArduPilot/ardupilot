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
 * Setup for STMicroelectronics STM32F746G-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F746G_DISCOVERY
#define BOARD_NAME                  "STMicroelectronics STM32F746G-Discovery"

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID                MII_LAN8742A_ID
#define BOARD_PHY_RMII

/*
 * The board has an ULPI USB PHY.
 */
#define BOARD_OTG2_USES_ULPI

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
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#undef STM32F746xx
#define STM32F746xx

/*
 * IO pins assignments.
 */
#define GPIOA_ARD_A0                0U
#define GPIOA_RMII_REF_CLK          1U
#define GPIOA_RMII_MDIO             2U
#define GPIOA_ULPI_D0               3U
#define GPIOA_DCMI_HSYNC            4U
#define GPIOA_ULPI_CK               5U
#define GPIOA_DCMI_PIXCK            6U
#define GPIOA_RMII_CRS_DV           7U
#define GPIOA_ARD_D10               8U
#define GPIOA_VCP_TX                9U
#define GPIOA_OTG_FS_ID             10U
#define GPIOA_OTG_FS_DM             11U
#define GPIOA_OTG_FS_DP             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_ARD_D9                15U

#define GPIOB_ULPI_D1               0U
#define GPIOB_ULPI_D2               1U
#define GPIOB_QSPI_CLK              2U
#define GPIOB_SWO                   3U
#define GPIOB_ARD_D3                4U
#define GPIOB_ULPI_D7               5U
#define GPIOB_QSPI_NCS              6U
#define GPIOB_VCP_RX                7U
#define GPIOB_ARD_D15               8U
#define GPIOB_ARD_D14               9U
#define GPIOB_ULPI_D3               10U
#define GPIOB_ULPI_D4               11U
#define GPIOB_ULPI_D5               12U
#define GPIOB_ULPI_D6               13U
#define GPIOB_ARD_D12               14U
#define GPIOB_ARD_D11               15U

#define GPIOC_ULPI_STP              0U
#define GPIOC_RMII_MDC              1U
#define GPIOC_ULPI_DIR              2U
#define GPIOC_FMC_SDCKE0            3U
#define GPIOC_RMII_RXD0             4U
#define GPIOC_RMII_RXD1             5U
#define GPIOC_ARD_D1                6U
#define GPIOC_ARD_D0                7U
#define GPIOC_SD_D0                 8U
#define GPIOC_SD_D1                 9U
#define GPIOC_SD_D2                 10U
#define GPIOC_SD_D3                 11U
#define GPIOC_SD_CLK                12U
#define GPIOC_SD_DETECT             13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_FMC_D2                0U
#define GPIOD_FMC_D3                1U
#define GPIOD_SD_CMD                2U
#define GPIOD_DCMI_D5               3U
#define GPIOD_OTG_FS_OVER_CURRENT   4U
#define GPIOD_OTG_FS_PWR_SW_ON      5U
#define GPIOD_AUDIO_INT             6U
#define GPIOD_SPDIF_RX0             7U
#define GPIOD_FMC_D13               8U
#define GPIOD_FMC_D14               9U
#define GPIOD_FMC_D15               10U
#define GPIOD_QSPI_D0               11U
#define GPIOD_QSPI_D1               12U
#define GPIOD_QSPI_D3               13U
#define GPIOD_FMC_D0                14U
#define GPIOD_FMC_D1                15U

#define GPIOE_FMC_NBL0              0U
#define GPIOE_FMC_NBL1              1U
#define GPIOE_QSPI_D2               2U
#define GPIOE_OTG_HS_OVER_CURRENT   3U
#define GPIOE_LCD_B0                4U
#define GPIOE_DCMI_D6               5U
#define GPIOE_DCMI_D7               6U
#define GPIOE_FMC_D4                7U
#define GPIOE_FMC_D5                8U
#define GPIOE_FMC_D6                9U
#define GPIOE_FMC_D7                10U
#define GPIOE_FMC_D8                11U
#define GPIOE_FMC_D9                12U
#define GPIOE_FMC_D10               13U
#define GPIOE_FMC_11                14U
#define GPIOE_FMC_D12               15U

#define GPIOF_FMC_A0                0U
#define GPIOF_FMC_A1                1U
#define GPIOF_FMC_A2                2U
#define GPIOF_FMC_A3                3U
#define GPIOF_FMC_A4                4U
#define GPIOF_FMC_A5                5U
#define GPIOF_ARD_A5                6U
#define GPIOF_ARD_A4                7U
#define GPIOF_ARD_A3                8U
#define GPIOF_ARD_A2                9U
#define GPIOF_ARD_A1                10U
#define GPIOF_FMC_SDNRAS            11U
#define GPIOF_FMC_A6                12U
#define GPIOF_FMC_A7                13U
#define GPIOF_FMC_A8                14U
#define GPIOF_FMC_A9                15U

#define GPIOG_FMC_A10               0U
#define GPIOG_FMC_A11               1U
#define GPIOG_RMII_RXER             2U
#define GPIOG_EXT_RST               3U
#define GPIOG_FMC_BA0               4U
#define GPIOG_FMC_BA1               5U
#define GPIOG_ARD_D2                6U
#define GPIOG_ARD_D4                7U
#define GPIOG_FMC_SDCLK             8U
#define GPIOG_DCMI_VSYNC            9U
#define GPIOG_SAI2_SDB              10U
#define GPIOG_RMII_TX_EN            11U
#define GPIOG_LCD_B4                12U
#define GPIOG_RMII_TXD0             13U
#define GPIOG_RMII_TXD1             14U
#define GPIOG_FMC_SDNCAS            15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_TP1                   2U
#define GPIOH_FMC_SDNE0             3U
#define GPIOH_ULPI_NXT              4U
#define GPIOH_FMC_SDNWE             5U
#define GPIOH_ARD_D6                6U
#define GPIOH_LCD_SCL               7U
#define GPIOH_LCD_SDA               8U
#define GPIOH_DCMI_D0               9U
#define GPIOH_DCMI_D1               10U
#define GPIOH_DCMI_D2               11U
#define GPIOH_DCMI_D3               12U
#define GPIOH_DCMI_PWR_EN           13U
#define GPIOH_DCMI_D4               14U
#define GPIOH_TP_PH15               15U

#define GPIOI_ARD_D5                0U
#define GPIOI_ARD_D13               1U
#define GPIOI_ARD_D8                2U
#define GPIOI_ARD_D7                3U
#define GPIOI_SAI2_MCLKA            4U
#define GPIOI_SAI2_SCKA             5U
#define GPIOI_SAI2_SDA              6U
#define GPIOI_SAI2_FSA              7U
#define GPIOI_TP2                   8U
#define GPIOI_LCD_VSYNC             9U
#define GPIOI_LCD_HSYNC             10U
#define GPIOI_BUTTON_USER           11U
#define GPIOI_LCD_DISP              12U
#define GPIOI_LCD_INT               13U
#define GPIOI_LCD_CLK               14U
#define GPIOI_LCD_R0                15U

#define GPIOJ_LCD_R1                0U
#define GPIOJ_LCD_R2                1U
#define GPIOJ_LCD_R3                2U
#define GPIOJ_LCD_R4                3U
#define GPIOJ_LCD_R5                4U
#define GPIOJ_LCD_R6                5U
#define GPIOJ_LCD_R7                6U
#define GPIOJ_LCD_G0                7U
#define GPIOJ_LCD_G1                8U
#define GPIOJ_LCD_G2                9U
#define GPIOJ_LCD_G3                10U
#define GPIOJ_LCD_G4                11U
#define GPIOJ_OTG_FS_VBUS           12U
#define GPIOJ_LCD_B1                13U
#define GPIOJ_LCD_B2                14U
#define GPIOJ_LCD_B3                15U

#define GPIOK_LCD_G5                0U
#define GPIOK_LCD_G6                1U
#define GPIOK_LCD_G7                2U
#define GPIOK_LCD_BL_CTRL           3U
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
#define LINE_ARD_A0                 PAL_LINE(GPIOA, 0U)
#define LINE_RMII_REF_CLK           PAL_LINE(GPIOA, 1U)
#define LINE_RMII_MDIO              PAL_LINE(GPIOA, 2U)
#define LINE_ULPI_D0                PAL_LINE(GPIOA, 3U)
#define LINE_DCMI_HSYNC             PAL_LINE(GPIOA, 4U)
#define LINE_ULPI_CK                PAL_LINE(GPIOA, 5U)
#define LINE_DCMI_PIXCK             PAL_LINE(GPIOA, 6U)
#define LINE_RMII_CRS_DV            PAL_LINE(GPIOA, 7U)
#define LINE_ARD_D10                PAL_LINE(GPIOA, 8U)
#define LINE_VCP_TX                 PAL_LINE(GPIOA, 9U)
#define LINE_OTG_FS_ID              PAL_LINE(GPIOA, 10U)
#define LINE_OTG_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP              PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_ARD_D9                 PAL_LINE(GPIOA, 15U)
#define LINE_ULPI_D1                PAL_LINE(GPIOB, 0U)
#define LINE_ULPI_D2                PAL_LINE(GPIOB, 1U)
#define LINE_QSPI_CLK               PAL_LINE(GPIOB, 2U)
#define LINE_SWO                    PAL_LINE(GPIOB, 3U)
#define LINE_ARD_D3                 PAL_LINE(GPIOB, 4U)
#define LINE_ULPI_D7                PAL_LINE(GPIOB, 5U)
#define LINE_QSPI_NCS               PAL_LINE(GPIOB, 6U)
#define LINE_VCP_RX                 PAL_LINE(GPIOB, 7U)
#define LINE_ARD_D15                PAL_LINE(GPIOB, 8U)
#define LINE_ARD_D14                PAL_LINE(GPIOB, 9U)
#define LINE_ULPI_D3                PAL_LINE(GPIOB, 10U)
#define LINE_ULPI_D4                PAL_LINE(GPIOB, 11U)
#define LINE_ULPI_D5                PAL_LINE(GPIOB, 12U)
#define LINE_ULPI_D6                PAL_LINE(GPIOB, 13U)
#define LINE_ARD_D12                PAL_LINE(GPIOB, 14U)
#define LINE_ARD_D11                PAL_LINE(GPIOB, 15U)
#define LINE_ULPI_STP               PAL_LINE(GPIOC, 0U)
#define LINE_RMII_MDC               PAL_LINE(GPIOC, 1U)
#define LINE_ULPI_DIR               PAL_LINE(GPIOC, 2U)
#define LINE_FMC_SDCKE0             PAL_LINE(GPIOC, 3U)
#define LINE_RMII_RXD0              PAL_LINE(GPIOC, 4U)
#define LINE_RMII_RXD1              PAL_LINE(GPIOC, 5U)
#define LINE_ARD_D1                 PAL_LINE(GPIOC, 6U)
#define LINE_ARD_D0                 PAL_LINE(GPIOC, 7U)
#define LINE_SD_D0                  PAL_LINE(GPIOC, 8U)
#define LINE_SD_D1                  PAL_LINE(GPIOC, 9U)
#define LINE_SD_D2                  PAL_LINE(GPIOC, 10U)
#define LINE_SD_D3                  PAL_LINE(GPIOC, 11U)
#define LINE_SD_CLK                 PAL_LINE(GPIOC, 12U)
#define LINE_SD_DETECT              PAL_LINE(GPIOC, 13U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)
#define LINE_FMC_D2                 PAL_LINE(GPIOD, 0U)
#define LINE_FMC_D3                 PAL_LINE(GPIOD, 1U)
#define LINE_SD_CMD                 PAL_LINE(GPIOD, 2U)
#define LINE_DCMI_D5                PAL_LINE(GPIOD, 3U)
#define LINE_OTG_FS_OVER_CURRENT    PAL_LINE(GPIOD, 4U)
#define LINE_OTG_FS_PWR_SW_ON       PAL_LINE(GPIOD, 5U)
#define LINE_AUDIO_INT              PAL_LINE(GPIOD, 6U)
#define LINE_SPDIF_RX0              PAL_LINE(GPIOD, 7U)
#define LINE_FMC_D13                PAL_LINE(GPIOD, 8U)
#define LINE_FMC_D14                PAL_LINE(GPIOD, 9U)
#define LINE_FMC_D15                PAL_LINE(GPIOD, 10U)
#define LINE_QSPI_D0                PAL_LINE(GPIOD, 11U)
#define LINE_QSPI_D1                PAL_LINE(GPIOD, 12U)
#define LINE_QSPI_D3                PAL_LINE(GPIOD, 13U)
#define LINE_FMC_D0                 PAL_LINE(GPIOD, 14U)
#define LINE_FMC_D1                 PAL_LINE(GPIOD, 15U)
#define LINE_FMC_NBL0               PAL_LINE(GPIOE, 0U)
#define LINE_FMC_NBL1               PAL_LINE(GPIOE, 1U)
#define LINE_QSPI_D2                PAL_LINE(GPIOE, 2U)
#define LINE_OTG_HS_OVER_CURRENT    PAL_LINE(GPIOE, 3U)
#define LINE_LCD_B0                 PAL_LINE(GPIOE, 4U)
#define LINE_DCMI_D6                PAL_LINE(GPIOE, 5U)
#define LINE_DCMI_D7                PAL_LINE(GPIOE, 6U)
#define LINE_FMC_D4                 PAL_LINE(GPIOE, 7U)
#define LINE_FMC_D5                 PAL_LINE(GPIOE, 8U)
#define LINE_FMC_D6                 PAL_LINE(GPIOE, 9U)
#define LINE_FMC_D7                 PAL_LINE(GPIOE, 10U)
#define LINE_FMC_D8                 PAL_LINE(GPIOE, 11U)
#define LINE_FMC_D9                 PAL_LINE(GPIOE, 12U)
#define LINE_FMC_D10                PAL_LINE(GPIOE, 13U)
#define LINE_FMC_11                 PAL_LINE(GPIOE, 14U)
#define LINE_FMC_D12                PAL_LINE(GPIOE, 15U)
#define LINE_FMC_A0                 PAL_LINE(GPIOF, 0U)
#define LINE_FMC_A1                 PAL_LINE(GPIOF, 1U)
#define LINE_FMC_A2                 PAL_LINE(GPIOF, 2U)
#define LINE_FMC_A3                 PAL_LINE(GPIOF, 3U)
#define LINE_FMC_A4                 PAL_LINE(GPIOF, 4U)
#define LINE_FMC_A5                 PAL_LINE(GPIOF, 5U)
#define LINE_ARD_A5                 PAL_LINE(GPIOF, 6U)
#define LINE_ARD_A4                 PAL_LINE(GPIOF, 7U)
#define LINE_ARD_A3                 PAL_LINE(GPIOF, 8U)
#define LINE_ARD_A2                 PAL_LINE(GPIOF, 9U)
#define LINE_ARD_A1                 PAL_LINE(GPIOF, 10U)
#define LINE_FMC_SDNRAS             PAL_LINE(GPIOF, 11U)
#define LINE_FMC_A6                 PAL_LINE(GPIOF, 12U)
#define LINE_FMC_A7                 PAL_LINE(GPIOF, 13U)
#define LINE_FMC_A8                 PAL_LINE(GPIOF, 14U)
#define LINE_FMC_A9                 PAL_LINE(GPIOF, 15U)
#define LINE_FMC_A10                PAL_LINE(GPIOG, 0U)
#define LINE_FMC_A11                PAL_LINE(GPIOG, 1U)
#define LINE_RMII_RXER              PAL_LINE(GPIOG, 2U)
#define LINE_EXT_RST                PAL_LINE(GPIOG, 3U)
#define LINE_FMC_BA0                PAL_LINE(GPIOG, 4U)
#define LINE_FMC_BA1                PAL_LINE(GPIOG, 5U)
#define LINE_ARD_D2                 PAL_LINE(GPIOG, 6U)
#define LINE_ARD_D4                 PAL_LINE(GPIOG, 7U)
#define LINE_FMC_SDCLK              PAL_LINE(GPIOG, 8U)
#define LINE_DCMI_VSYNC             PAL_LINE(GPIOG, 9U)
#define LINE_SAI2_SDB               PAL_LINE(GPIOG, 10U)
#define LINE_RMII_TX_EN             PAL_LINE(GPIOG, 11U)
#define LINE_LCD_B4                 PAL_LINE(GPIOG, 12U)
#define LINE_RMII_TXD0              PAL_LINE(GPIOG, 13U)
#define LINE_RMII_TXD1              PAL_LINE(GPIOG, 14U)
#define LINE_FMC_SDNCAS             PAL_LINE(GPIOG, 15U)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)
#define LINE_TP1                    PAL_LINE(GPIOH, 2U)
#define LINE_FMC_SDNE0              PAL_LINE(GPIOH, 3U)
#define LINE_ULPI_NXT               PAL_LINE(GPIOH, 4U)
#define LINE_FMC_SDNWE              PAL_LINE(GPIOH, 5U)
#define LINE_ARD_D6                 PAL_LINE(GPIOH, 6U)
#define LINE_LCD_SCL                PAL_LINE(GPIOH, 7U)
#define LINE_LCD_SDA                PAL_LINE(GPIOH, 8U)
#define LINE_DCMI_D0                PAL_LINE(GPIOH, 9U)
#define LINE_DCMI_D1                PAL_LINE(GPIOH, 10U)
#define LINE_DCMI_D2                PAL_LINE(GPIOH, 11U)
#define LINE_DCMI_D3                PAL_LINE(GPIOH, 12U)
#define LINE_DCMI_PWR_EN            PAL_LINE(GPIOH, 13U)
#define LINE_DCMI_D4                PAL_LINE(GPIOH, 14U)
#define LINE_TP_PH15                PAL_LINE(GPIOH, 15U)
#define LINE_ARD_D5                 PAL_LINE(GPIOI, 0U)
#define LINE_ARD_D13                PAL_LINE(GPIOI, 1U)
#define LINE_ARD_D8                 PAL_LINE(GPIOI, 2U)
#define LINE_ARD_D7                 PAL_LINE(GPIOI, 3U)
#define LINE_SAI2_MCLKA             PAL_LINE(GPIOI, 4U)
#define LINE_SAI2_SCKA              PAL_LINE(GPIOI, 5U)
#define LINE_SAI2_SDA               PAL_LINE(GPIOI, 6U)
#define LINE_SAI2_FSA               PAL_LINE(GPIOI, 7U)
#define LINE_TP2                    PAL_LINE(GPIOI, 8U)
#define LINE_LCD_VSYNC              PAL_LINE(GPIOI, 9U)
#define LINE_LCD_HSYNC              PAL_LINE(GPIOI, 10U)
#define LINE_BUTTON_USER            PAL_LINE(GPIOI, 11U)
#define LINE_LCD_DISP               PAL_LINE(GPIOI, 12U)
#define LINE_LCD_INT                PAL_LINE(GPIOI, 13U)
#define LINE_LCD_CLK                PAL_LINE(GPIOI, 14U)
#define LINE_LCD_R0                 PAL_LINE(GPIOI, 15U)
#define LINE_LCD_R1                 PAL_LINE(GPIOJ, 0U)
#define LINE_LCD_R2                 PAL_LINE(GPIOJ, 1U)
#define LINE_LCD_R3                 PAL_LINE(GPIOJ, 2U)
#define LINE_LCD_R4                 PAL_LINE(GPIOJ, 3U)
#define LINE_LCD_R5                 PAL_LINE(GPIOJ, 4U)
#define LINE_LCD_R6                 PAL_LINE(GPIOJ, 5U)
#define LINE_LCD_R7                 PAL_LINE(GPIOJ, 6U)
#define LINE_LCD_G0                 PAL_LINE(GPIOJ, 7U)
#define LINE_LCD_G1                 PAL_LINE(GPIOJ, 8U)
#define LINE_LCD_G2                 PAL_LINE(GPIOJ, 9U)
#define LINE_LCD_G3                 PAL_LINE(GPIOJ, 10U)
#define LINE_LCD_G4                 PAL_LINE(GPIOJ, 11U)
#define LINE_OTG_FS_VBUS            PAL_LINE(GPIOJ, 12U)
#define LINE_LCD_B1                 PAL_LINE(GPIOJ, 13U)
#define LINE_LCD_B2                 PAL_LINE(GPIOJ, 14U)
#define LINE_LCD_B3                 PAL_LINE(GPIOJ, 15U)
#define LINE_LCD_G5                 PAL_LINE(GPIOK, 0U)
#define LINE_LCD_G6                 PAL_LINE(GPIOK, 1U)
#define LINE_LCD_G7                 PAL_LINE(GPIOK, 2U)
#define LINE_LCD_BL_CTRL            PAL_LINE(GPIOK, 3U)
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
 * PA0  - ARD_A0                    (input pullup).
 * PA1  - RMII_REF_CLK              (alternate 11).
 * PA2  - RMII_MDIO                 (alternate 11).
 * PA3  - ULPI_D0                   (alternate 10).
 * PA4  - DCMI_HSYNC                (input pullup).
 * PA5  - ULPI_CK                   (alternate 10).
 * PA6  - DCMI_PIXCK                (input pullup).
 * PA7  - RMII_CRS_DV               (alternate 11).
 * PA8  - ARD_D10                   (input pullup).
 * PA9  - VCP_TX                    (alternate 7).
 * PA10 - OTG_FS_ID                 (alternate 10).
 * PA11 - OTG_FS_DM                 (alternate 10).
 * PA12 - OTG_FS_DP                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - ARD_D9                    (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_ARD_A0) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_REF_CLK) |\
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_MDIO) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_ULPI_D0) |    \
                                     PIN_MODE_INPUT(GPIOA_DCMI_HSYNC) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_ULPI_CK) |    \
                                     PIN_MODE_INPUT(GPIOA_DCMI_PIXCK) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_CRS_DV) |\
                                     PIN_MODE_INPUT(GPIOA_ARD_D10) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_VCP_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_ID) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_INPUT(GPIOA_ARD_D9))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_ARD_A0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RMII_REF_CLK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_RMII_MDIO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ULPI_D0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DCMI_HSYNC) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ULPI_CK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DCMI_PIXCK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RMII_CRS_DV) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_D10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_VCP_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_ID) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_D9))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_ARD_A0) |        \
                                     PIN_OSPEED_HIGH(GPIOA_RMII_REF_CLK) |  \
                                     PIN_OSPEED_HIGH(GPIOA_RMII_MDIO) |     \
                                     PIN_OSPEED_HIGH(GPIOA_ULPI_D0) |       \
                                     PIN_OSPEED_HIGH(GPIOA_DCMI_HSYNC) |    \
                                     PIN_OSPEED_HIGH(GPIOA_ULPI_CK) |       \
                                     PIN_OSPEED_HIGH(GPIOA_DCMI_PIXCK) |    \
                                     PIN_OSPEED_VERYLOW(GPIOA_RMII_CRS_DV) |\
                                     PIN_OSPEED_HIGH(GPIOA_ARD_D10) |       \
                                     PIN_OSPEED_HIGH(GPIOA_VCP_TX) |        \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_ID) |     \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DM) |     \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DP) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_ARD_D9))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_ARD_A0) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_RMII_REF_CLK) |\
                                     PIN_PUPDR_PULLUP(GPIOA_RMII_MDIO) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_ULPI_D0) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_DCMI_HSYNC) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_ULPI_CK) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_DCMI_PIXCK) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_RMII_CRS_DV) |\
                                     PIN_PUPDR_PULLUP(GPIOA_ARD_D10) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_VCP_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_ID) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_ARD_D9))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_ARD_A0) |           \
                                     PIN_ODR_HIGH(GPIOA_RMII_REF_CLK) |     \
                                     PIN_ODR_HIGH(GPIOA_RMII_MDIO) |        \
                                     PIN_ODR_HIGH(GPIOA_ULPI_D0) |          \
                                     PIN_ODR_HIGH(GPIOA_DCMI_HSYNC) |       \
                                     PIN_ODR_HIGH(GPIOA_ULPI_CK) |          \
                                     PIN_ODR_HIGH(GPIOA_DCMI_PIXCK) |       \
                                     PIN_ODR_HIGH(GPIOA_RMII_CRS_DV) |      \
                                     PIN_ODR_HIGH(GPIOA_ARD_D10) |          \
                                     PIN_ODR_HIGH(GPIOA_VCP_TX) |           \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_ID) |        \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |        \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |        \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_ARD_D9))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_ARD_A0, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_RMII_REF_CLK, 11U) | \
                                     PIN_AFIO_AF(GPIOA_RMII_MDIO, 11U) |    \
                                     PIN_AFIO_AF(GPIOA_ULPI_D0, 10U) |      \
                                     PIN_AFIO_AF(GPIOA_DCMI_HSYNC, 0U) |    \
                                     PIN_AFIO_AF(GPIOA_ULPI_CK, 10U) |      \
                                     PIN_AFIO_AF(GPIOA_DCMI_PIXCK, 0U) |    \
                                     PIN_AFIO_AF(GPIOA_RMII_CRS_DV, 11U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_ARD_D10, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_VCP_TX, 7U) |        \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_ID, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_ARD_D9, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - ULPI_D1                   (alternate 10).
 * PB1  - ULPI_D2                   (alternate 10).
 * PB2  - QSPI_CLK                  (input pullup).
 * PB3  - SWO                       (alternate 0).
 * PB4  - ARD_D3                    (input pullup).
 * PB5  - ULPI_D7                   (alternate 10).
 * PB6  - QSPI_NCS                  (input pullup).
 * PB7  - VCP_RX                    (alternate 7).
 * PB8  - ARD_D15                   (input pullup).
 * PB9  - ARD_D14                   (input pullup).
 * PB10 - ULPI_D3                   (alternate 10).
 * PB11 - ULPI_D4                   (alternate 10).
 * PB12 - ULPI_D5                   (alternate 10).
 * PB13 - ULPI_D6                   (alternate 10).
 * PB14 - ARD_D12                   (input pullup).
 * PB15 - ARD_D11                   (input pullup).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(GPIOB_ULPI_D1) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_ULPI_D2) |    \
                                     PIN_MODE_INPUT(GPIOB_QSPI_CLK) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_SWO) |        \
                                     PIN_MODE_INPUT(GPIOB_ARD_D3) |         \
                                     PIN_MODE_ALTERNATE(GPIOB_ULPI_D7) |    \
                                     PIN_MODE_INPUT(GPIOB_QSPI_NCS) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_VCP_RX) |     \
                                     PIN_MODE_INPUT(GPIOB_ARD_D15) |        \
                                     PIN_MODE_INPUT(GPIOB_ARD_D14) |        \
                                     PIN_MODE_ALTERNATE(GPIOB_ULPI_D3) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_ULPI_D4) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_ULPI_D5) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_ULPI_D6) |    \
                                     PIN_MODE_INPUT(GPIOB_ARD_D12) |        \
                                     PIN_MODE_INPUT(GPIOB_ARD_D11))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_ULPI_D1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ULPI_D2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_QSPI_CLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ULPI_D7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_QSPI_NCS) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_VCP_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D15) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D14) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ULPI_D3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ULPI_D4) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ULPI_D5) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ULPI_D6) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D12) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ARD_D11))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_ULPI_D1) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D2) |       \
                                     PIN_OSPEED_HIGH(GPIOB_QSPI_CLK) |      \
                                     PIN_OSPEED_HIGH(GPIOB_SWO) |           \
                                     PIN_OSPEED_HIGH(GPIOB_ARD_D3) |        \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D7) |       \
                                     PIN_OSPEED_HIGH(GPIOB_QSPI_NCS) |      \
                                     PIN_OSPEED_HIGH(GPIOB_VCP_RX) |        \
                                     PIN_OSPEED_HIGH(GPIOB_ARD_D15) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ARD_D14) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D3) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D4) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D5) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D6) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ARD_D12) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ARD_D11))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_ULPI_D1) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_ULPI_D2) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_QSPI_CLK) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_SWO) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_ARD_D3) |       \
                                     PIN_PUPDR_PULLUP(GPIOB_ULPI_D7) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_QSPI_NCS) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_VCP_RX) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_ARD_D15) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_ARD_D14) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_ULPI_D3) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_ULPI_D4) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_ULPI_D5) |    \
                                     PIN_PUPDR_FLOATING(GPIOB_ULPI_D6) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_ARD_D12) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_ARD_D11))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_ULPI_D1) |          \
                                     PIN_ODR_HIGH(GPIOB_ULPI_D2) |          \
                                     PIN_ODR_HIGH(GPIOB_QSPI_CLK) |         \
                                     PIN_ODR_HIGH(GPIOB_SWO) |              \
                                     PIN_ODR_HIGH(GPIOB_ARD_D3) |           \
                                     PIN_ODR_HIGH(GPIOB_ULPI_D7) |          \
                                     PIN_ODR_HIGH(GPIOB_QSPI_NCS) |         \
                                     PIN_ODR_HIGH(GPIOB_VCP_RX) |           \
                                     PIN_ODR_HIGH(GPIOB_ARD_D15) |          \
                                     PIN_ODR_HIGH(GPIOB_ARD_D14) |          \
                                     PIN_ODR_HIGH(GPIOB_ULPI_D3) |          \
                                     PIN_ODR_HIGH(GPIOB_ULPI_D4) |          \
                                     PIN_ODR_HIGH(GPIOB_ULPI_D5) |          \
                                     PIN_ODR_HIGH(GPIOB_ULPI_D6) |          \
                                     PIN_ODR_HIGH(GPIOB_ARD_D12) |          \
                                     PIN_ODR_HIGH(GPIOB_ARD_D11))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_ULPI_D1, 10U) |      \
                                     PIN_AFIO_AF(GPIOB_ULPI_D2, 10U) |      \
                                     PIN_AFIO_AF(GPIOB_QSPI_CLK, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_SWO, 0U) |           \
                                     PIN_AFIO_AF(GPIOB_ARD_D3, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_ULPI_D7, 10U) |      \
                                     PIN_AFIO_AF(GPIOB_QSPI_NCS, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_VCP_RX, 7U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_ARD_D15, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_ARD_D14, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_ULPI_D3, 10U) |      \
                                     PIN_AFIO_AF(GPIOB_ULPI_D4, 10U) |      \
                                     PIN_AFIO_AF(GPIOB_ULPI_D5, 10U) |      \
                                     PIN_AFIO_AF(GPIOB_ULPI_D6, 10U) |      \
                                     PIN_AFIO_AF(GPIOB_ARD_D12, 0U) |       \
                                     PIN_AFIO_AF(GPIOB_ARD_D11, 0U))

/*
 * GPIOC setup:
 *
 * PC0  - ULPI_STP                  (alternate 10).
 * PC1  - RMII_MDC                  (alternate 11).
 * PC2  - ULPI_DIR                  (alternate 10).
 * PC3  - FMC_SDCKE0                (alternate 12).
 * PC4  - RMII_RXD0                 (alternate 11).
 * PC5  - RMII_RXD1                 (alternate 11).
 * PC6  - ARD_D1                    (input pullup).
 * PC7  - ARD_D0                    (input floating).
 * PC8  - SD_D0                     (alternate 12).
 * PC9  - SD_D1                     (alternate 12).
 * PC10 - SD_D2                     (alternate 12).
 * PC11 - SD_D3                     (alternate 12).
 * PC12 - SD_CLK                    (alternate 12).
 * PC13 - SD_DETECT                 (input pullup).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ALTERNATE(GPIOC_ULPI_STP) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_MDC) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_ULPI_DIR) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_FMC_SDCKE0) | \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_RXD0) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_RXD1) |  \
                                     PIN_MODE_INPUT(GPIOC_ARD_D1) |         \
                                     PIN_MODE_INPUT(GPIOC_ARD_D0) |         \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D0) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D1) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D2) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D3) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_CLK) |     \
                                     PIN_MODE_INPUT(GPIOC_SD_DETECT) |      \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_ULPI_STP) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_RMII_MDC) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ULPI_DIR) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_FMC_SDCKE0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_RMII_RXD0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_RMII_RXD1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_D1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D0) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D2) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_CLK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_DETECT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_ULPI_STP) |      \
                                     PIN_OSPEED_HIGH(GPIOC_RMII_MDC) |      \
                                     PIN_OSPEED_HIGH(GPIOC_ULPI_DIR) |      \
                                     PIN_OSPEED_HIGH(GPIOC_FMC_SDCKE0) |    \
                                     PIN_OSPEED_HIGH(GPIOC_RMII_RXD0) |     \
                                     PIN_OSPEED_HIGH(GPIOC_RMII_RXD1) |     \
                                     PIN_OSPEED_HIGH(GPIOC_ARD_D1) |        \
                                     PIN_OSPEED_HIGH(GPIOC_ARD_D0) |        \
                                     PIN_OSPEED_HIGH(GPIOC_SD_D0) |         \
                                     PIN_OSPEED_HIGH(GPIOC_SD_D1) |         \
                                     PIN_OSPEED_HIGH(GPIOC_SD_D2) |         \
                                     PIN_OSPEED_HIGH(GPIOC_SD_D3) |         \
                                     PIN_OSPEED_HIGH(GPIOC_SD_CLK) |        \
                                     PIN_OSPEED_HIGH(GPIOC_SD_DETECT) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_IN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_ULPI_STP) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_MDC) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_ULPI_DIR) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_FMC_SDCKE0) | \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_RXD0) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_RXD1) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_ARD_D1) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_ARD_D0) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D0) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D1) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D2) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_D3) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_CLK) |     \
                                     PIN_PUPDR_PULLUP(GPIOC_SD_DETECT) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_ULPI_STP) |         \
                                     PIN_ODR_HIGH(GPIOC_RMII_MDC) |         \
                                     PIN_ODR_HIGH(GPIOC_ULPI_DIR) |         \
                                     PIN_ODR_HIGH(GPIOC_FMC_SDCKE0) |       \
                                     PIN_ODR_HIGH(GPIOC_RMII_RXD0) |        \
                                     PIN_ODR_HIGH(GPIOC_RMII_RXD1) |        \
                                     PIN_ODR_HIGH(GPIOC_ARD_D1) |           \
                                     PIN_ODR_HIGH(GPIOC_ARD_D0) |           \
                                     PIN_ODR_HIGH(GPIOC_SD_D0) |            \
                                     PIN_ODR_HIGH(GPIOC_SD_D1) |            \
                                     PIN_ODR_HIGH(GPIOC_SD_D2) |            \
                                     PIN_ODR_HIGH(GPIOC_SD_D3) |            \
                                     PIN_ODR_HIGH(GPIOC_SD_CLK) |           \
                                     PIN_ODR_HIGH(GPIOC_SD_DETECT) |        \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_ULPI_STP, 10U) |     \
                                     PIN_AFIO_AF(GPIOC_RMII_MDC, 11U) |     \
                                     PIN_AFIO_AF(GPIOC_ULPI_DIR, 10U) |     \
                                     PIN_AFIO_AF(GPIOC_FMC_SDCKE0, 12U) |   \
                                     PIN_AFIO_AF(GPIOC_RMII_RXD0, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_RMII_RXD1, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_ARD_D1, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_ARD_D0, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SD_D0, 12U) |        \
                                     PIN_AFIO_AF(GPIOC_SD_D1, 12U) |        \
                                     PIN_AFIO_AF(GPIOC_SD_D2, 12U) |        \
                                     PIN_AFIO_AF(GPIOC_SD_D3, 12U) |        \
                                     PIN_AFIO_AF(GPIOC_SD_CLK, 12U) |       \
                                     PIN_AFIO_AF(GPIOC_SD_DETECT, 0U) |     \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - FMC_D2                    (alternate 12).
 * PD1  - FMC_D3                    (alternate 12).
 * PD2  - SD_CMD                    (alternate 12).
 * PD3  - DCMI_D5                   (input pullup).
 * PD4  - OTG_FS_OVER_CURRENT       (input floating).
 * PD5  - OTG_FS_PWR_SW_ON          (output pushpull maximum).
 * PD6  - AUDIO_INT                 (input pullup).
 * PD7  - SPDIF_RX0                 (input pullup).
 * PD8  - FMC_D13                   (alternate 12).
 * PD9  - FMC_D14                   (alternate 12).
 * PD10 - FMC_D15                   (alternate 12).
 * PD11 - QSPI_D0                   (input pullup).
 * PD12 - QSPI_D1                   (input pullup).
 * PD13 - QSPI_D3                   (input pullup).
 * PD14 - FMC_D0                    (alternate 12).
 * PD15 - FMC_D1                    (alternate 12).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_FMC_D2) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D3) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_SD_CMD) |     \
                                     PIN_MODE_INPUT(GPIOD_DCMI_D5) |        \
                                     PIN_MODE_INPUT(GPIOD_OTG_FS_OVER_CURRENT) |\
                                     PIN_MODE_OUTPUT(GPIOD_OTG_FS_PWR_SW_ON) |\
                                     PIN_MODE_INPUT(GPIOD_AUDIO_INT) |      \
                                     PIN_MODE_INPUT(GPIOD_SPDIF_RX0) |      \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D13) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D14) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D15) |    \
                                     PIN_MODE_INPUT(GPIOD_QSPI_D0) |        \
                                     PIN_MODE_INPUT(GPIOD_QSPI_D1) |        \
                                     PIN_MODE_INPUT(GPIOD_QSPI_D3) |        \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D0) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D1))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_FMC_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CMD) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DCMI_D5) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_OTG_FS_OVER_CURRENT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_OTG_FS_PWR_SW_ON) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_AUDIO_INT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SPDIF_RX0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D13) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D14) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D15) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_QSPI_D0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_QSPI_D1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_QSPI_D3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D1))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_FMC_D2) |        \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D3) |        \
                                     PIN_OSPEED_HIGH(GPIOD_SD_CMD) |        \
                                     PIN_OSPEED_HIGH(GPIOD_DCMI_D5) |       \
                                     PIN_OSPEED_HIGH(GPIOD_OTG_FS_OVER_CURRENT) |\
                                     PIN_OSPEED_HIGH(GPIOD_OTG_FS_PWR_SW_ON) |\
                                     PIN_OSPEED_HIGH(GPIOD_AUDIO_INT) |     \
                                     PIN_OSPEED_HIGH(GPIOD_SPDIF_RX0) |     \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D13) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D14) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D15) |       \
                                     PIN_OSPEED_HIGH(GPIOD_QSPI_D0) |       \
                                     PIN_OSPEED_HIGH(GPIOD_QSPI_D1) |       \
                                     PIN_OSPEED_HIGH(GPIOD_QSPI_D3) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D0) |        \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D1))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_FMC_D2) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D3) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_SD_CMD) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_DCMI_D5) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_OTG_FS_OVER_CURRENT) |\
                                     PIN_PUPDR_FLOATING(GPIOD_OTG_FS_PWR_SW_ON) |\
                                     PIN_PUPDR_PULLUP(GPIOD_AUDIO_INT) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_SPDIF_RX0) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D13) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D14) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D15) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_QSPI_D0) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_QSPI_D1) |      \
                                     PIN_PUPDR_PULLUP(GPIOD_QSPI_D3) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D0) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D1))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_FMC_D2) |           \
                                     PIN_ODR_HIGH(GPIOD_FMC_D3) |           \
                                     PIN_ODR_HIGH(GPIOD_SD_CMD) |           \
                                     PIN_ODR_HIGH(GPIOD_DCMI_D5) |          \
                                     PIN_ODR_HIGH(GPIOD_OTG_FS_OVER_CURRENT) |\
                                     PIN_ODR_HIGH(GPIOD_OTG_FS_PWR_SW_ON) | \
                                     PIN_ODR_HIGH(GPIOD_AUDIO_INT) |        \
                                     PIN_ODR_HIGH(GPIOD_SPDIF_RX0) |        \
                                     PIN_ODR_HIGH(GPIOD_FMC_D13) |          \
                                     PIN_ODR_HIGH(GPIOD_FMC_D14) |          \
                                     PIN_ODR_HIGH(GPIOD_FMC_D15) |          \
                                     PIN_ODR_HIGH(GPIOD_QSPI_D0) |          \
                                     PIN_ODR_LOW(GPIOD_QSPI_D1) |           \
                                     PIN_ODR_LOW(GPIOD_QSPI_D3) |           \
                                     PIN_ODR_LOW(GPIOD_FMC_D0) |            \
                                     PIN_ODR_LOW(GPIOD_FMC_D1))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_FMC_D2, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_D3, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_SD_CMD, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_DCMI_D5, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_OTG_FS_OVER_CURRENT, 0U) |\
                                     PIN_AFIO_AF(GPIOD_OTG_FS_PWR_SW_ON, 0U) |\
                                     PIN_AFIO_AF(GPIOD_AUDIO_INT, 0U) |     \
                                     PIN_AFIO_AF(GPIOD_SPDIF_RX0, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_FMC_D13, 12U) |      \
                                     PIN_AFIO_AF(GPIOD_FMC_D14, 12U) |      \
                                     PIN_AFIO_AF(GPIOD_FMC_D15, 12U) |      \
                                     PIN_AFIO_AF(GPIOD_QSPI_D0, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_QSPI_D1, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_QSPI_D3, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_D0, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_D1, 12U))

/*
 * GPIOE setup:
 *
 * PE0  - FMC_NBL0                  (alternate 12).
 * PE1  - FMC_NBL1                  (alternate 12).
 * PE2  - QSPI_D2                   (input pullup).
 * PE3  - OTG_HS_OVER_CURRENT       (input floating).
 * PE4  - LCD_B0                    (alternate 14).
 * PE5  - DCMI_D6                   (input pullup).
 * PE6  - DCMI_D7                   (input pullup).
 * PE7  - FMC_D4                    (alternate 12).
 * PE8  - FMC_D5                    (alternate 12).
 * PE9  - FMC_D6                    (alternate 12).
 * PE10 - FMC_D7                    (alternate 12).
 * PE11 - FMC_D8                    (alternate 12).
 * PE12 - FMC_D9                    (alternate 12).
 * PE13 - FMC_D10                   (alternate 12).
 * PE14 - FMC_11                    (alternate 12).
 * PE15 - FMC_D12                   (alternate 12).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_FMC_NBL0) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_NBL1) |   \
                                     PIN_MODE_INPUT(GPIOE_QSPI_D2) |        \
                                     PIN_MODE_INPUT(GPIOE_OTG_HS_OVER_CURRENT) |\
                                     PIN_MODE_ALTERNATE(GPIOE_LCD_B0) |     \
                                     PIN_MODE_INPUT(GPIOE_DCMI_D6) |        \
                                     PIN_MODE_INPUT(GPIOE_DCMI_D7) |        \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D4) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D5) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D6) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D7) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D8) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D9) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D10) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_11) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_FMC_D12))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_FMC_NBL0) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_NBL1) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_QSPI_D2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_OTG_HS_OVER_CURRENT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_LCD_B0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DCMI_D6) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_DCMI_D7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_11) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_FMC_D12))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_FMC_NBL0) |      \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_NBL1) |      \
                                     PIN_OSPEED_HIGH(GPIOE_QSPI_D2) |       \
                                     PIN_OSPEED_HIGH(GPIOE_OTG_HS_OVER_CURRENT) |\
                                     PIN_OSPEED_HIGH(GPIOE_LCD_B0) |        \
                                     PIN_OSPEED_HIGH(GPIOE_DCMI_D6) |       \
                                     PIN_OSPEED_HIGH(GPIOE_DCMI_D7) |       \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D4) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D5) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D6) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D7) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D8) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D9) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D10) |       \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_11) |        \
                                     PIN_OSPEED_HIGH(GPIOE_FMC_D12))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_FMC_NBL0) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_NBL1) |   \
                                     PIN_PUPDR_PULLUP(GPIOE_QSPI_D2) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_OTG_HS_OVER_CURRENT) |\
                                     PIN_PUPDR_FLOATING(GPIOE_LCD_B0) |     \
                                     PIN_PUPDR_PULLUP(GPIOE_DCMI_D6) |      \
                                     PIN_PUPDR_PULLUP(GPIOE_DCMI_D7) |      \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D4) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D5) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D6) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D7) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D8) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D9) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D10) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_11) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_FMC_D12))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_FMC_NBL0) |         \
                                     PIN_ODR_HIGH(GPIOE_FMC_NBL1) |         \
                                     PIN_ODR_HIGH(GPIOE_QSPI_D2) |          \
                                     PIN_ODR_HIGH(GPIOE_OTG_HS_OVER_CURRENT) |\
                                     PIN_ODR_HIGH(GPIOE_LCD_B0) |           \
                                     PIN_ODR_HIGH(GPIOE_DCMI_D6) |          \
                                     PIN_ODR_HIGH(GPIOE_DCMI_D7) |          \
                                     PIN_ODR_HIGH(GPIOE_FMC_D4) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D5) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D6) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D7) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D8) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D9) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D10) |          \
                                     PIN_ODR_HIGH(GPIOE_FMC_11) |           \
                                     PIN_ODR_HIGH(GPIOE_FMC_D12))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_FMC_NBL0, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_FMC_NBL1, 12U) |     \
                                     PIN_AFIO_AF(GPIOE_QSPI_D2, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_OTG_HS_OVER_CURRENT, 0U) |\
                                     PIN_AFIO_AF(GPIOE_LCD_B0, 14U) |       \
                                     PIN_AFIO_AF(GPIOE_DCMI_D6, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_DCMI_D7, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D4, 12U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_FMC_D5, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D6, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D7, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D8, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D9, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D10, 12U) |      \
                                     PIN_AFIO_AF(GPIOE_FMC_11, 12U) |       \
                                     PIN_AFIO_AF(GPIOE_FMC_D12, 12U))

/*
 * GPIOF setup:
 *
 * PF0  - FMC_A0                    (alternate 12).
 * PF1  - FMC_A1                    (alternate 12).
 * PF2  - FMC_A2                    (alternate 12).
 * PF3  - FMC_A3                    (alternate 12).
 * PF4  - FMC_A4                    (alternate 12).
 * PF5  - FMC_A5                    (alternate 12).
 * PF6  - ARD_A5                    (input pullup).
 * PF7  - ARD_A4                    (input pullup).
 * PF8  - ARD_A3                    (input pullup).
 * PF9  - ARD_A2                    (input pullup).
 * PF10 - ARD_A1                    (input pullup).
 * PF11 - FMC_SDNRAS                (alternate 12).
 * PF12 - FMC_A6                    (alternate 12).
 * PF13 - FMC_A7                    (alternate 12).
 * PF14 - FMC_A8                    (alternate 12).
 * PF15 - FMC_A9                    (alternate 12).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_FMC_A0) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A1) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A2) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A3) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A4) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A5) |     \
                                     PIN_MODE_INPUT(GPIOF_ARD_A5) |         \
                                     PIN_MODE_INPUT(GPIOF_ARD_A4) |         \
                                     PIN_MODE_INPUT(GPIOF_ARD_A3) |         \
                                     PIN_MODE_INPUT(GPIOF_ARD_A2) |         \
                                     PIN_MODE_INPUT(GPIOF_ARD_A1) |         \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_SDNRAS) | \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A6) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A7) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A8) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_FMC_A9))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_FMC_A0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_A5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_A4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_A3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_A2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_A1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_SDNRAS) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FMC_A9))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_FMC_A0) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A1) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A2) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A3) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A4) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A5) |        \
                                     PIN_OSPEED_HIGH(GPIOF_ARD_A5) |        \
                                     PIN_OSPEED_HIGH(GPIOF_ARD_A4) |        \
                                     PIN_OSPEED_HIGH(GPIOF_ARD_A3) |        \
                                     PIN_OSPEED_HIGH(GPIOF_ARD_A2) |        \
                                     PIN_OSPEED_HIGH(GPIOF_ARD_A1) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_SDNRAS) |    \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A6) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A7) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A8) |        \
                                     PIN_OSPEED_HIGH(GPIOF_FMC_A9))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_FMC_A0) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A1) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A2) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A3) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A4) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A5) |     \
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_A5) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_A4) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_A3) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_A2) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_A1) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_SDNRAS) | \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A6) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A7) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A8) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_FMC_A9))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_FMC_A0) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A1) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A2) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A3) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A4) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A5) |           \
                                     PIN_ODR_HIGH(GPIOF_ARD_A5) |           \
                                     PIN_ODR_HIGH(GPIOF_ARD_A4) |           \
                                     PIN_ODR_HIGH(GPIOF_ARD_A3) |           \
                                     PIN_ODR_HIGH(GPIOF_ARD_A2) |           \
                                     PIN_ODR_HIGH(GPIOF_ARD_A1) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_SDNRAS) |       \
                                     PIN_ODR_HIGH(GPIOF_FMC_A6) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A7) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A8) |           \
                                     PIN_ODR_HIGH(GPIOF_FMC_A9))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_FMC_A0, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A1, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A2, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A3, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A4, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A5, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_ARD_A5, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_ARD_A4, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_ARD_A3, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_ARD_A2, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_ARD_A1, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_FMC_SDNRAS, 12U) |   \
                                     PIN_AFIO_AF(GPIOF_FMC_A6, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A7, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A8, 12U) |       \
                                     PIN_AFIO_AF(GPIOF_FMC_A9, 12U))

/*
 * GPIOG setup:
 *
 * PG0  - FMC_A10                   (alternate 12).
 * PG1  - FMC_A11                   (alternate 12).
 * PG2  - RMII_RXER                 (input pullup).
 * PG3  - EXT_RST                   (input pullup).
 * PG4  - FMC_BA0                   (alternate 12).
 * PG5  - FMC_BA1                   (alternate 12).
 * PG6  - ARD_D2                    (input pullup).
 * PG7  - ARD_D4                    (input pullup).
 * PG8  - FMC_SDCLK                 (alternate 12).
 * PG9  - DCMI_VSYNC                (input pullup).
 * PG10 - SAI2_SDB                  (input pullup).
 * PG11 - RMII_TX_EN                (alternate 11).
 * PG12 - LCD_B4                    (alternate 9).
 * PG13 - RMII_TXD0                 (alternate 11).
 * PG14 - RMII_TXD1                 (alternate 11).
 * PG15 - FMC_SDNCAS                (alternate 12).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_ALTERNATE(GPIOG_FMC_A10) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_A11) |    \
                                     PIN_MODE_INPUT(GPIOG_RMII_RXER) |      \
                                     PIN_MODE_INPUT(GPIOG_EXT_RST) |        \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_BA0) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_BA1) |    \
                                     PIN_MODE_INPUT(GPIOG_ARD_D2) |         \
                                     PIN_MODE_INPUT(GPIOG_ARD_D4) |         \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_SDCLK) |  \
                                     PIN_MODE_INPUT(GPIOG_DCMI_VSYNC) |     \
                                     PIN_MODE_INPUT(GPIOG_SAI2_SDB) |       \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TX_EN) | \
                                     PIN_MODE_ALTERNATE(GPIOG_LCD_B4) |     \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TXD0) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TXD1) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_FMC_A10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_A11) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_RXER) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_EXT_RST) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_BA0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_BA1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ARD_D4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_SDCLK) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_DCMI_VSYNC) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SAI2_SDB) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TX_EN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_LCD_B4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TXD0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TXD1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_FMC_A10) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_A11) |       \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_RXER) |     \
                                     PIN_OSPEED_HIGH(GPIOG_EXT_RST) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_BA0) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_BA1) |       \
                                     PIN_OSPEED_HIGH(GPIOG_ARD_D2) |        \
                                     PIN_OSPEED_HIGH(GPIOG_ARD_D4) |        \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_SDCLK) |     \
                                     PIN_OSPEED_HIGH(GPIOG_DCMI_VSYNC) |    \
                                     PIN_OSPEED_HIGH(GPIOG_SAI2_SDB) |      \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TX_EN) |    \
                                     PIN_OSPEED_HIGH(GPIOG_LCD_B4) |        \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TXD0) |     \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TXD1) |     \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_FMC_A10) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_A11) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_RMII_RXER) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_EXT_RST) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_BA0) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_BA1) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D2) |       \
                                     PIN_PUPDR_PULLUP(GPIOG_ARD_D4) |       \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_SDCLK) |  \
                                     PIN_PUPDR_PULLUP(GPIOG_DCMI_VSYNC) |   \
                                     PIN_PUPDR_PULLUP(GPIOG_SAI2_SDB) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_RMII_TX_EN) | \
                                     PIN_PUPDR_FLOATING(GPIOG_LCD_B4) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_RMII_TXD0) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_RMII_TXD1) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_FMC_A10) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_A11) |          \
                                     PIN_ODR_HIGH(GPIOG_RMII_RXER) |        \
                                     PIN_ODR_HIGH(GPIOG_EXT_RST) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_BA0) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_BA1) |          \
                                     PIN_ODR_HIGH(GPIOG_ARD_D2) |           \
                                     PIN_ODR_HIGH(GPIOG_ARD_D4) |           \
                                     PIN_ODR_HIGH(GPIOG_FMC_SDCLK) |        \
                                     PIN_ODR_HIGH(GPIOG_DCMI_VSYNC) |       \
                                     PIN_ODR_HIGH(GPIOG_SAI2_SDB) |         \
                                     PIN_ODR_HIGH(GPIOG_RMII_TX_EN) |       \
                                     PIN_ODR_HIGH(GPIOG_LCD_B4) |           \
                                     PIN_ODR_HIGH(GPIOG_RMII_TXD0) |        \
                                     PIN_ODR_HIGH(GPIOG_RMII_TXD1) |        \
                                     PIN_ODR_HIGH(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_FMC_A10, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_FMC_A11, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_RMII_RXER, 0U) |     \
                                     PIN_AFIO_AF(GPIOG_EXT_RST, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_FMC_BA0, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_FMC_BA1, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_ARD_D2, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_ARD_D4, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_FMC_SDCLK, 12U) |    \
                                     PIN_AFIO_AF(GPIOG_DCMI_VSYNC, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_SAI2_SDB, 0U) |      \
                                     PIN_AFIO_AF(GPIOG_RMII_TX_EN, 11U) |   \
                                     PIN_AFIO_AF(GPIOG_LCD_B4, 9U) |        \
                                     PIN_AFIO_AF(GPIOG_RMII_TXD0, 11U) |    \
                                     PIN_AFIO_AF(GPIOG_RMII_TXD1, 11U) |    \
                                     PIN_AFIO_AF(GPIOG_FMC_SDNCAS, 12U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - TP1                       (input pullup).
 * PH3  - FMC_SDNE0                 (alternate 12).
 * PH4  - ULPI_NXT                  (alternate 10).
 * PH5  - FMC_SDNWE                 (alternate 12).
 * PH6  - ARD_D6                    (input pullup).
 * PH7  - LCD_SCL                   (alternate 4).
 * PH8  - LCD_SDA                   (alternate 4).
 * PH9  - DCMI_D0                   (input pullup).
 * PH10 - DCMI_D1                   (input pullup).
 * PH11 - DCMI_D2                   (input pullup).
 * PH12 - DCMI_D3                   (input pullup).
 * PH13 - DCMI_PWR_EN               (input pullup).
 * PH14 - DCMI_D4                   (input pullup).
 * PH15 - TP_PH15                   (input pullup).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOH_TP1) |            \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_SDNE0) |  \
                                     PIN_MODE_ALTERNATE(GPIOH_ULPI_NXT) |   \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_SDNWE) |  \
                                     PIN_MODE_INPUT(GPIOH_ARD_D6) |         \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_SCL) |    \
                                     PIN_MODE_ALTERNATE(GPIOH_LCD_SDA) |    \
                                     PIN_MODE_INPUT(GPIOH_DCMI_D0) |        \
                                     PIN_MODE_INPUT(GPIOH_DCMI_D1) |        \
                                     PIN_MODE_INPUT(GPIOH_DCMI_D2) |        \
                                     PIN_MODE_INPUT(GPIOH_DCMI_D3) |        \
                                     PIN_MODE_INPUT(GPIOH_DCMI_PWR_EN) |    \
                                     PIN_MODE_INPUT(GPIOH_DCMI_D4) |        \
                                     PIN_MODE_INPUT(GPIOH_TP_PH15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_TP1) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_SDNE0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_ULPI_NXT) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_SDNWE) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_ARD_D6) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_LCD_SCL) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_LCD_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DCMI_D0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DCMI_D1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DCMI_D2) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DCMI_D3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_DCMI_PWR_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_DCMI_D4) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_TP_PH15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_HIGH(GPIOH_TP1) |           \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_SDNE0) |     \
                                     PIN_OSPEED_HIGH(GPIOH_ULPI_NXT) |      \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_SDNWE) |     \
                                     PIN_OSPEED_HIGH(GPIOH_ARD_D6) |        \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_SCL) |       \
                                     PIN_OSPEED_HIGH(GPIOH_LCD_SDA) |       \
                                     PIN_OSPEED_HIGH(GPIOH_DCMI_D0) |       \
                                     PIN_OSPEED_HIGH(GPIOH_DCMI_D1) |       \
                                     PIN_OSPEED_HIGH(GPIOH_DCMI_D2) |       \
                                     PIN_OSPEED_HIGH(GPIOH_DCMI_D3) |       \
                                     PIN_OSPEED_HIGH(GPIOH_DCMI_PWR_EN) |   \
                                     PIN_OSPEED_HIGH(GPIOH_DCMI_D4) |       \
                                     PIN_OSPEED_HIGH(GPIOH_TP_PH15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_TP1) |          \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_SDNE0) |  \
                                     PIN_PUPDR_FLOATING(GPIOH_ULPI_NXT) |   \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_SDNWE) |  \
                                     PIN_PUPDR_PULLUP(GPIOH_ARD_D6) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_SCL) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_LCD_SDA) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_DCMI_D0) |      \
                                     PIN_PUPDR_PULLUP(GPIOH_DCMI_D1) |      \
                                     PIN_PUPDR_PULLUP(GPIOH_DCMI_D2) |      \
                                     PIN_PUPDR_PULLUP(GPIOH_DCMI_D3) |      \
                                     PIN_PUPDR_PULLUP(GPIOH_DCMI_PWR_EN) |  \
                                     PIN_PUPDR_PULLUP(GPIOH_DCMI_D4) |      \
                                     PIN_PUPDR_PULLUP(GPIOH_TP_PH15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_TP1) |              \
                                     PIN_ODR_HIGH(GPIOH_FMC_SDNE0) |        \
                                     PIN_ODR_HIGH(GPIOH_ULPI_NXT) |         \
                                     PIN_ODR_HIGH(GPIOH_FMC_SDNWE) |        \
                                     PIN_ODR_HIGH(GPIOH_ARD_D6) |           \
                                     PIN_ODR_HIGH(GPIOH_LCD_SCL) |          \
                                     PIN_ODR_HIGH(GPIOH_LCD_SDA) |          \
                                     PIN_ODR_HIGH(GPIOH_DCMI_D0) |          \
                                     PIN_ODR_HIGH(GPIOH_DCMI_D1) |          \
                                     PIN_ODR_HIGH(GPIOH_DCMI_D2) |          \
                                     PIN_ODR_HIGH(GPIOH_DCMI_D3) |          \
                                     PIN_ODR_HIGH(GPIOH_DCMI_PWR_EN) |      \
                                     PIN_ODR_HIGH(GPIOH_DCMI_D4) |          \
                                     PIN_ODR_HIGH(GPIOH_TP_PH15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_TP1, 0U) |           \
                                     PIN_AFIO_AF(GPIOH_FMC_SDNE0, 12U) |    \
                                     PIN_AFIO_AF(GPIOH_ULPI_NXT, 10U) |     \
                                     PIN_AFIO_AF(GPIOH_FMC_SDNWE, 12U) |    \
                                     PIN_AFIO_AF(GPIOH_ARD_D6, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_LCD_SCL, 4U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_LCD_SDA, 4U) |       \
                                     PIN_AFIO_AF(GPIOH_DCMI_D0, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_DCMI_D1, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_DCMI_D2, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_DCMI_D3, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_DCMI_PWR_EN, 0U) |   \
                                     PIN_AFIO_AF(GPIOH_DCMI_D4, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_TP_PH15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - ARD_D5                    (input pullup).
 * PI1  - ARD_D13                   (input pullup).
 * PI2  - ARD_D8                    (input pullup).
 * PI3  - ARD_D7                    (input pullup).
 * PI4  - SAI2_MCLKA                (input pullup).
 * PI5  - SAI2_SCKA                 (input pullup).
 * PI6  - SAI2_SDA                  (input pullup).
 * PI7  - SAI2_FSA                  (input pullup).
 * PI8  - TP2                       (input pullup).
 * PI9  - LCD_VSYNC                 (alternate 14).
 * PI10 - LCD_HSYNC                 (alternate 14).
 * PI11 - BUTTON_USER               (input floating).
 * PI12 - LCD_DISP                  (output pushpull maximum).
 * PI13 - LCD_INT                   (alternate 14).
 * PI14 - LCD_CLK                   (alternate 14).
 * PI15 - LCD_R0                    (alternate 14).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_ARD_D5) |         \
                                     PIN_MODE_INPUT(GPIOI_ARD_D13) |        \
                                     PIN_MODE_INPUT(GPIOI_ARD_D8) |         \
                                     PIN_MODE_INPUT(GPIOI_ARD_D7) |         \
                                     PIN_MODE_INPUT(GPIOI_SAI2_MCLKA) |     \
                                     PIN_MODE_INPUT(GPIOI_SAI2_SCKA) |      \
                                     PIN_MODE_INPUT(GPIOI_SAI2_SDA) |       \
                                     PIN_MODE_INPUT(GPIOI_SAI2_FSA) |       \
                                     PIN_MODE_INPUT(GPIOI_TP2) |            \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_VSYNC) |  \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_HSYNC) |  \
                                     PIN_MODE_INPUT(GPIOI_BUTTON_USER) |    \
                                     PIN_MODE_OUTPUT(GPIOI_LCD_DISP) |      \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_INT) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_CLK) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_R0))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_ARD_D5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_ARD_D13) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_ARD_D8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_ARD_D7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOI_SAI2_MCLKA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOI_SAI2_SCKA) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOI_SAI2_SDA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_SAI2_FSA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_TP2) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_VSYNC) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_HSYNC) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOI_BUTTON_USER) |\
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_DISP) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_INT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_CLK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_R0))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_HIGH(GPIOI_ARD_D5) |        \
                                     PIN_OSPEED_HIGH(GPIOI_ARD_D13) |       \
                                     PIN_OSPEED_HIGH(GPIOI_ARD_D8) |        \
                                     PIN_OSPEED_HIGH(GPIOI_ARD_D7) |        \
                                     PIN_OSPEED_HIGH(GPIOI_SAI2_MCLKA) |    \
                                     PIN_OSPEED_HIGH(GPIOI_SAI2_SCKA) |     \
                                     PIN_OSPEED_HIGH(GPIOI_SAI2_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOI_SAI2_FSA) |      \
                                     PIN_OSPEED_HIGH(GPIOI_TP2) |           \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_VSYNC) |     \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_HSYNC) |     \
                                     PIN_OSPEED_HIGH(GPIOI_BUTTON_USER) |   \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_DISP) |      \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_INT) |       \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_CLK) |       \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_R0))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_ARD_D5) |       \
                                     PIN_PUPDR_PULLUP(GPIOI_ARD_D13) |      \
                                     PIN_PUPDR_PULLUP(GPIOI_ARD_D8) |       \
                                     PIN_PUPDR_PULLUP(GPIOI_ARD_D7) |       \
                                     PIN_PUPDR_PULLUP(GPIOI_SAI2_MCLKA) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_SAI2_SCKA) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_SAI2_SDA) |     \
                                     PIN_PUPDR_PULLUP(GPIOI_SAI2_FSA) |     \
                                     PIN_PUPDR_PULLUP(GPIOI_TP2) |          \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_VSYNC) |  \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_HSYNC) |  \
                                     PIN_PUPDR_FLOATING(GPIOI_BUTTON_USER) |\
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_DISP) |   \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_INT) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_CLK) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_R0))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_ARD_D5) |           \
                                     PIN_ODR_HIGH(GPIOI_ARD_D13) |          \
                                     PIN_ODR_HIGH(GPIOI_ARD_D8) |           \
                                     PIN_ODR_HIGH(GPIOI_ARD_D7) |           \
                                     PIN_ODR_HIGH(GPIOI_SAI2_MCLKA) |       \
                                     PIN_ODR_HIGH(GPIOI_SAI2_SCKA) |        \
                                     PIN_ODR_HIGH(GPIOI_SAI2_SDA) |         \
                                     PIN_ODR_HIGH(GPIOI_SAI2_FSA) |         \
                                     PIN_ODR_HIGH(GPIOI_TP2) |              \
                                     PIN_ODR_HIGH(GPIOI_LCD_VSYNC) |        \
                                     PIN_ODR_HIGH(GPIOI_LCD_HSYNC) |        \
                                     PIN_ODR_HIGH(GPIOI_BUTTON_USER) |      \
                                     PIN_ODR_HIGH(GPIOI_LCD_DISP) |         \
                                     PIN_ODR_HIGH(GPIOI_LCD_INT) |          \
                                     PIN_ODR_HIGH(GPIOI_LCD_CLK) |          \
                                     PIN_ODR_HIGH(GPIOI_LCD_R0))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_ARD_D5, 0U) |        \
                                     PIN_AFIO_AF(GPIOI_ARD_D13, 0U) |       \
                                     PIN_AFIO_AF(GPIOI_ARD_D8, 0U) |        \
                                     PIN_AFIO_AF(GPIOI_ARD_D7, 0U) |        \
                                     PIN_AFIO_AF(GPIOI_SAI2_MCLKA, 0U) |    \
                                     PIN_AFIO_AF(GPIOI_SAI2_SCKA, 0U) |     \
                                     PIN_AFIO_AF(GPIOI_SAI2_SDA, 0U) |      \
                                     PIN_AFIO_AF(GPIOI_SAI2_FSA, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_TP2, 0U) |           \
                                     PIN_AFIO_AF(GPIOI_LCD_VSYNC, 14U) |    \
                                     PIN_AFIO_AF(GPIOI_LCD_HSYNC, 14U) |    \
                                     PIN_AFIO_AF(GPIOI_BUTTON_USER, 0U) |   \
                                     PIN_AFIO_AF(GPIOI_LCD_DISP, 0U) |      \
                                     PIN_AFIO_AF(GPIOI_LCD_INT, 14U) |      \
                                     PIN_AFIO_AF(GPIOI_LCD_CLK, 14U) |      \
                                     PIN_AFIO_AF(GPIOI_LCD_R0, 14U))

/*
 * GPIOJ setup:
 *
 * PJ0  - LCD_R1                    (alternate 14).
 * PJ1  - LCD_R2                    (alternate 14).
 * PJ2  - LCD_R3                    (alternate 14).
 * PJ3  - LCD_R4                    (alternate 14).
 * PJ4  - LCD_R5                    (alternate 14).
 * PJ5  - LCD_R6                    (alternate 14).
 * PJ6  - LCD_R7                    (alternate 14).
 * PJ7  - LCD_G0                    (alternate 14).
 * PJ8  - LCD_G1                    (alternate 14).
 * PJ9  - LCD_G2                    (alternate 14).
 * PJ10 - LCD_G3                    (alternate 14).
 * PJ11 - LCD_G4                    (alternate 14).
 * PJ12 - OTG_FS_VBUS               (input floating).
 * PJ13 - LCD_B1                    (alternate 14).
 * PJ14 - LCD_B2                    (alternate 14).
 * PJ15 - LCD_B3                    (alternate 14).
 */
#define VAL_GPIOJ_MODER             (PIN_MODE_ALTERNATE(GPIOJ_LCD_R1) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R2) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R3) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R4) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R5) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R6) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_R7) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G0) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G1) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G2) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G3) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_G4) |     \
                                     PIN_MODE_INPUT(GPIOJ_OTG_FS_VBUS) |    \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_B1) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_B2) |     \
                                     PIN_MODE_ALTERNATE(GPIOJ_LCD_B3))
#define VAL_GPIOJ_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_R7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_G4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_OTG_FS_VBUS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_B1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_B2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LCD_B3))
#define VAL_GPIOJ_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOJ_LCD_R1) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_R2) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_R3) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_R4) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_R5) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_R6) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_R7) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_G0) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_G1) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_G2) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_G3) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_G4) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_OTG_FS_VBUS) |\
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_B1) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_B2) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_LCD_B3))
#define VAL_GPIOJ_PUPDR             (PIN_PUPDR_FLOATING(GPIOJ_LCD_R1) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R2) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R3) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R4) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R5) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R6) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_R7) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_G0) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_G1) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_G2) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_G3) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_G4) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_OTG_FS_VBUS) |\
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_B1) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_B2) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LCD_B3))
#define VAL_GPIOJ_ODR               (PIN_ODR_HIGH(GPIOJ_LCD_R1) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R2) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R3) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R4) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R5) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R6) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_R7) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G0) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G1) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G2) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G3) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_G4) |           \
                                     PIN_ODR_HIGH(GPIOJ_OTG_FS_VBUS) |      \
                                     PIN_ODR_HIGH(GPIOJ_LCD_B1) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_B2) |           \
                                     PIN_ODR_HIGH(GPIOJ_LCD_B3))
#define VAL_GPIOJ_AFRL              (PIN_AFIO_AF(GPIOJ_LCD_R1, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R2, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R3, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R4, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R5, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R6, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_R7, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_G0, 14U))
#define VAL_GPIOJ_AFRH              (PIN_AFIO_AF(GPIOJ_LCD_G1, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_G2, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_G3, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_G4, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_OTG_FS_VBUS, 0U) |   \
                                     PIN_AFIO_AF(GPIOJ_LCD_B1, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_B2, 14U) |       \
                                     PIN_AFIO_AF(GPIOJ_LCD_B3, 14U))

/*
 * GPIOK setup:
 *
 * PK0  - LCD_G5                    (alternate 14).
 * PK1  - LCD_G6                    (alternate 14).
 * PK2  - LCD_G7                    (alternate 14).
 * PK3  - LCD_BL_CTRL               (output pushpull minimum).
 * PK4  - LCD_B5                    (alternate 14).
 * PK5  - LCD_B6                    (alternate 14).
 * PK6  - LCD_B7                    (alternate 14).
 * PK7  - LCD_DE                    (alternate 14).
 * PK8  - PIN8                      (input floating).
 * PK9  - PIN9                      (input floating).
 * PK10 - PIN10                     (input floating).
 * PK11 - PIN11                     (input floating).
 * PK12 - PIN12                     (input floating).
 * PK13 - PIN13                     (input floating).
 * PK14 - PIN14                     (input floating).
 * PK15 - PIN15                     (input floating).
 */
#define VAL_GPIOK_MODER             (PIN_MODE_ALTERNATE(GPIOK_LCD_G5) |     \
                                     PIN_MODE_ALTERNATE(GPIOK_LCD_G6) |     \
                                     PIN_MODE_ALTERNATE(GPIOK_LCD_G7) |     \
                                     PIN_MODE_OUTPUT(GPIOK_LCD_BL_CTRL) |   \
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
#define VAL_GPIOK_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOK_LCD_G5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LCD_G6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LCD_G7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOK_LCD_BL_CTRL) |\
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
#define VAL_GPIOK_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOK_LCD_G5) |     \
                                     PIN_OSPEED_VERYLOW(GPIOK_LCD_G6) |     \
                                     PIN_OSPEED_VERYLOW(GPIOK_LCD_G7) |     \
                                     PIN_OSPEED_VERYLOW(GPIOK_LCD_BL_CTRL) |\
                                     PIN_OSPEED_VERYLOW(GPIOK_LCD_B5) |     \
                                     PIN_OSPEED_VERYLOW(GPIOK_LCD_B6) |     \
                                     PIN_OSPEED_VERYLOW(GPIOK_LCD_B7) |     \
                                     PIN_OSPEED_VERYLOW(GPIOK_LCD_DE) |     \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN15))
#define VAL_GPIOK_PUPDR             (PIN_PUPDR_FLOATING(GPIOK_LCD_G5) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_G6) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_G7) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_BL_CTRL) |\
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_B5) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_B6) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_B7) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_LCD_DE) |     \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN15))
#define VAL_GPIOK_ODR               (PIN_ODR_HIGH(GPIOK_LCD_G5) |           \
                                     PIN_ODR_HIGH(GPIOK_LCD_G6) |           \
                                     PIN_ODR_HIGH(GPIOK_LCD_G7) |           \
                                     PIN_ODR_LOW(GPIOK_LCD_BL_CTRL) |       \
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
#define VAL_GPIOK_AFRL              (PIN_AFIO_AF(GPIOK_LCD_G5, 14U) |       \
                                     PIN_AFIO_AF(GPIOK_LCD_G6, 14U) |       \
                                     PIN_AFIO_AF(GPIOK_LCD_G7, 14U) |       \
                                     PIN_AFIO_AF(GPIOK_LCD_BL_CTRL, 0U) |   \
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
