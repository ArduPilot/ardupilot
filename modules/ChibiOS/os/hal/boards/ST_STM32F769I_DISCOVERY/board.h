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
 * Setup for STMicroelectronics STM32F769I-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F769I_DISCOVERY
#define BOARD_NAME                  "STMicroelectronics STM32F769I-Discovery"

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
#undef STM32F769xx
#define STM32F769xx

/*
 * IO pins assignments.
 */
#define GPIOA_BUTTON_USER           0U
#define GPIOA_RMII_REF_CLK          1U
#define GPIOA_RMII_MDIO             2U
#define GPIOA_ULPI_D0               3U
#define GPIOA_ARD_A1                4U
#define GPIOA_ULPI_CK               5U
#define GPIOA_ARD_A0                6U
#define GPIOA_RMII_CRS_DV           7U
#define GPIOA_CEC_CLK               8U
#define GPIOA_VCP_TX                9U
#define GPIOA_VCP_RX                10U
#define GPIOA_ARD_D10               11U
#define GPIOA_ARD_D13               12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_CEC                   15U

#define GPIOB_ULPI_D1               0U
#define GPIOB_ULPI_D2               1U
#define GPIOB_QSPI_CLK              2U
#define GPIOB_SD_D2                 3U
#define GPIOB_SD_D3                 4U
#define GPIOB_ULPI_D7               5U
#define GPIOB_QSPI_NCS              6U
#define GPIOB_AUDIO_SDA             7U
#define GPIOB_LCD_SDA               7U
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
#define GPIOC_ARD_A2                2U
#define GPIOC_DFSDM_DATIN1          3U
#define GPIOC_RMII_RXD0             4U
#define GPIOC_RMII_RXD1             5U
#define GPIOC_ARD_D1                6U
#define GPIOC_ARD_D0                7U
#define GPIOC_ARD_D5                8U
#define GPIOC_QSPI_D0               9U
#define GPIOC_QSPI_D1               10U
#define GPIOC_DFSDM_DATIN5          11U
#define GPIOC_WIFI_RX               12U
#define GPIOC_PIN13                 13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_FMC_D2                0U
#define GPIOD_FMC_D3                1U
#define GPIOD_WIFI_TX               2U
#define GPIOD_DFSDM_CKOUT           3U
#define GPIOD_OTG_HS_OVER_CURRENT   4U
#define GPIOD_RMII_RXER             5U
#define GPIOD_SD_CLK                6U
#define GPIOD_SD_CMD                7U
#define GPIOD_FMC_D13               8U
#define GPIOD_FMC_D14               9U
#define GPIOD_FMC_D15               10U
#define GPIOD_SPDIF_I2S             11U
#define GPIOD_AUDIO_SCL             12U
#define GPIOD_LCD_SCL               12U
#define GPIOD_QSPI_D3               13U
#define GPIOD_FMC_D0                14U
#define GPIOD_FMC_D1                15U

#define GPIOE_FMC_NBL0              0U
#define GPIOE_FMC_NBL1              1U
#define GPIOE_QSPI_D2               2U
#define GPIOE_SAI1_SDB              3U
#define GPIOE_SAI1_FSA              4U
#define GPIOE_SAI1_SCKA             5U
#define GPIOE_SAI1_SDA              6U
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
#define GPIOF_ARD_D3                6U
#define GPIOF_ARD_D6                7U
#define GPIOF_ARD_A4                8U
#define GPIOF_ARD_A5                9U
#define GPIOF_ARD_A3                10U
#define GPIOF_FMC_SDNRAS            11U
#define GPIOF_FMC_A6                12U
#define GPIOF_FMC_A7                13U
#define GPIOF_FMC_A8                14U
#define GPIOF_FMC_A9                15U

#define GPIOG_FMC_A10               0U
#define GPIOG_FMC_A11               1U
#define GPIOG_FMC_A12               2U
#define GPIOG_EXT_SCL               3U
#define GPIOG_FMC_BA0               4U
#define GPIOG_FMC_BA1               5U
#define GPIOG_EXT_SDA               6U
#define GPIOG_SAI1_MCLKA            7U
#define GPIOG_FMC_SDCLK             8U
#define GPIOG_SD_D0                 9U
#define GPIOG_SD_D1                 10U
#define GPIOG_RMII_TX_EN            11U
#define GPIOG_SPDIF_RX              12U
#define GPIOG_RMII_TXD0             13U
#define GPIOG_RMII_TXD1             14U
#define GPIOG_FMC_SDNCAS            15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_FMC_SDCKE0            2U
#define GPIOH_FMC_SDNE0             3U
#define GPIOH_ULPI_NXT              4U
#define GPIOH_FMC_SDNWE             5U
#define GPIOH_ARD_D9                6U
#define GPIOH_EXT_RST               7U
#define GPIOH_FMC_D16               8U
#define GPIOH_FMC_D17               9U
#define GPIOH_FMC_D18               10U
#define GPIOH_FMC_D19               11U
#define GPIOH_FMC_D20               12U
#define GPIOH_FMC_D21               13U
#define GPIOH_FMC_D22               14U
#define GPIOH_FMC_D23               15U

#define GPIOI_FMC_D24               0U
#define GPIOI_FMC_D25               1U
#define GPIOI_FMC_D26               2U
#define GPIOI_FMC_D27               3U
#define GPIOI_FMC_NBL2              4U
#define GPIOI_FMC_NBL3              5U
#define GPIOI_FMC_D28               6U
#define GPIOI_FMC_D29               7U
#define GPIOI_PIN8                  8U
#define GPIOI_FMC_D30               9U
#define GPIOI_FMC_D31               10U
#define GPIOI_ULPI_DIR              11U
#define GPIOI_PIN12                 12U
#define GPIOI_LCD_INT               13U
#define GPIOI_LCD_BL_CTRL           14U
#define GPIOI_SD_DETECT             15U

#define GPIOJ_ARD_D4                0U
#define GPIOJ_ARD_D2                1U
#define GPIOJ_DSI_TE                2U
#define GPIOJ_ARD_D7                3U
#define GPIOJ_ARD_D8                4U
#define GPIOJ_LED2_GREEN            5U
#define GPIOJ_PIN6                  6U
#define GPIOJ_PIN7                  7U
#define GPIOJ_PIN8                  8U
#define GPIOJ_PIN9                  9U
#define GPIOJ_PIN10                 10U
#define GPIOJ_PIN11                 11U
#define GPIOJ_AUDIO_INT             12U
#define GPIOJ_LED1_RED              13U
#define GPIOJ_WIFI_RST              14U
#define GPIOJ_DSI_RESET             15U

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
#define LINE_BUTTON_USER            PAL_LINE(GPIOA, 0U)
#define LINE_RMII_REF_CLK           PAL_LINE(GPIOA, 1U)
#define LINE_RMII_MDIO              PAL_LINE(GPIOA, 2U)
#define LINE_ULPI_D0                PAL_LINE(GPIOA, 3U)
#define LINE_ARD_A1                 PAL_LINE(GPIOA, 4U)
#define LINE_ULPI_CK                PAL_LINE(GPIOA, 5U)
#define LINE_ARD_A0                 PAL_LINE(GPIOA, 6U)
#define LINE_RMII_CRS_DV            PAL_LINE(GPIOA, 7U)
#define LINE_CEC_CLK                PAL_LINE(GPIOA, 8U)
#define LINE_VCP_TX                 PAL_LINE(GPIOA, 9U)
#define LINE_VCP_RX                 PAL_LINE(GPIOA, 10U)
#define LINE_ARD_D10                PAL_LINE(GPIOA, 11U)
#define LINE_ARD_D13                PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_CEC                    PAL_LINE(GPIOA, 15U)
#define LINE_ULPI_D1                PAL_LINE(GPIOB, 0U)
#define LINE_ULPI_D2                PAL_LINE(GPIOB, 1U)
#define LINE_QSPI_CLK               PAL_LINE(GPIOB, 2U)
#define LINE_SD_D2                  PAL_LINE(GPIOB, 3U)
#define LINE_SD_D3                  PAL_LINE(GPIOB, 4U)
#define LINE_ULPI_D7                PAL_LINE(GPIOB, 5U)
#define LINE_QSPI_NCS               PAL_LINE(GPIOB, 6U)
#define LINE_AUDIO_SDA              PAL_LINE(GPIOB, 7U)
#define LINE_LCD_SDA                PAL_LINE(GPIOB, 7U)
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
#define LINE_ARD_A2                 PAL_LINE(GPIOC, 2U)
#define LINE_DFSDM_DATIN1           PAL_LINE(GPIOC, 3U)
#define LINE_RMII_RXD0              PAL_LINE(GPIOC, 4U)
#define LINE_RMII_RXD1              PAL_LINE(GPIOC, 5U)
#define LINE_ARD_D1                 PAL_LINE(GPIOC, 6U)
#define LINE_ARD_D0                 PAL_LINE(GPIOC, 7U)
#define LINE_ARD_D5                 PAL_LINE(GPIOC, 8U)
#define LINE_QSPI_D0                PAL_LINE(GPIOC, 9U)
#define LINE_QSPI_D1                PAL_LINE(GPIOC, 10U)
#define LINE_DFSDM_DATIN5           PAL_LINE(GPIOC, 11U)
#define LINE_WIFI_RX                PAL_LINE(GPIOC, 12U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)
#define LINE_FMC_D2                 PAL_LINE(GPIOD, 0U)
#define LINE_FMC_D3                 PAL_LINE(GPIOD, 1U)
#define LINE_WIFI_TX                PAL_LINE(GPIOD, 2U)
#define LINE_DFSDM_CKOUT            PAL_LINE(GPIOD, 3U)
#define LINE_OTG_HS_OVER_CURRENT    PAL_LINE(GPIOD, 4U)
#define LINE_RMII_RXER              PAL_LINE(GPIOD, 5U)
#define LINE_SD_CLK                 PAL_LINE(GPIOD, 6U)
#define LINE_SD_CMD                 PAL_LINE(GPIOD, 7U)
#define LINE_FMC_D13                PAL_LINE(GPIOD, 8U)
#define LINE_FMC_D14                PAL_LINE(GPIOD, 9U)
#define LINE_FMC_D15                PAL_LINE(GPIOD, 10U)
#define LINE_SPDIF_I2S              PAL_LINE(GPIOD, 11U)
#define LINE_AUDIO_SCL              PAL_LINE(GPIOD, 12U)
#define LINE_LCD_SCL                PAL_LINE(GPIOD, 12U)
#define LINE_QSPI_D3                PAL_LINE(GPIOD, 13U)
#define LINE_FMC_D0                 PAL_LINE(GPIOD, 14U)
#define LINE_FMC_D1                 PAL_LINE(GPIOD, 15U)
#define LINE_FMC_NBL0               PAL_LINE(GPIOE, 0U)
#define LINE_FMC_NBL1               PAL_LINE(GPIOE, 1U)
#define LINE_QSPI_D2                PAL_LINE(GPIOE, 2U)
#define LINE_SAI1_SDB               PAL_LINE(GPIOE, 3U)
#define LINE_SAI1_FSA               PAL_LINE(GPIOE, 4U)
#define LINE_SAI1_SCKA              PAL_LINE(GPIOE, 5U)
#define LINE_SAI1_SDA               PAL_LINE(GPIOE, 6U)
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
#define LINE_ARD_D3                 PAL_LINE(GPIOF, 6U)
#define LINE_ARD_D6                 PAL_LINE(GPIOF, 7U)
#define LINE_ARD_A4                 PAL_LINE(GPIOF, 8U)
#define LINE_ARD_A5                 PAL_LINE(GPIOF, 9U)
#define LINE_ARD_A3                 PAL_LINE(GPIOF, 10U)
#define LINE_FMC_SDNRAS             PAL_LINE(GPIOF, 11U)
#define LINE_FMC_A6                 PAL_LINE(GPIOF, 12U)
#define LINE_FMC_A7                 PAL_LINE(GPIOF, 13U)
#define LINE_FMC_A8                 PAL_LINE(GPIOF, 14U)
#define LINE_FMC_A9                 PAL_LINE(GPIOF, 15U)
#define LINE_FMC_A10                PAL_LINE(GPIOG, 0U)
#define LINE_FMC_A11                PAL_LINE(GPIOG, 1U)
#define LINE_FMC_A12                PAL_LINE(GPIOG, 2U)
#define LINE_EXT_SCL                PAL_LINE(GPIOG, 3U)
#define LINE_FMC_BA0                PAL_LINE(GPIOG, 4U)
#define LINE_FMC_BA1                PAL_LINE(GPIOG, 5U)
#define LINE_EXT_SDA                PAL_LINE(GPIOG, 6U)
#define LINE_SAI1_MCLKA             PAL_LINE(GPIOG, 7U)
#define LINE_FMC_SDCLK              PAL_LINE(GPIOG, 8U)
#define LINE_SD_D0                  PAL_LINE(GPIOG, 9U)
#define LINE_SD_D1                  PAL_LINE(GPIOG, 10U)
#define LINE_RMII_TX_EN             PAL_LINE(GPIOG, 11U)
#define LINE_SPDIF_RX               PAL_LINE(GPIOG, 12U)
#define LINE_RMII_TXD0              PAL_LINE(GPIOG, 13U)
#define LINE_RMII_TXD1              PAL_LINE(GPIOG, 14U)
#define LINE_FMC_SDNCAS             PAL_LINE(GPIOG, 15U)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)
#define LINE_FMC_SDCKE0             PAL_LINE(GPIOH, 2U)
#define LINE_FMC_SDNE0              PAL_LINE(GPIOH, 3U)
#define LINE_ULPI_NXT               PAL_LINE(GPIOH, 4U)
#define LINE_FMC_SDNWE              PAL_LINE(GPIOH, 5U)
#define LINE_ARD_D9                 PAL_LINE(GPIOH, 6U)
#define LINE_EXT_RST                PAL_LINE(GPIOH, 7U)
#define LINE_FMC_D16                PAL_LINE(GPIOH, 8U)
#define LINE_FMC_D17                PAL_LINE(GPIOH, 9U)
#define LINE_FMC_D18                PAL_LINE(GPIOH, 10U)
#define LINE_FMC_D19                PAL_LINE(GPIOH, 11U)
#define LINE_FMC_D20                PAL_LINE(GPIOH, 12U)
#define LINE_FMC_D21                PAL_LINE(GPIOH, 13U)
#define LINE_FMC_D22                PAL_LINE(GPIOH, 14U)
#define LINE_FMC_D23                PAL_LINE(GPIOH, 15U)
#define LINE_FMC_D24                PAL_LINE(GPIOI, 0U)
#define LINE_FMC_D25                PAL_LINE(GPIOI, 1U)
#define LINE_FMC_D26                PAL_LINE(GPIOI, 2U)
#define LINE_FMC_D27                PAL_LINE(GPIOI, 3U)
#define LINE_FMC_NBL2               PAL_LINE(GPIOI, 4U)
#define LINE_FMC_NBL3               PAL_LINE(GPIOI, 5U)
#define LINE_FMC_D28                PAL_LINE(GPIOI, 6U)
#define LINE_FMC_D29                PAL_LINE(GPIOI, 7U)
#define LINE_FMC_D30                PAL_LINE(GPIOI, 9U)
#define LINE_FMC_D31                PAL_LINE(GPIOI, 10U)
#define LINE_ULPI_DIR               PAL_LINE(GPIOI, 11U)
#define LINE_LCD_INT                PAL_LINE(GPIOI, 13U)
#define LINE_LCD_BL_CTRL            PAL_LINE(GPIOI, 14U)
#define LINE_SD_DETECT              PAL_LINE(GPIOI, 15U)
#define LINE_ARD_D4                 PAL_LINE(GPIOJ, 0U)
#define LINE_ARD_D2                 PAL_LINE(GPIOJ, 1U)
#define LINE_DSI_TE                 PAL_LINE(GPIOJ, 2U)
#define LINE_ARD_D7                 PAL_LINE(GPIOJ, 3U)
#define LINE_ARD_D8                 PAL_LINE(GPIOJ, 4U)
#define LINE_LED2_GREEN             PAL_LINE(GPIOJ, 5U)
#define LINE_AUDIO_INT              PAL_LINE(GPIOJ, 12U)
#define LINE_LED1_RED               PAL_LINE(GPIOJ, 13U)
#define LINE_WIFI_RST               PAL_LINE(GPIOJ, 14U)
#define LINE_DSI_RESET              PAL_LINE(GPIOJ, 15U)

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
 * PA0  - BUTTON_USER               (input floating).
 * PA1  - RMII_REF_CLK              (alternate 11).
 * PA2  - RMII_MDIO                 (alternate 11).
 * PA3  - ULPI_D0                   (alternate 10).
 * PA4  - ARD_A1                    (input pullup).
 * PA5  - ULPI_CK                   (alternate 10).
 * PA6  - ARD_A0                    (input pullup).
 * PA7  - RMII_CRS_DV               (alternate 11).
 * PA8  - CEC_CLK                   (alternate 0).
 * PA9  - VCP_TX                    (alternate 7).
 * PA10 - VCP_RX                    (alternate 7).
 * PA11 - ARD_D10                   (input pullup).
 * PA12 - ARD_D13                   (input pullup).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - CEC                       (alternate 3).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_BUTTON_USER) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_REF_CLK) |\
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_MDIO) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_ULPI_D0) |    \
                                     PIN_MODE_INPUT(GPIOA_ARD_A1) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_ULPI_CK) |    \
                                     PIN_MODE_INPUT(GPIOA_ARD_A0) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_RMII_CRS_DV) |\
                                     PIN_MODE_ALTERNATE(GPIOA_CEC_CLK) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_VCP_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_VCP_RX) |     \
                                     PIN_MODE_INPUT(GPIOA_ARD_D10) |        \
                                     PIN_MODE_INPUT(GPIOA_ARD_D13) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_CEC))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_BUTTON_USER) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_RMII_REF_CLK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_RMII_MDIO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ULPI_D0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_A1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ULPI_CK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_A0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_RMII_CRS_DV) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_CEC_CLK) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_VCP_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_VCP_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_D10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ARD_D13) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CEC))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_BUTTON_USER) |   \
                                     PIN_OSPEED_HIGH(GPIOA_RMII_REF_CLK) |  \
                                     PIN_OSPEED_HIGH(GPIOA_RMII_MDIO) |     \
                                     PIN_OSPEED_HIGH(GPIOA_ULPI_D0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_ARD_A1) |     \
                                     PIN_OSPEED_HIGH(GPIOA_ULPI_CK) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_ARD_A0) |     \
                                     PIN_OSPEED_HIGH(GPIOA_RMII_CRS_DV) |   \
                                     PIN_OSPEED_HIGH(GPIOA_CEC_CLK) |       \
                                     PIN_OSPEED_HIGH(GPIOA_VCP_TX) |        \
                                     PIN_OSPEED_HIGH(GPIOA_VCP_RX) |        \
                                     PIN_OSPEED_VERYLOW(GPIOA_ARD_D10) |    \
                                     PIN_OSPEED_VERYLOW(GPIOA_ARD_D13) |    \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_CEC))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_BUTTON_USER) |\
                                     PIN_PUPDR_FLOATING(GPIOA_RMII_REF_CLK) |\
                                     PIN_PUPDR_PULLUP(GPIOA_RMII_MDIO) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_ULPI_D0) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_ARD_A1) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_ULPI_CK) |    \
                                     PIN_PUPDR_PULLUP(GPIOA_ARD_A0) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_RMII_CRS_DV) |\
                                     PIN_PUPDR_FLOATING(GPIOA_CEC_CLK) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_VCP_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_VCP_RX) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_ARD_D10) |      \
                                     PIN_PUPDR_PULLUP(GPIOA_ARD_D13) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_CEC))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_BUTTON_USER) |      \
                                     PIN_ODR_HIGH(GPIOA_RMII_REF_CLK) |     \
                                     PIN_ODR_HIGH(GPIOA_RMII_MDIO) |        \
                                     PIN_ODR_HIGH(GPIOA_ULPI_D0) |          \
                                     PIN_ODR_HIGH(GPIOA_ARD_A1) |           \
                                     PIN_ODR_HIGH(GPIOA_ULPI_CK) |          \
                                     PIN_ODR_HIGH(GPIOA_ARD_A0) |           \
                                     PIN_ODR_HIGH(GPIOA_RMII_CRS_DV) |      \
                                     PIN_ODR_HIGH(GPIOA_CEC_CLK) |          \
                                     PIN_ODR_HIGH(GPIOA_VCP_TX) |           \
                                     PIN_ODR_HIGH(GPIOA_VCP_RX) |           \
                                     PIN_ODR_HIGH(GPIOA_ARD_D10) |          \
                                     PIN_ODR_HIGH(GPIOA_ARD_D13) |          \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_CEC))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_BUTTON_USER, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_RMII_REF_CLK, 11U) | \
                                     PIN_AFIO_AF(GPIOA_RMII_MDIO, 11U) |    \
                                     PIN_AFIO_AF(GPIOA_ULPI_D0, 10U) |      \
                                     PIN_AFIO_AF(GPIOA_ARD_A1, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_ULPI_CK, 10U) |      \
                                     PIN_AFIO_AF(GPIOA_ARD_A0, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_RMII_CRS_DV, 11U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_CEC_CLK, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_VCP_TX, 7U) |        \
                                     PIN_AFIO_AF(GPIOA_VCP_RX, 7U) |        \
                                     PIN_AFIO_AF(GPIOA_ARD_D10, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_ARD_D13, 0U) |       \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_CEC, 3U))

/*
 * GPIOB setup:
 *
 * PB0  - ULPI_D1                   (alternate 10).
 * PB1  - ULPI_D2                   (alternate 10).
 * PB2  - QSPI_CLK                  (input pullup).
 * PB3  - SD_D2                     (alternate 10).
 * PB4  - SD_D3                     (alternate 10).
 * PB5  - ULPI_D7                   (alternate 10).
 * PB6  - QSPI_NCS                  (input pullup).
 * PB7  - AUDIO_SDA LCD_SDA         (alternate 11).
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
                                     PIN_MODE_ALTERNATE(GPIOB_SD_D2) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_SD_D3) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_ULPI_D7) |    \
                                     PIN_MODE_INPUT(GPIOB_QSPI_NCS) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_AUDIO_SDA) |  \
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
                                     PIN_OTYPE_PUSHPULL(GPIOB_SD_D2) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SD_D3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ULPI_D7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOB_QSPI_NCS) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_AUDIO_SDA) |  \
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
                                     PIN_OSPEED_HIGH(GPIOB_SD_D2) |         \
                                     PIN_OSPEED_HIGH(GPIOB_SD_D3) |         \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D7) |       \
                                     PIN_OSPEED_HIGH(GPIOB_QSPI_NCS) |      \
                                     PIN_OSPEED_HIGH(GPIOB_AUDIO_SDA) |     \
                                     PIN_OSPEED_VERYLOW(GPIOB_ARD_D15) |    \
                                     PIN_OSPEED_VERYLOW(GPIOB_ARD_D14) |    \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D3) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D4) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D5) |       \
                                     PIN_OSPEED_HIGH(GPIOB_ULPI_D6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOB_ARD_D12) |    \
                                     PIN_OSPEED_VERYLOW(GPIOB_ARD_D11))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_ULPI_D1) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_ULPI_D2) |    \
                                     PIN_PUPDR_PULLUP(GPIOB_QSPI_CLK) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_SD_D2) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_SD_D3) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_ULPI_D7) |      \
                                     PIN_PUPDR_PULLUP(GPIOB_QSPI_NCS) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_AUDIO_SDA) |  \
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
                                     PIN_ODR_HIGH(GPIOB_SD_D2) |            \
                                     PIN_ODR_HIGH(GPIOB_SD_D3) |            \
                                     PIN_ODR_HIGH(GPIOB_ULPI_D7) |          \
                                     PIN_ODR_HIGH(GPIOB_QSPI_NCS) |         \
                                     PIN_ODR_HIGH(GPIOB_AUDIO_SDA) |        \
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
                                     PIN_AFIO_AF(GPIOB_SD_D2, 10U) |        \
                                     PIN_AFIO_AF(GPIOB_SD_D3, 10U) |        \
                                     PIN_AFIO_AF(GPIOB_ULPI_D7, 10U) |      \
                                     PIN_AFIO_AF(GPIOB_QSPI_NCS, 0U) |      \
                                     PIN_AFIO_AF(GPIOB_AUDIO_SDA, 11U))
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
 * PC2  - ARD_A2                    (input pullup).
 * PC3  - DFSDM_DATIN1              (alternate 3).
 * PC4  - RMII_RXD0                 (alternate 11).
 * PC5  - RMII_RXD1                 (alternate 11).
 * PC6  - ARD_D1                    (input pullup).
 * PC7  - ARD_D0                    (input floating).
 * PC8  - ARD_D5                    (input floating).
 * PC9  - QSPI_D0                   (alternate 9).
 * PC10 - QSPI_D1                   (alternate 9).
 * PC11 - DFSDM_DATIN5              (alternate 3).
 * PC12 - WIFI_RX                   (alternate 8).
 * PC13 - PIN13                     (input pullup).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ALTERNATE(GPIOC_ULPI_STP) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_MDC) |   \
                                     PIN_MODE_INPUT(GPIOC_ARD_A2) |         \
                                     PIN_MODE_ALTERNATE(GPIOC_DFSDM_DATIN1) |\
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_RXD0) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_RMII_RXD1) |  \
                                     PIN_MODE_INPUT(GPIOC_ARD_D1) |         \
                                     PIN_MODE_INPUT(GPIOC_ARD_D0) |         \
                                     PIN_MODE_INPUT(GPIOC_ARD_D5) |         \
                                     PIN_MODE_ALTERNATE(GPIOC_QSPI_D0) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_QSPI_D1) |    \
                                     PIN_MODE_ALTERNATE(GPIOC_DFSDM_DATIN5) |\
                                     PIN_MODE_ALTERNATE(GPIOC_WIFI_RX) |    \
                                     PIN_MODE_INPUT(GPIOC_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_ULPI_STP) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_RMII_MDC) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_A2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_DFSDM_DATIN1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_RMII_RXD0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_RMII_RXD1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_D1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ARD_D5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_QSPI_D0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_QSPI_D1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_DFSDM_DATIN5) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_WIFI_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_ULPI_STP) |      \
                                     PIN_OSPEED_HIGH(GPIOC_RMII_MDC) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_ARD_A2) |     \
                                     PIN_OSPEED_HIGH(GPIOC_DFSDM_DATIN1) |  \
                                     PIN_OSPEED_HIGH(GPIOC_RMII_RXD0) |     \
                                     PIN_OSPEED_HIGH(GPIOC_RMII_RXD1) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_ARD_D1) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_ARD_D0) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_ARD_D5) |     \
                                     PIN_OSPEED_HIGH(GPIOC_QSPI_D0) |       \
                                     PIN_OSPEED_HIGH(GPIOC_QSPI_D1) |       \
                                     PIN_OSPEED_HIGH(GPIOC_DFSDM_DATIN5) |  \
                                     PIN_OSPEED_HIGH(GPIOC_WIFI_RX) |       \
                                     PIN_OSPEED_HIGH(GPIOC_PIN13) |         \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_IN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_ULPI_STP) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_MDC) |   \
                                     PIN_PUPDR_PULLUP(GPIOC_ARD_A2) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_DFSDM_DATIN1) |\
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_RXD0) |  \
                                     PIN_PUPDR_FLOATING(GPIOC_RMII_RXD1) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_ARD_D1) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_ARD_D0) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_ARD_D5) |     \
                                     PIN_PUPDR_FLOATING(GPIOC_QSPI_D0) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_QSPI_D1) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_DFSDM_DATIN5) |\
                                     PIN_PUPDR_FLOATING(GPIOC_WIFI_RX) |    \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN13) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_ULPI_STP) |         \
                                     PIN_ODR_HIGH(GPIOC_RMII_MDC) |         \
                                     PIN_ODR_HIGH(GPIOC_ARD_A2) |           \
                                     PIN_ODR_HIGH(GPIOC_DFSDM_DATIN1) |     \
                                     PIN_ODR_HIGH(GPIOC_RMII_RXD0) |        \
                                     PIN_ODR_HIGH(GPIOC_RMII_RXD1) |        \
                                     PIN_ODR_HIGH(GPIOC_ARD_D1) |           \
                                     PIN_ODR_HIGH(GPIOC_ARD_D0) |           \
                                     PIN_ODR_HIGH(GPIOC_ARD_D5) |           \
                                     PIN_ODR_HIGH(GPIOC_QSPI_D0) |          \
                                     PIN_ODR_HIGH(GPIOC_QSPI_D1) |          \
                                     PIN_ODR_HIGH(GPIOC_DFSDM_DATIN5) |     \
                                     PIN_ODR_HIGH(GPIOC_WIFI_RX) |          \
                                     PIN_ODR_HIGH(GPIOC_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_ULPI_STP, 10U) |     \
                                     PIN_AFIO_AF(GPIOC_RMII_MDC, 11U) |     \
                                     PIN_AFIO_AF(GPIOC_ARD_A2, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_DFSDM_DATIN1, 3U) |  \
                                     PIN_AFIO_AF(GPIOC_RMII_RXD0, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_RMII_RXD1, 11U) |    \
                                     PIN_AFIO_AF(GPIOC_ARD_D1, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_ARD_D0, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_ARD_D5, 0U) |        \
                                     PIN_AFIO_AF(GPIOC_QSPI_D0, 9U) |       \
                                     PIN_AFIO_AF(GPIOC_QSPI_D1, 9U) |       \
                                     PIN_AFIO_AF(GPIOC_DFSDM_DATIN5, 3U) |  \
                                     PIN_AFIO_AF(GPIOC_WIFI_RX, 8U) |       \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - FMC_D2                    (alternate 12).
 * PD1  - FMC_D3                    (alternate 12).
 * PD2  - WIFI_TX                   (alternate 8).
 * PD3  - DFSDM_CKOUT               (alternate 3).
 * PD4  - OTG_HS_OVER_CURRENT       (input floating).
 * PD5  - RMII_RXER                 (alternate 7).
 * PD6  - SD_CLK                    (alternate 11).
 * PD7  - SD_CMD                    (alternate 11).
 * PD8  - FMC_D13                   (alternate 12).
 * PD9  - FMC_D14                   (alternate 12).
 * PD10 - FMC_D15                   (alternate 12).
 * PD11 - SPDIF_I2S                 (alternate 10).
 * PD12 - AUDIO_SCL LCD_SCL         (alternate 4).
 * PD13 - QSPI_D3                   (input pullup).
 * PD14 - FMC_D0                    (alternate 12).
 * PD15 - FMC_D1                    (alternate 12).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_FMC_D2) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D3) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_WIFI_TX) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_DFSDM_CKOUT) |\
                                     PIN_MODE_INPUT(GPIOD_OTG_HS_OVER_CURRENT) |\
                                     PIN_MODE_ALTERNATE(GPIOD_RMII_RXER) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_SD_CLK) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_SD_CMD) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D13) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D14) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D15) |    \
                                     PIN_MODE_ALTERNATE(GPIOD_SPDIF_I2S) |  \
                                     PIN_MODE_ALTERNATE(GPIOD_AUDIO_SCL) |  \
                                     PIN_MODE_INPUT(GPIOD_QSPI_D3) |        \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D0) |     \
                                     PIN_MODE_ALTERNATE(GPIOD_FMC_D1))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_FMC_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_WIFI_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_DFSDM_CKOUT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_OTG_HS_OVER_CURRENT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_RMII_RXER) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CLK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CMD) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D13) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D14) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D15) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SPDIF_I2S) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_AUDIO_SCL) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_QSPI_D3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D0) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_FMC_D1))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_FMC_D2) |        \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D3) |        \
                                     PIN_OSPEED_HIGH(GPIOD_WIFI_TX) |       \
                                     PIN_OSPEED_HIGH(GPIOD_DFSDM_CKOUT) |   \
                                     PIN_OSPEED_HIGH(GPIOD_OTG_HS_OVER_CURRENT) |\
                                     PIN_OSPEED_HIGH(GPIOD_RMII_RXER) |     \
                                     PIN_OSPEED_HIGH(GPIOD_SD_CLK) |        \
                                     PIN_OSPEED_HIGH(GPIOD_SD_CMD) |        \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D13) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D14) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D15) |       \
                                     PIN_OSPEED_HIGH(GPIOD_SPDIF_I2S) |     \
                                     PIN_OSPEED_HIGH(GPIOD_AUDIO_SCL) |     \
                                     PIN_OSPEED_HIGH(GPIOD_QSPI_D3) |       \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D0) |        \
                                     PIN_OSPEED_HIGH(GPIOD_FMC_D1))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_FMC_D2) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D3) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_WIFI_TX) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_DFSDM_CKOUT) |\
                                     PIN_PUPDR_FLOATING(GPIOD_OTG_HS_OVER_CURRENT) |\
                                     PIN_PUPDR_FLOATING(GPIOD_RMII_RXER) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_SD_CLK) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_SD_CMD) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D13) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D14) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D15) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_SPDIF_I2S) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_AUDIO_SCL) |  \
                                     PIN_PUPDR_PULLUP(GPIOD_QSPI_D3) |      \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D0) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_FMC_D1))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_FMC_D2) |           \
                                     PIN_ODR_HIGH(GPIOD_FMC_D3) |           \
                                     PIN_ODR_HIGH(GPIOD_WIFI_TX) |          \
                                     PIN_ODR_HIGH(GPIOD_DFSDM_CKOUT) |      \
                                     PIN_ODR_HIGH(GPIOD_OTG_HS_OVER_CURRENT) |\
                                     PIN_ODR_HIGH(GPIOD_RMII_RXER) |        \
                                     PIN_ODR_HIGH(GPIOD_SD_CLK) |           \
                                     PIN_ODR_HIGH(GPIOD_SD_CMD) |           \
                                     PIN_ODR_HIGH(GPIOD_FMC_D13) |          \
                                     PIN_ODR_HIGH(GPIOD_FMC_D14) |          \
                                     PIN_ODR_HIGH(GPIOD_FMC_D15) |          \
                                     PIN_ODR_HIGH(GPIOD_SPDIF_I2S) |        \
                                     PIN_ODR_HIGH(GPIOD_AUDIO_SCL) |        \
                                     PIN_ODR_LOW(GPIOD_QSPI_D3) |           \
                                     PIN_ODR_LOW(GPIOD_FMC_D0) |            \
                                     PIN_ODR_LOW(GPIOD_FMC_D1))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_FMC_D2, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_D3, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_WIFI_TX, 8U) |       \
                                     PIN_AFIO_AF(GPIOD_DFSDM_CKOUT, 3U) |   \
                                     PIN_AFIO_AF(GPIOD_OTG_HS_OVER_CURRENT, 0U) |\
                                     PIN_AFIO_AF(GPIOD_RMII_RXER, 7U) |     \
                                     PIN_AFIO_AF(GPIOD_SD_CLK, 11U) |       \
                                     PIN_AFIO_AF(GPIOD_SD_CMD, 11U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_FMC_D13, 12U) |      \
                                     PIN_AFIO_AF(GPIOD_FMC_D14, 12U) |      \
                                     PIN_AFIO_AF(GPIOD_FMC_D15, 12U) |      \
                                     PIN_AFIO_AF(GPIOD_SPDIF_I2S, 10U) |    \
                                     PIN_AFIO_AF(GPIOD_AUDIO_SCL, 4U) |     \
                                     PIN_AFIO_AF(GPIOD_QSPI_D3, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_D0, 12U) |       \
                                     PIN_AFIO_AF(GPIOD_FMC_D1, 12U))

/*
 * GPIOE setup:
 *
 * PE0  - FMC_NBL0                  (alternate 12).
 * PE1  - FMC_NBL1                  (alternate 12).
 * PE2  - QSPI_D2                   (input pullup).
 * PE3  - SAI1_SDB                  (alternate 6).
 * PE4  - SAI1_FSA                  (alternate 6).
 * PE5  - SAI1_SCKA                 (alternate 6).
 * PE6  - SAI1_SDA                  (alternate 6).
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
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_SDB) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_FSA) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_SCKA) |  \
                                     PIN_MODE_ALTERNATE(GPIOE_SAI1_SDA) |   \
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
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SDB) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_FSA) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SCKA) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SAI1_SDA) |   \
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
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_SDB) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_FSA) |      \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_SCKA) |     \
                                     PIN_OSPEED_HIGH(GPIOE_SAI1_SDA) |      \
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
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_SDB) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_FSA) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_SCKA) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_SAI1_SDA) |   \
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
                                     PIN_ODR_HIGH(GPIOE_SAI1_SDB) |         \
                                     PIN_ODR_HIGH(GPIOE_SAI1_FSA) |         \
                                     PIN_ODR_HIGH(GPIOE_SAI1_SCKA) |        \
                                     PIN_ODR_HIGH(GPIOE_SAI1_SDA) |         \
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
                                     PIN_AFIO_AF(GPIOE_SAI1_SDB, 6U) |      \
                                     PIN_AFIO_AF(GPIOE_SAI1_FSA, 6U) |      \
                                     PIN_AFIO_AF(GPIOE_SAI1_SCKA, 6U) |     \
                                     PIN_AFIO_AF(GPIOE_SAI1_SDA, 6U) |      \
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
 * PF6  - ARD_D3                    (input pullup).
 * PF7  - ARD_D6                    (input pullup).
 * PF8  - ARD_A4                    (input pullup).
 * PF9  - ARD_A5                    (input pullup).
 * PF10 - ARD_A3                    (input pullup).
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
                                     PIN_MODE_INPUT(GPIOF_ARD_D3) |         \
                                     PIN_MODE_INPUT(GPIOF_ARD_D6) |         \
                                     PIN_MODE_INPUT(GPIOF_ARD_A4) |         \
                                     PIN_MODE_INPUT(GPIOF_ARD_A5) |         \
                                     PIN_MODE_INPUT(GPIOF_ARD_A3) |         \
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
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_D3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_D6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_A4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_A5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ARD_A3) |     \
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
                                     PIN_OSPEED_VERYLOW(GPIOF_ARD_D3) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_ARD_D6) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_ARD_A4) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_ARD_A5) |     \
                                     PIN_OSPEED_VERYLOW(GPIOF_ARD_A3) |     \
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
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_D3) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_D6) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_A4) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_A5) |       \
                                     PIN_PUPDR_PULLUP(GPIOF_ARD_A3) |       \
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
                                     PIN_ODR_HIGH(GPIOF_ARD_D3) |           \
                                     PIN_ODR_HIGH(GPIOF_ARD_D6) |           \
                                     PIN_ODR_HIGH(GPIOF_ARD_A4) |           \
                                     PIN_ODR_HIGH(GPIOF_ARD_A5) |           \
                                     PIN_ODR_HIGH(GPIOF_ARD_A3) |           \
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
                                     PIN_AFIO_AF(GPIOF_ARD_D3, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_ARD_D6, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_ARD_A4, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_ARD_A5, 0U) |        \
                                     PIN_AFIO_AF(GPIOF_ARD_A3, 0U) |        \
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
 * PG2  - FMC_A12                   (alternate 12).
 * PG3  - EXT_SCL                   (input pullup).
 * PG4  - FMC_BA0                   (alternate 12).
 * PG5  - FMC_BA1                   (alternate 12).
 * PG6  - EXT_SDA                   (input pullup).
 * PG7  - SAI1_MCLKA                (alternate 6).
 * PG8  - FMC_SDCLK                 (alternate 12).
 * PG9  - SD_D0                     (alternate 11).
 * PG10 - SD_D1                     (alternate 11).
 * PG11 - RMII_TX_EN                (alternate 11).
 * PG12 - SPDIF_RX                  (alternate 7).
 * PG13 - RMII_TXD0                 (alternate 11).
 * PG14 - RMII_TXD1                 (alternate 11).
 * PG15 - FMC_SDNCAS                (alternate 12).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_ALTERNATE(GPIOG_FMC_A10) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_A11) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_A12) |    \
                                     PIN_MODE_INPUT(GPIOG_EXT_SCL) |        \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_BA0) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_BA1) |    \
                                     PIN_MODE_INPUT(GPIOG_EXT_SDA) |        \
                                     PIN_MODE_ALTERNATE(GPIOG_SAI1_MCLKA) | \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_SDCLK) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_SD_D0) |      \
                                     PIN_MODE_ALTERNATE(GPIOG_SD_D1) |      \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TX_EN) | \
                                     PIN_MODE_ALTERNATE(GPIOG_SPDIF_RX) |   \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TXD0) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_RMII_TXD1) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_FMC_A10) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_A11) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_A12) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_EXT_SCL) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_BA0) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_BA1) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_EXT_SDA) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SAI1_MCLKA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_SDCLK) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SD_D0) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SD_D1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TX_EN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_SPDIF_RX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TXD0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_RMII_TXD1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_FMC_A10) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_A11) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_A12) |       \
                                     PIN_OSPEED_HIGH(GPIOG_EXT_SCL) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_BA0) |       \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_BA1) |       \
                                     PIN_OSPEED_HIGH(GPIOG_EXT_SDA) |       \
                                     PIN_OSPEED_HIGH(GPIOG_SAI1_MCLKA) |    \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_SDCLK) |     \
                                     PIN_OSPEED_HIGH(GPIOG_SD_D0) |         \
                                     PIN_OSPEED_HIGH(GPIOG_SD_D1) |         \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TX_EN) |    \
                                     PIN_OSPEED_HIGH(GPIOG_SPDIF_RX) |      \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TXD0) |     \
                                     PIN_OSPEED_HIGH(GPIOG_RMII_TXD1) |     \
                                     PIN_OSPEED_HIGH(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_FMC_A10) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_A11) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_A12) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_EXT_SCL) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_BA0) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_BA1) |    \
                                     PIN_PUPDR_PULLUP(GPIOG_EXT_SDA) |      \
                                     PIN_PUPDR_PULLUP(GPIOG_SAI1_MCLKA) |   \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_SDCLK) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_SD_D0) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_SD_D1) |      \
                                     PIN_PUPDR_FLOATING(GPIOG_RMII_TX_EN) | \
                                     PIN_PUPDR_FLOATING(GPIOG_SPDIF_RX) |   \
                                     PIN_PUPDR_FLOATING(GPIOG_RMII_TXD0) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_RMII_TXD1) |  \
                                     PIN_PUPDR_FLOATING(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_FMC_A10) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_A11) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_A12) |          \
                                     PIN_ODR_HIGH(GPIOG_EXT_SCL) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_BA0) |          \
                                     PIN_ODR_HIGH(GPIOG_FMC_BA1) |          \
                                     PIN_ODR_HIGH(GPIOG_EXT_SDA) |          \
                                     PIN_ODR_HIGH(GPIOG_SAI1_MCLKA) |       \
                                     PIN_ODR_HIGH(GPIOG_FMC_SDCLK) |        \
                                     PIN_ODR_HIGH(GPIOG_SD_D0) |            \
                                     PIN_ODR_HIGH(GPIOG_SD_D1) |            \
                                     PIN_ODR_HIGH(GPIOG_RMII_TX_EN) |       \
                                     PIN_ODR_HIGH(GPIOG_SPDIF_RX) |         \
                                     PIN_ODR_HIGH(GPIOG_RMII_TXD0) |        \
                                     PIN_ODR_HIGH(GPIOG_RMII_TXD1) |        \
                                     PIN_ODR_HIGH(GPIOG_FMC_SDNCAS))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_FMC_A10, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_FMC_A11, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_FMC_A12, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_EXT_SCL, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_FMC_BA0, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_FMC_BA1, 12U) |      \
                                     PIN_AFIO_AF(GPIOG_EXT_SDA, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_SAI1_MCLKA, 6U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_FMC_SDCLK, 12U) |    \
                                     PIN_AFIO_AF(GPIOG_SD_D0, 11U) |        \
                                     PIN_AFIO_AF(GPIOG_SD_D1, 11U) |        \
                                     PIN_AFIO_AF(GPIOG_RMII_TX_EN, 11U) |   \
                                     PIN_AFIO_AF(GPIOG_SPDIF_RX, 7U) |      \
                                     PIN_AFIO_AF(GPIOG_RMII_TXD0, 11U) |    \
                                     PIN_AFIO_AF(GPIOG_RMII_TXD1, 11U) |    \
                                     PIN_AFIO_AF(GPIOG_FMC_SDNCAS, 12U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - FMC_SDCKE0                (alternate 12).
 * PH3  - FMC_SDNE0                 (alternate 12).
 * PH4  - ULPI_NXT                  (alternate 10).
 * PH5  - FMC_SDNWE                 (alternate 12).
 * PH6  - ARD_D9                    (input pullup).
 * PH7  - EXT_RST                   (input floating).
 * PH8  - FMC_D16                   (alternate 12).
 * PH9  - FMC_D17                   (alternate 12).
 * PH10 - FMC_D18                   (alternate 12).
 * PH11 - FMC_D19                   (alternate 12).
 * PH12 - FMC_D20                   (alternate 12).
 * PH13 - FMC_D21                   (alternate 12).
 * PH14 - FMC_D22                   (alternate 12).
 * PH15 - FMC_D23                   (alternate 12).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_SDCKE0) | \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_SDNE0) |  \
                                     PIN_MODE_ALTERNATE(GPIOH_ULPI_NXT) |   \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_SDNWE) |  \
                                     PIN_MODE_INPUT(GPIOH_ARD_D9) |         \
                                     PIN_MODE_INPUT(GPIOH_EXT_RST) |        \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_D16) |    \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_D17) |    \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_D18) |    \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_D19) |    \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_D20) |    \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_D21) |    \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_D22) |    \
                                     PIN_MODE_ALTERNATE(GPIOH_FMC_D23))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_SDCKE0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_SDNE0) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_ULPI_NXT) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_SDNWE) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOH_ARD_D9) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOH_EXT_RST) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_D16) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_D17) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_D18) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_D19) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_D20) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_D21) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_D22) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_FMC_D23))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_SDCKE0) |    \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_SDNE0) |     \
                                     PIN_OSPEED_HIGH(GPIOH_ULPI_NXT) |      \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_SDNWE) |     \
                                     PIN_OSPEED_VERYLOW(GPIOH_ARD_D9) |     \
                                     PIN_OSPEED_HIGH(GPIOH_EXT_RST) |       \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_D16) |       \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_D17) |       \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_D18) |       \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_D19) |       \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_D20) |       \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_D21) |       \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_D22) |       \
                                     PIN_OSPEED_HIGH(GPIOH_FMC_D23))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_SDCKE0) | \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_SDNE0) |  \
                                     PIN_PUPDR_FLOATING(GPIOH_ULPI_NXT) |   \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_SDNWE) |  \
                                     PIN_PUPDR_PULLUP(GPIOH_ARD_D9) |       \
                                     PIN_PUPDR_FLOATING(GPIOH_EXT_RST) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_D16) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_D17) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_D18) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_D19) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_D20) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_D21) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_D22) |    \
                                     PIN_PUPDR_FLOATING(GPIOH_FMC_D23))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_FMC_SDCKE0) |       \
                                     PIN_ODR_HIGH(GPIOH_FMC_SDNE0) |        \
                                     PIN_ODR_HIGH(GPIOH_ULPI_NXT) |         \
                                     PIN_ODR_HIGH(GPIOH_FMC_SDNWE) |        \
                                     PIN_ODR_HIGH(GPIOH_ARD_D9) |           \
                                     PIN_ODR_HIGH(GPIOH_EXT_RST) |          \
                                     PIN_ODR_HIGH(GPIOH_FMC_D16) |          \
                                     PIN_ODR_HIGH(GPIOH_FMC_D17) |          \
                                     PIN_ODR_HIGH(GPIOH_FMC_D18) |          \
                                     PIN_ODR_HIGH(GPIOH_FMC_D19) |          \
                                     PIN_ODR_HIGH(GPIOH_FMC_D20) |          \
                                     PIN_ODR_HIGH(GPIOH_FMC_D21) |          \
                                     PIN_ODR_HIGH(GPIOH_FMC_D22) |          \
                                     PIN_ODR_HIGH(GPIOH_FMC_D23))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_FMC_SDCKE0, 12U) |   \
                                     PIN_AFIO_AF(GPIOH_FMC_SDNE0, 12U) |    \
                                     PIN_AFIO_AF(GPIOH_ULPI_NXT, 10U) |     \
                                     PIN_AFIO_AF(GPIOH_FMC_SDNWE, 12U) |    \
                                     PIN_AFIO_AF(GPIOH_ARD_D9, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_EXT_RST, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_FMC_D16, 12U) |      \
                                     PIN_AFIO_AF(GPIOH_FMC_D17, 12U) |      \
                                     PIN_AFIO_AF(GPIOH_FMC_D18, 12U) |      \
                                     PIN_AFIO_AF(GPIOH_FMC_D19, 12U) |      \
                                     PIN_AFIO_AF(GPIOH_FMC_D20, 12U) |      \
                                     PIN_AFIO_AF(GPIOH_FMC_D21, 12U) |      \
                                     PIN_AFIO_AF(GPIOH_FMC_D22, 12U) |      \
                                     PIN_AFIO_AF(GPIOH_FMC_D23, 12U))

/*
 * GPIOI setup:
 *
 * PI0  - FMC_D24                   (alternate 12).
 * PI1  - FMC_D25                   (alternate 12).
 * PI2  - FMC_D26                   (alternate 12).
 * PI3  - FMC_D27                   (alternate 12).
 * PI4  - FMC_NBL2                  (alternate 12).
 * PI5  - FMC_NBL3                  (alternate 12).
 * PI6  - FMC_D28                   (alternate 12).
 * PI7  - FMC_D29                   (alternate 12).
 * PI8  - PIN8                      (input pullup).
 * PI9  - FMC_D30                   (alternate 12).
 * PI10 - FMC_D31                   (alternate 12).
 * PI11 - ULPI_DIR                  (alternate 10).
 * PI12 - PIN12                     (input pullup).
 * PI13 - LCD_INT                   (alternate 14).
 * PI14 - LCD_BL_CTRL               (alternate 14).
 * PI15 - SD_DETECT                 (input pullup).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_ALTERNATE(GPIOI_FMC_D24) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_D25) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_D26) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_D27) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_NBL2) |   \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_NBL3) |   \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_D28) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_D29) |    \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_D30) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_FMC_D31) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_ULPI_DIR) |   \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_INT) |    \
                                     PIN_MODE_ALTERNATE(GPIOI_LCD_BL_CTRL) |\
                                     PIN_MODE_INPUT(GPIOI_SD_DETECT))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_FMC_D24) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_D25) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_D26) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_D27) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_NBL2) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_NBL3) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_D28) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_D29) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_D30) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_FMC_D31) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_ULPI_DIR) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_INT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOI_LCD_BL_CTRL) |\
                                     PIN_OTYPE_PUSHPULL(GPIOI_SD_DETECT))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_HIGH(GPIOI_FMC_D24) |       \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_D25) |       \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_D26) |       \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_D27) |       \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_NBL2) |      \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_NBL3) |      \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_D28) |       \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_D29) |       \
                                     PIN_OSPEED_HIGH(GPIOI_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_D30) |       \
                                     PIN_OSPEED_HIGH(GPIOI_FMC_D31) |       \
                                     PIN_OSPEED_HIGH(GPIOI_ULPI_DIR) |      \
                                     PIN_OSPEED_HIGH(GPIOI_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_INT) |       \
                                     PIN_OSPEED_HIGH(GPIOI_LCD_BL_CTRL) |   \
                                     PIN_OSPEED_HIGH(GPIOI_SD_DETECT))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_FLOATING(GPIOI_FMC_D24) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_D25) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_D26) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_D27) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_NBL2) |   \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_NBL3) |   \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_D28) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_D29) |    \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_D30) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_FMC_D31) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_ULPI_DIR) |   \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN12) |        \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_INT) |    \
                                     PIN_PUPDR_FLOATING(GPIOI_LCD_BL_CTRL) |\
                                     PIN_PUPDR_PULLUP(GPIOI_SD_DETECT))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_FMC_D24) |          \
                                     PIN_ODR_HIGH(GPIOI_FMC_D25) |          \
                                     PIN_ODR_HIGH(GPIOI_FMC_D26) |          \
                                     PIN_ODR_HIGH(GPIOI_FMC_D27) |          \
                                     PIN_ODR_HIGH(GPIOI_FMC_NBL2) |         \
                                     PIN_ODR_HIGH(GPIOI_FMC_NBL3) |         \
                                     PIN_ODR_HIGH(GPIOI_FMC_D28) |          \
                                     PIN_ODR_HIGH(GPIOI_FMC_D29) |          \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_FMC_D30) |          \
                                     PIN_ODR_HIGH(GPIOI_FMC_D31) |          \
                                     PIN_ODR_HIGH(GPIOI_ULPI_DIR) |         \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOI_LCD_INT) |          \
                                     PIN_ODR_HIGH(GPIOI_LCD_BL_CTRL) |      \
                                     PIN_ODR_HIGH(GPIOI_SD_DETECT))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_FMC_D24, 12U) |      \
                                     PIN_AFIO_AF(GPIOI_FMC_D25, 12U) |      \
                                     PIN_AFIO_AF(GPIOI_FMC_D26, 12U) |      \
                                     PIN_AFIO_AF(GPIOI_FMC_D27, 12U) |      \
                                     PIN_AFIO_AF(GPIOI_FMC_NBL2, 12U) |     \
                                     PIN_AFIO_AF(GPIOI_FMC_NBL3, 12U) |     \
                                     PIN_AFIO_AF(GPIOI_FMC_D28, 12U) |      \
                                     PIN_AFIO_AF(GPIOI_FMC_D29, 12U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_FMC_D30, 12U) |      \
                                     PIN_AFIO_AF(GPIOI_FMC_D31, 12U) |      \
                                     PIN_AFIO_AF(GPIOI_ULPI_DIR, 10U) |     \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_LCD_INT, 14U) |      \
                                     PIN_AFIO_AF(GPIOI_LCD_BL_CTRL, 14U) |  \
                                     PIN_AFIO_AF(GPIOI_SD_DETECT, 0U))

/*
 * GPIOJ setup:
 *
 * PJ0  - ARD_D4                    (input floating).
 * PJ1  - ARD_D2                    (input floating).
 * PJ2  - DSI_TE                    (alternate 13).
 * PJ3  - ARD_D7                    (input floating).
 * PJ4  - ARD_D8                    (input floating).
 * PJ5  - LED2_GREEN                (output pushpull maximum).
 * PJ6  - PIN6                      (input floating).
 * PJ7  - PIN7                      (input floating).
 * PJ8  - PIN8                      (input floating).
 * PJ9  - PIN9                      (input floating).
 * PJ10 - PIN10                     (input floating).
 * PJ11 - PIN11                     (input floating).
 * PJ12 - AUDIO_INT                 (input floating).
 * PJ13 - LED1_RED                  (output pushpull maximum).
 * PJ14 - WIFI_RST                  (input floating).
 * PJ15 - DSI_RESET                 (input floating).
 */
#define VAL_GPIOJ_MODER             (PIN_MODE_INPUT(GPIOJ_ARD_D4) |         \
                                     PIN_MODE_INPUT(GPIOJ_ARD_D2) |         \
                                     PIN_MODE_ALTERNATE(GPIOJ_DSI_TE) |     \
                                     PIN_MODE_INPUT(GPIOJ_ARD_D7) |         \
                                     PIN_MODE_INPUT(GPIOJ_ARD_D8) |         \
                                     PIN_MODE_OUTPUT(GPIOJ_LED2_GREEN) |    \
                                     PIN_MODE_INPUT(GPIOJ_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOJ_AUDIO_INT) |      \
                                     PIN_MODE_OUTPUT(GPIOJ_LED1_RED) |      \
                                     PIN_MODE_INPUT(GPIOJ_WIFI_RST) |       \
                                     PIN_MODE_INPUT(GPIOJ_DSI_RESET))
#define VAL_GPIOJ_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOJ_ARD_D4) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_ARD_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_DSI_TE) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_ARD_D7) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_ARD_D8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LED2_GREEN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_AUDIO_INT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_LED1_RED) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_WIFI_RST) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_DSI_RESET))
#define VAL_GPIOJ_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOJ_ARD_D4) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_ARD_D2) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_DSI_TE) |        \
                                     PIN_OSPEED_VERYLOW(GPIOJ_ARD_D7) |     \
                                     PIN_OSPEED_VERYLOW(GPIOJ_ARD_D8) |     \
                                     PIN_OSPEED_HIGH(GPIOJ_LED2_GREEN) |    \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_AUDIO_INT) |  \
                                     PIN_OSPEED_HIGH(GPIOJ_LED1_RED) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_WIFI_RST) |   \
                                     PIN_OSPEED_VERYLOW(GPIOJ_DSI_RESET))
#define VAL_GPIOJ_PUPDR             (PIN_PUPDR_FLOATING(GPIOJ_ARD_D4) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_ARD_D2) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_DSI_TE) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_ARD_D7) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_ARD_D8) |     \
                                     PIN_PUPDR_FLOATING(GPIOJ_LED2_GREEN) | \
                                     PIN_PUPDR_FLOATING(GPIOJ_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOJ_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOJ_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOJ_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOJ_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOJ_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOJ_AUDIO_INT) |  \
                                     PIN_PUPDR_FLOATING(GPIOJ_LED1_RED) |   \
                                     PIN_PUPDR_FLOATING(GPIOJ_WIFI_RST) |   \
                                     PIN_PUPDR_FLOATING(GPIOJ_DSI_RESET))
#define VAL_GPIOJ_ODR               (PIN_ODR_HIGH(GPIOJ_ARD_D4) |           \
                                     PIN_ODR_HIGH(GPIOJ_ARD_D2) |           \
                                     PIN_ODR_HIGH(GPIOJ_DSI_TE) |           \
                                     PIN_ODR_HIGH(GPIOJ_ARD_D7) |           \
                                     PIN_ODR_HIGH(GPIOJ_ARD_D8) |           \
                                     PIN_ODR_LOW(GPIOJ_LED2_GREEN) |        \
                                     PIN_ODR_HIGH(GPIOJ_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOJ_AUDIO_INT) |        \
                                     PIN_ODR_LOW(GPIOJ_LED1_RED) |          \
                                     PIN_ODR_HIGH(GPIOJ_WIFI_RST) |         \
                                     PIN_ODR_HIGH(GPIOJ_DSI_RESET))
#define VAL_GPIOJ_AFRL              (PIN_AFIO_AF(GPIOJ_ARD_D4, 0U) |        \
                                     PIN_AFIO_AF(GPIOJ_ARD_D2, 0U) |        \
                                     PIN_AFIO_AF(GPIOJ_DSI_TE, 13U) |       \
                                     PIN_AFIO_AF(GPIOJ_ARD_D7, 0U) |        \
                                     PIN_AFIO_AF(GPIOJ_ARD_D8, 0U) |        \
                                     PIN_AFIO_AF(GPIOJ_LED2_GREEN, 0U) |    \
                                     PIN_AFIO_AF(GPIOJ_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN7, 0U))
#define VAL_GPIOJ_AFRH              (PIN_AFIO_AF(GPIOJ_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_AUDIO_INT, 0U) |     \
                                     PIN_AFIO_AF(GPIOJ_LED1_RED, 0U) |      \
                                     PIN_AFIO_AF(GPIOJ_WIFI_RST, 0U) |      \
                                     PIN_AFIO_AF(GPIOJ_DSI_RESET, 0U))

/*
 * GPIOK setup:
 *
 * PK0  - PIN0                      (input floating).
 * PK1  - PIN1                      (input floating).
 * PK2  - PIN2                      (input floating).
 * PK3  - PIN3                      (input floating).
 * PK4  - PIN4                      (input floating).
 * PK5  - PIN5                      (input floating).
 * PK6  - PIN6                      (input floating).
 * PK7  - PIN7                      (input floating).
 * PK8  - PIN8                      (input floating).
 * PK9  - PIN9                      (input floating).
 * PK10 - PIN10                     (input floating).
 * PK11 - PIN11                     (input floating).
 * PK12 - PIN12                     (input floating).
 * PK13 - PIN13                     (input floating).
 * PK14 - PIN14                     (input floating).
 * PK15 - PIN15                     (input floating).
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
#define VAL_GPIOK_PUPDR             (PIN_PUPDR_FLOATING(GPIOK_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOK_PIN15))
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
