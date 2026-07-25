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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for Olimex STM32-P407 board.
 * NOTE: Part of JTAG signals are used for other functions, this board can be
 *       used using SWD only.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_STM32_P407
#define BOARD_NAME              "Olimex STM32-P407"

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID            MII_KS8721_ID
#define BOARD_PHY_RMII

/*
 * Board frequencies.
 * NOTE: The LSE crystal is not fitted by default on the board.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            25000000

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD               330

/*
 * MCU type as defined in the ST header.
 */
#define STM32F407xx

/*
 * IO pins assignments.
 */
#define GPIOA_BUTTON_WKUP       0
#define GPIOA_ETH_RMII_REF_CLK  1
#define GPIOA_ETH_RMII_MDIO     2
#define GPIOA_ETH_RMII_MDINT    3
#define GPIOA_DCMI_HSYNC        4
#define GPIOA_LCD_SCK           5
#define GPIOA_DCMI_PIXCLK       6
#define GPIOA_ETH_RMII_CRS_DV   7
#define GPIOA_MCO1              8
#define GPIOA_OTG_FS_VBUS       9
#define GPIOA_DCMI_D1           10
#define GPIOA_OTG_FS_DM         11
#define GPIOA_OTG_FS_DP         12
#define GPIOA_SWDIO             13
#define GPIOA_SWCLK             14
#define GPIOA_I2S3_WS           15

#define GPIOB_LCD_BL            0
#define GPIOB_BUZ               1
#define GPIOB_CAM_ENB           2
#define GPIOB_I2S3_CK           3
#define GPIOB_LCD_MISO          4
#define GPIOB_I2S3_SD           5
#define GPIOB_DCMI_D5           6
#define GPIOB_DCMI_VSYNC        7
#define GPIOB_CAN1_RX           8
#define GPIOB_CAN1_TX           9
#define GPIOB_USB_FS_FAULT      10
#define GPIOB_ETH_RMII_TX_EN    11
#define GPIOB_OTG_HS_ID         12
#define GPIOB_OTG_HS_VBUS       13
#define GPIOB_OTG_HS_DM         14
#define GPIOB_OTG_HS_DP         15

#define GPIOC_TRIM              0
#define GPIOC_ETH_RMII_MDC      1
#define GPIOC_USB_FS_VBUSON     2
#define GPIOC_LCD_MOSI          3
#define GPIOC_ETH_RMII_RXD0     4
#define GPIOC_ETH_RMII_RXD1     5
#define GPIOC_DCMI_D0_US6_TX    6
#define GPIOC_I2S3_MCK          7
#define GPIOC_DCMI_D2           8
#define GPIOC_DCMI_D3           9
#define GPIOC_SPI3_SCK          10
#define GPIOC_SPI3_MISO         11
#define GPIOC_SPI3_MOSI         12
#define GPIOC_SWITCH_TAMPER     13
#define GPIOC_OSC32_IN          14
#define GPIOC_OSC32_OUT         15

#define GPIOD_USELESS0          0
#define GPIOD_USELESS1          1
#define GPIOD_SPI3_CS           2
#define GPIOD_LCD_RST           3
#define GPIOD_USELESS4          4
#define GPIOD_USELESS5          5
#define GPIOD_LCD_CS            6
#define GPIOD_USELESS7          7
#define GPIOD_USART3_TX         8
#define GPIOD_USART3_RX         9
#define GPIOD_USELESS10         10
#define GPIOD_USART3_CTS        11
#define GPIOD_USART3_RTS        12
#define GPIOD_USB_HS_FAULT      13
#define GPIOD_USELESS14         14
#define GPIOD_USELESS15         15

#define GPIOE_0                 0
#define GPIOE_1                 1
#define GPIOE_TEMP_ALERT        2
#define GPIOE_USB_HS_VBUSON     3
#define GPIOE_4                 4
#define GPIOE_5                 5
#define GPIOE_6                 6
#define GPIOE_7                 7
#define GPIOE_8                 8
#define GPIOE_9                 9
#define GPIOE_10                10
#define GPIOE_11                11
#define GPIOE_12                12
#define GPIOE_13                13
#define GPIOE_14                14
#define GPIOE_15                15

#define GPIOF_0                 0
#define GPIOF_1                 1
#define GPIOF_2                 2
#define GPIOF_3                 3
#define GPIOF_4                 4
#define GPIOF_5                 5
#define GPIOF_STAT1             6
#define GPIOF_STAT2             7
#define GPIOF_STAT3             8
#define GPIOF_CAM_PWR           9
#define GPIOF_10                10
#define GPIOF_CAM_RS            11
#define GPIOF_12                12
#define GPIOF_13                13
#define GPIOF_14                14
#define GPIOF_15                15

#define GPIOG_0                 0
#define GPIOG_1                 1
#define GPIOG_2                 2
#define GPIOG_3                 3
#define GPIOG_4                 4
#define GPIOG_5                 5
#define GPIOG_RIGHT             6
#define GPIOG_UP                7
#define GPIOG_DOWN              8
#define GPIOG_USART6_RX         9
#define GPIOG_10                10
#define GPIOG_LEFT              11
#define GPIOG_12                12
#define GPIOG_ETH_RMII_TXD0     13
#define GPIOG_ETH_RMII_TXD1     14
#define GPIOG_CENT              15

#define GPIOH_OSC_IN            0
#define GPIOH_OSC_OUT           1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUDR_FLOATING(n)        (0U << ((n) * 2))
#define PIN_PUDR_PULLUP(n)          (1U << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)        (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << (((n) % 8) * 4))

/*
 * Port A setup.
 *
 * PA0  - GPIOA_BUTTON_WKUP     (input floating).
 * PA1  - GPIOA_ETH_RMII_REF_CLK(alternate 11).
 * PA2  - GPIOA_ETH_RMII_MDIO   (alternate 11).
 * PA3  - GPIOA_ETH_RMII_MDINT  (input floating).
 * PA4  - GPIOA_DCMI_HSYNC      (input pull-up).
 * PA5  - GPIOA_LCD_SCK         (output push-pull).
 * PA6  - GPIOA_DCMI_PIXCLK     (input pull-up).
 * PA7  - GPIOA_ETH_RMII_CRS_DV (alternate 11).
 * PA8  - GPIOA_MCO1            (alternate 0).
 * PA9  - GPIOA_OTG_FS_VBUS     (input pull-up).
 * PA10 - GPIOA_DCMI_D1         (input pull-up).
 * PA11 - GPIOA_OTG_FS_DM       (alternate 10).
 * PA12 - GPIOA_OTG_FS_DP       (alternate 10).
 * PA13 - GPIOA_SWDIO           (alternate 0).
 * PA14 - GPIOA_SWCLK           (alternate 0, pull-down).
 * PA15 - GPIOA_I2S3_WS         (alternate 6).
 */
#define VAL_GPIOA_MODER     (PIN_MODE_INPUT(GPIOA_BUTTON_WKUP) |            \
                             PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_REF_CLK) |   \
                             PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_MDIO) |      \
                             PIN_MODE_INPUT(GPIOA_ETH_RMII_MDINT) |         \
                             PIN_MODE_INPUT(GPIOA_DCMI_HSYNC) |             \
                             PIN_MODE_OUTPUT(GPIOA_LCD_SCK) |               \
                             PIN_MODE_INPUT(GPIOA_DCMI_PIXCLK) |            \
                             PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_CRS_DV) |    \
                             PIN_MODE_ALTERNATE(GPIOA_MCO1) |               \
                             PIN_MODE_INPUT(GPIOA_OTG_FS_VBUS) |            \
                             PIN_MODE_INPUT(GPIOA_DCMI_D1) |                \
                             PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |          \
                             PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |          \
                             PIN_MODE_ALTERNATE(GPIOA_SWDIO) |              \
                             PIN_MODE_ALTERNATE(GPIOA_SWCLK) |              \
                             PIN_MODE_ALTERNATE(GPIOA_I2S3_WS))
#define VAL_GPIOA_OTYPER    0x00000000
#define VAL_GPIOA_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOA_PUPDR     (PIN_PUDR_PULLUP(GPIOA_DCMI_HSYNC) |            \
                             PIN_PUDR_PULLUP(GPIOA_DCMI_PIXCLK) |           \
                             PIN_PUDR_PULLDOWN(GPIOA_OTG_FS_VBUS) |         \
                             PIN_PUDR_PULLUP(GPIOA_DCMI_D1) |               \
                             PIN_PUDR_PULLDOWN(GPIOA_SWCLK))
#define VAL_GPIOA_ODR       0xFFFFFFDF
#define VAL_GPIOA_AFRL      (PIN_AFIO_AF(GPIOA_ETH_RMII_REF_CLK, 11) |      \
                             PIN_AFIO_AF(GPIOA_ETH_RMII_MDIO, 11) |         \
                             PIN_AFIO_AF(GPIOA_ETH_RMII_CRS_DV, 11))
#define VAL_GPIOA_AFRH      (PIN_AFIO_AF(GPIOA_MCO1, 0) |                   \
                             PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) |             \
                             PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) |             \
                             PIN_AFIO_AF(GPIOA_SWDIO, 0) |                  \
                             PIN_AFIO_AF(GPIOA_SWCLK, 0) |                  \
                             PIN_AFIO_AF(GPIOA_I2S3_WS, 6))

/*
 * Port B setup.
 *
 * PB0  - GPIOB_LCD_BL          (output push-pull).
 * PB1  - GPIOB_BUZ             (output push-pull).
 * PB2  - GPIOB_CAM_ENB         (input floating).
 * PB3  - GPIOB_I2S3_CK         (alternate 6).
 * PB4  - GPIOB_LCD_MISO        (input floating).
 * PB5  - GPIOB_I2S3_SD         (alternate 6).
 * PB6  - GPIOB_DCMI_D5         (input pull-up).
 * PB7  - GPIOB_DCMI_VSYNC      (input pull-up).
 * PB8  - GPIOB_CAN1_RX         (alternate 9).
 * PB9  - GPIOB_CAN1_TX         (alternate 9).
 * PB10 - GPIOB_USB_FS_FAULT    (input floating).
 * PB11 - GPIOB_ETH_RMII_TX_EN  (alternate 11).
 * PB12 - GPIOB_OTG_HS_ID       (alternate 12).
 * PB13 - GPIOB_OTG_HS_VBUS     (input pull-up).
 * PB14 - GPIOB_OTG_HS_DM       (alternate 12).
 * PB15 - GPIOB_OTG_HS_DP       (alternate 12).
 */
#define VAL_GPIOB_MODER     (PIN_MODE_OUTPUT(GPIOB_LCD_BL) |                \
                             PIN_MODE_OUTPUT(GPIOB_BUZ) |                   \
                             PIN_MODE_INPUT(GPIOB_CAM_ENB) |                \
                             PIN_MODE_ALTERNATE(GPIOB_I2S3_CK) |            \
                             PIN_MODE_INPUT(GPIOB_LCD_MISO) |               \
                             PIN_MODE_ALTERNATE(GPIOB_I2S3_SD) |            \
                             PIN_MODE_INPUT(GPIOB_DCMI_D5) |                \
                             PIN_MODE_INPUT(GPIOB_DCMI_VSYNC) |             \
                             PIN_MODE_ALTERNATE(GPIOB_CAN1_RX) |            \
                             PIN_MODE_ALTERNATE(GPIOB_CAN1_TX) |            \
                             PIN_MODE_INPUT(GPIOB_USB_FS_FAULT) |           \
                             PIN_MODE_ALTERNATE(GPIOB_ETH_RMII_TX_EN) |     \
                             PIN_MODE_ALTERNATE(GPIOB_OTG_HS_ID) |          \
                             PIN_MODE_INPUT(GPIOB_OTG_HS_VBUS) |            \
                             PIN_MODE_ALTERNATE(GPIOB_OTG_HS_DM) |          \
                             PIN_MODE_ALTERNATE(GPIOB_OTG_HS_DP))
#define VAL_GPIOB_OTYPER    0x00000000
#define VAL_GPIOB_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOB_PUPDR     (PIN_PUDR_PULLUP(GPIOB_DCMI_D5) |               \
                             PIN_PUDR_PULLUP(GPIOB_DCMI_VSYNC) |            \
                             PIN_PUDR_PULLDOWN(GPIOB_OTG_HS_VBUS))
#define VAL_GPIOB_ODR       0xFFFFFFFC
#define VAL_GPIOB_AFRL      (PIN_AFIO_AF(GPIOB_I2S3_CK, 6) |                \
                             PIN_AFIO_AF(GPIOB_I2S3_SD, 6))
#define VAL_GPIOB_AFRH      (PIN_AFIO_AF(GPIOB_CAN1_RX, 9) |                \
                             PIN_AFIO_AF(GPIOB_CAN1_TX, 9) |                \
                             PIN_AFIO_AF(GPIOB_ETH_RMII_TX_EN, 11) |        \
                             PIN_AFIO_AF(GPIOB_OTG_HS_ID, 12) |             \
                             PIN_AFIO_AF(GPIOB_OTG_HS_DM, 12) |             \
                             PIN_AFIO_AF(GPIOB_OTG_HS_DP, 12))

/*
 * Port C setup.
 *
 * PC0  - GPIOC_TRIM            (input floating).
 * PC1  - GPIOC_ETH_RMII_MDC    (alternate 11).
 * PC2  - GPIOC_USB_FS_VBUSON   (output push-pull).
 * PC3  - GPIOC_LCD_MOSI        (output push-pull).
 * PC4  - GPIOC_ETH_RMII_RXD0   (alternate 11).
 * PC5  - GPIOC_ETH_RMII_RXD1   (alternate 11).
 * PC6  - GPIOC_DCMI_D0_US6_TX  (alternate 8).
 * PC7  - GPIOC_I2S3_MCK        (alternate 6).
 * PC8  - GPIOC_DCMI_D2         (input pull-up).
 * PC9  - GPIOC_DCMI_D3         (input pull-up).
 * PC10 - GPIOC_SPI3_SCK        (alternate 6).
 * PC11 - GPIOC_SPI3_MISO       (alternate 6).
 * PC12 - GPIOC_SPI3_MOSI       (alternate 6).
 * PC13 - GPIOC_SWITCH_TAMPER   (input floating).
 * PC14 - GPIOC_OSC32_IN        (input floating).
 * PC15 - GPIOC_OSC32_OUT       (input floating).
 */
#define VAL_GPIOC_MODER     (PIN_MODE_INPUT(GPIOC_TRIM) |                   \
                             PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_MDC) |       \
                             PIN_MODE_OUTPUT(GPIOC_USB_FS_VBUSON) |         \
                             PIN_MODE_OUTPUT(GPIOC_LCD_MOSI) |              \
                             PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_RXD0) |      \
                             PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_RXD1) |      \
                             PIN_MODE_ALTERNATE(GPIOC_DCMI_D0_US6_TX) |     \
                             PIN_MODE_ALTERNATE(GPIOC_I2S3_MCK) |           \
                             PIN_MODE_INPUT(GPIOC_DCMI_D2) |                \
                             PIN_MODE_INPUT(GPIOC_DCMI_D3) |                \
                             PIN_MODE_ALTERNATE(GPIOC_SPI3_SCK) |           \
                             PIN_MODE_ALTERNATE(GPIOC_SPI3_MISO) |          \
                             PIN_MODE_ALTERNATE(GPIOC_SPI3_MOSI) |          \
                             PIN_MODE_INPUT(GPIOC_SWITCH_TAMPER) |          \
                             PIN_MODE_INPUT(GPIOC_OSC32_IN) |               \
                             PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER    0x00000000
#define VAL_GPIOC_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOC_PUPDR     (PIN_PUDR_PULLUP(GPIOC_DCMI_D2) |               \
                             PIN_PUDR_PULLUP(GPIOC_DCMI_D3))
#define VAL_GPIOC_ODR       0xFFFFFFF3
#define VAL_GPIOC_AFRL      (PIN_AFIO_AF(GPIOC_ETH_RMII_MDC, 11) |          \
                             PIN_AFIO_AF(GPIOC_ETH_RMII_RXD0, 11) |         \
                             PIN_AFIO_AF(GPIOC_ETH_RMII_RXD1, 11) |         \
                             PIN_AFIO_AF(GPIOC_DCMI_D0_US6_TX, 8) |         \
                             PIN_AFIO_AF(GPIOC_I2S3_MCK, 6))
#define VAL_GPIOC_AFRH      (PIN_AFIO_AF(GPIOC_SPI3_SCK, 6) |               \
                             PIN_AFIO_AF(GPIOC_SPI3_MISO, 6) |              \
                             PIN_AFIO_AF(GPIOC_SPI3_MOSI, 6))

/*
 * Port D setup.
 *
 * PD0  - GPIOD_USELESS0        (input pull-up).
 * PD1  - GPIOD_USELESS1        (input pull-up).
 * PD2  - GPIOD_SPI3_CS         (output opendrain).
 * PD3  - GPIOD_LCD_RST         (output push-pull).
 * PD4  - GPIOD_USELESS4        (input pull-up).
 * PD5  - GPIOD_USELESS5        (input pull-up).
 * PD6  - GPIOD_LCD_CS          (output push-pull).
 * PD7  - GPIOD_USELESS7        (input pull-up).
 * PD8  - GPIOD_USART3_TX       (alternate 8).
 * PD9  - GPIOD_USART3_RX       (alternate 8).
 * PD10 - GPIOD_USELESS10       (input pull-up).
 * PD11 - GPIOD_USART3_CTS      (alternate 8).
 * PD12 - GPIOD_USART3_RTS      (alternate 8).
 * PD13 - GPIOD_USB_HS_FAULT    (input floating).
 * PD14 - GPIOD_USELESS14       (input pull-up).
 * PD15 - GPIOD_USELESS15       (input pull-up).
 */
#define VAL_GPIOD_MODER     (PIN_MODE_INPUT(GPIOD_USELESS0) |               \
                             PIN_MODE_INPUT(GPIOD_USELESS1) |               \
                             PIN_MODE_OUTPUT(GPIOD_SPI3_CS) |               \
                             PIN_MODE_OUTPUT(GPIOD_LCD_RST) |               \
                             PIN_MODE_INPUT(GPIOD_USELESS4) |               \
                             PIN_MODE_INPUT(GPIOD_USELESS5) |               \
                             PIN_MODE_OUTPUT(GPIOD_LCD_CS) |                \
                             PIN_MODE_INPUT(GPIOD_USELESS7) |               \
                             PIN_MODE_ALTERNATE(GPIOD_USART3_TX) |          \
                             PIN_MODE_ALTERNATE(GPIOD_USART3_RX) |          \
                             PIN_MODE_INPUT(GPIOD_USELESS10) |              \
                             PIN_MODE_ALTERNATE(GPIOD_USART3_CTS) |         \
                             PIN_MODE_ALTERNATE(GPIOD_USART3_RTS) |         \
                             PIN_MODE_INPUT(GPIOD_USB_HS_FAULT) |           \
                             PIN_MODE_INPUT(GPIOD_USELESS14) |              \
                             PIN_MODE_INPUT(GPIOD_USELESS15))
#define VAL_GPIOD_OTYPER    PIN_OTYPE_OPENDRAIN(GPIOD_SPI3_CS)
#define VAL_GPIOD_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOD_PUPDR     (PIN_PUDR_PULLUP(GPIOD_USELESS0) |              \
                             PIN_PUDR_PULLUP(GPIOD_USELESS1) |              \
                             PIN_PUDR_PULLUP(GPIOD_USELESS4) |              \
                             PIN_PUDR_PULLUP(GPIOD_USELESS5) |              \
                             PIN_PUDR_PULLUP(GPIOD_USELESS7) |              \
                             PIN_PUDR_PULLUP(GPIOD_USELESS10) |             \
                             PIN_PUDR_PULLUP(GPIOD_USELESS14) |             \
                             PIN_PUDR_PULLUP(GPIOD_USELESS15))
#define VAL_GPIOD_ODR       0xFFFFFFFF
#define VAL_GPIOD_AFRL      0x00000000
#define VAL_GPIOD_AFRH      (PIN_AFIO_AF(GPIOD_USART3_TX, 7) |              \
                             PIN_AFIO_AF(GPIOD_USART3_RX, 7) |              \
                             PIN_AFIO_AF(GPIOD_USART3_CTS, 7) |             \
                             PIN_AFIO_AF(GPIOD_USART3_RTS, 7))

/*
 * Port E setup.
 *
 * PE0  - GPIOE_0               (input pull-up).
 * PE1  - GPIOE_1               (input pull-up).
 * PE2  - GPIOE_TEMP_ALERT      (input floating).
 * PE3  - GPIOE_USB_HS_VBUSON   (output push-pull).
 * PE4  - GPIOE_4               (input pull-up).
 * PE5  - GPIOE_5               (input pull-up).
 * PE6  - GPIOE_6               (input pull-up).
 * PE7  - GPIOE_7               (input pull-up).
 * PE8  - GPIOE_8               (input pull-up).
 * PE9  - GPIOE_9               (input pull-up).
 * PE10 - GPIOE_10              (input pull-up).
 * PE11 - GPIOE_11              (input pull-up).
 * PE12 - GPIOE_12              (input pull-up).
 * PE13 - GPIOE_13              (input pull-up).
 * PE14 - GPIOE_14              (input pull-up).
 * PE15 - GPIOE_15              (input pull-up).
 */
#define VAL_GPIOE_MODER     (PIN_MODE_INPUT(GPIOE_0) |                      \
                             PIN_MODE_INPUT(GPIOE_1) |                      \
                             PIN_MODE_INPUT(GPIOE_TEMP_ALERT) |             \
                             PIN_MODE_OUTPUT(GPIOE_USB_HS_VBUSON) |         \
                             PIN_MODE_INPUT(GPIOE_4) |                      \
                             PIN_MODE_INPUT(GPIOE_5) |                      \
                             PIN_MODE_INPUT(GPIOE_6) |                      \
                             PIN_MODE_INPUT(GPIOE_7) |                      \
                             PIN_MODE_INPUT(GPIOE_8) |                      \
                             PIN_MODE_INPUT(GPIOE_9) |                      \
                             PIN_MODE_INPUT(GPIOE_10) |                     \
                             PIN_MODE_INPUT(GPIOE_11) |                     \
                             PIN_MODE_INPUT(GPIOE_12) |                     \
                             PIN_MODE_INPUT(GPIOE_13) |                     \
                             PIN_MODE_INPUT(GPIOE_14) |                     \
                             PIN_MODE_INPUT(GPIOE_15))
#define VAL_GPIOE_OTYPER    0x00000000
#define VAL_GPIOE_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOE_PUPDR     (PIN_PUDR_PULLUP(GPIOE_0) |                     \
                             PIN_PUDR_PULLUP(GPIOE_1) |                     \
                             PIN_PUDR_PULLUP(GPIOE_4) |                     \
                             PIN_PUDR_PULLUP(GPIOE_5) |                     \
                             PIN_PUDR_PULLUP(GPIOE_6) |                     \
                             PIN_PUDR_PULLUP(GPIOE_7) |                     \
                             PIN_PUDR_PULLUP(GPIOE_8) |                     \
                             PIN_PUDR_PULLUP(GPIOE_9) |                     \
                             PIN_PUDR_PULLUP(GPIOE_10) |                    \
                             PIN_PUDR_PULLUP(GPIOE_11) |                    \
                             PIN_PUDR_PULLUP(GPIOE_12) |                    \
                             PIN_PUDR_PULLUP(GPIOE_13) |                    \
                             PIN_PUDR_PULLUP(GPIOE_14) |                    \
                             PIN_PUDR_PULLUP(GPIOE_15))
#define VAL_GPIOE_ODR       0xFFFFFFF7
#define VAL_GPIOE_AFRL      0x00000000
#define VAL_GPIOE_AFRH      0x00000000

/*
 * Port F setup.
 *
 * PF0  - GPIOF_0               (input pull-up).
 * PF1  - GPIOF_1               (input pull-up).
 * PF2  - GPIOF_2               (input pull-up).
 * PF3  - GPIOF_3               (input pull-up).
 * PF4  - GPIOF_4               (input pull-up).
 * PF5  - GPIOF_5               (input pull-up).
 * PF6  - GPIOF_STAT1           (output push-pull).
 * PF7  - GPIOF_STAT2           (output push-pull).
 * PF8  - GPIOF_STAT3           (output push-pull).
 * PF9  - GPIOF_CAM_PWR         (output push-pull).
 * PF10 - GPIOF_10              (input pull-up).
 * PF11 - GPIOF_CAM_RS          (output push-pull).
 * PF12 - GPIOF_12              (input pull-up).
 * PF13 - GPIOF_13              (input pull-up).
 * PF14 - GPIOF_14              (input pull-up).
 * PF15 - GPIOF_15              (input pull-up).
 */
#define VAL_GPIOF_MODER     (PIN_MODE_INPUT(GPIOF_0) |                      \
                             PIN_MODE_INPUT(GPIOF_1) |                      \
                             PIN_MODE_INPUT(GPIOF_2) |                      \
                             PIN_MODE_INPUT(GPIOF_3) |                      \
                             PIN_MODE_INPUT(GPIOF_4) |                      \
                             PIN_MODE_INPUT(GPIOF_5) |                      \
                             PIN_MODE_OUTPUT(GPIOF_STAT1) |                 \
                             PIN_MODE_OUTPUT(GPIOF_STAT2) |                 \
                             PIN_MODE_OUTPUT(GPIOF_STAT3) |                 \
                             PIN_MODE_OUTPUT(GPIOF_CAM_PWR) |               \
                             PIN_MODE_INPUT(GPIOF_10) |                     \
                             PIN_MODE_OUTPUT(GPIOF_CAM_RS) |                \
                             PIN_MODE_INPUT(GPIOF_12) |                     \
                             PIN_MODE_INPUT(GPIOF_13) |                     \
                             PIN_MODE_INPUT(GPIOF_14) |                     \
                             PIN_MODE_INPUT(GPIOF_15))
#define VAL_GPIOF_OTYPER    0x00000000
#define VAL_GPIOF_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOF_PUPDR     (PIN_PUDR_PULLUP(GPIOF_0) |                     \
                             PIN_PUDR_PULLUP(GPIOF_1) |                     \
                             PIN_PUDR_PULLUP(GPIOF_2) |                     \
                             PIN_PUDR_PULLUP(GPIOF_3) |                     \
                             PIN_PUDR_PULLUP(GPIOF_4) |                     \
                             PIN_PUDR_PULLUP(GPIOF_5) |                     \
                             PIN_PUDR_PULLUP(GPIOF_10) |                    \
                             PIN_PUDR_PULLUP(GPIOF_12) |                    \
                             PIN_PUDR_PULLUP(GPIOF_13) |                    \
                             PIN_PUDR_PULLUP(GPIOF_14) |                    \
                             PIN_PUDR_PULLUP(GPIOF_15))
#define VAL_GPIOF_ODR       0xFFFFFC3F
#define VAL_GPIOF_AFRL      0x00000000
#define VAL_GPIOF_AFRH      0x00000000

/*
 * Port G setup.
 *
 * PG0  - GPIOG_0               (input pull-up).
 * PG1  - GPIOG_1               (input pull-up).
 * PG2  - GPIOG_2               (input pull-up).
 * PG3  - GPIOG_3               (input pull-up).
 * PG4  - GPIOG_4               (input pull-up).
 * PG5  - GPIOG_5               (input pull-up).
 * PG6  - GPIOG_RIGHT           (input floating).
 * PG7  - GPIOG_UP              (input floating).
 * PG8  - GPIOG_DOWN            (input floating).
 * PG9  - GPIOG_USART6_RX       (alternate 8).
 * PG10 - GPIOG_10              (input pull-up).
 * PG11 - GPIOG_LEFT            (input floating).
 * PG12 - GPIOG_12              (input pull-up).
 * PG13 - GPIOG_ETH_RMII_TXD0   (alternate 11).
 * PG14 - GPIOG_ETH_RMII_TXD1   (alternate 11).
 * PG15 - GPIOG_CENT            (input pull-up).
 */
#define VAL_GPIOG_MODER     (PIN_MODE_INPUT(GPIOG_0) |                      \
                             PIN_MODE_INPUT(GPIOG_1) |                      \
                             PIN_MODE_INPUT(GPIOG_2) |                      \
                             PIN_MODE_INPUT(GPIOG_3) |                      \
                             PIN_MODE_INPUT(GPIOG_4) |                      \
                             PIN_MODE_INPUT(GPIOG_5) |                      \
                             PIN_MODE_INPUT(GPIOG_RIGHT) |                  \
                             PIN_MODE_INPUT(GPIOG_UP) |                     \
                             PIN_MODE_INPUT(GPIOG_DOWN) |                   \
                             PIN_MODE_ALTERNATE(GPIOG_USART6_RX) |          \
                             PIN_MODE_INPUT(GPIOG_10) |                     \
                             PIN_MODE_INPUT(GPIOG_LEFT) |                   \
                             PIN_MODE_INPUT(GPIOG_12) |                     \
                             PIN_MODE_ALTERNATE(GPIOG_ETH_RMII_TXD0) |      \
                             PIN_MODE_ALTERNATE(GPIOG_ETH_RMII_TXD1) |      \
                             PIN_MODE_INPUT(GPIOG_CENT))
#define VAL_GPIOG_OTYPER    0x00000000
#define VAL_GPIOG_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOG_PUPDR     (PIN_PUDR_PULLUP(GPIOG_0) |                     \
                             PIN_PUDR_PULLUP(GPIOG_1) |                     \
                             PIN_PUDR_PULLUP(GPIOG_2) |                     \
                             PIN_PUDR_PULLUP(GPIOG_3) |                     \
                             PIN_PUDR_PULLUP(GPIOG_4) |                     \
                             PIN_PUDR_PULLUP(GPIOG_5) |                     \
                             PIN_PUDR_PULLUP(GPIOG_10) |                    \
                             PIN_PUDR_PULLUP(GPIOG_12))
#define VAL_GPIOG_ODR       0xFFFFFFFF
#define VAL_GPIOG_AFRL      0x00000000
#define VAL_GPIOG_AFRH      (PIN_AFIO_AF(GPIOG_USART6_RX, 8) |              \
                             PIN_AFIO_AF(GPIOG_ETH_RMII_TXD0, 11) |         \
                             PIN_AFIO_AF(GPIOG_ETH_RMII_TXD1, 11))

/*
 * Port H setup.
 * All input with pull-up except:
 * PH0  - GPIOH_OSC_IN          (input floating).
 * PH1  - GPIOH_OSC_OUT         (input floating).
 */
#define VAL_GPIOH_MODER     (PIN_MODE_INPUT(GPIOH_OSC_IN) |                 \
                             PIN_MODE_INPUT(GPIOH_OSC_OUT) |                \
                             PIN_MODE_INPUT(2) |                            \
                             PIN_MODE_INPUT(3) |                            \
                             PIN_MODE_INPUT(4) |                            \
                             PIN_MODE_INPUT(5) |                            \
                             PIN_MODE_INPUT(6) |                            \
                             PIN_MODE_INPUT(7) |                            \
                             PIN_MODE_INPUT(8) |                            \
                             PIN_MODE_INPUT(9) |                            \
                             PIN_MODE_INPUT(10) |                           \
                             PIN_MODE_INPUT(11) |                           \
                             PIN_MODE_INPUT(12) |                           \
                             PIN_MODE_INPUT(13) |                           \
                             PIN_MODE_INPUT(14) |                           \
                             PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER    0x00000000
#define VAL_GPIOH_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOH_PUPDR     (PIN_PUDR_FLOATING(GPIOH_OSC_IN) |              \
                             PIN_PUDR_FLOATING(GPIOH_OSC_OUT) |             \
                             PIN_PUDR_PULLUP(2) |                           \
                             PIN_PUDR_PULLUP(3) |                           \
                             PIN_PUDR_PULLUP(4) |                           \
                             PIN_PUDR_PULLUP(5) |                           \
                             PIN_PUDR_PULLUP(6) |                           \
                             PIN_PUDR_PULLUP(7) |                           \
                             PIN_PUDR_PULLUP(8) |                           \
                             PIN_PUDR_PULLUP(9) |                           \
                             PIN_PUDR_PULLUP(10) |                          \
                             PIN_PUDR_PULLUP(11) |                          \
                             PIN_PUDR_PULLUP(12) |                          \
                             PIN_PUDR_PULLUP(13) |                          \
                             PIN_PUDR_PULLUP(14) |                          \
                             PIN_PUDR_PULLUP(15))
#define VAL_GPIOH_ODR       0xFFFFFFFF
#define VAL_GPIOH_AFRL      0x00000000
#define VAL_GPIOH_AFRH      0x00000000

/*
 * Port I setup.
 * All input with pull-up.
 */
#define VAL_GPIOI_MODER     0x00000000
#define VAL_GPIOI_OTYPER    0x00000000
#define VAL_GPIOI_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOI_PUPDR     (PIN_PUDR_PULLUP(0) |                           \
                             PIN_PUDR_PULLUP(1) |                           \
                             PIN_PUDR_PULLUP(2) |                           \
                             PIN_PUDR_PULLUP(3) |                           \
                             PIN_PUDR_PULLUP(4) |                           \
                             PIN_PUDR_PULLUP(5) |                           \
                             PIN_PUDR_PULLUP(6) |                           \
                             PIN_PUDR_PULLUP(7) |                           \
                             PIN_PUDR_PULLUP(8) |                           \
                             PIN_PUDR_PULLUP(9) |                           \
                             PIN_PUDR_PULLUP(10) |                          \
                             PIN_PUDR_PULLUP(11) |                          \
                             PIN_PUDR_PULLUP(12) |                          \
                             PIN_PUDR_PULLUP(13) |                          \
                             PIN_PUDR_PULLUP(14) |                          \
                             PIN_PUDR_PULLUP(15))
#define VAL_GPIOI_ODR       0xFFFFFFFF
#define VAL_GPIOI_AFRL      0x00000000
#define VAL_GPIOI_AFRH      0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
