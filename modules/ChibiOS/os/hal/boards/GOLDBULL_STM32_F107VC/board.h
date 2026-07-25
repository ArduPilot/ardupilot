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
 * Setup for the GoldBull STM32F107 V3 evaluation board.
 */

/*
 * Board identifier.
 */
#define BOARD_GOLDBULL_STM32F107VC
#define BOARD_NAME              "GoldBull STM32F107VC V3"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            25000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F107xC

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID            MII_DP83848I_ID
#define BOARD_PHY_RMII

/*
 * IO pins assignments.
 */
#define GPIOA_SWITCH_WKUP       0
#define GPIOB_SWITCH_USER       2
#define GPIOC_SWITCH_TAMPER     13

#define GPIOC_SPI3_SD_CS        3
#define GPIOC_SPI3_SD_CD        2

#define GPIOD_LED3              2
#define GPIOD_LED4              3
#define GPIOD_LED5              4
#define GPIOD_LED6              7

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * PA0  - Normal input      (WKUP BUTTON).
 * PA1  - Normal input      (ETH_RMII_REF_CLK).
 * PA2  - Alternate output  (ETH_RMII_MDIO).
 * PA3  - Input with PU     (TP_IRQ).
 * PA4  - Push Pull output  (TP_CS).
 * PA5  - Alternate output  (SPI1 SCK).
 * PA6  - Input with PU     (SPI1 MISO).
 * PA7  - Alternate output  (SPI1 MOSI).
 * PA8  - Alternate output  (MCO).
 * PA9  - Normal input      (OTG_VBUS).
 * PA10 - Normal input      (OTG_ID).
 * PA11 - Normal input      (OTG_DM).
 * PA12 - Normal input      (OTG_DP).
 * PA13 - Normal input      (TMS).
 * PA14 - Normal input      (TCK).
 * PA15 - Normal input      (TDI).
 */
#define VAL_GPIOACRL            0xB8B38B44      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x4444444B      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 */

/*
 * Port B setup:
 * PB0  - Input with PU     (unconnected).
 * PB1  - Input with PU     (unconnected).
 * PB2  - Normal input      (BOOT1, KEY_USER).
 * PB3  - Normal input      (TDO).
 * PB4  - Normal input      (TRST).
 * PB5  - Input with PU     (CAN2 RX).
 * PB6  - Alternate output  (USART1 TX, remapped).
 * PB7  - Input with PU     (USART1 RX, remapped).
 * PB8  - Alternate O.D.    (I2C1 SCL, remapped).
 * PB9  - Alternate O.D.    (I2C1 SDA, remapped).
 * PB10 - Input with PU     (BL_CNT).
 * PB11 - Alternate output  (ETH_RMII_TX_EN).
 * PB12 - Alternate output  (ETH_RMII_TXD0).
 * PB13 - Alternate output  (ETH_RMII_TXD1).
 * PB14 - Input with PU     (unconnected).
 * PB15 - Input with PU     (unconnected).
 */
#define VAL_GPIOBCRL            0x8B844488      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x88BBB8FF      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 */

/*
 * Port C setup:
 * PC0  - Analog input      (ADC_IN).
 * PC1  - Alternate output  (ETH_MDC).
 * PC2  - Input with PU     (SD_CD).
 * PC3  - Push Pull output  (SD_CS).
 * PC4  - Push Pull output  (OTG_PW_ON).
 * PC5  - Push Pull output  (SF_CS).
 * PC6  - Push Pull output  (LCD_RD).
 * PC7  - Push Pull output  (LCD_WR).
 * PC8  - Push Pull output  (LCD_RS).
 * PC9  - Push Pull output  (LCD_CS).
 * PC10 - Alternate output  (SPI3 SCK).
 * PC11 - Input with PU     (SPI3 MISO).
 * PC12 - Alternate output  (SPI3 MOSI).
 * PC13 - Normal input      (KEY_TAMPER).
 * PC14 - Normal input      (OSC32 IN).
 * PC15 - Normal input      (OSC32 OUT).
 */
#define VAL_GPIOCCRL            0x333338B0      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x444B8B33      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 */

/*
 * Port D setup:
 * PD0  - Input with PU     (CAN1_RX, remapped).
 * PD1  - Alternate output  (CAN1_TX, remapped).
 * PD2  - Push Pull output  (LED3).
 * PD3  - Push Pull output  (LED4).
 * PD4  - Push Pull output  (LED5).
 * PD5  - Alternate output  (USART2 TX, remapped).
 * PD6  - Input with PU     (USART2 RX, remapped).
 * PD7  - Push Pull output  (LED6).
 * PD8  - Normal input      (ETH_RMII_CRS_DV, remapped).
 * PD9  - Normal input      (ETH_RMII_RXD0, remapped).
 * PD10 - Normal input      (ETH_RMII_RXD1, remapped).
 * PD11 - Input with PU     (JOY_UP).
 * PD12 - Input with PU     (JOY_DOWN).
 * PD13 - Input with PU     (JOY_LEFT).
 * PD14 - Input with PU     (JOY_RIGHT).
 * PD15 - Input with PU     (JOY_SEL).
 */
#define VAL_GPIODCRL            0x38B333B8      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888444      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

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
