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
   Concepts and parts of this file have been contributed by Rocco Marco
   Guglielmi.
 */
#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the ST7580 Evaluation kit board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_EVALKIT_ST7580_1
#define BOARD_NAME              "STMicroelectronics EVALKITST7580_1"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            8000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F103xB

/*
 * IO pins assignments.
 */
#define GPIOA_CL_ADC            0
#define GPIOA_PLC_T_REQ         1
#define GPIOA_PLC_TXD           2
#define GPIOA_PLC_RXD           3
#define GPIOA_SD_NSS            4
#define GPIOA_SD_SCLK           5
#define GPIOA_SD_MISO           6
#define GPIOA_SD_MOSI           7
#define GPIOA_8MHZ_MCO          8
#define GPIOA_UART1_TXD         9
#define GPIOA_UART1_RXD         10
#define GPIOA_USBDM             11
#define GPIOA_USBDP             12
#define GPIOA_JTMS              13
#define GPIOA_JTCK              14
#define GPIOA_JTDI              15

#define GPIOB_SD_CD             0
#define GPIOB_PA_ADC            1
#define GPIOB_BOOT1             2
#define GPIOB_J_TDO             3
#define GPIOB_J_TRSTN           4
#define GPIOB_LED_RED           5
#define GPIOB_LED_YELLOW        6
#define GPIOB_LED_GREEN         7
#define GPIOB_I2C1_SCL          8
#define GPIOB_I2C1_SDA          9
#define GPIOB_SW4               10
#define GPIOB_SW3               11
#define GPIOB_DIP_SW4           12
#define GPIOB_DIP_SW3           13
#define GPIOB_DIP_SW2           14
#define GPIOB_DIP_SW1           15

#define GPIOC_PC0               0
#define GPIOC_PC1               1
#define GPIOC_PC2               2
#define GPIOC_PC3               3
#define GPIOC_PC4               4
#define GPIOC_PC5               5
#define GPIOC_PC6               6
#define GPIOC_PC7               7
#define GPIOC_PC8               8
#define GPIOC_PC9               9
#define GPIOC_PC10              10
#define GPIOC_PC11              11
#define GPIOC_PC12              12
#define GPIOC_PL_RX_ON          13
#define GPIOC_PL_TX_ON          14
#define GPIOC_PLC_RESETN        15

#define GPIOD_OSC_IN            0
#define GPIOD_OSC_OUT           1

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
 * PA0  - Analog input              (GPIOA_CL_ADC)
 * PA1  - Push Pull output          (GPIOA_PLC_T_REQ).
 * PA2  - Alternate output          (GPIOA_PLC_TXD).
 * PA3  - Pull-up input             (GPIOA_PLC_RXD).
 * PA4  - Push Pull output          (GPIOA_SD_NSS).
 * PA5  - Alternate output          (GPIOA_SD_SCLK).
 * PA6  - Alternate output          (GPIOA_SD_MISO).
 * PA7  - Alternate output          (GPIOA_SD_MOSI).
 * PA8  - Alternate output          (GPIOA_8MHZ_MCO).
 * PA9  - Alternate output          (GPIOA_UART1_TXD).
 * PA10 - Normal input              (GPIOA_UART1_RXD).
 * PA11 - Alternate output          (GPIOA_USBDM).
 * PA12 - Alternate output          (GPIOA_USBDP).
 * PA13 - Pull-up input             (GPIOA_JTMS).
 * PA14 - Pull-down input           (GPIOA_JTCK).
 * PA15 - Pull-up input             (GPIOA_JTDI).
 */
#define VAL_GPIOACRL            0xBBB38B30      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x888BB4BB      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFBFFF

/*
 * Port B setup.
 * PB0  - Pull-up input             (GPIOB_SD_CD)
 * PB1  - Analog input              (GPIOB_PA_ADC).
 * PB2  - Normal input              (GPIOB_BOOT1).
 * PB3  - Pull-up input             (GPIOB_J_TDO).
 * PB4  - Pull-up input             (GPIOB_J_TRSTN).
 * PB5  - Push Pull output          (GPIOB_LED_RED).
 * PB6  - Push Pull output          (GPIOB_LED_YELLOW).
 * PB7  - Push Pull output          (GPIOB_LED_GREEN).
 * PB8  - Alternate output          (GPIOB_I2C1_SCL).
 * PB9  - Alternate output          (GPIOB_I2C1_SDA).
 * PB10 - Normal input              (GPIOB_SW4).
 * PB11 - Normal input              (GPIOB_SW3).
 * PB12 - Normal input              (GPIOB_DIP_SW4).
 * PB13 - Normal input              (GPIOB_DIP_SW3).
 * PB14 - Normal input              (GPIOB_DIP_SW2).
 * PB15 - Normal input              (GPIOB_DIP_SW1).
 */
#define VAL_GPIOBCRL            0x33388408      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x444444BB      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 * PC13 - Normal input              (GPIOC_PL_RX_ON).
 * PC14 - Normal input              (GPIOC_PL_TX_ON).
 * PC15 - Push Pull output          (GPIOC_PLC_RESETN).
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x34488888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input              (GPIOD_OSC_IN).
 * PD1  - Normal input              (GPIOD_OSC_OUT).
 */
#define VAL_GPIODCRL            0x88888844      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

/*
 * USB bus activation macro, required by the USB driver.
 */
#define usb_lld_connect_bus(usbp)

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp)

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
