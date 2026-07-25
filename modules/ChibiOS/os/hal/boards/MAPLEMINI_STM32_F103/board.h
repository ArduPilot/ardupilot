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
 * Setup for the LeafLabs Maple Mini.
 */

/*
 * Board identifier.
 */
#define BOARD_MAPLEMINI_STM32_F103
#define BOARD_NAME              "LeafLabs Maple Mini"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            8000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F103xB

/*
 * IO pins assignments
 *
 * numbering is sorted by onboard/connectors, as from the schematics in
 * https://github.com/leaflabs/maplemini
 */

/* on-board */

#define GPIOB_LED               1
#define GPIOB_USB_DISC          9
#define GPIOD_OSC_IN            0
#define GPIOD_OSC_OUT           1

/* J1 connector */

// pin 1:       AV+
// pin 2:       AV-
// pin 3:       VBAT
#define GPIOC_TAMPER_RTC        13      // pin 4
#define GPIOC_OSC32_IN          14      // pin 5
#define GPIOC_OSD32_OUT         15      // pin 6
// pin 7:       RESET
#define GPIOA_USART2_CTS        0       // pin 8
#define GPIOA_USART2_RTS        1       // pin 9
#define GPIOA_USART2_TX         2       // pin 10
#define GPIOA_USART2_RX         3       // pin 11
#define GPIOA_SPI1_NSS          4       // pin 12
#define GPIOA_SPI1_SCK          5       // pin 13
#define GPIOA_SPI1_MISO         6       // pin 14
#define GPIOA_SPI1_MOSI         7       // pin 15
#define GPIOB_ADC12_IN8         0       // pin 16
#define GPIOB_BOOT1             2       // pin 17
#define GPIOB_I2C2_SCL          10      // pin 18
#define GPIOB_I2C2_SDA          11      // pin 19
// pin 20: VIN

/* J2 connector */

#define GPIOB_SPI2_NSS          12      // pin 1
#define GPIOB_SPI2_SCK          13      // pin 2
#define GPIOB_SPI2_MISO         14      // pin 3
#define GPIOB_SPI2_MOSI         15      // pin 4
#define GPIOA_USART1_CK         8       // pin 5
#define GPIOA_USART1_TX         9       // pin 6
#define GPIOA_USART1_RX         10      // pin 7
#define GPIOA_USBDM             11      // pin 8
#define GPIOA_USBDP             12      // pin 9
#define GPIOA_JTMS              13      // pin 10
#define GPIOA_JTCK              14      // pin 11
#define GPIOA_JTDI              15      // pin 12
#define GPIOB_JTDO              3       // pin 13
#define GPIOB_JTRST             4       // pin 14
#define GPIOB_I2C1_SMBA         5       // pin 15
#define GPIOB_I2C1_SCL          6       // pin 16
#define GPIOB_I2C1_SDA          7       // pin 17
#define GPIOB_BOOT0_BUTTON      8       // pin 18
// pin 19: GND
// pin 20: VCC

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
 * PA2  - Alternate output  (USART2 TX).
 * PA3  - Normal input      (USART2 RX).
 * PA9  - Alternate output  (USART1 TX).
 * PA10 - Normal input      (USART1 RX).
 */
#define VAL_GPIOACRL            0x88884B88      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x888884B8      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB1    - Push Pull output  (LED).
 * PB9    - Push Pull output  (USB switch).
 */
#define VAL_GPIOBCRL            0x88888838      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x88888838      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88888888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).
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
#define usb_lld_connect_bus(usbp) palClearPad(GPIOB, GPIOB_USB_DISC)

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp) palSetPad(GPIOB, GPIOB_USB_DISC)

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
