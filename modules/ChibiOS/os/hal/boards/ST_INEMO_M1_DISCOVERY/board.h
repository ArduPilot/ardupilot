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
 * Setup for the ST INEMO-M1 Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_INEMO_M1_DISCOVERY
#define BOARD_NAME              "STMicroelectronics INEMO-M1 Discovery"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            8000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F103xE

/*
 * IO pins assignments.
 */
#define GPIOA_PRESS_INT         0
#define GPIOA_LED_BLUE          1
#define GPIOA_PA2               2
#define GPIOA_PA3               3
#define GPIOA_PA4               4
#define GPIOA_PA5               5
#define GPIOA_PA6               6
#define GPIOA_PA7               7
#define GPIOA_USB_EN            9
#define GPIOA_BUTTON            10
#define GPIOA_USB_DM            11
#define GPIOA_USB_DP            12
#define GPIOA_SWDIO             13
#define GPIOA_SWCLK             14

#define GPIOB_SWO               3
#define GPIOB_I2C1_SCL          6
#define GPIOB_I2C1_SDA          7
#define GPIOB_PB8               8
#define GPIOB_PB9               9
#define GPIOB_I2C2_SCL          10
#define GPIOB_I2C2_SDA          11
#define GPIOB_SPI2_CS           12
#define GPIOB_SPI2_SCK          13
#define GPIOB_SPI2_MISO         14
#define GPIOB_SPI2_MOSI         15

#define GPIOC_GYRO_INT1         6
#define GPIOC_LMS_DRDY          7
#define GPIOC_LMS_INT1          8
#define GPIOC_GYRO_DRDY         9

#define GPIOD_LMS_INT2          2

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
 * PA0  - Push Pull output          (GPIOA_PRESS_INT).
 * PA1  - Push Pull output          (GPIOA_LED_BLUE).
 * PA9  - Push Pull output          (GPIOA_USB_EN).
 * PA10 - Normal input              (GPIOA_BUTTON).
 * PA11 - Normal input              (GPIOA_USB_DM).
 * PA12 - Normal input              (GPIOA_USB_DP).
 * PA13 - Pull-up input             (GPIOA_SWDIO).
 * PA14 - Pull-down input           (GPIOA_SWCLK).
 */
#define VAL_GPIOACRL            0x88888838      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x88844438      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFBDFD

/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB3     - Pull-up input          (GPIOB_SWO).
 * PB6,7   - Alternate open drain   (I2C1).
 * PB10,11 - Alternate open drain   (I2C2).
 * PB12    - Push Pull output       (GPIOB_SPI2_CS).
 * PB13    - Alternate output       (GPIOB_SPI2_SCK).
 * PB14    - Normal input           (GPIOB_SPI2_MISO).
 * PB15    - Alternate output       (GPIOB_SPI2_MOSI).
 */
#define VAL_GPIOBCRL            0xEE888888      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0xB4B3EE88      /* PB15...PB8 */
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
 * Port F setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOFCRL            0x88888888      /*  PF7...PF0 */
#define VAL_GPIOFCRH            0x88888888      /* PF15...PF8 */
#define VAL_GPIOFODR            0xFFFFFFFF

/*
 * Port G setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOGCRL            0x88888888      /*  PG7...PG0 */
#define VAL_GPIOGCRH            0x88888888      /* PG15...PG8 */
#define VAL_GPIOGODR            0xFFFFFFFF

/*
 * USB bus activation macro, required by the USB driver.
 */
#define usb_lld_connect_bus(usbp) palSetPad(GPIOA, GPIOA_USB_EN)

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp) palClearPad(GPIOA, GPIOA_USB_EN)

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
