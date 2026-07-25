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
 * Setup for Embedded Artists LPCXpresso Base Board with LPC1114 daughter
 * board.
 */

/*
 * Board identifiers.
 */
#define BOARD_EA_BB_LPC11U14
#define BOARD_NAME "Embedded Artists LPCXpresso Base Board + LPC11U14"

/*
 * Board frequencies.
 */
#define SYSOSCCLK               12000000

/*
 * SCK0 connection on this board.
 */
#define LPC11xx_SPI_SCK0_SELECTOR SCK0_IS_PIO2_11

/*
 * GPIO 0 initial setup.
 */
#define VAL_GPIO0DIR            PAL_PORT_BIT(GPIO0_OLEDSEL)   |             \
                                PAL_PORT_BIT(GPIO0_USB_DPCTL) |             \
                                PAL_PORT_BIT(GPIO0_LED2)      |             \
                                PAL_PORT_BIT(GPIO0_LED3B)     |             \
                                PAL_PORT_BIT(GPIO0_LED3R)     |             \
                                PAL_PORT_BIT(GPIO0_LED3G)     |             \
                                PAL_PORT_BIT(GPIO0_SPI0SEL)
#define VAL_GPIO0DATA           PAL_PORT_BIT(GPIO0_OLEDSEL)   |             \
                                PAL_PORT_BIT(GPIO0_LED2)      |             \
                                PAL_PORT_BIT(GPIO0_LED3B)     |             \
                                PAL_PORT_BIT(GPIO0_LED3R)     |             \
                                PAL_PORT_BIT(GPIO0_LED3G)     |             \
                                PAL_PORT_BIT(GPIO0_SPI0SEL)

/*
 * GPIO 1 initial setup.
 */
#define VAL_GPIO1DIR            0x00000000
#define VAL_GPIO1DATA           0x00000000

/*
 * Pin definitions.
 */
#define GPIO0_SW3               1
#define GPIO0_OLEDSEL           2
#define GPIO0_USB_VBUS          3
#define GPIO0_USB_DPCTL         6
#define GPIO0_LED2              7
#define GPIO0_SW4               16
#define GPIO0_LED3B             14
#define GPIO0_LED3R             21
#define GPIO0_LED3G             22
#define GPIO0_SPI0SEL           23

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
