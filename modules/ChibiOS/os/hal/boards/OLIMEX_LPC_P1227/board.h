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
 * Setup for Olimex LPC-P1227 board.
 *
 */

/*
 * Board identifiers.
 */
#define OLIMEX_LPC_P1227
#define BOARD_NAME "Olimex LPC-P1227"

/*
 * Board frequencies.
 */
#define SYSOSCCLK               12000000


/*
 * GPIO 0 initial setup.
 */
#define VAL_GPIO0DIR            0x00000000
#define VAL_GPIO0DATA           0x00000000

/*
 * GPIO 1 initial setup.
 */
#define VAL_GPIO1DIR            PAL_PORT_BIT(GPIO1_LED1)   |                 \
                                PAL_PORT_BIT(GPIO1_LED2)   |                 \
                                PAL_PORT_BIT(GPIO1_BUZZER)

#define VAL_GPIO1DATA           PAL_PORT_BIT(GPIO1_LED1)


/*
 * GPIO 2 initial setup.
 */
#define VAL_GPIO2DIR            PAL_PORT_BIT(GPIO2_LCD_DC)   |               \
                                PAL_PORT_BIT(GPIO2_LCD_SS)   |               \
                                PAL_PORT_BIT(GPIO2_LCD_RES)
#define VAL_GPIO2DATA           PAL_PORT_BIT(GPIO2_LCD_SS)


/*
 * Pin definitions.
 */

#define GPIO1_LED1              5
#define GPIO1_LED2              4
#define GPIO1_SW_WAKEUP         3
#define GPIO1_BUZZER            6

#define GPIO2_SW_USER1          12
#define GPIO2_SW_USER2          11
#define GPIO2_SW_USER3          10
#define GPIO2_LCD_DC            15
#define GPIO2_LCD_SS            14
#define GPIO2_LCD_RES           13

/* LCD3310 pins */
#define LCD3310_RES_PIN   GPIO2_LCD_RES
#define LCD3310_RES_PORT  GPIO2
#define LCD3310_DC_PIN    GPIO2_LCD_DC
#define LCD3310_DC_PORT   GPIO2

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
