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
 * Setup for the Olimex LPC-P1343 proto board.
 */

/*
 * Board identifiers.
 */
#define BOARD_OLIMEX_LPC_P1343
#define BOARD_NAME "Olimex LPC-P1343"

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
#define VAL_GPIO1DIR            PAL_PORT_BIT(GPIO1_SW2)

#define VAL_GPIO1DATA           PAL_PORT_BIT(GPIO1_SW2)

/*
 * GPIO 2 initial setup.
 */

#define VAL_GPIO2DIR            PAL_PORT_BIT(GPIO2_SW1)     | \
                                PAL_PORT_BIT(GPIO2_LED5)    | \
                                PAL_PORT_BIT(GPIO2_LED6)    | \
                                PAL_PORT_BIT(GPIO2_LED7)    | \
                                PAL_PORT_BIT(GPIO2_LED8)

#define VAL_GPIO2DATA           PAL_PORT_BIT(GPIO2_LED5)    | \
                                PAL_PORT_BIT(GPIO2_LED6)    | \
                                PAL_PORT_BIT(GPIO2_LED7)    | \
                                PAL_PORT_BIT(GPIO2_LED8)

/*
 * GPIO 3 initial setup.
 */

#define VAL_GPIO3DIR            PAL_PORT_BIT(GPIO3_LED1)    | \
                                PAL_PORT_BIT(GPIO3_LED2)    | \
                                PAL_PORT_BIT(GPIO3_LED3)    | \
                                PAL_PORT_BIT(GPIO3_LED4)

#define VAL_GPIO3DATA           PAL_PORT_BIT(GPIO3_LED1)    | \
                                PAL_PORT_BIT(GPIO3_LED2)    | \
                                PAL_PORT_BIT(GPIO3_LED3)    | \
                                PAL_PORT_BIT(GPIO3_LED4)

/*
 * Pin definitions.
 */
#define GPIO1_SW2               4
#define GPIO1_SPI0SEL           11

#define GPIO2_SW1               9

#define GPIO3_LED1              0
#define GPIO3_LED2              1
#define GPIO3_LED3              2
#define GPIO3_LED4              3
#define GPIO2_LED5              4
#define GPIO2_LED6              5
#define GPIO2_LED7              6
#define GPIO2_LED8              7

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
