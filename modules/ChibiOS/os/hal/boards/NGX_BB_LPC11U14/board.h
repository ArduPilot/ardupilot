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
 * Setup for NGX BlueBoard LPC11U14.
 */

/*
 * Board identifiers.
 */
#define BOARD_NGX_BB_LPC11U14
#define BOARD_NAME "NGX BlueBoard LPC11U14"

/*
 * Board frequencies.
 */
#define SYSOSCCLK               12000000

/*
 * Pin definitions.
 */
 
/* GPIO Port0 */
#define BUTTON_ISP_PORT       GPIO0
#define BUTTON_ISP            1

#define LCD_ERD_PORT          GPIO0
#define LCD_ERD               12

#define LCD_RWR_PORT          GPIO0
#define LCD_RWR               13

#define LCD_RS_PORT           GPIO0
#define LCD_RS                14

#define LED_PORT              GPIO0
#define LED_TEST1             22
#define LED_TEST2             23

/* GPIO Port1 */
#define LCD_RST_PORT          GPIO1
#define LCD_RST               0

#define LCD_CS_PORT           GPIO1
#define LCD_CS                13

#define LCD_BL_PORT           GPIO1
#define LCD_BL                14
#define LCD_VCCEN_PORT        GPIO1
#define LCD_VCCEN             14

#define WHEEL_SENSOR_PORT     GPIO0
#define WHEEL_SENSOR          21

#define LCD_DATA_PORT         GPIO1
#define LCD_D0                19
#define LCD_D1                20
#define LCD_D2                21
#define LCD_D3                22
#define LCD_D4                26
#define LCD_D5                27
#define LCD_D6                28
#define LCD_D7                31

#define LCD_DATA_MASK           PAL_PORT_BIT(LCD_D0)  | \
                                PAL_PORT_BIT(LCD_D1)  | \
                                PAL_PORT_BIT(LCD_D2)  | \
                                PAL_PORT_BIT(LCD_D3)  | \
                                PAL_PORT_BIT(LCD_D4)  | \
                                PAL_PORT_BIT(LCD_D5)  | \
                                PAL_PORT_BIT(LCD_D6)  | \
                                PAL_PORT_BIT(LCD_D7)

/*
 * GPIO 0 initial setup.
 */
#define VAL_GPIO0DIR            PAL_PORT_BIT(LCD_ERD)   | \
                                PAL_PORT_BIT(LCD_RWR)   | \
                                PAL_PORT_BIT(LCD_RS)    | \
                                PAL_PORT_BIT(LED_TEST1) | \
                                PAL_PORT_BIT(LED_TEST2)
                                
#define VAL_GPIO0DATA           PAL_PORT_BIT(LCD_ERD)   | \
                                PAL_PORT_BIT(LCD_RWR)   | \
                                PAL_PORT_BIT(LED_TEST1) | \
                                PAL_PORT_BIT(LED_TEST2)

/*
 * GPIO 1 initial setup.
 */
#define VAL_GPIO1DIR            PAL_PORT_BIT(LCD_RST) | \
                                PAL_PORT_BIT(LCD_CS)  | \
                                PAL_PORT_BIT(LCD_BL)  | \
                                LCD_DATA_MASK
                                
#define VAL_GPIO1DATA           0x00000000


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif

#endif
