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
 * Setup for Analog Devices MAX32625MBED# board.
 */

/*
 * Board identifier.
 */
#define BOARD_ADI_EVAL_SDP_CK1Z
#define BOARD_NAME                  "Analog Devices MAX32625MBED#"

/*
 * MCU type as defined in the ST header.
 */
#define MAX32625

/*
 * IO pins assignments.
 */
#define GPIO0_0                       0U
#define GPIO0_1                       1U
#define GPIO0_2                       2U
#define GPIO0_3                       3U
#define GPIO0_4                       4U
#define GPIO0_5                       5U
#define GPIO0_6                       6U
#define GPIO0_7                       7U

#define GPIO1_0                       0U
#define GPIO1_1                       1U
#define GPIO1_2                       2U
#define GPIO1_3                       3U
#define GPIO1_4                       4U
#define GPIO1_5                       5U
#define GPIO1_6                       6U
#define GPIO1_7                       7U

#define GPIO2_0                       0U
#define GPIO2_DAPLINK_RX              0U
#define GPIO2_1                       1U
#define GPIO2_DAPLINK_TX              1U
#define GPIO2_2                       2U
#define GPIO2_BUTTON2                 2U
#define GPIO2_3                       3U
#define GPIO2_BUTTON3                 3U
#define GPIO2_4                       4U
#define GPIO2_5                       5U
#define GPIO2_6                       6U
#define GPIO2_7                       7U

#define GPIO3_0                       0U
#define GPIO3_LED_RED                 0U
#define GPIO3_1                       1U
#define GPIO3_LED_GREEN               0U
#define GPIO3_2                       2U
#define GPIO3_LED_BLUE                0U
#define GPIO3_3                       3U
#define GPIO3_LED_ORANGE              0U
#define GPIO3_4                       4U
#define GPIO3_5                       5U
#define GPIO3_6                       6U
#define GPIO3_7                       7U

#define GPIO4_0                       0U
#define GPIO4_1                       1U
#define GPIO4_2                       2U
#define GPIO4_3                       3U
#define GPIO4_4                       4U
#define GPIO4_5                       5U
#define GPIO4_6                       6U
#define GPIO4_7                       7U

/*
 * IO lines assignments.
 */
#define LINE_DAPLINK_RX             PAL_LINE(GPIO2, 0U)
#define LINE_DAPLINK_TX             PAL_LINE(GPIO2, 1U)
#define LINE_BUTTON2                PAL_LINE(GPIO2, 2U)
#define LINE_BUTTON3                PAL_LINE(GPIO2, 3U)
#define LINE_LED_RED                PAL_LINE(GPIO3, 0U)
#define LINE_LED_GREEN              PAL_LINE(GPIO3, 1U)
#define LINE_LED_BLUE               PAL_LINE(GPIO3, 2U)
#define LINE_LED_ORANGE             PAL_LINE(GPIO3, 3U)

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
 * Please refer to the MAX32625 Reference Manual for details.
 */
#define PIN_OMODE_HIGHZ_PULLUP(n)         (0U << (4 * n))
#define PIN_OMODE_OPENDRAIN(n)            (1U << (4 * n))  
#define PIN_OMODE_OPENDRAIN_PULLUP(n)     (2U << (4 * n)) 
#define PIN_OMODE_NORMAL_HIGHZ(n)         (4U << (4 * n))
#define PIN_OMODE_NORMAL_DRIVE(n)         (5U << (4 * n))   
#define PIN_OMODE_SLOW_HIGHZ(n)           (6U << (4 * n))  
#define PIN_OMODE_SLOW_DRIVE(n)           (7U << (4 * n))  
#define PIN_OMODE_FAST_HIGHZ(n)           (8U << (4 * n))  
#define PIN_OMODE_FAST_DRIVE(n)           (9U << (4 * n))  
#define PIN_OMODE_HIGHZ_PULLDOWN(n)       (10U << (4 * n)) 
#define PIN_OMODE_OPENSOURCE(n)           (11U << (4 * n)) 
#define PIN_OMODE_OPENSOURCE_PULLDOWN(n)  (12U << (4 * n)) 
#define PIN_OMODE_DISABLED(n)             (15U << (4 * n)) 

#define PIN_OV_LOW(n)                     (0U << (n))  
#define PIN_OV_HIGH(n)                    (1U << (n))

#define PIN_FUNC_GPIO(n)                  (0U << (4 * n))
#define PIN_FUNC_PULSE(n)                 (1U << (4 * n))
#define PIN_FUNC_TIMER(n)                 (2U << (4 * n))

#define PIN_IMODE_NORMAL_INPUT(n)         (0U << (4 * n))  
#define PIN_IMODE_INVERTED_INPUT(n)       (1U << (4 * n))  
#define PIN_IMODE_ALWAYS_LOW(n)           (2U << (4 * n))  
#define PIN_IMODE_ALWAYS_HIGH(n)          (3U << (4 * n))

/*
 * GPIO0 setup:
 *
 * GPIO0.0  - PIN0                  High-Z, input disabled.
 * GPIO0.1  - PIN1                  High-Z, input disabled.
 * GPIO0.2  - PIN2                  High-Z, input disabled.
 * GPIO0.3  - PIN3                  High-Z, input disabled.
 * GPIO0.4  - PIN4                  High-Z, input disabled.
 * GPIO0.5  - PIN5                  High-Z, input disabled.
 * GPIO0.6  - PIN6                  High-Z, input disabled. 
 * GPIO0.7  - PIN6                  High-Z, input disabled.
 */
#define VAL_GPIO0_OUTMODE           (PIN_OMODE_DISABLED(GPIO0_0) |          \
                                     PIN_OMODE_DISABLED(GPIO0_1) |          \
                                     PIN_OMODE_DISABLED(GPIO0_2) |          \
                                     PIN_OMODE_DISABLED(GPIO0_3) |          \
                                     PIN_OMODE_DISABLED(GPIO0_4) |          \
                                     PIN_OMODE_DISABLED(GPIO0_5) |          \
                                     PIN_OMODE_DISABLED(GPIO0_6) |          \
                                     PIN_OMODE_DISABLED(GPIO0_7))
#define VAL_GPIO0_OUTVAL            (PIN_OV_LOW(GPIO0_0) |                  \
                                     PIN_OV_LOW(GPIO0_1) |                  \
                                     PIN_OV_LOW(GPIO0_2) |                  \
                                     PIN_OV_LOW(GPIO0_3) |                  \
                                     PIN_OV_LOW(GPIO0_4) |                  \
                                     PIN_OV_LOW(GPIO0_5) |                  \
                                     PIN_OV_LOW(GPIO0_6) |                  \
                                     PIN_OV_LOW(GPIO0_7))
#define VAL_GPIO0_FUNCSEL           (PIN_FUNC_GPIO(GPIO0_0) |               \
                                     PIN_FUNC_GPIO(GPIO0_1) |               \
                                     PIN_FUNC_GPIO(GPIO0_2) |               \
                                     PIN_FUNC_GPIO(GPIO0_3) |               \
                                     PIN_FUNC_GPIO(GPIO0_4) |               \
                                     PIN_FUNC_GPIO(GPIO0_5) |               \
                                     PIN_FUNC_GPIO(GPIO0_6) |               \
                                     PIN_FUNC_GPIO(GPIO0_7))
#define VAL_GPIO0_INMODE            (PIN_IMODE_NORMAL_INPUT(GPIO0_0) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO0_1) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO0_2) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO0_3) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO0_4) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO0_5) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO0_6) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO0_7))

/*
 * GPIO1 setup:
 *
 * GPIO1.0  - PIN0                  High-Z, input disabled.
 * GPIO1.1  - PIN1                  High-Z, input disabled.
 * GPIO1.2  - PIN2                  High-Z, input disabled.
 * GPIO1.3  - PIN3                  High-Z, input disabled.
 * GPIO1.4  - PIN4                  High-Z, input disabled.
 * GPIO1.5  - PIN5                  High-Z, input disabled.
 * GPIO1.6  - PIN6                  High-Z, input disabled. 
 * GPIO1.7  - PIN6                  High-Z, input disabled.
 */
#define VAL_GPIO1_OUTMODE           (PIN_OMODE_DISABLED(GPIO1_0) |          \
                                     PIN_OMODE_DISABLED(GPIO1_1) |          \
                                     PIN_OMODE_DISABLED(GPIO1_2) |          \
                                     PIN_OMODE_DISABLED(GPIO1_3) |          \
                                     PIN_OMODE_DISABLED(GPIO1_4) |          \
                                     PIN_OMODE_DISABLED(GPIO1_5) |          \
                                     PIN_OMODE_DISABLED(GPIO1_6) |          \
                                     PIN_OMODE_DISABLED(GPIO1_7))
#define VAL_GPIO1_OUTVAL            (PIN_OV_LOW(GPIO1_0) |                  \
                                     PIN_OV_LOW(GPIO1_1) |                  \
                                     PIN_OV_LOW(GPIO1_2) |                  \
                                     PIN_OV_LOW(GPIO1_3) |                  \
                                     PIN_OV_LOW(GPIO1_4) |                  \
                                     PIN_OV_LOW(GPIO1_5) |                  \
                                     PIN_OV_LOW(GPIO1_6) |                  \
                                     PIN_OV_LOW(GPIO1_7))
#define VAL_GPIO1_FUNCSEL           (PIN_FUNC_GPIO(GPIO1_0) |               \
                                     PIN_FUNC_GPIO(GPIO1_1) |               \
                                     PIN_FUNC_GPIO(GPIO1_2) |               \
                                     PIN_FUNC_GPIO(GPIO1_3) |               \
                                     PIN_FUNC_GPIO(GPIO1_4) |               \
                                     PIN_FUNC_GPIO(GPIO1_5) |               \
                                     PIN_FUNC_GPIO(GPIO1_6) |               \
                                     PIN_FUNC_GPIO(GPIO1_7))
#define VAL_GPIO1_INMODE            (PIN_IMODE_NORMAL_INPUT(GPIO1_0) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO1_1) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO1_2) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO1_3) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO1_4) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO1_5) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO1_6) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO1_7))
                                     
/*
 * GPIO2 setup:
 *
 * GPIO2.0  - DAPLINK_RX            High-Z, input disabled.
 * GPIO2.1  - DAPLINK_TX            High-Z, input disabled.
 * GPIO2.2  - BUTTON2               Normal input.
 * GPIO2.3  - BUTTON3               Normal input.
 * GPIO2.4  - PIN4                  High-Z, input disabled.
 * GPIO2.5  - PIN5                  High-Z, input disabled.
 * GPIO2.6  - PIN6                  High-Z, input disabled. 
 * GPIO2.7  - PIN6                  High-Z, input disabled.
 */
#define VAL_GPIO2_OUTMODE           (PIN_OMODE_DISABLED(GPIO2_0) |          \
                                     PIN_OMODE_DISABLED(GPIO2_1) |          \
                                     PIN_OMODE_NORMAL_HIGHZ(GPIO2_2) |      \
                                     PIN_OMODE_NORMAL_HIGHZ(GPIO2_3) |      \
                                     PIN_OMODE_DISABLED(GPIO2_4) |          \
                                     PIN_OMODE_DISABLED(GPIO2_5) |          \
                                     PIN_OMODE_DISABLED(GPIO2_6) |          \
                                     PIN_OMODE_DISABLED(GPIO2_7))
#define VAL_GPIO2_OUTVAL            (PIN_OV_LOW(GPIO2_0) |                  \
                                     PIN_OV_LOW(GPIO2_1) |                  \
                                     PIN_OV_LOW(GPIO2_2) |                  \
                                     PIN_OV_LOW(GPIO2_3) |                  \
                                     PIN_OV_LOW(GPIO2_4) |                  \
                                     PIN_OV_LOW(GPIO2_5) |                  \
                                     PIN_OV_LOW(GPIO2_6) |                  \
                                     PIN_OV_LOW(GPIO2_7))
#define VAL_GPIO2_FUNCSEL           (PIN_FUNC_GPIO(GPIO2_0) |               \
                                     PIN_FUNC_GPIO(GPIO2_1) |               \
                                     PIN_FUNC_GPIO(GPIO2_2) |               \
                                     PIN_FUNC_GPIO(GPIO2_3) |               \
                                     PIN_FUNC_GPIO(GPIO2_4) |               \
                                     PIN_FUNC_GPIO(GPIO2_5) |               \
                                     PIN_FUNC_GPIO(GPIO2_6) |               \
                                     PIN_FUNC_GPIO(GPIO2_7))
#define VAL_GPIO2_INMODE            (PIN_IMODE_NORMAL_INPUT(GPIO2_0) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO2_1) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO2_2) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO2_3) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO2_4) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO2_5) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO2_6) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO2_7))
                                   
/*
 * GPIO3 setup:
 *
 * GPIO3.0  - LED_RED               Fast output, high state.
 * GPIO3.1  - LED_GREEN             Fast output, high state.
 * GPIO3.2  - LED_BLUE              Fast output, high state.
 * GPIO3.3  - LED_ORANGE            Fast output, high state.
 * GPIO3.4  - PIN4                  High-Z, input disabled.
 * GPIO3.5  - PIN5                  High-Z, input disabled.
 * GPIO3.6  - PIN6                  High-Z, input disabled. 
 * GPIO3.7  - PIN6                  High-Z, input disabled.
 */
#define VAL_GPIO3_OUTMODE           (PIN_OMODE_FAST_DRIVE(GPIO3_0) |        \
                                     PIN_OMODE_FAST_DRIVE(GPIO3_1) |        \
                                     PIN_OMODE_FAST_DRIVE(GPIO3_2) |        \
                                     PIN_OMODE_FAST_DRIVE(GPIO3_3) |        \
                                     PIN_OMODE_DISABLED(GPIO3_4) |          \
                                     PIN_OMODE_DISABLED(GPIO3_5) |          \
                                     PIN_OMODE_DISABLED(GPIO3_6) |          \
                                     PIN_OMODE_DISABLED(GPIO3_7))
#define VAL_GPIO3_OUTVAL            (PIN_OV_HIGH(GPIO3_0) |                 \
                                     PIN_OV_HIGH(GPIO3_1) |                 \
                                     PIN_OV_HIGH(GPIO3_2) |                 \
                                     PIN_OV_HIGH(GPIO3_3) |                 \
                                     PIN_OV_LOW(GPIO3_4) |                  \
                                     PIN_OV_LOW(GPIO3_5) |                  \
                                     PIN_OV_LOW(GPIO3_6) |                  \
                                     PIN_OV_LOW(GPIO3_7))
#define VAL_GPIO3_FUNCSEL           (PIN_FUNC_GPIO(GPIO3_0) |               \
                                     PIN_FUNC_GPIO(GPIO3_1) |               \
                                     PIN_FUNC_GPIO(GPIO3_2) |               \
                                     PIN_FUNC_GPIO(GPIO3_3) |               \
                                     PIN_FUNC_GPIO(GPIO3_4) |               \
                                     PIN_FUNC_GPIO(GPIO3_5) |               \
                                     PIN_FUNC_GPIO(GPIO3_6) |               \
                                     PIN_FUNC_GPIO(GPIO3_7))
#define VAL_GPIO3_INMODE            (PIN_IMODE_NORMAL_INPUT(GPIO3_0) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO3_1) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO3_2) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO3_3) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO3_4) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO3_5) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO3_6) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO3_7))
                                   
/*
 * GPIO4 setup:
 *
 * GPIO4.0  - PIN0                  High-Z, input disabled.
 * GPIO4.1  - PIN1                  High-Z, input disabled.
 * GPIO4.2  - PIN2                  High-Z, input disabled.
 * GPIO4.3  - PIN3                  High-Z, input disabled.
 * GPIO4.4  - PIN4                  High-Z, input disabled.
 * GPIO4.5  - PIN5                  High-Z, input disabled.
 * GPIO4.6  - PIN6                  High-Z, input disabled. 
 * GPIO4.7  - PIN6                  High-Z, input disabled.
 */
#define VAL_GPIO4_OUTMODE           (PIN_OMODE_DISABLED(GPIO4_0) |          \
                                     PIN_OMODE_DISABLED(GPIO4_1) |          \
                                     PIN_OMODE_DISABLED(GPIO4_2) |          \
                                     PIN_OMODE_DISABLED(GPIO4_3) |          \
                                     PIN_OMODE_DISABLED(GPIO4_4) |          \
                                     PIN_OMODE_DISABLED(GPIO4_5) |          \
                                     PIN_OMODE_DISABLED(GPIO4_6) |          \
                                     PIN_OMODE_DISABLED(GPIO4_7))
#define VAL_GPIO4_OUTVAL            (PIN_OV_LOW(GPIO4_0) |                  \
                                     PIN_OV_LOW(GPIO4_1) |                  \
                                     PIN_OV_LOW(GPIO4_2) |                  \
                                     PIN_OV_LOW(GPIO4_3) |                  \
                                     PIN_OV_LOW(GPIO4_4) |                  \
                                     PIN_OV_LOW(GPIO4_5) |                  \
                                     PIN_OV_LOW(GPIO4_6) |                  \
                                     PIN_OV_LOW(GPIO4_7))
#define VAL_GPIO4_FUNCSEL           (PIN_FUNC_GPIO(GPIO4_0) |               \
                                     PIN_FUNC_GPIO(GPIO4_1) |               \
                                     PIN_FUNC_GPIO(GPIO4_2) |               \
                                     PIN_FUNC_GPIO(GPIO4_3) |               \
                                     PIN_FUNC_GPIO(GPIO4_4) |               \
                                     PIN_FUNC_GPIO(GPIO4_5) |               \
                                     PIN_FUNC_GPIO(GPIO4_6) |               \
                                     PIN_FUNC_GPIO(GPIO4_7))
#define VAL_GPIO4_INMODE            (PIN_IMODE_NORMAL_INPUT(GPIO4_0) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO4_1) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO4_2) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO4_3) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO4_4) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO4_5) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO4_6) |      \
                                     PIN_IMODE_NORMAL_INPUT(GPIO4_7))
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
