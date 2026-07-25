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
 * Setup for Analog Devices ADICUP360 board.
 */

/*
 * Board identifier.
 */
#define BOARD_ADICUP360
#define BOARD_NAME                  "Analog Devices ADICUP360"

/*
 * Board oscillators-related settings.
 */
#if !defined(ADUCM_LFXTAL)
#define ADUCM_LFXTAL                32768U
#endif

/*
 * MCU type as defined in the ADI header.
 */
#define ADUCM360

/*
 * IO pins assignments.
 */
#define GP0_0                       0U
#define GP0_1                       1U
#define GP0_2                       2U
#define GP0_3                       3U
#define GP0_LED_BLUE                4U
#define GP0_LED_GREEN               5U
#define GP0_6                       6U
#define GP0_7                       7U
#define GP0_NPOR                    7U

#define GP1_0                       0U
#define GP1_1                       1U
#define GP1_2                       2U
#define GP1_3                       3U
#define GP1_4                       4U
#define GP1_5                       5U
#define GP1_6                       6U
#define GP1_7                       7U

#define GP2_0                       0U
#define GP2_1                       1U
#define GP2_BUTTON                  2U
#define GP2_SWCLK                   3U
#define GP2_SWD                     4U
#define GP2_5                       5U
#define GP2_6                       6U
#define GP2_7                       7U

/*
 * IO lines assignments.
 */
#define LINE_LED_BLUE               PAL_LINE(GP0, 4U)
#define LINE_LED_GREEN              PAL_LINE(GP0, 5U)
#define LINE_NPOR                   PAL_LINE(GP0, 7U)
#define LINE_BUTTON                 PAL_LINE(GP2, 2U)
#define LINE_SWCLK                  PAL_LINE(GP2, 3U)
#define LINE_SWD                    PAL_LINE(GP2, 4U)

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
 * Please refer to the ADUCM360 Reference Manual for details.
 */
#define PIN_CON_CON(n, v)           ((v) << (n * 2U))
#define PIN_OEN_INPUT(n)            (0U << (n))
#define PIN_OEN_OUTPUT(n)           (1U << (n))
#define PIN_PUL_FLOATING(n)         (0U << (n))
#define PIN_PUL_PULLUP(n)           (1U << (n))
#define PIN_OCE_NORMAL(n)           (0U << (n))
#define PIN_OCE_HIGHZ(n)            (1U << (n))

/*
 * P0 setup:
 *
 * P0.0  - PIN0                     (CON0, GPIO - input pullup).
 * P0.1  - PIN1                     (CON0, GPIO - input pullup).
 * P0.2  - PIN2                     (CON0, GPIO - input pullup).
 * P0.3  - PIN3                     (CON0, GPIO - input pullup).
 * P0.4  - LED_BLUE                 (CON0, GPIO - output floating).
 * P0.5  - LED_GREEN                (CON0, GPIO - output floating).
 * P0.6  - PIN6                     (CON0, GPIO - input pullup).
 * P0.7  - NPOR                     (CON0, NPOR).
 */
#define VAL_GP0CON                  (PIN_CON_CON(GP0_0, 0) |                \
                                     PIN_CON_CON(GP0_1, 0) |                \
                                     PIN_CON_CON(GP0_2, 0) |                \
                                     PIN_CON_CON(GP0_3, 0) |                \
                                     PIN_CON_CON(GP0_LED_BLUE, 0) |         \
                                     PIN_CON_CON(GP0_LED_GREEN, 0) |        \
                                     PIN_CON_CON(GP0_6, 0) |                \
                                     PIN_CON_CON(GP0_NPOR, 0))
#define VAL_GP0OEN                  (PIN_OEN_INPUT(GP0_0) |                 \
                                     PIN_OEN_INPUT(GP0_1) |                 \
                                     PIN_OEN_INPUT(GP0_2) |                 \
                                     PIN_OEN_INPUT(GP0_3) |                 \
                                     PIN_OEN_OUTPUT(GP0_LED_BLUE) |         \
                                     PIN_OEN_OUTPUT(GP0_LED_GREEN) |        \
                                     PIN_OEN_INPUT(GP0_6) |                 \
                                     PIN_OEN_INPUT(GP0_NPOR))
#define VAL_GP0PUL                  (PIN_PUL_PULLUP(GP0_0) |                \
                                     PIN_PUL_PULLUP(GP0_1) |                \
                                     PIN_PUL_PULLUP(GP0_2) |                \
                                     PIN_PUL_PULLUP(GP0_3) |                \
                                     PIN_PUL_FLOATING(GP0_LED_BLUE) |       \
                                     PIN_PUL_FLOATING(GP0_LED_GREEN) |      \
                                     PIN_PUL_PULLUP(GP0_6) |                \
                                     PIN_PUL_PULLUP(GP0_NPOR))
#define VAL_GP0OCE                  (PIN_OCE_NORMAL(GP0_0) |                \
                                     PIN_OCE_NORMAL(GP0_1) |                \
                                     PIN_OCE_NORMAL(GP0_2) |                \
                                     PIN_OCE_NORMAL(GP0_3) |                \
                                     PIN_OCE_NORMAL(GP0_LED_BLUE) |         \
                                     PIN_OCE_NORMAL(GP0_LED_GREEN) |        \
                                     PIN_OCE_NORMAL(GP0_6) |                \
                                     PIN_OCE_NORMAL(GP0_NPOR))

/*
 * P1 setup:
 *
 * P1.0  - PIN0                     (CON0, GPIO - input pullup).
 * P1.1  - PIN1                     (CON0, GPIO - input pullup).
 * P1.2  - PIN2                     (CON0, GPIO - input pullup).
 * P1.3  - PIN3                     (CON0, GPIO - input pullup).
 * P1.4  - PIN4                     (CON0, GPIO - input pullup).
 * P1.5  - PIN5                     (CON0, GPIO - input pullup).
 * P1.6  - PIN6                     (CON0, GPIO - input pullup).
 * P1.7  - PIN7                     (CON0, GPIO - input pullup).
 */
#define VAL_GP1CON                  (PIN_CON_CON(GP1_0, 0) |                \
                                     PIN_CON_CON(GP1_1, 0) |                \
                                     PIN_CON_CON(GP1_2, 0) |                \
                                     PIN_CON_CON(GP1_3, 0) |                \
                                     PIN_CON_CON(GP1_4, 0) |                \
                                     PIN_CON_CON(GP1_5, 0) |                \
                                     PIN_CON_CON(GP1_6, 0) |                \
                                     PIN_CON_CON(GP1_7, 0))                                     
#define VAL_GP1OEN                  (PIN_OEN_INPUT(GP1_0) |                 \
                                     PIN_OEN_INPUT(GP1_1) |                 \
                                     PIN_OEN_INPUT(GP1_2) |                 \
                                     PIN_OEN_INPUT(GP1_3) |                 \
                                     PIN_OEN_INPUT(GP1_4) |                 \
                                     PIN_OEN_INPUT(GP1_5) |                 \
                                     PIN_OEN_INPUT(GP1_6) |                 \
                                     PIN_OEN_INPUT(GP1_7))                                     
#define VAL_GP1PUL                  (PIN_PUL_PULLUP(GP1_0) |                \
                                     PIN_PUL_PULLUP(GP1_1) |                \
                                     PIN_PUL_PULLUP(GP1_2) |                \
                                     PIN_PUL_PULLUP(GP1_3) |                \
                                     PIN_PUL_PULLUP(GP1_4) |                \
                                     PIN_PUL_PULLUP(GP1_5) |                \
                                     PIN_PUL_PULLUP(GP1_6) |                \
                                     PIN_PUL_PULLUP(GP1_7))
#define VAL_GP1OCE                  (PIN_OCE_NORMAL(GP1_0) |                \
                                     PIN_OCE_NORMAL(GP1_1) |                \
                                     PIN_OCE_NORMAL(GP1_2) |                \
                                     PIN_OCE_NORMAL(GP1_3) |                \
                                     PIN_OCE_NORMAL(GP1_4) |                \
                                     PIN_OCE_NORMAL(GP1_5) |                \
                                     PIN_OCE_NORMAL(GP1_6) |                \
                                     PIN_OCE_NORMAL(GP1_7))

/*
 * P2 setup:
 *
 * P2.0  - PIN0                     (CON0, GPIO - input pullup).
 * P2.1  - PIN1                     (CON0, GPIO - input pullup).
 * P2.2  - BUTTON                   (CON0, GPIO - input floating).
 * P2.3  - SWCLK                    (CON0, SWCLK).
 * P2.4  - SWD                      (CON0, SWD).
 * P2.5  - PIN5                     (CON0, GPIO - input pullup).
 * P2.6  - PIN6                     (CON0, GPIO - input pullup).
 * P2.7  - PIN7                     (CON0, GPIO - input pullup).
 */
#define VAL_GP2CON                  (PIN_CON_CON(GP2_0, 0) |                \
                                     PIN_CON_CON(GP2_1, 0) |                \
                                     PIN_CON_CON(GP2_BUTTON, 0) |           \
                                     PIN_CON_CON(GP2_SWCLK, 0) |            \
                                     PIN_CON_CON(GP2_SWD, 0) |              \
                                     PIN_CON_CON(GP2_5, 0) |                \
                                     PIN_CON_CON(GP2_6, 0) |                \
                                     PIN_CON_CON(GP2_7, 0))
#define VAL_GP2OEN                  (PIN_OEN_INPUT(GP2_0) |                 \
                                     PIN_OEN_INPUT(GP2_1) |                 \
                                     PIN_OEN_INPUT(GP2_BUTTON) |            \
                                     PIN_OEN_INPUT(GP2_SWCLK) |             \
                                     PIN_OEN_INPUT(GP2_SWD) |               \
                                     PIN_OEN_INPUT(GP2_5) |                 \
                                     PIN_OEN_INPUT(GP2_6) |                 \
                                     PIN_OEN_INPUT(GP2_7))
#define VAL_GP2PUL                  (PIN_PUL_PULLUP(GP2_0) |                \
                                     PIN_PUL_PULLUP(GP2_1) |                \
                                     PIN_PUL_FLOATING(GP2_BUTTON) |         \
                                     PIN_PUL_PULLUP(GP2_SWCLK) |            \
                                     PIN_PUL_PULLUP(GP2_SWD) |              \
                                     PIN_PUL_PULLUP(GP2_5) |                \
                                     PIN_PUL_PULLUP(GP2_6) |                \
                                     PIN_PUL_PULLUP(GP2_7))
#define VAL_GP2OCE                  (PIN_OCE_NORMAL(GP2_0) |                \
                                     PIN_OCE_NORMAL(GP2_1) |                \
                                     PIN_OCE_NORMAL(GP2_BUTTON) |           \
                                     PIN_OCE_NORMAL(GP2_SWCLK) |            \
                                     PIN_OCE_NORMAL(GP2_SWD) |              \
                                     PIN_OCE_NORMAL(GP2_5) |                \
                                     PIN_OCE_NORMAL(GP2_6) |                \
                                     PIN_OCE_NORMAL(GP2_7))

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
