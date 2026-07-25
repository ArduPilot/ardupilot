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
 * Setup for the Olimex MSP430-P1611 proto board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_MSP430_P1611
#define BOARD_NAME "Olimex MSP430-P1611"

/*
 * Clock constants.
 */
#define LFXT1CLK        32768
#define XT2CLK          8000000
#define DCOCLK          750000

/*
 * Pin definitions for the Olimex MSP430-P1611 board.
 */
#define P3_O_TXD0               4
#define P3_O_TXD0_MASK          (1 << P3_O_TXD0)
#define P3_I_RXD0               5
#define P3_I_RXD0_MASK          (1 << P3_I_RXD0)
#define P6_O_LED                0
#define P6_O_LED_MASK           (1 << P6_O_LED)
#define P6_I_BUTTON             1
#define P6_I_BUTTON_MASK        (1 << P6_I_BUTTON)

/*
 * Initial I/O ports settings.
 */
#define VAL_P1OUT       0x00
#define VAL_P1DIR       0xFF

#define VAL_P2OUT       0x00
#define VAL_P2DIR       0xFF

#define VAL_P3OUT       P3_O_TXD0_MASK
#define VAL_P3DIR       ~P3_I_RXD0_MASK

#define VAL_P4OUT       0x00
#define VAL_P4DIR       0xFF

#define VAL_P5OUT       0x00
#define VAL_P5DIR       0xFF

#define VAL_P6OUT       P6_O_LED_MASK
#define VAL_P6DIR       ~P6_I_BUTTON_MASK

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
