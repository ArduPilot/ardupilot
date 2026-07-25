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
 * Setup for a generic SPC560B/Cxx proto board.
 */

/*
 * Board identifiers.
 */
#define BOARD_GENERIC_SPC560BC
#define BOARD_NAME                  "Generic SPC560B/Cxx"

/*
 * Board frequencies.
 */
#if !defined(SPC5_XOSC_CLK)
#define SPC5_XOSC_CLK               8000000
#endif

/*
 * I/O definitions.
 */
#define PB_LIN0_TDX     2
#define PB_LIN0_RDX     3

#define PE_BUTTON1      0
#define PE_BUTTON2      1
#define PE_BUTTON3      2
#define PE_BUTTON4      3

#define PE_LED1         4
#define PE_LED2         5
#define PE_LED3         6
#define PE_LED4         7

/*
 * Support macros.
 */
#define PCR(port, pin)  (((port) * 16) + (pin))

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
