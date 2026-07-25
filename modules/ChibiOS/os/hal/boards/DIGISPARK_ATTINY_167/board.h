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

#ifndef BOARD_H
#define BOARD_H

/*
 * Setup for the Digispark ATtiny 167  board.
 */

/*
 * Board identifier.
 */
#define BOARD_NAME "Digispark Pro board ATtiny167"

/* All inputs.  */
#define VAL_DDRA  0x00
#define VAL_PORTA 0xFF

/* All inputs except PB1 which has a LED connected. */
#define VAL_DDRB  0x02
#define VAL_PORTB 0xFF

#define PORTB_LED1       1

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
