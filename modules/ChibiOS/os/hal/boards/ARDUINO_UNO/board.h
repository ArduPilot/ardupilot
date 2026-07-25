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
 * Setup for the Arduino Uno or  board.
 */

/*
 * Board identifier.
 */
#define BOARD_ARDUINO_UNO
#define BOARD_NAME "Arduino Uno"

/*
 * IO pins assignments.
 */
#define PORTB_LED1       5

/*
 * IO lines assignments.
 */
#define LINE_LED1   PAL_LINE(IOPORT2, 5U)

/*
 * Port B setup.
 * All inputs except PB5 which has a LED connected.
 */
#define VAL_DDRB  0x20
#define VAL_PORTB 0xFF

/*
 * Port C setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRC  0x00
#define VAL_PORTC 0xFF

/*
 * Port D setup.
 * All inputs except PD1 (Serial TX0).
 */
#define VAL_DDRD  0x02
#define VAL_PORTD 0xFF

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
