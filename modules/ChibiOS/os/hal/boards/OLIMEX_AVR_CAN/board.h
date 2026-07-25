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
 * Setup for the Olimex AVR-CAN proto board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_AVR_CAN
#define BOARD_NAME "Olimex AVR-CAN"

/*
 * All inputs with pullups.
 */
#define VAL_DDRA  0x00
#define VAL_PORTA 0xFF

/*
 * All inputs with pullups.
 */
#define VAL_DDRB  0x00
#define VAL_PORTB 0xFF

/*
 * All inputs with pullups.
 */
#define VAL_DDRC  0x00
#define VAL_PORTC 0xFF

/*       PD7 PD6 PD5 PD4 PD3 PD2 PD1 PD0
 *        IN  IN OUT  IN OUT  IN  IN  IN
 * DDRD    0   0   1   0   1   0   0   0
 *        PU HiZ VAL  PU VAL HiZ HiZ HiZ
 * PORTD   1   0  ?1   1   1   0   0   0
 */
#define VAL_DDRD  0x28
#define VAL_PORTD 0xB8

/*       PE7 PE6 BUT LED PE3 PE2 PE1 PE0
 *        IN  IN  IN OUT  IN  IN OUT  IN
 * DDRE    0   0   0   1   0   0   1   0
 *        PU  PU HiZ VAL  PU  PU VAL HiZ
 * PORTE   1   1   0   1   1   1   1   0
 */
#define VAL_DDRE  0x12
#define VAL_PORTE 0xDE

/*       TDI TDO TMS TCK PF3 PF2 PF1 PF0
 *         x   x   x   x  IN  IN  IN  IN
 * DDRF    0   0   0   0   0   0   0   0
 *         x   x   x   x  PU  PU  PU  PU
 * PORTF   0   0   0   0   1   1   1   1
 *
 */
#define VAL_DDRF  0x00
#define VAL_PORTF 0x0F

/*         x   x   x   x   x PG2 PG1 PG0
 *         x   x   x   x   x  IN  IN  IN
 * DDRG    0   0   0   0   0   0   0   0
 *         x   x   x   x   x  PU  PU  PU
 * PORTG   0   0   0   0   0   1   1   1
 *
 */
#define VAL_DDRG  0x00
#define VAL_PORTG 0x07

#define PORTE_LED               4
#define PORTE_BUTTON            5

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
