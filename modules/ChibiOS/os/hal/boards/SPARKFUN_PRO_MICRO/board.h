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
 * Setup for the PJRC Teensy2++ board.
 */

/*
 * Board identifier.
 */
#define BOARD_SPARKFUN_PRO_MICRO
#define BOARD_NAME "Sparkfun Pro Micro"

/* All inputs with pull-ups */
#define VAL_DDRA  0x00
#define VAL_PORTA 0xFF

/* All inputs with pull-ups */
#define VAL_DDRB  0x00
#define VAL_PORTB 0xFF

/* All inputs with pull-ups */
#define VAL_DDRC  0x00
#define VAL_PORTC 0xFF

/* All inputs with pull-ups, LED on D6, serial TX1 on D3 */
#define VAL_DDRD  0x48
#define VAL_PORTD 0xFF

/* All inputs with pull-ups */
#define VAL_DDRE  0x00
#define VAL_PORTE 0xFF

/* All inputs with pull-ups */
#define VAL_DDRF  0x00
#define VAL_PORTF 0xFF

/* All inputs with pull-ups */
#define VAL_DDRG  0x00
#define VAL_PORTG 0xFF

/* All inputs with pull-ups */
#define VAL_DDRH  0x00
#define VAL_PORTH 0xFF

/* All inputs with pull-ups */
#define VAL_DDRJ  0x00
#define VAL_PORTJ 0xFF

/* All inputs with pull-ups */
#define VAL_DDRK  0x00
#define VAL_PORTK 0xFF

/* All inputs with pull-ups */
#define VAL_DDRL  0x00
#define VAL_PORTL 0xFF

#define BOARD_LED1        5

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
