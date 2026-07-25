/*
    ChibiOS - Copyright (C) 2017 Theodore Ateba

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
 * Setup for the Mattair board.
 */

/*
 * Board identifier.
 */
#define BOARD_MT_DB_X4
#define BOARD_NAME  "MT-DB-X4 board"

/* All inputs with pull-ups */
#define VAL_DDRA    0x00
#define VAL_PORTA   0xFF

/* All inputs. */
#define VAL_DDRB    0x00
#define VAL_PORTB   0xFF

/* All inputs with pull-ups */
#define VAL_DDRC    0x00
#define VAL_PORTC   0xFF

/* All inputs with pull-ups */
#define VAL_DDRD    0x00
#define VAL_PORTD   0xFF

/* All inputs except PE1 wich has a LED connected. */
#define VAL_DDRE    0x02
#define VAL_PORTE   0xFF

/* All inputs with pull-ups */
#define VAL_DDRF    0x00
#define VAL_PORTF   0xFF

/* All inputs with pull-ups */
#define VAL_DDRG    0x00
#define VAL_PORTG   0xFF

/* All inputs with pull-ups */
#define VAL_DDRH    0x00
#define VAL_PORTH   0xFF

/* All inputs with pull-ups */
#define VAL_DDRJ    0x00
#define VAL_PORTJ   0xFF

/* All inputs with pull-ups */
#define VAL_DDRK    0x00
#define VAL_PORTK   0xFF

/* All inputs with pull-ups */
#define VAL_DDRL    0x00
#define VAL_PORTL   0xFF

#define PORTE_LED   1

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
