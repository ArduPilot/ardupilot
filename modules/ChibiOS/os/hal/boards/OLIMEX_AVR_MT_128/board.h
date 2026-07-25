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
 * Setup for the Olimex AVR-MT-128 proto board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_AVR_MT_128
#define BOARD_NAME "Olimex AVR-MT-128"

/*       PA7 RLY  DS  B5  B4  B3  B2  B1
 *        IN OUT  IN  IN  IN  IN  IN  IN
 * DDRA    0   1   0   0   0   0   0   0
 *        PU VAL HiZ HiZ HiZ HiZ HiZ HiZ
 * PORTA   1   0   0   0   0   0   0   0
 */
#define VAL_DDRA  0x40
#define VAL_PORTA 0x80

/*
 * All inputs with pullups.
 */
#define VAL_DDRB  0x00
#define VAL_PORTB 0xFF

/*        D7  D6  D5  D4 PC3   E R/W  RS
 *        OUT OUT OUT OUT IN OUT OUT OUT
 * DDRC    1   1   1   1   0   1   1   1
 *        PU  PU  PU  PU  PU VAL VAL VAL
 * PORTC   0   0   0   0   1   0   0   0
 */
#define VAL_DDRC  0xF7
#define VAL_PORTC 0x08

/*       PD7 PD6 PD5 PD4 TXD RXD PD1 PD0
 *        IN  IN  IN  IN OUT  IN  IN  IN
 * DDRD    0   0   0   0   1   0   0   0
 *        PU  PU  PU  PU VAL HiZ  PU  PU
 * PORTD   1   1   1   1   1   0   1   1
 */
#define VAL_DDRD  0x08
#define VAL_PORTD 0xFB

/*       PE7 PE6 BZ2 BZ2 PE3 PE2 PE1 PE0
 *        IN  IN OUT OUT  IN  IN OUT  IN
 * DDRE    0   0   1   1   0   0   1   0
 *        PU  PU VAL VAL  PU  PU VAL  PU
 * PORTE   1   1   1   1   1   1   1   1
 */
#define VAL_DDRE  0x32
#define VAL_PORTE 0xFF

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


#define PORTA_BUTTON1           0
#define PORTA_BUTTON2           1
#define PORTA_BUTTON3           2
#define PORTA_BUTTON4           3
#define PORTA_BUTTON5           4
#define PORTA_DALLAS            5
#define PORTA_RELAY             6

#define PORTC_44780_RS_MASK     (1 << 0)
#define PORTC_44780_RW_MASK     (1 << 1)
#define PORTC_44780_E_MASK      (1 << 2)
#define PORTC_44780_D4_MASK     (1 << 4)
#define PORTC_44780_D5_MASK     (1 << 5)
#define PORTC_44780_D6_MASK     (1 << 6)
#define PORTC_44780_D7_MASK     (1 << 7)
#define PORTC_44780_DATA_MASK   (PORTC_44780_D4_MASK | PORTC_44780_D5_MASK | \
                                 PORTC_44780_D6_MASK | PORTC_44780_D7_MASK)

#define PORTE_BUZZ1             (1 << 4)
#define PORTE_BUZZ2             (1 << 5)

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
