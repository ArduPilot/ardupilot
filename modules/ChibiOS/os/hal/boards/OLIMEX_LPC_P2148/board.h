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
 * Setup for the Olimex LPC-P2148 proto board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_LPC_P2148
#define BOARD_NAME "Olimex LPC-P2148"

/*
 * The following values are implementation dependent. You may change them in
 * order to match your HW.
 */
#define FOSC 12000000
#define CCLK 48000000
#define PCLK 12000000

/*
 * Pins configuration for Olimex LPC-P2148.
 *
 * PINSEL0
 *  P0  P0  P0  P0  P0  P0  RXD TXD SSE MOS MIS SCK SDA SCL RXD TXD
 *  15  14  13  12  11  10  1   1   L0  I0  O0  0   0   0   0   0
 *  00  00  00  00  00  00  01  01  01  01  01  01  01  01  01  01
 * FIO0DIR (15...0)
 *  IN  IN  OUT OUT OUT OUT --  --  --  --  --  --  --  --  --  --
 *  0   0   1   1   1   1   0   0   0   0   0   0   0   0   0   0
 *
 * PINSEL1
 *  P0  AD  P0  P0  --  --  AO  --  VB  P0  P0  P0  MOS MIS SCK P0
 *  31  03  29  28  --  --  UT  --  US  22  21  20  I1  O1  1   16
 *  00  01  00  00  00  00  10  00  01  00  00  00  10  10  10  00
 * FIO0DIR (31...16)
 *  OUT --  OUT OUT --  --  --  --  --  OUT OUT OUT --  --  --  IN
 *  1   0   1   1   0   0   0   0   0   1   1   1   0   0   0   0
 *
 * FIO1DIR (31...16)
 *  --  --  --  --  --  IN  IN  OUT OUT OUT OUT OUT OUT OUT OUT OUT
 *  0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1
 */
#define VAL_PINSEL0 0x00055555
#define VAL_PINSEL1 0x100840A8
#define VAL_PINSEL2 0x00000004  /* Do not modify */
#define VAL_FIO0DIR 0xB0703C00
#define VAL_FIO1DIR 0x01FF0000
#define VAL_FIO0PIN 0xFFFFFFFF
#define VAL_FIO1PIN 0xFFFFFFFF

#define PA_LED1         10
#define PA_LED2         11
#define PA_BUZZ1        12
#define PA_BUZZ2        13
#define PA_BSL          14
#define PA_BUTTON1      15
#define PA_BUTTON2      16
#define PA_SSEL1        20
#define PA_LEDUSB       31

#define PB_WP1          24
#define PB_CP1          25

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
