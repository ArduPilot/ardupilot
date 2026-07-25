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
 * Setup for the Studiel AT91SAM7A3-EK board.
 */

/*
 * Board identifier.
 */
#define BOARD_STUDIEL_AT91SAM7A3_EK
#define BOARD_NAME "Studiel AT91SAM7A3-EK eval. board"

/*
 * Select your platform by modifying the following line.
 */
#if !defined(SAM7A3_PLATFORM)
#define SAM7_PLATFORM   SAM7A3
#endif

#include "at91sam7.h"

#define CLK             18432000
#define MCK             48054857

/*
 * I/O definitions.
 */
#define PIOA_RXD0               2
#define PIOA_RXD0_MASK          (1 << PIOA_RXD0)
#define PIOA_TXD0               3
#define PIOA_TXD0_MASK          (1 << PIOA_TXD0)
#define PIOA_LED1               20
#define PIOA_LED1_MASK          (1 << PIOA_LED1)
#define PIOA_LED2               21
#define PIOA_LED2_MASK          (1 << PIOA_LED2)
#define PIOA_LED3               24
#define PIOA_LED3_MASK          (1 << PIOA_LED3)
#define PIOA_LED4               25
#define PIOA_LED4_MASK          (1 << PIOA_LED4)
// mmc-spi
#define PIOA_SPI0_NSS           14
#define PIOA_SPI0_NSS_MASK      (1 << PIOA_SPI0_NSS)
#define PIOA_SPI0_MISO          15
#define PIOA_SPI0_MISO_MASK     (1 << PIOA_SPI0_MISO)
#define PIOA_SPI0_MOSI          16
#define PIOA_SPI0_MOSI_MASK     (1 << PIOA_SPI0_MOSI)
#define PIOA_SPI0_CLK           17
#define PIOA_SPI0_CLK_MASK      (1 << PIOA_SPI0_CLK)

/*
 * Initial I/O setup.
 */
/* Output data. */
#define VAL_PIOA_ODSR           0x00000000
/* Direction. */
#define VAL_PIOA_OSR            0x00000000 | PIOA_LED1_MASK | PIOA_LED2_MASK | \
                                             PIOA_LED3_MASK | PIOA_LED4_MASK
/* Pull-up. */
#define VAL_PIOA_PUSR           0xFFFFFFFF

#define VAL_PIOB_ODSR           0x00000000      /* Output data. */
#define VAL_PIOB_OSR            0x00000000      /* Direction. */
#define VAL_PIOB_PUSR           0xFFFFFFFF      /* Pull-up. */


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
