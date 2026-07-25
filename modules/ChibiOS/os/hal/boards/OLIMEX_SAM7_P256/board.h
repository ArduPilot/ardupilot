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
 * Setup for the Olimex SAM7-P256 development board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_SAM7_P256

/*
 * Select your platform by modifying the following line.
 */
#if !defined(SAM7_PLATFORM)
#define SAM7_PLATFORM   SAM7S256
#endif

#include "at91sam7.h"

#define CLK             18432000
#define MCK             48054857

/*
 * Initial I/O setup.
 */
#define VAL_PIOA_ODSR           0x00000000      /* Output data. */
#define VAL_PIOA_OSR            0x00000000      /* Direction. */
#define VAL_PIOA_PUSR           0xFFFFFFFF      /* Pull-up. */

/*
 * I/O definitions.
 */
#define PIOA_LED1               18
#define PIOA_LED1_MASK          (1 << PIOA_LED1_MASK)
#define PIOA_LED2               17
#define PIOA_LED2_MASK          (1 << PIOA_LED2_MASK)
#define PIOA_B1                 19
#define PIOA_B1_MASK            (1 << PIOA_B1)
#define PIOA_B2                 20
#define PIOA_B2_MASK            (1 << PIOA_B2)
#define PIOA_DP_PUP             25
#define PIOA_DD_PUP_MASK        (1 << PIOA_DP_PUP)
#define PIOA_USB_D              26
#define PIOA_USB_D_MASK         (1 << PIOA_USB_D)

#define PIOA_MMC_WP             25
#define PIOA_MMC_WP_MASK        (1 << PIOA_MMC_WP)
#define PIOA_MMC_CP             15
#define PIOA_MMC_CP_MASK        (1 << PIOA_MMC_CP)
#define PIOA_MMC_NPCS0          11
#define PIOA_MMC_NPCS0_MASK     (1 << PIOA_MMC_NPCS0_MASK)

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
