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
 * Setup for the Olimex SAM7-EX256 development board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_SAM7_EX256
#define BOARD_NAME "Olimex SAM7-EX256"

/*
 * Select your platform by modifying the following line.
 */
#if !defined(SAM7_PLATFORM)
#define SAM7_PLATFORM   SAM7X256
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

#define VAL_PIOB_ODSR           0x00000000      /* Output data. */
#define VAL_PIOB_OSR            0x00000000      /* Direction. */
#define VAL_PIOB_PUSR           0xFFFFFFFF      /* Pull-up. */

/*
 * I/O definitions.
 */
#define PIOA_LCD_RESET          2
#define PIOA_LCD_RESET_MASK     (1 << PIOA_LCD_RESET)
#define PIOA_B1                 7
#define PIOA_B1_MASK            (1 << PIOA_B1)
#define PIOA_B2                 8
#define PIOA_B2_MASK            (1 << PIOA_B2)
#define PIOA_B3                 9
#define PIOA_B3_MASK            (1 << PIOA_B3)
#define PIOA_B4                 14
#define PIOA_B4_MASK            (1 << PIOA_B4)
#define PIOA_B5                 15
#define PIOA_B5_MASK            (1 << PIOA_B5)
#define PIOA_USB_PUP            25
#define PIOA_USB_PUP_MASK       (1 << PIOA_USB_PUP)
#define PIOA_USB_PR             26
#define PIOA_USB_PR_MASK        (1 << PIOA_USB_PR)
#define PIOA_CS_MMC             13

#define PIOB_PHY_PD             18
#define PIOB_PHY_PD_MASK        (1 << PIOB_PHY_PD)
#define PIOB_AUDIO_OUT          19
#define PIOB_AUDIO_OUT_MASK     (1 << PIOB_AUDIO_OUT)
#define PIOB_LCD_BL             20
#define PIOB_LCD_BL_MASK        (1 << PIOB_LCD_BL)
#define PIOB_MMC_WP             22
#define PIOB_MMC_WP_MASK        (1 << PIOB_MMC_WP)
#define PIOB_MMC_CP             23
#define PIOB_MMC_CP_MASK        (1 << PIOB_MMC_CP)
#define PIOB_SW1                24
#define PIOB_SW1_MASK           (1 << PIOB_SW1)
#define PIOB_SW2                25
#define PIOB_SW2_MASK           (1 << PIOB_SW2)
#define PIOB_PHY_IRQ            26
#define PIOB_PHY_IRQ_MASK       (1 << PIOB_PHY_IRQ)

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
