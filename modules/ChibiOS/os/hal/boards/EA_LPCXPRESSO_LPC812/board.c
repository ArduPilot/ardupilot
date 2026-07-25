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

#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config = {VAL_GPIO0DATA, VAL_GPIO0DIR};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void){

  lpc8xx_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void){

  /* Enable clocks to IOCON & SWM */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((1<<18)|(1<<7));
   
#if defined VAL_PIO0_0
  LPC_IOCON->PIO0_0 =  PIN_RSVD|VAL_PIO0_0;
#endif
#if defined VAL_PIO0_1
  LPC_IOCON->PIO0_1 =  PIN_RSVD|VAL_PIO0_1;
#endif
#if defined VAL_PIO0_2
  LPC_IOCON->PIO0_2 =  PIN_RSVD|VAL_PIO0_2;
#endif
#if defined VAL_PIO0_3
  LPC_IOCON->PIO0_3 =  PIN_RSVD|VAL_PIO0_3;
#endif
#if defined VAL_PIO0_4
  LPC_IOCON->PIO0_4 =  PIN_RSVD|VAL_PIO0_4;
#endif
#if defined VAL_PIO0_5
  LPC_IOCON->PIO0_5 =  PIN_RSVD|VAL_PIO0_5;
#endif
#if defined VAL_PIO0_6
  LPC_IOCON->PIO0_6 =  PIN_RSVD|VAL_PIO0_6;
#endif
#if defined VAL_PIO0_7
  LPC_IOCON->PIO0_7 =  PIN_RSVD|VAL_PIO0_7;
#endif
#if defined VAL_PIO0_8
  LPC_IOCON->PIO0_8 =  PIN_RSVD|VAL_PIO0_8;
#endif
#if defined VAL_PIO0_9
  LPC_IOCON->PIO0_9 =  PIN_RSVD|VAL_PIO0_9;
#endif
#if defined VAL_PIO0_10
  LPC_IOCON->PIO0_10 =  PIN_RSVD|VAL_PIO0_10;
#endif
#if defined VAL_PIO0_11
  LPC_IOCON->PIO0_11 =  PIN_RSVD|VAL_PIO0_11;
#endif
#if defined VAL_PIO0_12
  LPC_IOCON->PIO0_12 =  PIN_RSVD|VAL_PIO0_12;
#endif
#if defined VAL_PIO0_13
  LPC_IOCON->PIO0_13 =  PIN_RSVD|VAL_PIO0_13;
#endif
#if defined VAL_PIO0_14
  LPC_IOCON->PIO0_14 =  PIN_RSVD|VAL_PIO0_14;
#endif
#if defined VAL_PIO0_15
  LPC_IOCON->PIO0_15 =  PIN_RSVD|VAL_PIO0_15;
#endif
#if defined VAL_PIO0_16
  LPC_IOCON->PIO0_16 =  PIN_RSVD|VAL_PIO0_16;
#endif
#if defined VAL_PIO0_17
  LPC_IOCON->PIO0_17 =  PIN_RSVD|VAL_PIO0_17;
#endif


#if defined VAL_PINASSIGN0
  LPC_SWM->PINASSIGN0 = VAL_PINASSIGN0;
#endif
#if defined VAL_PINASSIGN1
  LPC_SWM->PINASSIGN1 = VAL_PINASSIGN1;
#endif
#if defined VAL_PINASSIGN2
  LPC_SWM->PINASSIGN2 = VAL_PINASSIGN2;
#endif
#if defined VAL_PINASSIGN3
  LPC_SWM->PINASSIGN3 = VAL_PINASSIGN3;
#endif
#if defined VAL_PINASSIGN4
  LPC_SWM->PINASSIGN4 = VAL_PINASSIGN4;
#endif
#if defined VAL_PINASSIGN5
  LPC_SWM->PINASSIGN5 = VAL_PINASSIGN5;
#endif
#if defined VAL_PINASSIGN6
  LPC_SWM->PINASSIGN6 = VAL_PINASSIGN6;
#endif
#if defined VAL_PINASSIGN7
  LPC_SWM->PINASSIGN7 = VAL_PINASSIGN7;
#endif
#if defined VAL_PINASSIGN8
  LPC_SWM->PINASSIGN8 = VAL_PINASSIGN8;
#endif

  /* Disable clocks to IOCON & SWM */
  LPC_SYSCON->SYSAHBCLKCTRL &= ~((1<<18)|(1<<7));

}


