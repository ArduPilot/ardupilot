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
const PALConfig pal_default_config =
{
  {VAL_PIOA_ODSR, VAL_PIOA_OSR, VAL_PIOA_PUSR},
#if (SAM7_PLATFORM == SAM7X128) || (SAM7_PLATFORM == SAM7X256) || \
    (SAM7_PLATFORM == SAM7X512) || (SAM7_PLATFORM == SAM7A3)
  {VAL_PIOB_ODSR, VAL_PIOB_OSR, VAL_PIOB_PUSR}
#endif
};
#endif

/*
 * SYS IRQ handling here.
 */
static CH_IRQ_HANDLER(SYSIrqHandler) {

  CH_IRQ_PROLOGUE();

  if (AT91C_BASE_PITC->PITC_PISR & AT91C_PITC_PITS) {
    (void) AT91C_BASE_PITC->PITC_PIVR;
    chSysLockFromIsr();
    chSysTimerHandlerI();
    chSysUnlockFromIsr();
  }
  AT91C_BASE_AIC->AIC_EOICR = 0;

  CH_IRQ_EPILOGUE();
}

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {
  
  /* Watchdog disabled.*/
  AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;
  
  at91sam7_clock_init();
}

#if HAL_USE_MMC_SPI
/* Board-related functions related to the MMC_SPI driver.*/
bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {

  (void)mmcp;
  return !palReadPad(IOPORT1, PIOA_MMC_CP);
}

bool mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  return palReadPad(IOPORT1, PIOA_MMC_WP);
}
#endif

/*
 * Board-specific initialization code.
 */
void boardInit(void) {

  /*
   * LED pins setup.
   */
  palClearPad(IOPORT1, PIOA_LED1);
  palSetPadMode(IOPORT1, PIOA_LED1, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(IOPORT1, PIOA_LED2);
  palSetPadMode(IOPORT1, PIOA_LED2, PAL_MODE_OUTPUT_PUSHPULL);

  /*
   * buttons setup.
   */
  palSetGroupMode(IOPORT1, PIOA_B1_MASK | PIOA_B2_MASK, 0, PAL_MODE_INPUT);

  /*
   * MMC/SD slot setup.
   */
  palSetGroupMode(IOPORT1,
                  PIOA_MMC_WP_MASK | PIOA_MMC_CP_MASK,
                  0,
                  PAL_MODE_INPUT);

  /*
   * PIT Initialization.
   */
  AIC_ConfigureIT(AT91C_ID_SYS,
                  AT91C_AIC_SRCTYPE_HIGH_LEVEL | (AT91C_AIC_PRIOR_HIGHEST - 1),
                  SYSIrqHandler);
  AIC_EnableIT(AT91C_ID_SYS);
  AT91C_BASE_PITC->PITC_PIMR = (MCK / 16 / CH_FREQUENCY) - 1;
  AT91C_BASE_PITC->PITC_PIMR |= AT91C_PITC_PITEN | AT91C_PITC_PITIEN;
}
