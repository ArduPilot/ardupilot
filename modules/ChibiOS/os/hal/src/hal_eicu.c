/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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
/*
   Rewritten by Emil Fresk (1/5 - 2014) for extended input capture
   functionality. And fix for spurious callbacks in the interrupt handler.
*/
/*
   Improved by Uladzimir Pylinsky aka barthess (1/3 - 2015) for support of
   32-bit timers and timers with single capture/compare channels.
*/

/*
 * Hardware Abstraction Layer for Extended Input Capture Unit
 */
#include "hal.h"

#if (HAL_USE_EICU == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   EICU Driver initialization.
 *
 * @init
 */
void eicuInit(void) {

  eicu_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p EICUDriver structure.
 *
 * @param[out] eicup    Pointer to the @p EICUDriver object
 *
 * @init
 */
void eicuObjectInit(EICUDriver *eicup) {

  eicup->state  = EICU_STOP;
  eicup->config = NULL;
}

/**
 * @brief   Configures and activates the EICU peripheral.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 * @param[in] config    Pointer to the @p EICUConfig object
 *
 * @api
 */
void eicuStart(EICUDriver *eicup, const EICUConfig *config) {

  osalDbgCheck((eicup != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((eicup->state == EICU_STOP) || (eicup->state == EICU_READY),
                "invalid state");
  eicup->config = config;
  eicu_lld_start(eicup);
  eicup->state = EICU_READY;
  osalSysUnlock();
}

/**
 * @brief   Deactivates the EICU peripheral.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @api
 */
void eicuStop(EICUDriver *eicup) {

  osalDbgCheck(eicup != NULL);

  osalSysLock();
  osalDbgAssert((eicup->state == EICU_STOP) || (eicup->state == EICU_READY),
                "invalid state");
  eicu_lld_stop(eicup);
  eicup->state = EICU_STOP;
  osalSysUnlock();
}

/**
 * @brief   Enables the extended input capture.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @api
 */
void eicuEnable(EICUDriver *eicup) {

  osalDbgCheck(eicup != NULL);

  osalSysLock();
  osalDbgAssert(eicup->state == EICU_READY, "invalid state");
  eicu_lld_enable(eicup);
  eicup->state = EICU_WAITING;
  osalSysUnlock();
}

/**
 * @brief   Disables the extended input capture.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @api
 */
void eicuDisable(EICUDriver *eicup) {

  osalDbgCheck(eicup != NULL);

  osalSysLock();
  osalDbgAssert((eicup->state == EICU_READY) || (eicup->state == EICU_IDLE) ||
                (eicup->state == EICU_ACTIVE) || (eicup->state == EICU_WAITING),
                 "invalid state");
  eicu_lld_disable(eicup);
  eicup->state = EICU_READY;
  osalSysUnlock();
}

#endif /* HAL_USE_EICU */