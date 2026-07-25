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

#ifndef HAL_EICU_H_
#define HAL_EICU_H_

#if (HAL_USE_EICU == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  EICU_UNINIT,                /* Not initialized.                           */
  EICU_STOP,                  /* Stopped.                                   */
  EICU_READY,                 /* Ready.                                     */
  EICU_WAITING,               /* Waiting for first edge.                    */
  EICU_ACTIVE,                /* Active cycle phase.                        */
  EICU_IDLE                   /* Idle cycle phase.                          */
} eicustate_t;

/**
 * @brief   Channel state machine possible states.
 */
typedef enum {
  EICU_CH_IDLE,               /* Idle cycle phase.                          */
  EICU_CH_ACTIVE              /* Active cycle phase.                        */
} eicuchannelstate_t;

/** 
 * @brief EICU channel selection definition
 */
typedef enum {
  EICU_CHANNEL_1,
  EICU_CHANNEL_2,
  EICU_CHANNEL_3,
  EICU_CHANNEL_4,
  EICU_CHANNEL_ENUM_END
} eicuchannel_t;

/**
 * @brief   Type of a structure representing an EICU driver.
 */
typedef struct EICUDriver EICUDriver;

/**
 * @brief EICU notification callback type.
 *
 * @param[in] eicup     Pointer to a EICUDriver object
 * @param[in] channel   EICU channel that fired the interrupt
 * @param[in] width     Pulse width
 * @param[in] period    Pulse period
 */
typedef void (*eicucallback_t)(EICUDriver *eicup, eicuchannel_t channel);

#include "hal_eicu_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Enables the extended input capture.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @iclass
 */
#define eicuEnableI(eicup) eicu_lld_enable(eicup)

/**
 * @brief   Disables the extended input capture.
 *
 * @param[in] eicup     Pointer to the @p EICUDriver object
 *
 * @iclass
 */
#define eicuDisableI(eicup) eicu_lld_disable(eicup)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void eicuInit(void);
  void eicuObjectInit(EICUDriver *eicup);
  void eicuStart(EICUDriver *eicup, const EICUConfig *config);
  void eicuStop(EICUDriver *eicup);
  void eicuEnable(EICUDriver *eicup);
  void eicuDisable(EICUDriver *eicup);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_EICU */

#endif /* HAL_EICU_H_ */

/** @} */