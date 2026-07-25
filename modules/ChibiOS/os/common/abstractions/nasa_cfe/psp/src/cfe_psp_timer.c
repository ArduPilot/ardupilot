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

/**
 * @file    cfe_psp_timer.c
 * @brief   CFE PSP timer module code.
 *
 * @addtogroup nasa_cfe_psp_timer
 * @{
 */

#include "ch.h"

#include "common_types.h"
#include "osapi.h"
#include "cfe_psp.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void CFE_PSP_GetTime(OS_time_t *LocalTime) {

  (void)LocalTime;
}

uint32 CFE_PSP_GetTimerTicksPerSecond(void) {

  return 0;
}

uint32 CFE_PSP_GetTimerLow32Rollover(void) {

  return 0;
}

void CFE_PSP_Get_Timebase(uint32 *Tbu, uint32* Tbl) {

  (void)Tbu;
  (void)Tbl;
}

uint32 CFE_PSP_Get_Dec(void) {

  return 0;
}

/** @} */
