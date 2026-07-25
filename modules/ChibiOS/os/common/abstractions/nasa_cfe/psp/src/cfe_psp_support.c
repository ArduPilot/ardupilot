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
 * @file    cfe_psp_support.c
 * @brief   CFE PSP support module code.
 *
 * @addtogroup nasa_cfe_psp_support
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

/**
 * @brief   Provides a common interface to the processor reset.
 * @note    Not currently implemented.
 */
void CFE_PSP_Restart(uint32 reset_type) {

  (void)reset_type;
}

/**
 * @brief   Generic panic handler.
 */
void CFE_PSP_Panic(int32 ErrorCode) {

  chSysHalt((char *)ErrorCode);
}

/** @} */
