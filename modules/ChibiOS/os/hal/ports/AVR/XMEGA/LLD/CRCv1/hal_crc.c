/*
    ChibiOS - Copyright (C) 2006..2024 Theodore Ateba
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
 * @file    hal_crc.c
 * @brief   CRC Driver code.
 *
 * @addtogroup CRCv1
 * @{
 */

#include "hal.h"

/* Custom Driver must directly include the low level driver. */
#include "hal_crc_lld.h"

#if HAL_USE_CRC || defined(__DOXYGEN__)

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
 * @brief   CRC Driver initialization.
 * @note    This function is not implicitly invoked by @p halInit(). So we
 *          need to explicitly initialize the driver by calling this function.
 *
 * @init
 */
void crcInit(void) {

  crc_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p CRCDriver structure.
 *
 * @param[out] crcp     pointer to the @p CRCDriver object
 *
 * @init
 */
void crcObjectInit(CRCDriver *crcp) {

  crc_lld_object_init(crcp);
}

/**
 * @brief   Configures and activates the CRC peripheral.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 * @param[in] config    pointer to the @p CRCConfig object
 * @return              The operation status.
 *
 * @api
 */
msg_t crcStart(CRCDriver *crcp, CRCConfig *config) {

  osalDbgCheck((crcp != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((crcp->state == CRC_STOP) || (crcp->state == CRC_READY),
                "invalid state");

  crcp->config = config;

  crc_lld_start(crcp, config);
  crcp->state = CRC_READY;

  osalSysUnlock();

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Deactivates the CRC peripheral.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 *
 * @api
 */
void crcStop(CRCDriver *crcp) {

  osalDbgCheck(crcp != NULL);

  osalSysLock();

  osalDbgAssert((crcp->state == CRC_STOP) || (crcp->state == CRC_READY),
                "invalid state");

  crc_lld_stop(crcp);
  crcp->config = NULL;
  crcp->state  = CRC_STOP;

  osalSysUnlock();
}

/**
 * @brief   Resets CRC module.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 * @param[in] rt        type of reset to perform, all to zeros or all to ones
 *
 * @api
 */
void crcReset(CRCDriver *crcp, crcreset_t rt) {

  osalDbgCheck(crcp != NULL);

  osalSysLock();
  osalDbgAssert(crcp->state == CRC_READY, "not ready");
  crc_lld_reset(crcp, rt);
  osalSysUnlock();
}

uint16_t crcComputeCRC16OnData(CRCDriver *crcp, uint8_t *srcp, size_t size) {

  uint16_t retVal = 0;

  osalDbgCheck(crcp != NULL);

  osalSysLock();
  retVal = crc_lld_compute_crc16_on_data(crcp, srcp, size);
  osalSysUnlock();

  return retVal;
}

#endif /* HAL_USE_CRC */

/** @} */
