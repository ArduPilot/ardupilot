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
 * @file    CRCv1/hal_crc_lld.c
 * @brief   XMEGA CRC subsystem low level driver source.
 *
 * @addtogroup CRC
 * @{
 */

#include "hal.h"

/* Include CRC low level driver header directly for XMEGA specific driver. */
#include "hal_crc_lld.h"

#if HAL_USE_CRC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief  CRC1 driver identifier. */
#if AVR_CRC_USE_CRC1 || defined(__DOXYGEN__)
CRCDriver CRCD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Enable the CRC module.
 *
 * @param[in]   crcp    pointer to the @p CRCDriver object.
 */
static void crc_enable(CRCDriver *crcp) {

  switch (crcp->config->source) {
    case CRC_SOURCE_IO:
    case CRC_SOURCE_FLASH:
    case CRC_SOURCE_EDMACH0:
    case CRC_SOURCE_EDMACH1:
    case CRC_SOURCE_EDMACH2:
         crcp->crc->CTRL.bit.SOURCE = crcp->config->source;
      break;
    default:
      break;
  }
}

/**
 * @brief   Select CRC16 algorithm for crc computation.
 *
 * @param[in]   crcp    pointer to the @p CRCDriver object.
 */
static void crc_select_algo_crc16(CRCDriver *crcp) {

  /* Wait the CRC module to be ready. */
  while (crcp->crc->STATUS.bit.BUSY);

  /* Enable CRC16 algorithm. */
  crcp->crc->CTRL.bit.CRC32 = 0;
}

/**
 * @brief   Select CRC32 algorithm for crc computation.
 *
 * @param[in]   crcp    pointer to the @p CRCDriver object.
 */
static void crc_select_algo_crc32(CRCDriver *crcp) {

  /* Wait the CRC module to be ready. */
  while (crcp->crc->STATUS.bit.BUSY);

  /* Enable the CRC32 algorithm.  */
  crcp->crc->CTRL.bit.CRC32 = 1;
}

/**
 * @brief Reset CRC module.
 *
 * @param[in] crcp    pointer to the @p CRCDriver object.
 * @param[in] rt      reset type that need to be used to reset the CRC module.
 */
static void crc_reset(CRCDriver *crcp, crcreset_t rt) {

  switch (rt) {
    case CRC_RESET_RESET0:
      /* Initialize the CRC with all Zeros. */
      crcp->crc->CTRL.bit.RESET = CRC_RESET_RESET0;
      break;

    case CRC_RESET_RESET1:
      /* Initialize the CRC with all Ones. */
      crcp->crc->CTRL.bit.RESET = CRC_RESET_RESET1;
      break;

    case CRC_RESET_NO:
    default:
      break;
  }
}

/**
 * @brief   Disabble the CRC module.
 *
 * @param[in]   crcp    pointer to the @p CRCDriver object.
 */
static void crc_disable(CRCDriver *crcp) {

  crcp->crc->CTRL.bit.SOURCE = CRC_SOURCE_DISABLE;
}

/**
 * @brief   Load the data into the CRC module to perform.
 *
 * @param[in]   crcp    pointer to the @p CRCDriver object.
 * @param[in]   srcp    pointer to the data used to compute CRC.
 * @parma[in]   size    lenght of the data used to compute CRC.
 */
static void crc_lld_load_data(CRCDriver *crcp, uint8_t *srcp, size_t size) {

  uint8_t index = 0;

  for (index = 0; index < size; index++) {
    crcp->crc->DATAIN.reg = srcp[index];
  }
}

/**
 * @brief   Get the checksum value computed with CRC16 algorithm
 *
 * @param   crcp    pointer to the @p CRCDriver object.
 * @return  retVal  the value of the CRC16 computed .
 */
static uint16_t crc_get_crc16_checksum(CRCDriver *crcp) {

  crc16_t retVal;
  retVal.crc16 = 0;

  retVal.crc8.byte0 = crcp->crc->CHECKSUM0.reg;
  retVal.crc8.byte1 = crcp->crc->CHECKSUM1.reg;

  return retVal.crc16;
}

/**
 * @brief   Get the checksum value computed with CRC32 algorithm
 *
 * @param   crcp    pointer to the @p CRCDriver object.
 * @return  retVal  the value of the CRC32 computed.
 */
static uint32_t crc_get_crc32_checksum(CRCDriver *crcp) {

  crc32_t retVal;

  retVal.crc32 = 0;

  retVal.crc8.byte0 = crcp->crc->CHECKSUM0.reg;
  retVal.crc8.byte1 = crcp->crc->CHECKSUM1.reg;
  retVal.crc8.byte2 = crcp->crc->CHECKSUM2.reg;
  retVal.crc8.byte3 = crcp->crc->CHECKSUM3.reg;

  return retVal.crc32;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the standard part of a @p CRCDriver structure.
 *
 * @param[out] crcp     pointer to the @p CRCDriver object.
 *
 * @init
 */
void crc_lld_object_init(CRCDriver *crcp) {

  crcp->state   = CRC_STOP;
  crcp->config  = NULL;
}

/**
 * @brief   Low level CRC driver initialization.
 *
 * @notapi
 */
void crc_lld_init(void) {

#if AVR_CRC_USE_CRC1
  crc_lld_object_init(&CRCD1);
  CRCD1.crc = CRC;
#endif
}

/**
 * @brief   Configures and activates the CRC peripheral.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 *
 * @notapi
 */
void crc_lld_start(CRCDriver *crcp, CRCConfig *config) {

  /* Copy the config to the CRC driver config. */
  crcp->config = config;

  /* CRC source I/O interface only support CRC16 algorithm. */
  if (crcp->config->source == CRC_SOURCE_IO) {
    crc_select_algo_crc16(crcp);
  }

  /* CRC source Flash Memory only support CRC32 algorithm. */
  else if (crcp->config->source == CRC_SOURCE_FLASH) {
    crc_select_algo_crc32(crcp);
  }

  /* CRC source DMA support both CRC16 and CRC32. */
  else {
    if (crcp->config->algo == CRC_TYPE_CRC16) {
      crc_select_algo_crc16(crcp);
    }
    else {
      crc_select_algo_crc32(crcp);
    }
  }

  /* Enable the CRC module. */
  crc_enable(crcp);
}

/**
 * @brief   Deactivates the CRC peripheral.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object
 *
 * @notapi
 */
void crc_lld_stop(CRCDriver *crcp) {

  crc_disable(crcp);
}

/**
 * @brief   Reset the CRC module with all checksum byte to 0x00.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object.
 * @param[in] rt        reset type use to initialize the checksum all to zeros or ones.
 *
 * @notapi
 */
void crc_lld_reset(CRCDriver *crcp, crcreset_t rt) {

  crc_reset(crcp, rt);
}

/**
 * @brief   Compute the CRC16 on data IO interface.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object.
 * @param[in] srcp      pointer of data used to compute CRC16.
 * @param[in] size      length of data used to compute CRC16.
 * @return    retVal    result of CRC16 operation.
 *
 * @notapi
 */
uint16_t crc_lld_compute_crc16_on_data(CRCDriver *crcp, uint8_t *srcp, size_t size) {

  uint16_t retVal = 0;

  /* Load data to compute the CRC16. */
  crc_lld_load_data(crcp, srcp, size);

  /* Read the computed CRC result. */
  retVal = crc_get_crc16_checksum(crcp);

  return retVal;
}

/**
 * @brief   Compute the CRC32 on data IO interface.
 *
 * @param[in] crcp      pointer to the @p CRCDriver object.
 * @param[in] srcp      pointer of data used to compute CRC32.
 * @param[in] size      length of data used to compute CRC32.
 * @return    retVal    result of CRC32 operation.
 *
 * @notapi
 */
uint32_t crc_lld_compute_crc32_on_data(CRCDriver *crcp, uint8_t *srcp, size_t size) {

  uint32_t retVal = 0;

  /* Load data to compute the CRC32. */
  crc_lld_load_data(crcp, srcp, size);

  /* Read the computed CRC result. */
  retVal = crc_get_crc32_checksum(crcp);

  return retVal;
}

#endif /* HAL_USE_CRC */

/** @} */
