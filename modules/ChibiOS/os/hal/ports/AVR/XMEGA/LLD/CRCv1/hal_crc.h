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
 * @file    hal_crc.h
 * @brief   CRC Driver macros and structures.
 *
 * @addtogroup CRC
 * @{
 */

#ifndef HAL_CRC_H
#define HAL_CRC_H

#include <stdint.h>
#include <stddef.h>
#if (HAL_USE_CRC == TRUE) || defined(__DOXYGEN__)

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

#include "hal_crc_lld.h"

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void crcInit(void);
  void crcObjectInit(CRCDriver *crcp);

  msg_t crcStart(CRCDriver *crcp, CRCConfig * config);
  void  crcStop(CRCDriver *crcp);

  void  crcReset(CRCDriver *crcp, crcreset_t rt);

  /* @TODO: uint16_t crcComputeCRC16OnMemory(CRCDriver *crcp, uint8_t *src, size_t size); */
  /* @TODO: uint32_t crcComputeCRC32OnMemory(CRCDriver *crcp, uint8_t *src, size_t size); */

  /* @TODO: uint16_t crcComputeCRC16OnDMA(CRCDriver *crcp, uint8_t *src, size_t size); */
  /* @TODO: uint32_t crcComputeCRC32OnDMA(CRCDriver *crcp, uint8_t *src, size_t size); */

  uint16_t crcComputeCRC16OnData(CRCDriver *crcp, uint8_t *srcp, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CRC == TRUE */

#endif /* HAL_CRC_H */

/** @} */
