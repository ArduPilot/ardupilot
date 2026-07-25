/*
    ChibiOS - Copyright (C) 2006..2022 Theodore Ateba

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
 * @file    CRCv1/hal_crc_lld.h
 * @brief   XMEGA CRC low level driver header.
 *
 * @addtogroup CRC
 * @{
 */

#ifndef HAL_CRC_LLD_H
#define HAL_CRC_LLD_H

#include "xmega_crc.h"

#if HAL_USE_CRC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   CRC1 driver enable switch.
 * @note    The default is @p FALSE.
 */
#if !defined(AVR_CRC_USE_CRC1) || defined(__DOXYGEN__)
#define AVR_CRC_USE_CRC1                  FALSE
#endif
/** @} */

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
  CRC_UNINIT = 0,                   /**< Not initialized.                   */
  CRC_STOP,                         /**< Stopped.                           */
  CRC_READY                         /**< Ready.                             */
} crcstate_t;

/**
 * @brief CRC available sources.
 */
typedef enum {
  CRC_SOURCE_DISABLE  = 0,   /* CRC disable.               */
  CRC_SOURCE_IO       = 1,   /* I/O interface.             */
  CRC_SOURCE_FLASH    = 2,   /* Flash.                     */
  CRC_SOURCE_EDMACH0  = 4,   /* EDMA controller channel 0. */
  CRC_SOURCE_EDMACH1  = 5,   /* EDMA controller channel 1. */
  CRC_SOURCE_EDMACH2  = 6,   /* EDMA controller channel 2. */
  CRC_SOURCE_EDMACH3  = 7    /* EDMA controller channel 3. */
} crcsource_t;

/**
 * @brief Available CRC polynomial algorithme.
 */
typedef enum {
  CRC_TYPE_CRC16 = 0,  /* CRC 16 = CRC-CCITT.  */
  CRC_TYPE_CRC32       /* CRC 32 = IEEE 802.3. */
} crcalgo_t;

/**
 * @brief   CRC available reset method.
 */
typedef enum {
  CRC_RESET_NO      = 0,  /* No reset.                                  */
  CRC_RESET_RESET0  = 2,  /* Reset all CRC with checksum to all zeros.  */
  CRC_RESET_RESET1  = 3   /* Reset all CRC with checksum to all ones.   */
} crcreset_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  crcsource_t     source;
  crcalgo_t       algo;
} CRCConfig;

/**
 * @brief   Structure representing an CRC driver.
 */
typedef struct {
  /**
   * @brief   Driver state.
   */
  crcstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const CRCConfig           *config;
  /* End of the mandatory fields.*/
  /**
   * @brief   Pointer to the CRC registers block.
   */
  xmega_crc_t               *crc;
} CRCDriver;

/**
 * @brief   Type used while reading CRC16 result.
 */
typedef union {
  uint16_t crc16;
  struct {
    uint16_t byte0 : 8;
    uint16_t byte1 : 8;
  } crc8;
} crc16_t;

/**
 * @brief   Type used while reading CRC32 result.
 */
typedef union {
  uint32_t crc32;
  struct {
    uint32_t byte0 : 8;
    uint32_t byte1 : 8;
    uint32_t byte2 : 8;
    uint32_t byte3 : 8;
  } crc8;
} crc32_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if AVR_CRC_USE_CRC1 && !defined(__DOXYGEN__)
extern CRCDriver CRCD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void crc_lld_object_init(CRCDriver *crcp);
  void crc_lld_init(void);
  void crc_lld_start(CRCDriver *crcp, CRCConfig *config);
  void crc_lld_stop(CRCDriver *crcp);
  void crc_lld_reset(CRCDriver *crcp, crcreset_t rt);
  uint16_t crc_lld_compute_crc16_on_data(CRCDriver *crcp, uint8_t *srcp, size_t size);
  uint32_t crc_lld_compute_crc32_on_data(CRCDriver *crcp, uint8_t *srcp, size_t size);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_CRC */

#endif /* HAL_CRC_LLD_H */

/** @} */
