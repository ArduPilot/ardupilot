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
 * @file    CRCv1/xmega_crc.h
 * @brief   XMEGA CRC units common header.
 * @note    This file requires definitions from the ATMEL XMEGA header file.
 *
 * @addtogroup XMEGA_CRCv1
 * @{
 */

#ifndef XMEGA_CRC_H
#define XMEGA_CRC_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   CRC module base address.
 */
#define CRC_BASE        0x00D0

/**
 * @note    Discarded definition from Atmel ATxmega headers.
 *          The CRC driver uses its own definition in order to have an
 *          unified handling for all devices.
 */
#undef CRC

/**
 * @name    CRC Cyclic redondancy check definitions.
 * @{
 */
#define CRC             ((xmega_crc_t *)CRC_BASE)
/** @} */

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
 * @brief  XMEGA CRC registers block.
 */
typedef struct {

  volatile union {
    uint8_t reg;
    struct {
      uint8_t SOURCE    : 4;
      uint8_t           : 1;
      uint8_t CRC32     : 1;
      uint8_t RESET     : 2;
    } bit;
  }  CTRL;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t BUSY      : 1;
      uint8_t ZERO      : 1;
      uint8_t           : 6;
    } bit;
  }  STATUS;
  uint8_t reserved1;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t DATAIN    : 8;
    } bit;
  }  DATAIN;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t CHECKSUM  : 8;
    } bit;
  }  CHECKSUM0;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t CHECKSUM  : 8;
    } bit;
  }  CHECKSUM1;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t CHECKSUM  : 8;
    } bit;
  }  CHECKSUM2;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t CHECKSUM  : 8;
    } bit;
  }  CHECKSUM3;
} xmega_crc_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* XMEGA_CRC_H */

/** @} */
