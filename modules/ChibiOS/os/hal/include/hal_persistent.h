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
 * @file    hal_persistent.h
 * @brief   Generic persistent storage class header.
 *
 * @addtogroup HAL_PERSISTENT
 * @details This module define an abstract interface for generic persistent
 *          storage. Such storage has a fixed size and can be read and
 *          written.
 * @{
 */

#ifndef HAL_PERSISTENT_H
#define HAL_PERSISTENT_H

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
 * @brief   Type of a persistent storage error code.
 * @note    Code values are kept equal to the equivalent codes in the flash
 *          interface, this is intentional.
 */
typedef enum {
  PS_NO_ERROR = 0,              /* No error.                                */
  PS_ERROR_READ = 2,            /* ECC or other error during read operation.*/
  PS_ERROR_WRITE = 3,           /* Program operation failed.                */
  PS_ERROR_VERIFY = 5,          /* Verify operation failed.                 */
  PS_ERROR_HW_FAILURE = 6       /* Controller or communication error.       */
} ps_error_t;

/**
 * @brief   Type of a persistent storage offset.
 */
typedef uint32_t ps_offset_t;

/**
 * @brief   @p BasePersistentStorage specific methods.
 */
#define _base_pers_storage_methods_alone                                    \
  /* Storage size.*/                                                        \
  size_t (*getsize)(void *instance);                                        \
  /* Read operation.*/                                                      \
  ps_error_t (*read)(void *instance, ps_offset_t offset,                    \
                     size_t n, uint8_t *rp);                                \
  /* Write operation.*/                                                     \
  ps_error_t (*write)(void *instance, ps_offset_t offset,                   \
                      size_t n, const uint8_t *wp);

/**
 * @brief   @p BasePersistentStorage specific methods with inherited ones.
 */
#define _base_pers_storage_methods                                          \
  _base_object_methods                                                      \
  _base_pers_storage_methods_alone

/**
 * @brief   @p BasePersistentStorage virtual methods table.
 */
struct BasePersistentStorageVMT {
  _base_pers_storage_methods
};

/**
 * @brief   @p BasePersistentStorage specific data.
 */
#define _base_persistent_storage_data                                       \
  _base_object_data

/**
 * @extends BaseObject
 *
 * @brief   Base persistent storage class.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BasePersistentStorageVMT *vmt;
  _base_persistent_storage_data
} BasePersistentStorage;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions (BasePersistentStorage)
 * @{
 */
/**
 * @brief   Instance getter.
 * @details This special method is used to get the instance of this class
 *          object from a derived class.
 */
#define getBasePersistentStorage(ip) ((BasePersistentStorage *)&(ip)->vmt)

/**
 * @brief   Get storage size.
 *
 * @param[in] ip        pointer to a @p BasePersistentStorage or derived class
 * @return              The storage size in bytes.
 *
 * @api
 */
#define psGetStorageSize(ip)                                                \
  (ip)->vmt->getsize(ip)

/**
 * @brief   Read operation.
 *
 * @param[in] ip        pointer to a @p BasePersistentStorage or derived class
 * @param[in] offset    persistent storage offset
 * @param[in] n         number of bytes to be read
 * @param[out] rp       pointer to the data buffer
 * @return              An error code.
 * @retval PS_NO_ERROR  if there is no erase operation in progress.
 * @retval PS_ERROR_READ if the read operation failed.
 * @retval PS_ERROR_HW_FAILURE if access to the memory failed.
 *
 * @api
 */
#define psRead(ip, offset, n, rp)                                           \
  (ip)->vmt->read(ip, offset, n, rp)

/**
 * @brief   Write operation.
 *
 * @param[in] ip        pointer to a @p BasePersistentStorage or derived class
 * @param[in] offset    persistent storage offset
 * @param[in] n         number of bytes to be written
 * @param[in] wp        pointer to the data buffer
 * @return              An error code.
 * @retval PS_NO_ERROR  if there is no erase operation in progress.
 * @retval PS_ERROR_WRITE if the write operation failed.
 * @retval PS_ERROR_HW_FAILURE if access to the memory failed.
 *
 * @api
 */
#define psWrite(ip, offset, n, wp)                                          \
  (ip)->vmt->write(ip, offset, n, wp)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_PERSISTENT_H */

/** @} */
