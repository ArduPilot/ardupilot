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
 * @file    hal_flash.h
 * @brief   Generic flash driver class header.
 *
 * @addtogroup HAL_FLASH
 * @{
 */

#ifndef HAL_FLASH_H
#define HAL_FLASH_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Flash attributes
 * @{
 */
/**
 * @brief   Defines one as the erased bit state.
 */
#define FLASH_ATTR_ERASED_IS_ONE            0x00000001U
/**
 * @brief   The memory is accessible in a memory mapped mode.
 */
#define FLASH_ATTR_MEMORY_MAPPED            0x00000002U
/**
 * @brief   Programmed pages can be programmed again.
 * @note    This is incompatible and alternative to @p FLASH_ATTR_ECC_CAPABLE.
 */
#define FLASH_ATTR_REWRITABLE               0x00000004U
/**
 * @brief   The memory is protected by an ECC mechanism.
 * @note    This usually puts restrictions on the program operations.
 *          - Program operations can only happen at offsets aligned to
 *            write page boundaries.
 *          - The programmed data size must be a multiple of the write
 *            page size.
 *          - Programmed pages cannot be re-programmed.
 */
#define FLASH_ATTR_ECC_CAPABLE              0x00000008U
/**
 * @brief   The device is able to overwrite zero to a line.
 * @note    This attribute is only meaningful for those devices that support
 *          ECC, so also @p FLASH_ATTR_ECC_CAPABLE must be specified.
 */
#define FLASH_ATTR_ECC_ZERO_LINE_CAPABLE    0x00000010U
/**
 * @brief   The device is able to suspend erase operations.
 */
#define FLASH_ATTR_SUSPEND_ERASE_CAPABLE    0x00000020U
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
 * @brief   Driver state machine possible states.
 */
typedef enum {
  FLASH_UNINIT = 0,
  FLASH_STOP = 1,
  FLASH_READY = 2,
  FLASH_READ = 3,
  FLASH_PGM = 4,
  FLASH_ERASE = 5
} flash_state_t;

/**
 * @brief   Type of a flash error code.
 */
typedef enum {
  FLASH_NO_ERROR = 0,           /* No error.                                */
  FLASH_BUSY_ERASING = 1,       /* Erase operation in progress.             */
  FLASH_ERROR_READ = 2,         /* ECC or other error during read operation.*/
  FLASH_ERROR_PROGRAM = 3,      /* Program operation failed.                */
  FLASH_ERROR_ERASE = 4,        /* Erase operation failed.                  */
  FLASH_ERROR_VERIFY = 5,       /* Verify operation failed.                 */
  FLASH_ERROR_HW_FAILURE = 6,   /* Controller or communication error.       */
  FLASH_ERROR_UNIMPLEMENTED = 7 /* Unimplemented functionality.             */
} flash_error_t;

/**
 * @brief   Type of a flash offset.
 */
typedef uint32_t flash_offset_t;

/**
 * @brief   Type of a flash sector number.
 */
typedef uint32_t flash_sector_t;

/**
 * @brief   Flash sector descriptor.
 */
typedef struct {
  /**
   * @brief         Sector offset.
   */
  flash_offset_t        offset;
  /**
   * @brief         Sector size.
   */
  uint32_t              size;
} flash_sector_descriptor_t;

/**
 * @brief   Type of a flash device descriptor.
 */
typedef struct {
  /**
   * @brief     Device_attributes.
   */
  uint32_t              attributes;
  /**
   * @brief     Size of write page.
   */
  uint32_t              page_size;
  /**
   * @brief     Number of sectors in the device.
   */
  flash_sector_t        sectors_count;
  /**
   * @brief     List of sectors for devices with non-uniform sector sizes.
   * @note      If @p NULL then the device has uniform sectors size equal
   *            to @p sector_size.
   */
  const flash_sector_descriptor_t *sectors;
  /**
   * @brief     Size of sectors for devices with uniform sector size.
   * @note      If zero then the device has non uniform sectors described
   *            by the @p sectors array.
   */
  uint32_t              sectors_size;
  /**
   * @brief     Flash address if memory mapped or zero.
   * @note      Conventionally, non memory mapped devices have address @p NULL.
   */
  uint8_t               *address;
  /**
   * @brief     Flash size.
   */
  uint32_t              size;
} flash_descriptor_t;

/**
 * @brief   @p BaseFlash specific methods.
 */
#define _base_flash_methods_alone                                           \
  /* Get flash device attributes.*/                                         \
  const flash_descriptor_t * (*get_descriptor)(void *instance);             \
  /* Read operation.*/                                                      \
  flash_error_t (*read)(void *instance, flash_offset_t offset,              \
                        size_t n, uint8_t *rp);                             \
  /* Program operation.*/                                                   \
  flash_error_t (*program)(void *instance, flash_offset_t offset,           \
                           size_t n, const uint8_t *pp);                    \
  /* Erase whole flash device.*/                                            \
  flash_error_t (*start_erase_all)(void *instance);                         \
  /* Erase single sector.*/                                                 \
  flash_error_t (*start_erase_sector)(void *instance,                       \
                                      flash_sector_t sector);               \
  flash_error_t (*query_erase)(void *instance, uint32_t *wait_time);        \
  /* Verify erase single sector.*/                                          \
  flash_error_t (*verify_erase)(void *instance, flash_sector_t sector);     \
  /* Acquire exclusive use of flash.*/                                      \
  flash_error_t (*acquire_exclusive)(void *instance);                       \
  /* Release exclusive use of flash.*/                                      \
  flash_error_t (*release_exclusive)(void *instance);

/**
 * @brief   @p BaseFlash specific methods with inherited ones.
 */
#define _base_flash_methods                                                 \
  _base_object_methods                                                      \
  _base_flash_methods_alone

/**
 * @brief   @p BaseFlash virtual methods table.
 */
struct BaseFlashVMT {
  _base_flash_methods
};

/**
 * @brief   @p BaseFlash specific data.
 */
#define _base_flash_data                                                    \
  _base_object_data                                                         \
  /* Driver state.*/                                                        \
  flash_state_t         state;

/**
 * @extends BaseObject
 *
 * @brief   Base flash class.
 */
typedef struct {
  /** @brief Virtual Methods Table.*/
  const struct BaseFlashVMT *vmt;
  _base_flash_data
} BaseFlash;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions (BaseFlash)
 * @{
 */
/**
 * @brief   Instance getter.
 * @details This special method is used to get the instance of this class
 *          object from a derived class.
 */
#define getBaseFlash(ip) ((BaseFlash *)&(ip)->vmt)

/**
 * @brief   Gets the flash descriptor structure.
 *
 * @param[in] ip                    pointer to a @p BaseFlash or derived class
 * @return                          A flash device descriptor.
 *
 * @api
 */
#define flashGetDescriptor(ip)                                              \
  (ip)->vmt->get_descriptor(ip)

/**
 * @brief   Read operation.
 *
 * @param[in] ip                    pointer to a @p BaseFlash or derived class
 * @param[in] offset                flash offset
 * @param[in] n                     number of bytes to be read
 * @param[out] rp                   pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_READ         if the read operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
#define flashRead(ip, offset, n, rp)                                        \
  (ip)->vmt->read(ip, offset, n, rp)

/**
 * @brief   Program operation.
 *
 * @param[in] ip                    pointer to a @p BaseFlash or derived class
 * @param[in] offset                flash offset
 * @param[in] n                     number of bytes to be programmed
 * @param[in] pp                    pointer to the data buffer
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_PROGRAM      if the program operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
#define flashProgram(ip, offset, n, pp)                                     \
  (ip)->vmt->program(ip, offset, n, pp)

/**
 * @brief   Starts a whole-device erase operation.
 *
 * @param[in] ip                    pointer to a @p BaseFlash or derived class
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
#define flashStartEraseAll(ip)                                              \
  (ip)->vmt->start_erase_all(ip)

/**
 * @brief   Starts an sector erase operation.
 *
 * @param[in] ip                    pointer to a @p BaseFlash or derived class
 * @param[in] sector                sector to be erased
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
#define flashStartEraseSector(ip, sector)                                   \
  (ip)->vmt->start_erase_sector(ip, sector)

/**
 * @brief   Queries the driver for erase operation progress.
 *
 * @param[in] ip                    pointer to a @p BaseFlash or derived class
 * @param[out] msec                 recommended time, in milliseconds, that
 *                                  should be spent before calling this
 *                                  function again, can be @p NULL
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_ERASE        if the erase operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
#define flashQueryErase(ip, msec)                                           \
  (ip)->vmt->query_erase(ip, msec)

/**
 * @brief   Returns the erase state of a sector.
 *
 * @param[in] ip                    pointer to a @p BaseFlash or derived class
 * @param[in] sector                sector to be verified
 * @return                          An error code.
 * @retval FLASH_NO_ERROR           if the sector is erased.
 * @retval FLASH_BUSY_ERASING       if there is an erase operation in progress.
 * @retval FLASH_ERROR_VERIFY       if the verify operation failed.
 * @retval FLASH_ERROR_HW_FAILURE   if access to the memory failed.
 *
 * @api
 */
#define flashVerifyErase(ip, sector)                                        \
  (ip)->vmt->verify_erase(ip, sector)

/**
 * @brief   Acquires exclusive access to flash.
 *
 * @param[in] ip                      pointer to a @p BaseFlash or derived class
 * @return                            An error code.
 * @retval FLASH_NO_ERROR             if the access is obtained.
 * @retval FLASH_ERROR_UNIMPLEMENTED  if exclusive access not enabled
 * @api
 */
#define flashAcquireExclusive(ip)                                           \
    (ip)->vmt->acquire_exclusive(ip)

/**
 * @brief   Releases exclusive access to flash.
 *
 * @param[in] ip                      pointer to a @p BaseFlash or derived class
 * @return                            An error code.
 * @retval FLASH_NO_ERROR             if the access is released.
 * @retval FLASH_ERROR_UNIMPLEMENTED  if exclusive access not enabled
 * @api
 */
#define flashReleaseExclusive(ip)                                           \
  (ip)->vmt->release_exclusive(ip)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  flash_error_t  flashWaitErase(BaseFlash *devp);
  flash_offset_t flashGetSectorOffset(BaseFlash *devp, flash_sector_t sector);
  uint32_t       flashGetSectorSize(BaseFlash *devp, flash_sector_t sector);
  flash_sector_t flashGetOffsetSector(BaseFlash *devp, flash_offset_t offset);
  void *         flashGetOffsetAddress(BaseFlash *devp, flash_offset_t offset);
  flash_offset_t flashGetAddressOffset(BaseFlash *devp, void * addr);
#ifdef __cplusplus
}
#endif

#endif /* HAL_FLASH_H */

/** @} */
