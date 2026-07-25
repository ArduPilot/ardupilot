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
 * @file        hal_flash_interface.h
 * @brief       Generated Flash Interface header.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  HAL_FLASH_INTERFACE
 * @brief       Flash Interface.
 * @details     Base interface for flash devices.
 * @{
 */

#ifndef HAL_FLASH_INTERFACE_H
#define HAL_FLASH_INTERFACE_H

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @interface   flash_interface_i
 *
 * @brief       Generic flash interface.
 *
 * @name        Interface @p flash_interface_i structures
 * @{
 */

/**
 * @brief       Type of a flash interface interface.
 */
typedef struct flash_interface flash_interface_i;

/**
 * @brief       Interface @p flash_interface_i virtual methods table.
 */
struct flash_interface_vmt {
  /* Memory offset between this interface structure and begin of
     the implementing class structure.*/
  size_t instance_offset;
  /* From flash_interface_i.*/
  const flash_descriptor_t * (*get_descriptor)(void *ip);
  flash_error_t (*read)(void *ip, flash_offset_t offset, size_t n, uint8_t *rp);
  flash_error_t (*program)(void *ip, flash_offset_t offset, size_t n, const uint8_t *pp);
  flash_error_t (*start_erase_all)(void *ip);
  flash_error_t (*start_erase_sector)(void *ip, flash_sector_t sector);
  flash_error_t (*query_erase)(void *ip, unsigned *msec);
  flash_error_t (*verify_erase)(void *ip, flash_sector_t sector);
  flash_error_t (*acquire_exclusive)(void *ip);
  flash_error_t (*release_exclusive)(void *ip);
};

/**
 * @brief       Structure representing a flash interface interface.
 */
struct flash_interface {
  /**
   * @brief       Virtual Methods Table.
   */
  const struct flash_interface_vmt *vmt;
};
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

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @name        Virtual methods of flash_interface_i
 * @{
 */
/**
 * @brief       Gets the flash descriptor structure.
 *
 * @param[in,out] ip            Pointer to a @p flash_interface_i instance.
 * @return                      A flash device descriptor.
 */
CC_FORCE_INLINE
static inline const flash_descriptor_t *flsGetDescriptor(void *ip) {
  flash_interface_i *self = (flash_interface_i *)ip;

  return self->vmt->get_descriptor(ip);
}

/**
 * @brief       Read operation.
 *
 * @param[in,out] ip            Pointer to a @p flash_interface_i instance.
 * @param[in]     offset        Flash offset.
 * @param[in]     n             Number of bytes to be read.
 * @param[out]    rp            Pointer to the data buffer.
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       If there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING   If there is an erase operation in progress.
 * @retval FLASH_ERROR_READ     If the read operation failed.
 * @retval FLASH_ERROR_HW_FAILURE If access to the memory failed.
 */
CC_FORCE_INLINE
static inline flash_error_t flsRead(void *ip, flash_offset_t offset, size_t n,
                                    uint8_t *rp) {
  flash_interface_i *self = (flash_interface_i *)ip;

  return self->vmt->read(ip, offset, n, rp);
}

/**
 * @brief       Program operation.
 *
 * @param[in,out] ip            Pointer to a @p flash_interface_i instance.
 * @param[in]     offset        Flash offset.
 * @param[in]     n             Number of bytes to be programmed.
 * @param[in]     pp            Pointer to the data buffer.
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       If there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING   If there is an erase operation in progress.
 * @retval FLASH_ERROR_PROGRAM  If the program operation failed.
 * @retval FLASH_ERROR_HW_FAILURE If access to the memory failed.
 */
CC_FORCE_INLINE
static inline flash_error_t flsProgram(void *ip, flash_offset_t offset,
                                       size_t n, const uint8_t *pp) {
  flash_interface_i *self = (flash_interface_i *)ip;

  return self->vmt->program(ip, offset, n, pp);
}

/**
 * @brief       Starts a whole-device erase operation.
 *
 * @param[in,out] ip            Pointer to a @p flash_interface_i instance.
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       If there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING   If there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE If access to the memory failed.
 */
CC_FORCE_INLINE
static inline flash_error_t flsStartEraseAll(void *ip) {
  flash_interface_i *self = (flash_interface_i *)ip;

  return self->vmt->start_erase_all(ip);
}

/**
 * @brief       Starts a sector erase operation.
 *
 * @param[in,out] ip            Pointer to a @p flash_interface_i instance.
 * @param[in]     sector        Sector to be erased.
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       If there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING   If there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE If access to the memory failed.
 */
CC_FORCE_INLINE
static inline flash_error_t flsStartEraseSector(void *ip,
                                                flash_sector_t sector) {
  flash_interface_i *self = (flash_interface_i *)ip;

  return self->vmt->start_erase_sector(ip, sector);
}

/**
 * @brief       Queries the driver for erase operation progress.
 *
 * @param[in,out] ip            Pointer to a @p flash_interface_i instance.
 * @param[out]    msec          Recommended time, in milliseconds, that should
 *                              be spent before calling this function again,
 *                              can be @p NULL
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       If there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING   If there is an erase operation in progress.
 * @retval FLASH_ERROR_HW_FAILURE If access to the memory failed.
 */
CC_FORCE_INLINE
static inline flash_error_t flsQueryErase(void *ip, unsigned *msec) {
  flash_interface_i *self = (flash_interface_i *)ip;

  return self->vmt->query_erase(ip, msec);
}

/**
 * @brief       Returns the erase state of a sector.
 *
 * @param[in,out] ip            Pointer to a @p flash_interface_i instance.
 * @param[in]     sector        Sector to be verified.
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       If there is no erase operation in progress.
 * @retval FLASH_BUSY_ERASING   If there is an erase operation in progress.
 * @retval FLASH_ERROR_VERIFY   If the verify operation failed.
 * @retval FLASH_ERROR_HW_FAILURE If access to the memory failed.
 */
CC_FORCE_INLINE
static inline flash_error_t flsVerifyErase(void *ip, flash_sector_t sector) {
  flash_interface_i *self = (flash_interface_i *)ip;

  return self->vmt->verify_erase(ip, sector);
}

/**
 * @brief       Acquires exclusive access to flash.
 *
 * @param[in,out] ip            Pointer to a @p flash_interface_i instance.
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       If there is no erase operation in progress.
 * @retval FLASH_ERROR_UNIMPLEMENTED UNIMPLEMENTED if exclusive access not
 *                              enabled.
 */
CC_FORCE_INLINE
static inline flash_error_t flsAcquireExclusive(void *ip) {
  flash_interface_i *self = (flash_interface_i *)ip;

  return self->vmt->acquire_exclusive(ip);
}

/**
 * @brief       Releases exclusive access to flash.
 *
 * @param[in,out] ip            Pointer to a @p flash_interface_i instance.
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       If there is no erase operation in progress.
 * @retval FLASH_ERROR_UNIMPLEMENTED UNIMPLEMENTED if exclusive access not
 *                              enabled.
 */
CC_FORCE_INLINE
static inline flash_error_t flsReleaseExclusive(void *ip) {
  flash_interface_i *self = (flash_interface_i *)ip;

  return self->vmt->release_exclusive(ip);
}
/** @} */

#endif /* HAL_FLASH_INTERFACE_H */

/** @} */
