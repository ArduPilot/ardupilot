/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    hal_mfs.h
 * @brief   Managed Flash Storage module header.
 *
 * @addtogroup HAL_MFS
 * @{
 */

#ifndef HAL_MFS_H
#define HAL_MFS_H

#include "hal_flash.h"

#if defined(MFS_USE_MFSCONF)
#include "mfsconf.h"
#endif

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define MFS_BANK_MAGIC_1                    0xEC705ADEU
#define MFS_BANK_MAGIC_2                    0xF0339CC5U
#define MFS_HEADER_MAGIC_1                  0x5FAE45F0U
#define MFS_HEADER_MAGIC_2                  0xF045AE5FU

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Maximum number of indexed records in the managed storage.
 * @note    Record indexes go from 1 to @p MFS_CFG_MAX_RECORDS.
 */
#if !defined(MFS_CFG_MAX_RECORDS) || defined(__DOXYGEN__)
#define MFS_CFG_MAX_RECORDS                 32
#endif

/**
 * @brief   Maximum record size in the managed storage.
 */
#if !defined(MFS_CFG_MAX_RECORD_SIZE) || defined(__DOXYGEN__)
#define MFS_CFG_MAX_RECORD_SIZE             16384
#endif

/**
 * @brief   Maximum number of repair attempts on partition mount.
 */
#if !defined(MFS_CFG_MAX_REPAIR_ATTEMPTS) || defined(__DOXYGEN__)
#define MFS_CFG_MAX_REPAIR_ATTEMPTS         3
#endif

/**
 * @brief   Verify written data.
 */
#if !defined(MFS_CFG_WRITE_VERIFY) || defined(__DOXYGEN__)
#define MFS_CFG_WRITE_VERIFY                TRUE
#endif

/**
 * @brief   Enables a stronger and slower check procedure on mount.
 * @details Strong checking requires reading of the whole written data and
 *          this can be slow, normal checking only checks integrity of
 *          metadata, data errors would be detected on read.
 */
#if !defined(MFS_CFG_STRONG_CHECKING) || defined(__DOXYGEN__)
#define MFS_CFG_STRONG_CHECKING             TRUE
#endif

/**
 * @brief   Size of the buffer used for data copying.
 * @note    The buffer size must be a power of two and not smaller than
 *          16 bytes.
 * @note    Larger buffers improve performance, buffers with size multiple
 *          of the flash program page size work better.
 */
#if !defined(MFS_CFG_BUFFER_SIZE) || defined(__DOXYGEN__)
#define MFS_CFG_BUFFER_SIZE                 32
#endif

/**
 * @brief   Enforced memory alignment.
 * @details This value must be a power of two, it enforces a memory alignment
 *          for records in the flash array. This is required when alignment
 *          constraints exist, for example when using a DTR mode on OSPI
 *          devices.
 */
#if !defined(MFS_CFG_MEMORY_ALIGNMENT) || defined(__DOXYGEN__)
#define MFS_CFG_MEMORY_ALIGNMENT            2
#endif

/**
 * @brief   Maximum number of objects writable in a single transaction.
 */
#if !defined(MFS_CFG_TRANSACTION_MAX) || defined(__DOXYGEN__)
#define MFS_CFG_TRANSACTION_MAX             16
#endif

/**
 * @brief   Enables mutual exclusion on flash accesses.
 * @details Mutual exclusion on flash is required when multiple threads are
 *          accessing different flash areas through multiple MFS instances
 *          or different modules.
 * @note    Requires exclusive access support in the associated flash driver.
 */
#if !defined(MFS_USE_FLASH_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define MFS_USE_FLASH_MUTUAL_EXCLUSION      FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (MFS_CFG_MAX_RECORDS < 0) || (MFS_CFG_MAX_RECORDS > 65535)
#error "invalid MFS_CFG_MAX_RECORDS value"
#endif

#if (MFS_CFG_MAX_REPAIR_ATTEMPTS < 1) ||                                    \
    (MFS_CFG_MAX_REPAIR_ATTEMPTS > 10)
#error "invalid MFS_MAX_REPAIR_ATTEMPTS value"
#endif

#if MFS_CFG_BUFFER_SIZE < 16
#error "invalid MFS_CFG_BUFFER_SIZE value"
#endif

#if (MFS_CFG_BUFFER_SIZE & (MFS_CFG_BUFFER_SIZE - 1)) != 0
#error "MFS_CFG_BUFFER_SIZE is not a power of two"
#endif

#if (MFS_CFG_MEMORY_ALIGNMENT < 1) ||                                       \
    (MFS_CFG_MEMORY_ALIGNMENT > MFS_CFG_BUFFER_SIZE)
#error "invalid MFS_CFG_MEMORY_ALIGNMENT value"
#endif

#if (MFS_CFG_MEMORY_ALIGNMENT & (MFS_CFG_MEMORY_ALIGNMENT - 1)) != 0
#error "MFS_CFG_MEMORY_ALIGNMENT is not a power of two"
#endif

#if (MFS_CFG_TRANSACTION_MAX < 0) ||                                        \
    (MFS_CFG_TRANSACTION_MAX > MFS_CFG_MAX_RECORDS)
#error "invalid MFS_CFG_TRANSACTION_MAX value"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a flash bank.
 */
typedef enum {
  MFS_BANK_0 = 0,
  MFS_BANK_1 = 1
} mfs_bank_t;

/**
 * @brief   Type of driver state machine states.
 */
typedef enum {
  MFS_UNINIT = 0,
  MFS_STOP = 1,
  MFS_READY = 2,
  MFS_TRANSACTION = 3,
  MFS_ERROR = 4
} mfs_state_t;

/**
 * @brief   Type of an MFS error code.
 * @note    Errors are negative integers, informative warnings are positive
 *          integers.
 */
typedef enum {
  MFS_NO_ERROR = 0,
  MFS_WARN_REPAIR = 1,
  MFS_WARN_GC = 2,
  MFS_ERR_INV_STATE = -1,
  MFS_ERR_INV_SIZE = -2,
  MFS_ERR_NOT_FOUND = -3,
  MFS_ERR_OUT_OF_MEM = -4,
  MFS_ERR_TRANSACTION_NUM = -5,
  MFS_ERR_TRANSACTION_SIZE = -6,
  MFS_ERR_NOT_ERASED = -7,
  MFS_ERR_FLASH_FAILURE = -8,
  MFS_ERR_INTERNAL = -9
} mfs_error_t;

/**
 * @brief   Type of a bank state assessment.
 */
typedef enum {
  MFS_BANK_ERASED = 0,
  MFS_BANK_OK = 1,
  MFS_BANK_GARBAGE = 2
} mfs_bank_state_t;

/**
 * @brief   Type of a record identifier.
 */
typedef uint32_t mfs_id_t;

/**
 * @brief   Type of a bank header.
 * @note    The header resides in the first 16 bytes of a bank.
 */
typedef union {
  struct {
    /**
     * @brief   Bank magic 1.
     */
    uint32_t                magic1;
    /**
     * @brief   Bank magic 2.
     */
    uint32_t                magic2;
    /**
     * @brief   Usage counter of the bank.
     * @details This value is increased each time a bank swap is performed. It
     *          indicates how much wearing the flash has already endured.
     */
    uint32_t                counter;
    /**
     * @brief   Reserved field.
     */
    uint16_t                reserved1;
    /**
     * @brief   Header CRC.
     */
    uint16_t                crc;
  } fields;
  uint8_t                   hdr8[16];
  uint32_t                  hdr32[4];
} mfs_bank_header_t;

/**
 * @brief   Type of a data block header.
 * @details This structure is placed before each written data block.
 */
typedef union {
  struct {
    /**
     * @brief   Data header magic 1.
     */
    uint32_t                magic1;
    /**
     * @brief   Data header magic 2.
     */
    uint32_t                magic2;
    /**
     * @brief   Record identifier.
     */
    uint16_t                id;
    /**
     * @brief   Data CRC.
     */
    uint16_t                crc;
    /**
     * @brief   Data size.
     * @note    The next record is located at @p MFS_ALIGN_NEXT(size).
     */
    uint32_t                size;
  } fields;
  uint8_t                   hdr8[16];
  uint32_t                  hdr32[4];
} mfs_data_header_t;

typedef struct {
  /**
   * @brief   Offset of the record header.
   */
  flash_offset_t            offset;
  /**
   * @brief   Record data size.
   */
  uint32_t                  size;
} mfs_record_descriptor_t;

/**
 * @brief   Type of a MFS configuration structure.
 */
typedef struct {
  /**
   * @brief   Flash driver associated to this MFS instance.
   */
  BaseFlash                 *flashp;
  /**
   * @brief   Erased value.
   */
  uint32_t                  erased;
  /**
   * @brief   Banks size.
   */
  flash_offset_t            bank_size;
  /**
   * @brief   Base sector index for bank 0.
   */
  flash_sector_t            bank0_start;
  /**
   * @brief   Number of sectors for bank 0.
   * @note    The total size of bank0 sectors must be greater or equal to
   *          @p bank_size.
   */
  flash_sector_t            bank0_sectors;
  /**
   * @brief   Base sector index for bank 1.
   */
  flash_sector_t            bank1_start;
  /**
   * @brief   Number of sectors for bank 1.
   * @note    The total size of bank1 sectors must be greater or equal to
   *          @p bank_size.
   */
  flash_sector_t            bank1_sectors;
} MFSConfig;

/**
 * @brief   Type of a buffered write/erase operation within a transaction.
 */
typedef struct {
  /**
   * @brief   Written header offset.
   */
  flash_offset_t            offset;
  /**
   * @brief   Written data size.
   */
  size_t                    size;
  /**
   * @brief   Record identifier.
   */
  mfs_id_t                  id;
} mfs_transaction_op_t;

/**
 * @brief   Type of an non-cacheable MFS buffer.
 */
typedef union mfs_nocache_buffer {
  mfs_data_header_t       dhdr;
  mfs_bank_header_t       bhdr;
  uint8_t                 data8[MFS_CFG_BUFFER_SIZE];
  uint16_t                data16[MFS_CFG_BUFFER_SIZE / sizeof (uint16_t)];
  uint32_t                data32[MFS_CFG_BUFFER_SIZE / sizeof (uint32_t)];
} mfs_nocache_buffer_t;

/**
 * @brief   Type of an MFS instance.
 */
typedef struct {
  /**
   * @brief   Driver state.
   */
  mfs_state_t               state;
  /**
   * @brief   Current configuration data.
   */
  const MFSConfig           *config;
  /**
   * @brief   Bank currently in use.
   */
  mfs_bank_t                current_bank;
  /**
   * @brief   Usage counter of the current bank.
   */
  uint32_t                  current_counter;
  /**
   * @brief   Pointer to the next free position in the current bank.
   */
  flash_offset_t            next_offset;
  /**
   * @brief   Used space in the current bank without considering erased records.
   */
  flash_offset_t            used_space;
  /**
   * @brief   Offsets of the most recent instance of the records.
   * @note    Zero means that there is not a record with that id.
   */
  mfs_record_descriptor_t   descriptors[MFS_CFG_MAX_RECORDS];
#if (MFS_CFG_TRANSACTION_MAX > 0) || defined(__DOXYGEN__)
  /**
   * @brief   Next write offset for current transaction.
   */
  flash_offset_t            tr_next_offset;
  /**
   * @brief   Maximum offset for the transaction.
   */
  flash_offset_t            tr_limit_offset;
  /**
   * @brief   Number of buffered operations in current transaction.
   */
  uint32_t                  tr_nops;
  /**
   * @brief   Buffered operations in current transaction.
   */
  mfs_transaction_op_t      tr_ops[MFS_CFG_TRANSACTION_MAX];
#endif
  /**
   * @brief   Associated non-cacheable buffer.
   */
  mfs_nocache_buffer_t      *ncbuf;
} MFSDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name   Error codes handling macros
 * @{
 */
#define MFS_IS_ERROR(err) ((err) < MFS_NO_ERROR)
#define MFS_IS_WARNING(err) ((err) > MFS_NO_ERROR)
/** @} */

/**
 * @name   Alignment macros
 * @{
 */
#define MFS_ALIGN_MASK      ((uint32_t)MFS_CFG_MEMORY_ALIGNMENT - 1U)
#define MFS_IS_ALIGNED(v)   (((uint32_t)(v) & MFS_ALIGN_MASK) == 0U)
#define MFS_ALIGN_PREV(v)   ((uint32_t)(v) & ~MFS_ALIGN_MASK)
#define MFS_ALIGN_NEXT(v)   (MFS_ALIGN_PREV(((uint32_t)(v) - 1U)) +         \
                                            MFS_CFG_MEMORY_ALIGNMENT)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void mfsObjectInit(MFSDriver *devp, mfs_nocache_buffer_t *ncbuf);
  mfs_error_t mfsStart(MFSDriver *devp, const MFSConfig *config);
  void mfsStop(MFSDriver *devp);
  mfs_error_t mfsErase(MFSDriver *mfsp);
  mfs_error_t mfsReadRecord(MFSDriver *devp, mfs_id_t id,
                            size_t *np, uint8_t *buffer);
  mfs_error_t mfsWriteRecord(MFSDriver *devp, mfs_id_t id,
                             size_t n, const uint8_t *buffer);
  mfs_error_t mfsEraseRecord(MFSDriver *devp, mfs_id_t id);
  mfs_error_t mfsPerformGarbageCollection(MFSDriver *mfsp);
#if MFS_CFG_TRANSACTION_MAX > 0
  mfs_error_t mfsStartTransaction(MFSDriver *mfsp, size_t size);
  mfs_error_t mfsCommitTransaction(MFSDriver *mfsp);
  mfs_error_t mfsRollbackTransaction(MFSDriver *mfsp);
#endif /* MFS_CFG_TRANSACTION_MAX > 0 */
#ifdef __cplusplus
}
#endif

#endif /* HAL_MFS_H */

/** @} */
