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
 * @file    hal_flash_device.h
 * @brief   Micron N25Q serial flash driver header.
 *
 * @addtogroup MICRON_N25Q
 * @{
 */

#ifndef HAL_FLASH_DEVICE_H
#define HAL_FLASH_DEVICE_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Device capabilities
 * @{
 */
#define SNOR_DEVICE_SUPPORTS_XIP            TRUE
/** @} */

/**
 * @name    Device identification
 * @{
 */
#define N25Q_SUPPORTED_MANUFACTURE_IDS      {0x20}
#define N25Q_SUPPORTED_MEMORY_TYPE_IDS      {0xBA, 0xBB}
/** @} */

/**
 * @name    Command codes
 * @{
 */
#define N25Q_CMD_RESET_ENABLE                       0x66
#define N25Q_CMD_RESET_MEMORY                       0x99
#define N25Q_CMD_READ_ID                            0x9F
#define N25Q_CMD_MULTIPLE_IO_READ_ID                0xAF
#define N25Q_CMD_READ_DISCOVERY_PARAMETER           0x5A
#define N25Q_CMD_READ                               0x03
#define N25Q_CMD_FAST_READ                          0x0B
#define N25Q_CMD_WRITE_ENABLE                       0x06
#define N25Q_CMD_WRITE_DISABLE                      0x04
#define N25Q_CMD_READ_STATUS_REGISTER               0x05
#define N25Q_CMD_WRITE_STATUS_REGISTER              0x01
#define N25Q_CMD_READ_LOCK_REGISTER                 0xE8
#define N25Q_CMD_WRITE_LOCK_REGISTER                0xE5
#define N25Q_CMD_READ_FLAG_STATUS_REGISTER          0x70
#define N25Q_CMD_CLEAR_FLAG_STATUS_REGISTER         0x50
#define N25Q_CMD_READ_NV_CONFIGURATION_REGISTER     0xB5
#define N25Q_CMD_WRITE_NV_CONFIGURATION_REGISTER    0xB1
#define N25Q_CMD_READ_V_CONF_REGISTER               0x85
#define N25Q_CMD_WRITE_V_CONF_REGISTER              0x81
#define N25Q_CMD_READ_ENHANCED_V_CONF_REGISTER      0x65
#define N25Q_CMD_WRITE_ENHANCED_V_CONF_REGISTER     0x61
#define N25Q_CMD_PAGE_PROGRAM                       0x02
#define N25Q_CMD_SUBSECTOR_ERASE                    0x20
#define N25Q_CMD_SECTOR_ERASE                       0xD8
#define N25Q_CMD_BULK_ERASE                         0xC7
#define N25Q_CMD_PROGRAM_ERASE_RESUME               0x7A
#define N25Q_CMD_PROGRAM_ERASE_SUSPEND              0x75
#define N25Q_CMD_READ_OTP_ARRAY                     0x4B
#define N25Q_CMD_PROGRAM_OTP_ARRAY                  0x42
/** @} */

/**
 * @name    Flags status register bits
 * @{
 */
#define N25Q_FLAGS_PROGRAM_ERASE                    0x80U
#define N25Q_FLAGS_ERASE_SUSPEND                    0x40U
#define N25Q_FLAGS_ERASE_ERROR                      0x20U
#define N25Q_FLAGS_PROGRAM_ERROR                    0x10U
#define N25Q_FLAGS_VPP_ERROR                        0x08U
#define N25Q_FLAGS_PROGRAM_SUSPEND                  0x04U
#define N25Q_FLAGS_PROTECTION_ERROR                 0x02U
#define N25Q_FLAGS_RESERVED                         0x01U
#define N25Q_FLAGS_ALL_ERRORS                   (N25Q_FLAGS_ERASE_ERROR |   \
                                                 N25Q_FLAGS_PROGRAM_ERROR | \
                                                 N25Q_FLAGS_VPP_ERROR |     \
                                                 N25Q_FLAGS_PROTECTION_ERROR)
/** @} */

/**
 * @name    Bus interface modes.
 * @{
 */
#define N25Q_BUS_MODE_WSPI1L                1U
#define N25Q_BUS_MODE_WSPI2L                2U
#define N25Q_BUS_MODE_WSPI4L                4U
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Switch WSPI bus width on initialization.
 * @details A bus width initialization is performed by writing the
 *          Enhanced Volatile Configuration Register. If the flash
 *          device is configured using the Non Volatile Configuration
 *          Register then this option is not required.
 * @note    This option is only valid in WSPI bus mode.
 */
#if !defined(N25Q_SWITCH_WIDTH) || defined(__DOXYGEN__)
#define N25Q_SWITCH_WIDTH                   TRUE
#endif

/**
 * @brief   Device bus mode to be used.
 * @note    if @p N25Q_SWITCH_WIDTH is @p FALSE then this is the bus mode
 *          that the device is expected to be using.
 * @note    if @p N25Q_SWITCH_WIDTH is @p TRUE then this is the bus mode
 *          that the device will be switched in.
 * @note    This option is only valid in WSPI bus mode.
 */
#if !defined(N25Q_BUS_MODE) || defined(__DOXYGEN__)
#define N25Q_BUS_MODE                       N25Q_BUS_MODE_WSPI4L
#endif

/**
 * @brief   Delays insertions.
 * @details If enabled this options inserts delays into the flash waiting
 *          routines releasing some extra CPU time for threads with lower
 *          priority, this may slow down the driver a bit however.
 */
#if !defined(N25Q_NICE_WAITING) || defined(__DOXYGEN__)
#define N25Q_NICE_WAITING                   TRUE
#endif

/**
 * @brief   Uses 4kB sub-sectors rather than 64kB sectors.
 */
#if !defined(N25Q_USE_SUB_SECTORS) || defined(__DOXYGEN__)
#define N25Q_USE_SUB_SECTORS                FALSE
#endif

/**
 * @brief   Size of the compare buffer.
 * @details This buffer is allocated in the stack frame of the function
 *          @p flashVerifyErase() and its size must be a power of two.
 *          Larger buffers lead to better verify performance but increase
 *          stack usage for that function.
 */
#if !defined(N25Q_COMPARE_BUFFER_SIZE) || defined(__DOXYGEN__)
#define N25Q_COMPARE_BUFFER_SIZE            32
#endif

/**
 * @brief   Number of dummy cycles for fast read (1..15).
 * @details This is the number of dummy cycles to be used for fast read
 *          operations.
 */
#if !defined(N25Q_READ_DUMMY_CYCLES) || defined(__DOXYGEN__)
#define N25Q_READ_DUMMY_CYCLES              8
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (N25Q_COMPARE_BUFFER_SIZE & (N25Q_COMPARE_BUFFER_SIZE - 1)) != 0
#error "invalid N25Q_COMPARE_BUFFER_SIZE value"
#endif

#if (N25Q_READ_DUMMY_CYCLES < 1) || (N25Q_READ_DUMMY_CYCLES > 15)
#error "invalid N25Q_READ_DUMMY_CYCLES value (1..15)"
#endif

#if (N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI4L) || defined(__DOXYGEN__)
/**
 * @brief   WSPI settings for command only.
 */
#define SNOR_WSPI_CFG_CMD               (WSPI_CFG_CMD_MODE_FOUR_LINES     | \
                                         WSPI_CFG_ADDR_MODE_NONE          | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_NONE          | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

/**
 * @brief   WSPI settings for command and address.
 */
#define SNOR_WSPI_CFG_CMD_ADDR          (WSPI_CFG_CMD_MODE_FOUR_LINES     | \
                                         WSPI_CFG_ADDR_MODE_FOUR_LINES    | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_NONE          | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

/**
 * @brief   WSPI settings for command and data.
 */
#define SNOR_WSPI_CFG_CMD_DATA          (WSPI_CFG_CMD_MODE_FOUR_LINES     | \
                                         WSPI_CFG_ADDR_MODE_NONE          | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_FOUR_LINES    | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

/**
 * @brief   WSPI settings for command, address and data.
 */
#define SNOR_WSPI_CFG_CMD_ADDR_DATA     (WSPI_CFG_CMD_MODE_FOUR_LINES     | \
                                         WSPI_CFG_ADDR_MODE_FOUR_LINES    | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_FOUR_LINES    | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI2L
#define SNOR_WSPI_CFG_CMD               (WSPI_CFG_CMD_MODE_TWO_LINES      | \
                                         WSPI_CFG_ADDR_MODE_NONE          | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_NONE          | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

#define SNOR_WSPI_CFG_CMD_ADDR          (WSPI_CFG_CMD_MODE_TWO_LINES      | \
                                         WSPI_CFG_ADDR_MODE_TWO_LINES     | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_NONE          | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

#define SNOR_WSPI_CFG_CMD_DATA          (WSPI_CFG_CMD_MODE_TWO_LINES      | \
                                         WSPI_CFG_ADDR_MODE_NONE          | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_TWO_LINES     | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

#define SNOR_WSPI_CFG_CMD_ADDR_DATA     (WSPI_CFG_CMD_MODE_ONE_LINE       | \
                                         WSPI_CFG_ADDR_MODE_ONE_LINE      | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_ONE_LINE      | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI1L
#define SNOR_WSPI_CFG_CMD               (WSPI_CFG_CMD_MODE_ONE_LINE       | \
                                         WSPI_CFG_ADDR_MODE_NONE          | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_NONE          | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

#define SNOR_WSPI_CFG_CMD_ADDR          (WSPI_CFG_CMD_MODE_ONE_LINE       | \
                                         WSPI_CFG_ADDR_MODE_ONE_LINE      | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_NONE          | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

#define SNOR_WSPI_CFG_CMD_DATA          (WSPI_CFG_CMD_MODE_ONE_LINE       | \
                                         WSPI_CFG_ADDR_MODE_NONE          | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_ONE_LINE      | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

#define SNOR_WSPI_CFG_CMD_ADDR_DATA     (WSPI_CFG_CMD_MODE_ONE_LINE       | \
                                         WSPI_CFG_ADDR_MODE_ONE_LINE      | \
                                         WSPI_CFG_ALT_MODE_NONE           | \
                                         WSPI_CFG_DATA_MODE_ONE_LINE      | \
                                         WSPI_CFG_CMD_SIZE_8              | \
                                         WSPI_CFG_ADDR_SIZE_24)

#else
#error "invalid N25Q_BUS_MODE setting"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern flash_descriptor_t snor_descriptor;
#endif

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) && (WSPI_SUPPORTS_MEMMAP == TRUE)
extern const wspi_command_t snor_memmap_read;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void snor_device_init(SNORDriver *devp);
  flash_error_t snor_device_read(SNORDriver *devp, flash_offset_t offset,
                                 size_t n, uint8_t *rp);
  flash_error_t snor_device_program(SNORDriver *devp, flash_offset_t offset,
                                    size_t n, const uint8_t *pp);
  flash_error_t snor_device_start_erase_all(SNORDriver *devp);
  flash_error_t snor_device_start_erase_sector(SNORDriver *devp,
                                               flash_sector_t sector);
  flash_error_t snor_device_verify_erase(SNORDriver *devp,
                                         flash_sector_t sector);
  flash_error_t snor_device_query_erase(SNORDriver *devp, uint32_t *msec);
  flash_error_t snor_device_read_sfdp(SNORDriver *devp, flash_offset_t offset,
                                      size_t n, uint8_t *rp);
#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) &&                            \
    (SNOR_DEVICE_SUPPORTS_XIP == TRUE)
  void snor_activate_xip(SNORDriver *devp);
  void snor_reset_xip(SNORDriver *devp);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_FLASH_DEVICE_H */

/** @} */

