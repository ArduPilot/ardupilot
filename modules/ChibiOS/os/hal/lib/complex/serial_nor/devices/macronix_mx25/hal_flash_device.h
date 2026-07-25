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
 * @brief   Macronix MX25 serial flash driver header.
 *
 * @addtogroup MACRONIX_MX25
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
#define SNOR_DEVICE_SUPPORTS_XIP            FALSE
/** @} */

/**
 * @name    Device identification
 * @{
 */
#define MX25_SUPPORTED_MANUFACTURE_IDS      {0xC2}
#define MX25_SUPPORTED_MEMORY_TYPE_IDS      {0x85}
/** @} */

/**
 * @name    Command codes, SPI mode
 * @{
 */
#define MX25_CMD_SPI_READ3B                 0x03U
#define MX25_CMD_SPI_FAST_READ3B            0x0BU
#define MX25_CMD_SPI_PP3B                   0x02U
#define MX25_CMD_SPI_SE3B                   0x20U
#define MX25_CMD_SPI_BE3B                   0xD8U
#define MX25_CMD_SPI_CE                     0xC7U
#define MX25_CMD_SPI_READ4B                 0x13U
#define MX25_CMD_SPI_FAST_READ4B            0x0CU
#define MX25_CMD_SPI_PP4B                   0x12U
#define MX25_CMD_SPI_SE4B                   0x21U
#define MX25_CMD_SPI_BE4B                   0xDCU
#define MX25_CMD_SPI_WREN                   0x06U
#define MX25_CMD_SPI_WRDI                   0x04U
#define MX25_CMD_SPI_PE_SUSPEND             0xB0U
#define MX25_CMD_SPI_PE_RESUME              0x30U
#define MX25_CMD_SPI_DP                     0xB9U
#define MX25_CMD_SPI_SBL                    0xC0U
#define MX25_CMD_SPI_ENSO                   0xB1U
#define MX25_CMD_SPI_EXSO                   0xC1U
#define MX25_CMD_SPI_NOP                    0x00U
#define MX25_CMD_SPI_RSTEN                  0x66U
#define MX25_CMD_SPI_RST                    0x99U
#define MX25_CMD_SPI_RDID                   0x9FU
#define MX25_CMD_SPI_RDSFDP                 0x5AU
#define MX25_CMD_SPI_RDSR                   0x05U
#define MX25_CMD_SPI_RDCR                   0x15U
#define MX25_CMD_SPI_WRSR                   0x01U
#define MX25_CMD_SPI_RDCR2                  0x71U
#define MX25_CMD_SPI_WRCR2                  0x72U
#define MX25_CMD_SPI_RDFBR                  0x16U
#define MX25_CMD_SPI_WRFBR                  0x17U
#define MX25_CMD_SPI_ESFBR                  0x18U
#define MX25_CMD_SPI_RDSCUR                 0x2BU
#define MX25_CMD_SPI_WRSCUR                 0x2FU
#define MX25_CMD_SPI_WRLR                   0x2CU
#define MX25_CMD_SPI_RDLR                   0x2DU
#define MX25_CMD_SPI_WRSPB                  0xE3U
#define MX25_CMD_SPI_ESSPB                  0xE4U
#define MX25_CMD_SPI_RDSPB                  0xE2U
#define MX25_CMD_SPI_WRDPB                  0xE1U
#define MX25_CMD_SPI_RDDPB                  0xE0U
#define MX25_CMD_SPI_WPSEL                  0x68U
#define MX25_CMD_SPI_GBLK                   0x7EU
#define MX25_CMD_SPI_GBULK                  0x98U
#define MX25_CMD_SPI_RDPASS                 0x27U
#define MX25_CMD_SPI_WRPASS                 0x28U
#define MX25_CMD_SPI_PASSULK                0x29U
/** @} */

/**
 * @name    Command codes, OPI mode
 * @{
 */
#define MX25_CMD_OPI_8READ                  0xEC13U
#define MX25_CMD_OPI_8DTRD                  0xEE11U
#define MX25_CMD_OPI_RDID                   0x9F60U
#define MX25_CMD_OPI_RDSFDP                 0x5AA5U
#define MX25_CMD_OPI_PP                     0x12EDU
#define MX25_CMD_OPI_SE                     0x21DEU
#define MX25_CMD_OPI_BE                     0xDC23U
#define MX25_CMD_OPI_CE                     0xC738U
#define MX25_CMD_OPI_WREN                   0x06F9U
#define MX25_CMD_OPI_WRDI                   0x04FBU
#define MX25_CMD_OPI_PE_SUSPEND             0xB04FU
#define MX25_CMD_OPI_PE_RESUME              0x30CFU
#define MX25_CMD_OPI_DP                     0xB946U
#define MX25_CMD_OPI_SBL                    0xC03FU
#define MX25_CMD_OPI_ENSO                   0xB14EU
#define MX25_CMD_OPI_EXSO                   0xC13EU
#define MX25_CMD_OPI_NOP                    0x00FFU
#define MX25_CMD_OPI_RSTEN                  0x6699U
#define MX25_CMD_OPI_RST                    0x9966U
#define MX25_CMD_OPI_RDSR                   0x05FAU
#define MX25_CMD_OPI_RDCR                   0x15EAU
#define MX25_CMD_OPI_WRSR                   0x01FEU
#define MX25_CMD_OPI_WRCR                   0x01FEU
#define MX25_CMD_OPI_RDCR2                  0x718EU
#define MX25_CMD_OPI_WRCR2                  0x728DU
#define MX25_CMD_OPI_RDFBR                  0x16E9U
#define MX25_CMD_OPI_WRFBR                  0x17E8U
#define MX25_CMD_OPI_ESFBR                  0x18E7U
#define MX25_CMD_OPI_RDSCUR                 0x2BD4U
#define MX25_CMD_OPI_WRSCUR                 0x2FD0U
#define MX25_CMD_OPI_WRLR                   0x2CD3U
#define MX25_CMD_OPI_RDLR                   0x2DD2U
#define MX25_CMD_OPI_WRSPB                  0xE31CU
#define MX25_CMD_OPI_ESSPB                  0xE41BU
#define MX25_CMD_OPI_RDSPB                  0xE21DU
#define MX25_CMD_OPI_WRDPB                  0xE11EU
#define MX25_CMD_OPI_RDDPB                  0xE01FU
#define MX25_CMD_OPI_WPSEL                  0x6897U
#define MX25_CMD_OPI_GBLK                   0x7E81U
#define MX25_CMD_OPI_GBULK                  0x9867U
#define MX25_CMD_OPI_RDPASS                 0x27D8U
#define MX25_CMD_OPI_WRPASS                 0x28D7U
#define MX25_CMD_OPI_PASSULK                0x29D6U
/** @} */

/**
 * @name    Flags status register bits
 * @{
 */
#define MX25_FLAGS_WPSEL                    0x80U
#define MX25_FLAGS_E_FAIL                   0x40U
#define MX25_FLAGS_P_FAIL                   0x20U
#define MX25_FLAGS_ESB                      0x08U
#define MX25_FLAGS_PSB                      0x04U
#define MX25_FLAGS_LDSO                     0x02U
#define MX25_FLAGS_SECURED_OTP              0x01U
#define MX25_FLAGS_ALL_ERRORS               (MX25_FLAGS_E_FAIL |            \
                                             MX25_FLAGS_P_FAIL)
/** @} */

/**
 * @name    Bus interface modes.
 * @{
 */
#define MX25_BUS_MODE_SPI                   0U
#define MX25_BUS_MODE_OPI_STR               1U
#define MX25_BUS_MODE_OPI_DTR               2U
/** @} */

/**
 * @name    MX25-required transfer modes
 * @{
 */
#define MX25_CFG_C8_SPI                 (WSPI_CFG_CMD_SIZE_8            |   \
                                         WSPI_CFG_CMD_MODE_ONE_LINE     |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE)

#define MX25_CFG_C16_8STR               (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE)

#define MX25_CFG_C16_8DTR               (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE        |   \
                                         WSPI_CFG_ALL_DTR)

#define MX25_CFG_C8_A32_SPI             (WSPI_CFG_CMD_SIZE_8            |   \
                                         WSPI_CFG_CMD_MODE_ONE_LINE     |   \
                                         WSPI_CFG_ADDR_MODE_ONE_LINE    |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE)

#define MX25_CFG_C16_A32_8STR           (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE)

#define MX25_CFG_C16_A32_8DTR           (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE        |   \
                                         WSPI_CFG_ALL_DTR)

#define MX25_CFG_C8_DATA_SPI            (WSPI_CFG_CMD_SIZE_8            |   \
                                         WSPI_CFG_CMD_MODE_ONE_LINE     |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_ONE_LINE)

#define MX25_CFG_C16_DATA_8STR          (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_EIGHT_LINES)

#define MX25_CFG_C16_DATA_8DTR          (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ALL_DTR               |   \
                                         WSPI_CFG_DQS_ENABLE)

#define MX25_CFG_C8_A32_DATA_SPI        (WSPI_CFG_CMD_SIZE_8            |   \
                                         WSPI_CFG_CMD_MODE_ONE_LINE     |   \
                                         WSPI_CFG_ADDR_MODE_ONE_LINE    |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_ONE_LINE)

#define MX25_CFG_C16_A32_DATA_8STR      (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_EIGHT_LINES)

#define MX25_CFG_C16_A32_DATA_8DTR      (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ALL_DTR               |   \
                                         WSPI_CFG_DQS_ENABLE)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   SW reset on initialization.
 * @details Enforces a reset on initialization, this could be required if
 *          the device is not reset in HW or during debugging.
 * @note    It is only effective if the WSPI driver is in use, it does
 *          nothing when SPI driver is used.
 */
#if !defined(MX25_RESET_ON_INIT) || defined(__DOXYGEN__)
#define MX25_RESET_ON_INIT                  TRUE
#endif

/**
 * @brief   Switch WSPI bus width on initialization.
 * @details A bus width initialization is performed by writing the
 *          Enhanced Volatile Configuration Register. If the flash
 *          device is configured using the Non Volatile Configuration
 *          Register then this option is not required.
 * @note    This option is only valid in QSPI bus modes.
 */
#if !defined(MX25_SWITCH_WIDTH) || defined(__DOXYGEN__)
#define MX25_SWITCH_WIDTH                   TRUE
#endif

/**
 * @brief   Device bus mode to be used.
 * @note    if @p MX25_SWITCH_WIDTH is @p FALSE then this is the bus mode
 *          that the device is expected to be using.
 * @note    if @p MX25_SWITCH_WIDTH is @p TRUE then this is the bus mode
 *          that the device will be switched in.
 * @note    This option is only valid in WSPI bus mode.
 */
#if !defined(MX25_BUS_MODE) || defined(__DOXYGEN__)
#define MX25_BUS_MODE                       MX25_BUS_MODE_OPI_DTR
#endif

/**
 * @brief   Delays insertions.
 * @details If enabled this options inserts delays into the flash waiting
 *          routines releasing some extra CPU time for threads with lower
 *          priority, this may slow down the driver a bit however.
 */
#if !defined(MX25_NICE_WAITING) || defined(__DOXYGEN__)
#define MX25_NICE_WAITING                   TRUE
#endif

/**
 * @brief   Uses 4kB sub-sectors rather than 64kB sectors.
 */
#if !defined(MX25_USE_SUB_SECTORS) || defined(__DOXYGEN__)
#define MX25_USE_SUB_SECTORS                FALSE
#endif

/**
 * @brief   Number of dummy cycles for fast read (1..15).
 * @details This is the number of dummy cycles to be used for fast read
 *          operations.
 */
#if !defined(MX25_READ_DUMMY_CYCLES) || defined(__DOXYGEN__)
#define MX25_READ_DUMMY_CYCLES              6
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (MX25_READ_DUMMY_CYCLES < 6) || (MX25_READ_DUMMY_CYCLES > 20) ||        \
    ((MX25_READ_DUMMY_CYCLES & 1) != 0)
#error "invalid MX25_READ_DUMMY_CYCLES value (6, 8, 10, 12, 14, 16, 18, 20)"
#endif

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_SPI) &&                             \
    (MX25_BUS_MODE != MX25_BUS_MODE_SPI)
#error "only MX25_BUS_MODE_SPI is allowed when using SPI driver"
#endif

#if (MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR) || defined(__DOXYGEN__)
/**
 * @brief   WSPI settings for command only.
 */
#define SNOR_WSPI_CFG_CMD               MX25_CFG_C16_8DTR

/**
 * @brief   WSPI settings for command and address.
 */
#define SNOR_WSPI_CFG_CMD_ADDR          MX25_CFG_C16_A32_8DTR

/**
 * @brief   WSPI settings for command and data.
 */
#define SNOR_WSPI_CFG_CMD_DATA          MX25_CFG_C16_DATA_8DTR

/**
 * @brief   WSPI settings for command, address and data.
 */
#define SNOR_WSPI_CFG_CMD_ADDR_DATA     MX25_CFG_C16_A32_DATA_8DTR

#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_STR
#define SNOR_WSPI_CFG_CMD               MX25_CFG_C16_8STR
#define SNOR_WSPI_CFG_CMD_ADDR          MX25_CFG_C16_A32_8STR
#define SNOR_WSPI_CFG_CMD_DATA          MX25_CFG_C16_DATA_8STR
#define SNOR_WSPI_CFG_CMD_ADDR_DATA     MX25_CFG_C16_A32_DATA_8STR

#elif MX25_BUS_MODE == MX25_BUS_MODE_SPI
#define SNOR_WSPI_CFG_CMD               MX25_CFG_C8_SPI
#define SNOR_WSPI_CFG_CMD_ADDR          MX25_CFG_C8_A32_SPI
#define SNOR_WSPI_CFG_CMD_DATA          MX25_CFG_C8_DATA_SPI
#define SNOR_WSPI_CFG_CMD_ADDR_DATA     MX25_CFG_C8_A32_DATA_SPI

#else
#error "invalid MX25_BUS_MODE setting"
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

