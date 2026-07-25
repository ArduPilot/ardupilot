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
 * @file    hal_flash_device.c
 * @brief   Macronix MX25 serial flash driver code.
 *
 * @addtogroup MACRONIX_MX25
 * @{
 */

#include <string.h>

#include "hal.h"
#include "hal_serial_nor.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define PAGE_SIZE                           256U
#define PAGE_MASK                           (PAGE_SIZE - 1U)

#if MX25_USE_SUB_SECTORS == TRUE
#define SECTOR_SIZE                         0x00001000U
#define CMD_SECTOR_ERASE                    MX25_CMD_SUBSECTOR_ERASE
#else
#define SECTOR_SIZE                         0x00010000U
#define CMD_SECTOR_ERASE                    MX25_CMD_SECTOR_ERASE
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   MX25LM51245G descriptor.
 */
flash_descriptor_t snor_descriptor = {
  .attributes       = FLASH_ATTR_ERASED_IS_ONE | FLASH_ATTR_REWRITABLE |
                      FLASH_ATTR_SUSPEND_ERASE_CAPABLE,
  .page_size        = 256U,
  .sectors_count    = 0U,           /* It is overwritten.*/
  .sectors          = NULL,
  .sectors_size     = SECTOR_SIZE,
  .address          = 0U,
  .size             = 0U            /* It is overwritten.*/
};

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) || defined(__DOXYGEN__)
#if (WSPI_SUPPORTS_MEMMAP == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Fast read command for memory mapped mode.
 */
const wspi_command_t snor_memmap_read = {
  .addr             = 0U,
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
  .cmd              = MX25_CMD_SPI_FAST_READ4B,
  .dummy            = 8,                /* Note, always 8 for this command. */
  .cfg              = MX25_CFG_C8_A32_DATA_SPI
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_STR
  .cmd              = MX25_CMD_OPI_8READ,
  .dummy            = MX25_READ_DUMMY_CYCLES,
  .cfg              = MX25_CFG_C16_A32_DATA_8STR
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR
  .cmd              = MX25_CMD_OPI_8DTRD,
  .dummy            = MX25_READ_DUMMY_CYCLES,
  .cfg              = MX25_CFG_C16_A32_DATA_8DTR
#endif
};
#endif /* WSPI_SUPPORTS_MEMMAP == TRUE */
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
/* Initial MX25_CMD_READ_ID command.*/
static const wspi_command_t mx25_cmd_read_id = {

#if MX25_SWITCH_WIDTH == TRUE
  .cmd              = MX25_CMD_SPI_RDID,
  .cfg              = MX25_CFG_C8_DATA_SPI,
  .dummy            = 0,
#else
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
  .cmd              = MX25_CMD_SPI_RDID,
  .cfg              = MX25_CFG_C8_DATA_SPI,
  .dummy            = 0,
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_STR
  .cmd              = MX25_CMD_OPI_RDID,
  .cfg              = MX25_CFG_C16_A32_DATA_8STR,
  .dummy            = 4U,                       /* Note: always 4 dummies.  */
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR
  .cmd              = MX25_CMD_OPI_RDID,
  .cfg              = MX25_CFG_C16_A32_DATA_8DTR
  .dummy            = 4U,                       /* Note: always 4 dummies.  */
#endif
#endif
  .addr             = 0,
  .alt              = 0
};

static const uint8_t mx25_manufacturer_ids[] = MX25_SUPPORTED_MANUFACTURE_IDS;
static const uint8_t mx25_memory_type_ids[] = MX25_SUPPORTED_MEMORY_TYPE_IDS;
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static bool mx25_find_id(const uint8_t *set, size_t size, uint8_t element) {
  size_t i;

  for (i = 0; i < size; i++) {
    if (set[i] == element) {
      return true;
    }
  }
  return false;
}

static flash_error_t mx25_poll_status(SNORDriver *devp) {

  do {
#if MX25_NICE_WAITING == TRUE
    osalThreadSleepMilliseconds(1);
#endif
    /* Read status command.*/
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
    bus_cmd_receive(devp->config->busp, MX25_CMD_SPI_RDSR, 1U, devp->nocache->buf1);
#else
    bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_RDSR,
                               0U, 4U, 2U,
                               devp->nocache->buf); /* Note: always 4 dummies.*/
#endif
  } while ((devp->nocache->buf[0] & 1U) != 0U);

  /* Reading security register and checking for errors.*/
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
  bus_cmd_receive(devp->config->busp, MX25_CMD_SPI_RDSCUR,
                  1U, devp->nocache->buf1);
#else
  bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_RDSCUR,
                             0U, 4U, 2U,
                             devp->nocache->buf); /* Note: always 4 dummies.*/
#endif
  if ((devp->nocache->buf[0] & MX25_FLAGS_ALL_ERRORS) != 0U) {

    return FLASH_ERROR_PROGRAM;
  }

  return FLASH_NO_ERROR;
}

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) || defined(__DOXYGEN__)
/**
 * @brief   Device software reset.
 * @note    It attempts to reset first in supposed final bus mode then tries
 *          in SPI mode.
 *
 * @param[in] devp      pointer to a @p SNORDriver instance
 */
static void mx25_reset(SNORDriver *devp) {

  /* 1x MX25_CMD_SPI_RSTEN command.*/
  static const wspi_command_t cmd_reset_enable_1 = {
    .cmd              = MX25_CMD_SPI_RSTEN,
    .cfg              = MX25_CFG_C8_SPI,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* 1x MX25_CMD_SPI_RST command.*/
  static const wspi_command_t cmd_reset_memory_1 = {
    .cmd              = MX25_CMD_SPI_RST,
    .cfg              = MX25_CFG_C8_SPI,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* If the device is in one bit mode then the following commands are
     rejected because shorter than 8 bits. If the device is in multiple
     bits mode then the commands are accepted and the device is reset to
     one bit mode.*/
#if MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR
  /* 8xDTR MX25_CMD_OPI_RSTEN command.*/
  static const wspi_command_t cmd_reset_enable_8dtr = {
    .cmd              = MX25_CMD_OPI_RSTEN,
    .cfg              = MX25_CFG_C16_8DTR,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* 8xDTR MX25_CMD_OPI_RST command.*/
  static const wspi_command_t cmd_reset_memory_8dtr = {
    .cmd              = MX25_CMD_OPI_RST,
    .cfg              = MX25_CFG_C16_8DTR,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  wspiCommand(devp->config->busp, &cmd_reset_enable_8dtr);
  wspiCommand(devp->config->busp, &cmd_reset_memory_8dtr);
#else
  /* 8xSTR MX25_CMD_OPI_RSTEN command.*/
  static const wspi_command_t cmd_reset_enable_8str = {
    .cmd              = MX25_CMD_OPI_RSTEN,
    .cfg              = MX25_CFG_C16_8STR,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* 8xSTR MX25_CMD_OPI_RST command.*/
  static const wspi_command_t cmd_reset_memory_8str = {
    .cmd              = MX25_CMD_OPI_RST,
    .cfg              = MX25_CFG_C16_8STR,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  wspiCommand(devp->config->busp, &cmd_reset_enable_8str);
  wspiCommand(devp->config->busp, &cmd_reset_memory_8str);
#endif

  /* Now the device should be in one bit mode for sure and we perform a
     device reset.*/
  wspiCommand(devp->config->busp, &cmd_reset_enable_1);
  wspiCommand(devp->config->busp, &cmd_reset_memory_1);
}

/**
 * @brief   Writes a CR2 register in after-reset bus mode.
 * @note    This function can only be used before the device is switched to
 *          the final bus width.
 *
 * @param[in] devp      pointer to a @p SNORDriver instance
 * @param[in] addr      address field
 * @param[in] value     value to be written
 */
static void mx25_write_cr2(SNORDriver *devp, uint32_t addr, const uint8_t *value) {

  static const wspi_command_t cmd_write_enable = {
#if MX25_SWITCH_WIDTH == TRUE
    .cmd              = MX25_CMD_SPI_WREN,
    .cfg              = MX25_CFG_C8_SPI,
#else
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
    .cmd              = MX25_CMD_SPI_WREN,
    .cfg              = MX25_CFG_C8_SPI,
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_STR
    .cmd              = MX25_CMD_OPI_WREN,
    .cfg              = MX25_CFG_C16_8STR,
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR
    .cmd              = MX25_CMD_OPI_WREN,
    .cfg              = MX25_CFG_C16_8DTR,
#endif
#endif
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

#if MX25_SWITCH_WIDTH == TRUE
  devp->nocache->cmd.cmd    = MX25_CMD_SPI_WRCR2,
  devp->nocache->cmd.cfg    = MX25_CFG_C8_A32_DATA_SPI,
#else
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
  devp->nocache->cmd.cmd    = MX25_CMD_SPI_WRCR2,
  devp->nocache->cmd.cfg    = MX25_CFG_C8_A32_DATA_SPI,
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_STR
  devp->nocache->cmd.cmd    = MX25_CMD_OPI_WRCR2,
  devp->nocache->cmd.cfg    = MX25_CFG_C16_A32_DATA_8STR,
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR
  devp->nocache->cmd.cmd    = MX25_CMD_OPI_WRCR2,
  devp->nocache->cmd.cfg    = MX25_CFG_C16_A32_DATA_8DTR,
#endif
#endif
  devp->nocache->cmd.addr   = addr;
  devp->nocache->cmd.alt    = 0U;
  devp->nocache->cmd.dummy  = 0U;

  wspiCommand(devp->config->busp, &cmd_write_enable);
  wspiSend(devp->config->busp, &devp->nocache->cmd, 1, value);
}
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Device initialization.
 *
 * @param[in] devp      pointer to a @p SNORDriver instance
 */
void snor_device_init(SNORDriver *devp) {

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_SPI
  /* Reading device ID.*/
  bus_cmd_receive(devp->config->busp, MX25_CMD_READ_ID,
                  3U, &devp->nocache->buf[0]);

#else /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

#if MX25_RESET_ON_INIT == TRUE
  {
    /* Attempting a reset of the device, it could be in an unexpected state
       because a CPU reset does not reset the memory too.*/
    mx25_reset(devp);

    /* The device requires at least 10uS to recover after a reset, it could
       need up to 100mS in cause a reset occurred during a chip erase, 50uS
       covers most cases.*/
    osalThreadSleepMicroseconds(50);
  }
#endif

  /* Reading device ID and unique ID.*/
  wspiReceive(devp->config->busp, &mx25_cmd_read_id, 3U,
              &devp->nocache->buf[0]);
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

  /* Checking if the device is white listed.*/
  osalDbgAssert(mx25_find_id(mx25_manufacturer_ids,
                             sizeof mx25_manufacturer_ids,
                             devp->nocache->buf[0]),
                "invalid manufacturer id");
  osalDbgAssert(mx25_find_id(mx25_memory_type_ids,
                             sizeof mx25_memory_type_ids,
                             devp->nocache->buf[1]),
                "invalid memory type id");

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
  /* Setting up the dummy cycles to be used for fast read operations.*/
  {
    static const uint8_t v[1] = {~((MX25_READ_DUMMY_CYCLES - 6U) / 2U) & 7U};
    mx25_write_cr2(devp, 0x00000300U, v);
  }
#endif

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) && (MX25_SWITCH_WIDTH == TRUE)
  {
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
    static const uint8_t v[1] = {0x00};
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_STR
    static const uint8_t v[1] = {0x01};
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR
    static const uint8_t v[1] = {0x02};
#endif

    /* Setting up final bus width.*/
    mx25_write_cr2(devp, 0x00000000U, v);

    /* Reading ID again for confirmation, in DTR mode bytes are read twice,
       it needs adjusting.*/
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
    bus_cmd_receive(devp->config->busp, MX25_CMD_SPI_RDID, 3U,
                    &devp->nocache->buf[16]);
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_STR
    bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_RDID,
                               0U, 4U, 3U,
                               &devp->nocache->buf[16]); /* Note: always 4 dummies.*/
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR
    bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_RDID,
                               0U, 4U, 6U,
                               &devp->nocache->buf[16]); /* Note: always 4 dummies.*/
    devp->nocache->buf[16 + 1] = devp->nocache->buf[16 + 2];
    devp->nocache->buf[16 + 2] = devp->nocache->buf[16 + 4];
#endif

    /* Checking if the device is white listed.*/
    osalDbgAssert(memcmp(&devp->nocache->buf[0],
                         &devp->nocache->buf[16], 3) == 0,
                  "id confirmation failed");
  }
#endif

  /* Setting up the device size.*/
  snor_descriptor.sectors_count = (1U << ((size_t)devp->nocache->buf[2] & 0x1FU)) /
                                  SECTOR_SIZE;
  snor_descriptor.size = (size_t)snor_descriptor.sectors_count * SECTOR_SIZE;
}

/**
 * @brief   Device read.
 *
 * @param[in] devp      pointer to a @p SNORDriver instance
 * @param[in] offset    flash offset
 * @param[in] n         number of bytes
 * @param[out] rp       pointer to the buffer
 */
flash_error_t snor_device_read(SNORDriver *devp, flash_offset_t offset,
                               size_t n, uint8_t *rp) {

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
  /* Fast read command in WSPI mode.*/
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
  bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_SPI_FAST_READ4B,
                             offset, 8,     /* Note, always 8 dummy cycles. */
                             n, rp);
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR
  bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_8DTRD,
                             offset, MX25_READ_DUMMY_CYCLES, n, rp);
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_STR
  bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_8READ,
                             offset, MX25_READ_DUMMY_CYCLES, n, rp);
#endif
#else
  /* Normal read command in SPI mode.*/
  bus_cmd_addr_receive(devp->config->busp, MX25_CMD_SPI_READ4B,
                       offset, n, rp);
#endif

  return FLASH_NO_ERROR;
}


/**
 * @brief   Device program.
 *
 * @param[in] devp      pointer to a @p SNORDriver instance
 * @param[in] offset    flash offset
 * @param[in] n         number of bytes
 * @param[in] pp        pointer to the buffer
 */
flash_error_t snor_device_program(SNORDriver *devp, flash_offset_t offset,
                                  size_t n, const uint8_t *pp) {

  /* Data is programmed page by page.*/
  while (n > 0U) {
    flash_error_t err;

    /* Data size that can be written in a single program page operation.*/
    size_t chunk = (size_t)(((offset | PAGE_MASK) + 1U) - offset);
    if (chunk > n) {
      chunk = n;
    }

#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
    /* Enabling write operation.*/
    bus_cmd(devp->config->busp, MX25_CMD_SPI_WREN);

    /* Page program command.*/
    bus_cmd_addr_send(devp->config->busp, MX25_CMD_SPI_PP4B, offset,
                      chunk, pp);
#else
    /* Enabling write operation.*/
    bus_cmd(devp->config->busp, MX25_CMD_OPI_WREN);

    /* Page program command.*/
    bus_cmd_addr_send(devp->config->busp, MX25_CMD_OPI_PP, offset,
                      chunk, pp);
#endif

    /* Wait for status and check errors.*/
    err = mx25_poll_status(devp);
    if (err != FLASH_NO_ERROR) {

      return err;
    }

    /* Next page.*/
    offset += chunk;
    pp     += chunk;
    n      -= chunk;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief   Device global erase start.
 *
 * @param[in] devp      pointer to a @p SNORDriver instance
 */
flash_error_t snor_device_start_erase_all(SNORDriver *devp) {

#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
  /* Enabling write operation.*/
  bus_cmd(devp->config->busp, MX25_CMD_SPI_WREN);

  /* Bulk erase command.*/
  bus_cmd(devp->config->busp, MX25_CMD_SPI_CE);
#else
  /* Enabling write operation.*/
  bus_cmd(devp->config->busp, MX25_CMD_OPI_WREN);

  /* Bulk erase command.*/
  bus_cmd(devp->config->busp, MX25_CMD_OPI_CE);
#endif

  return FLASH_NO_ERROR;
}


/**
 * @brief   Device sector erase start.
 *
 * @param[in] devp      pointer to a @p SNORDriver instance
 * @param[in] sector    flash sector
 */
flash_error_t snor_device_start_erase_sector(SNORDriver *devp,
                                             flash_sector_t sector) {
  flash_offset_t offset = (flash_offset_t)(sector * SECTOR_SIZE);

#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
  /* Enabling write operation.*/
  bus_cmd(devp->config->busp, MX25_CMD_SPI_WREN);

#if MX25_USE_SUB_SECTORS == FALSE
  /* Block erase command.*/
  bus_cmd_addr(devp->config->busp, MX25_CMD_SPI_BE4B, offset);
#else
  /* Sector erase command.*/
  bus_cmd_addr(devp->config->busp, MX25_CMD_SPI_SE4B, offset);
#endif
#else
  /* Enabling write operation.*/
  bus_cmd(devp->config->busp, MX25_CMD_OPI_WREN);

#if MX25_USE_SUB_SECTORS == FALSE
  /* Block erase command.*/
  bus_cmd_addr(devp->config->busp, MX25_CMD_OPI_BE, offset);
#else
  /* Sector erase command.*/
  bus_cmd_addr(devp->config->busp, MX25_CMD_OPI_SE, offset);
#endif
#endif

  return FLASH_NO_ERROR;
}

/**
 * @brief   Device erase verify.
 *
 * @param[in] devp      pointer to a @p SNORDriver instance
 * @param[in] sector    flash sector
 */
flash_error_t snor_device_verify_erase(SNORDriver *devp,
                                       flash_sector_t sector) {
  flash_offset_t offset;
  size_t n;

  /* Read command.*/
  offset = (flash_offset_t)(sector * SECTOR_SIZE);
  n = SECTOR_SIZE;
  while (n > 0U) {
    uint8_t *p;

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
    bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_SPI_FAST_READ4B,
                               offset, 8,   /* Note, always 8 dummy cycles.*/
                               SNOR_BUFFER_SIZE, devp->nocache->buf);
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_STR
   bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_8READ,
                              offset, MX25_READ_DUMMY_CYCLES,
                              SNOR_BUFFER_SIZE, devp->nocache->buf);
#elif MX25_BUS_MODE == MX25_BUS_MODE_OPI_DTR
   bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_8DTRD,
                              offset, MX25_READ_DUMMY_CYCLES,
                              SNOR_BUFFER_SIZE, devp->nocache->buf);
#endif
#else
   /* Normal read command in SPI mode.*/
   bus_cmd_addr_receive(devp->config->busp, MX25_CMD_SPI_READ4B,
                        offset, sizeof cmpbuf, cmpbuf);
#endif

    /* Checking for erased state of current buffer.*/
    for (p = devp->nocache->buf; p < &devp->nocache->buf[SNOR_BUFFER_SIZE]; p++) {
      if (*p != 0xFFU) {
        return FLASH_ERROR_VERIFY;
      }
    }

    offset += SNOR_BUFFER_SIZE;
    n -= SNOR_BUFFER_SIZE;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief   Queries if there is an erase in progress.
 *
 * @param[in] devp      pointer to a @p SNORDriver instance
 * @param[out] msec     suggested number of milliseconds before calling this
 *                      function again
 */
flash_error_t snor_device_query_erase(SNORDriver *devp, uint32_t *msec) {

  /* Read status register.*/
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
  bus_cmd_receive(devp->config->busp, MX25_CMD_SPI_RDSR, 1U,
                  &devp->nocache->buf[0]);
#else
  bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_RDSR,
                             0U, 4U, 2U,
                             &devp->nocache->buf[0]); /* Note: always 4 dummies.*/
#endif

  /* Read security register.*/
#if MX25_BUS_MODE == MX25_BUS_MODE_SPI
  bus_cmd_receive(devp->config->busp, MX25_CMD_SPI_RDSCUR, 1U,
                  &devp->nocache->buf[16]);
#else
  bus_cmd_addr_dummy_receive(devp->config->busp, MX25_CMD_OPI_RDSCUR,
                             0U, 4U, 2U,
                             &devp->nocache->buf[16]); /* Note: always 4 dummies.*/
#endif

  /* If the WIP bit is one (busy) or the flash in a suspended state then
     report that the operation is still in progress.*/
  if (((devp->nocache->buf[0] & 1) != 0U) ||
      ((devp->nocache->buf[16] & 8) != 0U)) {

    /* Recommended time before polling again, this is a simplified
       implementation.*/
    if (msec != NULL) {
      *msec = 1U;
    }

    return FLASH_BUSY_ERASING;
  }

  /* Checking for errors.*/
  if ((devp->nocache->buf[16] & MX25_FLAGS_ALL_ERRORS) != 0U) {

    /* Erase operation failed.*/
    return FLASH_ERROR_ERASE;
  }

  return FLASH_NO_ERROR;
}

flash_error_t snor_device_read_sfdp(SNORDriver *devp, flash_offset_t offset,
                                    size_t n, uint8_t *rp) {

  (void)devp;
  (void)rp;
  (void)offset;
  (void)n;

  return FLASH_NO_ERROR;
}

/** @} */
