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
 * @brief   Micron N25Q serial flash driver code.
 *
 * @addtogroup MICRON_N25Q
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

#if N25Q_USE_SUB_SECTORS == TRUE
#define SECTOR_SIZE                         0x00001000U
#define CMD_SECTOR_ERASE                    N25Q_CMD_SUBSECTOR_ERASE
#else
#define SECTOR_SIZE                         0x00010000U
#define CMD_SECTOR_ERASE                    N25Q_CMD_SECTOR_ERASE
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   N25Q128 descriptor.
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
  .cmd              = N25Q_CMD_FAST_READ,
  .addr             = 0,
  .dummy            = N25Q_READ_DUMMY_CYCLES - 2,
  .cfg              = WSPI_CFG_ADDR_SIZE_24 |
#if N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI1L
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_ADDR_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE |
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI2L
                      WSPI_CFG_CMD_MODE_TWO_LINES |
                      WSPI_CFG_ADDR_MODE_TWO_LINES |
                      WSPI_CFG_DATA_MODE_TWO_LINES |
#else
                      WSPI_CFG_CMD_MODE_FOUR_LINES |
                      WSPI_CFG_ADDR_MODE_FOUR_LINES |
                      WSPI_CFG_DATA_MODE_FOUR_LINES |
#endif
                      WSPI_CFG_ALT_MODE_FOUR_LINES |  /* Always 4 lines, note.*/
                      WSPI_CFG_ALT_SIZE_8 |
                      WSPI_CFG_SIOO
};
#endif
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
/* Initial N25Q_CMD_READ_ID command.*/
static const wspi_command_t n25q_cmd_read_id = {
  .cmd              = N25Q_CMD_READ_ID,
  .cfg              = 0U |
#if N25Q_SWITCH_WIDTH == TRUE
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE,
#else
#if N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI1L
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE,
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI2L
                      WSPI_CFG_CMD_MODE_TWO_LINES |
                      WSPI_CFG_DATA_MODE_TWO_LINES,
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI4L
                      WSPI_CFG_CMD_MODE_FOUR_LINES |
                      WSPI_CFG_DATA_MODE_FOUR_LINES,
#else
                      WSPI_CFG_CMD_MODE_EIGHT_LINES |
                      WSPI_CFG_DATA_MODE_EIGHT_LINES,
#endif
#endif
  .addr             = 0,
  .alt              = 0,
  .dummy            = 0
};

/* Initial N25Q_CMD_WRITE_ENHANCED_V_CONF_REGISTER command.*/
static const wspi_command_t n25q_cmd_write_evconf = {
  .cmd              = N25Q_CMD_WRITE_ENHANCED_V_CONF_REGISTER,
  .cfg              = 0U |
#if N25Q_SWITCH_WIDTH == TRUE
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE,
#else
#if N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI1L
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE,
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI2L
                      WSPI_CFG_CMD_MODE_TWO_LINES |
                      WSPI_CFG_DATA_MODE_TWO_LINES,
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI4L
                      WSPI_CFG_CMD_MODE_FOUR_LINES |
                      WSPI_CFG_DATA_MODE_FOUR_LINES,
#else
                      WSPI_CFG_CMD_MODE_EIGHT_LINES |
                      WSPI_CFG_DATA_MODE_EIGHT_LINES,
#endif
#endif
  .addr             = 0,
  .alt              = 0,
  .dummy            = 0
};

/* Initial N25Q_CMD_WRITE_ENABLE command.*/
static const wspi_command_t n25q_cmd_write_enable = {
  .cmd              = N25Q_CMD_WRITE_ENABLE,
  .cfg              = 0U |
#if N25Q_SWITCH_WIDTH == TRUE
                      WSPI_CFG_CMD_MODE_ONE_LINE,
#else
#if N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI1L
                      WSPI_CFG_CMD_MODE_ONE_LINE,
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI2L
                      WSPI_CFG_CMD_MODE_TWO_LINES,
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI4L
                      WSPI_CFG_CMD_MODE_FOUR_LINES,
#else
                      WSPI_CFG_CMD_MODE_EIGHT_LINES,
#endif
#endif
  .addr             = 0,
  .alt              = 0,
  .dummy            = 0
};

/* Bus width initialization.*/
#if N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI1L
static const uint8_t n25q_evconf_value[1] = {0xCF};
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI2L
static const uint8_t n25q_evconf_value[1] = {0x8F};
#else
static const uint8_t n25q_evconf_value[1] = {0x4F};
#endif
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static bool n25q_find_id(const uint8_t *set, size_t size, uint8_t element) {
  size_t i;

  for (i = 0; i < size; i++) {
    if (set[i] == element) {
      return true;
    }
  }
  return false;
}

static flash_error_t n25q_poll_status(SNORDriver *devp) {
  uint8_t sts;

  do {
#if N25Q_NICE_WAITING == TRUE
    osalThreadSleepMilliseconds(1);
#endif
    /* Read status command.*/
    bus_cmd_receive(devp->config->busp, N25Q_CMD_READ_FLAG_STATUS_REGISTER,
                    1, &sts);
  } while ((sts & N25Q_FLAGS_PROGRAM_ERASE) == 0U);

  /* Checking for errors.*/
  if ((sts & N25Q_FLAGS_ALL_ERRORS) != 0U) {
    /* Clearing status register.*/
    bus_cmd(devp->config->busp, N25Q_CMD_CLEAR_FLAG_STATUS_REGISTER);

    /* Program operation failed.*/
    return FLASH_ERROR_PROGRAM;
  }

  return FLASH_NO_ERROR;
}

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) || defined(__DOXYGEN__)
static void n25q_reset_memory(SNORDriver *devp) {

  /* 1x N25Q_CMD_RESET_ENABLE command.*/
  static const wspi_command_t cmd_reset_enable_1 = {
    .cmd              = N25Q_CMD_RESET_ENABLE,
    .cfg              = WSPI_CFG_CMD_MODE_ONE_LINE,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* 1x N25Q_CMD_RESET_MEMORY command.*/
  static const wspi_command_t cmd_reset_memory_1 = {
    .cmd              = N25Q_CMD_RESET_MEMORY,
    .cfg              = WSPI_CFG_CMD_MODE_ONE_LINE,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* If the device is in one bit mode then the following commands are
     rejected because shorter than 8 bits. If the device is in multiple
     bits mode then the commands are accepted and the device is reset to
     one bit mode.*/
#if N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI4L
  /* 4x N25Q_CMD_RESET_ENABLE command.*/
  static const wspi_command_t cmd_reset_enable_4 = {
    .cmd              = N25Q_CMD_RESET_ENABLE,
    .cfg              = WSPI_CFG_CMD_MODE_FOUR_LINES,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* 4x N25Q_CMD_RESET_MEMORY command.*/
  static const wspi_command_t cmd_reset_memory_4 = {
    .cmd              = N25Q_CMD_RESET_MEMORY,
    .cfg              = WSPI_CFG_CMD_MODE_FOUR_LINES,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  wspiCommand(devp->config->busp, &cmd_reset_enable_4);
  wspiCommand(devp->config->busp, &cmd_reset_memory_4);
#else
  /* 2x N25Q_CMD_RESET_ENABLE command.*/
  static const wspi_command_t cmd_reset_enable_2 = {
    .cmd              = N25Q_CMD_RESET_ENABLE,
    .cfg              = WSPI_CFG_CMD_MODE_TWO_LINES,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* 2x N25Q_CMD_RESET_MEMORY command.*/
  static const wspi_command_t cmd_reset_memory_2 = {
    .cmd              = N25Q_CMD_RESET_MEMORY,
    .cfg              = WSPI_CFG_CMD_MODE_TWO_LINES,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  wspiCommand(devp->config->busp, &cmd_reset_enable_2);
  wspiCommand(devp->config->busp, &cmd_reset_memory_2);
#endif

  /* Now the device should be in one bit mode for sure and we perform a
     device reset.*/
  wspiCommand(devp->config->busp, &cmd_reset_enable_1);
  wspiCommand(devp->config->busp, &cmd_reset_memory_1);
}
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

static const uint8_t n25q_manufacturer_ids[] = N25Q_SUPPORTED_MANUFACTURE_IDS;
static const uint8_t n25q_memory_type_ids[] = N25Q_SUPPORTED_MEMORY_TYPE_IDS;

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void snor_device_init(SNORDriver *devp) {

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_SPI
  /* Reading device ID.*/
  bus_cmd_receive(devp->config->busp, N25Q_CMD_READ_ID,
                  3U, &devp->nocache->buf[0]);

#else /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */
  /* Attempting a reset of the XIP mode, it could be in an unexpected state
     because a CPU reset does not reset the memory too.*/
  snor_reset_xip(devp);

  /* Attempting a reset of the device, it could be in an unexpected state
     because a CPU reset does not reset the memory too.*/
  n25q_reset_memory(devp);

  /* Reading device ID and unique ID.*/
  wspiReceive(devp->config->busp, &n25q_cmd_read_id,
              3U, &devp->nocache->buf[0]);
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

  /* Checking if the device is white listed.*/
  osalDbgAssert(n25q_find_id(n25q_manufacturer_ids,
                             sizeof n25q_manufacturer_ids,
                             devp->nocache->buf[0]),
                "invalid manufacturer id");
  osalDbgAssert(n25q_find_id(n25q_memory_type_ids,
                             sizeof n25q_memory_type_ids,
                             devp->nocache->buf[1]),
                "invalid memory type id");

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) && (N25Q_SWITCH_WIDTH == TRUE)
  /* Setting up final bus width.*/
  wspiCommand(devp->config->busp, &n25q_cmd_write_enable);
  wspiSend(devp->config->busp, &n25q_cmd_write_evconf, 1, n25q_evconf_value);

  {
    /* Reading ID again for confirmation.*/
    bus_cmd_receive(devp->config->busp, N25Q_CMD_MULTIPLE_IO_READ_ID,
                    3U, &devp->nocache->buf[16]);

    /* Checking if the device is white listed.*/
    osalDbgAssert(memcmp(&devp->nocache->buf[0],
                         &devp->nocache->buf[16],
                         3U) == 0,
                  "id confirmation failed");
  }
#endif

  /* Setting up the device size.*/
  snor_descriptor.sectors_count = (1U << (size_t)devp->nocache->buf[2]) /
                                  SECTOR_SIZE;
  snor_descriptor.size = (size_t)snor_descriptor.sectors_count * SECTOR_SIZE;

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
  {
    static const uint8_t flash_conf[1] = {
      (N25Q_READ_DUMMY_CYCLES << 4U) | 0x0FU
    };

    /* Setting up the dummy cycles to be used for fast read operations.*/
    bus_cmd(devp->config->busp, N25Q_CMD_WRITE_ENABLE);
    bus_cmd_send(devp->config->busp, N25Q_CMD_WRITE_V_CONF_REGISTER,
                 1, flash_conf);
  }
#endif
}

flash_error_t snor_device_read(SNORDriver *devp, flash_offset_t offset,
                               size_t n, uint8_t *rp) {

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
  /* Fast read command in WSPI mode.*/
  bus_cmd_addr_dummy_receive(devp->config->busp, N25Q_CMD_FAST_READ,
                             offset, N25Q_READ_DUMMY_CYCLES, n, rp);
#else
  /* Normal read command in SPI mode.*/
  bus_cmd_addr_receive(devp->config->busp, N25Q_CMD_READ,
                       offset, n, rp);
#endif

  return FLASH_NO_ERROR;
}

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

    /* Enabling write operation.*/
    bus_cmd(devp->config->busp, N25Q_CMD_WRITE_ENABLE);

    /* Page program command.*/
    bus_cmd_addr_send(devp->config->busp, N25Q_CMD_PAGE_PROGRAM, offset,
                      chunk, pp);

    /* Wait for status and check errors.*/
    err = n25q_poll_status(devp);
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

flash_error_t snor_device_start_erase_all(SNORDriver *devp) {

  /* Enabling write operation.*/
  bus_cmd(devp->config->busp, N25Q_CMD_WRITE_ENABLE);

  /* Bulk erase command.*/
  bus_cmd(devp->config->busp, N25Q_CMD_BULK_ERASE);

  return FLASH_NO_ERROR;
}

flash_error_t snor_device_start_erase_sector(SNORDriver *devp,
                                             flash_sector_t sector) {
  flash_offset_t offset = (flash_offset_t)(sector * SECTOR_SIZE);

  /* Enabling write operation.*/
  bus_cmd(devp->config->busp, N25Q_CMD_WRITE_ENABLE);

  /* Sector erase command.*/
  bus_cmd_addr(devp->config->busp, N25Q_CMD_SECTOR_ERASE, offset);

  return FLASH_NO_ERROR;
}

flash_error_t snor_device_verify_erase(SNORDriver *devp,
                                       flash_sector_t sector) {
  uint8_t cmpbuf[N25Q_COMPARE_BUFFER_SIZE];
  flash_offset_t offset;
  size_t n;

  /* Read command.*/
  offset = (flash_offset_t)(sector * SECTOR_SIZE);
  n = SECTOR_SIZE;
  while (n > 0U) {
    uint8_t *p;

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
   bus_cmd_addr_dummy_receive(devp->config->busp, N25Q_CMD_FAST_READ,
                              offset, N25Q_READ_DUMMY_CYCLES,
                              sizeof cmpbuf, cmpbuf);
#else
   /* Normal read command in SPI mode.*/
   bus_cmd_addr_receive(devp->config->busp, N25Q_CMD_READ,
                        offset, sizeof cmpbuf, cmpbuf);
#endif

    /* Checking for erased state of current buffer.*/
    for (p = cmpbuf; p < &cmpbuf[N25Q_COMPARE_BUFFER_SIZE]; p++) {
      if (*p != 0xFFU) {
        return FLASH_ERROR_VERIFY;
      }
    }

    offset += sizeof cmpbuf;
    n -= sizeof cmpbuf;
  }

  return FLASH_NO_ERROR;
}

flash_error_t snor_device_query_erase(SNORDriver *devp, uint32_t *msec) {
  uint8_t sts;

  /* Read status command.*/
  bus_cmd_receive(devp->config->busp, N25Q_CMD_READ_FLAG_STATUS_REGISTER,
                  1, &sts);

  /* If the P/E bit is zero (busy) or the flash in a suspended state then
     report that the operation is still in progress.*/
  if (((sts & N25Q_FLAGS_PROGRAM_ERASE) == 0U) ||
      ((sts & N25Q_FLAGS_ERASE_SUSPEND) != 0U)) {

    /* Recommended time before polling again, this is a simplified
       implementation.*/
    if (msec != NULL) {
      *msec = 1U;
    }

    return FLASH_BUSY_ERASING;
  }

  /* Checking for errors.*/
  if ((sts & N25Q_FLAGS_ALL_ERRORS) != 0U) {

    /* Clearing status register.*/
    bus_cmd(devp->config->busp, N25Q_CMD_CLEAR_FLAG_STATUS_REGISTER);

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

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) || defined(__DOXYGEN__)
void snor_activate_xip(SNORDriver *devp) {
  static const uint8_t flash_status_xip[1] = {
    (N25Q_READ_DUMMY_CYCLES << 4U) | 0x07U
  };

  /* Activating XIP mode in the device.*/
  bus_cmd(devp->config->busp, N25Q_CMD_WRITE_ENABLE);
  bus_cmd_send(devp->config->busp, N25Q_CMD_WRITE_V_CONF_REGISTER,
               1, flash_status_xip);
}

void snor_reset_xip(SNORDriver *devp) {
  static const uint8_t flash_conf[1] = {
    (N25Q_READ_DUMMY_CYCLES << 4U) | 0x0FU
  };
  wspi_command_t cmd;
  uint8_t buf[1];

  /* Resetting XIP mode by reading one byte without XIP confirmation bit.*/
  cmd.cmd   = 0U;
  cmd.alt   = 0xFFU;
  cmd.addr  = 0U;
  cmd.dummy = N25Q_READ_DUMMY_CYCLES - 2U;
  cmd.cfg   = WSPI_CFG_CMD_MODE_NONE |
              WSPI_CFG_ADDR_SIZE_24 |
#if N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI1L
              WSPI_CFG_ADDR_MODE_ONE_LINE |
              WSPI_CFG_DATA_MODE_ONE_LINE |
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI2L
              WSPI_CFG_ADDR_MODE_TWO_LINES |
              WSPI_CFG_DATA_MODE_TWO_LINES |
#elif N25Q_BUS_MODE == N25Q_BUS_MODE_WSPI4L
              WSPI_CFG_ADDR_MODE_FOUR_LINES |
              WSPI_CFG_DATA_MODE_FOUR_LINES |
#else
              WSPI_CFG_ADDR_MODE_EIGHT_LINES |
              WSPI_CFG_DATA_MODE_EIGHT_LINES |
#endif
              WSPI_CFG_ALT_MODE_FOUR_LINES |  /* Always 4 lines, note.*/
              WSPI_CFG_ALT_SIZE_8;
  wspiReceive(devp->config->busp, &cmd, 1, buf);

  /* Enabling write operation.*/
  bus_cmd(devp->config->busp, N25Q_CMD_WRITE_ENABLE);

  /* Rewriting volatile configuration register.*/
  bus_cmd_send(devp->config->busp, N25Q_CMD_WRITE_V_CONF_REGISTER,
               1, flash_conf);
}
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

/** @} */
