/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @brief   Winbond W25Q256JV serial flash driver code.
 *
 * @addtogroup WINBOND_W25Q
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

#if W25Q_USE_SUB_SECTORS == TRUE
#define SECTOR_SIZE                         0x00001000U
#define CMD_SECTOR_ERASE                    W25Q_CMD_SUBSECTOR_ERASE
#else
#define SECTOR_SIZE                         0x00010000U
#define CMD_SECTOR_ERASE                    W25Q_CMD_SECTOR_ERASE
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   W25Q128 descriptor.
 */
flash_descriptor_t snor_descriptor = {
  .attributes       = FLASH_ATTR_ERASED_IS_ONE | FLASH_ATTR_ECC_CAPABLE |
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
  .cmd              = W25Q_CMD_FAST_READ,
  .addr             = 0,
  .dummy            = W25Q_READ_DUMMY_CYCLES - 2,
  .cfg              = WSPI_CFG_ADDR_SIZE_24 |
#if W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI1L
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_ADDR_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE |
#elif W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI2L
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
/* Initial W25Q_CMD_READ_ID command.*/
static const wspi_command_t W25Q_cmd_read_id = {
  .cmd              = W25Q_CMD_READ_ID,
  .cfg              = 0U |
#if W25Q_SWITCH_WIDTH == TRUE
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE,
#else
#if W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI1L
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE,
#elif W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI2L
                      WSPI_CFG_CMD_MODE_TWO_LINES |
                      WSPI_CFG_DATA_MODE_TWO_LINES,
#elif W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI4L
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

/* Initial W25Q_CMD_WRITE_ENHANCED_V_CONF_REGISTER command.*/
static const wspi_command_t W25Q_cmd_write_quadspi = {
  .cmd              = W25Q_CMD_WRITE_STATUS_REGISTER2,
  .cfg              = 0U |
#if W25Q_SWITCH_WIDTH == TRUE
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE,
#else
#if W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI1L
                      WSPI_CFG_CMD_MODE_ONE_LINE |
                      WSPI_CFG_DATA_MODE_ONE_LINE,
#elif W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI2L
                      WSPI_CFG_CMD_MODE_TWO_LINES |
                      WSPI_CFG_DATA_MODE_TWO_LINES,
#elif W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI4L
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

/* Initial W25Q_CMD_WRITE_ENABLE command.*/
static const wspi_command_t W25Q_cmd_write_enable = {
  .cmd              = W25Q_CMD_WRITE_ENABLE,
  .cfg              = 0U |
#if W25Q_SWITCH_WIDTH == TRUE
                      WSPI_CFG_CMD_MODE_ONE_LINE,
#else
#if W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI1L
                      WSPI_CFG_CMD_MODE_ONE_LINE,
#elif W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI2L
                      WSPI_CFG_CMD_MODE_TWO_LINES,
#elif W25Q_BUS_MODE == W25Q_BUS_MODE_WSPI4L
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
static const uint8_t W25Q_quadspi_value[1] = {W25Q_FLAGS_QUADSPI_ENABLE};
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static bool W25Q_find_id(const uint8_t *set, size_t size, uint8_t element) {
  size_t i;

  for (i = 0; i < size; i++) {
    if (set[i] == element) {
      return true;
    }
  }
  return false;
}

static flash_error_t W25Q_poll_status(SNORDriver *devp) {
  uint8_t sts;

  do {
#if W25Q_NICE_WAITING == TRUE
    osalThreadSleepMilliseconds(1);
#endif
    /* Read status command.*/
    bus_cmd_receive(devp->config->busp, W25Q_CMD_READ_STATUS_REGISTER,
                    1, &sts);
  } while ((sts & W25Q_FLAGS_PROGRAM_ERASE) != 0U);

  return FLASH_NO_ERROR;
}

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) || defined(__DOXYGEN__)
static void W25Q_reset_memory(SNORDriver *devp) {

  /* 1x W25Q_CMD_RESET_ENABLE command.*/
  static const wspi_command_t cmd_reset_enable_1 = {
    .cmd              = W25Q_CMD_RESET_ENABLE,
    .cfg              = WSPI_CFG_CMD_MODE_ONE_LINE,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* 1x W25Q_CMD_RESET_MEMORY command.*/
  static const wspi_command_t cmd_reset_memory_1 = {
    .cmd              = W25Q_CMD_RESET_MEMORY,
    .cfg              = WSPI_CFG_CMD_MODE_ONE_LINE,
    .addr             = 0,
    .alt              = 0,
    .dummy            = 0
  };

  /* Now the device should be in one bit mode for sure and we perform a
     device reset.*/
  wspiCommand(devp->config->busp, &cmd_reset_enable_1);
  wspiCommand(devp->config->busp, &cmd_reset_memory_1);
}
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

static const uint8_t W25Q_manufacturer_ids[] = W25Q_SUPPORTED_MANUFACTURE_IDS;
static const uint8_t W25Q_memory_type_ids[] = W25Q_SUPPORTED_MEMORY_TYPE_IDS;

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void snor_device_init(SNORDriver *devp) {

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_SPI
  /* Reading device ID.*/
  bus_cmd_receive(devp->config->busp, W25Q_CMD_READ_ID,
                  sizeof devp->device_id, devp->device_id);

#else /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */
  /* Attempting a reset of the device, it could be in an unexpected state
     because a CPU reset does not reset the memory too.*/
  W25Q_reset_memory(devp);

  /* Reading device ID and unique ID.*/
  wspiReceive(devp->config->busp, &W25Q_cmd_read_id, 3U, devp->device_id);
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

  /* Checking if the device is white listed.*/
  osalDbgAssert(W25Q_find_id(W25Q_manufacturer_ids,
                             sizeof W25Q_manufacturer_ids,
                             devp->device_id[0]),
                "invalid manufacturer id");
  osalDbgAssert(W25Q_find_id(W25Q_memory_type_ids,
                             sizeof W25Q_memory_type_ids,
                             devp->device_id[1]),
                "invalid memory type id");

#if (SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI) && (W25Q_SWITCH_WIDTH == TRUE)
  /* Setting up final bus width.*/
  wspiCommand(devp->config->busp, &W25Q_cmd_write_enable);
  /* enable QuadSPI */
  wspiSend(devp->config->busp, &W25Q_cmd_write_quadspi, 1, W25Q_quadspi_value);
  {
    uint8_t id[3];

    /* Reading ID again for confirmation.*/
    bus_cmd_receive(devp->config->busp, W25Q_CMD_READ_ID, 3, id);

    /* Checking if the device is white listed.*/
    osalDbgAssert(memcmp(id, devp->device_id, 3) == 0,
                  "id confirmation failed");
  }
#endif

  /* Setting up the device size.*/
  snor_descriptor.sectors_count = (1U << (size_t)devp->device_id[2]) /
                                  SECTOR_SIZE;
  snor_descriptor.size = (size_t)snor_descriptor.sectors_count * SECTOR_SIZE;
}

/* read some data in either fast or normal mode */
flash_error_t snor_device_read(SNORDriver *devp, flash_offset_t offset,
                               size_t n, uint8_t *rp) {

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
  /* Fast read command in WSPI mode.*/
  bus_cmd_addr_dummy_receive(devp->config->busp, W25Q_CMD_FAST_READ,
                             offset, W25Q_READ_DUMMY_CYCLES, n, rp);
#else
  /* Normal read command in SPI mode.*/
  bus_cmd_addr_receive(devp->config->busp, W25Q_CMD_READ,
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
    bus_cmd(devp->config->busp, W25Q_CMD_WRITE_ENABLE);

    /* Page program command.*/
    bus_cmd_addr_send(devp->config->busp, W25Q_CMD_PAGE_PROGRAM, offset,
                      chunk, pp);

    /* Wait for status and check errors.*/
    err = W25Q_poll_status(devp);
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
  bus_cmd(devp->config->busp, W25Q_CMD_WRITE_ENABLE);

  /* Bulk erase command.*/
  bus_cmd(devp->config->busp, W25Q_CMD_BULK_ERASE);

  return FLASH_NO_ERROR;
}

flash_error_t snor_device_start_erase_sector(SNORDriver *devp,
                                             flash_sector_t sector) {
  flash_offset_t offset = (flash_offset_t)(sector * SECTOR_SIZE);

  /* Enabling write operation.*/
  bus_cmd(devp->config->busp, W25Q_CMD_WRITE_ENABLE);

  /* Sector erase command.*/
  bus_cmd_addr(devp->config->busp, W25Q_CMD_SECTOR_ERASE, offset);

  return FLASH_NO_ERROR;
}

flash_error_t snor_device_verify_erase(SNORDriver *devp,
                                       flash_sector_t sector) {
  uint8_t cmpbuf[W25Q_COMPARE_BUFFER_SIZE];
  flash_offset_t offset;
  size_t n;

  /* Read command.*/
  offset = (flash_offset_t)(sector * SECTOR_SIZE);
  n = SECTOR_SIZE;
  while (n > 0U) {
    uint8_t *p;

#if SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI
   bus_cmd_addr_dummy_receive(devp->config->busp, W25Q_CMD_FAST_READ,
                              offset, W25Q_READ_DUMMY_CYCLES,
                              sizeof cmpbuf, cmpbuf);
#else
   /* Normal read command in SPI mode.*/
   bus_cmd_addr_receive(devp->config->busp, W25Q_CMD_READ,
                        offset, sizeof cmpbuf, cmpbuf);
#endif

    /* Checking for erased state of current buffer.*/
    for (p = cmpbuf; p < &cmpbuf[W25Q_COMPARE_BUFFER_SIZE]; p++) {
      if (*p != 0xFFU) {
        /* Ready state again.*/
        devp->state = FLASH_READY;

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
  bus_cmd_receive(devp->config->busp, W25Q_CMD_READ_STATUS_REGISTER,
                    1, &sts);

  /* If the P/E bit is zero (busy) or the flash in a suspended state then
     report that the operation is still in progress.*/
  if ((sts & W25Q_FLAGS_PROGRAM_ERASE) != 0U) {

    /* Recommended time before polling again, this is a simplified
       implementation.*/
    if (msec != NULL) {
      *msec = 1U;
    }

    return FLASH_BUSY_ERASING;
  }

  /* Read status command.*/
  bus_cmd_receive(devp->config->busp, W25Q_CMD_READ_STATUS_REGISTER2,
                    1, &sts);

  if ((sts & W25Q_FLAGS_ERASE_PROGRAM_SUSPEND) != 0U) {

    /* Recommended time before polling again, this is a simplified
       implementation.*/
    if (msec != NULL) {
      *msec = 1U;
    }

    return FLASH_BUSY_ERASING;
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
  (void)devp;
}

void snor_reset_xip(SNORDriver *devp) {
  (void)devp;
}
#endif /* SNOR_BUS_DRIVER == SNOR_BUS_DRIVER_WSPI */

/** @} */
