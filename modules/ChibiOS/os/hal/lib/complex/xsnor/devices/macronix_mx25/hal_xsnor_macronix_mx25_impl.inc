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
 * @file        hal_xsnor_macronix_mx25_impl.inc
 * @brief       Template of SNOR Macronix MX25 source.
 * @note        This is a template file, can be edited directly.
 *
 * @addtogroup  HAL_XSNOR_MACRONIX_MX25
 * @{
 */

/* This is an, automatically generated, implementation file that can be
   manually edited, it is not re-generated if already present.*/

#include <string.h>

#define PAGE_SIZE                                   256U
#define PAGE_MASK                                   (PAGE_SIZE - 1U)
/**
 * @name    Command codes, SPI mode
 * @{
 */
#define CMD_SPI_READ3B                              0x03U
#define CMD_SPI_FAST_READ3B                         0x0BU
#define CMD_SPI_PP3B                                0x02U
#define CMD_SPI_SE3B                                0x20U
#define CMD_SPI_BE3B                                0xD8U
#define CMD_SPI_CE                                  0xC7U
#define CMD_SPI_READ4B                              0x13U
#define CMD_SPI_FAST_READ4B                         0x0CU
#define CMD_SPI_PP4B                                0x12U
#define CMD_SPI_SE4B                                0x21U
#define CMD_SPI_BE4B                                0xDCU
#define CMD_SPI_WREN                                0x06U
#define CMD_SPI_WRDI                                0x04U
#define CMD_SPI_PE_SUSPEND                          0xB0U
#define CMD_SPI_PE_RESUME                           0x30U
#define CMD_SPI_DP                                  0xB9U
#define CMD_SPI_SBL                                 0xC0U
#define CMD_SPI_ENSO                                0xB1U
#define CMD_SPI_EXSO                                0xC1U
#define CMD_SPI_NOP                                 0x00U
#define CMD_SPI_RSTEN                               0x66U
#define CMD_SPI_RST                                 0x99U
#define CMD_SPI_RDID                                0x9FU
#define CMD_SPI_RDSFDP                              0x5AU
#define CMD_SPI_RDSR                                0x05U
#define CMD_SPI_RDCR                                0x15U
#define CMD_SPI_WRSR                                0x01U
#define CMD_SPI_RDCR2                               0x71U
#define CMD_SPI_WRCR2                               0x72U
#define CMD_SPI_RDFBR                               0x16U
#define CMD_SPI_WRFBR                               0x17U
#define CMD_SPI_ESFBR                               0x18U
#define CMD_SPI_RDSCUR                              0x2BU
#define CMD_SPI_WRSCUR                              0x2FU
#define CMD_SPI_WRLR                                0x2CU
#define CMD_SPI_RDLR                                0x2DU
#define CMD_SPI_WRSPB                               0xE3U
#define CMD_SPI_ESSPB                               0xE4U
#define CMD_SPI_RDSPB                               0xE2U
#define CMD_SPI_WRDPB                               0xE1U
#define CMD_SPI_RDDPB                               0xE0U
#define CMD_SPI_WPSEL                               0x68U
#define CMD_SPI_GBLK                                0x7EU
#define CMD_SPI_GBULK                               0x98U
#define CMD_SPI_RDPASS                              0x27U
#define CMD_SPI_WRPASS                              0x28U
#define CMD_SPI_PASSULK                             0x29U
/** @} */

/**
 * @name    Command codes, OPI mode
 * @{
 */
#define CMD_OPI_8READ                               0xEC13U
#define CMD_OPI_8DTRD                               0xEE11U
#define CMD_OPI_RDID                                0x9F60U
#define CMD_OPI_RDSFDP                              0x5AA5U
#define CMD_OPI_PP                                  0x12EDU
#define CMD_OPI_SE                                  0x21DEU
#define CMD_OPI_BE                                  0xDC23U
#define CMD_OPI_CE                                  0xC738U
#define CMD_OPI_WREN                                0x06F9U
#define CMD_OPI_WRDI                                0x04FBU
#define CMD_OPI_PE_SUSPEND                          0xB04FU
#define CMD_OPI_PE_RESUME                           0x30CFU
#define CMD_OPI_DP                                  0xB946U
#define CMD_OPI_SBL                                 0xC03FU
#define CMD_OPI_ENSO                                0xB14EU
#define CMD_OPI_EXSO                                0xC13EU
#define CMD_OPI_NOP                                 0x00FFU
#define CMD_OPI_RSTEN                               0x6699U
#define CMD_OPI_RST                                 0x9966U
#define CMD_OPI_RDSR                                0x05FAU
#define CMD_OPI_RDCR                                0x15EAU
#define CMD_OPI_WRSR                                0x01FEU
#define CMD_OPI_WRCR                                0x01FEU
#define CMD_OPI_RDCR2                               0x718EU
#define CMD_OPI_WRCR2                               0x728DU
#define CMD_OPI_RDFBR                               0x16E9U
#define CMD_OPI_WRFBR                               0x17E8U
#define CMD_OPI_ESFBR                               0x18E7U
#define CMD_OPI_RDSCUR                              0x2BD4U
#define CMD_OPI_WRSCUR                              0x2FD0U
#define CMD_OPI_WRLR                                0x2CD3U
#define CMD_OPI_RDLR                                0x2DD2U
#define CMD_OPI_WRSPB                               0xE31CU
#define CMD_OPI_ESSPB                               0xE41BU
#define CMD_OPI_RDSPB                               0xE21DU
#define CMD_OPI_WRDPB                               0xE11EU
#define CMD_OPI_RDDPB                               0xE01FU
#define CMD_OPI_WPSEL                               0x6897U
#define CMD_OPI_GBLK                                0x7E81U
#define CMD_OPI_GBULK                               0x9867U
#define CMD_OPI_RDPASS                              0x27D8U
#define CMD_OPI_WRPASS                              0x28D7U
#define CMD_OPI_PASSULK                             0x29D6U
/** @} */

/**
 * @name    Flags status register bits
 * @{
 */
#define FLAGS_WPSEL                                 0x80U
#define FLAGS_E_FAIL                                0x40U
#define FLAGS_P_FAIL                                0x20U
#define FLAGS_ESB                                   0x08U
#define FLAGS_PSB                                   0x04U
#define FLAGS_LDSO                                  0x02U
#define FLAGS_SECURED_OTP                           0x01U
#define FLAGS_ALL_ERRORS                            (FLAGS_E_FAIL | FLAGS_P_FAIL)
/** @} */

/**
 * @name    MX25-required transfer modes
 * @{
 */
#define CFG_C8_SPI                      (WSPI_CFG_CMD_SIZE_8            |   \
                                         WSPI_CFG_CMD_MODE_ONE_LINE     |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE)

#define CFG_C16_8STR                    (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE)

#define CFG_C16_8DTR                    (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE        |   \
                                         WSPI_CFG_ALL_DTR)

#define CFG_C8_A32_SPI                  (WSPI_CFG_CMD_SIZE_8            |   \
                                         WSPI_CFG_CMD_MODE_ONE_LINE     |   \
                                         WSPI_CFG_ADDR_MODE_ONE_LINE    |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE)

#define CFG_C16_A32_8STR                (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE)

#define CFG_C16_A32_8DTR                (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_NONE        |   \
                                         WSPI_CFG_ALL_DTR)

#define CFG_C8_DATA_SPI                 (WSPI_CFG_CMD_SIZE_8            |   \
                                         WSPI_CFG_CMD_MODE_ONE_LINE     |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_ONE_LINE)

#define CFG_C16_DATA_8STR               (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_EIGHT_LINES)

#define CFG_C16_DATA_8DTR               (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_NONE        |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ALL_DTR               |   \
                                         WSPI_CFG_DQS_ENABLE)

#define CFG_C8_A32_DATA_SPI             (WSPI_CFG_CMD_SIZE_8            |   \
                                         WSPI_CFG_CMD_MODE_ONE_LINE     |   \
                                         WSPI_CFG_ADDR_MODE_ONE_LINE    |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_ONE_LINE)

#define CFG_C16_A32_DATA_8STR           (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_EIGHT_LINES)

#define CFG_C16_A32_DATA_8DTR           (WSPI_CFG_CMD_SIZE_16           |   \
                                         WSPI_CFG_CMD_MODE_EIGHT_LINES  |   \
                                         WSPI_CFG_ADDR_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ADDR_SIZE_32          |   \
                                         WSPI_CFG_ALT_MODE_NONE         |   \
                                         WSPI_CFG_DATA_MODE_EIGHT_LINES |   \
                                         WSPI_CFG_ALL_DTR               |   \
                                         WSPI_CFG_DQS_ENABLE)
/** @} */

#if (XSNOR_USE_WSPI == TRUE) || defined(__DOXYGEN__)
static const xsnor_commands_t cmd_spi = {
  .cmd                  = CFG_C8_SPI,
  .cmd_addr             = CFG_C8_A32_SPI,
  .cmd_data             = CFG_C8_DATA_SPI,
  .cmd_addr_data        = CFG_C8_A32_DATA_SPI
};

static const xsnor_commands_t cmd_8str = {
  .cmd                  = CFG_C16_8STR,
  .cmd_addr             = CFG_C16_A32_8STR,
  .cmd_data             = CFG_C16_DATA_8STR,
  .cmd_addr_data        = CFG_C16_A32_DATA_8STR
};

static const xsnor_commands_t cmd_8dtr = {
  .cmd                  = CFG_C16_8DTR,
  .cmd_addr             = CFG_C16_A32_8DTR,
  .cmd_data             = CFG_C16_DATA_8DTR,
  .cmd_addr_data        = CFG_C16_A32_DATA_8DTR
};

/* 1x MX25_CMD_SPI_RSTEN command.*/
static const wspi_command_t cmd_reset_enable_spi = {
  .cmd              = CMD_SPI_RSTEN,
  .cfg              = CFG_C8_SPI,
  .addr             = 0,
  .alt              = 0,
  .dummy            = 0
};

/* 1x MX25_CMD_SPI_RST command.*/
static const wspi_command_t cmd_reset_memory_spi = {
  .cmd              = CMD_SPI_RST,
  .cfg              = CFG_C8_SPI,
  .addr             = 0,
  .alt              = 0,
  .dummy            = 0
};

/* 8xSTR MX25_CMD_OPI_RSTEN command.*/
static const wspi_command_t cmd_reset_enable_8str = {
  .cmd              = CMD_OPI_RSTEN,
  .cfg              = CFG_C16_8STR,
  .addr             = 0,
  .alt              = 0,
  .dummy            = 0
};

/* 8xSTR MX25_CMD_OPI_RST command.*/
static const wspi_command_t cmd_reset_memory_8str = {
  .cmd              = CMD_OPI_RST,
  .cfg              = CFG_C16_8STR,
  .addr             = 0,
  .alt              = 0,
  .dummy            = 0
};

/* 8xDTR MX25_CMD_OPI_RSTEN command.*/
static const wspi_command_t cmd_reset_enable_8dtr = {
  .cmd              = CMD_OPI_RSTEN,
  .cfg              = CFG_C16_8DTR,
  .addr             = 0,
  .alt              = 0,
  .dummy            = 0
};

/* 8xDTR MX25_CMD_OPI_RST command.*/
static const wspi_command_t cmd_reset_memory_8dtr = {
  .cmd              = CMD_OPI_RST,
  .cfg              = CFG_C16_8DTR,
  .addr             = 0,
  .alt              = 0,
  .dummy            = 0
};
#endif /* XSNOR_USE_WSPI == TRUE */

/* Device identifiers.*/
static const uint8_t mx25_manufacturer_ids[] = {0xC2};
static const uint8_t mx25_memory_type_ids[] = {0x85};

/*===========================================================================*/
/* Module local functions.                                                   */
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

static flash_error_t mx25_poll_status(hal_xsnor_macronix_mx25_c *self) {

  do {
    if ((self->config->options & MX25_OPT_NICE_WAITING) != 0U) {
      osalThreadSleepMilliseconds(1);
    }

    /* Read status command.*/
    if (self->config->bus_type == XSNOR_BUS_MODE_SPI) {
      __xsnor_bus_cmd_receive(self, CMD_SPI_RDSR,
                            1U, &self->config->buffers->databuf[0]);
    }
    else {
      __xsnor_bus_cmd_addr_dummy_receive(self, CMD_OPI_RDSR,
                                         0U, 4U, 2U,
                                         &self->config->buffers->databuf[0]);
    }
  } while ((self->config->buffers->databuf[0] & 1U) != 0U);

  /* Reading security register and checking for errors.*/
  if (self->config->bus_type == XSNOR_BUS_MODE_SPI) {
    __xsnor_bus_cmd_receive(self, CMD_SPI_RDSCUR,
                          1U, &self->config->buffers->databuf[0]);
  }
  else {
    __xsnor_bus_cmd_addr_dummy_receive(self, CMD_OPI_RDSCUR,
                                       0U, 4U, 2U,  /* Note: always 4 dummies.*/
                                       &self->config->buffers->databuf[0]);
  }

  if ((self->config->buffers->databuf[0] & FLAGS_ALL_ERRORS) != 0U) {

    return FLASH_ERROR_PROGRAM;
  }

  return FLASH_NO_ERROR;
}

static inline bool mx25_spi_4bytes(const hal_xsnor_macronix_mx25_c *self) {

  return (self->descriptor.attributes & FLASH_ATTR_SPI_4BYTES_ADDR_HINT) != 0U;
}

#if (XSNOR_USE_WSPI == TRUE) || defined(__DOXYGEN__)
static void mx25_reset_memory(hal_xsnor_macronix_mx25_c *self) {

  /* If the device is in one bit mode then the following commands are
     rejected because shorter than 8 bits. If the device is in multiple
     bits mode then the commands are accepted and the device is reset to
     one bit mode.*/
  if ((self->config->options &MX25_OPT_USE_DTR) != 0U) {
    wspiCommand(self->config->bus.wspi.drv, &cmd_reset_enable_8dtr);
    wspiCommand(self->config->bus.wspi.drv, &cmd_reset_memory_8dtr);
  }
  else {
    wspiCommand(self->config->bus.wspi.drv, &cmd_reset_enable_8str);
    wspiCommand(self->config->bus.wspi.drv, &cmd_reset_memory_8str);
  }

  /* Now the device should be in one bit mode for sure and we perform a
     device reset.*/
  wspiCommand(self->config->bus.wspi.drv, &cmd_reset_enable_spi);
  wspiCommand(self->config->bus.wspi.drv, &cmd_reset_memory_spi);

  /* The device requires at least 10uS to recover after a reset, it could
     need up to 100mS in case a reset occurred during a chip erase, 50uS
     covers most cases.*/
  osalThreadSleepMicroseconds(50);
}

static void mx25_read_id(hal_xsnor_macronix_mx25_c *self, uint8_t *buf) {
  wspi_command_t cmd;

  if ((self->config->options & MX25_OPT_NO_WIDTH_SWITCH) != 0U) {
    /* Not switching, assuming it is already in final bus mode.*/
    switch (self->config->bus_type) {
    case XSNOR_BUS_MODE_SPI:
      cmd.cfg = CFG_C8_DATA_SPI;
      cmd.cmd = CMD_SPI_RDID;
      break;
    case XSNOR_BUS_MODE_WSPI_8LINES:
      if ((self->config->options & MX25_OPT_USE_DTR) != 0U) {
        cmd.cfg = CFG_C16_A32_DATA_8DTR;
       }
      else {
        cmd.cfg = CFG_C16_A32_DATA_8STR;
      }
      cmd.cmd = CMD_OPI_RDID;
      break;
    default:
      osalDbgAssert(false, "invalid bus type");
      buf[0] = 0;
      buf[1] = 0;
      buf[2] = 0;
      return;
    }
  }
  else {
    /* It is post-reset, before going to switch, assuming SPI.*/
    cmd.cfg = CFG_C8_DATA_SPI;
    cmd.cmd = CMD_SPI_RDID;
  }
  cmd.addr  = 0U;
  cmd.alt   = 0U;
  cmd.dummy = 0U;
  wspiReceive(self->config->bus.wspi.drv, &cmd, 3U, buf);
}

static void mx25_write_cr2(hal_xsnor_macronix_mx25_c *self,
                           uint32_t addr, const uint8_t *value) {
  wspi_command_t cmd;

  if ((self->config->options & MX25_OPT_NO_WIDTH_SWITCH) != 0U) {
    /* Not switching, assuming it is already in final bus mode.*/
    switch (self->config->bus_type) {
    case XSNOR_BUS_MODE_SPI:
      cmd.cfg = CFG_C8_SPI;
      cmd.cmd = CMD_SPI_WREN;
      break;
    case XSNOR_BUS_MODE_WSPI_8LINES:
      if ((self->config->options & MX25_OPT_USE_DTR) != 0U) {
        cmd.cfg = CFG_C16_8DTR;
      }
      else {
        cmd.cfg = CFG_C16_8STR;
      }
      cmd.cmd = CMD_OPI_WREN;
      break;
    default:
      osalDbgAssert(false, "invalid bus type");
      return;
    }
  }
  else {
    /* It is post-reset, before going to switch, assuming SPI.*/
    cmd.cfg = CFG_C8_SPI;
    cmd.cmd = CMD_SPI_WREN;
  }
  cmd.addr  = 0U;
  cmd.alt   = 0U;
  cmd.dummy = 0U;
  wspiCommand(self->config->bus.wspi.drv, &cmd);

  if ((self->config->options & MX25_OPT_NO_WIDTH_SWITCH) != 0U) {
    /* Not switching, assuming it is already in final bus mode.*/
    switch (self->config->bus_type) {
    case XSNOR_BUS_MODE_SPI:
      cmd.cfg = CFG_C8_A32_DATA_SPI;
      cmd.cmd = CMD_SPI_WRCR2;
      break;
    case XSNOR_BUS_MODE_WSPI_8LINES:
      if ((self->config->options & MX25_OPT_USE_DTR) != 0U) {
        cmd.cfg = CFG_C16_A32_DATA_8DTR;
      }
      else {
        cmd.cfg = CFG_C16_A32_DATA_8STR;
      }
      cmd.cmd = CMD_OPI_WRCR2;
      break;
    default:
      osalDbgAssert(false, "invalid bus type");
      return;
    }
  }
  else {
    /* It is post-reset, before going to switch, assuming SPI.*/
    cmd.cfg = CFG_C8_A32_DATA_SPI;
    cmd.cmd = CMD_SPI_WRCR2;
  }
  cmd.addr  = addr;
  cmd.alt   = 0U;
  cmd.dummy = 0U;
  wspiSend(self->config->bus.wspi.drv, &cmd, 1, value);
}
#endif /* XSNOR_USE_WSPI == TRUE */

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module class "hal_xsnor_macronix_mx25_c" methods.                         */
/*===========================================================================*/

/**
 * @name        Methods implementations of hal_xsnor_macronix_mx25_c
 * @{
 */
/**
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance to be initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @return                      A new reference to the object.
 */
void *__mx25_objinit_impl(void *ip, const void *vmt) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;

  /* Initialization of the ancestors-defined parts.*/
  __xsnor_objinit_impl(self, vmt);

  /* Initialization code.*/
  self->descriptor.attributes    = FLASH_ATTR_ERASED_IS_ONE |
                                   FLASH_ATTR_REWRITABLE |
                                   FLASH_ATTR_SUSPEND_ERASE_CAPABLE;
  self->descriptor.page_size     = 256U;
  self->descriptor.sectors_count = 0U; /* Overwritten.*/
  self->descriptor.sectors       = NULL;
  self->descriptor.sectors_size  = 0U;
  self->descriptor.address       = 0U;
  self->descriptor.size          = 0U; /* Overwritten.*/

  return self;
}

/**
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance to be disposed.
 */
void __mx25_dispose_impl(void *ip) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;

  /* Finalization code.*/
  /* Implementation.*/

  /* Finalization of the ancestors-defined parts.*/
  __xsnor_dispose_impl(self);
}

/**
 * @brief       Override of method @p xsnor_device_init().
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance.
 * @return                      An error code.
 */
flash_error_t __mx25_init_impl(void *ip) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;
  const xsnor_config_t *config = self->config;

  /* Bus type and width setup, only necessary if WSPI is in use.*/
#if XSNOR_USE_WSPI == TRUE
  switch (config->bus_type) {
  case XSNOR_BUS_MODE_SPI:
    self->commands = NULL;
    break;
  case XSNOR_BUS_MODE_WSPI_8LINES:
    if ((self->config->options &MX25_OPT_USE_DTR) != 0U) {
      self->commands = &cmd_8dtr;
    }
    else {
      self->commands = &cmd_8str;
    }
    break;
  default:
    osalDbgAssert(false, "invalid bus type");
    self->commands = NULL;
    return FLASH_ERROR_HW_FAILURE;
  }
#endif

#if XSNOR_USE_BOTH == TRUE
  if (config->bus_type == XSNOR_BUS_MODE_SPI)
#endif
#if XSNOR_USE_SPI == TRUE
  {
    /* Initialization procedure in the SPI case.
       Reading device ID.*/
    __xsnor_bus_cmd_receive(self, CMD_SPI_RDID,
                            3U, &config->buffers->databuf[0]);

    /* Manufacturer identifier.*/
    if (!mx25_find_id(mx25_manufacturer_ids,
                      sizeof mx25_manufacturer_ids,
                      config->buffers->databuf[0])) {
      return FLASH_ERROR_HW_FAILURE;
    }

    /* Memory identifier.*/
    if (!mx25_find_id(mx25_memory_type_ids,
                      sizeof mx25_memory_type_ids,
                      config->buffers->databuf[1])) {
      return FLASH_ERROR_HW_FAILURE;
    }
  }
#endif /* XSNOR_USE_SPI == TRUE */
#if XSNOR_USE_BOTH == TRUE
  else
#endif
#if XSNOR_USE_WSPI == TRUE
  {
    /* Attempting a reset of the device, it could be in an unexpected state
       because a CPU reset does not reset the memory too.*/
    mx25_reset_memory(self);

    /* Reading device ID and unique ID.*/
    mx25_read_id(self, &config->buffers->databuf[0]);

    /* Manufacturer identifier.*/
    if (!mx25_find_id(mx25_manufacturer_ids,
                      sizeof mx25_manufacturer_ids,
                      config->buffers->databuf[0])) {
      return FLASH_ERROR_HW_FAILURE;
    }

    /* Memory identifier.*/
    if (!mx25_find_id(mx25_memory_type_ids,
                      sizeof mx25_memory_type_ids,
                      config->buffers->databuf[1])) {
      return FLASH_ERROR_HW_FAILURE;
    }

    /* Setting up dummy cycles, note this is a volatile-only setting.*/
    config->buffers->databuf[4] = ~(((self->config->options & MX25_OPT_DUMMY_CYCLES_MASK) - 6U) / 2U) & 7U;
    mx25_write_cr2(self, 0x00000300U, &config->buffers->databuf[4]);

    /* Checking if the bus switch is needed, there are extra steps in
       that case.*/
    if ((self->config->options & MX25_OPT_NO_WIDTH_SWITCH) == 0U) {

      /* Switching to final bus mode then reading the ID again for
         confirmation.*/
      switch (self->config->bus_type) {
      case XSNOR_BUS_MODE_SPI:
        {
          static const uint8_t cr2_spi[1] = {0x00};
          mx25_write_cr2(self, 0x00000000U, cr2_spi);
          __xsnor_bus_cmd_receive(self, CMD_SPI_RDID, 3U,
                                  &config->buffers->databuf[16]);
        }
        break;
      case XSNOR_BUS_MODE_WSPI_8LINES:
        if ((self->config->options & MX25_OPT_USE_DTR) != 0U) {
          static const uint8_t cr2_opi_dtr[1] = {0x02};
          mx25_write_cr2(self, 0x00000000U, cr2_opi_dtr);
          __xsnor_bus_cmd_addr_dummy_receive(self, CMD_OPI_RDID,
                                             0U, 4U, 6U, /* Note: always 4 dummies.*/
                                             &config->buffers->databuf[16]);

          /* In DTR mode bytes are read twice, adjusting.*/
          config->buffers->databuf[16 + 1] = config->buffers->databuf[16 + 2];
          config->buffers->databuf[16 + 2] = config->buffers->databuf[16 + 4];
        }
        else {
          static const uint8_t cr2_opi_str[1] = {0x01};
          mx25_write_cr2(self, 0x00000000U, cr2_opi_str);
          __xsnor_bus_cmd_addr_dummy_receive(self, CMD_OPI_RDID,
                                             0U, 4U, 3U, /* Note: always 4 dummies.*/
                                             &config->buffers->databuf[16]);
        }
        break;
      default:
        return FLASH_ERROR_HW_FAILURE;
      }

      /* Comparing with the previously read ID.*/
      if (memcmp(&config->buffers->databuf[0],
                 &config->buffers->databuf[16], 3) != 0U) {
        return FLASH_ERROR_HW_FAILURE;
      }
    }
  }
#endif /* XSNOR_USE_WSPI == TRUE */

  /* Variable parts of the descriptor.*/
  if ((self->config->options & MX25_OPT_USE_SUBSECTORS) != 0U) {
    self->descriptor.sectors_size = 0x00001000U;
  }
  else {
    self->descriptor.sectors_size = 0x00010000U;
  }
  self->descriptor.size = (uint32_t)(1U << ((size_t)config->buffers->databuf[2] & 0x1FU));
  self->descriptor.sectors_count = self->descriptor.size / self->descriptor.sectors_size;

  if (self->descriptor.size > 0x01000000U) {
    self->descriptor.attributes |= FLASH_ATTR_SPI_4BYTES_ADDR_HINT;
  }
  else {
    self->descriptor.attributes &= ~FLASH_ATTR_SPI_4BYTES_ADDR_HINT;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief       Override of method @p xsnor_device_read().
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance.
 * @param[in]     offset        Flash offset.
 * @param[in]     n             Number of bytes to be read.
 * @param[out]    rp            Pointer to the data buffer.
 * @return                      An error code.
 */
flash_error_t __mx25_read_impl(void *ip, flash_offset_t offset, size_t n,
                               uint8_t *rp) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;

  /* Command depends on bus mode.*/
  switch (self->config->bus_type) {
  case XSNOR_BUS_MODE_SPI:
    __xsnor_bus_cmd_addr_dummy_receive(self,
                                       mx25_spi_4bytes(self) ?
                                       CMD_SPI_FAST_READ4B : CMD_SPI_FAST_READ3B,
                                       offset, 8U,   /* Always 8 dummies. */
                                       n, rp);
    break;
  case XSNOR_BUS_MODE_WSPI_8LINES:
    if ((self->config->options & MX25_OPT_USE_DTR) != 0U) {
      __xsnor_bus_cmd_addr_dummy_receive(self, CMD_OPI_8DTRD, offset,
                                         self->config->options & MX25_OPT_DUMMY_CYCLES_MASK,
                                         n, rp);
    }
    else {
      __xsnor_bus_cmd_addr_dummy_receive(self, CMD_OPI_8READ, offset,
                                         self->config->options & MX25_OPT_DUMMY_CYCLES_MASK,
                                         n, rp);
    }
    break;
  default:
    osalDbgAssert(false, "invalid bus type");
    return FLASH_ERROR_HW_FAILURE;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief       Override of method @p xsnor_device_program().
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance.
 * @param[in]     offset        Flash offset.
 * @param[in]     n             Number of bytes to be programmed.
 * @param[in]     pp            Pointer to the data buffer.
 * @return                      An error code.
 */
flash_error_t __mx25_program_impl(void *ip, flash_offset_t offset, size_t n,
                                  const uint8_t *pp) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;

  /* Data is programmed page by page.*/
  while (n > 0U) {
    flash_error_t err;

    /* Data size that can be written in a single program page operation.*/
    size_t chunk = (size_t)(((offset | PAGE_MASK) + 1U) - offset);
    if (chunk > n) {
      chunk = n;
    }

    /* Command depends on bus mode.*/
    switch (self->config->bus_type) {
    case XSNOR_BUS_MODE_SPI:
      /* Enabling write operation.*/
      __xsnor_bus_cmd(self, CMD_SPI_WREN);

      /* Page program command.*/
      __xsnor_bus_cmd_addr_send(self,
                                mx25_spi_4bytes(self) ?
                                CMD_SPI_PP4B : CMD_SPI_PP3B,
                                offset, chunk, pp);
      break;
    case XSNOR_BUS_MODE_WSPI_8LINES:
      /* Enabling write operation.*/
      __xsnor_bus_cmd(self, CMD_OPI_WREN);

      /* Page program command.*/
      __xsnor_bus_cmd_addr_send(self, CMD_OPI_PP, offset, chunk, pp);
      break;
    default:
      osalDbgAssert(false, "invalid bus type");
      return FLASH_ERROR_HW_FAILURE;
    }

    /* Wait for status and check errors.*/
    err = mx25_poll_status(self);
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
 * @brief       Override of method @p xsnor_device_start_erase_all().
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance.
 * @return                      An error code.
 */
flash_error_t __mx25_start_erase_all_impl(void *ip) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;

  /* Command depends on bus mode.*/
  switch (self->config->bus_type) {
  case XSNOR_BUS_MODE_SPI:
    /* Enabling write operation.*/
    __xsnor_bus_cmd(self, CMD_SPI_WREN);

    /* Bulk erase command.*/
    __xsnor_bus_cmd(self, CMD_SPI_CE);
    break;
  case XSNOR_BUS_MODE_WSPI_8LINES:
    /* Enabling write operation.*/
    __xsnor_bus_cmd(self, CMD_OPI_WREN);

    /* Bulk erase command.*/
    __xsnor_bus_cmd(self, CMD_OPI_CE);
    break;
  default:
    osalDbgAssert(false, "invalid bus type");
    return FLASH_ERROR_HW_FAILURE;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief       Override of method @p xsnor_device_start_erase_sector().
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance.
 * @param[in]     sector        Sector to be erased.
 * @return                      An error code.
 */
flash_error_t __mx25_start_erase_sector_impl(void *ip, flash_sector_t sector) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;
  flash_offset_t offset = (flash_offset_t)(sector * self->descriptor.sectors_size);

  /* Command depends on bus mode.*/
  switch (self->config->bus_type) {
  case XSNOR_BUS_MODE_SPI:
    /* Enabling write operation.*/
    __xsnor_bus_cmd(self, CMD_SPI_WREN);

    /* Sector erase command.*/
    if ((self->config->options & MX25_OPT_USE_SUBSECTORS) == 0U) {
      __xsnor_bus_cmd_addr(self,
                           mx25_spi_4bytes(self) ?
                           CMD_SPI_BE4B : CMD_SPI_BE3B,
                           offset);
    }
    else {
      __xsnor_bus_cmd_addr(self,
                           mx25_spi_4bytes(self) ?
                           CMD_SPI_SE4B : CMD_SPI_SE3B,
                           offset);
    }
    break;
  case XSNOR_BUS_MODE_WSPI_8LINES:
    /* Enabling write operation.*/
    __xsnor_bus_cmd(self, CMD_OPI_WREN);

    /* Sector erase command.*/
    if ((self->config->options & MX25_OPT_USE_SUBSECTORS) == 0U) {
      __xsnor_bus_cmd_addr(self, CMD_OPI_BE, offset);
    }
    else {
      __xsnor_bus_cmd_addr(self, CMD_OPI_SE, offset);
    }
    break;
  default:
    osalDbgAssert(false, "invalid bus type");
    return FLASH_ERROR_HW_FAILURE;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief       Override of method @p xsnor_device_query_erase().
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance.
 * @param[out]    msec          Recommended time, in milliseconds, that should
 *                              be spent before calling this function again,
 *                              can be @p NULL
 * @return                      An error code.
 */
flash_error_t __mx25_query_erase_impl(void *ip, unsigned *msec) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;

  /* Command depends on bus mode.*/
  switch (self->config->bus_type) {
  case XSNOR_BUS_MODE_SPI:
    /* Reading SR.*/
    __xsnor_bus_cmd_receive(self, CMD_SPI_RDSR, 1U,
                            &self->config->buffers->databuf[0]);

    /* Reading SCUR.*/
    __xsnor_bus_cmd_receive(self, CMD_SPI_RDSCUR, 1U,
                            &self->config->buffers->databuf[16]);
    break;
  case XSNOR_BUS_MODE_WSPI_8LINES:
    /* Reading SR.*/
    __xsnor_bus_cmd_addr_dummy_receive(self, CMD_OPI_RDSR,
                                       0U, 4U, 2U,  /* Note: always 4 dummies.*/
                                       &self->config->buffers->databuf[0]);

    /* Reading SCUR.*/
    __xsnor_bus_cmd_addr_dummy_receive(self, CMD_OPI_RDSCUR,
                                       0U, 4U, 2U,  /* Note: always 4 dummies.*/
                                       &self->config->buffers->databuf[16]);
    break;
  default:
    osalDbgAssert(false, "invalid bus type");
    return FLASH_ERROR_HW_FAILURE;
  }

  /* If the WIP bit is one (busy) or the flash in a suspended state then
     report that the operation is still in progress.*/
  if (((self->config->buffers->databuf[0] & 1) != 0U) ||
      ((self->config->buffers->databuf[16] & 8) != 0U)) {

    /* Recommended time before polling again, this is a simplified
       implementation.*/
    if (msec != NULL) {
      *msec = 1U;
    }

    return FLASH_BUSY_ERASING;
  }

  /* Checking for errors.*/
  if ((self->config->buffers->databuf[16] & FLAGS_ALL_ERRORS) != 0U) {

    /* Erase operation failed.*/
    return FLASH_ERROR_ERASE;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief       Override of method @p xsnor_device_verify_erase().
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance.
 * @param[in]     sector        Sector to be verified.
 * @return                      An error code.
 */
flash_error_t __mx25_verify_erase_impl(void *ip, flash_sector_t sector) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;
  flash_offset_t offset;
  size_t n;

  /* Read command.*/
  offset = (flash_offset_t)(sector * self->descriptor.sectors_size);
  n = self->descriptor.sectors_size;
  while (n > 0U) {
    uint8_t *p;

   __mx25_read_impl(self, offset,
                    XSNOR_BUFFER_SIZE, &self->config->buffers->databuf[0]);

    /* Checking for erased state of current buffer.*/
    for (p = &self->config->buffers->databuf[0];
         p < &self->config->buffers->databuf[XSNOR_BUFFER_SIZE];
         p++) {

      if (*p != 0xFFU) {
        return FLASH_ERROR_VERIFY;
      }
    }

    offset += XSNOR_BUFFER_SIZE;
    n -= XSNOR_BUFFER_SIZE;
  }

  return FLASH_NO_ERROR;
}

/**
 * @brief       Override of method @p xsnor_device_mmap_on().
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance.
 * @param[out]    addrp         Pointer to the memory mapped memory or @p NULL
 * @return                      An error code.
 */
flash_error_t __mx25_mmap_on_impl(void *ip, uint8_t **addrp) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;

#if XSNOR_USE_BOTH == TRUE
  if (self->config->bus_type == XSNOR_BUS_MODE_SPI) {
    return FLASH_ERROR_UNIMPLEMENTED;
  }
#endif

#if XSNOR_USE_WSPI == FALSE
  return FLASH_ERROR_UNIMPLEMENTED;
#else /* XSNOR_USE_WSPI == TRUE */
  {
    wspi_command_t cmd;

    /* Bus acquisition.*/
    __xsnor_bus_acquire(self);

    /* Command depends on bus mode.*/
    switch (self->config->bus_type) {
    case XSNOR_BUS_MODE_SPI:
      cmd.cfg   = CFG_C8_A32_DATA_SPI;
      cmd.cmd   = CMD_SPI_FAST_READ4B;
      cmd.dummy = 8U;                   /* Note, always 8 for this command.*/
      break;
    case XSNOR_BUS_MODE_WSPI_8LINES:
      if ((self->config->options & MX25_OPT_USE_DTR) != 0U) {
        cmd.cfg = CFG_C16_A32_DATA_8DTR;
        cmd.cmd = CMD_OPI_8DTRD;
      }
      else {
        cmd.cfg = CFG_C16_A32_DATA_8STR;
        cmd.cmd = CMD_OPI_8READ;
      }
      cmd.dummy = (uint32_t)(self->config->options & MX25_OPT_DUMMY_CYCLES_MASK);
      break;
    default:
      osalDbgAssert(false, "invalid bus type");
      return FLASH_ERROR_HW_FAILURE;
    }
    cmd.addr  = 0U;
    cmd.alt   = 0U;
    wspiMapFlash(self->config->bus.wspi.drv, &cmd, addrp);

    /* Bus release.*/
    __xsnor_bus_release(self);

    return FLASH_NO_ERROR;
  }
#endif
}

/**
 * @brief       Override of method @p xsnor_device_mmap_off().
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_macronix_mx25_c
 *                              instance.
 */
void __mx25_mmap_off_impl(void *ip) {
  hal_xsnor_macronix_mx25_c *self = (hal_xsnor_macronix_mx25_c *)ip;

#if XSNOR_USE_WSPI == TRUE
  /* Bus acquisition.*/
  __xsnor_bus_acquire(self);

  /* Stopping WSPI memory mapped mode.*/
  wspiUnmapFlash(self->config->bus.wspi.drv);

  /* Bus release.*/
  __xsnor_bus_release(self);
#endif
}
/** @} */

/**
 * @brief       VMT structure of SNOR Macronix MX25 driver class.
 * @note        It is public because accessed by the inlined constructor.
 */
const struct hal_xsnor_macronix_mx25_vmt __hal_xsnor_macronix_mx25_vmt = {
  .dispose                  = __mx25_dispose_impl,
  .init                     = __mx25_init_impl,
  .read                     = __mx25_read_impl,
  .program                  = __mx25_program_impl,
  .start_erase_all          = __mx25_start_erase_all_impl,
  .start_erase_sector       = __mx25_start_erase_sector_impl,
  .query_erase              = __mx25_query_erase_impl,
  .verify_erase             = __mx25_verify_erase_impl,
  .mmap_on                  = __mx25_mmap_on_impl,
  .mmap_off                 = __mx25_mmap_off_impl
};

/** @} */
