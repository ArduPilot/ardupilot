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
 * @file        hal_xsnor_base.c
 * @brief       Generated SNOR Base Driver source.
 * @note        This is a generated file, do not edit directly.
 *
 * @addtogroup  HAL_XSNOR_BASE
 * @{
 */

#include "hal.h"
#include "hal_xsnor_base.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module local macros.                                                      */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module class "hal_xsnor_base_c" methods.                                  */
/*===========================================================================*/

/**
 * @name        Interfaces implementation of hal_xsnor_base_c
 * @{
 */
/**
 * @brief       Implementation of interface method @p flsGetDescriptor().
 *
 * @param[in,out] ip            Pointer to the @p flash_interface_i class
 *                              interface.
 * @return                      A flash device descriptor.
 */
static const flash_descriptor_t *__xsnor_fls_get_descriptor_impl(void *ip) {
  hal_xsnor_base_c *self = oopIfGetOwner(hal_xsnor_base_c, ip);

  return &self->descriptor;
}

/**
 * @brief       Implementation of interface method @p flsRead().
 *
 * @param[in,out] ip            Pointer to the @p flash_interface_i class
 *                              interface.
 * @param[in]     offset        Flash offset.
 * @param[in]     n             Number of bytes to be read.
 * @param[out]    rp            Pointer to the data buffer.
 * @return                      An error code.
 */
static flash_error_t __xsnor_fls_read_impl(void *ip, flash_offset_t offset,
                                           size_t n, uint8_t *rp) {
  hal_xsnor_base_c *self = oopIfGetOwner(hal_xsnor_base_c, ip);
  flash_error_t err;

  osalDbgCheck((self != NULL) && (rp != NULL) && (n > 0U));
  osalDbgAssert((self->state == FLASH_READY) || (self->state == FLASH_ERASE),
                "invalid state");

  if (self->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Bus acquired.*/
  __xsnor_bus_acquire(self);

  /* FLASH_READY state while the operation is performed.*/
  self->state = FLASH_READ;

  /* Actual read implementation.*/
  err = xsnor_device_read(self, offset, n, rp);

  /* Ready state again.*/
  self->state = FLASH_READY;

  /* Bus released.*/
  __xsnor_bus_release(self);

  return err;
}

/**
 * @brief       Implementation of interface method @p flsProgram().
 *
 * @param[in,out] ip            Pointer to the @p flash_interface_i class
 *                              interface.
 * @param[in]     offset        Flash offset.
 * @param[in]     n             Number of bytes to be programmed.
 * @param[in]     pp            Pointer to the data buffer.
 * @return                      An error code.
 */
static flash_error_t __xsnor_fls_program_impl(void *ip, flash_offset_t offset,
                                              size_t n, const uint8_t *pp) {
  hal_xsnor_base_c *self = oopIfGetOwner(hal_xsnor_base_c, ip);
  flash_error_t err;

  osalDbgCheck((self != NULL) && (pp != NULL) && (n > 0U));
  osalDbgAssert((self->state == FLASH_READY) || (self->state == FLASH_ERASE),
                "invalid state");

  if (self->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Bus acquired.*/
  __xsnor_bus_acquire(self);

  /* FLASH_PGM state while the operation is performed.*/
  self->state = FLASH_PGM;

  /* Actual program implementation.*/
  err = xsnor_device_program(self, offset, n, pp);

  /* Ready state again.*/
  self->state = FLASH_READY;

  /* Bus released.*/
  __xsnor_bus_release(self);

  return err;
}

/**
 * @brief       Implementation of interface method @p flsStartEraseAll().
 *
 * @param[in,out] ip            Pointer to the @p flash_interface_i class
 *                              interface.
 * @return                      An error code.
 */
static flash_error_t __xsnor_fls_start_erase_all_impl(void *ip) {
  hal_xsnor_base_c *self = oopIfGetOwner(hal_xsnor_base_c, ip);
  flash_error_t err;

  osalDbgCheck(self != NULL);
  osalDbgAssert((self->state == FLASH_READY) || (self->state == FLASH_ERASE),
                "invalid state");

  if (self->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Bus acquired.*/
  __xsnor_bus_acquire(self);

  /* FLASH_ERASE state while the operation is performed.*/
  self->state = FLASH_ERASE;

  /* Actual erase implementation.*/
  err = xsnor_device_start_erase_all(self);

  /* Restore ready state on failure.*/
  if (err != FLASH_NO_ERROR) {
    self->state = FLASH_READY;
  }

  /* Bus released.*/
  __xsnor_bus_release(self);

  return err;
}

/**
 * @brief       Implementation of interface method @p flsStartEraseSector().
 *
 * @param[in,out] ip            Pointer to the @p flash_interface_i class
 *                              interface.
 * @param[in]     sector        Sector to be erased.
 * @return                      An error code.
 */
static flash_error_t __xsnor_fls_start_erase_sector_impl(void *ip,
                                                         flash_sector_t sector) {
  hal_xsnor_base_c *self = oopIfGetOwner(hal_xsnor_base_c, ip);
  flash_error_t err;

  osalDbgCheck(self != NULL);
  osalDbgAssert((self->state == FLASH_READY) || (self->state == FLASH_ERASE),
                "invalid state");

  if (self->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Bus acquired.*/
  __xsnor_bus_acquire(self);

  /* FLASH_ERASE state while the operation is performed.*/
  self->state = FLASH_ERASE;

  /* Actual erase implementation.*/
  err = xsnor_device_start_erase_sector(self, sector);

  /* Restore ready state on failure.*/
  if (err != FLASH_NO_ERROR) {
    self->state = FLASH_READY;
  }

  /* Bus released.*/
  __xsnor_bus_release(self);

  return err;
}

/**
 * @brief       Implementation of interface method @p flsQueryErase().
 *
 * @param[in,out] ip            Pointer to the @p flash_interface_i class
 *                              interface.
 * @param[out]    msec          Recommended time, in milliseconds, that should
 *                              be spent before calling this function again,
 *                              can be @p NULL
 * @return                      An error code.
 */
static flash_error_t __xsnor_fls_query_erase_impl(void *ip, unsigned *msec) {
  hal_xsnor_base_c *self = oopIfGetOwner(hal_xsnor_base_c, ip);
  flash_error_t err;

  osalDbgCheck(self != NULL);
  osalDbgAssert((self->state == FLASH_READY) || (self->state == FLASH_ERASE),
                "invalid state");

  /* If there is an erase in progress then the device must be checked.*/
  if (self->state == FLASH_ERASE) {

    /* Bus acquired.*/
    __xsnor_bus_acquire(self);

    /* Actual query erase implementation.*/
    err = xsnor_device_query_erase(self, msec);

    /* The device is ready to accept commands.*/
    if (err == FLASH_NO_ERROR) {
      self->state = FLASH_READY;
    }

    /* Bus released.*/
    __xsnor_bus_release(self);
  }
  else {
    err = FLASH_NO_ERROR;
  }

  return err;
}

/**
 * @brief       Implementation of interface method @p flsVerifyErase().
 *
 * @param[in,out] ip            Pointer to the @p flash_interface_i class
 *                              interface.
 * @param[in]     sector        Sector to be verified.
 * @return                      An error code.
 */
static flash_error_t __xsnor_fls_verify_erase_impl(void *ip,
                                                   flash_sector_t sector) {
  hal_xsnor_base_c *self = oopIfGetOwner(hal_xsnor_base_c, ip);
  flash_error_t err;

  osalDbgCheck(self != NULL);
  osalDbgAssert((self->state == FLASH_READY) || (self->state == FLASH_ERASE),
                "invalid state");

  if (self->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* Bus acquired.*/
  __xsnor_bus_acquire(self);

  /* FLASH_READY state while the operation is performed.*/
  self->state = FLASH_READ;

  /* Actual verify erase implementation.*/
  err = xsnor_device_verify_erase(self, sector);

  /* Ready state again.*/
  self->state = FLASH_READY;

  /* Bus released.*/
  __xsnor_bus_release(self);

  return err;
}

/**
 * @brief       Implementation of interface method @p flsAcquireExclusive().
 *
 * @param[in,out] ip            Pointer to the @p flash_interface_i class
 *                              interface.
 * @return                      An error code.
 */
static flash_error_t __xsnor_fls_acquire_exclusive_impl(void *ip) {
  hal_xsnor_base_c *self = oopIfGetOwner(hal_xsnor_base_c, ip);

  osalMutexLock(&self->mutex);
  return FLASH_NO_ERROR;
}

/**
 * @brief       Implementation of interface method @p flsReleaseExclusive().
 *
 * @param[in,out] ip            Pointer to the @p flash_interface_i class
 *                              interface.
 * @return                      An error code.
 */
static flash_error_t __xsnor_fls_release_exclusive_impl(void *ip) {
  hal_xsnor_base_c *self = oopIfGetOwner(hal_xsnor_base_c, ip);

  osalMutexUnlock(&self->mutex);
  return FLASH_NO_ERROR;
}
/** @} */

/**
 * @name        Methods implementations of hal_xsnor_base_c
 * @{
 */
/**
 * @brief       Implementation of object creation.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[out]    ip            Pointer to a @p hal_xsnor_base_c instance to be
 *                              initialized.
 * @param[in]     vmt           VMT pointer for the new object.
 * @return                      A new reference to the object.
 */
void *__xsnor_objinit_impl(void *ip, const void *vmt) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;

  /* Initialization of the ancestors-defined parts.*/
  __bo_objinit_impl(self, vmt);

  /* Initialization of interface flash_interface_i.*/
  {
    static const struct flash_interface_vmt xsnor_fls_vmt = {
      .instance_offset      = offsetof(hal_xsnor_base_c, fls),
      .get_descriptor       = __xsnor_fls_get_descriptor_impl,
      .read                 = __xsnor_fls_read_impl,
      .program              = __xsnor_fls_program_impl,
      .start_erase_all      = __xsnor_fls_start_erase_all_impl,
      .start_erase_sector   = __xsnor_fls_start_erase_sector_impl,
      .query_erase          = __xsnor_fls_query_erase_impl,
      .verify_erase         = __xsnor_fls_verify_erase_impl,
      .acquire_exclusive    = __xsnor_fls_acquire_exclusive_impl,
      .release_exclusive    = __xsnor_fls_release_exclusive_impl
    };
    oopIfObjectInit(&self->fls, &xsnor_fls_vmt);
  }

  /* Initialization code.*/
  self->state    = FLASH_STOP;
  self->config   = NULL;
#if XSNOR_USE_WSPI == TRUE
  self->commands = NULL;
#endif
  osalMutexObjectInit(&self->mutex);

  return self;
}

/**
 * @brief       Implementation of object finalization.
 * @note        This function is meant to be used by derived classes.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance to be
 *                              disposed.
 */
void __xsnor_dispose_impl(void *ip) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;

  /* Finalization code.*/
  self->state = FLASH_UNINIT;

  /* Finalization of the ancestors-defined parts.*/
  __bo_dispose_impl(self);
}
/** @} */

/**
 * @name        Regular methods of hal_xsnor_base_c
 * @{
 */
#if (XSNOR_USE_SPI == TRUE) || defined (__DOXYGEN__)
/**
 * @brief       Sends command and address over SPI.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     cmd           Instruction code.
 * @param[in]     offset        Flash offset.
 */
void __xsnor_spi_cmd_addr(void *ip, uint32_t cmd, flash_offset_t offset) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  const xsnor_config_t *config = self->config;

  config->buffers->spibuf[0] = cmd;
  if ((self->descriptor.attributes & FLASH_ATTR_SPI_4BYTES_ADDR_HINT) != 0U) {
    config->buffers->spibuf[1] = (uint8_t)(offset >> 24);
    config->buffers->spibuf[2] = (uint8_t)(offset >> 16);
    config->buffers->spibuf[3] = (uint8_t)(offset >> 8);
    config->buffers->spibuf[4] = (uint8_t)(offset >> 0);
    spiSend(config->bus.spi.drv, 5, config->buffers->spibuf);
  }
  else {
    config->buffers->spibuf[1] = (uint8_t)(offset >> 16);
    config->buffers->spibuf[2] = (uint8_t)(offset >> 8);
    config->buffers->spibuf[3] = (uint8_t)(offset >> 0);
    spiSend(config->bus.spi.drv, 4, config->buffers->spibuf);
  }
}
#endif /* XSNOR_USE_SPI == TRUE */

#if (XSNOR_SHARED_BUS == TRUE) || defined (__DOXYGEN__)
/**
 * @brief       Bus acquisition and lock.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 */
void __xsnor_bus_acquire(void *ip) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;

#if XSNOR_USE_BOTH == TRUE
  if (self->config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspiAcquireBus(self->config->bus.wspi.drv);
    if (self->config->bus.wspi.cfg != self->config->bus.wspi.drv->config) {
      wspiStart(self->config->bus.wspi.drv, self->config->bus.wspi.cfg);
    }
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE
    spiAcquireBus(self->config->bus.spi.drv);
    if (self->config->bus.spi.cfg != self->config->bus.spi.drv->config) {
      spiStart(self->config->bus.spi.drv, self->config->bus.spi.cfg);
    }
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}

/**
 * @brief       Bus release and unlock.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 */
void __xsnor_bus_release(void *ip) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;

#if XSNOR_USE_BOTH == TRUE
  if (self->config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspiReleaseBus(self->config->bus.wspi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE
    spiReleaseBus(self->config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}
#endif /* XSNOR_SHARED_BUS == TRUE */

/**
 * @brief       Sends a naked command.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     cmd           Instruction code.
 */
void __xsnor_bus_cmd(void *ip, uint32_t cmd) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  const xsnor_config_t *config = self->config;

#if XSNOR_USE_BOTH == TRUE
  if (config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspi_command_t mode;

    mode.cmd   = cmd;
    mode.cfg   = self->commands->cmd;
    mode.addr  = 0U;
    mode.alt   = 0U;
    mode.dummy = 0U;
    wspiCommand(config->bus.wspi.drv, &mode);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE

    spiSelect(config->bus.spi.drv);
    config->buffers->spibuf[0] = cmd;
    spiSend(config->bus.spi.drv, 1, config->buffers->spibuf);
    spiUnselect(config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}

/**
 * @brief       Sends a command followed by a data transmit phase.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     cmd           Instruction code.
 * @param[in]     n             Number of bytes to transmit.
 * @param[in]     p             Data buffer.
 */
void __xsnor_bus_cmd_send(void *ip, uint32_t cmd, size_t n, const uint8_t *p) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  const xsnor_config_t *config = self->config;

#if XSNOR_USE_BOTH == TRUE
  if (config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspi_command_t mode;

    mode.cmd   = cmd;
    mode.cfg   = self->commands->cmd_data;
    mode.addr  = 0U;
    mode.alt   = 0U;
    mode.dummy = 0U;
    wspiSend(config->bus.wspi.drv, &mode, n, p);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE

    spiSelect(config->bus.spi.drv);
    config->buffers->spibuf[0] = cmd;
    spiSend(config->bus.spi.drv, 1, config->buffers->spibuf);
    spiSend(config->bus.spi.drv, n, p);
    spiUnselect(config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}

/**
 * @brief       Sends a command followed by a data receive phase.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     cmd           Instruction code.
 * @param[in]     n             Number of bytes to receive.
 * @param[out]    p             Data buffer.
 */
void __xsnor_bus_cmd_receive(void *ip, uint32_t cmd, size_t n, uint8_t *p) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  const xsnor_config_t *config = self->config;

#if XSNOR_USE_BOTH == TRUE
  if (config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspi_command_t mode;

    mode.cmd   = cmd;
    mode.cfg   = self->commands->cmd_data;
    mode.addr  = 0U;
    mode.alt   = 0U;
    mode.dummy = 0U;
    wspiReceive(config->bus.wspi.drv, &mode, n, p);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE

    spiSelect(config->bus.spi.drv);
    config->buffers->spibuf[0] = cmd;
    spiSend(config->bus.spi.drv, 1, config->buffers->spibuf);
    spiReceive(config->bus.spi.drv, n, p);
    spiUnselect(config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}

/**
 * @brief       Sends a command followed by a flash address.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     cmd           Instruction code.
 * @param[in]     offset        Flash offset.
 */
void __xsnor_bus_cmd_addr(void *ip, uint32_t cmd, flash_offset_t offset) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  const xsnor_config_t *config = self->config;

#if XSNOR_USE_BOTH == TRUE
  if (config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspi_command_t mode;

    mode.cmd   = cmd;
    mode.cfg   = self->commands->cmd_addr;
    mode.addr  = offset;
    mode.alt   = 0U;
    mode.dummy = 0U;
    wspiCommand(config->bus.wspi.drv, &mode);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE

    spiSelect(config->bus.spi.drv);
    __xsnor_spi_cmd_addr(self, cmd, offset);
    spiUnselect(config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}

/**
 * @brief       Sends a command followed by a flash address and a data transmit
 *              phase.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     cmd           Instruction code.
 * @param[in]     offset        Flash offset.
 * @param[in]     n             Number of bytes to transmit.
 * @param[in]     p             Data buffer.
 */
void __xsnor_bus_cmd_addr_send(void *ip, uint32_t cmd, flash_offset_t offset,
                               size_t n, const uint8_t *p) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  const xsnor_config_t *config = self->config;

#if XSNOR_USE_BOTH == TRUE
  if (config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspi_command_t mode;

    mode.cmd   = cmd;
    mode.cfg   = self->commands->cmd_addr_data;
    mode.addr  = offset;
    mode.alt   = 0U;
    mode.dummy = 0U;
    wspiSend(config->bus.wspi.drv, &mode, n, p);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE

    spiSelect(config->bus.spi.drv);
    __xsnor_spi_cmd_addr(self, cmd, offset);
    spiSend(config->bus.spi.drv, n, p);
    spiUnselect(config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}

/**
 * @brief       Sends a command followed by a flash address and a data receive
 *              phase.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     cmd           Instruction code.
 * @param[in]     offset        Flash offset.
 * @param[in]     n             Number of bytes to receive.
 * @param[in]     p             Data buffer.
 */
void __xsnor_bus_cmd_addr_receive(void *ip, uint32_t cmd,
                                  flash_offset_t offset, size_t n, uint8_t *p) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  const xsnor_config_t *config = self->config;

#if XSNOR_USE_BOTH == TRUE
  if (config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspi_command_t mode;

    mode.cmd   = cmd;
    mode.cfg   = self->commands->cmd_addr_data;
    mode.addr  = offset;
    mode.alt   = 0U;
    mode.dummy = 0U;
    wspiReceive(config->bus.wspi.drv, &mode, n, p);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE

    spiSelect(config->bus.spi.drv);
    __xsnor_spi_cmd_addr(self, cmd, offset);
    spiReceive(config->bus.spi.drv, n, p);
    spiUnselect(config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}

/**
 * @brief       Sends a command followed by dummy cycles and a data receive
 *              phase.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     cmd           Instruction code.
 * @param[in]     dummy         Number of dummy cycles.
 * @param[in]     n             Number of bytes to receive.
 * @param[out]    p             Data buffer.
 */
void __xsnor_bus_cmd_dummy_receive(void *ip, uint32_t cmd, uint32_t dummy,
                                   size_t n, uint8_t *p) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  const xsnor_config_t *config = self->config;

#if XSNOR_USE_BOTH == TRUE
  if (config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspi_command_t mode;

    mode.cmd   = cmd;
    mode.cfg   = self->commands->cmd_data;
    mode.addr  = 0U;
    mode.alt   = 0U;
    mode.dummy = dummy;
    wspiReceive(config->bus.wspi.drv, &mode, n, p);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE
    osalDbgAssert((dummy & 7) == 0U, "multiple of 8 dummy cycles");

    spiSelect(config->bus.spi.drv);
    config->buffers->spibuf[0] = cmd;
    spiSend(config->bus.spi.drv, 1, config->buffers->spibuf);
    if (dummy != 0U) {
      spiIgnore(config->bus.spi.drv, dummy / 8U);
    }
    spiReceive(config->bus.spi.drv, n, p);
    spiUnselect(config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}

/**
 * @brief       Sends a complete header followed by a data receive phase.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     cmd           Instruction code.
 * @param[in]     offset        Flash offset.
 * @param[in]     dummy         Number of dummy cycles.
 * @param[in]     n             Number of bytes to receive.
 * @param[out]    p             Data buffer.
 */
void __xsnor_bus_cmd_addr_dummy_receive(void *ip, uint32_t cmd,
                                        flash_offset_t offset, uint32_t dummy,
                                        size_t n, uint8_t *p) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  const xsnor_config_t *config = self->config;

#if XSNOR_USE_BOTH == TRUE
  if (config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
    wspi_command_t mode;

    mode.cmd   = cmd;
    mode.cfg   = self->commands->cmd_addr_data;
    mode.addr  = offset;
    mode.alt   = 0U;
    mode.dummy = dummy;
    wspiReceive(config->bus.wspi.drv, &mode, n, p);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
  else {
#endif
#if XSNOR_USE_SPI == TRUE
    osalDbgAssert((dummy & 7) == 0U, "multiple of 8 dummy cycles");

    spiSelect(config->bus.spi.drv);
    __xsnor_spi_cmd_addr(self, cmd, offset);
    if (dummy != 0U) {
      spiIgnore(config->bus.spi.drv, dummy / 8U);
    }
    spiReceive(config->bus.spi.drv, n, p);
    spiUnselect(config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
  }
#endif
}

/**
 * @brief       Configures and activates a SNOR driver.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[in]     config        Pointer to the configuration.
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       Operation successful.
 * @retval FLASH_ERROR_HW_FAILURE If initialization failed.
 *
 * @api
 */
flash_error_t xsnorStart(void *ip, const xsnor_config_t *config) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  flash_error_t err = FLASH_NO_ERROR;

  osalDbgCheck((self != NULL) && (config != NULL));
  osalDbgAssert(self->state != FLASH_UNINIT, "invalid state");

  self->config = config;

  if (self->state == FLASH_STOP) {

    /* Bus acquisition.*/
    __xsnor_bus_acquire(self);

#if XSNOR_USE_BOTH == TRUE
    if (self->config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
      wspiStart(self->config->bus.wspi.drv, self->config->bus.wspi.cfg);
#endif
#if XSNOR_USE_BOTH == TRUE
    }
  else {
#endif
#if XSNOR_USE_SPI == TRUE
      spiStart(self->config->bus.spi.drv, self->config->bus.spi.cfg);
#endif
#if XSNOR_USE_BOTH == TRUE
    }
#endif

    /* Device identification and initialization.*/
    err = xsnor_device_init(self);
    if (err == FLASH_NO_ERROR) {
      /* Driver in ready state.*/
      self->state = FLASH_READY;
    }

    /* Bus release.*/
    __xsnor_bus_release(self);
  }

  return err;
}

/**
 * @brief       Deactivates a SNOR driver.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 *
 * @api
 */
void xsnorStop(void *ip) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  osalDbgCheck(self != NULL);
  osalDbgAssert(self->state != FLASH_UNINIT, "invalid state");

  if (self->state != FLASH_STOP) {

    /* Stopping bus device.*/
#if XSNOR_USE_BOTH == TRUE
    if (self->config->bus_type != XSNOR_BUS_MODE_SPI) {
#endif
#if XSNOR_USE_WSPI == TRUE
      wspiStop(self->config->bus.wspi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
    }
  else {
#endif
#if XSNOR_USE_SPI == TRUE
      spiStop(self->config->bus.spi.drv);
#endif
#if XSNOR_USE_BOTH == TRUE
    }
#endif

    /* Driver stopped.*/
    self->state = FLASH_STOP;

    /* Deleting current configuration.*/
    self->config = NULL;
  }
}

#if (WSPI_SUPPORTS_MEMMAP == TRUE) || defined (__DOXYGEN__)
/**
 * @brief       Enters the memory mapped mode.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 * @param[out]    addrp         Pointer to the memory mapped memory or @p NULL
 * @return                      An error code.
 * @retval FLASH_NO_ERROR       Operation successful.
 * @retval FLASH_ERROR_HW_FAILURE If memory mapped mode failed.
 *
 * @api
 */
flash_error_t xsnorMemoryMap(void *ip, uint8_t **addrp) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;
  flash_error_t err;

  /* Activating XIP mode in the device.*/
  err = xsnor_device_mmap_on(self, addrp);

  return err;
}

/**
 * @brief       Leaves the memory mapped mode.
 *
 * @param[in,out] ip            Pointer to a @p hal_xsnor_base_c instance.
 *
 * @api
 */
void xsnorMemoryUnmap(void *ip) {
  hal_xsnor_base_c *self = (hal_xsnor_base_c *)ip;

  xsnor_device_mmap_off(self);
}
#endif /* WSPI_SUPPORTS_MEMMAP == TRUE */
/** @} */

/** @} */
