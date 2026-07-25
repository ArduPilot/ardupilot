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
/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

/**
 * @file    hal_i2c.c
 * @brief   I2C Driver code.
 *
 * @addtogroup I2C
 * @{
 */
#include "hal.h"

#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   I2C Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void i2cInit(void) {

  i2c_lld_init();
}

/**
 * @brief   Initializes the standard part of a @p I2CDriver structure.
 *
 * @param[out] i2cp     pointer to the @p I2CDriver object
 *
 * @init
 */
void i2cObjectInit(I2CDriver *i2cp) {

  i2cp->state  = I2C_STOP;
  i2cp->config = NULL;

#if I2C_USE_MUTUAL_EXCLUSION == TRUE
  osalMutexObjectInit(&i2cp->mutex);
#endif

#if defined(I2C_DRIVER_EXT_INIT_HOOK)
  I2C_DRIVER_EXT_INIT_HOOK(i2cp);
#endif
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] config    pointer to the @p I2CConfig object
 * @return              The operation status.
 *
 * @api
 */
msg_t i2cStart(I2CDriver *i2cp, const I2CConfig *config) {
  msg_t msg;

  osalDbgCheck((i2cp != NULL) && (config != NULL));

  osalSysLock();
  osalDbgAssert((i2cp->state == I2C_STOP) || (i2cp->state == I2C_READY) ||
                (i2cp->state == I2C_LOCKED), "invalid state");

  i2cp->config = config;

#if defined(I2C_LLD_ENHANCED_API)
  msg = i2c_lld_start(i2cp);
  if (msg == HAL_RET_SUCCESS) {
    i2cp->state = I2C_READY;
  }
  else {
    i2cp->state = I2C_STOP;
  }
#else
  i2c_lld_start(i2cp);
  i2cp->state = I2C_READY;
  msg = HAL_RET_SUCCESS;
#endif

  osalSysUnlock();

  return msg;
}

/**
 * @brief   Like i2cStop, but doesn't stop the clock nor disable the peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cSoftStop(I2CDriver *i2cp) {

  osalDbgCheck(i2cp != NULL);

  osalSysLock();

  osalDbgAssert((i2cp->state == I2C_STOP) || (i2cp->state == I2C_READY) ||
                (i2cp->state == I2C_LOCKED), "invalid state");

  i2c_lld_soft_stop(i2cp);
  i2cp->config = NULL;
  i2cp->state  = I2C_STOP;

  osalSysUnlock();
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cStop(I2CDriver *i2cp) {

  osalDbgCheck(i2cp != NULL);

  osalSysLock();

  osalDbgAssert((i2cp->state == I2C_STOP) || (i2cp->state == I2C_READY) ||
                (i2cp->state == I2C_LOCKED), "invalid state");

  i2c_lld_stop(i2cp);
  i2cp->config = NULL;
  i2cp->state  = I2C_STOP;

  osalSysUnlock();
}

/**
 * @brief   Returns the errors mask associated to the previous operation.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @return              The errors mask.
 *
 * @api
 */
i2cflags_t i2cGetErrors(I2CDriver *i2cp) {

  osalDbgCheck(i2cp != NULL);

  return i2c_lld_get_errors(i2cp);
}

/**
 * @brief   Sends data via the I2C bus.
 * @details Function designed to realize "read-through-write" transfer
 *          paradigm. If you want transmit data without any further read,
 *          than set @b rxbytes field to 0.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address (7 bits) without R/W bit
 * @param[in] txbuf     pointer to transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to receive buffer
 * @param[in] rxbytes   number of bytes to be received, set it to 0 if
 *                      you want transmit only
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
msg_t i2cMasterTransmitTimeout(I2CDriver *i2cp,
                               i2caddr_t addr,
                               const uint8_t *txbuf,
                               size_t txbytes,
                               uint8_t *rxbuf,
                               size_t rxbytes,
                               sysinterval_t timeout) {
  msg_t rdymsg;

  osalDbgCheck((i2cp != NULL) &&
               (txbytes > 0U) && (txbuf != NULL) &&
               ((rxbytes == 0U) || ((rxbytes > 0U) && (rxbuf != NULL))) &&
               (timeout != TIME_IMMEDIATE));

  osalDbgAssert(i2cp->state == I2C_READY, "not ready");

  osalSysLock();
  i2cp->errors = I2C_NO_ERROR;
  i2cp->state = I2C_ACTIVE_TX;
  rdymsg = i2c_lld_master_transmit_timeout(i2cp, addr, txbuf, txbytes,
                                           rxbuf, rxbytes, timeout);
  if (rdymsg == MSG_TIMEOUT) {
    i2cp->state = I2C_LOCKED;
  }
  else {
    i2cp->state = I2C_READY;
  }
  osalSysUnlock();
  return rdymsg;
}

/**
 * @brief   Receives data from the I2C bus.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address (7 bits) without R/W bit
 * @param[out] rxbuf    pointer to receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 *
 * @api
 */
msg_t i2cMasterReceiveTimeout(I2CDriver *i2cp,
                              i2caddr_t addr,
                              uint8_t *rxbuf,
                              size_t rxbytes,
                              sysinterval_t timeout) {

  msg_t rdymsg;

  osalDbgCheck((i2cp != NULL) && (addr != 0U) &&
               (rxbytes > 0U) && (rxbuf != NULL) &&
               (timeout != TIME_IMMEDIATE));

  osalDbgAssert(i2cp->state == I2C_READY, "not ready");

  osalSysLock();
  i2cp->errors = I2C_NO_ERROR;
  i2cp->state = I2C_ACTIVE_RX;
  rdymsg = i2c_lld_master_receive_timeout(i2cp, addr, rxbuf, rxbytes, timeout);
  if (rdymsg == MSG_TIMEOUT) {
    i2cp->state = I2C_LOCKED;
  }
  else {
    i2cp->state = I2C_READY;
  }
  osalSysUnlock();
  return rdymsg;
}

#if (I2C_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the I2C bus.
 * @details This function tries to gain ownership to the I2C bus, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option @p I2C_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cAcquireBus(I2CDriver *i2cp) {

  osalDbgCheck(i2cp != NULL);

  osalMutexLock(&i2cp->mutex);
}

/**
 * @brief   Releases exclusive access to the I2C bus.
 * @pre     In order to use this function the option @p I2C_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @api
 */
void i2cReleaseBus(I2CDriver *i2cp) {

  osalDbgCheck(i2cp != NULL);

  osalMutexUnlock(&i2cp->mutex);
}
#endif /* I2C_USE_MUTUAL_EXCLUSION == TRUE */

#if (I2C_ENABLE_SLAVE_MODE == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Listen I2C bus for address match.
 * @details Use 7 bit address (10 bit,dual and general call address dosn't implement yet) .
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 *
 * @notapi
 */
msg_t i2cSlaveMatchAddress(I2CDriver *i2cp, i2caddr_t addr) {

  osalDbgCheck((i2cp != NULL) && (addr != 0x00));

  osalSysLock();

  msg_t result = i2c_lld_match_address(i2cp, addr);

  osalSysUnlock();

  return result;
}

/**
 * @brief   Receive data via the I2C bus as slave and call handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   size of receive buffer
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @api
 */
msg_t i2cSlaveReceiveTimeout(I2CDriver *i2cp,
                             uint8_t *rxbuf,
                             size_t rxbytes,
                             sysinterval_t timeout) {
  msg_t rdymsg;

  osalDbgCheck((i2cp != NULL) &&
               (rxbytes > 0U) &&
               (rxbuf != NULL) &&
               (timeout != TIME_IMMEDIATE));

  osalDbgAssert(i2cp->state == I2C_READY, "not ready");

  osalSysLock();

  i2cp->errors = I2C_NO_ERROR;
  i2cp->state = I2C_ACTIVE_RX;

  rdymsg = i2c_lld_slave_receive_timeout(i2cp, rxbuf, rxbytes, timeout);
  if (rdymsg == MSG_TIMEOUT) {
    i2cp->state = I2C_LOCKED;
  }
  else {
    i2cp->state = I2C_READY;
  }

  osalSysUnlock();

  return rdymsg;
}

/**
 * @brief   Transmits data via the I2C bus as slave.
 * @details Call this function when Master request data (in request handler)
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] txbuf     pointer to the transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @api
 */
msg_t i2cSlaveTransmitTimeout(I2CDriver *i2cp,
                               const uint8_t *txbuf,
                               size_t txbytes,
                               sysinterval_t timeout){
  msg_t rdymsg;

  osalDbgCheck((i2cp != NULL) &&
               (txbytes > 0U) &&
               (txbuf != NULL) &&
               (timeout != TIME_IMMEDIATE));

  osalDbgAssert(i2cp->state == I2C_READY, "not ready");

  osalSysLock();

  i2cp->errors = I2C_NO_ERROR;
  i2cp->state = I2C_ACTIVE_TX;
  rdymsg = i2c_lld_slave_transmit_timeout(i2cp, txbuf, txbytes, timeout);
  if (rdymsg == MSG_TIMEOUT) {
    i2cp->state = I2C_LOCKED;
  }
  else {
    i2cp->state = I2C_READY;
  }

  osalSysUnlock();

  return rdymsg;
}
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */

#endif /* HAL_USE_I2C == TRUE */

/** @} */
