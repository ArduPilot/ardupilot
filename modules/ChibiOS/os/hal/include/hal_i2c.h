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
 * @file    hal_i2c.h
 * @brief   I2C Driver macros and structures.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H

#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* TODO: To be reviewed, too STM32-centric.*/
/**
 * @name    I2C bus error conditions
 * @{
 */
#define I2C_NO_ERROR               0x00    /**< @brief No error.            */
#define I2C_BUS_ERROR              0x01    /**< @brief Bus Error.           */
#define I2C_ARBITRATION_LOST       0x02    /**< @brief Arbitration Lost.    */
#define I2C_ACK_FAILURE            0x04    /**< @brief Acknowledge Failure. */
#define I2C_OVERRUN                0x08    /**< @brief Overrun/Underrun.    */
#define I2C_PEC_ERROR              0x10    /**< @brief PEC Error in
                                                reception.                  */
#define I2C_TIMEOUT                0x20    /**< @brief Hardware timeout.    */
#define I2C_SMB_ALERT              0x40    /**< @brief SMBus Alert.         */
#define I2C_ISR_LIMIT              0x80    /**< @brief exceeded maximum ISR limit */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Enables the mutual exclusion APIs on the I2C bus.
 */
#if !defined(I2C_USE_MUTUAL_EXCLUSION) || defined(__DOXYGEN__)
#define I2C_USE_MUTUAL_EXCLUSION            TRUE
#endif

/**
 * @brief   Slave mode API enable switch.
 * @note    The low level driver must support this capability.
 */
#if !defined(I2C_ENABLE_SLAVE_MODE)
#define I2C_ENABLE_SLAVE_MODE               FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  I2C_UNINIT = 0,                           /**< @brief Not initialized.    */
  I2C_STOP = 1,                             /**< @brief Stopped.            */
  I2C_READY = 2,                            /**< @brief Ready.              */
  I2C_ACTIVE_TX = 3,                        /**< @brief Transmitting.       */
  I2C_ACTIVE_RX = 4,                        /**< @brief Receiving.          */
  I2C_LOCKED = 5                            /**< @brief Bus locked.         */
} i2cstate_t;

#include "hal_i2c_lld.h"

/* For compatibility, some LLDs could not export this.*/
#if !defined(I2C_SUPPORTS_SLAVE_MODE)
#define I2C_SUPPORTS_SLAVE_MODE             FALSE
#endif

#if (I2C_SUPPORTS_SLAVE_MODE == FALSE) && (I2C_ENABLE_SLAVE_MODE == TRUE)
#error "I2C slave mode not supported"
#endif

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Wakes up the waiting thread notifying no errors.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define _i2c_wakeup_isr(i2cp) do {                                          \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(i2cp)->thread, MSG_OK);                               \
  osalSysUnlockFromISR();                                                   \
} while (0)

/**
 * @brief   Wakes up the waiting thread notifying errors.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define _i2c_wakeup_error_isr(i2cp) do {                                    \
  osalSysLockFromISR();                                                     \
  osalThreadResumeI(&(i2cp)->thread, MSG_RESET);                            \
  osalSysUnlockFromISR();                                                   \
} while (0)

/**
 * @brief   Wrap i2cMasterTransmitTimeout function with TIME_INFINITE timeout.
 * @api
 */
#define i2cMasterTransmit(i2cp, addr, txbuf, txbytes, rxbuf, rxbytes)       \
  (i2cMasterTransmitTimeout(i2cp, addr, txbuf, txbytes, rxbuf, rxbytes,     \
                            TIME_INFINITE))

/**
 * @brief   Wrap i2cMasterReceiveTimeout function with TIME_INFINITE timeout.
 * @api
 */
#define i2cMasterReceive(i2cp, addr, rxbuf, rxbytes)                        \
  (i2cMasterReceiveTimeout(i2cp, addr, rxbuf, rxbytes, TIME_INFINITE))

#if (I2C_ENABLE_SLAVE_MODE == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Answer required.
 * @note    This function is meant to be called after slave receive only.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @return              Slave answer required.
 * @retval              false if the slave must not answer.
 * @retval              true if the slave must answer.
 *
 * @special
 */
#define i2cSlaveIsAnswerRequired(i2cp) (((i2cp)->reply_required))
#endif
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void i2cInit(void);
  void i2cObjectInit(I2CDriver *i2cp);
  msg_t i2cStart(I2CDriver *i2cp, const I2CConfig *config);
  void i2cStop(I2CDriver *i2cp);
  void i2cSoftStop(I2CDriver *i2cp);
  i2cflags_t i2cGetErrors(I2CDriver *i2cp);
  msg_t i2cMasterTransmitTimeout(I2CDriver *i2cp,
                                 i2caddr_t addr,
                                 const uint8_t *txbuf, size_t txbytes,
                                 uint8_t *rxbuf, size_t rxbytes,
                                 sysinterval_t timeout);
  msg_t i2cMasterReceiveTimeout(I2CDriver *i2cp,
                                i2caddr_t addr,
                                uint8_t *rxbuf, size_t rxbytes,
                                sysinterval_t timeout);
#if I2C_USE_MUTUAL_EXCLUSION == TRUE
  void i2cAcquireBus(I2CDriver *i2cp);
  void i2cReleaseBus(I2CDriver *i2cp);
#endif
#if I2C_ENABLE_SLAVE_MODE == TRUE
  msg_t i2cSlaveMatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr);
  msg_t i2cSlaveReceiveTimeout(I2CDriver *i2cp, uint8_t *rxbuf,
                               size_t rxbytes, sysinterval_t timeout);
  msg_t i2cSlaveTransmitTimeout(I2CDriver *i2cp, const uint8_t *txbuf,
                                size_t txbytes, sysinterval_t timeout);
#endif
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C == TRUE */

#endif /* HAL_I2C_H */

/** @} */
