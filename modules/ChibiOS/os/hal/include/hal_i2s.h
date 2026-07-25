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
 * @file    hal_i2s.h
 * @brief   I2S Driver macros and structures.
 *
 * @addtogroup I2S
 * @{
 */

#ifndef HAL_I2S_H
#define HAL_I2S_H

#if (HAL_USE_I2S == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    I2S modes
 * @{
 */
#define I2S_MODE_SLAVE          0
#define I2S_MODE_MASTER         1
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

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
  I2S_UNINIT = 0,                   /**< Not initialized.                   */
  I2S_STOP = 1,                     /**< Stopped.                           */
  I2S_READY = 2,                    /**< Ready.                             */
  I2S_ACTIVE = 3,                   /**< Active.                            */
  I2S_COMPLETE = 4                  /**< Transmission complete.             */
} i2sstate_t;

/**
 * @brief   Type of a structure representing an I2S driver.
 */
typedef struct hal_i2s_driver I2SDriver;

/**
 * @brief   Type of a structure representing an I2S driver configuration.
 */
typedef struct hal_i2s_config I2SConfig;

/**
 * @brief   I2S notification callback type.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 */
typedef void (*i2scallback_t)(I2SDriver *i2sp);

/* Including the low level driver header, it exports information required
   for completing types.*/
#include "hal_i2s_lld.h"

/**
 * @brief   Structure representing an I2S driver.
 */
struct hal_i2s_driver {
  /**
   * @brief   Driver state.
   */
  i2sstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const I2SConfig           *config;
  /* End of the mandatory fields.*/
  i2s_lld_driver_fields
};

/**
 * @brief   Driver configuration structure.
 */
struct hal_i2s_config {
  /**
   * @brief   Transmission buffer pointer.
   * @note    Can be @p NULL if TX is not required.
   */
  const void                *tx_buffer;
  /**
   * @brief   Receive buffer pointer.
   * @note    Can be @p NULL if RX is not required.
   */
  void                      *rx_buffer;
  /**
   * @brief   TX and RX buffers size as number of samples.
   */
  size_t                    size;
  /**
   * @brief   Callback function called during streaming.
   */
  i2scallback_t             end_cb;
  /* End of the mandatory fields.*/
  i2s_lld_config_fields
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Buffer state.
 * @note    This function is meant to be called from the SPI callback only.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @return              The buffer state.
 * @retval              false if the driver filled/sent the first half of the
 *                      buffer.
 * @retval              true if the driver filled/sent the second half of the
 *                      buffer.
 *
 * @special
 */
#define i2sIsBufferComplete(i2sp) ((bool)((i2sp)->state == I2S_COMPLETE))

/**
 * @brief   Starts a I2S data exchange.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @iclass
 */
#define i2sStartExchangeI(i2sp) {                                           \
  i2s_lld_start_exchange(i2sp);                                             \
  (i2sp)->state = I2S_ACTIVE;                                               \
}

/**
 * @brief   Stops the ongoing data exchange.
 * @details The ongoing data exchange, if any, is stopped, if the driver
 *          was not active the function does nothing.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @iclass
 */
#define i2sStopExchangeI(i2sp) {                                            \
  i2s_lld_stop_exchange(i2sp);                                              \
  (i2sp)->state = I2S_READY;                                                \
}

/**
 * @brief   Common ISR code, half buffer event.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
#define _i2s_isr_half_code(i2sp) {                                          \
  if ((i2sp)->config->end_cb != NULL) {                                     \
    (i2sp)->config->end_cb(i2sp);                                           \
  }                                                                         \
}

/**
 * @brief   Common ISR code.
 * @details This code handles the portable part of the ISR code:
 *          - Callback invocation.
 *          - Driver state transitions.
 *          .
 * @note    This macro is meant to be used in the low level drivers
 *          implementation only.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
#define _i2s_isr_full_code(i2sp) {                                          \
  if ((i2sp)->config->end_cb) {                                             \
    (i2sp)->state = I2S_COMPLETE;                                           \
    (i2sp)->config->end_cb(i2sp);                                           \
    if ((i2sp)->state == I2S_COMPLETE) {                                    \
      (i2sp)->state = I2S_ACTIVE;                                           \
    }                                                                       \
  }                                                                         \
}
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void i2sInit(void);
  void i2sObjectInit(I2SDriver *i2sp);
  msg_t i2sStart(I2SDriver *i2sp, const I2SConfig *config);
  void i2sStop(I2SDriver *i2sp);
  void i2sStartExchange(I2SDriver *i2sp);
  void i2sStopExchange(I2SDriver *i2sp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2S == TRUE */

#endif /* HAL_I2S_H */

/** @} */
