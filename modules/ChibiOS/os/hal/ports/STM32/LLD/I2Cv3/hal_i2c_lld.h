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
 * @file    I2Cv3/hal_i2c_lld.h
 * @brief   STM32 I2C subsystem low level driver header.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef HAL_I2C_LLD_H
#define HAL_I2C_LLD_H

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   This I2C LLD supports slave mode.
 */
#define I2C_SUPPORTS_SLAVE_MODE             TRUE

/**
 * @name    TIMINGR register definitions
 * @{
 */
#define STM32_TIMINGR_PRESC_MASK        (15U << 28)
#define STM32_TIMINGR_PRESC(n)          ((n) << 28)
#define STM32_TIMINGR_SCLDEL_MASK       (15U << 20)
#define STM32_TIMINGR_SCLDEL(n)         ((n) << 20)
#define STM32_TIMINGR_SDADEL_MASK       (15U << 16)
#define STM32_TIMINGR_SDADEL(n)         ((n) << 16)
#define STM32_TIMINGR_SCLH_MASK         (255U << 8)
#define STM32_TIMINGR_SCLH(n)           ((n) << 8)
#define STM32_TIMINGR_SCLL_MASK         (255U << 0)
#define STM32_TIMINGR_SCLL(n)           ((n) << 0)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   I2C1 driver enable switch.
 * @details If set to @p TRUE the support for I2C1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_I2C_USE_I2C1) || defined(__DOXYGEN__)
#define STM32_I2C_USE_I2C1                  FALSE
#endif

/**
 * @brief   I2C2 driver enable switch.
 * @details If set to @p TRUE the support for I2C2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_I2C_USE_I2C2) || defined(__DOXYGEN__)
#define STM32_I2C_USE_I2C2                  FALSE
#endif

/**
 * @brief   I2C3 driver enable switch.
 * @details If set to @p TRUE the support for I2C3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_I2C_USE_I2C3) || defined(__DOXYGEN__)
#define STM32_I2C_USE_I2C3                  FALSE
#endif

/**
 * @brief   I2C4 driver enable switch.
 * @details If set to @p TRUE the support for I2C4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_I2C_USE_I2C4) || defined(__DOXYGEN__)
#define STM32_I2C_USE_I2C4                  FALSE
#endif

/**
 * @brief   I2C5 driver enable switch.
 * @details If set to @p TRUE the support for I2C5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_I2C_USE_I2C5) || defined(__DOXYGEN__)
#define STM32_I2C_USE_I2C5                  FALSE
#endif

/**
 * @brief   I2C timeout on busy condition in milliseconds.
 */
#if !defined(STM32_I2C_BUSY_TIMEOUT) || defined(__DOXYGEN__)
#define STM32_I2C_BUSY_TIMEOUT              50
#endif

/**
 * @brief   I2C1 interrupt priority level setting.
 */
#if !defined(STM32_I2C_I2C1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C1_IRQ_PRIORITY         10
#endif

/**
 * @brief   I2C2 interrupt priority level setting.
 */
#if !defined(STM32_I2C_I2C2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C2_IRQ_PRIORITY         10
#endif

/**
 * @brief   I2C3 interrupt priority level setting.
 */
#if !defined(STM32_I2C_I2C3_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C3_IRQ_PRIORITY         10
#endif

/**
 * @brief   I2C4 interrupt priority level setting.
 */
#if !defined(STM32_I2C_I2C4_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C4_IRQ_PRIORITY         10
#endif

/**
 * @brief   I2C5 interrupt priority level setting.
 */
#if !defined(STM32_I2C_I2C5_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C5_IRQ_PRIORITY         10
#endif

/**
 * @brief   DMA use switch.
 */
#if !defined(STM32_I2C_USE_DMA) || defined(__DOXYGEN__)
#define STM32_I2C_USE_DMA                   TRUE
#endif

/**
 * @brief   I2C1 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_I2C_I2C1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C1_DMA_PRIORITY         1
#endif

/**
 * @brief   I2C2 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_I2C_I2C2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C2_DMA_PRIORITY         1
#endif

/**
 * @brief   I2C3 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_I2C_I2C3_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C3_DMA_PRIORITY         1
#endif

/**
 * @brief   I2C4 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_I2C_I2C4_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C4_DMA_PRIORITY         1
#endif

/**
 * @brief   I2C5 DMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX DMA streams but
 *          because of the streams ordering the RX stream has always priority
 *          over the TX stream.
 */
#if !defined(STM32_I2C_I2C5_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_I2C_I2C5_DMA_PRIORITY         1
#endif

/**
 * @brief   I2C DMA error hook.
 * @note    The default action for DMA errors is a system halt because DMA
 *          error can only happen because programming errors.
 */
#if !defined(STM32_I2C_DMA_ERROR_HOOK) || defined(__DOXYGEN__)
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      osalSysHalt("DMA failure")
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks.*/
#if !defined(STM32_HAS_I2C1)
#error "STM32_HAS_I2C1 not defined in registry"
#endif

#if !defined(STM32_HAS_I2C2)
#error "STM32_HAS_I2C2 not defined in registry"
#endif

#if !defined(STM32_HAS_I2C3)
#error "STM32_HAS_I2C3 not defined in registry"
#endif

#if !defined(STM32_HAS_I2C4)
#error "STM32_HAS_I2C4 not defined in registry"
#endif

#if !defined(STM32_I2C4_USE_BDMA)
#error "STM32_I2C4_USE_BDMA not defined in registry"
#endif

#if !defined(STM32_HAS_I2C5)
#error "STM32_HAS_I2C5 not defined in registry"
#endif

/** @brief  error checks */
#if STM32_I2C_USE_I2C1 && !STM32_HAS_I2C1
#error "I2C1 not present in the selected device"
#endif

#if STM32_I2C_USE_I2C2 && !STM32_HAS_I2C2
#error "I2C2 not present in the selected device"
#endif

#if STM32_I2C_USE_I2C3 && !STM32_HAS_I2C3
#error "I2C3 not present in the selected device"
#endif

#if STM32_I2C_USE_I2C4 && !STM32_HAS_I2C4
#error "I2C4 not present in the selected device"
#endif

#if STM32_I2C_USE_I2C5 && !STM32_HAS_I2C5
#error "I2C5 not present in the selected device"
#endif

#if !STM32_I2C_USE_I2C1 && !STM32_I2C_USE_I2C2 && !STM32_I2C_USE_I2C3 &&    \
    !STM32_I2C_USE_I2C4 && !STM32_I2C_USE_I2C5
#error "I2C driver activated but no I2C peripheral assigned"
#endif

#if STM32_I2C_USE_I2C1 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_I2C_I2C1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to I2C1"
#endif

#if STM32_I2C_USE_I2C2 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_I2C_I2C2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to I2C2"
#endif

#if STM32_I2C_USE_I2C3 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_I2C_I2C3_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to I2C3"
#endif

#if STM32_I2C_USE_I2C4 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_I2C_I2C4_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to I2C4"
#endif

#if STM32_I2C_USE_I2C5 &&                                                   \
    !OSAL_IRQ_IS_VALID_PRIORITY(STM32_I2C_I2C5_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to I2C5"
#endif

#if STM32_I2C_USE_DMA == TRUE

#if STM32_I2C_USE_I2C1
#if !defined(STM32_I2C_I2C1_RX_DMA_STREAM)
#error "STM32_I2C_I2C1_RX_DMA_STREAM not defined"
#endif

#if !defined(STM32_I2C_I2C1_TX_DMA_STREAM)
#error "STM32_I2C_I2C1_TX_DMA_STREAM not defined"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C1_RX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C1 RX"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C1_TX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C1 TX"
#endif

#if !STM32_DMA_IS_VALID_PRIORITY(STM32_I2C_I2C1_DMA_PRIORITY)
#error "Invalid DMA priority assigned to I2C1"
#endif
#endif

#if STM32_I2C_USE_I2C2
#if !defined(STM32_I2C_I2C2_RX_DMA_STREAM)
#error "STM32_I2C_I2C2_RX_DMA_STREAM not defined"
#endif

#if !defined(STM32_I2C_I2C2_TX_DMA_STREAM)
#error "STM32_I2C_I2C2_TX_DMA_STREAM not defined"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C2_RX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C2 RX"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C2_TX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C2 TX"
#endif

#if !STM32_DMA_IS_VALID_PRIORITY(STM32_I2C_I2C2_DMA_PRIORITY)
#error "Invalid DMA priority assigned to I2C2"
#endif
#endif

#if STM32_I2C_USE_I2C3
#if !defined(STM32_I2C_I2C3_RX_DMA_STREAM)
#error "STM32_I2C_I2C3_RX_DMA_STREAM not defined"
#endif

#if !defined(STM32_I2C_I2C3_TX_DMA_STREAM)
#error "STM32_I2C_I2C3_TX_DMA_STREAM not defined"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C3_RX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C3 RX"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C3_TX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C3 TX"
#endif

#if !STM32_DMA_IS_VALID_PRIORITY(STM32_I2C_I2C3_DMA_PRIORITY)
#error "Invalid DMA priority assigned to I2C3"
#endif
#endif

#if STM32_I2C_USE_I2C4
#if STM32_I2C4_USE_BDMA

#if !defined(STM32_I2C_I2C4_RX_BDMA_STREAM)
#error "STM32_I2C_I2C4_RX_BDMA_STREAM not defined"
#endif

#if !defined(STM32_I2C_I2C4_TX_BDMA_STREAM)
#error "STM32_I2C_I2C4_TX_BDMA_STREAM not defined"
#endif

#if !STM32_BDMA_IS_VALID_STREAM(STM32_I2C_I2C4_RX_BDMA_STREAM)
#error "Invalid BDMA stream assigned to I2C4 RX"
#endif

#if !STM32_BDMA_IS_VALID_STREAM(STM32_I2C_I2C4_TX_BDMA_STREAM)
#error "Invalid BDMA stream assigned to I2C4 TX"
#endif

#if !STM32_BDMA_IS_VALID_PRIORITY(STM32_I2C_I2C4_DMA_PRIORITY)
#error "Invalid DMA priority assigned to I2C4"
#endif

#else /* !STM32_I2C4_USE_BDMA */

#if !defined(STM32_I2C_I2C4_RX_DMA_STREAM)
#error "STM32_I2C_I2C4_RX_DMA_STREAM not defined"
#endif

#if !defined(STM32_I2C_I2C4_TX_DMA_STREAM)
#error "STM32_I2C_I2C4_TX_DMA_STREAM not defined"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C4_RX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C4 RX"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C4_TX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C4 TX"
#endif

#if !STM32_DMA_IS_VALID_PRIORITY(STM32_I2C_I2C4_DMA_PRIORITY)
#error "Invalid DMA priority assigned to I2C4"
#endif

#endif /* !STM32_I2C4_USE_BDMA */
#endif /* STM32_I2C_USE_I2C4 */

#if STM32_I2C4_USE_BDMA == TRUE

#if STM32_I2C_USE_I2C1 || STM32_I2C_USE_I2C2 || STM32_I2C_USE_I2C3 || STM32_I2C_USE_I2C5
#define STM32_I2C_DMA_REQUIRED
#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif
#endif

#if STM32_I2C_USE_I2C4
#define STM32_I2C_BDMA_REQUIRED
#if !defined(STM32_BDMA_REQUIRED)
#define STM32_BDMA_REQUIRED
#endif
#if !defined(STM32_I2C_BDMA_REQUIRED)
#define STM32_I2C_BDMA_REQUIRED
#endif
#endif
#else /* STM32_I2C4_USE_BDMA != TRUE */

#if STM32_I2C_USE_I2C1 || STM32_I2C_USE_I2C2 || STM32_I2C_USE_I2C3 || STM32_I2C_USE_I2C4 || STM32_I2C_USE_I2C5
#define STM32_I2C_DMA_REQUIRED
#if !defined(STM32_DMA_REQUIRED)
#define STM32_DMA_REQUIRED
#endif
#endif

#endif /* STM32_I2C4_USE_BDMA != TRUE */

#if STM32_I2C_USE_I2C5
#if !defined(STM32_I2C_I2C5_RX_DMA_STREAM)
#error "STM32_I2C_I2C5_RX_DMA_STREAM not defined"
#endif

#if !defined(STM32_I2C_I2C5_TX_DMA_STREAM)
#error "STM32_I2C_I2C5_TX_DMA_STREAM not defined"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C5_RX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C5 RX"
#endif

#if !STM32_DMA_IS_VALID_STREAM(STM32_I2C_I2C5_TX_DMA_STREAM)
#error "Invalid DMA stream assigned to I2C5 TX"
#endif

#if !STM32_DMA_IS_VALID_PRIORITY(STM32_I2C_I2C5_DMA_PRIORITY)
#error "Invalid DMA priority assigned to I2C5"
#endif
#endif

#endif /* STM32_I2C_USE_DMA == TRUE */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type representing an I2C address.
 */
typedef uint16_t i2caddr_t;

/**
 * @brief   Type of I2C driver condition flags.
 */
typedef uint32_t i2cflags_t;

/**
 * @brief   I2C driver configuration structure.
 */
struct hal_i2c_config {
  /**
   * @brief   TIMINGR register initialization.
   * @note    Refer to the STM32 reference manual, the values are affected
   *          by the system clock settings in mcuconf.h.
   */
  uint32_t        timingr;
  /**
   * @brief   CR1 register initialization.
   * @note    Leave to zero unless you know what you are doing.
   */
  uint32_t        cr1;
  /**
   * @brief   CR2 register initialization.
   * @note    Only the ADD10 bit can eventually be specified here.
   */
  uint32_t        cr2;
};

/**
 * @brief   Type of a structure representing an I2C configuration.
 */
typedef struct hal_i2c_config I2CConfig;

/**
 * @brief   Type of a structure representing an I2C driver.
 */
typedef struct hal_i2c_driver I2CDriver;

/**
 * @brief   Structure representing an I2C driver.
 */
struct hal_i2c_driver {
  /**
   * @brief   Driver state.
   */
  i2cstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const I2CConfig           *config;
  /**
   * @brief   Error flags.
   */
  i2cflags_t                errors;
#if I2C_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
  mutex_t                   mutex;
#endif /* I2C_USE_MUTUAL_EXCLUSION */
#if defined(I2C_DRIVER_EXT_FIELDS)
  I2C_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   Thread waiting for I/O completion.
   */
  thread_reference_t        thread;
  /**
   * @brief     Number of bytes in TX phase.
   */
  size_t                    txbytes;
  /**
   * @brief     Number of bytes in RX phase.
   */
  size_t                    rxbytes;
#if (STM32_I2C_USE_DMA == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief RX DMA mode bit mask.
   */
  uint32_t                  rxdmamode;
  /**
   * @brief TX DMA mode bit mask.
   */
  uint32_t                  txdmamode;
#if defined(STM32_I2C_DMA_REQUIRED) && defined(STM32_BDMA_REQUIRED)
  /**
   * @brief   DMA type for this instance.
   */
  bool                      is_bdma;
#endif
  /**
   * @brief   Union of the RX DMA streams.
   */
  union {
#if defined(STM32_I2C_DMA_REQUIRED) || defined(__DOXYGEN__)
    /**
     * @brief   Receive DMA stream.
     */
    const stm32_dma_stream_t  *dma;
#endif
#if (STM32_I2C4_USE_BDMA == TRUE) || defined(__DOXYGEN__)
#if defined(STM32_BDMA_REQUIRED) || defined(__DOXYGEN__)
    /**
     * @brief   Receive BDMA stream.
     */
    const stm32_bdma_stream_t  *bdma;
#endif
#endif
  } rx;
  /**
   * @brief   Union of the TX DMA streams.
   */
  union {
#if defined(STM32_I2C_DMA_REQUIRED) || defined(__DOXYGEN__)
    /**
     * @brief   Transmit DMA stream.
     */
    const stm32_dma_stream_t  *dma;
#endif
#if (STM32_I2C4_USE_BDMA == TRUE) || defined(__DOXYGEN__)
#if defined(STM32_BDMA_REQUIRED) || defined(__DOXYGEN__)
    /**
     * @brief   Transmit DMA stream.
     */
    const stm32_bdma_stream_t  *bdma;
#endif
#endif
  } tx;
#else /* STM32_I2C_USE_DMA == FALSE */
  /**
   * @brief     Pointer to the next TX buffer location.
   */
  const uint8_t             *txptr;
  /**
   * @brief     Pointer to the next RX buffer location.
   */
  uint8_t                   *rxptr;
#endif /* STM32_I2C_USE_DMA == FALSE */
  /**
   * @brief     Pointer to the I2Cx registers block.
   */
  I2C_TypeDef               *i2c;

#if (I2C_ENABLE_SLAVE_MODE == TRUE)
  /**
   * @brief     Master needed a reply.
   */
  bool                      reply_required;
  /**
   * @brief     Master/Slave mode.
   */
  bool                      isMaster;
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
#ifdef STM32_I2C_ISR_LIMIT
  /**
   * @brief count of interrupts since transfer start
   */
  uint32_t                  isr_count;

  /**
   * @brief limit of interrupts for this transfer
   */
  uint32_t                  isr_limit;
#endif

  /**
     true when we are in an I2C transaction
   */
  bool                      in_transaction;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Get errors from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_errors(i2cp) ((i2cp)->errors)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
#if STM32_I2C_USE_I2C1
extern I2CDriver I2CD1;
#endif

#if STM32_I2C_USE_I2C2
extern I2CDriver I2CD2;
#endif

#if STM32_I2C_USE_I2C3
extern I2CDriver I2CD3;
#endif

#if STM32_I2C_USE_I2C4
extern I2CDriver I2CD4;
#endif

#if STM32_I2C_USE_I2C5
extern I2CDriver I2CD5;
#endif

#endif /* !defined(__DOXYGEN__) */

#ifdef __cplusplus
extern "C" {
#endif
  void i2c_lld_init(void);
  void i2c_lld_start(I2CDriver *i2cp);
  void i2c_lld_stop(I2CDriver *i2cp);
  void i2c_lld_soft_stop(I2CDriver *i2cp);
  msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                        const uint8_t *txbuf, size_t txbytes,
                                        uint8_t *rxbuf, size_t rxbytes,
                                        sysinterval_t timeout);
  msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                       uint8_t *rxbuf, size_t rxbytes,
                                       sysinterval_t timeout);
#if (I2C_ENABLE_SLAVE_MODE == TRUE)
  msg_t i2c_lld_match_address(I2CDriver *i2cp, i2caddr_t addr);
  msg_t i2c_lld_slave_receive_timeout(I2CDriver *i2cp, uint8_t *rxbuf,
                                      size_t rxbytes, sysinterval_t timeout);
  msg_t i2c_lld_slave_transmit_timeout(I2CDriver *i2cp, const uint8_t *txbuf,
                                       size_t txbytes, sysinterval_t timeout);
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C  */

#endif /* HAL_I2C_LLD_H */

/** @} */
