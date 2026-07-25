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
 * @file    I2Cv1/hal_i2c_lld.c
 * @brief   STM32 I2C subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */

#include "hal.h"

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define I2C1_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C1_RX_DMA_STREAM,                        \
                       STM32_I2C1_RX_DMA_CHN)

#define I2C1_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C1_TX_DMA_STREAM,                        \
                       STM32_I2C1_TX_DMA_CHN)

#define I2C2_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C2_RX_DMA_STREAM,                        \
                       STM32_I2C2_RX_DMA_CHN)

#define I2C2_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C2_TX_DMA_STREAM,                        \
                       STM32_I2C2_TX_DMA_CHN)

#define I2C3_RX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C3_RX_DMA_STREAM,                        \
                       STM32_I2C3_RX_DMA_CHN)

#define I2C3_TX_DMA_CHANNEL                                                 \
  STM32_DMA_GETCHANNEL(STM32_I2C_I2C3_TX_DMA_STREAM,                        \
                       STM32_I2C3_TX_DMA_CHN)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#define I2C_EV5_MASTER_MODE_SELECT                                          \
  ((uint32_t)(((I2C_SR2_MSL | I2C_SR2_BUSY) << 16) | I2C_SR1_SB))

#define I2C_EV5_MASTER_MODE_SELECT_NO_BUSY                                  \
  ((uint32_t)((I2C_SR2_MSL << 16) | I2C_SR1_SB))

#define I2C_EV6_MASTER_TRA_MODE_SELECTED                                    \
  ((uint32_t)(((I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA) << 16) |          \
              I2C_SR1_ADDR | I2C_SR1_TXE))

#define I2C_EV6_MASTER_REC_MODE_SELECTED                                    \
  ((uint32_t)(((I2C_SR2_MSL | I2C_SR2_BUSY)<< 16) | I2C_SR1_ADDR))

#define I2C_EV8_2_MASTER_BYTE_TRANSMITTED                                   \
  ((uint32_t)(((I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA) << 16) |          \
              I2C_SR1_BTF | I2C_SR1_TXE))

#define I2C_EV9_MASTER_ADD10                                                \
  ((uint32_t)(((I2C_SR2_MSL | I2C_SR2_BUSY) << 16) | I2C_SR1_ADD10))

#define I2C_EV5_MASTER_MODE_INVALID                                         \
  ((uint32_t)(((I2C_SR2_MSL | I2C_SR2_BUSY) << 16) | (I2C_SR1_SB |          \
              I2C_SR1_STOPF)))

#define I2C_EV_MASK 0x00FF00FF

#define I2C_ERROR_MASK                                                      \
  ((uint16_t)(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR |      \
              I2C_SR1_PECERR | I2C_SR1_TIMEOUT | I2C_SR1_SMBALERT))

#if (I2C_ENABLE_SLAVE_MODE == TRUE)
#define  I2C_EV1_SLAVE_RECEIVER_ADDRESS_MATCHED                             \
  ((uint32_t)(I2C_SR2_BUSY << 16) | I2C_SR1_ADDR)

#define I2C_EV1_SLAVE_TRANSMITTER_ADDRESS_MATCHED                           \
  ((uint32_t)((I2C_SR2_TRA | I2C_SR2_BUSY) << 16) |                         \
              I2C_SR1_TXE | I2C_SR1_ADDR)

#define  I2C_EV1_SLAVE_RECEIVER_SECONDADDRESS_MATCHED                       \
  ((uint32_t)(I2C_SR2_DUALF | I2C_SR2_BUSY) << 16)

#define  I2C_EV1_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED                    \
  ((uint32_t)((I2C_SR2_DUALF | I2C_SR2_TRA | I2C_SR2_BUSY) << 16)           \
              | I2C_SR1_TXE)

#define  I2C_EV1_SLAVE_GENERALCALLADDRESS_MATCHED                           \
  ((uint32_t)(I2C_SR2_GENCALL | I2C_SR2_BUSY) << 16)

#define  I2C_EV2_SLAVE_BYTE_RECEIVED                                        \
  ((uint32_t)(I2C_SR2_BUSY << 16) | I2C_SR1_RXNE)

#define I2C_EV4_SLAVE_STOP_DETECTED  \
  ((uint32_t) I2C_SR1_STOPF)

#define  I2C_EV3_SLAVE_BYTE_TRANSMITTED                                     \
  ((uint32_t)((I2C_SR2_TRA | I2C_SR2_BUSY) << 16) | I2C_SR1_TXE             \
   | I2C_SR1_BTF)

#define  I2C_EV3_SLAVE_BYTE_TRANSMITTING                                    \
  ((uint32_t)((I2C_SR2_TRA | I2C_SR2_BUSY) << 16) | I2C_SR1_TXE)

#define  I2C_EV2_SLAVE_ACK_FAILURE                                          \
  ((uint32_t) I2C_SR1_AF)
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief I2C1 driver identifier.*/
#if STM32_I2C_USE_I2C1 || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/** @brief I2C2 driver identifier.*/
#if STM32_I2C_USE_I2C2 || defined(__DOXYGEN__)
I2CDriver I2CD2;
#endif

/** @brief I2C3 driver identifier.*/
#if STM32_I2C_USE_I2C3 || defined(__DOXYGEN__)
I2CDriver I2CD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Aborts an I2C transaction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_abort_operation(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* Stops the I2C peripheral.*/
  dp->CR1 = I2C_CR1_SWRST;
  dp->CR1 = 0;
  dp->CR2 = 0;
  dp->SR1 = 0;

  /* Stops the associated DMA streams.*/
  dmaStreamDisable(i2cp->dmatx);
  dmaStreamDisable(i2cp->dmarx);
}

/**
 * @brief   Set clock speed.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_set_clock(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  uint16_t regCCR, clock_div;
  int32_t clock_speed = i2cp->config->clock_speed;
  i2cdutycycle_t duty = i2cp->config->duty_cycle;

  osalDbgCheck((i2cp != NULL) &&
               (clock_speed > 0) &&
               (clock_speed <= 400000));

  /* CR2 Configuration.*/
  dp->CR2 &= (uint16_t)~I2C_CR2_FREQ;
  dp->CR2 |= (uint16_t)I2C_CLK_FREQ;

  /* CCR Configuration.*/
  regCCR = 0;
  clock_div = I2C_CCR_CCR;

  if (clock_speed <= 100000) {
    /* Configure clock_div in standard mode.*/
    osalDbgAssert(duty == STD_DUTY_CYCLE, "invalid standard mode duty cycle");

    /* Standard mode clock_div calculate: Tlow/Thigh = 1/1.*/
    osalDbgAssert((STM32_PCLK1 % (clock_speed * 2)) == 0,
                  "PCLK1 must be divisible without remainder");
    clock_div = (uint16_t)(STM32_PCLK1 / (clock_speed * 2));

    osalDbgAssert(clock_div >= 0x04,
                  "clock divider less then 0x04 not allowed");
    regCCR |= (clock_div & I2C_CCR_CCR);

    /* Sets the Maximum Rise Time for standard mode.*/
    dp->TRISE = I2C_CLK_FREQ + 1;
  }
  else if (clock_speed <= 400000) {
    /* Configure clock_div in fast mode.*/
    osalDbgAssert((duty == FAST_DUTY_CYCLE_2) ||
                  (duty == FAST_DUTY_CYCLE_16_9),
                  "invalid fast mode duty cycle");

    if (duty == FAST_DUTY_CYCLE_2) {
      /* Fast mode clock_div calculate: Tlow/Thigh = 2/1.*/
      osalDbgAssert((STM32_PCLK1 % (clock_speed * 3)) == 0,
                    "PCLK1 must be divided without remainder");
      clock_div = (uint16_t)(STM32_PCLK1 / (clock_speed * 3));
    }
    else if (duty == FAST_DUTY_CYCLE_16_9) {
      /* Fast mode clock_div calculate: Tlow/Thigh = 16/9.*/
      osalDbgAssert((STM32_PCLK1 % (clock_speed * 25)) == 0,
                    "PCLK1 must be divided without remainder");
      clock_div = (uint16_t)(STM32_PCLK1 / (clock_speed * 25));
      regCCR |= I2C_CCR_DUTY;
    }

    osalDbgAssert(clock_div >= 0x01,
                  "clock divider less then 0x04 not allowed");
    regCCR |= (I2C_CCR_FS | (clock_div & I2C_CCR_CCR));

    /* Sets the Maximum Rise Time for fast mode.*/
    dp->TRISE = (I2C_CLK_FREQ * 300 / 1000) + 1;
  }

  osalDbgAssert((clock_div <= I2C_CCR_CCR), "the selected clock is too low");

  dp->CCR = regCCR;
}

#ifdef I2C_FLTR_ANOFF
/**
 * @brief   Set filter params.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_set_filter(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  i2cdutycycle_t duty = i2cp->config->duty_cycle;
  uint8_t filter;

  if (duty == STD_DUTY_CYCLE) {
    if (I2C_CLK_FREQ <= 5) {
      filter = 2;
    } else if (I2C_CLK_FREQ <= 10) {
      filter = 12;
    } else {
      filter = 15;
    }
  } else {
    if (I2C_CLK_FREQ <= 10) {
      filter = 0;
    } else if (I2C_CLK_FREQ <= 20) {
      filter = 1;
    } else if (I2C_CLK_FREQ <= 30) {
      filter = 7;
    } else if (I2C_CLK_FREQ <= 40) {
      filter = 13;
    } else {
      filter = 15;
    }
  }

  dp->FLTR = (I2C_FLTR_ANOFF) | (I2C_FLTR_DNF & filter);
}
#endif // I2C_FLTR_ANOFF

/**
 * @brief   Set operation mode of I2C hardware.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_set_opmode(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  i2copmode_t opmode = i2cp->config->op_mode;
  uint16_t regCR1;

  regCR1 = dp->CR1;
  switch (opmode) {
  case OPMODE_I2C:
    regCR1 &= (uint16_t)~(I2C_CR1_SMBUS|I2C_CR1_SMBTYPE);
    break;
  case OPMODE_SMBUS_DEVICE:
    regCR1 |= I2C_CR1_SMBUS;
    regCR1 &= (uint16_t)~(I2C_CR1_SMBTYPE);
    break;
  case OPMODE_SMBUS_HOST:
    regCR1 |= (I2C_CR1_SMBUS|I2C_CR1_SMBTYPE);
    break;
  }
  dp->CR1 = regCR1;
}

#ifdef STM32_I2C_ISR_LIMIT
/**
 * @brief reset I2C peripheral
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_reset(I2CDriver *i2cp) {
#if STM32_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      rccResetI2C1();
    }
#endif /* STM32_I2C_USE_I2C1 */

#if STM32_I2C_USE_I2C2
    if (&I2CD2 == i2cp) {
      rccResetI2C2();
    }
#endif /* STM32_I2C_USE_I2C2 */

#if STM32_I2C_USE_I2C3
    if (&I2CD3 == i2cp) {
      rccResetI2C3();
    }
#endif /* STM32_I2C_USE_I2C3 */
}
#endif // STM32_I2C_ISR_LIMIT

/**
 * @brief   I2C shared ISR code.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_serve_event_interrupt(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;
  uint32_t regSR2 = dp->SR2;
  uint32_t event = dp->SR1;

#ifdef STM32_I2C_ISR_LIMIT
    if (i2cp->isr_count++ > i2cp->isr_limit) {
        i2cp->errors |= I2C_ISR_LIMIT;
        if (i2cp->dmatx)
            dmaStreamDisable(i2cp->dmatx);
        if (i2cp->dmarx)
            dmaStreamDisable(i2cp->dmarx);
        dp->CR1 |= I2C_CR1_SWRST;
        dp->CR1 &= ~I2C_CR1_PE;
        i2c_lld_reset(i2cp);
        _i2c_wakeup_error_isr(i2cp);
        return;
    }
#endif

  if (!i2cp->in_transaction) {
      // we have an interrupt while not inside an I2C operation. This
      // can happen with sufficient bus noise. The best we can do is
      // disable the peripheral and reset it
      dp->CR1 |= I2C_CR1_SWRST;
      dp->CR1 &= ~I2C_CR1_PE;
      return;
  }

  /* Interrupts are disabled just before dmaStreamEnable() because there
     is no need of interrupts until next transaction begin. All the work is
     done by the DMA.*/
  switch (I2C_EV_MASK & (event | (regSR2 << 16))) {
  case I2C_EV5_MASTER_MODE_SELECT:
  case I2C_EV5_MASTER_MODE_SELECT_NO_BUSY:
    if ((i2cp->addr >> 8) > 0) {
      /* 10-bit address: 1 1 1 1 0 X X R/W */
      dp->DR = 0xF0 | (0x6 & (i2cp->addr >> 8)) | (0x1 & i2cp->addr);
    } else {
      dp->DR = i2cp->addr;
    }
    break;
  case I2C_EV9_MASTER_ADD10:
    /* Set second addr byte (10-bit addressing)*/
    dp->DR = (0xFF & (i2cp->addr >> 1));
    break;
  case I2C_EV6_MASTER_REC_MODE_SELECTED:
    dp->CR2 &= ~I2C_CR2_ITEVTEN;
    dmaStreamEnable(i2cp->dmarx);
    dp->CR2 |= I2C_CR2_LAST;                 /* Needed in receiver mode. */
    if (dmaStreamGetTransactionSize(i2cp->dmarx) < 2)
      dp->CR1 &= ~I2C_CR1_ACK;
    break;
  case I2C_EV6_MASTER_TRA_MODE_SELECTED:
    dp->CR2 &= ~I2C_CR2_ITEVTEN;
    dmaStreamEnable(i2cp->dmatx);
    break;
  case I2C_EV8_2_MASTER_BYTE_TRANSMITTED:
    /* Catches BTF event after the end of transmission.*/
    (void)dp->DR; /* Clear BTF.*/
    if (dmaStreamGetTransactionSize(i2cp->dmarx) > 0) {
      /* Starts "read after write" operation, LSB = 1 -> receive.*/
      i2cp->addr |= 0x01;
      dp->CR1 |= I2C_CR1_START | I2C_CR1_ACK;
      return;
    }
    dp->CR2 &= ~I2C_CR2_ITEVTEN;
    dp->CR1 |= I2C_CR1_STOP;
    _i2c_wakeup_isr(i2cp);
    break;
  case I2C_EV5_MASTER_MODE_INVALID:
    i2c_lld_abort_operation(i2cp);
    dp->CR2 &= ~I2C_CR2_ITEVTEN;
    break;
#if (I2C_ENABLE_SLAVE_MODE == TRUE)
  case I2C_EV1_SLAVE_RECEIVER_ADDRESS_MATCHED:
    dmaStreamEnable(i2cp->dmarx);
    break;
  case I2C_EV1_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
    dp->CR1 &= ~I2C_CR1_ACK;
    dp->CR2 &= ~I2C_CR2_ITEVTEN;
    i2cp->reply_required = true;
    if (i2cp->state == I2C_ACTIVE_RX) {
      if (dmaStreamGetTransactionSize(i2cp->dmarx)) {
        dmaStreamDisable(i2cp->dmarx);
      }
      _i2c_wakeup_isr(i2cp);
    }
    break;
 // case  I2C_EV1_SLAVE_RECEIVER_SECONDADDRESS_MATCHED:
 // case  I2C_EV1_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED:
 // case  I2C_EV1_SLAVE_GENERALCALLADDRESS_MATCHED:
 // case  I2C_EV2_SLAVE_BYTE_RECEIVED:
  case I2C_EV4_SLAVE_STOP_DETECTED:
    if (dmaStreamGetTransactionSize(i2cp->dmarx)) {
      dmaStreamDisable(i2cp->dmarx);
    }
    _i2c_wakeup_isr(i2cp);

    dp->CR1 &= ~I2C_CR1_ACK;
    dp->CR2 &= ~I2C_CR2_ITEVTEN;
    break;
//  case  I2C_EV3_SLAVE_BYTE_TRANSMITTED:
  case I2C_EV3_SLAVE_BYTE_TRANSMITTING:
    dmaStreamEnable(i2cp->dmatx);
    dp->CR2 &= ~I2C_CR2_ITEVTEN;
    break;
//case  I2C_EV2_SLAVE_ACK_FAILURE:
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
  default:
    break;
  }
  /* Clear ADDR flag. */
  if (event & (I2C_SR1_ADDR | I2C_SR1_ADD10))
    (void)dp->SR2;

  /* BERR flag doesn't happen anymore in event handling */
#if 0
  /* Errata 2.4.6 for STM32F40x, Spurious Bus Error detection in Master mode.*/
  if (event & I2C_SR1_BERR) {
    dp->SR1 &= ~I2C_SR1_BERR;
  }
#endif
}

/**
 * @brief   DMA RX end IRQ handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 *
 * @notapi
 */
static void i2c_lld_serve_rx_end_irq(I2CDriver *i2cp, uint32_t flags) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* DMA errors handling.*/
#if defined(STM32_I2C_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_I2C_DMA_ERROR_HOOK(i2cp);
  }
#else
  (void)flags;
#endif

  dmaStreamDisable(i2cp->dmarx);

#if (I2C_ENABLE_SLAVE_MODE == TRUE)
  /* If Slave: lines are released by hardware, do nothing
     If Master: a Stop or repeated Start condition must be generated by software */
  if(i2cp->isMaster)
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
  {
    dp->CR2 &= ~I2C_CR2_LAST;
    dp->CR1 &= ~I2C_CR1_ACK;
    dp->CR1 |= I2C_CR1_STOP;
    _i2c_wakeup_isr(i2cp);
  }
#if (I2C_ENABLE_SLAVE_MODE == TRUE)
  else {
    dp->CR1 |= I2C_CR1_ACK;
    dp->CR2 |= I2C_CR2_ITEVTEN;
  }
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
}

/**
 * @brief    DMA TX end IRQ handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
static void i2c_lld_serve_tx_end_irq(I2CDriver *i2cp, uint32_t flags) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* DMA errors handling.*/
#if defined(STM32_I2C_DMA_ERROR_HOOK)
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    STM32_I2C_DMA_ERROR_HOOK(i2cp);
  }
#else
  (void)flags;
#endif

  dmaStreamDisable(i2cp->dmatx);
  /* Enables interrupts to catch BTF event meaning transmission part complete.
     Interrupt handler will decide to generate STOP or to begin receiving part
     of R/W transaction itself.*/
  dp->CR2 |= I2C_CR2_ITEVTEN;
}

/**
 * @brief   I2C error handler.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in]  sr       content of the SR1 register to be decoded
 *
 * @notapi
 */
static void i2c_lld_serve_error_interrupt(I2CDriver *i2cp, uint16_t sr) {

  /* Clears interrupt flags just to be safe.*/
  if (i2cp->dmatx)
  dmaStreamDisable(i2cp->dmatx);
  if (i2cp->dmarx)
  dmaStreamDisable(i2cp->dmarx);

#ifdef STM32_I2C_ISR_LIMIT
    if (i2cp->isr_count++ > i2cp->isr_limit) {
        i2cp->errors |= I2C_ISR_LIMIT;
        i2cp->i2c->CR1 |= I2C_CR1_SWRST;
        i2cp->i2c->CR1 &= ~I2C_CR1_PE;
        i2c_lld_reset(i2cp);
        _i2c_wakeup_error_isr(i2cp);
        return;
    }
#endif

  if (!i2cp->in_transaction) {
      // we have an interrupt while not inside an I2C operation. This
      // can happen with sufficient bus noise. The best we can do is
      // disable the peripheral and reset it
      i2cp->i2c->CR1 |= I2C_CR1_SWRST;
      i2cp->i2c->CR1 &= ~I2C_CR1_PE;
      return;
  }

  i2cp->errors = I2C_NO_ERROR;

  if (sr & I2C_SR1_BERR) {                          /* Bus error.           */
    i2cp->errors |= I2C_BUS_ERROR;
    /* No more needed */
#if 0
    /* Errata 2.4.6 for STM32F40x, Spurious Bus Error detection in
       Master mode.*/
    i2cp->i2c->SR1 &= ~I2C_SR1_BERR;
#endif
  }

  if (sr & I2C_SR1_ARLO)                            /* Arbitration lost.    */
    i2cp->errors |= I2C_ARBITRATION_LOST;

  if (sr & I2C_SR1_AF) {                            /* Acknowledge fail.    */
#if (I2C_ENABLE_SLAVE_MODE == TRUE)
     /* If Slave: lines are released by hardware, do nothing
        If Master: a Stop or repeated Start condition must be generated by software */
    if(i2cp->isMaster)
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
    {
      i2cp->i2c->CR2 &= ~I2C_CR2_ITEVTEN;
      i2cp->i2c->CR1 |= I2C_CR1_STOP;                 /* Setting stop bit.    */
      i2cp->errors |= I2C_ACK_FAILURE;
    }
#if (I2C_ENABLE_SLAVE_MODE == TRUE)
    else {
      i2cp->i2c->SR1 &= ~I2C_SR1_AF;
      i2cp->i2c->CR2 &= ~I2C_CR2_ITBUFEN;
      i2cp->reply_required = false;
      _i2c_wakeup_isr(i2cp);
    }
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
  }

  if (sr & I2C_SR1_OVR)                             /* Overrun.             */
    i2cp->errors |= I2C_OVERRUN;

  if (sr & I2C_SR1_TIMEOUT)                         /* SMBus Timeout.       */
    i2cp->errors |= I2C_TIMEOUT;

  if (sr & I2C_SR1_PECERR)                          /* PEC error.           */
    i2cp->errors |= I2C_PEC_ERROR;

  if (sr & I2C_SR1_SMBALERT)                        /* SMBus alert.         */
    i2cp->errors |= I2C_SMB_ALERT;

  /* If some error has been identified then sends wakes the waiting thread.*/
  if (i2cp->errors != I2C_NO_ERROR)
    _i2c_wakeup_error_isr(i2cp);
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_I2C_USE_I2C1 || defined(__DOXYGEN__)
/**
 * @brief   I2C1 event interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(STM32_I2C1_EVENT_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD1);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   I2C1 error interrupt handler.
 */
OSAL_IRQ_HANDLER(STM32_I2C1_ERROR_HANDLER) {
  uint16_t sr = I2CD1.i2c->SR1;

  OSAL_IRQ_PROLOGUE();

  I2CD1.i2c->SR1 = ~(sr & I2C_ERROR_MASK);
  i2c_lld_serve_error_interrupt(&I2CD1, sr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_I2C_USE_I2C1 */

#if STM32_I2C_USE_I2C2 || defined(__DOXYGEN__)
/**
 * @brief   I2C2 event interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(STM32_I2C2_EVENT_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD2);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   I2C2 error interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(STM32_I2C2_ERROR_HANDLER) {
  uint16_t sr = I2CD2.i2c->SR1;

  OSAL_IRQ_PROLOGUE();

  I2CD2.i2c->SR1 = ~(sr & I2C_ERROR_MASK);
  i2c_lld_serve_error_interrupt(&I2CD2, sr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_I2C_USE_I2C2 */

#if STM32_I2C_USE_I2C3 || defined(__DOXYGEN__)
/**
 * @brief   I2C3 event interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(STM32_I2C3_EVENT_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  i2c_lld_serve_event_interrupt(&I2CD3);

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   I2C3 error interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(STM32_I2C3_ERROR_HANDLER) {
  uint16_t sr = I2CD3.i2c->SR1;

  OSAL_IRQ_PROLOGUE();

  I2CD3.i2c->SR1 = ~(sr & I2C_ERROR_MASK);
  i2c_lld_serve_error_interrupt(&I2CD3, sr);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_I2C_USE_I2C3 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {

#if STM32_I2C_USE_I2C1
  i2cObjectInit(&I2CD1);
  I2CD1.thread = NULL;
  I2CD1.i2c    = I2C1;
  I2CD1.dmarx  = NULL;
  I2CD1.dmatx  = NULL;
#endif /* STM32_I2C_USE_I2C1 */

#if STM32_I2C_USE_I2C2
  i2cObjectInit(&I2CD2);
  I2CD2.thread = NULL;
  I2CD2.i2c    = I2C2;
  I2CD2.dmarx  = NULL;
  I2CD2.dmatx  = NULL;
#endif /* STM32_I2C_USE_I2C2 */

#if STM32_I2C_USE_I2C3
  i2cObjectInit(&I2CD3);
  I2CD3.thread = NULL;
  I2CD3.i2c    = I2C3;
  I2CD3.dmarx  = NULL;
  I2CD3.dmatx  = NULL;
#endif /* STM32_I2C_USE_I2C3 */
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cp) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* If in stopped state then enables the I2C and DMA clocks.*/
  if (i2cp->state == I2C_STOP) {

    i2cp->txdmamode = STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE |
                      STM32_DMA_CR_MINC       | STM32_DMA_CR_DMEIE |
                      STM32_DMA_CR_TEIE       | STM32_DMA_CR_TCIE |
                      STM32_DMA_CR_DIR_M2P;
    i2cp->rxdmamode = STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE |
                      STM32_DMA_CR_MINC       | STM32_DMA_CR_DMEIE |
                      STM32_DMA_CR_TEIE       | STM32_DMA_CR_TCIE |
                      STM32_DMA_CR_DIR_P2M;

#if STM32_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      rccResetI2C1();

      i2cp->dmarx = dmaStreamAllocI(STM32_I2C_I2C1_RX_DMA_STREAM,
                                    STM32_I2C_I2C1_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2c_lld_serve_rx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmarx != NULL, "unable to allocate stream");
      i2cp->dmatx = dmaStreamAllocI(STM32_I2C_I2C1_TX_DMA_STREAM,
                                    STM32_I2C_I2C1_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2c_lld_serve_tx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmatx != NULL, "unable to allocate stream");

      rccEnableI2C1(true);
      nvicEnableVector(I2C1_EV_IRQn, STM32_I2C_I2C1_IRQ_PRIORITY);
      nvicEnableVector(I2C1_ER_IRQn, STM32_I2C_I2C1_IRQ_PRIORITY);

      i2cp->rxdmamode |= STM32_DMA_CR_CHSEL(I2C1_RX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C1_DMA_PRIORITY);
      i2cp->txdmamode |= STM32_DMA_CR_CHSEL(I2C1_TX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C1_DMA_PRIORITY);
#if (I2C_ENABLE_SLAVE_MODE == TRUE)
      i2cp->reply_required = false;
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
    }
#endif /* STM32_I2C_USE_I2C1 */

#if STM32_I2C_USE_I2C2
    if (&I2CD2 == i2cp) {
      rccResetI2C2();

      i2cp->dmarx = dmaStreamAllocI(STM32_I2C_I2C2_RX_DMA_STREAM,
                                    STM32_I2C_I2C2_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2c_lld_serve_rx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmarx != NULL, "unable to allocate stream");
      i2cp->dmatx = dmaStreamAllocI(STM32_I2C_I2C2_TX_DMA_STREAM,
                                    STM32_I2C_I2C2_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2c_lld_serve_tx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmatx != NULL, "unable to allocate stream");

      rccEnableI2C2(true);
      nvicEnableVector(I2C2_EV_IRQn, STM32_I2C_I2C2_IRQ_PRIORITY);
      nvicEnableVector(I2C2_ER_IRQn, STM32_I2C_I2C2_IRQ_PRIORITY);

      i2cp->rxdmamode |= STM32_DMA_CR_CHSEL(I2C2_RX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C2_DMA_PRIORITY);
      i2cp->txdmamode |= STM32_DMA_CR_CHSEL(I2C2_TX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C2_DMA_PRIORITY);
#if (I2C_ENABLE_SLAVE_MODE == TRUE)
      i2cp->reply_required = false;
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
    }
#endif /* STM32_I2C_USE_I2C2 */

#if STM32_I2C_USE_I2C3
    if (&I2CD3 == i2cp) {
      rccResetI2C3();

      i2cp->dmarx = dmaStreamAllocI(STM32_I2C_I2C3_RX_DMA_STREAM,
                                    STM32_I2C_I2C3_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2c_lld_serve_rx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmarx != NULL, "unable to allocate stream");
      i2cp->dmatx = dmaStreamAllocI(STM32_I2C_I2C3_TX_DMA_STREAM,
                                    STM32_I2C_I2C3_IRQ_PRIORITY,
                                    (stm32_dmaisr_t)i2c_lld_serve_tx_end_irq,
                                    (void *)i2cp);
      osalDbgAssert(i2cp->dmatx != NULL, "unable to allocate stream");

      rccEnableI2C3(true);
      nvicEnableVector(I2C3_EV_IRQn, STM32_I2C_I2C3_IRQ_PRIORITY);
      nvicEnableVector(I2C3_ER_IRQn, STM32_I2C_I2C3_IRQ_PRIORITY);

      i2cp->rxdmamode |= STM32_DMA_CR_CHSEL(I2C3_RX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C3_DMA_PRIORITY);
      i2cp->txdmamode |= STM32_DMA_CR_CHSEL(I2C3_TX_DMA_CHANNEL) |
                       STM32_DMA_CR_PL(STM32_I2C_I2C3_DMA_PRIORITY);
#if (I2C_ENABLE_SLAVE_MODE == TRUE)
      i2cp->reply_required = false;
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
    }
#endif /* STM32_I2C_USE_I2C3 */
  }

  /* I2C registers pointed by the DMA.*/
  dmaStreamSetPeripheral(i2cp->dmarx, &dp->DR);
  dmaStreamSetPeripheral(i2cp->dmatx, &dp->DR);

  /* Reset i2c peripheral.*/
  dp->CR1 = I2C_CR1_SWRST;
  dp->CR1 = 0;
  dp->CR2 = I2C_CR2_ITERREN | I2C_CR2_DMAEN;

  /* Setup I2C parameters.*/
  i2c_lld_set_clock(i2cp);
#ifdef I2C_FLTR_ANOFF
  i2c_lld_set_filter(i2cp);
#endif
  i2c_lld_set_opmode(i2cp);

  /* Ready to go.*/
  dp->CR1 |= I2C_CR1_PE;
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  /* If not in stopped state then disables the I2C clock.*/
  if (i2cp->state != I2C_STOP) {

    /* I2C disable.*/
    i2c_lld_abort_operation(i2cp);
    dmaStreamFreeI(i2cp->dmatx);
    dmaStreamFreeI(i2cp->dmarx);
    i2cp->dmatx = NULL;
    i2cp->dmarx = NULL;

#if STM32_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      nvicDisableVector(I2C1_EV_IRQn);
      nvicDisableVector(I2C1_ER_IRQn);
      rccDisableI2C1();
    }
#endif

#if STM32_I2C_USE_I2C2
    if (&I2CD2 == i2cp) {
      nvicDisableVector(I2C2_EV_IRQn);
      nvicDisableVector(I2C2_ER_IRQn);
      rccDisableI2C2();
    }
#endif

#if STM32_I2C_USE_I2C3
    if (&I2CD3 == i2cp) {
      nvicDisableVector(I2C3_EV_IRQn);
      nvicDisableVector(I2C3_ER_IRQn);
      rccDisableI2C3();
    }
#endif
  }
}

/**
 * @brief   Disable DMA, disable interrupts, but leave the peripheral enabled
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_soft_stop(I2CDriver *i2cp) {

  /* If not in stopped state then disables the I2C clock.*/
  if (i2cp->state != I2C_STOP) {

    /* I2C disable.*/
    dmaStreamFreeI(i2cp->dmatx);
    dmaStreamFreeI(i2cp->dmarx);
    i2cp->dmatx = NULL;
    i2cp->dmarx = NULL;

#if STM32_I2C_USE_I2C1
    if (&I2CD1 == i2cp) {
      nvicDisableVector(I2C1_EV_IRQn);
      nvicDisableVector(I2C1_ER_IRQn);
    }
#endif

#if STM32_I2C_USE_I2C2
    if (&I2CD2 == i2cp) {
      nvicDisableVector(I2C2_EV_IRQn);
      nvicDisableVector(I2C2_ER_IRQn);
    }
#endif

#if STM32_I2C_USE_I2C3
    if (&I2CD3 == i2cp) {
      nvicDisableVector(I2C3_EV_IRQn);
      nvicDisableVector(I2C3_ER_IRQn);
    }
#endif
  }
}


/**
 * @brief   Receives data via the I2C bus as master.
 * @details Number of receiving bytes must be more than 1 on STM32F1x. This is
 *          hardware restriction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                     uint8_t *rxbuf, size_t rxbytes,
                                     sysinterval_t timeout) {
  I2C_TypeDef *dp = i2cp->i2c;
  msg_t msg;

#ifdef STM32_I2C_ISR_LIMIT
  i2cp->isr_limit = rxbytes * STM32_I2C_ISR_LIMIT;
  i2cp->isr_count = 0;
#endif

#if defined(STM32F1XX_I2C)
  osalDbgCheck(rxbytes > 1);
#endif

#if (I2C_ENABLE_SLAVE_MODE == TRUE)
  i2cp->isMaster = true;
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */

  /* Resetting error flags for this transfer.*/
  i2cp->errors = I2C_NO_ERROR;

  /* Initializes driver fields, LSB = 1 -> receive.*/
  i2cp->addr = (addr << 1) | 0x01;

  /* we expect the bus to not be busy when this is called. If it is
   * busy then immediately fail and let higher level code restart
   * I2C. Note that we return with lock held. */
  if ((dp->SR2 & I2C_SR2_BUSY) || (dp->CR1 & I2C_CR1_STOP)) {
      return MSG_TIMEOUT;
  }

  /* Releases the lock from high level driver.*/
  osalSysUnlock();

  /* RX DMA setup.*/
  dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
  dmaStreamSetMemory0(i2cp->dmarx, rxbuf);
  dmaStreamSetTransactionSize(i2cp->dmarx, rxbytes);

    osalSysLock();

  i2cp->in_transaction = true;

  /* Starts the operation.*/
  dp->CR2 |= I2C_CR2_ITEVTEN;
  dp->CR1 |= I2C_CR1_START | I2C_CR1_ACK;

  /* Waits for the operation completion or a timeout.*/
  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);
    dmaStreamDisable(i2cp->dmarx);

  i2cp->in_transaction = false;

  return msg;
}

/**
 * @brief   Transmits data via the I2C bus as master.
 * @details Number of receiving bytes must be 0 or more than 1 on STM32F1x.
 *          This is hardware restriction.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[in] txbuf     pointer to the transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts,
 *                      the following special values are allowed:
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      sysinterval_t timeout) {
  I2C_TypeDef *dp = i2cp->i2c;
  msg_t msg;

#if defined(STM32F1XX_I2C) && !defined(_ARDUPILOT_)
  osalDbgCheck((rxbytes == 0) || ((rxbytes > 1) && (rxbuf != NULL)));
#endif

#if (I2C_ENABLE_SLAVE_MODE == TRUE)
  i2cp->isMaster = true;
#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */
#ifdef STM32_I2C_ISR_LIMIT
  i2cp->isr_limit = (txbytes + rxbytes) * STM32_I2C_ISR_LIMIT;
  i2cp->isr_count = 0;
#endif

  /* Resetting error flags for this transfer.*/
  i2cp->errors = I2C_NO_ERROR;

  /* Initializes driver fields, LSB = 0 -> transmit.*/
  i2cp->addr = (addr << 1);

  /* we expect the bus to not be busy when this is called. If it is
   * busy then immediately fail and let higher level code restart
   * I2C. Note that we return with lock held. */
  if ((dp->SR2 & I2C_SR2_BUSY) || (dp->CR1 & I2C_CR1_STOP)) {
      return MSG_TIMEOUT;
  }

  /* Releases the lock from high level driver.*/
  osalSysUnlock();

  /* TX DMA setup.*/
  dmaStreamSetMode(i2cp->dmatx, i2cp->txdmamode);
  dmaStreamSetMemory0(i2cp->dmatx, txbuf);
  dmaStreamSetTransactionSize(i2cp->dmatx, txbytes);

  /* RX DMA setup.*/
  dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
  dmaStreamSetMemory0(i2cp->dmarx, rxbuf);
  dmaStreamSetTransactionSize(i2cp->dmarx, rxbytes);

    osalSysLock();

  i2cp->in_transaction = true;

  /* Starts the operation.*/
  dp->CR2 |= I2C_CR2_ITEVTEN;
  dp->CR1 |= I2C_CR1_START;

  /* Waits for the operation completion or a timeout.*/
  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);
    dmaStreamDisable(i2cp->dmatx);
    dmaStreamDisable(i2cp->dmarx);

  i2cp->in_transaction = false;

  return msg;
}

#if (I2C_ENABLE_SLAVE_MODE == TRUE)
/**
 * @brief   Listen I2C bus for address match.
 * @details Use 7 bit address.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 *
 * @notapi
 */
msg_t i2c_lld_match_address(I2CDriver *i2cp, i2caddr_t addr) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* Slave mode */
  i2cp->isMaster = false;

  uint16_t i2cadr = addr << 1;
  uint16_t ownAdr = dp->OAR1 & (0x7f<<1);

  if (ownAdr == 0 || ownAdr == i2cadr)
      /* 14 bit should always be kept at 1 by software */
    dp->OAR1 = i2cadr | 0x4000;
  else
    /* cannot add this address to set of those matched */
    return MSG_RESET;

  return MSG_OK;
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
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_slave_receive_timeout(I2CDriver *i2cp, uint8_t *rxbuf, size_t rxbytes, sysinterval_t timeout) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* Slave mode */
  i2cp->isMaster = false;

  /* Reset Reply flag */
  i2cp->reply_required = false;

  dp->CR1 |= I2C_CR1_ACK;
  dp->CR2 |= I2C_CR2_ITEVTEN;

  /* RX DMA setup.*/
  dmaStreamSetMode(i2cp->dmarx, i2cp->rxdmamode);
  dmaStreamSetMemory0(i2cp->dmarx, rxbuf);
  dmaStreamSetTransactionSize(i2cp->dmarx, rxbytes);

  /* Waits for the operation completion or a timeout.*/
  return osalThreadSuspendTimeoutS(&i2cp->thread, timeout);
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
 *                      .
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_slave_transmit_timeout(I2CDriver *i2cp,
                                     const uint8_t *txbuf,
                                     size_t txbytes,
                                     sysinterval_t timeout) {
  I2C_TypeDef *dp = i2cp->i2c;

  /* Slave mode */
  i2cp->isMaster = false;

  /* TX DMA setup.*/
  dmaStreamSetMode(i2cp->dmatx, i2cp->txdmamode);
  dmaStreamSetMemory0(i2cp->dmatx, txbuf);
  dmaStreamSetTransactionSize(i2cp->dmatx, txbytes);

  dp->CR1 &= ~I2C_CR1_ACK;
  dp->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN;

  /* Waits for the operation completion or a timeout.*/
  return osalThreadSuspendTimeoutS(&i2cp->thread, timeout);
}

#endif /* I2C_ENABLE_SLAVE_MODE == TRUE */

#endif /* HAL_USE_I2C */

/** @} */
