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
 * @file    FDCANv2/hal_can_lld.c
 * @brief   STM32 CAN subsystem low level driver source.
 *
 * @addtogroup CAN
 * @{
 */

#include "hal.h"

#if HAL_USE_CAN || defined(__DOXYGEN__)

/*
 * TODO
 * The message received are stored only in RX FIFO 0 and RX FIFO 1, it depends on the configuration (GFC and SFEC/EFEC).
 * With this revision of driver, the filter is always active (es. FDCAN_SIDFC.LSS > 0).
 * The management of interrupt RX BUFFER (FDCAN_IR.DRX) is not implemented.
 * */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#define STM32_FDCAN_FIFO_WATERMARK          1U

/*===========================================================================*/
/* STM32H723xx, STM32H733xx, STM32H725xx, STM32H735xx, STM32H730xx.          */
/*===========================================================================*/
#if defined(STM32H723xx) || defined(STM32H733xx) ||                         \
    defined(STM32H725xx) || defined(STM32H735xx) ||                         \
    defined(STM32H730xx) ||                                                 \
    defined(__DOXYGEN__)

#if (STM32_CAN_USE_FDCAN1 && STM32_CAN_USE_FDCAN2 && STM32_CAN_USE_FDCAN3)
#define STM32_FDCAN_FLS_NBR                 21U
#define STM32_FDCAN_FLE_NBR                 10U
#define STM32_FDCAN_RF0_NBR                 10U
#define STM32_FDCAN_RF1_NBR                 10U
#define STM32_FDCAN_RB_NBR                  10U
#define STM32_FDCAN_TEF_NBR                 5U
#define STM32_FDCAN_TB_NBR                  10U
#define STM32_FDCAN_TM_NBR                  10U
#elif ((STM32_CAN_USE_FDCAN1 && STM32_CAN_USE_FDCAN2 && !STM32_CAN_USE_FDCAN3) || \
      (STM32_CAN_USE_FDCAN1 && STM32_CAN_USE_FDCAN3 && !STM32_CAN_USE_FDCAN2) || \
      (STM32_CAN_USE_FDCAN2 && STM32_CAN_USE_FDCAN3 && !STM32_CAN_USE_FDCAN1))
#define STM32_FDCAN_FLS_NBR                 32U
#define STM32_FDCAN_FLE_NBR                 16U
#define STM32_FDCAN_RF0_NBR                 16U
#define STM32_FDCAN_RF1_NBR                 16U
#define STM32_FDCAN_RB_NBR                  16U
#define STM32_FDCAN_TEF_NBR                 8U
#define STM32_FDCAN_TB_NBR                  16U
#define STM32_FDCAN_TM_NBR                  16U
#elif ((STM32_CAN_USE_FDCAN1 && !STM32_CAN_USE_FDCAN2 && !STM32_CAN_USE_FDCAN3) || \
       (STM32_CAN_USE_FDCAN2 && !STM32_CAN_USE_FDCAN1 && !STM32_CAN_USE_FDCAN3) || \
       (STM32_CAN_USE_FDCAN3 && !STM32_CAN_USE_FDCAN1 && !STM32_CAN_USE_FDCAN2))
#define STM32_FDCAN_FLS_NBR                 64U
#define STM32_FDCAN_FLE_NBR                 32U
#define STM32_FDCAN_RF0_NBR                 32U
#define STM32_FDCAN_RF1_NBR                 32U
#define STM32_FDCAN_RB_NBR                  32U
#define STM32_FDCAN_TEF_NBR                 16U
#define STM32_FDCAN_TB_NBR                  32U
#define STM32_FDCAN_TM_NBR                  32U
#endif
#endif /* defined(STM32H723xx) || defined(STM32H733xx) ||
          defined(STM32H725xx) || defined(STM32H735xx) ||
          defined(STM32H730xx) */

/*===========================================================================*/
/* STM32H750xx.                                                              */
/*===========================================================================*/
#if defined(STM32H750xx) || defined(STM32H7B0xx) ||                         \
    defined(__DOXYGEN__)

#if (STM32_CAN_USE_FDCAN1 && STM32_CAN_USE_FDCAN2)
#define STM32_FDCAN_FLS_NBR                 32U
#define STM32_FDCAN_FLE_NBR                 16U
#define STM32_FDCAN_RF0_NBR                 16U
#define STM32_FDCAN_RF1_NBR                 16U
#define STM32_FDCAN_RB_NBR                  16U
#define STM32_FDCAN_TEF_NBR                 8U
#define STM32_FDCAN_TB_NBR                  16U
#define STM32_FDCAN_TM_NBR                  16U
#else
#define STM32_FDCAN_FLS_NBR                 64U
#define STM32_FDCAN_FLE_NBR                 32U
#define STM32_FDCAN_RF0_NBR                 32U
#define STM32_FDCAN_RF1_NBR                 32U
#define STM32_FDCAN_RB_NBR                  32U
#define STM32_FDCAN_TEF_NBR                 16U
#define STM32_FDCAN_TB_NBR                  32U
#define STM32_FDCAN_TM_NBR                  32U
#endif
#endif /* defined(STM32H750xx) */

/*===========================================================================*/
/* STM32H743xx, STM32H753xx, STM32H745xx, STM32H755xx, STM32H747xx,          */
/* STM32H757xx.                                                              */
/*===========================================================================*/
#if defined(STM32H743xx) || defined(STM32H753xx) ||                         \
    defined(STM32H745xx) || defined(STM32H755xx) ||                         \
    defined(STM32H747xx) || defined(STM32H757xx) ||                         \
    defined(__DOXYGEN__)

#if (STM32_CAN_USE_FDCAN1 && STM32_CAN_USE_FDCAN2)
#define STM32_FDCAN_FLS_NBR                 32U
#define STM32_FDCAN_FLE_NBR                 16U
#define STM32_FDCAN_RF0_NBR                 16U
#define STM32_FDCAN_RF1_NBR                 16U
#define STM32_FDCAN_RB_NBR                  16U
#define STM32_FDCAN_TEF_NBR                 8U
#define STM32_FDCAN_TB_NBR                  16U
#define STM32_FDCAN_TM_NBR                  16U
#else
#define STM32_FDCAN_FLS_NBR                 64U
#define STM32_FDCAN_FLE_NBR                 32U
#define STM32_FDCAN_RF0_NBR                 32U
#define STM32_FDCAN_RF1_NBR                 32U
#define STM32_FDCAN_RB_NBR                  32U
#define STM32_FDCAN_TEF_NBR                 16U
#define STM32_FDCAN_TB_NBR                  32U
#define STM32_FDCAN_TM_NBR                  32U
#endif
#endif /* defined(STM32H743xx) || defined(STM32H753xx) */

/*===========================================================================*/
/* STM32H7A3xx, STM32H7B3xx, STM32H7A3xxQ, STM32H7B3xxQ.                     */
/*===========================================================================*/
#if defined(STM32H7A3xx)  || defined(STM32H7B3xx)  ||                       \
    defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) ||                       \
    defined(__DOXYGEN__)

#if (STM32_CAN_USE_FDCAN1 && STM32_CAN_USE_FDCAN2)
#define STM32_FDCAN_FLS_NBR                 32U
#define STM32_FDCAN_FLE_NBR                 16U
#define STM32_FDCAN_RF0_NBR                 16U
#define STM32_FDCAN_RF1_NBR                 16U
#define STM32_FDCAN_RB_NBR                  16U
#define STM32_FDCAN_TEF_NBR                 8U
#define STM32_FDCAN_TB_NBR                  16U
#define STM32_FDCAN_TM_NBR                  16U
#else
#define STM32_FDCAN_FLS_NBR                 64U
#define STM32_FDCAN_FLE_NBR                 32U
#define STM32_FDCAN_RF0_NBR                 32U
#define STM32_FDCAN_RF1_NBR                 32U
#define STM32_FDCAN_RB_NBR                  32U
#define STM32_FDCAN_TEF_NBR                 16U
#define STM32_FDCAN_TB_NBR                  32U
#define STM32_FDCAN_TM_NBR                  32U
#endif
#endif /* defined(STM32H7A3xx)  || defined(STM32H7B3xx) ||
          defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) */

/* Size of element size (RAM words) */
#define CAN_SIZE_RAM_WORDS              (4U)
#define FDCAN_SIZE_RAM_WORDS            (18U)

/* Value of instance */
#if (STM32_CAN_USE_FDCAN1 && STM32_CAN_USE_FDCAN2 && STM32_CAN_USE_FDCAN3)
#define CAN1_OFFSET_INSTANCE            (0U) // CAN1 DEFINED
#define CAN2_OFFSET_INSTANCE            (1U) // CAN2 DEFINED
#define CAN3_OFFSET_INSTANCE            (2U) // CAN3 DEFINED
#elif (STM32_CAN_USE_FDCAN1 && STM32_CAN_USE_FDCAN2 && !STM32_CAN_USE_FDCAN3)
#define CAN1_OFFSET_INSTANCE            (0U) // CAN1 DEFINED
#define CAN2_OFFSET_INSTANCE            (1U) // CAN2 DEFINED
#elif (STM32_CAN_USE_FDCAN1 && STM32_CAN_USE_FDCAN3 && !STM32_CAN_USE_FDCAN2)
#define CAN1_OFFSET_INSTANCE            (0U) // CAN1 DEFINED
#define CAN3_OFFSET_INSTANCE            (1U) // CAN3 DEFINED
#elif (STM32_CAN_USE_FDCAN2 && STM32_CAN_USE_FDCAN3 && !STM32_CAN_USE_FDCAN1)
#define CAN2_OFFSET_INSTANCE            (0U) // CAN2 DEFINED
#define CAN3_OFFSET_INSTANCE            (1U) // CAN3 DEFINED
#elif (STM32_CAN_USE_FDCAN1 && !STM32_CAN_USE_FDCAN2 && !STM32_CAN_USE_FDCAN3)
#define CAN1_OFFSET_INSTANCE            (0U) // CAN1 DEFINED
#elif (STM32_CAN_USE_FDCAN2 && !STM32_CAN_USE_FDCAN1 && !STM32_CAN_USE_FDCAN3)
#define CAN2_OFFSET_INSTANCE            (0U) // CAN2 DEFINED
#elif (STM32_CAN_USE_FDCAN3 && !STM32_CAN_USE_FDCAN1 && !STM32_CAN_USE_FDCAN2)
#define CAN3_OFFSET_INSTANCE            (0U) // CAN3 DEFINED
#endif

/* Filter Standard Element MAX Size in WORDS for single element. */
#define SRAMCAN_FLS_SIZE                (1U)

/* Filter Extended Element MAX Size in WORDS for single element.*/
#define SRAMCAN_FLE_SIZE                (2U)

/* RX FIFO 0 Elements MAX Size in WORDS for single element.*/
#define SRAMCAN_RF0_SIZE                (18U)

/* RX FIFO 1 Elements MAX Size in WORDS for single element.*/
#define SRAMCAN_RF1_SIZE                (18U)

/* RX Buffer MAX Size in WORDS for single element.*/
#define SRAMCAN_RB_SIZE                 (18U)

/* TX Event FIFO Elements MAX Size in WORDS for single element.*/
#define SRAMCAN_TEF_SIZE                (2U)

/* TX FIFO/Queue Elements MAX Size in WORDS for single element.*/
#define SRAMCAN_TB_SIZE                 (18U)

/* Trigger Memory MAX Size in WORDS for single element.*/
#define SRAMCAN_TM_SIZE                 (2U)

/* Filter List Standard Offset.*/
#define SRAMCAN_FLSSA ((uint32_t)0)

/* Filter List Extended Offset.*/
#define SRAMCAN_FLESA ((uint32_t)(SRAMCAN_FLSSA +                           \
                                  (STM32_FDCAN_FLS_NBR * SRAMCAN_FLS_SIZE)))

/* RX FIFO 0 Offset.*/
#define SRAMCAN_RF0SA ((uint32_t)(SRAMCAN_FLESA +                           \
                                  (STM32_FDCAN_FLE_NBR * SRAMCAN_FLE_SIZE)))

/* RX FIFO 1 Offset.*/
#define SRAMCAN_RF1SA ((uint32_t)(SRAMCAN_RF0SA +                           \
                                  (STM32_FDCAN_RF0_NBR * SRAMCAN_RF0_SIZE)))

/* RX Buffer Offset.*/
#define SRAMCAN_RBSA  ((uint32_t)(SRAMCAN_RF1SA +                           \
                                  (STM32_FDCAN_RF1_NBR * SRAMCAN_RF1_SIZE)))

/* TX Event FIFO Offset.*/
#define SRAMCAN_TEFSA ((uint32_t)(SRAMCAN_RBSA +                            \
                                  (STM32_FDCAN_RB_NBR * SRAMCAN_RB_SIZE)))

/* TX Buffers Offset.*/
#define SRAMCAN_TBSA  ((uint32_t)(SRAMCAN_TEFSA +                           \
                                  (STM32_FDCAN_TEF_NBR * SRAMCAN_TEF_SIZE)))

/* Trigger Memory Offset.*/
#define SRAMCAN_TMSA  ((uint32_t)(SRAMCAN_TBSA +                            \
                                  (STM32_FDCAN_TB_NBR * SRAMCAN_TB_SIZE)))

/* Message RAM size.*/
#define SRAMCAN_SIZE  ((uint32_t)(SRAMCAN_TMSA +                            \
                                  (STM32_FDCAN_TM_NBR * SRAMCAN_TM_SIZE)))

#define TIMEOUT_INIT_MS                 250U
#define TIMEOUT_CSA_MS                  250U

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief CAN1 driver identifier.*/
#if STM32_CAN_USE_FDCAN1 || defined(__DOXYGEN__)
CANDriver CAND1;
#endif

/** @brief CAN2 driver identifier.*/
#if STM32_CAN_USE_FDCAN2 || defined(__DOXYGEN__)
CANDriver CAND2;
#endif

/** @brief CAN3 driver identifier.*/
#if STM32_CAN_USE_FDCAN3 || defined(__DOXYGEN__)
CANDriver CAND3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static bool fdcan_clock_stop(CANDriver *canp) {
  systime_t start, end;

  /* Requesting clock stop then waiting for it to happen.*/
  canp->fdcan->CCCR |= FDCAN_CCCR_CSR;
  start = osalOsGetSystemTimeX();
  end = osalTimeAddX(start, TIME_MS2I(TIMEOUT_INIT_MS));
  while ((canp->fdcan->CCCR & FDCAN_CCCR_CSA) == 0U) {
    if (!osalTimeIsInRangeX(osalOsGetSystemTimeX(), start, end)) {
      return true;
    }
    osalThreadSleepS(1);
  }

  return false;
}

static bool fdcan_init_mode(CANDriver *canp) {
  systime_t start, end;

  /* Going in initialization mode then waiting for it to happen.*/
  canp->fdcan->CCCR |= FDCAN_CCCR_INIT;
  start = osalOsGetSystemTimeX();
  end = osalTimeAddX(start, TIME_MS2I(TIMEOUT_INIT_MS));
  while ((canp->fdcan->CCCR & FDCAN_CCCR_INIT) == 0U) {
    if (!osalTimeIsInRangeX(osalOsGetSystemTimeX(), start, end)) {
      return true;
    }
    osalThreadSleepS(1);
  }

  return false;
}

static bool fdcan_active_mode(CANDriver *canp) {
  systime_t start, end;

  /* Going in active mode then waiting for it to happen.*/
  canp->fdcan->CCCR &= ~FDCAN_CCCR_INIT;
  start = osalOsGetSystemTimeX();
  end = osalTimeAddX(start, TIME_MS2I(TIMEOUT_INIT_MS));
  while ((canp->fdcan->CCCR & FDCAN_CCCR_INIT) != 0U) {
    if (!osalTimeIsInRangeX(osalOsGetSystemTimeX(), start, end)) {
      return true;
    }
    osalThreadSleepS(1);
  }

  return false;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level CAN driver initialization.
 *
 * @notapi
 */
void can_lld_init(void) {

  /* Unit reset.*/
  rccResetFDCAN();

#if STM32_CAN_USE_FDCAN1
  /* Driver initialization.*/
  canObjectInit(&CAND1);
  CAND1.fdcan = FDCAN1;
  CAND1.ram_base = ((uint32_t *)SRAMCAN_BASE + (CAN1_OFFSET_INSTANCE * SRAMCAN_SIZE));
#endif

#if STM32_CAN_USE_FDCAN2
  /* Driver initialization.*/
  canObjectInit(&CAND2);
  CAND2.fdcan = FDCAN2;
  CAND2.ram_base = ((uint32_t *)SRAMCAN_BASE + (CAN2_OFFSET_INSTANCE * SRAMCAN_SIZE));
#endif

#if STM32_CAN_USE_FDCAN3
  /* Driver initialization.*/
  canObjectInit(&CAND3);
  CAND3.fdcan = FDCAN3;
  CAND3.ram_base = ((uint32_t *)SRAMCAN_BASE + (CAN3_OFFSET_INSTANCE * SRAMCAN_SIZE));
#endif
}

/**
 * @brief   Configures and activates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @return              The operation result.
 * @retval false        if the operation succeeded.
 * @retval true         if the operation failed.
 *
 * @notapi
 */
bool can_lld_start(CANDriver *canp) {

  /* Clock activation.*/
  rccEnableFDCAN(true);

  /* If it is the first activation then performing some extra
     initializations.*/
  for (uint32_t *wp = canp->ram_base;
       wp < canp->ram_base + SRAMCAN_SIZE;
       wp += 1U) {
    *wp = (uint32_t)0U;
  }

  /* Requesting clock stop.*/
  if (fdcan_clock_stop(canp)) {
    osalDbgAssert(false, "CAN clock stop failed, check clocks and pin config");
    return true;
  }

  /* Going in initialization mode.*/
  if (fdcan_init_mode(canp)) {
    osalDbgAssert(false, "CAN initialization failed, check clocks and pin config");
    return true;
  }

  /* Configuration of element size (RAM words). */
  if (canp->config->op_mode == OPMODE_FDCAN) {
    canp->word_size = FDCAN_SIZE_RAM_WORDS;
  }
  else if(canp->config->op_mode == OPMODE_CAN) {
    canp->word_size = CAN_SIZE_RAM_WORDS;
  }
  else {
    osalDbgAssert(false, "CAN initialization failed, invalid FDCAN operation mode");
  }

  /* Configuration can be performed now.*/
  canp->fdcan->CCCR |= FDCAN_CCCR_CCE;

  /* Setting up operation mode except driver-controlled bits.*/
  canp->fdcan->NBTP = canp->config->NBTP;
  canp->fdcan->DBTP = canp->config->DBTP;
  canp->fdcan->TDCR = canp->config->TDCR;
  canp->fdcan->CCCR |= canp->config->CCCR;

  /* TEST is only writable when FDCAN_CCCR_TEST is set and FDCAN is still in
   * configuration mode */
  if (canp->config->CCCR & FDCAN_CCCR_TEST) {
    canp->fdcan->TEST = canp->config->TEST;
  }

  canp->fdcan->GFC = canp->config->RXGFC;

#if STM32_CAN_USE_FDCAN1
  if (&CAND1 == canp) {
    /* H7 version of FDCAN has configurable memory layout, so configure it */
    canp->fdcan->SIDFC = FDCAN_CONFIG_SIDFC_LSS(STM32_FDCAN_FLS_NBR) |
                         FDCAN_CONFIG_SIDFC_FLSSA((CAN1_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_FLSSA);
    canp->fdcan->XIDFC = FDCAN_CONFIG_XIDFC_LSE(STM32_FDCAN_FLE_NBR) |
                         FDCAN_CONFIG_XIDFC_FLESA((CAN1_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_FLESA);
    canp->fdcan->RXF0C = FDCAN_CONFIG_RXF0C_F0S(STM32_FDCAN_RF0_NBR) |
                         FDCAN_CONFIG_RXF0C_F0SA((CAN1_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_RF0SA) |
                         FDCAN_CONFIG_RXF0C_F0WM(STM32_FDCAN_FIFO_WATERMARK);
    canp->fdcan->RXF1C = FDCAN_CONFIG_RXF1C_F1S(STM32_FDCAN_RF1_NBR) |
                         FDCAN_CONFIG_RXF1C_F1SA((CAN1_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_RF1SA) |
                         FDCAN_CONFIG_RXF1C_F1WM(STM32_FDCAN_FIFO_WATERMARK);
    canp->fdcan->RXBC  = FDCAN_CONFIG_RXBC_RBSA((CAN1_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_RBSA);
    canp->fdcan->TXEFC = FDCAN_CONFIG_TXEFC_EFS(STM32_FDCAN_TEF_NBR) |
                         FDCAN_CONFIG_TXEFC_EFSA((CAN1_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_TEFSA);
    /* NB: this doesn't set NDTB, but sets TFQS to run in FIFO mode with no dedicated buffers */
    canp->fdcan->TXBC  = FDCAN_CONFIG_TXBC_TFQS(STM32_FDCAN_TB_NBR) |
                         FDCAN_CONFIG_TXBC_TBSA((CAN1_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_TBSA);
  }
#endif

#if STM32_CAN_USE_FDCAN2
  if (&CAND2 == canp) {
    /* H7 version of FDCAN has configurable memory layout, so configure it */
    canp->fdcan->SIDFC = FDCAN_CONFIG_SIDFC_LSS(STM32_FDCAN_FLS_NBR) |
                         FDCAN_CONFIG_SIDFC_FLSSA((CAN2_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_FLSSA);
    canp->fdcan->XIDFC = FDCAN_CONFIG_XIDFC_LSE(STM32_FDCAN_FLE_NBR) |
                         FDCAN_CONFIG_XIDFC_FLESA((CAN2_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_FLESA);
    canp->fdcan->RXF0C = FDCAN_CONFIG_RXF0C_F0S(STM32_FDCAN_RF0_NBR) |
                         FDCAN_CONFIG_RXF0C_F0SA((CAN2_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_RF0SA) |
                         FDCAN_CONFIG_RXF0C_F0WM(STM32_FDCAN_FIFO_WATERMARK);
    canp->fdcan->RXF1C = FDCAN_CONFIG_RXF1C_F1S(STM32_FDCAN_RF1_NBR) |
                         FDCAN_CONFIG_RXF1C_F1SA((CAN2_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_RF1SA) |
                         FDCAN_CONFIG_RXF1C_F1WM(STM32_FDCAN_FIFO_WATERMARK);
    canp->fdcan->RXBC  = FDCAN_CONFIG_RXBC_RBSA((CAN2_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_RBSA);
    canp->fdcan->TXEFC = FDCAN_CONFIG_TXEFC_EFS(STM32_FDCAN_TEF_NBR) |
                         FDCAN_CONFIG_TXEFC_EFSA((CAN2_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_TEFSA);
    /* NB: this doesn't set NDTB, but sets TFQS to run in FIFO mode with no dedicated buffers */
    canp->fdcan->TXBC  = FDCAN_CONFIG_TXBC_TFQS(STM32_FDCAN_TB_NBR) |
                         FDCAN_CONFIG_TXBC_TBSA((CAN2_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_TBSA);
  }
#endif

#if STM32_CAN_USE_FDCAN3
  if (&CAND3 == canp) {
    /* H7 version of FDCAN has configurable memory layout, so configure it */
    canp->fdcan->SIDFC = FDCAN_CONFIG_SIDFC_LSS(STM32_FDCAN_FLS_NBR) |
                         FDCAN_CONFIG_SIDFC_FLSSA((CAN3_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_FLSSA);
    canp->fdcan->XIDFC = FDCAN_CONFIG_XIDFC_LSE(STM32_FDCAN_FLE_NBR) |
                         FDCAN_CONFIG_XIDFC_FLESA((CAN3_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_FLESA);
    canp->fdcan->RXF0C = FDCAN_CONFIG_RXF0C_F0S(STM32_FDCAN_RF0_NBR) |
                         FDCAN_CONFIG_RXF0C_F0SA((CAN3_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_RF0SA) |
                         FDCAN_CONFIG_RXF0C_F0WM(STM32_FDCAN_FIFO_WATERMARK);
    canp->fdcan->RXF1C = FDCAN_CONFIG_RXF1C_F1S(STM32_FDCAN_RF1_NBR) |
                         FDCAN_CONFIG_RXF1C_F1SA((CAN3_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_RF1SA) |
                         FDCAN_CONFIG_RXF1C_F1WM(STM32_FDCAN_FIFO_WATERMARK);
    canp->fdcan->RXBC  = FDCAN_CONFIG_RXBC_RBSA((CAN3_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_RBSA);
    canp->fdcan->TXEFC = FDCAN_CONFIG_TXEFC_EFS(STM32_FDCAN_TEF_NBR) |
                         FDCAN_CONFIG_TXEFC_EFSA((CAN3_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_TEFSA);
    /* NB: this doesn't set NDTB, but sets TFQS to run in FIFO mode with no dedicated buffers */
    canp->fdcan->TXBC  = FDCAN_CONFIG_TXBC_TFQS(STM32_FDCAN_TB_NBR) |
                         FDCAN_CONFIG_TXBC_TBSA((CAN3_OFFSET_INSTANCE * SRAMCAN_SIZE) + SRAMCAN_TBSA);
  }
#endif

  /* Data field. Data field sizes > 8
     bytes are intended for CAN FD operation only. */
  if (canp->config->op_mode == OPMODE_FDCAN) {
    canp->fdcan->TXESC = FDCAN_CONFIG_TXESC_TBDS_64BDF;
    canp->fdcan->RXESC = FDCAN_CONFIG_RXESC_F0DS_64BDF | FDCAN_CONFIG_RXESC_F1DS_64BDF |
                         FDCAN_CONFIG_RXESC_RBDS_64BDF;
  }
  else {
    canp->fdcan->TXESC = FDCAN_CONFIG_TXESC_TBDS_8BDF;
    canp->fdcan->RXESC = FDCAN_CONFIG_RXESC_F0DS_8BDF | FDCAN_CONFIG_RXESC_F1DS_8BDF |
                         FDCAN_CONFIG_RXESC_RBDS_8BDF;
  }

  /* Start clock and disable configuration mode.*/
  canp->fdcan->CCCR &= ~(FDCAN_CCCR_CSR);

  /* Enable FDCAN operation. */
  if (canp->config->op_mode == OPMODE_FDCAN) {
    canp->fdcan->CCCR |= FDCAN_CCCR_FDOE;
  }

  /* Enabling interrupts, only using interrupt zero.*/
  canp->fdcan->IR     = (uint32_t)-1;
  canp->fdcan->IE     = FDCAN_IE_RF1WE | FDCAN_IE_RF1LE |
                        FDCAN_IE_RF0WE | FDCAN_IE_RF0LE |
                        FDCAN_IE_TCE | FDCAN_IE_BOE;
  canp->fdcan->TXBTIE = FDCAN_TXBTIE_TIE;
  canp->fdcan->ILE    = FDCAN_ILE_EINT0;

  /* Going in active mode.*/
  if (fdcan_active_mode(canp)) {
    osalDbgAssert(false, "CAN initialization failed, check clocks and pin config");
    return true;
  }

  return false;
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_stop(CANDriver *canp) {

  /* If in ready state then disables the CAN peripheral.*/
  if (canp->state == CAN_READY) {
    /* Disabling and clearing interrupts.*/
    canp->fdcan->IE  = 0U;
    canp->fdcan->IR  = (uint32_t)-1;
    canp->fdcan->ILE = 0U;
    canp->fdcan->TXBTIE = 0U;

    /* Disables the peripheral.*/
    (void) fdcan_clock_stop(canp);

    rccDisableFDCAN();
  }
}

/**
 * @brief   Determines whether a frame can be transmitted.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval false        no space in the transmit queue.
 * @retval true         transmit slot available.
 *
 * @notapi
 */
bool can_lld_is_tx_empty(CANDriver *canp, canmbx_t mailbox) {

  (void)mailbox;

  return (bool)(((canp->fdcan->TXFQS & FDCAN_TXFQS_TFQF) == 0U) &&
                ((canp->fdcan->TXFQS & FDCAN_TXFQS_TFFL) > 0U));
}

/**
 * @brief   Inserts a frame into the transmit queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] ctfp      pointer to the CAN frame to be transmitted
 * @param[in] mailbox   mailbox number,  @p CAN_ANY_MAILBOX for any mailbox
 *
 * @notapi
 */
void can_lld_transmit(CANDriver *canp, canmbx_t mailbox, const CANTxFrame *ctfp) {
  uint32_t put_index = 0;
  uint32_t *tx_address = 0;

  (void)mailbox;

  osalDbgCheck(dlc_to_bytes[ctfp->DLC] <= CAN_MAX_DLC_BYTES);

  /* Retrieve the TX FIFO put index.*/
  put_index = ((canp->fdcan->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos);

  /* Writing frame. */
  tx_address = canp->ram_base + (SRAMCAN_TBSA + (put_index * canp->word_size));

  *tx_address++ = ctfp->header32[0];
  *tx_address++ = ctfp->header32[1];
  for (unsigned i = 0U; i < dlc_to_bytes[ctfp->DLC]; i += 4U) {
    *tx_address++ = ctfp->data32[i / 4U];
  }

  /* Starting transmission.*/
  canp->fdcan->TXBAR = ((uint32_t)1 << put_index);

}

/**
 * @brief   Determines whether a frame has been received.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 *
 * @return              The queue space availability.
 * @retval false        no new messages available.
 * @retval true         new messages available.
 *
 * @notapi
 */
bool can_lld_is_rx_nonempty(CANDriver *canp, canmbx_t mailbox) {

  switch (mailbox) {
    case CAN_ANY_MAILBOX:
      return can_lld_is_rx_nonempty(canp, 1U) ||
             can_lld_is_rx_nonempty(canp, 2U);
    case 1:
      return (bool)((canp->fdcan->RXF0S & FDCAN_RXF0S_F0FL) != 0U);
    case 2:
      return (bool)((canp->fdcan->RXF1S & FDCAN_RXF1S_F1FL) != 0U);
    default:
      return false;
  }
}

/**
 * @brief   Receives a frame from the input queue.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number, @p CAN_ANY_MAILBOX for any mailbox
 * @param[out] crfp     pointer to the buffer where the CAN frame is copied
 *
 * @notapi
 */
void can_lld_receive(CANDriver *canp, canmbx_t mailbox, CANRxFrame *crfp) {
  uint32_t get_index;
  uint32_t *rx_address;

  if (mailbox == CAN_ANY_MAILBOX) {
    if (can_lld_is_rx_nonempty(canp, 1U)) {
      mailbox = 1U;
    }
    else if (can_lld_is_rx_nonempty(canp, 2U)) {
      mailbox = 2U;
    }
    else {
      return;
    }
  }

  if (mailbox == 1U) {
     /* GET index RXF0, add it and the length to the rx_address.*/
     get_index = (canp->fdcan->RXF0S & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
     rx_address = canp->ram_base + ((SRAMCAN_RF0SA + (get_index * canp->word_size)));
  }
  else {
     /* GET index RXF1, add it and the length to the rx_address.*/
     get_index = (canp->fdcan->RXF1S & FDCAN_RXF1S_F1GI_Msk) >> FDCAN_RXF1S_F1GI_Pos;
     rx_address = canp->ram_base + ((SRAMCAN_RF1SA + (get_index * canp->word_size)));
  }
  crfp->header32[0] = *rx_address++;
  crfp->header32[1] = *rx_address++;

  /* Copy message from FDCAN peripheral's SRAM to structure. RAM is restricted
     to word aligned accesses, so up to 3 extra bytes may be copied.*/
  for (unsigned i = 0U; i < dlc_to_bytes[crfp->DLC]; i += 4U) {
    crfp->data32[i / 4U] = *rx_address++;
  }

  /* Acknowledge receipt by writing the get-index to the acknowledge
     register RXFxA then re-enable RX FIFO message arrived interrupt once
     the FIFO is emptied.*/
  if (mailbox == 1U) {
    uint32_t rxf0a = canp->fdcan->RXF0A;
    rxf0a &= ~FDCAN_RXF0A_F0AI_Msk;
    rxf0a |= get_index << FDCAN_RXF0A_F0AI_Pos;
    canp->fdcan->RXF0A = rxf0a;
  }
  else {
    uint32_t rxf1a = canp->fdcan->RXF1A;
    rxf1a &= ~FDCAN_RXF1A_F1AI_Msk;
    rxf1a |= get_index << FDCAN_RXF1A_F1AI_Pos;
    canp->fdcan->RXF1A = rxf1a;
  }
}

/**
 * @brief   Tries to abort an ongoing transmission.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] mailbox   mailbox number
 *
 * @notapi
 */
void can_lld_abort(CANDriver *canp, canmbx_t mailbox) {

  (void)canp;
  (void)mailbox;
}

#if CAN_USE_SLEEP_MODE || defined(__DOXYGEN__)
/**
 * @brief   Enters the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_sleep(CANDriver *canp) {

  (void)canp;
}

/**
 * @brief   Enforces leaving the sleep mode.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_wakeup(CANDriver *canp) {

  (void)canp;
}
#endif /* CAN_USE_SLEEP_MODE */

/**
 * @brief   FDCAN IRQ0 service routine.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 *
 * @notapi
 */
void can_lld_serve_interrupt(CANDriver *canp) {
  uint32_t ir = 0;

  /* Getting and clearing active IRQs.*/
  ir = canp->fdcan->IR;
  canp->fdcan->IR = ir;

  /* RX events.*/
  if ((ir & FDCAN_IR_RF0W) != 0U) {
    _can_rx_full_isr(canp, CAN_MAILBOX_TO_MASK(1U));
  }
  if ((ir & FDCAN_IR_RF1W) != 0U) {
    _can_rx_full_isr(canp, CAN_MAILBOX_TO_MASK(2U));
  }

  /* Overflow events.*/
  if (((ir & FDCAN_IR_RF0L) != 0U) || ((ir & FDCAN_IR_RF1L) != 0U)) {
    _can_error_isr(canp, CAN_OVERFLOW_ERROR);
  }

  /* Bus_off events.*/
  if ((ir & FDCAN_IR_BO) != 0U)  {
    _can_error_isr(canp, CAN_BUS_OFF_ERROR);
  }

  /* TX events.*/
  if ((ir & FDCAN_IR_TC) != 0U) {
    eventflags_t flags = 0U;

    flags |= CAN_MAILBOX_TO_MASK(1U);
    _can_tx_empty_isr(canp, flags);
  }
}

/**
 * @brief   Programs the filters.
 * @note    This is an STM32-specific API.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] num       number of entries in the filters array
 * @param[in] cfp       pointer to the filters array
 *
 * @notapi
 */
void can_lld_set_filters(CANDriver *canp, uint8_t num, const CANFilter *cfp) {

  uint8_t i;
  uint8_t num_std_filter = 0;
  uint8_t num_ext_filter = 0;

  CANRxStandardFilter *filter_std;
  CANRxExtendedFilter *filter_ext;

  /* Check number of filters. */
  for (i = 0; i < num; i++) {
    if ((cfp[i].filter_type) == CAN_FILTER_TYPE_STD) {
      num_std_filter++;
    }
    else {
      num_ext_filter++;
    }
  }

  osalDbgAssert(((num_std_filter <= STM32_FDCAN_FLS_NBR) &&
                 (num_ext_filter <= STM32_FDCAN_FLE_NBR)),
                "out of range of filters supported");

  /* Base address of standard filter. */
  filter_std = (CANRxStandardFilter *)(canp->ram_base + SRAMCAN_FLSSA);
  /* Base address of extended filter. */
  filter_ext = (CANRxExtendedFilter *)(canp->ram_base + SRAMCAN_FLESA);

  /* Scanning the filters array. */
  for (i = 0; i < num; i++) {
    /* Standard filter configuration */
    if ((cfp[i].filter_type) == CAN_FILTER_TYPE_STD) {
      /* Configure */
      filter_std->data32 = (FDCAN_STD_FILTER_SFID2(cfp[i].identifier2) |
                            FDCAN_STD_FILTER_SFID1(cfp[i].identifier1) |
                            FDCAN_STD_FILTER_SFEC(cfp[i].filter_cfg) |
                            FDCAN_STD_FILTER_SFT(cfp[i].filter_mode));
      filter_std++;
    }
    /* Extended filter configuration */
    else {

      /* Configure */
      filter_ext->data32[0] = (FDCAN_EXT_FILTER_EFID1(cfp[i].identifier1) |
                               FDCAN_EXT_FILTER_EFEC(cfp[i].filter_cfg));
      filter_ext->data32[1] = (FDCAN_EXT_FILTER_EFID2(cfp[i].identifier2) |
                               FDCAN_EXT_FILTER_EFT(cfp[i].filter_mode));
      filter_ext++;
    }
  }
}
/**
 * @brief   Programs the filters.
 * @note    This is an STM32-specific API.
 *
 * @param[in] canp      pointer to the @p CANDriver object
 * @param[in] num       number of entries in the filters array
 * @param[in] cfp       pointer to the filters array
 *
 * @api
 */
void canSTM32SetFilters(CANDriver *canp, uint8_t num, const CANFilter *cfp) {

  osalDbgAssert(canp->state == CAN_READY, "invalid state");

  can_lld_set_filters(canp, num, cfp);
}

#endif /* HAL_USE_CAN */

/** @} */
