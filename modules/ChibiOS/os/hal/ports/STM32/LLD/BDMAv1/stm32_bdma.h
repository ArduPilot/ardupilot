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
 * @file    BDMAv1/stm32_bdma.h
 * @brief   BDMA helper driver header.
 * @note    This driver uses the new naming convention used for the STM32F2xx
 *          so the "BDMA channels" are referred as "BDMA streams".
 *
 * @addtogroup STM32_BDMA
 * @{
 */

#ifndef STM32_BDMA_H
#define STM32_BDMA_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Total number of BDMA streams.
 * @details This is the total number of streams among all the BDMA units.
 */
#define STM32_BDMA_STREAMS                  8U

/**
 * @brief   Mask of the ISR bits passed to the BDMA callback functions.
 */
#define STM32_BDMA_ISR_MASK                 0x0EU

/**
 * @brief   Checks if a BDMA priority is within the valid range.
 *
 * @param[in] prio      BDMA priority
 * @retval              The check result.
 * @retval false        invalid BDMA priority.
 * @retval true         correct BDMA priority.
 */
#define STM32_BDMA_IS_VALID_PRIORITY(prio)  (((prio) >= 0U) && ((prio) <= 3U))

/**
 * @brief   Checks if a BDMA stream id is within the valid range.
 *
 * @param[in] id        BDMA stream id
 * @retval              The check result.
 * @retval false        invalid DMA stream.
 * @retval true         correct DMA stream.
 */
#define STM32_BDMA_IS_VALID_STREAM(id)      (((id) >= 0U) &&                \
                                             ((id) <= STM32_BDMA_STREAMS))

/**
 * @name    Special stream identifiers
 * @{
 */
#define STM32_BDMA_STREAM_ID_ANY            STM32_BDMA_STREAMS
/** @} */

/**
 * @name    BDMA streams identifiers
 * @{
 */
/**
 * @brief   Returns a pointer to a stm32_dma_stream_t structure.
 *
 * @param[in] id        the stream numeric identifier
 * @return              A pointer to the stm32_bdma_stream_t constant structure
 *                      associated to the BDMA stream.
 */
#define STM32_BDMA_STREAM(id)               (&_stm32_bdma_streams[id])

#define STM32_BDMA1_STREAM0                 STM32_BDMA_STREAM(0)
#define STM32_BDMA1_STREAM1                 STM32_BDMA_STREAM(1)
#define STM32_BDMA1_STREAM2                 STM32_BDMA_STREAM(2)
#define STM32_BDMA1_STREAM3                 STM32_BDMA_STREAM(3)
#define STM32_BDMA1_STREAM4                 STM32_BDMA_STREAM(4)
#define STM32_BDMA1_STREAM5                 STM32_BDMA_STREAM(5)
#define STM32_BDMA1_STREAM6                 STM32_BDMA_STREAM(6)
#define STM32_BDMA1_STREAM7                 STM32_BDMA_STREAM(7)
/** @} */

/**
 * @name    CR register constants
 * @{
 */
#define STM32_BDMA_CR_RESET_VALUE           0x00000000U
#define STM32_BDMA_CR_EN                    BDMA_CCR_EN_Msk
#define STM32_BDMA_CR_TCIE                  BDMA_CCR_TCIE
#define STM32_BDMA_CR_HTIE                  BDMA_CCR_HTIE
#define STM32_BDMA_CR_TEIE                  BDMA_CCR_TEIE
#define STM32_BDMA_CR_DIR_MASK              (BDMA_CCR_DIR | BDMA_CCR_MEM2MEM)
#define STM32_BDMA_CR_DIR_P2M               0U
#define STM32_BDMA_CR_DIR_M2P               BDMA_CCR_DIR
#define STM32_BDMA_CR_DIR_M2M               BDMA_CCR_MEM2MEM
#define STM32_BDMA_CR_CIRC                  BDMA_CCR_CIRC
#define STM32_BDMA_CR_PINC                  BDMA_CCR_PINC
#define STM32_BDMA_CR_MINC                  BDMA_CCR_MINC
#define STM32_BDMA_CR_PSIZE_MASK            BDMA_CCR_PSIZE_Msk
#define STM32_BDMA_CR_PSIZE_BYTE            0U
#define STM32_BDMA_CR_PSIZE_HWORD           BDMA_CCR_PSIZE_0
#define STM32_BDMA_CR_PSIZE_WORD            BDMA_CCR_PSIZE_1
#define STM32_BDMA_CR_MSIZE_MASK            BDMA_CCR_MSIZE_Msk
#define STM32_BDMA_CR_MSIZE_BYTE            0U
#define STM32_BDMA_CR_MSIZE_HWORD           BDMA_CCR_MSIZE_0
#define STM32_BDMA_CR_MSIZE_WORD            BDMA_CCR_MSIZE_1
#define STM32_BDMA_CR_SIZE_MASK             (STM32_BDMA_CR_PSIZE_MASK |     \
                                             STM32_BDMA_CR_MSIZE_MASK)
#define STM32_BDMA_CR_PL_MASK               BDMA_CCR_PL_Msk
#define STM32_BDMA_CR_PL(n)                 ((n) << 12U)
#define STM32_BDMA_CR_DBM                   BDMA_CCR_DBM
#define STM32_BDMA_CR_CM                    BDMA_CCR_CT
/** @} */

/**
 * @name    Status flags passed to the ISR callbacks
 * @{
 */
#define STM32_BDMA_ISR_TEIF                 BDMA_ISR_TEIF0
#define STM32_BDMA_ISR_HTIF                 BDMA_ISR_HTIF0
#define STM32_BDMA_ISR_TCIF                 BDMA_ISR_TCIF0
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

#if !defined(STM32_HAS_BDMA1)
#error "STM32_HAS_BDMA1 missing in registry"
#endif

#if !defined(STM32_BDMA1_CH0_HANDLER)
#error "STM32_BDMA1_CH0_HANDLER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH1_HANDLER)
#error "STM32_BDMA1_CH1_HANDLER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH2_HANDLER)
#error "STM32_BDMA1_CH2_HANDLER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH3_HANDLER)
#error "STM32_BDMA1_CH3_HANDLER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH4_HANDLER)
#error "STM32_BDMA1_CH4_HANDLER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH5_HANDLER)
#error "STM32_BDMA1_CH5_HANDLER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH6_HANDLER)
#error "STM32_BDMA1_CH6_HANDLER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH7_HANDLER)
#error "STM32_BDMA1_CH7_HANDLER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH0_NUMBER)
#error "STM32_BDMA1_CH0_NUMBER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH1_NUMBER)
#error "STM32_BDMA1_CH1_NUMBER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH2_NUMBER)
#error "STM32_BDMA1_CH2_NUMBER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH3_NUMBER)
#error "STM32_BDMA1_CH3_NUMBER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH4_NUMBER)
#error "STM32_BDMA1_CH4_NUMBER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH5_NUMBER)
#error "STM32_BDMA1_CH5_NUMBER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH6_NUMBER)
#error "STM32_BDMA1_CH6_NUMBER missing in registry"
#endif

#if !defined(STM32_BDMA1_CH7_NUMBER)
#error "STM32_BDMA1_CH7_NUMBER missing in registry"
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   STM32 BDMA ISR function type.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] flags     pre-shifted content of the ISR register, the bits
 *                      are aligned to bit zero
 */
typedef void (*stm32_bdmaisr_t)(void *p, uint32_t flags);

/**
 * @brief   STM32 BDMA stream descriptor structure.
 */
typedef struct {
  BDMA_TypeDef              *bdma;      /**< @brief Associated BDMA.        */
  BDMA_Channel_TypeDef      *channel;   /**< @brief Associated BDMA channel.*/
  uint8_t                   shift;      /**< @brief Bit offset in ISR and
                                             IFCR registers.                */
  DMAMUX_Channel_TypeDef    *mux;       /**< @brief Associated BDMA stream. */
  uint8_t                   selfindex;  /**< @brief Index to self in array. */
  uint8_t                   vector;     /**< @brief Associated IRQ vector.  */
} stm32_bdma_stream_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Associates a peripheral data register to a BDMA stream.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 * @param[in] addr      value to be written in the CPAR register
 *
 * @special
 */
#define bdmaStreamSetPeripheral(stp, addr) {                                \
  (stp)->channel->CPAR  = (uint32_t)(addr);                                 \
}

/**
 * @brief   Associates a memory destination to a BDMA stream.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 * @param[in] addr      value to be written in the CM0AR register
 *
 * @special
 */
#define bdmaStreamSetMemory0(stp, addr) {                                   \
  (stp)->channel->CM0AR  = (uint32_t)(addr);                                \
}

/**
 * @brief   Associates a memory destination to a BDMA stream.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 * @param[in] addr      value to be written in the CM1AR register
 *
 * @special
 */
#define bdmaStreamSetMemory1(stp, addr) {                                   \
  (stp)->channel->CM1AR  = (uint32_t)(addr);                                \
}

/**
 * @brief   Alias of @p bdmaStreamSetMemory0() for compatibility.
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 * @param[in] addr      value to be written in the CM0AR register
 *
 * @special
 */
#define bdmaStreamSetMemory(stp, addr)  bdmaStreamSetMemory0(stp, addr)

/**
 * @brief   Sets the number of transfers to be performed.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 * @param[in] size      value to be written in the CNDTR register
 *
 * @special
 */
#define bdmaStreamSetTransactionSize(stp, size) {                           \
  (stp)->channel->CNDTR  = (uint32_t)(size);                                \
}

/**
 * @brief   Returns the number of transfers to be performed.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 * @return              The number of transfers to be performed.
 *
 * @special
 */
#define bdmaStreamGetTransactionSize(stp) ((size_t)((stp)->channel->CNDTR))

/**
 * @brief   Programs the stream mode settings.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 * @param[in] mode      value to be written in the CCR register
 *
 * @special
 */
#define bdmaStreamSetMode(stp, mode) {                                      \
  (stp)->channel->CCR  = (uint32_t)(mode);                                  \
}

/**
 * @brief   BDMA stream enable.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 *
 * @special
 */
#define bdmaStreamEnable(stp) {                                             \
  (stp)->channel->CCR |= STM32_BDMA_CR_EN;                                  \
}

/**
 * @brief   BDMA stream disable.
 * @details The function disables the specified stream and then clears any
 *          pending interrupt.
 * @note    This function can be invoked in both ISR or thread context.
 * @note    Interrupts enabling flags are set to zero after this call, see
 *          bug 3607518.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 *
 * @special
 */
#define bdmaStreamDisable(stp) {                                            \
  (stp)->channel->CCR &= ~(STM32_BDMA_CR_TCIE | STM32_BDMA_CR_HTIE |        \
                           STM32_BDMA_CR_TEIE | STM32_BDMA_CR_EN);          \
  bdmaStreamClearInterrupt(stp);                                            \
}

/**
 * @brief   BDMA stream interrupt sources clear.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 *
 * @special
 */
#define bdmaStreamClearInterrupt(stp) {                                     \
  (stp)->bdma->IFCR = STM32_BDMA_ISR_MASK << (stp)->shift;                  \
}

/**
 * @brief   Starts a memory to memory operation using the specified stream.
 * @note    The default transfer data mode is "byte to byte" but it can be
 *          changed by specifying extra options in the @p mode parameter.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 * @param[in] mode      value to be written in the CCR register, this value
 *                      is implicitly ORed with:
 *                      - @p STM32_BDMA_CR_MINC
 *                      - @p STM32_BDMA_CR_PINC
 *                      - @p STM32_BDMA_CR_DIR_M2M
 *                      - @p STM32_BDMA_CR_EN
 *                      .
 * @param[in] src       source address
 * @param[in] dst       destination address
 * @param[in] n         number of data units to copy
 */
#define bdmaStartMemCopy(stp, mode, src, dst, n) {                          \
  bdmaStreamSetPeripheral(stp, src);                                        \
  bdmaStreamSetMemory0(stp, dst);                                           \
  bdmaStreamSetTransactionSize(stp, n);                                     \
  bdmaStreamSetMode(stp, (mode) |                                           \
                         STM32_BDMA_CR_MINC | STM32_BDMA_CR_PINC |          \
                         STM32_BDMA_CR_DIR_M2M | STM32_BDMA_CR_EN);         \
}

/**
 * @brief   Polled wait for BDMA transfer end.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 */
#define bdmaWaitCompletion(stp) {                                           \
  while ((stp)->channel->CNDTR > 0U)                                        \
    ;                                                                       \
  bdmaStreamDisable(stp);                                                   \
}

/**
 * @brief   BDMA stream current target.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p bdmaStreamAllocate().
 * @post    After use the stream can be released using @p bdmaStreamRelease().
 *
 * @param[in] stp       pointer to an @p stm32_bdma_stream_t structure
 * @return              Current memory target index.
 *
 * @special
 */
#define bdmaStreamGetCurrentTarget(stp)                                     \
  (((stp)->channel->CCR >> BDMA_CCR_CT_Pos) & 1U)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern const stm32_bdma_stream_t _stm32_bdma_streams[STM32_BDMA_STREAMS];
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void bdmaInit(void);
  const stm32_bdma_stream_t *bdmaStreamAllocI(uint32_t id,
                                              uint32_t priority,
                                              stm32_bdmaisr_t func,
                                              void *param);
  const stm32_bdma_stream_t *bdmaStreamAlloc(uint32_t id,
                                             uint32_t priority,
                                             stm32_bdmaisr_t func,
                                             void *param);
  void bdmaStreamFreeI(const stm32_bdma_stream_t *stp);
  void bdmaStreamFree(const stm32_bdma_stream_t *stp);
  void bdmaSetRequestSource(const stm32_bdma_stream_t *stp, uint32_t per);
#ifdef __cplusplus
}
#endif

#endif /* STM32_BDMA_H */

/** @} */
