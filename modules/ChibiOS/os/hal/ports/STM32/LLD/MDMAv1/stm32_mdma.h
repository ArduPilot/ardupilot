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
 * @file    MDMAv1/stm32_mdma.h
 * @brief   MDMA helper driver header.
 *
 * @addtogroup STM32_MDMA
 * @{
 */

#ifndef STM32_MDMA_H
#define STM32_MDMA_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Total number of MDMA streams.
 */
#define STM32_MDMA_CHANNELS         16U

/**
 * @brief   Mask of the ISR bits passed to the MDMA callback functions.
 */
#define STM32_MDMA_ISR_MASK         0x1FU

/**
 * @brief   Checks if a MDMA priority is within the valid range.
 * @param[in] prio      MDMA priority
 *
 * @retval              The check result.
 * @retval false        invalid MDMA priority.
 * @retval true         correct MDMA priority.
 */
#define STM32_MDMA_IS_VALID_PRIORITY(prio) (((prio) >= 0U) && ((prio) <= 3U))

/**
 * @brief   Checks if a MDMA channel id is within the valid range.
 *
 * @param[in] id        MDMA channel id
 * @retval              The check result.
 * @retval false        invalid MDMA channel.
 * @retval true         correct MDMA channel.
 */
#define STM32_MDMA_IS_VALID_CHANNEL(id) (((id) >= 0U) &&                    \
                                         ((id) <= STM32_MDMA_CHANNELS))

/**
 * @brief   Special stream identifier
 */
#define STM32_MDMA_CHANNEL_ID_ANY   STM32_MDMA_CHANNELS

/**
 * @name    CISR register constants
 * @{
 */
#define STM32_MDMA_CISR_TEIF        (1U << 0)
#define STM32_MDMA_CISR_CTCIF       (1U << 1)
#define STM32_MDMA_CISR_BRTIF       (1U << 2)
#define STM32_MDMA_CISR_BTIF        (1U << 3)
#define STM32_MDMA_CISR_TCIF        (1U << 4)
#define STM32_MDMA_CISR_CRQA        (1U << 16)
/** @} */

/**
 * @name    CIFCR register constants
 * @{
 */
#define STM32_MDMA_CIFCR_CTEIF      (1U << 0)
#define STM32_MDMA_CIFCR_CCTCIF     (1U << 1)
#define STM32_MDMA_CIFCR_CBRTIF     (1U << 2)
#define STM32_MDMA_CIFCR_CBTIF      (1U << 3)
#define STM32_MDMA_CIFCR_CTCIF      (1U << 4)
/** @} */

/**
 * @name    CESR register constants
 * @{
 */
#define STM32_MDMA_CESR_TEA_MASK    (127U << 0)
#define STM32_MDMA_CESR_TED         (1U << 7)
#define STM32_MDMA_CESR_TELD        (1U << 8)
#define STM32_MDMA_CESR_TEMD        (1U << 9)
#define STM32_MDMA_CESR_ASE         (1U << 10)
#define STM32_MDMA_CESR_BSE         (1U << 11)
/** @} */

/**
 * @name    CCR register constants
 * @{
 */
#define STM32_MDMA_CCR_RESET_VALUE  0x00000000U
#define STM32_MDMA_CCR_EN           (1U << 0)
#define STM32_MDMA_CCR_TEIE         (1U << 1)
#define STM32_MDMA_CCR_CTCIE        (1U << 2)
#define STM32_MDMA_CCR_BRTIE        (1U << 3)
#define STM32_MDMA_CCR_BTIE         (1U << 4)
#define STM32_MDMA_CCR_TCIE         (1U << 5)
#define STM32_MDMA_CCR_PL_MASK      (3U << 6)
#define STM32_MDMA_CCR_PL(n)        ((n) << 6)
#define STM32_MDMA_CCR_BEX          (1U << 12)
#define STM32_MDMA_CCR_HEX          (1U << 13)
#define STM32_MDMA_CCR_WEX          (1U << 14)
#define STM32_MDMA_CCR_SWRQ         (1U << 16)
/** @} */

/**
 * @name    CTCR register constants
 * @{
 */
#define STM32_MDMA_CTCR_RESET_VALUE     0x00000000U

#define STM32_MDMA_CTCR_SINC_MASK       (3U << 0)
#define STM32_MDMA_CTCR_SINC(n)         ((n) << 0)
#define STM32_MDMA_CTCR_SINC_FIXED      STM32_MDMA_CTCR_SINC(0U)
#define STM32_MDMA_CTCR_SINC_INC        STM32_MDMA_CTCR_SINC(2U)
#define STM32_MDMA_CTCR_SINC_DEC        STM32_MDMA_CTCR_SINC(3U)

#define STM32_MDMA_CTCR_DINC_MASK       (3U << 2)
#define STM32_MDMA_CTCR_DINC(n)         ((n) << 2)
#define STM32_MDMA_CTCR_DINC_FIXED      STM32_MDMA_CTCR_DINC(0U)
#define STM32_MDMA_CTCR_DINC_INC        STM32_MDMA_CTCR_DINC(2U)
#define STM32_MDMA_CTCR_DINC_DEC        STM32_MDMA_CTCR_DINC(3U)

#define STM32_MDMA_CTCR_SSIZE_MASK      (3U << 4)
#define STM32_MDMA_CTCR_SSIZE(n)        ((n) << 4)
#define STM32_MDMA_CTCR_SSIZE_BYTE      STM32_MDMA_CTCR_SSIZE(0U)
#define STM32_MDMA_CTCR_SSIZE_HALF      STM32_MDMA_CTCR_SSIZE(1U)
#define STM32_MDMA_CTCR_SSIZE_WORD      STM32_MDMA_CTCR_SSIZE(2U)
#define STM32_MDMA_CTCR_SSIZE_DWORD     STM32_MDMA_CTCR_SSIZE(3U)

#define STM32_MDMA_CTCR_DSIZE_MASK      (3U << 6)
#define STM32_MDMA_CTCR_DSIZE(n)        ((n) << 6)
#define STM32_MDMA_CTCR_DSIZE_BYTE      STM32_MDMA_CTCR_DSIZE(0U)
#define STM32_MDMA_CTCR_DSIZE_HALF      STM32_MDMA_CTCR_DSIZE(1U)
#define STM32_MDMA_CTCR_DSIZE_WORD      STM32_MDMA_CTCR_DSIZE(2U)
#define STM32_MDMA_CTCR_DSIZE_DWORD     STM32_MDMA_CTCR_DSIZE(3U)

#define STM32_MDMA_CTCR_SINCOS_MASK     (3U << 8)
#define STM32_MDMA_CTCR_SINCOS(n)       ((n) << 8)
#define STM32_MDMA_CTCR_SINCOS_BYTE     STM32_MDMA_CTCR_SINCOS(0U)
#define STM32_MDMA_CTCR_SINCOS_HALF     STM32_MDMA_CTCR_SINCOS(1U)
#define STM32_MDMA_CTCR_SINCOS_WORD     STM32_MDMA_CTCR_SINCOS(2U)
#define STM32_MDMA_CTCR_SINCOS_DWORD    STM32_MDMA_CTCR_SINCOS(3U)

#define STM32_MDMA_CTCR_DINCOS_MASK     (3U << 10)
#define STM32_MDMA_CTCR_DINCOS(n)       ((n) << 10)
#define STM32_MDMA_CTCR_DINCOS_BYTE     STM32_MDMA_CTCR_DINCOS(0U)
#define STM32_MDMA_CTCR_DINCOS_HALF     STM32_MDMA_CTCR_DINCOS(1U)
#define STM32_MDMA_CTCR_DINCOS_WORD     STM32_MDMA_CTCR_DINCOS(2U)
#define STM32_MDMA_CTCR_DINCOS_DWORD    STM32_MDMA_CTCR_DINCOS(3U)

#define STM32_MDMA_CTCR_SBURST_MASK     (7U << 12)
#define STM32_MDMA_CTCR_SBURST(n)       ((n) << 12)
#define STM32_MDMA_CTCR_SBURST_1        STM32_MDMA_CTCR_SBURST(0U)
#define STM32_MDMA_CTCR_SBURST_2        STM32_MDMA_CTCR_SBURST(1U)
#define STM32_MDMA_CTCR_SBURST_4        STM32_MDMA_CTCR_SBURST(2U)
#define STM32_MDMA_CTCR_SBURST_8        STM32_MDMA_CTCR_SBURST(3U)
#define STM32_MDMA_CTCR_SBURST_16       STM32_MDMA_CTCR_SBURST(4U)
#define STM32_MDMA_CTCR_SBURST_32       STM32_MDMA_CTCR_SBURST(5U)
#define STM32_MDMA_CTCR_SBURST_64       STM32_MDMA_CTCR_SBURST(6U)
#define STM32_MDMA_CTCR_SBURST_128      STM32_MDMA_CTCR_SBURST(7U)

#define STM32_MDMA_CTCR_DBURST_MASK     (7U << 15)
#define STM32_MDMA_CTCR_DBURST(n)       ((n) << 15)
#define STM32_MDMA_CTCR_DBURST_1        STM32_MDMA_CTCR_DBURST(0U)
#define STM32_MDMA_CTCR_DBURST_2        STM32_MDMA_CTCR_DBURST(1U)
#define STM32_MDMA_CTCR_DBURST_4        STM32_MDMA_CTCR_DBURST(2U)
#define STM32_MDMA_CTCR_DBURST_8        STM32_MDMA_CTCR_DBURST(3U)
#define STM32_MDMA_CTCR_DBURST_16       STM32_MDMA_CTCR_DBURST(4U)
#define STM32_MDMA_CTCR_DBURST_32       STM32_MDMA_CTCR_DBURST(5U)
#define STM32_MDMA_CTCR_DBURST_64       STM32_MDMA_CTCR_DBURST(6U)
#define STM32_MDMA_CTCR_DBURST_128      STM32_MDMA_CTCR_DBURST(7U)

#define STM32_MDMA_CTCR_TLEN_MASK       (127U << 18)
#define STM32_MDMA_CTCR_TLEN(n)         ((n) << 18)

#define STM32_MDMA_CTCR_PKE             (1U << 25)

#define STM32_MDMA_CTCR_PAM_MASK        (3U << 26)
#define STM32_MDMA_CTCR_PAM(n)          ((n) << 26)
#define STM32_MDMA_CTCR_PAM_RIGHT       STM32_MDMA_CTCR_PAM(0U)
#define STM32_MDMA_CTCR_PAM_RIGHT_SE    STM32_MDMA_CTCR_PAM(1U)
#define STM32_MDMA_CTCR_PAM_LEFT        STM32_MDMA_CTCR_PAM(2U)

#define STM32_MDMA_CTCR_TRGM_MASK       (3U << 28)
#define STM32_MDMA_CTCR_TRGM(n)         ((n) << 28)
#define STM32_MDMA_CTCR_TRGM_BUFFER     STM32_MDMA_CTCR_TRGM(0U)
#define STM32_MDMA_CTCR_TRGM_BLOCK      STM32_MDMA_CTCR_TRGM(1U)
#define STM32_MDMA_CTCR_TRGM_REP_BLOCK  STM32_MDMA_CTCR_TRGM(2U)
#define STM32_MDMA_CTCR_TRGM_WHOLE      STM32_MDMA_CTCR_TRGM(3U)

#define STM32_MDMA_CTCR_SWRM            (1U << 30)

#define STM32_MDMA_CTCR_BWM_MASK        (1U << 31)
#define STM32_MDMA_CTCR_BWM_NON_BUFF    (0U << 31)
#define STM32_MDMA_CTCR_BWM_BUFF        (1U << 31)
/** @} */

/**
 * @name    BNDTR register constants
 * @{
 */
#define STM32_MDMA_CBNDTR_BNDT_MASK     (0x1FFFFU << 0)
#define STM32_MDMA_CBNDTR_BNDT(n)       ((n) << 0)
#define STM32_MDMA_CBNDTR_BRSUM         (1U << 18)
#define STM32_MDMA_CBNDTR_BRDUM         (1U << 19)
#define STM32_MDMA_CBNDTR_BRC_MASK      (0xFFFU << 20)
#define STM32_MDMA_CBNDTR_BRC(n)        ((n) << 20)
/** @} */

/**
 * @name    CBRUR register constants
 * @{
 */
#define STM32_MDMA_CBRUR_SUV_MASK       (0xFFFFU << 0)
#define STM32_MDMA_CBRUR_SUV(n)         ((n) << 0)
#define STM32_MDMA_CBRUR_DUV_MASK       (0xFFFFU << 16)
#define STM32_MDMA_CBRUR_DUV(n)         ((n) << 16)
/** @} */

/**
 * @name    CTBR register constants
 * @{
 */
#define STM32_MDMA_CTBR_TSEL_MASK       (0x3FU << 0)
#define STM32_MDMA_CTBR_TSEL(n)         ((n) << 0)
#define STM32_MDMA_CTBR_TSEL_SBUS       (1U << 16)
#define STM32_MDMA_CTBR_TSEL_DBUS       (1U << 17)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if !defined(STM32_MDMA_HANDLER)
#error "STM32_MDMA_HANDLER missing in registry"
#endif

#if !defined(STM32_MDMA_NUMBER)
#error "STM32_MDMA_NUMBER missing in registry"
#endif

/* Priority settings checks.*/
#if !defined(STM32_IRQ_MDMA_PRIORITY)
#error "STM32_IRQ_MDMA_PRIORITY not defined in mcuconf.h"
#endif

#if !OSAL_IRQ_IS_VALID_PRIORITY(STM32_IRQ_MDMA_PRIORITY)
#error "Invalid IRQ priority assigned to STM32_IRQ_MDMA_PRIORITY"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   STM32 MDMA ISR function type.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] flags     content of the CISR register in the lower 16 bits,
 *                      content of the CESR register in the upper 16 bits
 */
typedef void (*stm32_mdmaisr_t)(void *p, uint32_t flags);

/**
 * @brief   STM32 MDMA stream descriptor structure.
 */
typedef struct {
  /**
   * @brief Associated MDMA channel.
   */
  MDMA_Channel_TypeDef  *channel;
  /**
   * @brief   MDMA callback function.
   */
  stm32_mdmaisr_t       func;
  /**
   * @brief   MDMA callback parameter.
   */
  void                  *param;
} stm32_mdma_channel_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Associates a source address to a MDMA stream.
 * @pre     The stream must have been allocated using @p mdmaChannelAlloc().
 * @post    After use the stream can be released using @p mdmaChannelFree().
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 * @param[in] addr      value to be written in the CSAR register
 *
 * @xclass
 */
#define mdmaChannelSetSourceX(mdmachp, addr) do {                           \
  (mdmachp)->channel->CSAR  = (uint32_t)(addr);                             \
} while (0)

/**
 * @brief   Associates a memory destination to a MDMA stream.
 * @pre     The stream must have been allocated using @p mdmaChannelAlloc().
 * @post    After use the stream can be released using @p mdmaChannelFree().
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 * @param[in] addr      value to be written in the CDAR register
 *
 * @xclass
 */
#define mdmaChannelSetDestinationX(mdmachp, addr) do {                      \
  (mdmachp)->channel->CDAR  = (uint32_t)(addr);                             \
} while (0)

/**
 * @brief   Sets parameters related to the transaction size.
 * @pre     The stream must have been allocated using @p mdmaChannelAlloc().
 * @post    After use the stream can be released using @p mdmaChannelFree().
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 * @param[in] size      number of bytes per block
 * @param[in] n         number of blocks repetitions
 * @param[in] opt       other option bits for the CBNDTR register
 *
 * @xclass
 */
#define mdmaChannelSetTransactionSizeX(mdmachp, size, n, opt) do {          \
  (mdmachp)->channel->CBNDTR  = (uint32_t)STM32_MDMA_CBNDTR_BNDT(size) |    \
                                (uint32_t)STM32_MDMA_CBNDTR_BRC(n) |        \
                                (uint32_t)opt;                              \
} while (0)

/**
 * @brief   Programs the stream mode settings.
 * @pre     The stream must have been allocated using @p mdmaChannelAlloc().
 * @post    After use the stream can be released using @p mdmaChannelFree().
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 * @param[in] ctcr      value to be written in the CTCR register
 * @param[in] ccr       value to be written in the CCR register
 *
 * @xclass
 */
#define mdmaChannelSetModeX(mdmachp, ctcr, ccr) do {                        \
  (mdmachp)->channel->CTCR = (uint32_t)(ctcr);                              \
  (mdmachp)->channel->CCR  = (uint32_t)(ccr);                               \
} while (0)

/**
 * @brief   Programs the trigger mode settings.
 * @pre     The stream must have been allocated using @p mdmaChannelAlloc().
 * @post    After use the stream can be released using @p mdmaChannelFree().
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 * @param[in] tsel      value to be written in the CTBR register
 *
 * @xclass
 */
#define mdmaChannelSetTrigModeX(mdmachp, tsel) do {                         \
  (mdmachp)->channel->CTBR = STM32_MDMA_CTBR_TSEL(tsel);                    \
} while (0)

/**
 * @brief   MDMA stream enable.
 * @pre     The stream must have been allocated using @p mdmaChannelAlloc().
 * @post    After use the stream can be released using @p mdmaChannelFree().
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 *
 * @xclass
 */
#define mdmaChannelEnableX(mdmachp) do {                                    \
  (mdmachp)->channel->CCR |= STM32_MDMA_CCR_EN;                             \
} while (0)

/**
 * @brief   Channel enable check.
 * @pre     The stream must have been allocated using @p mdmaChannelAlloc().
 * @post    After use the stream can be released using @p mdmaChannelFree().
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 */
#define mdmaChannelIsEnabled(mdmachp)                                       \
  (((mdmachp)->channel->CCR & STM32_MDMA_CCR_EN) != 0U)

/**
 * @brief   MDMA stream interrupt sources clear.
 * @pre     The stream must have been allocated using @p mdmaChannelAlloc().
 * @post    After use the stream can be released using @p mdmaChannelFree().
 *
 * @param[in] mdmachp   pointer to a stm32_mdma_channel_t structure
 *
 * @xclass
 */
#define mdmaChannelClearInterruptX(mdmachp) do {                            \
  (mdmachp)->channel->CIFCR = (STM32_MDMA_CIFCR_CTEIF  |                    \
                               STM32_MDMA_CIFCR_CBRTIF |                    \
                               STM32_MDMA_CIFCR_CBRTIF |                    \
                               STM32_MDMA_CIFCR_CCTCIF |                    \
                               STM32_MDMA_CIFCR_CTEIF);                     \
} while (0)

/**
 * @brief   MDMA IRQ enable.
 */
#define mdma_irq_init()                                                     \
  nvicEnableVector(STM32_MDMA_NUMBER, STM32_IRQ_MDMA_PRIORITY)

/**
 * @brief   MDMA IRQ disable.
 */
#define mdma_irq_deinit()                                                     \
  nvicDisableVector(STM32_MDMA_NUMBER)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void mdmaInit(void);
  const stm32_mdma_channel_t *mdmaChannelAllocI(uint32_t id,
                                                stm32_mdmaisr_t func,
                                                void *param);
  const stm32_mdma_channel_t *mdmaChannelAlloc(uint32_t id,
                                               stm32_mdmaisr_t func,
                                               void *param);
  void mdmaChannelFreeI(const stm32_mdma_channel_t *mdmachp);
  void mdmaChannelFree(const stm32_mdma_channel_t *mdmachp);
  void mdmaChannelDisableX(const stm32_mdma_channel_t *mdmachp);
#ifdef __cplusplus
}
#endif

#endif /* STM32_MDMA_H */

/** @} */
