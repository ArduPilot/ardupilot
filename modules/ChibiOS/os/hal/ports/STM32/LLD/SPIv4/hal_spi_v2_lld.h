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
 * @file    SPIv4/hal_spi_v2_lld.h
 * @brief   STM32 SPI (v2) subsystem low level driver header.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef HAL_SPI_V2_LLD_H
#define HAL_SPI_V2_LLD_H

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#if !defined(__SPI_DISABLE_CIRCULAR)
/**
 * @brief   Circular mode support flag.
 */
#define SPI_SUPPORTS_CIRCULAR           TRUE
#else
#define SPI_SUPPORTS_CIRCULAR           FALSE
#endif

#if !defined(__SPI_DISABLE_SLAVE)
/**
 * @brief   Slave mode support flag.
 */
#define SPI_SUPPORTS_SLAVE_MODE         TRUE
#else
#define SPI_SUPPORTS_SLAVE_MODE         FALSE
#endif

/**
 * @name    Register helpers not found in ST headers
 * @{
 */
#define SPI_CFG1_MBR_VALUE(n)           ((n) << SPI_CFG1_MBR_Pos)
#define SPI_CFG1_MBR_DIV2               SPI_CFG1_MBR_VALUE(0)
#define SPI_CFG1_MBR_DIV4               SPI_CFG1_MBR_VALUE(1)
#define SPI_CFG1_MBR_DIV8               SPI_CFG1_MBR_VALUE(2)
#define SPI_CFG1_MBR_DIV16              SPI_CFG1_MBR_VALUE(3)
#define SPI_CFG1_MBR_DIV32              SPI_CFG1_MBR_VALUE(4)
#define SPI_CFG1_MBR_DIV64              SPI_CFG1_MBR_VALUE(5)
#define SPI_CFG1_MBR_DIV128             SPI_CFG1_MBR_VALUE(6)
#define SPI_CFG1_MBR_DIV256             SPI_CFG1_MBR_VALUE(7)
#define SPI_CFG1_CRCSIZE_VALUE(n)       ((n) << SPI_CFG1_CRCSIZE_Pos)
#define SPI_CFG1_UDRDET_VALUE(n)        ((n) << SPI_CFG1_UDRDET_Pos)
#define SPI_CFG1_UDRCFG_VALUE(n)        ((n) << SPI_CFG1_UDRCFG_Pos)
#define SPI_CFG1_FTHLV_VALUE(n)         ((n) << SPI_CFG1_FTHLV_Pos)
#define SPI_CFG1_DSIZE_MASK             (0x1FU << SPI_CFG1_DSIZE_Pos)
#define SPI_CFG1_DSIZE_POS              SPI_CFG1_DSIZE_Pos
#define SPI_CFG1_DSIZE_VALUE(n)         ((n) << SPI_CFG1_DSIZE_POS)
#define SPI_CFG1_DSIZE_4BITS            SPI_CFG1_DSIZE_VALUE(3U)
#define SPI_CFG1_DSIZE_5BITS            SPI_CFG1_DSIZE_VALUE(4U)
#define SPI_CFG1_DSIZE_6BITS            SPI_CFG1_DSIZE_VALUE(5U)
#define SPI_CFG1_DSIZE_7BITS            SPI_CFG1_DSIZE_VALUE(6U)
#define SPI_CFG1_DSIZE_8BITS            SPI_CFG1_DSIZE_VALUE(7U)
#define SPI_CFG1_DSIZE_9BITS            SPI_CFG1_DSIZE_VALUE(8U)
#define SPI_CFG1_DSIZE_10BITS           SPI_CFG1_DSIZE_VALUE(9U)
#define SPI_CFG1_DSIZE_11BITS           SPI_CFG1_DSIZE_VALUE(10U)
#define SPI_CFG1_DSIZE_12BITS           SPI_CFG1_DSIZE_VALUE(11U)
#define SPI_CFG1_DSIZE_13BITS           SPI_CFG1_DSIZE_VALUE(12U)
#define SPI_CFG1_DSIZE_14BITS           SPI_CFG1_DSIZE_VALUE(13U)
#define SPI_CFG1_DSIZE_15BITS           SPI_CFG1_DSIZE_VALUE(14U)
#define SPI_CFG1_DSIZE_16BITS           SPI_CFG1_DSIZE_VALUE(15U)
#define SPI_CFG1_DSIZE_17BITS           SPI_CFG1_DSIZE_VALUE(16U)
#define SPI_CFG1_DSIZE_18BITS           SPI_CFG1_DSIZE_VALUE(17U)
#define SPI_CFG1_DSIZE_19BITS           SPI_CFG1_DSIZE_VALUE(18U)
#define SPI_CFG1_DSIZE_20BITS           SPI_CFG1_DSIZE_VALUE(19U)
#define SPI_CFG1_DSIZE_21BITS           SPI_CFG1_DSIZE_VALUE(20U)
#define SPI_CFG1_DSIZE_22BITS           SPI_CFG1_DSIZE_VALUE(21U)
#define SPI_CFG1_DSIZE_23BITS           SPI_CFG1_DSIZE_VALUE(22U)
#define SPI_CFG1_DSIZE_24BITS           SPI_CFG1_DSIZE_VALUE(23U)
#define SPI_CFG1_DSIZE_25BITS           SPI_CFG1_DSIZE_VALUE(24U)
#define SPI_CFG1_DSIZE_26BITS           SPI_CFG1_DSIZE_VALUE(25U)
#define SPI_CFG1_DSIZE_27BITS           SPI_CFG1_DSIZE_VALUE(26U)
#define SPI_CFG1_DSIZE_28BITS           SPI_CFG1_DSIZE_VALUE(27U)
#define SPI_CFG1_DSIZE_29BITS           SPI_CFG1_DSIZE_VALUE(28U)
#define SPI_CFG1_DSIZE_30BITS           SPI_CFG1_DSIZE_VALUE(29U)
#define SPI_CFG1_DSIZE_31BITS           SPI_CFG1_DSIZE_VALUE(30U)
#define SPI_CFG1_DSIZE_32BITS           SPI_CFG1_DSIZE_VALUE(31U)

#define SPI_CFG2_SP_VALUE(n)            ((n) << SPI_CFG2_SP_Pos)
#define SPI_CFG2_COMM_VALUE(n)          ((n) << SPI_CFG2_COMM_Pos)
#define SPI_CFG2_COMM_FULL_DUPLEX       SPI_CFG2_COMM_VALUE(0)
#define SPI_CFG2_COMM_TRANSMITTER       SPI_CFG2_COMM_VALUE(1)
#define SPI_CFG2_COMM_RECEIVER          SPI_CFG2_COMM_VALUE(2)
#define SPI_CFG2_COMM_HALF_DUPLEX       SPI_CFG2_COMM_VALUE(3)
#define SPI_CFG2_MIDI_VALUE(n)          ((n) << SPI_CFG2_MIDI_Pos)
#define SPI_CFG2_MSSI_VALUE(n)          ((n) << SPI_CFG2_MSSI_Pos)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   SPI1 driver enable switch.
 * @details If set to @p TRUE the support for SPI1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI1) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI1                  FALSE
#endif

/**
 * @brief   SPI2 driver enable switch.
 * @details If set to @p TRUE the support for SPI2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI2) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI2                  FALSE
#endif

/**
 * @brief   SPI3 driver enable switch.
 * @details If set to @p TRUE the support for SPI3 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI3) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI3                  FALSE
#endif

/**
 * @brief   SPI4 driver enable switch.
 * @details If set to @p TRUE the support for SPI4 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI4) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI4                  FALSE
#endif

/**
 * @brief   SPI5 driver enable switch.
 * @details If set to @p TRUE the support for SPI5 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI5) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI5                  FALSE
#endif

/**
 * @brief   SPI6 driver enable switch.
 * @details If set to @p TRUE the support for SPI6 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(STM32_SPI_USE_SPI6) || defined(__DOXYGEN__)
#define STM32_SPI_USE_SPI6                  FALSE
#endif

/**
 * @brief   Filler pattern used when there is nothing to transmit.
 */
#if !defined(STM32_SPI_FILLER_PATTERN) || defined(__DOXYGEN__)
#define STM32_SPI_FILLER_PATTERN            0xFFFFFFFFU
#endif

/**
 * @brief   SPI1 GPDMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX GPDMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(STM32_SPI_SPI1_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI2 GPDMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX GPDMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(STM32_SPI_SPI2_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI3 GPDMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX GPDMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(STM32_SPI_SPI3_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI3_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI4 GPDMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX GPDMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(STM32_SPI_SPI4_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI4_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI5 GPDMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX GPDMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(STM32_SPI_SPI5_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI5_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI6 GPDMA priority (0..3|lowest..highest).
 * @note    The priority level is used for both the TX and RX GPDMA channels but
 *          because of the channels ordering the RX channel has always priority
 *          over the TX channel.
 */
#if !defined(STM32_SPI_SPI6_DMA_PRIORITY) || defined(__DOXYGEN__)
#define STM32_SPI_SPI6_DMA_PRIORITY         1
#endif

/**
 * @brief   SPI GPDMA error hook.
 */
#if !defined(STM32_SPI_DMA_ERROR_HOOK) || defined(__DOXYGEN__)
#define STM32_SPI_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* Registry checks.*/
#if !defined(STM32_HAS_SPI1) || !defined(STM32_HAS_SPI2) ||                 \
    !defined(STM32_HAS_SPI3) || !defined(STM32_HAS_SPI4) ||                 \
    !defined(STM32_HAS_SPI5) || !defined(STM32_HAS_SPI6)
#error "STM32_HAS_SPIx not defined in registry"
#endif

/* IP instances check.*/
#if STM32_SPI_USE_SPI1 && !STM32_HAS_SPI1
#error "SPI1 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI2 && !STM32_HAS_SPI2
#error "SPI2 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI3 && !STM32_HAS_SPI3
#error "SPI3 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI4 && !STM32_HAS_SPI4
#error "SPI4 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI5 && !STM32_HAS_SPI5
#error "SPI5 not present in the selected device"
#endif

#if STM32_SPI_USE_SPI6 && !STM32_HAS_SPI6
#error "SPI6 not present in the selected device"
#endif

#if !STM32_SPI_USE_SPI1 && !STM32_SPI_USE_SPI2 && !STM32_SPI_USE_SPI3 &&    \
    !STM32_SPI_USE_SPI4 && !STM32_SPI_USE_SPI5 && !STM32_SPI_USE_SPI6
#error "SPI driver activated but no SPI peripheral assigned"
#endif

#if STM32_SPI_USE_SPI1
#if defined(STM32_SPI1_IS_USED)
#error "SPID1 requires SPI1 but it is already used"
#else
#define STM32_SPI1_IS_USED
#endif
#endif

#if STM32_SPI_USE_SPI2
#if defined(STM32_SPI2_IS_USED)
#error "SPID2 requires SPI2 but it is already used"
#else
#define STM32_SPI2_IS_USED
#endif
#endif

#if STM32_SPI_USE_SPI3
#if defined(STM32_SPI3_IS_USED)
#error "SPID3 requires SPI3 but it is already used"
#else
#define STM32_SPI3_IS_USED
#endif
#endif

#if STM32_SPI_USE_SPI4
#if defined(STM32_SPI4_IS_USED)
#error "SPID4 requires SPI4 but it is already used"
#else
#define STM32_SPI4_IS_USED
#endif
#endif

#if STM32_SPI_USE_SPI5
#if defined(STM32_SPI5_IS_USED)
#error "SPID5 requires SPI5 but it is already used"
#else
#define STM32_SPI5_IS_USED
#endif
#endif

#if STM32_SPI_USE_SPI6
#if defined(STM32_SPI6_IS_USED)
#error "SPID6 requires SPI6 but it is already used"
#else
#define STM32_SPI6_IS_USED
#endif
#endif

/* Check on the presence of the GPDMA channels settings in mcuconf.h.*/
#if STM32_SPI_USE_SPI1 && (!defined(STM32_SPI_SPI1_RX_DMA3_CHANNEL) ||     \
                           !defined(STM32_SPI_SPI1_TX_DMA3_CHANNEL))
#error "SPI1 GPDMA channels not defined"
#endif

#if STM32_SPI_USE_SPI2 && (!defined(STM32_SPI_SPI2_RX_DMA3_CHANNEL) ||     \
                           !defined(STM32_SPI_SPI2_TX_DMA3_CHANNEL))
#error "SPI2 GPDMA channels not defined"
#endif

#if STM32_SPI_USE_SPI3 && (!defined(STM32_SPI_SPI3_RX_DMA3_CHANNEL) ||     \
                           !defined(STM32_SPI_SPI3_TX_DMA3_CHANNEL))
#error "SPI3 GPDMA channels not defined"
#endif

#if STM32_SPI_USE_SPI4 && (!defined(STM32_SPI_SPI4_RX_DMA3_CHANNEL) ||     \
                           !defined(STM32_SPI_SPI4_TX_DMA3_CHANNEL))
#error "SPI4 GPDMA channels not defined"
#endif

#if STM32_SPI_USE_SPI5 && (!defined(STM32_SPI_SPI5_RX_DMA3_CHANNEL) ||     \
                           !defined(STM32_SPI_SPI5_TX_DMA3_CHANNEL))
#error "SPI5 GPDMA channels not defined"
#endif

#if STM32_SPI_USE_SPI6 && (!defined(STM32_SPI_SPI6_RX_DMA3_CHANNEL) ||     \
                           !defined(STM32_SPI_SPI6_TX_DMA3_CHANNEL))
#error "SPI6 GPDMA channels not defined"
#endif

/* Check on the validity of the assigned GPDMA channels.*/
#if STM32_SPI_USE_SPI1 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI1_RX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI1 RX"
#endif

#if STM32_SPI_USE_SPI1 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI1_TX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI1 TX"
#endif

#if STM32_SPI_USE_SPI2 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI2_RX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI2 RX"
#endif

#if STM32_SPI_USE_SPI2 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI2_TX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI2 TX"
#endif

#if STM32_SPI_USE_SPI3 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI3_RX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI3 RX"
#endif

#if STM32_SPI_USE_SPI3 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI3_TX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI3 TX"
#endif

#if STM32_SPI_USE_SPI4 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI4_RX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI4 RX"
#endif

#if STM32_SPI_USE_SPI4 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI4_TX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI4 TX"
#endif

#if STM32_SPI_USE_SPI5 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI5_RX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI5 RX"
#endif

#if STM32_SPI_USE_SPI5 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI5_TX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI5 TX"
#endif

#if STM32_SPI_USE_SPI6 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI6_RX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI6 RX"
#endif

#if STM32_SPI_USE_SPI6 &&                                                   \
    !STM32_DMA3_ARE_VALID_CHANNELS(STM32_SPI_SPI6_TX_DMA3_CHANNEL)
#error "Invalid GPDMA channel assigned to SPI6 TX"
#endif

#if STM32_SPI_USE_SPI1 &&                                                   \
    !STM32_DMA3_IS_VALID_PRIORITY(STM32_SPI_SPI1_DMA_PRIORITY)
#error "Invalid GPDMA priority assigned to SPI1"
#endif

#if STM32_SPI_USE_SPI2 &&                                                   \
    !STM32_DMA3_IS_VALID_PRIORITY(STM32_SPI_SPI2_DMA_PRIORITY)
#error "Invalid GPDMA priority assigned to SPI2"
#endif

#if STM32_SPI_USE_SPI3 &&                                                   \
    !STM32_DMA3_IS_VALID_PRIORITY(STM32_SPI_SPI3_DMA_PRIORITY)
#error "Invalid GPDMA priority assigned to SPI3"
#endif

#if STM32_SPI_USE_SPI4 &&                                                   \
    !STM32_DMA3_IS_VALID_PRIORITY(STM32_SPI_SPI4_DMA_PRIORITY)
#error "Invalid GPDMA priority assigned to SPI4"
#endif

#if STM32_SPI_USE_SPI5 &&                                                   \
    !STM32_DMA3_IS_VALID_PRIORITY(STM32_SPI_SPI5_DMA_PRIORITY)
#error "Invalid GPDMA priority assigned to SPI5"
#endif

#if STM32_SPI_USE_SPI6 &&                                                   \
    !STM32_DMA3_IS_VALID_PRIORITY(STM32_SPI_SPI6_DMA_PRIORITY)
#error "Invalid GPDMA priority assigned to SPI6"
#endif

#if STM32_SPI_USE_SPI1 || STM32_SPI_USE_SPI2 || STM32_SPI_USE_SPI3 ||       \
    STM32_SPI_USE_SPI4 || STM32_SPI_USE_SPI5 || STM32_SPI_USE_SPI6
#if !defined(STM32_DMA3_REQUIRED)
#define STM32_DMA3_REQUIRED
#endif
#endif

#if SPI_SELECT_MODE == SPI_SELECT_MODE_LLD
#error "SPI_SELECT_MODE_LLD not supported by this driver"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure containing DMA-accessible driver fields.
 */
typedef struct spi_dmabuf {
  /**
   * @brief   Sink for discarded data.
   */
  uint32_t                          rxsink;
  /**
   * @brief   Source for default TX pattern.
   */
  uint32_t                          txsource;
#if SPI_SUPPORTS_CIRCULAR || defined(__DOXYGEN__)
  /**
   * @brief   GPDMA link structure for circular mode RX channel.
   */
  uint32_t                          rxdar;
  /**
   * @brief   GPDMA link structure for circular mode TX channel.
   */
  uint32_t                          txsar;
#endif
} spi_dmabuf_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#define spi_lld_driver_fields                                               \
  /* Pointer to the SPIx registers block.*/                                 \
  SPI_TypeDef                       *spi;                                   \
  /* Receive GPDMA channel.*/                                               \
  const stm32_dma3_channel_t        *dmarx;                                 \
  /* Transmit GPDMA channel.*/                                              \
  const stm32_dma3_channel_t        *dmatx;                                 \
  /* DMA BNDT shift value.*/                                                \
  uint8_t                           dnshift;                                \
  /* DMA request line for RX.*/                                             \
  uint8_t                           dreqrx;                                 \
  /* DMA request line for TX.*/                                             \
  uint8_t                           dreqtx;                                 \
  /* DMA priority.*/                                                        \
  uint8_t                           dprio;                                  \
  /* DMA RX settings.*/                                                     \
  uint32_t                          dtr1rx;                                 \
  /* DMA TX settings.*/                                                     \
  uint32_t                          dtr1tx;                                 \
  /* DMA buffers.*/                                                         \
  spi_dmabuf_t                      *dbuf;

/**
 * @brief   Low level fields of the SPI configuration structure.
 */
#define spi_lld_config_fields                                               \
  /* SPI CFG1 register initialization data.*/                               \
  uint32_t                          cfg1;                                   \
  /* SPI CFG2 register initialization data.*/                               \
  uint32_t                          cfg2;                                   \
  /* DMA RX extra TR1 settings.*/                                           \
  uint32_t                          dtr1rx;                                 \
  /* DMA TX extra TR1 settings.*/                                           \
  uint32_t                          dtr1tx;                                 \
  /* DMA RX extra TR2 settings.*/                                           \
  uint32_t                          dtr2rx;                                 \
  /* DMA TX extra TR2 settings.*/                                           \
  uint32_t                          dtr2tx;

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if STM32_SPI_USE_SPI1 && !defined(__DOXYGEN__)
extern SPIDriver SPID1;
#endif

#if STM32_SPI_USE_SPI2 && !defined(__DOXYGEN__)
extern SPIDriver SPID2;
#endif

#if STM32_SPI_USE_SPI3 && !defined(__DOXYGEN__)
extern SPIDriver SPID3;
#endif

#if STM32_SPI_USE_SPI4 && !defined(__DOXYGEN__)
extern SPIDriver SPID4;
#endif

#if STM32_SPI_USE_SPI5 && !defined(__DOXYGEN__)
extern SPIDriver SPID5;
#endif

#if STM32_SPI_USE_SPI6 && !defined(__DOXYGEN__)
extern SPIDriver SPID6;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void spi_lld_init(void);
  msg_t spi_lld_start(SPIDriver *spip);
  void spi_lld_stop(SPIDriver *spip);
#if (SPI_SELECT_MODE == SPI_SELECT_MODE_LLD) || defined(__DOXYGEN__)
  void spi_lld_select(SPIDriver *spip);
  void spi_lld_unselect(SPIDriver *spip);
#endif
  msg_t spi_lld_ignore(SPIDriver *spip, size_t n);
  msg_t spi_lld_exchange(SPIDriver *spip, size_t n,
                         const void *txbuf, void *rxbuf);
  msg_t spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf);
  msg_t spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf);
  msg_t spi_lld_stop_transfer(SPIDriver *spip, size_t *sizep);
  uint32_t spi_lld_polled_exchange(SPIDriver *spip, uint32_t frame);
  void spi_lld_serve_interrupt(SPIDriver *spip);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SPI */

#endif /* HAL_SPI_V2_LLD_H */

/** @} */
