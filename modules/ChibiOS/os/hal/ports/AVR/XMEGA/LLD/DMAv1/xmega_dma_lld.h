/*
    ChibiOS - Copyright (C) 2016..2018 Theodore Ateba

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
 * @file    DMAv1/xmega_dma_lld.h
 * @brief   AVR XMEGA DMA low level driver header file.
 *
 * @addtogroup DMA
 * @{
 */

#ifndef XMEGA_DMA_LLD_H
#define XMEGA_DMA_LLD_H

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief   Programmable channel priority
 */

/**
 * @brief   DMA channels index
 */
typedef enum {
  DMA_CHANNEL0 = 0, /**< DMA channel 0. */
  DMA_CHANNEL1 = 1, /**< DMA channel 1. */
  DMA_CHANNEL2 = 2, /**< DMA channel 2. */
  DMA_CHANNEL3 = 3, /**< DMA channel 3. */
} dmachannelid_t;

/**
 * @brief   DMA possible addressing modes.
 */
typedef enum {
  DMA_ADDRMODE_STATIC = 0, /* Static addressing mode.       */
  DMA_ADDRMODE_INCRE  = 1, /* Incremental addressing mode.  */
  DMA_ADDRMODE_DECRE  = 2, /* Decremental addressing mode.  */
} dmaaddrmode_t;

/**
 * @brief   DMA possible double buffer modes settings.
 */
typedef enum {
  DMA_DBUFMODE_DISABLE  = 0, /* No double buffer enabled.                   */
  DMA_DBUFMODE_CH01     = 1, /* Double buffer enabled on channel01.         */
  DMA_DBUFMODE_CH23     = 2, /* Double buffer enabled on channel23.         */
  DMA_DBUFMODE_CH01CH23 = 3, /* Double buffer enabled on channe01 and 2/3.  */
} dmadbufmode_t;

/**
 * @brief   DMA possible channel burst mode.
 */
typedef enum {
  DMA_BURSTMODE_1BYTE = 0,  /* Burst mode is 1 byte.                        */
  DMA_BURSTMODE_2BYTE = 1,  /* Burst mode is 2 byte.                        */
  DMA_BURSTMODE_4BYTE = 2,  /* Burst mode is 4 byte.                        */
  DMA_BURSTMODE_8BYTE = 3,  /* Burst mode is 8 byte.                        */
} dmachannelburstmode_t;

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
void damReset(void);
void dmaConfigDoubleBuffering(DMA_DBUFMODE_t dbufMode);
void dmaSetPriority(DMA_PRIMODE_t priMode);
uint8_t dmaChannelIsOngoing(DMA_CH_t * channel);
uint8_t dmaIsOngoing(void);
uint8_t dmaChannelIsPending(DMA_CH_t * channel);
uint8_t dmaIsPending(void);
uint8_t dmaReturnStatusNonBlocking(DMA_CH_t * channel);
uint8_t dmaReturnStatusBlocking(DMA_CH_t * channel);

void dmaChannelEnable(DMA_CH_t * channel);
void dmaChannelDisable(DMA_CH_t * channel);
void dmaChannelReset(DMA_CH_t * channel);
void dmaSetIntLevel(DMA_CH_t * channel,
                      DMA_CH_TRNINTLVL_t transferInt,
                      DMA_CH_ERRINTLVL_t errorInt);
void dmaSetupBlock(DMA_CH_t * channel,
                     const void * srcAddr,
                     DMA_CH_SRCRELOAD_t srcReload,
                     DMA_CH_SRCDIR_t srcDirection,
                     void * destAddr,
                     DMA_CH_DESTRELOAD_t destReload,
                     DMA_CH_DESTDIR_t destDirection,
                     uint16_t blockSize,
                     DMA_CH_BURSTLEN_t burstMode,
                     uint8_t repeatCount,
                     bool useRepeat);
void dmaEnableSingleShot(DMA_CH_t * channel);
void dmaDisableSingleShot(DMA_CH_t * channel);
void dmaSetTriggerSource(DMA_CH_t * channel, uint8_t trigger);
void dmaStartTransfer(DMA_CH_t * channel);

#ifdef __cplusplus
}
#endif
//#endif /* HAL_USE_DMA */

#endif /* XMEGA_DMA_LLD_H */

/** @} */
