/*
    SPC5 HAL - Copyright (C) 2013 STMicroelectronics

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
 * @file    SPC5xx/spc5_edma.h
 * @brief   EDMA helper driver header.
 *
 * @addtogroup SPC5xx_EDMA
 * @{
 */

#ifndef _SPC5_EDMA_H_
#define _SPC5_EDMA_H_

#if SPC5_HAS_EDMA || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   EDMA channel allocation error.
 */
#define EDMA_ERROR                  -1

/**
 * @name    EDMA CR register definitions
 * @{
 */
#define EDMA_CR_CX                  (1U << 17)
#define EDMA_CR_ECX                 (1U << 16)
#define EDMA_CR_GRP3PRI_MASK        (3U << 14)
#define EDMA_CR_GRP3PRI(n)          ((n) << 14)
#define EDMA_CR_GRP2PRI_MASK        (3U << 12)
#define EDMA_CR_GRP2PRI(n)          ((n) << 12)
#define EDMA_CR_GRP1PRI_MASK        (3U << 10)
#define EDMA_CR_GRP1PRI(n)          ((n) << 10)
#define EDMA_CR_GRP0PRI_MASK        (3U << 8)
#define EDMA_CR_GRP0PRI(n)          ((n) << 8)
#define EDMA_CR_EMLM                (1U << 7)
#define EDMA_CR_CLM                 (1U << 6)
#define EDMA_CR_HALT                (1U << 5)
#define EDMA_CR_HOE                 (1U << 4)
#define EDMA_CR_ERGA                (1U << 3)
#define EDMA_CR_ERCA                (1U << 2)
#define EDMA_CR_EDBG                (1U << 1)
#define EDMA_CR_EBW                 (1U << 0)
/** @} */

/**
 * @name    EDMA mode constants
 * @{
 */
#define EDMA_TCD_MODE_START         (1U << 0)
#define EDMA_TCD_MODE_INT_END       (1U << 1)
#define EDMA_TCD_MODE_INT_HALF      (1U << 2)
#define EDMA_TCD_MODE_DREQ          (1U << 3)
#define EDMA_TCD_MODE_SG            (1U << 4)
#define EDMA_TCD_MODE_MELINK        (1U << 5)
#define EDMA_TCD_MODE_ACTIVE        (1U << 6)
#define EDMA_TCD_MODE_DONE          (1U << 7)
#define EDMA_TCD_MODE_MLINKCH_MASK  (63U << 8)
#define EDMA_TCD_MODE_MLINKCH(n)    ((uint32_t)(n) << 8)
#define EDMA_TCD_MODE_BWC_MASK      (3U << 14)
#define EDMA_TCD_MODE_BWC(n)        ((uint32_t)(n) << 14)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Default EDMA CR register initialization.
 */
#if !defined(SPC5_EDMA_CR_SETTING) || defined(__DOXYGEN__)
#define SPC5_EDMA_CR_SETTING                (EDMA_CR_GRP3PRI(3) |           \
                                             EDMA_CR_GRP2PRI(2) |           \
                                             EDMA_CR_GRP1PRI(1) |           \
                                             EDMA_CR_GRP0PRI(0) |           \
                                             EDMA_CR_EMLM       |           \
                                             EDMA_CR_ERGA)
#endif

/**
 * @brief   Static priorities for channels group 0.
 */
#if !defined(SPC5_EDMA_GROUP0_PRIORITIES) || defined(__DOXYGEN__)
#define SPC5_EDMA_GROUP0_PRIORITIES                                         \
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
#endif

/**
 * @brief   Static priorities for channels group 1.
 */
#if !defined(SPC5_EDMA_GROUP1_PRIORITIES) || defined(__DOXYGEN__)
#define SPC5_EDMA_GROUP1_PRIORITIES                                         \
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
#endif

/**
 * @brief   Static priorities for channels group 2.
 */
#if !defined(SPC5_EDMA_GROUP2_PRIORITIES) || defined(__DOXYGEN__)
#define SPC5_EDMA_GROUP2_PRIORITIES                                         \
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
#endif

/**
 * @brief   Static priorities for channels group 3.
 */
#if !defined(SPC5_EDMA_GROUP3_PRIORITIES) || defined(__DOXYGEN__)
#define SPC5_EDMA_GROUP3_PRIORITIES                                         \
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
#endif

/**
 * @brief   EDMA error handler IRQ priority.
 */
#if !defined(SPC5_EDMA_ERROR_IRQ_PRIO) || defined(__DOXYGEN__)
#define SPC5_EDMA_ERROR_IRQ_PRIO            12
#endif

/**
 * @brief   EDMA peripheral configuration when started.
 * @note    The default configuration is 1 (always run) in run mode and
 *          2 (only halt) in low power mode. The defaults of the run modes
 *          are defined in @p hal_lld.h.
 */
#if !defined(SPC5_EDMA_MUX_START_PCTL) || defined(__DOXYGEN__)
#define SPC5_EDMA_MUX_START_PCTL            (SPC5_ME_PCTL_RUN(1) |          \
                                             SPC5_ME_PCTL_LP(2))
#endif

/**
 * @brief   EDMA critical error handler, must not return.
 */
#if !defined(SPC5_EDMA_ERROR_HANDLER) || defined(__DOXYGEN__)
#define SPC5_EDMA_ERROR_HANDLER()           osalSysHalt("EDMA failure")
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of and eDMA channel number.
 */
typedef int32_t edma_channel_t;

/**
 * @brief   Type of an eDMA TCD.
 */
typedef struct {
  union {
    uint32_t              word[8];
  };
} edma_tcd_t;

/**
 * @brief   Type of an eDMA peripheral.
 */
typedef struct {

  union {
    vuint32_t R;
    struct {
      vuint32_t :14;
      vuint32_t CX :1;
      vuint32_t ECX :1;
      vuint32_t GRP3PRI :2;
      vuint32_t GRP2PRI :2;
      vuint32_t GRP1PRI :2;
      vuint32_t GRP0PRI :2;
      vuint32_t EMLM :1;
      vuint32_t CLM :1;
      vuint32_t HALT :1;
      vuint32_t HOE :1;
      vuint32_t ERGA :1;
      vuint32_t ERCA :1;
      vuint32_t EDBG :1;
      vuint32_t :1;
    } B;
  } CR; /* DMA Control Register @baseaddress + 0x0 */

  union {
    vuint32_t R;
    struct {
      vuint32_t VLD :1;
      vuint32_t :14;
      vuint32_t ECX :1;
      vuint32_t GPE :1;
      vuint32_t CPE :1;
      vuint32_t ERRCHN :6;
      vuint32_t SAE :1;
      vuint32_t SOE :1;
      vuint32_t DAE :1;
      vuint32_t DOE :1;
      vuint32_t NCE :1;
      vuint32_t SGE :1;
      vuint32_t SBE :1;
      vuint32_t DBE :1;
    } B;
  } ESR; /* Error Status Register @baseaddress + 0x4 */

  union {
    vuint32_t R;
    struct {
      vuint32_t ERQ63 :1;
      vuint32_t ERQ62 :1;
      vuint32_t ERQ61 :1;
      vuint32_t ERQ60 :1;
      vuint32_t ERQ59 :1;
      vuint32_t ERQ58 :1;
      vuint32_t ERQ57 :1;
      vuint32_t ERQ56 :1;
      vuint32_t ERQ55 :1;
      vuint32_t ERQ54 :1;
      vuint32_t ERQ53 :1;
      vuint32_t ERQ52 :1;
      vuint32_t ERQ51 :1;
      vuint32_t ERQ50 :1;
      vuint32_t ERQ49 :1;
      vuint32_t ERQ48 :1;
      vuint32_t ERQ47 :1;
      vuint32_t ERQ46 :1;
      vuint32_t ERQ45 :1;
      vuint32_t ERQ44 :1;
      vuint32_t ERQ43 :1;
      vuint32_t ERQ42 :1;
      vuint32_t ERQ41 :1;
      vuint32_t ERQ40 :1;
      vuint32_t ERQ39 :1;
      vuint32_t ERQ38 :1;
      vuint32_t ERQ37 :1;
      vuint32_t ERQ36 :1;
      vuint32_t ERQ35 :1;
      vuint32_t ERQ34 :1;
      vuint32_t ERQ33 :1;
      vuint32_t ERQ32 :1;
    } B;
  } ERQRH; /* DMA Enable Request Register High @baseaddress + 0x8*/

  union {
    vuint32_t R;
    struct {
      vuint32_t ERQ31 :1;
      vuint32_t ERQ30 :1;
      vuint32_t ERQ29 :1;
      vuint32_t ERQ28 :1;
      vuint32_t ERQ27 :1;
      vuint32_t ERQ26 :1;
      vuint32_t ERQ25 :1;
      vuint32_t ERQ24 :1;
      vuint32_t ERQ23 :1;
      vuint32_t ERQ22 :1;
      vuint32_t ERQ21 :1;
      vuint32_t ERQ20 :1;
      vuint32_t ERQ19 :1;
      vuint32_t ERQ18 :1;
      vuint32_t ERQ17 :1;
      vuint32_t ERQ16 :1;
      vuint32_t ERQ15 :1;
      vuint32_t ERQ14 :1;
      vuint32_t ERQ13 :1;
      vuint32_t ERQ12 :1;
      vuint32_t ERQ11 :1;
      vuint32_t ERQ10 :1;
      vuint32_t ERQ09 :1;
      vuint32_t ERQ08 :1;
      vuint32_t ERQ07 :1;
      vuint32_t ERQ06 :1;
      vuint32_t ERQ05 :1;
      vuint32_t ERQ04 :1;
      vuint32_t ERQ03 :1;
      vuint32_t ERQ02 :1;
      vuint32_t ERQ01 :1;
      vuint32_t ERQ00 :1;
    } B;
  } ERQRL; /* DMA Enable Request Register Low @baseaddress + 0xC*/

  union {
    vuint32_t R;
    struct {

      vuint32_t EEI63 :1;
      vuint32_t EEI62 :1;
      vuint32_t EEI61 :1;
      vuint32_t EEI60 :1;
      vuint32_t EEI59 :1;
      vuint32_t EEI58 :1;
      vuint32_t EEI57 :1;
      vuint32_t EEI56 :1;
      vuint32_t EEI55 :1;
      vuint32_t EEI54 :1;
      vuint32_t EEI53 :1;
      vuint32_t EEI52 :1;
      vuint32_t EEI51 :1;
      vuint32_t EEI50 :1;
      vuint32_t EEI49 :1;
      vuint32_t EEI48 :1;
      vuint32_t EEI47 :1;
      vuint32_t EEI46 :1;
      vuint32_t EEI45 :1;
      vuint32_t EEI44 :1;
      vuint32_t EEI43 :1;
      vuint32_t EEI42 :1;
      vuint32_t EEI41 :1;
      vuint32_t EEI40 :1;
      vuint32_t EEI39 :1;
      vuint32_t EEI38 :1;
      vuint32_t EEI37 :1;
      vuint32_t EEI36 :1;
      vuint32_t EEI35 :1;
      vuint32_t EEI34 :1;
      vuint32_t EEI33 :1;
      vuint32_t EEI32 :1;
    } B;
  } EEIRH; /* DMA Enable Error Interrupt Register High @baseaddress + 0x10*/

  union {
    vuint32_t R;
    struct {
      vuint32_t EEI31 :1;
      vuint32_t EEI30 :1;
      vuint32_t EEI29 :1;
      vuint32_t EEI28 :1;
      vuint32_t EEI27 :1;
      vuint32_t EEI26 :1;
      vuint32_t EEI25 :1;
      vuint32_t EEI24 :1;
      vuint32_t EEI23 :1;
      vuint32_t EEI22 :1;
      vuint32_t EEI21 :1;
      vuint32_t EEI20 :1;
      vuint32_t EEI19 :1;
      vuint32_t EEI18 :1;
      vuint32_t EEI17 :1;
      vuint32_t EEI16 :1;
      vuint32_t EEI15 :1;
      vuint32_t EEI14 :1;
      vuint32_t EEI13 :1;
      vuint32_t EEI12 :1;
      vuint32_t EEI11 :1;
      vuint32_t EEI10 :1;
      vuint32_t EEI09 :1;
      vuint32_t EEI08 :1;
      vuint32_t EEI07 :1;
      vuint32_t EEI06 :1;
      vuint32_t EEI05 :1;
      vuint32_t EEI04 :1;
      vuint32_t EEI03 :1;
      vuint32_t EEI02 :1;
      vuint32_t EEI01 :1;
      vuint32_t EEI00 :1;
    } B;
  } EEIRL; /* DMA Enable Error Interrupt Register Low @baseaddress + 0x14*/

  union {
    vuint8_t R;
    struct {
      vuint8_t NOP :1;
      vuint8_t SERQ :7;
    } B;
  } SERQR; /* DMA Set Enable Request Register @baseaddress + 0x18*/

  union {
    vuint8_t R;
    struct {
      vuint8_t NOP :1;
      vuint8_t CERQ :7;
    } B;
  } CERQR; /* DMA Clear Enable Request Register @baseaddress + 0x19*/

  union {
    vuint8_t R;
    struct {
      vuint8_t NOP :1;
      vuint8_t SEEI :7;
    } B;
  } SEEIR; /* DMA Set Enable Error Interrupt Register @baseaddress + 0x1A*/

  union {
    vuint8_t R;
    struct {
      vuint8_t NOP :1;
      vuint8_t CEEI :7;
    } B;
  } CEEIR; /* DMA Clear Enable Error Interrupt Register @baseaddress + 0x1B*/

  union {
    vuint8_t R;
    struct {
      vuint8_t NOP :1;
      vuint8_t CINT :7;
    } B;
  } CIRQR; /* DMA Clear Interrupt Request Register @baseaddress + 0x1C */

  union {
    vuint8_t R;
    struct {
      vuint8_t NOP :1;
      vuint8_t CERR :7;
    } B;
  } CER; /* DMA Clear error Register @baseaddress + 0x1D */

  union {
    vuint8_t R;
    struct {
      vuint8_t NOP :1;
      vuint8_t SSB :7;
    } B;
  } SSBR; /* Set Start Bit Register @baseaddress + 0x1E */

  union {
    vuint8_t R;
    struct {
      vuint8_t NOP :1;
      vuint8_t CDSB :7;
    } B;
  } CDSBR; /* Clear Done Status Bit Register @baseaddress + 0x1F */

  union {
    vuint32_t R;
    struct {
      vuint32_t INT63 :1;
      vuint32_t INT62 :1;
      vuint32_t INT61 :1;
      vuint32_t INT60 :1;
      vuint32_t INT59 :1;
      vuint32_t INT58 :1;
      vuint32_t INT57 :1;
      vuint32_t INT56 :1;
      vuint32_t INT55 :1;
      vuint32_t INT54 :1;
      vuint32_t INT53 :1;
      vuint32_t INT52 :1;
      vuint32_t INT51 :1;
      vuint32_t INT50 :1;
      vuint32_t INT49 :1;
      vuint32_t INT48 :1;
      vuint32_t INT47 :1;
      vuint32_t INT46 :1;
      vuint32_t INT45 :1;
      vuint32_t INT44 :1;
      vuint32_t INT43 :1;
      vuint32_t INT42 :1;
      vuint32_t INT41 :1;
      vuint32_t INT40 :1;
      vuint32_t INT39 :1;
      vuint32_t INT38 :1;
      vuint32_t INT37 :1;
      vuint32_t INT36 :1;
      vuint32_t INT35 :1;
      vuint32_t INT34 :1;
      vuint32_t INT33 :1;
      vuint32_t INT32 :1;
    } B;
  } IRQRH; /* DMA Interrupt Request High @baseaddress + 0x20 */

  union {
    vuint32_t R;
    struct {
      vuint32_t INT31 :1;
      vuint32_t INT30 :1;
      vuint32_t INT29 :1;
      vuint32_t INT28 :1;
      vuint32_t INT27 :1;
      vuint32_t INT26 :1;
      vuint32_t INT25 :1;
      vuint32_t INT24 :1;
      vuint32_t INT23 :1;
      vuint32_t INT22 :1;
      vuint32_t INT21 :1;
      vuint32_t INT20 :1;
      vuint32_t INT19 :1;
      vuint32_t INT18 :1;
      vuint32_t INT17 :1;
      vuint32_t INT16 :1;
      vuint32_t INT15 :1;
      vuint32_t INT14 :1;
      vuint32_t INT13 :1;
      vuint32_t INT12 :1;
      vuint32_t INT11 :1;
      vuint32_t INT10 :1;
      vuint32_t INT09 :1;
      vuint32_t INT08 :1;
      vuint32_t INT07 :1;
      vuint32_t INT06 :1;
      vuint32_t INT05 :1;
      vuint32_t INT04 :1;
      vuint32_t INT03 :1;
      vuint32_t INT02 :1;
      vuint32_t INT01 :1;
      vuint32_t INT00 :1;
    } B;
  } IRQRL; /* DMA Interrupt Request Low @baseaddress + 0x24 */

  union {
    vuint32_t R;
    struct {
      vuint32_t ERR63 :1;
      vuint32_t ERR62 :1;
      vuint32_t ERR61 :1;
      vuint32_t ERR60 :1;
      vuint32_t ERR59 :1;
      vuint32_t ERR58 :1;
      vuint32_t ERR57 :1;
      vuint32_t ERR56 :1;
      vuint32_t ERR55 :1;
      vuint32_t ERR54 :1;
      vuint32_t ERR53 :1;
      vuint32_t ERR52 :1;
      vuint32_t ERR51 :1;
      vuint32_t ERR50 :1;
      vuint32_t ERR49 :1;
      vuint32_t ERR48 :1;
      vuint32_t ERR47 :1;
      vuint32_t ERR46 :1;
      vuint32_t ERR45 :1;
      vuint32_t ERR44 :1;
      vuint32_t ERR43 :1;
      vuint32_t ERR42 :1;
      vuint32_t ERR41 :1;
      vuint32_t ERR40 :1;
      vuint32_t ERR39 :1;
      vuint32_t ERR38 :1;
      vuint32_t ERR37 :1;
      vuint32_t ERR36 :1;
      vuint32_t ERR35 :1;
      vuint32_t ERR34 :1;
      vuint32_t ERR33 :1;
      vuint32_t ERR32 :1;
    } B;
  } ERH; /* DMA Error High @baseaddress + 0x28 */

  union {
    vuint32_t R;
    struct {
      vuint32_t ERR31 :1;
      vuint32_t ERR30 :1;
      vuint32_t ERR29 :1;
      vuint32_t ERR28 :1;
      vuint32_t ERR27 :1;
      vuint32_t ERR26 :1;
      vuint32_t ERR25 :1;
      vuint32_t ERR24 :1;
      vuint32_t ERR23 :1;
      vuint32_t ERR22 :1;
      vuint32_t ERR21 :1;
      vuint32_t ERR20 :1;
      vuint32_t ERR19 :1;
      vuint32_t ERR18 :1;
      vuint32_t ERR17 :1;
      vuint32_t ERR16 :1;
      vuint32_t ERR15 :1;
      vuint32_t ERR14 :1;
      vuint32_t ERR13 :1;
      vuint32_t ERR12 :1;
      vuint32_t ERR11 :1;
      vuint32_t ERR10 :1;
      vuint32_t ERR09 :1;
      vuint32_t ERR08 :1;
      vuint32_t ERR07 :1;
      vuint32_t ERR06 :1;
      vuint32_t ERR05 :1;
      vuint32_t ERR04 :1;
      vuint32_t ERR03 :1;
      vuint32_t ERR02 :1;
      vuint32_t ERR01 :1;
      vuint32_t ERR00 :1;
    } B;
  } ERL; /* DMA Error Low @baseaddress + 0x2C */

  union {
    vuint32_t R;
    struct {
      vuint32_t HRS63 :1;
      vuint32_t HRS62 :1;
      vuint32_t HRS61 :1;
      vuint32_t HRS60 :1;
      vuint32_t HRS59 :1;
      vuint32_t HRS58 :1;
      vuint32_t HRS57 :1;
      vuint32_t HRS56 :1;
      vuint32_t HRS55 :1;
      vuint32_t HRS54 :1;
      vuint32_t HRS53 :1;
      vuint32_t HRS52 :1;
      vuint32_t HRS51 :1;
      vuint32_t HRS50 :1;
      vuint32_t HRS49 :1;
      vuint32_t HRS48 :1;
      vuint32_t HRS47 :1;
      vuint32_t HRS46 :1;
      vuint32_t HRS45 :1;
      vuint32_t HRS44 :1;
      vuint32_t HRS43 :1;
      vuint32_t HRS42 :1;
      vuint32_t HRS41 :1;
      vuint32_t HRS40 :1;
      vuint32_t HRS39 :1;
      vuint32_t HRS38 :1;
      vuint32_t HRS37 :1;
      vuint32_t HRS36 :1;
      vuint32_t HRS35 :1;
      vuint32_t HRS34 :1;
      vuint32_t HRS33 :1;
      vuint32_t HRS32 :1;
    } B;
  } HRSH; /* hardware request status high @baseaddress + 0x30 */

  union {
    vuint32_t R;
    struct {
      vuint32_t HRS31 :1;
      vuint32_t HRS30 :1;
      vuint32_t HRS29 :1;
      vuint32_t HRS28 :1;
      vuint32_t HRS27 :1;
      vuint32_t HRS26 :1;
      vuint32_t HRS25 :1;
      vuint32_t HRS24 :1;
      vuint32_t HRS23 :1;
      vuint32_t HRS22 :1;
      vuint32_t HRS21 :1;
      vuint32_t HRS20 :1;
      vuint32_t HRS19 :1;
      vuint32_t HRS18 :1;
      vuint32_t HRS17 :1;
      vuint32_t HRS16 :1;
      vuint32_t HRS15 :1;
      vuint32_t HRS14 :1;
      vuint32_t HRS13 :1;
      vuint32_t HRS12 :1;
      vuint32_t HRS11 :1;
      vuint32_t HRS10 :1;
      vuint32_t HRS09 :1;
      vuint32_t HRS08 :1;
      vuint32_t HRS07 :1;
      vuint32_t HRS06 :1;
      vuint32_t HRS05 :1;
      vuint32_t HRS04 :1;
      vuint32_t HRS03 :1;
      vuint32_t HRS02 :1;
      vuint32_t HRS01 :1;
      vuint32_t HRS00 :1;
    } B;
  } HRSL; /* hardware request status low @baseaddress + 0x34 */

  uint32_t eDMA_reserved0038[50]; /* 0x0038-0x00FF */

  union {
    vuint8_t R;
    struct {
      vuint8_t ECP :1;
      vuint8_t DPA :1;
      vuint8_t GRPPRI :2;
      vuint8_t CHPRI :4;
    } B;
  } CPR[64]; /* Channel n Priority @baseaddress + 0x100 */

  uint32_t eDMA_reserved0140[944]; /* 0x0140-0x0FFF */

  edma_tcd_t TCD[64];
} edma_t;

#if SPC5_EDMA_HAS_MUX || defined(__DOXYGEN__)
/**
 * @brief   Type of a DMA-MUX peripheral.
 */
typedef struct {
  union {
    vuint8_t R;
    struct {
      vuint8_t ENBL:1;
      vuint8_t TRIG:1;
      vuint8_t SOURCE:6;
    } B;
  } CHCONFIG[SPC5_EDMA_NCHANNELS];
} dma_mux_t;
#endif /* SPC5_EDMA_HAS_MUX */

/**
 * @brief   DMA callback type.
 *
 * @param[in] channel   the channel number
 * @param[in] p         parameter for the registered function
 */
typedef void (*edma_callback_t)(edma_channel_t channel, void *p);

/**
 * @brief   DMA error callback type.
 *
 * @param[in] channel   the channel number
 * @param[in] p         parameter for the registered function
 * @param[in] esr       content of the ESR register
 */
typedef void (*edma_error_callback_t)(edma_channel_t channel,
                                      void *p,
                                      uint32_t esr);

/**
 * @brief   Type of an EDMA channel configuration structure.
 */
typedef struct {
  edma_channel_t        dma_channel;    /**< @brief Channel to be allocated.*/
#if SPC5_EDMA_HAS_MUX || defined(__DOXYGEN__)
  uint8_t               dma_periph;     /**< @brief Peripheral to be
                                             associated to the channel.     */
#endif
  uint8_t               dma_irq_prio;   /**< @brief IRQ priority level for
                                             this channel.                  */
  edma_callback_t       dma_func;       /**< @brief Channel callback,
                                             can be NULL if not required.  */
  edma_error_callback_t dma_error_func; /**< @brief Channel error callback,
                                             can be NULL if not required.  */
  void                  *dma_param;     /**< @brief Channel callback param. */
} edma_channel_config_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Peripherals references
 *
 * @{
 */
#if SPC5_HAS_EDMA || defined(__DOXYGEN__)
#define SPC5_EDMA       (*(edma_t *)0xFFF44000U)
#endif

#if SPC5_EDMA_HAS_MUX || defined(__DOXYGEN__)
#define SPC5_DMAMUX     (*(dma_mux_t *)0xFFFDC000UL)
#endif
/** @} */

/**
 * @brief   Returns the TCD address associated to a channel.
 *
 * @param[in] channel   the channel number
 * @return              A pointer to an @p edma_tcd_t structure.
 *
 * @api
 */
#define edmaGetTCD(channel) ((edma_tcd_t *)&SPC5_EDMA.TCD[channel])

/**
 * @brief   Sets the word 0 fields into a TCD.
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] src       the source address
 *
 * @api
 */
#define edmaTCDSetWord0(tcdp, src)                                          \
  ((tcdp)->word[0] = (uint32_t)(src))

/**
 * @brief   Sets the word 1 fields into a TCD.
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] ssize     the source width
 * @param[in] dst       the destination width
 * @param[in] soff      the source increment value
 *
 * @api
 */
#define edmaTCDSetWord1(tcdp, ssize, dsize, soff)                           \
  ((tcdp)->word[1] = (((uint32_t)(ssize) << 24) |                           \
                      ((uint32_t)(dsize) << 16) |                           \
                      ((uint32_t)(soff) << 0)))

/**
 * @brief   Sets the word 2 fields into a TCD.
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] nbytes    the inner counter value
 *
 * @api
 */
#define edmaTCDSetWord2(tcdp, nbytes)                                       \
  ((tcdp)->word[2] = (uint32_t)(nbytes))

/**
 * @brief   Sets the word 3 fields into a TCD.
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] slast      the adjustment value
 *
 * @api
 */
#define edmaTCDSetWord3(tcdp, slast)                                        \
  ((tcdp)->word[3] = (uint32_t)(slast))

/**
 * @brief   Sets the word 4 fields into a TCD.
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] dst       the destination address
 *
 * @api
 */
#define edmaTCDSetWord4(tcdp, dst)                                          \
  ((tcdp)->word[4] = (uint32_t)(dst))

/**
 * @brief   Sets the word 5 fields into a TCD.
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] citer     the current outer counter value
 * @param[in] doff      the destination increment value
 *
 * @api
 */
#define edmaTCDSetWord5(tcdp, citer, doff)                                  \
  ((tcdp)->word[5] = (((uint32_t)(citer) << 16) |                           \
                      ((uint32_t)(doff) << 0)))

/**
 * @brief   Sets the word 5 fields into a TCD.
 * @note    Transfers are limited to 512 operations using this modality
 *          (citer parameter).
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] linkch    channel linked on minor loop counter
 * @param[in] citer     the current outer counter value
 * @param[in] doff      the destination increment value
 *
 * @api
 */
#define edmaTCDSetWord5Linked(tcdp, linkch, citer, doff)                    \
  ((tcdp)->word[5] = (((uint32_t)0x80000000) |                              \
                      ((uint32_t)(linkch) << 25) |                          \
                      ((uint32_t)(citer) << 16) |                           \
                      ((uint32_t)(doff) << 0)))

/**
 * @brief   Sets the word 6 fields into a TCD.
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] dlast     the adjustment value
 *
 * @api
 */
#define edmaTCDSetWord6(tcdp, dlast)                                        \
  ((tcdp)->word[6] = (uint32_t)(dlast))

/**
 * @brief   Sets the word 7 fields into a TCD.
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] biter     the base outer counter value
 * @param[in] mode      the mode value
 *
 * @api
 */
#define edmaTCDSetWord7(tcdp, biter, mode)                                  \
  ((tcdp)->word[7] = (((uint32_t)(biter) << 16) |                           \
                      ((uint32_t)(mode) << 0)))

/**
 * @brief   Sets the word 7 fields into a TCD.
 * @note    Transfers are limited to 512 operations using this modality
 *          (biter parameter).
 *
 * @param[in] tcdp      pointer to an @p edma_tcd_t structure
 * @param[in] linkch    channel linked on minor loop counter
 * @param[in] biter     the base outer counter value
 * @param[in] mode      the mode value
 *
 * @api
 */
#define edmaTCDSetWord7Linked(tcdp, linkch, biter, mode)                    \
  ((tcdp)->word[7] = (((uint32_t)0x80000000) |                              \
                      ((uint32_t)(linkch) << 25) |                          \
                      ((uint32_t)(biter) << 16) |                           \
                      ((uint32_t)(mode) << 0)))

/**
 * @brief   Starts or restarts an EDMA channel.
 *
 * @param[in] channel   the channel number
 *
 * @api
 */
#define edmaChannelStart(channel) (SPC5_EDMA.SERQR.R = (channel))

/**
 * @brief   Stops an EDMA channel.
 *
 * @param[in] channel   the channel number
 *
 * @api
 */
#define edmaChannelStop(channel) {                                          \
  SPC5_EDMA.CERQR.R = (channel);                                            \
  SPC5_EDMA.CDSBR.R = (channel);                                            \
}

/**
 * @brief   EDMA channel setup.
 *
 * @param[in] channel   eDMA channel number
 * @param[in] src       source address
 * @param[in] dst       destination address
 * @param[in] soff      source address offset
 * @param[in] doff      destination address offset
 * @param[in] ssize     source transfer size
 * @param[in] dsize     destination transfer size
 * @param[in] nbytes    minor loop count
 * @param[in] iter      major loop count
 * @param[in] dlast     last destination address adjustment
 * @param[in] slast     last source address adjustment
 * @param[in] mode      LSW of TCD register 7
 *
 * @api
 */
#define edmaChannelSetup(channel, src, dst, soff, doff, ssize, dsize,       \
                         nbytes, iter, slast, dlast, mode) {                \
  edma_tcd_t *tcdp = edmaGetTCD(channel);                                   \
  edmaTCDSetWord0(tcdp, src);                                               \
  edmaTCDSetWord1(tcdp, ssize, dsize, soff);                                \
  edmaTCDSetWord2(tcdp, nbytes);                                            \
  edmaTCDSetWord3(tcdp, slast);                                             \
  edmaTCDSetWord4(tcdp, dst);                                               \
  edmaTCDSetWord5(tcdp, iter, doff);                                        \
  edmaTCDSetWord6(tcdp, dlast);                                             \
  edmaTCDSetWord7(tcdp, iter, mode);                                        \
}

/**
 * @brief   EDMA channel setup with linked channel on both minor and major
 *          loop counters.
 * @note    Transfers are limited to 512 operations using this modality
 *          (iter parameter).
 *
 * @param[in] channel   eDMA channel number
 * @param[in] linkch    channel linked on minor loop counter
 * @param[in] src       source address
 * @param[in] dst       destination address
 * @param[in] soff      source address offset
 * @param[in] doff      destination address offset
 * @param[in] ssize     source transfer size
 * @param[in] dsize     destination transfer size
 * @param[in] nbytes    minor loop count
 * @param[in] iter      major loop count
 * @param[in] dlast     last destination address adjustment
 * @param[in] slast     last source address adjustment
 * @param[in] mode      LSW of TCD register 7
 *
 * @api
 */
#define edmaChannelSetupLinked(channel, linkch, src, dst, soff,             \
                               doff, ssize, dsize, nbytes, iter,            \
                               slast, dlast, mode) {                        \
  edma_tcd_t *tcdp = edmaGetTCD(channel);                                   \
  edmaTCDSetWord0(tcdp, src);                                               \
  edmaTCDSetWord1(tcdp, ssize, dsize, soff);                                \
  edmaTCDSetWord2(tcdp, nbytes);                                            \
  edmaTCDSetWord3(tcdp, slast);                                             \
  edmaTCDSetWord4(tcdp, dst);                                               \
  edmaTCDSetWord5Linked(tcdp, linkch, iter, doff);                          \
  edmaTCDSetWord6(tcdp, dlast);                                             \
  edmaTCDSetWord7Linked(tcdp, linkch, iter, (mode) |                        \
                                            EDMA_TCD_MODE_MELINK |          \
                                            EDMA_TCD_MODE_MLINKCH(linkch)); \
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void edmaInit(void);
  edma_channel_t edmaChannelAllocate(const edma_channel_config_t *ccfg);
  void edmaChannelRelease(edma_channel_t channel);
#ifdef __cplusplus
}
#endif

#endif /* SPC5_HAS_EDMA */

#endif /* _SPC5_EDMA_H_ */

/** @} */
