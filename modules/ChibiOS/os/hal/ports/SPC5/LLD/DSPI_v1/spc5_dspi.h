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
 * @file    DSPI_v1/spc5_dspi.h
 * @brief   SPC5xx DSPI header file.
 *
 * @addtogroup SPI
 * @{
 */

#ifndef _SPC5_DSPI_H_
#define _SPC5_DSPI_H_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    MCR register definitions
 * @{
 */
#define SPC5_MCR_MSTR                       (1U << 31)
#define SPC5_MCR_CONT_SCKE                  (1U << 30)
#define SPC5_MCR_DCONF_MASK                 (3U << 28)
#define SPC5_MCR_FRZ                        (1U << 27)
#define SPC5_MCR_MTFE                       (1U << 26)
#define SPC5_MCR_PCSSE                      (1U << 25)
#define SPC5_MCR_ROOE                       (1U << 24)
#define SPC5_MCR_PCSIS7                     (1U << 23)
#define SPC5_MCR_PCSIS6                     (1U << 22)
#define SPC5_MCR_PCSIS5                     (1U << 21)
#define SPC5_MCR_PCSIS4                     (1U << 20)
#define SPC5_MCR_PCSIS3                     (1U << 19)
#define SPC5_MCR_PCSIS2                     (1U << 18)
#define SPC5_MCR_PCSIS1                     (1U << 17)
#define SPC5_MCR_PCSIS0                     (1U << 16)
#define SPC5_MCR_DOZE                       (1U << 15)
#define SPC5_MCR_MDIS                       (1U << 14)
#define SPC5_MCR_DIS_TXF                    (1U << 13)
#define SPC5_MCR_DIS_RXF                    (1U << 12)
#define SPC5_MCR_CLR_TXF                    (1U << 11)
#define SPC5_MCR_CLR_RXF                    (1U << 10)
#define SPC5_MCR_SMPL_PT_MASK               (3U << 8)
#define SPC5_MCR_SMPL_PT(n)                 ((n) << 8)
#define SPC5_MCR_FCPCS                      (1U << 2)
#define SPC5_MCR_PES                        (1U << 1)
#define SPC5_MCR_HALT                       (1U << 0)
/** @} */

/**
 * @name    RSER register definitions
 * @{
 */
#define SPC5_RSER_TCF_RE                    (1U << 31)
#define SPC5_RSER_DSITCF_RE                 (1U << 29)
#define SPC5_RSER_EOQF_RE                   (1U << 28)
#define SPC5_RSER_TFUF_RE                   (1U << 27)
#define SPC5_RSER_SPITCF_RE                 (1U << 26)
#define SPC5_RSER_TFFF_RE                   (1U << 25)
#define SPC5_RSER_TFFF_DIRS                 (1U << 24)
#define SPC5_RSER_DPEF_RE                   (1U << 22)
#define SPC5_RSER_SPEF_RE                   (1U << 21)
#define SPC5_RSER_DDIF_RE                   (1U << 20)
#define SPC5_RSER_RFOF_RE                   (1U << 19)
#define SPC5_RSER_RFDF_RE                   (1U << 17)
#define SPC5_RSER_RFDF_DIRS                 (1U << 16)
/** @} */

/**
 * @name    CTAR registers definitions
 * @{
 */
#define SPC5_CTAR_DBR                       (1U << 31)
#define SPC5_CTAR_FMSZ_MASK                 (15U << 27)
#define SPC5_CTAR_FMSZ(n)                   (((n) - 1) << 27)
#define SPC5_CTAR_CPOL                      (1U << 26)
#define SPC5_CTAR_CPHA                      (1U << 25)
#define SPC5_CTAR_LSBFE                     (1U << 24)
#define SPC5_CTAR_PCSSCK_MASK               (3U << 22)
#define SPC5_CTAR_PCSSCK_PRE1               (0U << 22)
#define SPC5_CTAR_PCSSCK_PRE3               (1U << 22)
#define SPC5_CTAR_PCSSCK_PRE5               (2U << 22)
#define SPC5_CTAR_PCSSCK_PRE7               (3U << 22)
#define SPC5_CTAR_PASC_MASK                 (3U << 20)
#define SPC5_CTAR_PASC_PRE1                 (0U << 20)
#define SPC5_CTAR_PASC_PRE3                 (1U << 20)
#define SPC5_CTAR_PASC_PRE5                 (2U << 20)
#define SPC5_CTAR_PASC_PRE7                 (3U << 20)
#define SPC5_CTAR_PDT_MASK                  (3U << 18)
#define SPC5_CTAR_PDT_PRE1                  (0U << 18)
#define SPC5_CTAR_PDT_PRE3                  (1U << 18)
#define SPC5_CTAR_PDT_PRE5                  (2U << 18)
#define SPC5_CTAR_PDT_PRE7                  (3U << 18)
#define SPC5_CTAR_PBR_MASK                  (3U << 16)
#define SPC5_CTAR_PBR_PRE2                  (0U << 16)
#define SPC5_CTAR_PBR_PRE3                  (1U << 16)
#define SPC5_CTAR_PBR_PRE5                  (2U << 16)
#define SPC5_CTAR_PBR_PRE7                  (3U << 16)
#define SPC5_CTAR_CSSCK_MASK                (15U << 12)
#define SPC5_CTAR_CSSCK_DIV2                (0U << 12)
#define SPC5_CTAR_CSSCK_DIV4                (1U << 12)
#define SPC5_CTAR_CSSCK_DIV8                (2U << 12)
#define SPC5_CTAR_CSSCK_DIV16               (3U << 12)
#define SPC5_CTAR_CSSCK_DIV32               (4U << 12)
#define SPC5_CTAR_CSSCK_DIV64               (5U << 12)
#define SPC5_CTAR_CSSCK_DIV128              (6U << 12)
#define SPC5_CTAR_CSSCK_DIV256              (7U << 12)
#define SPC5_CTAR_CSSCK_DIV512              (8U << 12)
#define SPC5_CTAR_CSSCK_DIV1024             (9U << 12)
#define SPC5_CTAR_CSSCK_DIV2048             (10U << 12)
#define SPC5_CTAR_CSSCK_DIV4096             (11U << 12)
#define SPC5_CTAR_CSSCK_DIV8192             (12U << 12)
#define SPC5_CTAR_CSSCK_DIV16384            (13U << 12)
#define SPC5_CTAR_CSSCK_DIV32768            (14U << 12)
#define SPC5_CTAR_CSSCK_DIV65536            (15U << 12)
#define SPC5_CTAR_ASC_MASK                  (15U << 8)
#define SPC5_CTAR_ASC_DIV2                  (0U << 8)
#define SPC5_CTAR_ASC_DIV4                  (1U << 8)
#define SPC5_CTAR_ASC_DIV8                  (2U << 8)
#define SPC5_CTAR_ASC_DIV16                 (3U << 8)
#define SPC5_CTAR_ASC_DIV32                 (4U << 8)
#define SPC5_CTAR_ASC_DIV64                 (5U << 8)
#define SPC5_CTAR_ASC_DIV128                (6U << 8)
#define SPC5_CTAR_ASC_DIV256                (7U << 8)
#define SPC5_CTAR_ASC_DIV512                (8U << 8)
#define SPC5_CTAR_ASC_DIV1024               (9U << 8)
#define SPC5_CTAR_ASC_DIV2048               (10U << 8)
#define SPC5_CTAR_ASC_DIV4096               (11U << 8)
#define SPC5_CTAR_ASC_DIV8192               (12U << 8)
#define SPC5_CTAR_ASC_DIV16384              (13U << 8)
#define SPC5_CTAR_ASC_DIV32768              (14U << 8)
#define SPC5_CTAR_ASC_DIV65536              (15U << 8)
#define SPC5_CTAR_DT_MASK                   (15U << 4)
#define SPC5_CTAR_DT_DIV2                   (0U << 4)
#define SPC5_CTAR_DT_DIV4                   (1U << 4)
#define SPC5_CTAR_DT_DIV8                   (2U << 4)
#define SPC5_CTAR_DT_DIV16                  (3U << 4)
#define SPC5_CTAR_DT_DIV32                  (4U << 4)
#define SPC5_CTAR_DT_DIV64                  (5U << 4)
#define SPC5_CTAR_DT_DIV128                 (6U << 4)
#define SPC5_CTAR_DT_DIV256                 (7U << 4)
#define SPC5_CTAR_DT_DIV512                 (8U << 4)
#define SPC5_CTAR_DT_DIV1024                (9U << 4)
#define SPC5_CTAR_DT_DIV2048                (10U << 4)
#define SPC5_CTAR_DT_DIV4096                (11U << 4)
#define SPC5_CTAR_DT_DIV8192                (12U << 4)
#define SPC5_CTAR_DT_DIV16384               (13U << 4)
#define SPC5_CTAR_DT_DIV32768               (14U << 4)
#define SPC5_CTAR_DT_DIV65536               (15U << 4)
#define SPC5_CTAR_BR_MASK                   (15U << 0)
#define SPC5_CTAR_BR_DIV2                   (0U << 0)
#define SPC5_CTAR_BR_DIV4                   (1U << 0)
#define SPC5_CTAR_BR_DIV6                   (2U << 0)
#define SPC5_CTAR_BR_DIV8                   (3U << 0)
#define SPC5_CTAR_BR_DIV16                  (4U << 0)
#define SPC5_CTAR_BR_DIV32                  (5U << 0)
#define SPC5_CTAR_BR_DIV64                  (6U << 0)
#define SPC5_CTAR_BR_DIV128                 (7U << 0)
#define SPC5_CTAR_BR_DIV256                 (8U << 0)
#define SPC5_CTAR_BR_DIV512                 (9U << 0)
#define SPC5_CTAR_BR_DIV1024                (10U << 0)
#define SPC5_CTAR_BR_DIV2048                (11U << 0)
#define SPC5_CTAR_BR_DIV4096                (12U << 0)
#define SPC5_CTAR_BR_DIV8192                (13U << 0)
#define SPC5_CTAR_BR_DIV16384               (14U << 0)
#define SPC5_CTAR_BR_DIV32768               (15U << 0)
/** @} */

/**
 * @name    PUSHR register definitions
 * @{
 */
#define SPC5_PUSHR_CONT                     (1U << 31)
#define SPC5_PUSHR_CTAS_MASK                (3U << 28)
#define SPC5_PUSHR_CTAS(n)                  ((n) << 29)
#define SPC5_PUSHR_EOQ                      (1U << 27)
#define SPC5_PUSHR_CTCNT                    (1U << 26)
#define SPC5_PUSHR_MASC                     (1U << 25)
#define SPC5_PUSHR_MCSC                     (1U << 24)
#define SPC5_PUSHR_PCS_MASK                 (255U << 16)
#define SPC5_PUSHR_PCS(n)                   ((1U << (n)) << 16)
#define SPC5_PUSHR_TXDATA_MASK              (0xFFFFU << 0)
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

struct spc5_dspi {
  union {
    vuint32_t R;
    struct {
      vuint32_t MSTR :1;
      vuint32_t CONT_SCKE :1;
      vuint32_t DCONF :2;
      vuint32_t FRZ :1;
      vuint32_t MTFE :1;
      vuint32_t PCSSE :1;
      vuint32_t ROOE :1;
      vuint32_t PCSIS7 :1;
      vuint32_t PCSIS6 :1;
      vuint32_t PCSIS5 :1;
      vuint32_t PCSIS4 :1;
      vuint32_t PCSIS3 :1;
      vuint32_t PCSIS2 :1;
      vuint32_t PCSIS1 :1;
      vuint32_t PCSIS0 :1;
      vuint32_t :1;
      vuint32_t MDIS :1;
      vuint32_t DIS_TXF :1;
      vuint32_t DIS_RXF :1;
      vuint32_t CLR_TXF :1;
      vuint32_t CLR_RXF :1;
      vuint32_t SMPL_PT :2;
      vuint32_t :7;
      vuint32_t HALT :1;
    } B;
  } MCR; /* Module Configuration Register */

  uint32_t dspi_reserved1;

  union {
    vuint32_t R;
    struct {
      vuint32_t TCNT :16;
      vuint32_t :16;
    } B;
  } TCR;

  union {
    vuint32_t R;
    struct {
      vuint32_t DBR :1;
      vuint32_t FMSZ :4;
      vuint32_t CPOL :1;
      vuint32_t CPHA :1;
      vuint32_t LSBFE :1;
      vuint32_t PCSSCK :2;
      vuint32_t PASC :2;
      vuint32_t PDT :2;
      vuint32_t PBR :2;
      vuint32_t CSSCK :4;
      vuint32_t ASC :4;
      vuint32_t DT :4;
      vuint32_t BR :4;
    } B;
  } CTAR[8]; /* Clock and Transfer Attributes Registers */

  union {
    vuint32_t R;
    struct {
      vuint32_t TCF :1;
      vuint32_t TXRXS :1;
      vuint32_t :1;
      vuint32_t EOQF :1;
      vuint32_t TFUF :1;
      vuint32_t :1;
      vuint32_t TFFF :1;
      vuint32_t :5;
      vuint32_t RFOF :1;
      vuint32_t :1;
      vuint32_t RFDF :1;
      vuint32_t :1;
      vuint32_t TXCTR :4;
      vuint32_t TXNXTPTR :4;
      vuint32_t RXCTR :4;
      vuint32_t POPNXTPTR :4;
    } B;
  } SR; /* Status Register */

  union {
    vuint32_t R;
    struct {
      vuint32_t TCFRE :1;
      vuint32_t :2;
      vuint32_t EOQFRE :1;
      vuint32_t TFUFRE :1;
      vuint32_t :1;
      vuint32_t TFFFRE :1;
      vuint32_t TFFFDIRS :1;
      vuint32_t :4;
      vuint32_t RFOFRE :1;
      vuint32_t :1;
      vuint32_t RFDFRE :1;
      vuint32_t RFDFDIRS :1;
      vuint32_t :16;
    } B;
  } RSER; /* DMA/Interrupt Request Select and Enable Register */

  union {
    vuint32_t R;
    struct {
      vuint32_t CONT :1;
      vuint32_t CTAS :3;
      vuint32_t EOQ :1;
      vuint32_t CTCNT :1;
      vuint32_t :2;
      vuint32_t PCS7 :1;
      vuint32_t PCS6 :1;
      vuint32_t PCS5 :1;
      vuint32_t PCS4 :1;
      vuint32_t PCS3 :1;
      vuint32_t PCS2 :1;
      vuint32_t PCS1 :1;
      vuint32_t PCS0 :1;
      vuint32_t TXDATA :16;
    } B;
  } PUSHR; /* PUSH TX FIFO Register */

  union {
    vuint32_t R;
    struct {
      vuint32_t :16;
      vuint32_t RXDATA :16;
    } B;
  } POPR; /* POP RX FIFO Register */

  union {
    vuint32_t R;
    struct {
      vuint32_t TXCMD :16;
      vuint32_t TXDATA :16;
    } B;
  } TXFR[5]; /* Transmit FIFO Registers */

  vuint32_t DSPI_reserved_txf[11];

  union {
    vuint32_t R;
    struct {
      vuint32_t :16;
      vuint32_t RXDATA :16;
    } B;
  } RXFR[5]; /* Receive FIFO Registers */

  vuint32_t DSPI_reserved_rxf[12];

  union {
    vuint32_t R;
    struct {
      vuint32_t MTOE :1;
      vuint32_t :1;
      vuint32_t MTOCNT :6;
      vuint32_t :4;
      vuint32_t TXSS :1;
      vuint32_t TPOL :1;
      vuint32_t TRRE :1;
      vuint32_t CID :1;
      vuint32_t DCONT :1;
      vuint32_t DSICTAS :3;
      vuint32_t :6;
      vuint32_t DPCS5 :1;
      vuint32_t DPCS4 :1;
      vuint32_t DPCS3 :1;
      vuint32_t DPCS2 :1;
      vuint32_t DPCS1 :1;
      vuint32_t DPCS0 :1;
    } B;
  } DSICR; /* DSI Configuration Register */

  union {
    vuint32_t R;
    struct {
      vuint32_t :16;
      vuint32_t SER_DATA :16;
    } B;
  } SDR; /* DSI Serialization Data Register */

  union {
    vuint32_t R;
    struct {
      vuint32_t :16;
      vuint32_t ASER_DATA :16;
    } B;
  } ASDR; /* DSI Alternate Serialization Data Register */

  union {
    vuint32_t R;
    struct {
      vuint32_t :16;
      vuint32_t COMP_DATA :16;
    } B;
  } COMPR; /* DSI Transmit Comparison Register */

  union {
    vuint32_t R;
    struct {
      vuint32_t :16;
      vuint32_t DESER_DATA :16;
    } B;
  } DDR; /* DSI deserialization Data Register */

};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    DSPI units references
 * @{
 */
#if SPC5_HAS_DSPI0 || defined(__DOXYGEN__)
#define SPC5_DSPI0      (*(struct spc5_dspi *)0xFFF90000U)
#endif

#if SPC5_HAS_DSPI1 || defined(__DOXYGEN__)
#define SPC5_DSPI1      (*(struct spc5_dspi *)0xFFF94000U)
#endif

#if SPC5_HAS_DSPI2 || defined(__DOXYGEN__)
#define SPC5_DSPI2      (*(struct spc5_dspi *)0xFFF98000U)
#endif

#if SPC5_HAS_DSPI3 || defined(__DOXYGEN__)
#define SPC5_DSPI3      (*(struct spc5_dspi *)0xFFF9C000U)
#endif

#if SPC5_HAS_DSPI4 || defined(__DOXYGEN__)
#define SPC5_DSPI4      (*(struct spc5_dspi *)0xFFFA0000U)
#endif

#if SPC5_HAS_DSPI5 || defined(__DOXYGEN__)
#define SPC5_DSPI5      (*(struct spc5_dspi *)0xFFFA4000U)
#endif

#if SPC5_HAS_DSPI6 || defined(__DOXYGEN__)
#define SPC5_DSPI6      (*(struct spc5_dspi *)0xFFFA8000U)
#endif

#if SPC5_HAS_DSPI7 || defined(__DOXYGEN__)
#define SPC5_DSPI7      (*(struct spc5_dspi *)0xFFFAC000U)
#endif
/** @} */

#endif /* _SPC5_DSPI_H_ */

/** @} */
