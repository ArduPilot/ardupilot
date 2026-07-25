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
 * @file    SPC5xx/spc5_linflex.h
 * @brief   LINFlex helper driver header.
 *
 * @addtogroup SPC5xx_LINFLEX
 * @{
 */

#ifndef _SPC5_LINFLEX_H_
#define _SPC5_LINFLEX_H_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    LINIER register bits definitions
 * @{
 */
#define SPC5_LINIER_HRIE                    (1U << 0)
#define SPC5_LINIER_DTIE                    (1U << 1)
#define SPC5_LINIER_DRIE                    (1U << 2)
#define SPC5_LINIER_DBEIE                   (1U << 3)
#define SPC5_LINIER_DBFIE                   (1U << 4)
#define SPC5_LINIER_WUIE                    (1U << 5)
#define SPC5_LINIER_LSIE                    (1U << 6)
#define SPC5_LINIER_BOIE                    (1U << 7)
#define SPC5_LINIER_FEIE                    (1U << 8)
#define SPC5_LINIER_HEIE                    (1U << 11)
#define SPC5_LINIER_CEIE                    (1U << 12)
#define SPC5_LINIER_BEIE                    (1U << 13)
#define SPC5_LINIER_OCIE                    (1U << 14)
#define SPC5_LINIER_SZIE                    (1U << 15)
/** @} */

/**
 * @name    UARTSR register bits definitions
 * @{
 */
#define SPC5_UARTSR_NF                      (1U << 0)
#define SPC5_UARTSR_DTF                     (1U << 1)
#define SPC5_UARTSR_DRF                     (1U << 2)
#define SPC5_UARTSR_WUF                     (1U << 5)
#define SPC5_UARTSR_RPS                     (1U << 6)
#define SPC5_UARTSR_BOF                     (1U << 7)
#define SPC5_UARTSR_FEF                     (1U << 8)
#define SPC5_UARTSR_RMB                     (1U << 9)
#define SPC5_UARTSR_PE0                     (1U << 10)
#define SPC5_UARTSR_PE1                     (1U << 11)
#define SPC5_UARTSR_PE2                     (1U << 12)
#define SPC5_UARTSR_PE3                     (1U << 13)
#define SPC5_UARTSR_OCF                     (1U << 14)
#define SPC5_UARTSR_SZF                     (1U << 15)
/** @} */

/**
 * @name    UARTCR register bits definitions
 * @{
 */
#define SPC5_UARTCR_UART                    (1U << 0)
#define SPC5_UARTCR_WL                      (1U << 1)
#define SPC5_UARTCR_PCE                     (1U << 2)
#define SPC5_UARTCR_OP                      (1U << 3)
#define SPC5_UARTCR_TXEN                    (1U << 4)
#define SPC5_UARTCR_RXEN                    (1U << 5)
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


struct spc5_linflex {

  int16_t LINFLEX_reserved1;

  union {
    vuint16_t R;
    struct {
      vuint16_t CCD :1;
      vuint16_t CFD :1;
      vuint16_t LASE :1;
      vuint16_t AWUM :1;
      vuint16_t MBL :4;
      vuint16_t BF :1;
      vuint16_t SFTM :1;
      vuint16_t LBKM :1;
      vuint16_t MME :1;
      vuint16_t SBDT :1;
      vuint16_t RBLM :1;
      vuint16_t SLEEP :1;
      vuint16_t INIT :1;
    } B;
  } LINCR1;

  int16_t LINFLEX_reserved2;

  union {
    vuint16_t R;
    struct {
      vuint16_t SZIE :1;
      vuint16_t OCIE :1;
      vuint16_t BEIE :1;
      vuint16_t CEIE :1;
      vuint16_t HEIE :1;
      vuint16_t :2;
      vuint16_t FEIE :1;
      vuint16_t BOIE :1;
      vuint16_t LSIE :1;
      vuint16_t WUIE :1;
      vuint16_t DBFIE :1;
      vuint16_t DBEIE :1;
      vuint16_t DRIE :1;
      vuint16_t DTIE :1;
      vuint16_t HRIE :1;
    } B;
  } LINIER;

  int16_t LINFLEX_reserved3;

  union {
    vuint16_t R;
    struct {
      vuint16_t LINS :4;
      vuint16_t :2;
      vuint16_t RMB :1;
      vuint16_t :1;
      vuint16_t RBSY :1;
      vuint16_t RPS :1;
      vuint16_t WUF :1;
      vuint16_t DBFF :1;
      vuint16_t DBEF :1;
      vuint16_t DRF :1;
      vuint16_t DTF :1;
      vuint16_t HRF :1;
    } B;
  } LINSR;

  int16_t LINFLEX_reserved4;

  union {
    vuint16_t R;
    struct {
      vuint16_t SZF :1;
      vuint16_t OCF :1;
      vuint16_t BEF :1;
      vuint16_t CEF :1;
      vuint16_t SFEF :1;
      vuint16_t BDEF :1;
      vuint16_t IDPEF :1;
      vuint16_t FEF :1;
      vuint16_t BOF :1;
      vuint16_t :6;
      vuint16_t NF :1;
    } B;
  } LINESR;

  int16_t LINFLEX_reserved5;

  union {
    vuint16_t R;
    struct {
      vuint16_t :1;
      vuint16_t TDFL :2;
      vuint16_t :1;
      vuint16_t RDFL :2;
      vuint16_t :4;
      vuint16_t RXEN :1;
      vuint16_t TXEN :1;
      vuint16_t OP :1;
      vuint16_t PCE :1;
      vuint16_t WL :1;
      vuint16_t UART :1;
    } B;
  } UARTCR;

  int16_t LINFLEX_reserved6;

  union {
    vuint16_t R;
    struct {
      vuint16_t SZF :1;
      vuint16_t OCF :1;
      vuint16_t PE :4;
      vuint16_t RMB :1;
      vuint16_t FEF :1;
      vuint16_t BOF :1;
      vuint16_t RPS :1;
      vuint16_t WUF :1;
      vuint16_t :2;
      vuint16_t DRF :1;
      vuint16_t DTF :1;
      vuint16_t NF :1;
    } B;
  } UARTSR;

  int16_t LINFLEX_reserved7;

  union {
    vuint16_t R;
    struct {
      vuint16_t :5;
      vuint16_t LTOM :1;
      vuint16_t IOT :1;
      vuint16_t TOCE :1;
      vuint16_t CNT :8;
    } B;
  } LINTCSR;

  int16_t LINFLEX_reserved8;

  union {
    vuint16_t R;
    struct {
      vuint16_t OC2 :8;
      vuint16_t OC1 :8;
    } B;
  } LINOCR;

  int16_t LINFLEX_reserved9;

  union {
    vuint16_t R;
    struct {
      vuint16_t :4;
      vuint16_t RTO :4;
      vuint16_t :1;
      vuint16_t HTO :7;
    } B;
  } LINTOCR;

  int16_t LINFLEX_reserved10;

  union {
    vuint16_t R;
    struct {
      vuint16_t :12;
      vuint16_t DIV_F :4;
    } B;
  } LINFBRR;

  int16_t LINFLEX_reserved11;

  union {
    vuint16_t R;
    struct {
      vuint16_t :3;
      vuint16_t DIV_M :13;
    } B;
  } LINIBRR;

  int16_t LINFLEX_reserved12;

  union {
    vuint16_t R;
    struct {
      vuint16_t :8;
      vuint16_t CF :8;
    } B;
  } LINCFR;

  int16_t LINFLEX_reserved13;

  union {
    vuint16_t R;
    struct {
      vuint16_t :1;
      vuint16_t IOBE :1;
      vuint16_t IOPE :1;
      vuint16_t WURQ :1;
      vuint16_t DDRQ :1;
      vuint16_t DTRQ :1;
      vuint16_t ABRQ :1;
      vuint16_t HTRQ :1;
      vuint16_t :8;
    } B;
  } LINCR2;

  int16_t LINFLEX_reserved14;

  union {
    vuint16_t R;
    struct {
      vuint16_t DFL :6;
      vuint16_t DIR :1;
      vuint16_t CCS :1;
      vuint16_t :2;
      vuint16_t ID :6;
    } B;
  } BIDR;

  union {
    vuint32_t R;
    struct {
      vuint32_t DATA3 :8;
      vuint32_t DATA2 :8;
      vuint32_t DATA1 :8;
      vuint32_t DATA0 :8;
    } B;
  } BDRL;

  union {
    vuint32_t R;
    struct {
      vuint32_t DATA7 :8;
      vuint32_t DATA6 :8;
      vuint32_t DATA5 :8;
      vuint32_t DATA4 :8;
    } B;
  } BDRM;

  int16_t LINFLEX_reserved15;

  union {
    vuint16_t R;
    struct {
      vuint16_t :8;
      vuint16_t FACT :8;
    } B;
  } IFER;

  int16_t LINFLEX_reserved16;

  union {
    vuint16_t R;
    struct {
      vuint16_t :12;
      vuint16_t IFMI :4;
    } B;
  } IFMI;

  int16_t LINFLEX_reserved17;

  union {
    vuint16_t R;
    struct {
      vuint16_t :12;
      vuint16_t IFM :4;
    } B;
  } IFMR;

  int16_t LINFLEX_reserved18;

  union {
    vuint16_t R;
    struct {
      vuint16_t :3;
      vuint16_t DFL :3;
      vuint16_t DIR :1;
      vuint16_t CCS :1;
      vuint16_t :2;
      vuint16_t ID :6;
    } B;
  } IFCR0;

  int16_t LINFLEX_reserved19;

  union {
    vuint16_t R;
    struct {
      vuint16_t :3;
      vuint16_t DFL :3;
      vuint16_t DIR :1;
      vuint16_t CCS :1;
      vuint16_t :2;
      vuint16_t ID :6;
    } B;
  } IFCR1;

  int16_t LINFLEX_reserved20;

  union {
    vuint16_t R;
    struct {
      vuint16_t :3;
      vuint16_t DFL :3;
      vuint16_t DIR :1;
      vuint16_t CCS :1;
      vuint16_t :2;
      vuint16_t ID :6;
    } B;
  } IFCR2;

  int16_t LINFLEX_reserved21;

  union {
    vuint16_t R;
    struct {
      vuint16_t :3;
      vuint16_t DFL :3;
      vuint16_t DIR :1;
      vuint16_t CCS :1;
      vuint16_t :2;
      vuint16_t ID :6;
    } B;
  } IFCR3;

  int16_t LINFLEX_reserved22;

  union {
    vuint16_t R;
    struct {
      vuint16_t :3;
      vuint16_t DFL :3;
      vuint16_t DIR :1;
      vuint16_t CCS :1;
      vuint16_t :2;
      vuint16_t ID :6;
    } B;
  } IFCR4;

  int16_t LINFLEX_reserved23;

  union {
    vuint16_t R;
    struct {
      vuint16_t :3;
      vuint16_t DFL :3;
      vuint16_t DIR :1;
      vuint16_t CCS :1;
      vuint16_t :2;
      vuint16_t ID :6;
    } B;
  } IFCR5;

  int16_t LINFLEX_reserved24;

  union {
    vuint16_t R;
    struct {
      vuint16_t :3;
      vuint16_t DFL :3;
      vuint16_t DIR :1;
      vuint16_t CCS :1;
      vuint16_t :2;
      vuint16_t ID :6;
    } B;
  } IFCR6;

  int16_t LINFLEX_reserved25;

  union {
    vuint16_t R;
    struct {
      vuint16_t :3;
      vuint16_t DFL :3;
      vuint16_t DIR :1;
      vuint16_t CCS :1;
      vuint16_t :2;
      vuint16_t ID :6;
    } B;
  } IFCR7;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @name    LINFlex units references
 * @{
 */
#if defined(_SPC570Sxx_)
/* Locations for SPC570Sxx devices.*/
#if SPC5_HAS_LINFLEX0 || defined(__DOXYGEN__)
#define SPC5_LINFLEX0   (*(struct spc5_linflex *)0xFFE8C000UL)
#endif

#if SPC5_HAS_LINFLEX1 || defined(__DOXYGEN__)
#define SPC5_LINFLEX1   (*(struct spc5_linflex *)0xFBE8C000UL)
#endif

#elif defined(_SPC57EMxx_)
/* Locations for SPC57EMxx devices.*/
#if SPC5_HAS_LINFLEX0 || defined(__DOXYGEN__)
#define SPC5_LINFLEX0   (*(struct spc5_linflex *)0xFFE8C000UL)
#endif

#if SPC5_HAS_LINFLEX1 || defined(__DOXYGEN__)
#define SPC5_LINFLEX1   (*(struct spc5_linflex *)0xFFE90000UL)
#endif

#if SPC5_HAS_LINFLEX2 || defined(__DOXYGEN__)
#define SPC5_LINFLEX2   (*(struct spc5_linflex *)0xFBE8C000UL)
#endif

#if SPC5_HAS_LINFLEX14 || defined(__DOXYGEN__)
#define SPC5_LINFLEX14  (*(struct spc5_linflex *)0xFFEA8000UL)
#endif

#if SPC5_HAS_LINFLEX15 || defined(__DOXYGEN__)
#define SPC5_LINFLEX15  (*(struct spc5_linflex *)0xFBEA8000UL)
#endif

#elif defined(_SPC58NExx_) || defined(_SPC58ECxx_)
/* Locations for _SPC58NExx_ and _SPC58ECxx_ devices.*/
#if SPC5_HAS_LINFLEX0 || defined(__DOXYGEN__)
#define SPC5_LINFLEX0   (*(struct spc5_linflex *)0xF7E8C000UL)
#endif

#if SPC5_HAS_LINFLEX1 || defined(__DOXYGEN__)
#define SPC5_LINFLEX1   (*(struct spc5_linflex *)0xFBE8C000UL)
#endif

#if SPC5_HAS_LINFLEX2 || defined(__DOXYGEN__)
#define SPC5_LINFLEX2   (*(struct spc5_linflex *)0xF7E90000UL)
#endif

#if SPC5_HAS_LINFLEX3 || defined(__DOXYGEN__)
#define SPC5_LINFLEX3   (*(struct spc5_linflex *)0xFBE90000UL)
#endif

#if SPC5_HAS_LINFLEX4 || defined(__DOXYGEN__)
#define SPC5_LINFLEX4   (*(struct spc5_linflex *)0xF7E94000UL)
#endif

#if SPC5_HAS_LINFLEX5 || defined(__DOXYGEN__)
#define SPC5_LINFLEX5   (*(struct spc5_linflex *)0xFBE94000UL)
#endif

#if SPC5_HAS_LINFLEX6 || defined(__DOXYGEN__)
#define SPC5_LINFLEX6   (*(struct spc5_linflex *)0xF7E98000UL)
#endif

#if SPC5_HAS_LINFLEX7 || defined(__DOXYGEN__)
#define SPC5_LINFLEX7   (*(struct spc5_linflex *)0xFBE98000UL)
#endif

#if SPC5_HAS_LINFLEX8 || defined(__DOXYGEN__)
#define SPC5_LINFLEX8   (*(struct spc5_linflex *)0x0xF7E9C000UL)
#endif

#if SPC5_HAS_LINFLEX9 || defined(__DOXYGEN__)
#define SPC5_LINFLEX9   (*(struct spc5_linflex *)0xFBE9C000UL)
#endif

#if SPC5_HAS_LINFLEX10 || defined(__DOXYGEN__)
#define SPC5_LINFLEX10  (*(struct spc5_linflex *)xF7EA0000UL)
#endif

#if SPC5_HAS_LINFLEX11 || defined(__DOXYGEN__)
#define SPC5_LINFLEX11  (*(struct spc5_linflex *)0xFBEA0000UL)
#endif

#if SPC5_HAS_LINFLEX12 || defined(__DOXYGEN__)
#define SPC5_LINFLEX12  (*(struct spc5_linflex *)0xF7EA4000UL)
#endif

#if SPC5_HAS_LINFLEX13 || defined(__DOXYGEN__)
#define SPC5_LINFLEX13  (*(struct spc5_linflex *)0xFBEA4000UL)
#endif

#if SPC5_HAS_LINFLEX14 || defined(__DOXYGEN__)
#define SPC5_LINFLEX14  (*(struct spc5_linflex *)0xFFEA8000UL)
#endif

#if SPC5_HAS_LINFLEX15 || defined(__DOXYGEN__)
#define SPC5_LINFLEX15  (*(struct spc5_linflex *)0xFBEA8000UL)
#endif

#else /* !defined(_SPC570Sxx_) && !defined(_SPC57EMxx_) */
/* Default locations for SPC56x devices.*/
#if SPC5_HAS_LINFLEX0 || defined(__DOXYGEN__)
#define SPC5_LINFLEX0   (*(struct spc5_linflex *)0xFFE40000UL)
#endif

#if SPC5_HAS_LINFLEX1 || defined(__DOXYGEN__)
#define SPC5_LINFLEX1   (*(struct spc5_linflex *)0xFFE44000UL)
#endif

#if SPC5_HAS_LINFLEX2 || defined(__DOXYGEN__)
#define SPC5_LINFLEX2   (*(struct spc5_linflex *)0xFFE48000UL)
#endif

#if SPC5_HAS_LINFLEX3 || defined(__DOXYGEN__)
#define SPC5_LINFLEX3   (*(struct spc5_linflex *)0xFFE4C000UL)
#endif

#if SPC5_HAS_LINFLEX4 || defined(__DOXYGEN__)
#define SPC5_LINFLEX4   (*(struct spc5_linflex *)0xFFE50000UL)
#endif

#if SPC5_HAS_LINFLEX5 || defined(__DOXYGEN__)
#define SPC5_LINFLEX5   (*(struct spc5_linflex *)0xFFE54000UL)
#endif

#if SPC5_HAS_LINFLEX6 || defined(__DOXYGEN__)
#define SPC5_LINFLEX6   (*(struct spc5_linflex *)0xFFE58000UL)
#endif

#if SPC5_HAS_LINFLEX7 || defined(__DOXYGEN__)
#define SPC5_LINFLEX7   (*(struct spc5_linflex *)0xFFE5C000UL)
#endif

#if SPC5_HAS_LINFLEX8 || defined(__DOXYGEN__)
#define SPC5_LINFLEX8   (*(struct spc5_linflex *)0xFFFB0000UL)
#endif

#if SPC5_HAS_LINFLEX9 || defined(__DOXYGEN__)
#define SPC5_LINFLEX9   (*(struct spc5_linflex *)0xFFFB4000UL)
#endif

#endif /* !defined(_SPC57EMxx_) */

/** @} */

#endif /* _SPC5_LINFLEX_H_ */

/** @} */
