/***************************************************************** 
 * PROJECT     : MPC5604P
 * FILE        : 5604P_Header_v1_10.h
 * 
 * DESCRIPTION : This is the header file describing the register
 *               set for the named projects. 
 * 
 * COPYRIGHT   :(c) 2012, Freescale
 * 
 * VERSION	   : 01.10
 * DATE	   : 12.06.2012  
 * AUTHOR       : B16991
 * HISTORY     : Changes: typo fixed in PSR1 register:SCP -> CSP, MCR: PRESCALE->BITRATE (b16991)
 * HISTORY     : Changes: typo fixed in ME register MTC bit, SSCM fix, CMU changes(b16991)
 * HISTORY     : Changes to CTU Module:  CR register (LC->FC), CLR changed to 24 bits  (b16991)
 * HISTORY      : Modified to add reserved space in CTU (b16991)
 * HISTORY      :  Modified to support ADC on Pictus cut 2 - do not distribute! (ttz778)
 * HISTORY      :  Modified to support CRC on Pictus cut 2 - do not distribute! (r60321)
 * HISTORY      :  Modified to support DSPI0 CS7&8 and new FlexPWM naming on Pictus cut 2 (r60321)
 * HISTORY      :  Modified to update MIDR1&2 registers and LINCR1-SFTM and LINESR-BDEF bit on Pictus (r60321)
 * HISTORY      :  Modified to update RGM, CFLASH & DFLASH registers and FlexCAN & CTU Registers on Pictus (r60321)
 * HISTORY      :  Modified to update DSPI Registers (FIFO deep) on Pictus  (b16991)
 *
 ***************************************************************** 
 * Copyright:
 *	Freescale Semiconductor, INC. All Rights Reserved.
 *  You are hereby granted a copyright license to use, modify, and
 *  distribute the SOFTWARE so long as this entire notice is
 *  retained without alteration in any modified and/or redistributed
 *  versions, and that such modified versions are clearly identified
 *  as such. No licenses are granted by implication, estoppel or
 *  otherwise under any patents or trademarks of Freescale
 *  Semiconductor, Inc. This software is provided on an "AS IS"
 *  basis and without warranty.
 *
 *  To the maximum extent permitted by applicable law, Freescale
 *  Semiconductor DISCLAIMS ALL WARRANTIES WHETHER EXPRESS OR IMPLIED,
 *  INCLUDING IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A
 *  PARTICULAR PURPOSE AND ANY WARRANTY AGAINST INFRINGEMENT WITH
 *  REGARD TO THE SOFTWARE (INCLUDING ANY MODIFIED VERSIONS THEREOF)
 *  AND ANY ACCOMPANYING WRITTEN MATERIALS.
 *
 *  To the maximum extent permitted by applicable law, IN NO EVENT
 *  SHALL Freescale Semiconductor BE LIABLE FOR ANY DAMAGES WHATSOEVER
 *  (INCLUDING WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS,
 *  BUSINESS INTERRUPTION, LOSS OF BUSINESS INFORMATION, OR OTHER
 *  PECUNIARY LOSS) ARISING OF THE USE OR INABILITY TO USE THE SOFTWARE.
 *
 *  Freescale Semiconductor assumes no responsibility for the
 *  maintenance and support of this software
 *
 ******************************************************************/
/***************************************************************** 
* Example instantiation and use:            
*                                           
*  <MODULE>.<REGISTER>.B.<BIT> = 1;         
*  <MODULE>.<REGISTER>.R       = 0x10000000;
*                                           
******************************************************************/

#ifndef _JDP_H_
#define _JDP_H_

#include "typedefs.h"

#ifdef  __cplusplus
extern "C" {
#endif

#ifdef __MWERKS__
#pragma push
#pragma ANSI_strict off
#endif
/****************************************************************************/
/*                          MODULE : ADC                                   */
/****************************************************************************/
    struct ADC_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t OWREN:1;
                vuint32_t WLSIDE:1;
                vuint32_t MODE:1;
                vuint32_t EDGLEV:1;
                vuint32_t TRGEN:1;
                vuint32_t EDGE:1;
                vuint32_t XSTRTEN:1;
                vuint32_t NSTART:1;
                vuint32_t:1;
                vuint32_t JTRGEN:1;
                vuint32_t JEDGE:1;
                vuint32_t JSTART:1;
                vuint32_t:2;
                vuint32_t CTUEN:1;
                vuint32_t:8;
                vuint32_t ADCLKSEL:1;
                vuint32_t ABORTCHAIN:1;
                vuint32_t ABORT:1;
                vuint32_t ACK0:1;
                vuint32_t OFFREFRESH:1;
                vuint32_t OFFCANC:1;
                vuint32_t:2;
                vuint32_t PWDN:1;
            } B;
        } MCR;                 /* MAIN CONFIGURATION REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:7;
                vuint32_t NSTART:1;
                vuint32_t JABORT:1;
                vuint32_t:2;
                vuint32_t JSTART:1;
                vuint32_t:3;
                vuint32_t CTUSTART:1;
                vuint32_t CHADDR:7;
                vuint32_t:3;
                vuint32_t ACK0:1;
                vuint32_t OFFREFRESH:1;
                vuint32_t OFFCANC:1;
                vuint32_t ADCSTATUS:3;
            } B;
        } MSR;                 /* MAIN STATUS REGISTER */

        int32_t ADC_reserved1[2];       /* (0x008 - 0x00F)/4 = 0x02 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:25;
                vuint32_t OFFCANCOVR:1;
                vuint32_t EOFFSET:1;
                vuint32_t EOCTU:1;
                vuint32_t JEOC:1;
                vuint32_t JECH:1;
                vuint32_t EOC:1;
                vuint32_t ECH:1;
            } B;
        } ISR;                 /* INTERRUPT STATUS REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t EOC31:1;
                vuint32_t EOC30:1;
                vuint32_t EOC29:1;
                vuint32_t EOC28:1;
                vuint32_t EOC27:1;
                vuint32_t EOC26:1;
                vuint32_t EOC25:1;
                vuint32_t EOC24:1;
                vuint32_t EOC23:1;
                vuint32_t EOC22:1;
                vuint32_t EOC21:1;
                vuint32_t EOC20:1;
                vuint32_t EOC19:1;
                vuint32_t EOC18:1;
                vuint32_t EOC17:1;
                vuint32_t EOC16:1;
                vuint32_t EOC15:1;
                vuint32_t EOC14:1;
                vuint32_t EOC13:1;
                vuint32_t EOC12:1;
                vuint32_t EOC11:1;
                vuint32_t EOC10:1;
                vuint32_t EOC9:1;
                vuint32_t EOC8:1;
                vuint32_t EOC7:1;
                vuint32_t EOC6:1;
                vuint32_t EOC5:1;
                vuint32_t EOC4:1;
                vuint32_t EOC3:1;
                vuint32_t EOC2:1;
                vuint32_t EOC1:1;
                vuint32_t EOC0:1;
            } B;
        } CEOCFR[3];                 /* Channel Pending Register 0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:25;			//One bit added
                vuint32_t MSKOFFCANCOVR:1;	//Moved up
                vuint32_t MSKEOFFSET:1;		//Moved up
                vuint32_t MSKEOCTU:1;		//New for cut 2
                vuint32_t MSKJEOC:1;
                vuint32_t MSKJECH:1;
                vuint32_t MSKEOC:1;
                vuint32_t MSKECH:1;
            } B;
        } IMR;                  /* INTERRUPT MASK REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t CIM31:1;
                vuint32_t CIM30:1;
                vuint32_t CIM29:1;
                vuint32_t CIM28:1;
                vuint32_t CIM27:1;
                vuint32_t CIM26:1;
                vuint32_t CIM25:1;
                vuint32_t CIM24:1;
                vuint32_t CIM23:1;
                vuint32_t CIM22:1;
                vuint32_t CIM21:1;
                vuint32_t CIM20:1;
                vuint32_t CIM19:1;
                vuint32_t CIM18:1;
                vuint32_t CIM17:1;
                vuint32_t CIM16:1;
                vuint32_t CIM15:1;
                vuint32_t CIM14:1;
                vuint32_t CIM13:1;
                vuint32_t CIM12:1;
                vuint32_t CIM11:1;
                vuint32_t CIM10:1;
                vuint32_t CIM9:1;
                vuint32_t CIM8:1;
                vuint32_t CIM7:1;
                vuint32_t CIM6:1;
                vuint32_t CIM5:1;
                vuint32_t CIM4:1;
                vuint32_t CIM3:1;
                vuint32_t CIM2:1;
                vuint32_t CIM1:1;
                vuint32_t CIM0:1;
            } B;
        } CIMR[3];                 /* Channel Interrupt Mask Register 0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t WDG3H:1;
                vuint32_t WDG2H:1;
                vuint32_t WDG1H:1;
                vuint32_t WDG0H:1;
                vuint32_t WDG3L:1;
                vuint32_t WDG2L:1;
                vuint32_t WDG1L:1;
                vuint32_t WDG0L:1;
            } B;
        } WTISR;               /* WATCHDOG INTERRUPT THRESHOLD REGISTER was WDGTHR */

        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t MSKWDG3H:1;
                vuint32_t MSKWDG2H:1;
                vuint32_t MSKWDG1H:1;
                vuint32_t MSKWDG0H:1;
                vuint32_t MSKWDG3L:1;
                vuint32_t MSKWDG2L:1;
                vuint32_t MSKWDG1L:1;
                vuint32_t MSKWDG0L:1;
            } B;
        } WTIMR;             /* WATCHDOG INTERRUPT MASK REGISTER was IMWDGTHR */

        int32_t ADC_reserved2[2];       /* (0x038 - 0x03F)/4 = 0x02 */

	union {
            vuint32_t R;
            struct {
                vuint32_t:30;			//was 16
                vuint32_t DCLR:1;		//moved
                vuint32_t DMAEN:1;		//moved
            } B;
        } DMAE;                 /* DMAE REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t DMA31:1;	//was unused [16]
                vuint32_t DMA30:1;
                vuint32_t DMA29:1;
                vuint32_t DMA28:1;
                vuint32_t DMA27:1;
                vuint32_t DMA26:1;
                vuint32_t DMA25:1;
                vuint32_t DMA24:1;
                vuint32_t DMA23:1;
                vuint32_t DMA22:1;
                vuint32_t DMA21:1;
                vuint32_t DMA20:1;
                vuint32_t DMA19:1;
                vuint32_t DMA18:1;
                vuint32_t DMA17:1;
                vuint32_t DMA16:1;
                vuint32_t DMA15:1;
                vuint32_t DMA14:1;
                vuint32_t DMA13:1;
                vuint32_t DMA12:1;
                vuint32_t DMA11:1;
                vuint32_t DMA10:1;
                vuint32_t DMA9:1;
                vuint32_t DMA8:1;
                vuint32_t DMA7:1;
                vuint32_t DMA6:1;
                vuint32_t DMA5:1;
                vuint32_t DMA4:1;
                vuint32_t DMA3:1;
                vuint32_t DMA2:1;
                vuint32_t DMA1:1;
                vuint32_t DMA0:1;
            } B;
        } DMAR[3];              /* DMA REGISTER  was [6] */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t THREN:1;
                vuint32_t THRINV:1;
                vuint32_t THROP:1;
                  vuint32_t:6;
                vuint32_t THRCH:7;
            } B;
        } TRC[4];               /* ADC THRESHOLD REGISTER REGISTER */

        union {
            vuint32_t R;
            struct {		//were in TRA & TRB
                vuint32_t:4;
                vuint32_t THRH:12;
                vuint32_t:4;
                vuint32_t THRL:12;
            } B;
        } THRHLR[4];               /* THRESHOLD REGISTER */

        union {
            vuint32_t R;
            struct {		//were in TRAALT & TRBALT
                vuint32_t:4;
                vuint32_t THRH:12;
                vuint32_t:4;
                vuint32_t THRL:12;
            } B;
        } THRALT[4];            /* ADC THRESHOLD REGISTER REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:25;		//was 26
                vuint32_t PREVAL2:2;	
                vuint32_t PREVAL1:2;
                vuint32_t PREVAL0:2;
                vuint32_t PREONCE:1;
            } B;
        } PSCR;               /* PRESAMPLING CONTROL REGISTER was PREREG */

        union {
            vuint32_t R;
            struct {
                vuint32_t PRES31:1;	//was reserved 16
                vuint32_t PRES30:1;
                vuint32_t PRES29:1;
                vuint32_t PRES28:1;
                vuint32_t PRES27:1;
                vuint32_t PRES26:1;
                vuint32_t PRES25:1;
                vuint32_t PRES24:1;
                vuint32_t PRES23:1;
                vuint32_t PRES22:1;
                vuint32_t PRES21:1;
                vuint32_t PRES20:1;
                vuint32_t PRES19:1;
                vuint32_t PRES18:1;
                vuint32_t PRES17:1;
                vuint32_t PRES16:1;
                vuint32_t PRES15:1;
                vuint32_t PRES14:1;
                vuint32_t PRES13:1;
                vuint32_t PRES12:1;
                vuint32_t PRES11:1;
                vuint32_t PRES10:1;
                vuint32_t PRES9:1;
                vuint32_t PRES8:1;
                vuint32_t PRES7:1;
                vuint32_t PRES6:1;
                vuint32_t PRES5:1;
                vuint32_t PRES4:1;
                vuint32_t PRES3:1;
                vuint32_t PRES2:1;
                vuint32_t PRES1:1;
                vuint32_t PRES0:1;
            } B;
        } PSR[3];              /* PRESAMPLING REGISTER was PRER[6]*/

        int32_t ADC_reserved3[1];       /* (0x090 - 0x093)/4 = 0x01 */

	union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t INPLATCH:1;
                  vuint32_t:1;
                vuint32_t OFFSHIFT:2;		//!!! This field only in CTR[0]
                  vuint32_t:1;
                vuint32_t INPCMP:2;
                  vuint32_t:1;
                vuint32_t INPSAMP:8;
            } B;
        } CTR[3];                /* CONVERSION TIMING REGISTER was CT[3] */

        int32_t ADC_reserved4[1];       /* (0x0A0 - 0x0A3)/4 = 0x01 */

	union {
            vuint32_t R;
            struct {
                vuint32_t CH31:1;		//was reserved 16
                vuint32_t CH30:1;
                vuint32_t CH29:1;
                vuint32_t CH28:1;
                vuint32_t CH27:1;
                vuint32_t CH26:1;
                vuint32_t CH25:1;
                vuint32_t CH24:1;
                vuint32_t CH23:1;
                vuint32_t CH22:1;
                vuint32_t CH21:1;
                vuint32_t CH20:1;
                vuint32_t CH19:1;
                vuint32_t CH18:1;
                vuint32_t CH17:1;
                vuint32_t CH16:1;
                vuint32_t CH15:1;
                vuint32_t CH14:1;
                vuint32_t CH13:1;
                vuint32_t CH12:1;
                vuint32_t CH11:1;
                vuint32_t CH10:1;
                vuint32_t CH9:1;
                vuint32_t CH8:1;
                vuint32_t CH7:1;
                vuint32_t CH6:1;
                vuint32_t CH5:1;
                vuint32_t CH4:1;
                vuint32_t CH3:1;
                vuint32_t CH2:1;
                vuint32_t CH1:1;
                vuint32_t CH0:1;
            } B;
        } NCMR[3];              /* NORMAL CONVERSION MASK REGISTER was [6] */

        int32_t ADC_reserved5[1];       /* (0x0B0 - 0x0B3)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CH31:1;		//was reserved 16
                vuint32_t CH30:1;
                vuint32_t CH29:1;
                vuint32_t CH28:1;
                vuint32_t CH27:1;
                vuint32_t CH26:1;
                vuint32_t CH25:1;
                vuint32_t CH24:1;
                vuint32_t CH23:1;
                vuint32_t CH22:1;
                vuint32_t CH21:1;
                vuint32_t CH20:1;
                vuint32_t CH19:1;
                vuint32_t CH18:1;
                vuint32_t CH17:1;
                vuint32_t CH16:1;
                vuint32_t CH15:1;
                vuint32_t CH14:1;
                vuint32_t CH13:1;
                vuint32_t CH12:1;
                vuint32_t CH11:1;
                vuint32_t CH10:1;
                vuint32_t CH9:1;
                vuint32_t CH8:1;
                vuint32_t CH7:1;
                vuint32_t CH6:1;
                vuint32_t CH5:1;
                vuint32_t CH4:1;
                vuint32_t CH3:1;
                vuint32_t CH2:1;
                vuint32_t CH1:1;
                vuint32_t CH0:1;
            } B;
        } JCMR[3];              /* Injected CONVERSION MASK REGISTER was ICMR[6] */

        union {
            vuint32_t R;
            struct {
                vuint32_t:15;
                vuint32_t OFFSETLOAD:1;		//new
                vuint32_t:8;
                vuint32_t OFFSETWORD:8;
            } B;
        } OFFWR;               /* OFFSET WORD REGISTER was OFFREG*/

        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t DSD:8;
            } B;
        } DSDR;                  /* DECODE SIGNALS DELAY REGISTER was DSD */

        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t PDED:8;	//was PDD
            } B;
        } PDEDR;                  /* POWER DOWN DELAY REGISTER was PDD */

        int32_t ADC_reserved6[9];       /* (0x0CC - 0x0EF)/4 = 0x09 */

        union {
            vuint32_t R;
            struct {
                vuint32_t TEST_CTL:32;
            } B;
        } TCTLR;                 /* Test control REGISTER */

        int32_t ADC_reserved7[3];       /* (0x0F4 - 0x0FF)/4 = 0x03 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t VALID:1;
                vuint32_t OVERW:1;
                vuint32_t RESULT:2;
                vuint32_t:4;
                vuint32_t CDATA:12;
            } B;
        } CDR[96];      /* Channel 0-95 Data REGISTER */

    };                          /* end of ADC_tag */
/****************************************************************************/
/*                          MODULE : CANSP                                   */
/****************************************************************************/
    struct CANSP_tag {
        union {
            vuint16_t R;
            struct {
                vuint16_t RX_COMPLETE:1;
                vuint16_t BUSY:1;
                vuint16_t ACTIVE_CK:1;
                  vuint16_t:3;
                vuint16_t MODE:1;
                vuint16_t CAN_RX_SEL:3;
                vuint16_t BRP:5;
                vuint16_t CAN_SMPLR_EN:1;
            } B;
        } CR;                   /* CANSP Control Register */

        int16_t CANSP_reserved;

        union {
            vuint32_t R;
        } SR[12];               /* CANSP Sample Register 0 to 11 */

    };                          /* end of CANSP_tag */
/****************************************************************************/
/*                          MODULE : MCM                                   */
/****************************************************************************/
    struct MCM_tag {

        union {
            vuint16_t R;
        } PCT;                  /* MCM Processor Core Type Register */

        union {
            vuint16_t R;
        } REV;                  /* MCM  Revision Register */

        int32_t MCM_reserved;

        union {
            vuint32_t R;
        } MC;                   /* MCM Configuration Register */

        int8_t MCM_reserved1[3];

        union {
            vuint8_t R;
            struct {
                vuint8_t POR:1;
                vuint8_t DIR:1;
                  vuint8_t:6;
            } B;
        } MRSR;                 /* MCM Miscellaneous Reset Status Register */

        int8_t MCM_reserved2[3];

        union {
            vuint8_t R;
            struct {
                vuint8_t ENBWCR:1;
                  vuint8_t:3;
                vuint8_t PRILVL:4;
            } B;
        } MWCR;                 /* MCM Miscellaneous Wakeup Control Register */

        int32_t MCM_reserved3[2];
        int8_t MCM_reserved4[3];

        union {
            vuint8_t R;
            struct {
                vuint8_t FB0AI:1;
                vuint8_t FB0SI:1;
                vuint8_t FB1AI:1;
                vuint8_t FB1SI:1;
                  vuint8_t:4;
            } B;
        } MIR;                  /* MCM Miscellaneous Interrupt Register */

        int32_t MCM_reserved5;

        union {
            vuint32_t R;
        } MUDCR;                /* MCM Miscellaneous User-Defined Control Register */

        int32_t MCM_reserved6[6];       /* (0x040- 0x028)/4 = 0x06 */
        int8_t MCM_reserved7[3];

        union {
            vuint8_t R;
            struct {
                vuint8_t:2;
                vuint8_t ER1BR:1;
                vuint8_t EF1BR:1;
                  vuint8_t:2;
                vuint8_t ERNCR:1;
                vuint8_t EFNCR:1;
            } B;
        } ECR;                  /* MCM ECC Configuration Register */

        int8_t MCM_reserved8[3];

        union {
            vuint8_t R;
            struct {
                vuint8_t:2;
                vuint8_t R1BC:1;
                vuint8_t F1BC:1;
                  vuint8_t:2;
                vuint8_t RNCE:1;
                vuint8_t FNCE:1;
            } B;
        } ESR;                  /* MCM ECC Status Register */

        int16_t MCM_reserved9;

        union {
            vuint16_t R;
            struct {
                vuint16_t:2;
                vuint16_t FRC1BI:1;
                vuint16_t FR11BI:1;
                  vuint16_t:2;
                vuint16_t FRCNCI:1;
                vuint16_t FR1NCI:1;
                  vuint16_t:1;
                vuint16_t ERRBIT:7;
            } B;
        } EEGR;                 /* MCM ECC Error Generation Register */

        int32_t MCM_reserved10;

        union {
            vuint32_t R;
        } FEAR;                 /* MCM Flash ECC Address Register */

        int16_t MCM_reserved11;

        union {
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t FEMR:4;
            } B;
        } FEMR;                 /* MCM Flash ECC Master Number Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t WRITE:1;
                vuint8_t SIZE:3;
                vuint8_t PROTECTION:4;
            } B;
        } FEAT;                 /* MCM Flash ECC Attributes Register */

        int32_t MCM_reserved12;

        union {
            vuint32_t R;
        } FEDR;                 /* MCM Flash ECC Data Register */

        union {
            vuint32_t R;
        } REAR;                 /* MCM RAM ECC Address Register */

        int8_t MCM_reserved13;

        union {
            vuint8_t R;
        } RESR;                 /* MCM RAM ECC Address Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t REMR:4;
            } B;
        } REMR;                 /* MCM RAM ECC Master Number Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t WRITE:1;
                vuint8_t SIZE:3;
                vuint8_t PROTECTION:4;
            } B;
        } REAT;                 /* MCM RAM ECC Attributes Register */

        int32_t MCM_reserved14;

        union {
            vuint32_t R;
        } REDR;                 /* MCM RAM ECC Data Register */

    };                          /* end of MCM_tag */
/****************************************************************************/
/*                          MODULE : RTC                                   */
/****************************************************************************/
    struct RTC_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t SUPV:1;
                  vuint32_t:31;
            } B;
        } RTCSUPV;              /* RTC Supervisor Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t CNTEN:1;
                vuint32_t RTCIE:1;
                vuint32_t FRZEN:1;
                vuint32_t ROVREN:1;
                vuint32_t RTCVAL:12;
                vuint32_t APIEN:1;
                vuint32_t APIE:1;
                vuint32_t CLKSEL:2;
                vuint32_t DIV512EN:1;
                vuint32_t DIV32EN:1;
                vuint32_t APIVAL:10;
            } B;
        } RTCC;                 /* RTC Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:2;
                vuint32_t RTCF:1;
                  vuint32_t:15;
                vuint32_t APIF:1;
                  vuint32_t:2;
                vuint32_t ROVRF:1;
                  vuint32_t:10;
            } B;
        } RTCS;                 /* RTC Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t RTCCNT:32;
            } B;
        } RTCCNT;               /* RTC Counter Register */

    };                          /* end of RTC_tag */
/****************************************************************************/
/*                     MODULE : SIU                                         */
/****************************************************************************/
    struct SIU_tag {

        int32_t SIU_reserved0;

        union {                 /* MCU ID Register 1 */
            vuint32_t R;
            struct {
                vuint32_t PARTNUM:16;
		vuint32_t CSP:1;
                vuint32_t PKG:5;
		vuint32_t:2;
                vuint32_t MAJORMASK:4;
		vuint32_t MINORMASK:4;
            } B;
        } MIDR;

        union {                 /* MCU ID Register 2 */
            vuint32_t R;
            struct {
                vuint32_t SF:1;
                vuint32_t FLASH_SIZE_1:4;
                vuint32_t FLASH_SIZE_2:4;
		vuint32_t:7;
                vuint32_t PARTNUM:8;
		vuint32_t:3;
                vuint32_t EE:1;
		vuint32_t:3;
		vuint32_t FR:1;
            } B;
        } MIDR2;

        int32_t SIU_reserved1[2];

        union {                 /* Interrupt Status Flag Register */
            vuint32_t R;
            struct {
                vuint32_t EIF31:1;
                vuint32_t EIF30:1;
                vuint32_t EIF29:1;
                vuint32_t EIF28:1;
                vuint32_t EIF27:1;
                vuint32_t EIF26:1;
                vuint32_t EIF25:1;
                vuint32_t EIF24:1;
                vuint32_t EIF23:1;
                vuint32_t EIF22:1;
                vuint32_t EIF21:1;
                vuint32_t EIF20:1;
                vuint32_t EIF19:1;
                vuint32_t EIF18:1;
                vuint32_t EIF17:1;
                vuint32_t EIF16:1;
                vuint32_t EIF15:1;
                vuint32_t EIF14:1;
                vuint32_t EIF13:1;
                vuint32_t EIF12:1;
                vuint32_t EIF11:1;
                vuint32_t EIF10:1;
                vuint32_t EIF9:1;
                vuint32_t EIF8:1;
                vuint32_t EIF7:1;
                vuint32_t EIF6:1;
                vuint32_t EIF5:1;
                vuint32_t EIF4:1;
                vuint32_t EIF3:1;
                vuint32_t EIF2:1;
                vuint32_t EIF1:1;
                vuint32_t EIF0:1;
            } B;
        } ISR;

        union {                 /* Interrupt Request Enable Register */
            vuint32_t R;
            struct {
                vuint32_t EIRE31:1;
                vuint32_t EIRE30:1;
                vuint32_t EIRE29:1;
                vuint32_t EIRE28:1;
                vuint32_t EIRE27:1;
                vuint32_t EIRE26:1;
                vuint32_t EIRE25:1;
                vuint32_t EIRE24:1;
                vuint32_t EIRE23:1;
                vuint32_t EIRE22:1;
                vuint32_t EIRE21:1;
                vuint32_t EIRE20:1;
                vuint32_t EIRE19:1;
                vuint32_t EIRE18:1;
                vuint32_t EIRE17:1;
                vuint32_t EIRE16:1;
                vuint32_t EIRE15:1;
                vuint32_t EIRE14:1;
                vuint32_t EIRE13:1;
                vuint32_t EIRE12:1;
                vuint32_t EIRE11:1;
                vuint32_t EIRE10:1;
                vuint32_t EIRE9:1;
                vuint32_t EIRE8:1;
                vuint32_t EIRE7:1;
                vuint32_t EIRE6:1;
                vuint32_t EIRE5:1;
                vuint32_t EIRE4:1;
                vuint32_t EIRE3:1;
                vuint32_t EIRE2:1;
                vuint32_t EIRE1:1;
                vuint32_t EIRE0:1;
            } B;
        } IRER;

        int32_t SIU_reserved2[3];

        union {                 /* Interrupt Rising-Edge Event Enable Register */
            vuint32_t R;
            struct {
                vuint32_t IREE31:1;
                vuint32_t IREE30:1;
                vuint32_t IREE29:1;
                vuint32_t IREE28:1;
                vuint32_t IREE27:1;
                vuint32_t IREE26:1;
                vuint32_t IREE25:1;
                vuint32_t IREE24:1;
                vuint32_t IREE23:1;
                vuint32_t IREE22:1;
                vuint32_t IREE21:1;
                vuint32_t IREE20:1;
                vuint32_t IREE19:1;
                vuint32_t IREE18:1;
                vuint32_t IREE17:1;
                vuint32_t IREE16:1;
                vuint32_t IREE15:1;
                vuint32_t IREE14:1;
                vuint32_t IREE13:1;
                vuint32_t IREE12:1;
                vuint32_t IREE11:1;
                vuint32_t IREE10:1;
                vuint32_t IREE9:1;
                vuint32_t IREE8:1;
                vuint32_t IREE7:1;
                vuint32_t IREE6:1;
                vuint32_t IREE5:1;
                vuint32_t IREE4:1;
                vuint32_t IREE3:1;
                vuint32_t IREE2:1;
                vuint32_t IREE1:1;
                vuint32_t IREE0:1;
            } B;
        } IREER;

        union {                 /* Interrupt Falling-Edge Event Enable Register */
            vuint32_t R;
            struct {
                vuint32_t IFEE31:1;
                vuint32_t IFEE30:1;
                vuint32_t IFEE29:1;
                vuint32_t IFEE28:1;
                vuint32_t IFEE27:1;
                vuint32_t IFEE26:1;
                vuint32_t IFEE25:1;
                vuint32_t IFEE24:1;
                vuint32_t IFEE23:1;
                vuint32_t IFEE22:1;
                vuint32_t IFEE21:1;
                vuint32_t IFEE20:1;
                vuint32_t IFEE19:1;
                vuint32_t IFEE18:1;
                vuint32_t IFEE17:1;
                vuint32_t IFEE16:1;
                vuint32_t IFEE15:1;
                vuint32_t IFEE14:1;
                vuint32_t IFEE13:1;
                vuint32_t IFEE12:1;
                vuint32_t IFEE11:1;
                vuint32_t IFEE10:1;
                vuint32_t IFEE9:1;
                vuint32_t IFEE8:1;
                vuint32_t IFEE7:1;
                vuint32_t IFEE6:1;
                vuint32_t IFEE5:1;
                vuint32_t IFEE4:1;
                vuint32_t IFEE3:1;
                vuint32_t IFEE2:1;
                vuint32_t IFEE1:1;
                vuint32_t IFEE0:1;
            } B;
        } IFEER;

        union {                 /* Interrupt Filter Enable Register */
            vuint32_t R;
            struct {
                vuint32_t IFE31:1;
                vuint32_t IFE30:1;
                vuint32_t IFE29:1;
                vuint32_t IFE28:1;
                vuint32_t IFE27:1;
                vuint32_t IFE26:1;
                vuint32_t IFE25:1;
                vuint32_t IFE24:1;
                vuint32_t IFE23:1;
                vuint32_t IFE22:1;
                vuint32_t IFE21:1;
                vuint32_t IFE20:1;
                vuint32_t IFE19:1;
                vuint32_t IFE18:1;
                vuint32_t IFE17:1;
                vuint32_t IFE16:1;
                vuint32_t IFE15:1;
                vuint32_t IFE14:1;
                vuint32_t IFE13:1;
                vuint32_t IFE12:1;
                vuint32_t IFE11:1;
                vuint32_t IFE10:1;
                vuint32_t IFE9:1;
                vuint32_t IFE8:1;
                vuint32_t IFE7:1;
                vuint32_t IFE6:1;
                vuint32_t IFE5:1;
                vuint32_t IFE4:1;
                vuint32_t IFE3:1;
                vuint32_t IFE2:1;
                vuint32_t IFE1:1;
                vuint32_t IFE0:1;
            } B;
        } IFER;

        int32_t SIU_reserved3[3];

        union {                 /* Pad Configuration Registers */
            vuint16_t R;
            struct {
                vuint16_t:1;
                vuint16_t SME:1;
                vuint16_t APC:1;
                  vuint16_t:1;
                vuint16_t PA:2;
                vuint16_t OBE:1;
                vuint16_t IBE:1;
                vuint16_t DCS:2;
                vuint16_t ODE:1;
                vuint16_t HYS:1;
                vuint16_t SRC:2;
                vuint16_t WPE:1;
                vuint16_t WPS:1;
            } B;
        } PCR[512];

        int32_t SIU_reserved4[48];      /* {0x500-0x440}/0x4 */

        union {                 /* Pad Selection for Multiplexed Input Register */
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t PADSEL:4;
            } B;
        } PSMI[256];

        union {                 /* GPIO Pin Data Output Registers */
            vuint8_t R;
            struct {
                vuint8_t:7;
                vuint8_t PDO:1;
            } B;
        } GPDO[512];

        union {                 /* GPIO Pin Data Input Registers */
            vuint8_t R;
            struct {
                vuint8_t:7;
                vuint8_t PDI:1;
            } B;
        } GPDI[512];

        int32_t SIU_reserved5[128];     /* {0xC00-0xA00}/0x4 */

        union {                 /* Parallel GPIO Pin Data Output Register */
            vuint32_t R;
            struct {
                vuint32_t PPD0:32;
            } B;
        } PGPDO[16];

        union {                 /* Parallel GPIO Pin Data Input Register */
            vuint32_t R;
            struct {
                vuint32_t PPDI:32;
            } B;
        } PGPDI[16];

        union {                 /* Masked Parallel GPIO Pin Data Out Register */
            vuint32_t R;
            struct {
                vuint32_t MASK:16;
                vuint32_t MPPDO:16;
            } B;
        } MPGPDO[32];

        int32_t SIU_reserved6[192];     /* {0x1000-0x0D00}/0x4 */

        union {                 /* Interrupt Filter Maximum Counter Register */
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t MAXCNT:4;
            } B;
        } IFMC[32];

        union {                 /* Interrupt Filter Clock Prescaler Register */
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t IFCP:4;
            } B;
        } IFCPR;

    };                          /* end of SIU_tag */
/****************************************************************************/
/*                          MODULE : SSCM                                   */
/****************************************************************************/
    struct SSCM_tag {
        union {
            vuint16_t R;
            struct {
                vuint16_t:4;
                vuint16_t NXEN:1;
                  vuint16_t PUB:1;
                vuint16_t SEC:1;
                  vuint16_t:1;
                vuint16_t BMODE:3;
                vuint16_t:1;
                vuint16_t ABD:1;
                  vuint16_t:3;
            } B;
        } STATUS;               /* Status Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t SRAMSIZE:5;
                vuint16_t IFLASHSIZE:5;
                vuint16_t IVLD:1;
                vuint16_t DFLASHSIZE:4;
                vuint16_t DVLD:1;
            } B;
        } MEMCONFIG;            /* System Memory Configuration Register */

        int16_t SSCM_reserved;

        union {
            vuint16_t R;
            struct {
                vuint16_t:14;
                vuint16_t PAE:1;
                vuint16_t RAE:1;
            } B;
        } ERROR;                /* Error Configuration Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:13;
                vuint16_t DEBUG_MODE:3;
            } B;
        } DEBUGPORT;            /* Debug Status Port Register */

        int16_t SSCM_reserved1;

        union {
            vuint32_t R;
            struct {
                vuint32_t PWD_HI:32;
            } B;
        } PWCMPH;               /* Password Comparison Register High Word */

        union {
            vuint32_t R;
            struct {
                vuint32_t PWD_LO:32;
            } B;
        } PWCMPL;               /* Password Comparison Register Low Word */

    };                          /* end of SSCM_tag */
/****************************************************************************/
/*                          MODULE : STM                                   */
/****************************************************************************/
    struct STM_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t CPS:8;
                  vuint32_t:6;
                vuint32_t FRZ:1;
                vuint32_t TEN:1;
            } B;
        } CR0;                  /* STM Control Register */

        union {
            vuint32_t R;
        } CNT0;                 /* STM Count Register */

        int32_t STM_reserved[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR0;                 /* STM Channel Control Register 0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR0;                 /* STM Channel Interrupt Register 0 */

        union {
            vuint32_t R;
        } CMP0;                 /* STM Channel Compare Register 0 */

        int32_t STM_reserved1;

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR1;                 /* STM Channel Control Register 1 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR1;                 /* STM Channel Interrupt Register 1 */

        union {
            vuint32_t R;
        } CMP1;                 /* STM Channel Compare Register 1 */

        int32_t STM_reserved2;

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR2;                 /* STM Channel Control Register 2 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR2;                 /* STM Channel Interrupt Register 2 */

        union {
            vuint32_t R;
        } CMP2;                 /* STM Channel Compare Register 2 */

        int32_t STM_reserved3;

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR3;                 /* STM Channel Control Register 3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR3;                 /* STM Channel Interrupt Register 3 */

        union {
            vuint32_t R;
        } CMP3;                 /* STM Channel Compare Register 3 */

    };                          /* end of STM_tag */
/****************************************************************************/
/*                          MODULE : SWT                                   */
/****************************************************************************/
    struct SWT_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t MAP0:1;
                vuint32_t MAP1:1;
                vuint32_t MAP2:1;
                vuint32_t MAP3:1;
                vuint32_t MAP4:1;
                vuint32_t MAP5:1;
                vuint32_t MAP6:1;
                vuint32_t MAP7:1;
                  vuint32_t:15;
                vuint32_t RIA:1;
                vuint32_t WND:1;
                vuint32_t ITR:1;
                vuint32_t HLK:1;
                vuint32_t SLK:1;
                vuint32_t CSL:1;
                vuint32_t STP:1;
                vuint32_t FRZ:1;
                vuint32_t WEN:1;
            } B;
        } CR;                   /* SWT Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t TIF:1;
            } B;
        } IR;                   /* SWT Interrupt Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t WTO:32;
            } B;
        } TO;                   /* SWT Time-Out Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t WST:32;
            } B;
        } WN;                   /* SWT Window Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t WSC:16;
            } B;
        } SR;                   /* SWT Service Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t CNT:32;
            } B;
        } CO;                   /* SWT Counter Output Register */

    };                          /* end of SWT_tag */
/****************************************************************************/
/*                          MODULE : WKUP                                   */
/****************************************************************************/
    struct WKUP_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t NIF0:1;
                vuint32_t NOVF0:1;
                  vuint32_t:6;
                vuint32_t NIF1:1;
                vuint32_t NOVF1:1;
                  vuint32_t:6;
                vuint32_t NIF2:1;
                vuint32_t NOVF2:1;
                  vuint32_t:6;
                vuint32_t NIF3:1;
                vuint32_t NOVF3:1;
                  vuint32_t:6;
            } B;
        } NSR;                  /* NMI Status Register */

        int32_t WKUP_reserved;

        union {
            vuint32_t R;
            struct {
                vuint32_t NLOCK0:1;
                vuint32_t NDSS0:2;
                vuint32_t NWRE0:1;
                  vuint32_t:1;
                vuint32_t NREE0:1;
                vuint32_t NFEE0:1;
                vuint32_t NFE0:1;
                vuint32_t NLOCK1:1;
                vuint32_t NDSS1:2;
                vuint32_t NWRE1:1;
                  vuint32_t:1;
                vuint32_t NREE1:1;
                vuint32_t NFEE1:1;
                vuint32_t NFE1:1;
                vuint32_t NLOCK2:1;
                vuint32_t NDSS2:2;
                vuint32_t NWRE2:1;
                  vuint32_t:1;
                vuint32_t NREE2:1;
                vuint32_t NFEE2:1;
                vuint32_t NFE2:1;
                vuint32_t NLOCK3:1;
                vuint32_t NDSS3:2;
                vuint32_t NWRE3:1;
                  vuint32_t:1;
                vuint32_t NREE3:1;
                vuint32_t NFEE3:1;
                vuint32_t NFE3:1;
            } B;
        } NCR;                  /* NMI Configuration Register */

        int32_t WKUP_reserved1[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t EIF:32;
            } B;
        } WISR;                 /* Wakeup/Interrupt Status Flag Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t EIRE:32;
            } B;
        } IRER;                 /* Interrupt Request Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t WRE:32;
            } B;
        } WRER;                 /* Wakeup Request Enable Register */

        int32_t WKUP_reserved2[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t IREE:32;
            } B;
        } WIREER;               /* Wakeup/Interrupt Rising-Edge Event Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t IFEE:32;
            } B;
        } WIFEER;               /* Wakeup/Interrupt Falling-Edge Event Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t IFE:32;
            } B;
        } WIFER;                /* Wakeup/Interrupt Filter Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t IPUE:32;
            } B;
        } WIPUER;               /* Wakeup/Interrupt Pullup Enable Register */

    };                          /* end of WKUP_tag */
/****************************************************************************/
/*                          MODULE : LINFLEX                                */
/****************************************************************************/

    struct LINFLEX_tag {

        int16_t LINFLEX_reserved1;

        union {
            vuint16_t R;
            struct {
                vuint16_t CCD:1;
                vuint16_t CFD:1;
                vuint16_t LASE:1;
                vuint16_t AWUM:1;       // LCH vuint16_t AUTOWU:1;
                vuint16_t MBL:4;
                vuint16_t BF:1;
                vuint16_t SFTM:1;
                vuint16_t LBKM:1;
                vuint16_t MME:1;
                vuint16_t SBDT:1;       // LCH vuint16_t SSBL:1;
                vuint16_t RBLM:1;
                vuint16_t SLEEP:1;
                vuint16_t INIT:1;
            } B;
        } LINCR1;               /* LINFLEX LIN Control Register 1 */

        int16_t LINFLEX_reserved2;

        union {
            vuint16_t R;
            struct {
                vuint16_t SZIE:1;
                vuint16_t OCIE:1;
                vuint16_t BEIE:1;
                vuint16_t CEIE:1;
                vuint16_t HEIE:1;
                  vuint16_t:2;
                vuint16_t FEIE:1;
                vuint16_t BOIE:1;
                vuint16_t LSIE:1;
                vuint16_t WUIE:1;
                vuint16_t DBFIE:1;
                vuint16_t DBEIE:1;
                vuint16_t DRIE:1;
                vuint16_t DTIE:1;
                vuint16_t HRIE:1;
            } B;
        } LINIER;               /* LINFLEX LIN Interrupt Enable Register */

        int16_t LINFLEX_reserved3;

        union {
            vuint16_t R;
            struct {
                vuint16_t LINS:4;
                  vuint16_t:2;
                vuint16_t RMB:1;
                  vuint16_t:1;
                vuint16_t RBSY:1;       // LCH vuint16_t RXBUSY:1;
                vuint16_t RPS:1;        // LCH vuint16_t RDI:1;
                vuint16_t WUF:1;
                vuint16_t DBFF:1;
                vuint16_t DBEF:1;
                vuint16_t DRF:1;
                vuint16_t DTF:1;
                vuint16_t HRF:1;
            } B;
        } LINSR;                /* LINFLEX LIN Status Register */

        int16_t LINFLEX_reserved4;

        union {
            vuint16_t R;
            struct {
                vuint16_t SZF:1;
                vuint16_t OCF:1;
                vuint16_t BEF:1;
                vuint16_t CEF:1;
                vuint16_t SFEF:1;
                vuint16_t BDEF:1;
                vuint16_t IDPEF:1;
                vuint16_t FEF:1;
                vuint16_t BOF:1;
                  vuint16_t:6;
                vuint16_t NF:1;
            } B;
        } LINESR;               /* LINFLEX LIN Error Status Register */

        int16_t LINFLEX_reserved5;

        union {
            vuint16_t R;
            struct {
                vuint16_t:1;
                vuint16_t TDFL:2;
                  vuint16_t:1;
                vuint16_t RDFL:2;
                  vuint16_t:4;
                vuint16_t RXEN:1;
                vuint16_t TXEN:1;
                vuint16_t OP:1; //LCH vuint16_t PARITYODD:1;
                vuint16_t PCE:1;
                vuint16_t WL:1;
                vuint16_t UART:1;
            } B;
        } UARTCR;               /* LINFLEX UART Mode Control Register */

        int16_t LINFLEX_reserved6;

        union {
            vuint16_t R;
            struct {
                vuint16_t SZF:1;
                vuint16_t OCF:1;
                vuint16_t PE:4;
                vuint16_t RMB:1;
                vuint16_t FEF:1;
                vuint16_t BOF:1;
                vuint16_t RPS:1;        // LCH vuint16_t RDI:1;
                vuint16_t WUF:1;
                  vuint16_t:2;
                vuint16_t DRF:1;
                vuint16_t DTF:1;
                vuint16_t NF:1;
            } B;
        } UARTSR;               /* LINFLEX UART Mode Status Register */

        int16_t LINFLEX_reserved7;

        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t LTOM:1;       //LCH vuint16_t MODE:1;
                vuint16_t IOT:1;
                vuint16_t TOCE:1;
                vuint16_t CNT:8;
            } B;
        } LINTCSR;              /* LINFLEX LIN Time-Out Control Status Register */

        int16_t LINFLEX_reserved8;

        union {
            vuint16_t R;
            struct {
                vuint16_t OC2:8;
                vuint16_t OC1:8;
            } B;
        } LINOCR;               /* LINFLEX LIN Output Compare Register */

        int16_t LINFLEX_reserved9;

        union {
            vuint16_t R;
            struct {
                vuint16_t:4;
                vuint16_t RTO:4;        // LCH vuint16_t RTC:4;
                  vuint16_t:1;
                vuint16_t HTO:7;        // LCH vuint16_t HTC:7;
            } B;
        } LINTOCR;              /* LINFLEX LIN Output Compare Register */

        int16_t LINFLEX_reserved10;

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t DIV_F:4;      // LCH vuint16_t FBR:4;
            } B;
        } LINFBRR;              /* LINFLEX LIN Fractional Baud Rate Register */

        int16_t LINFLEX_reserved11;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t DIV_M:13;     // LCH vuint16_t IBR:13;
            } B;
        } LINIBRR;              /* LINFLEX LIN Integer Baud Rate Register */

        int16_t LINFLEX_reserved12;

        union {
            vuint16_t R;
            struct {
                vuint16_t:8;
                vuint16_t CF:8;
            } B;
        } LINCFR;               /* LINFLEX LIN Checksum Field Register */

        int16_t LINFLEX_reserved13;

        union {
            vuint16_t R;
            struct {
                vuint16_t:1;
                vuint16_t IOBE:1;
                vuint16_t IOPE:1;
                vuint16_t WURQ:1;
                vuint16_t DDRQ:1;
                vuint16_t DTRQ:1;
                vuint16_t ABRQ:1;
                vuint16_t HTRQ:1;
                  vuint16_t:8;
            } B;
        } LINCR2;               /* LINFLEX LIN Control Register 2 */

        int16_t LINFLEX_reserved14;

        union {
            vuint16_t R;
            struct {
                vuint16_t DFL:6;
                vuint16_t DIR:1;
                vuint16_t CCS:1;
                  vuint16_t:2;  // LCH vuint16_t:1;
                vuint16_t ID:6;
            } B;
        } BIDR;                 /* LINFLEX Buffer Identifier Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t DATA3:8;
                vuint32_t DATA2:8;
                vuint32_t DATA1:8;
                vuint32_t DATA0:8;
            } B;
        } BDRL;                 /* LINFLEX Buffer Data Register Least Significant */

        union {
            vuint32_t R;
            struct {
                vuint32_t DATA7:8;
                vuint32_t DATA6:8;
                vuint32_t DATA5:8;
                vuint32_t DATA4:8;
            } B;
        } BDRM;                 /* LINFLEX Buffer Data Register Most Significant */

        int16_t LINFLEX_reserved15;

        union {
            vuint16_t R;
            struct {
                vuint16_t:8;
                vuint16_t FACT:8;
            } B;
        } IFER;                 /* LINFLEX Identifier Filter Enable Register */

        int16_t LINFLEX_reserved16;

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t IFMI:4;
            } B;
        } IFMI;                 /* LINFLEX Identifier Filter Match Index Register */

        int16_t LINFLEX_reserved17;

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t IFM:4;
            } B;
        } IFMR;                 /* LINFLEX Identifier Filter Mode Register */

        int16_t LINFLEX_reserved18;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t DFL:3;
                vuint16_t DIR:1;
                vuint16_t CCS:1;
                  vuint16_t:2;
                vuint16_t ID:6;
            } B;
        } IFCR0;                /* LINFLEX Identifier Filter Control Register 0 */

        int16_t LINFLEX_reserved19;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t DFL:3;
                vuint16_t DIR:1;
                vuint16_t CCS:1;
                  vuint16_t:2;
                vuint16_t ID:6;
            } B;
        } IFCR1;                /* LINFLEX Identifier Filter Control Register 1 */

        int16_t LINFLEX_reserved20;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t DFL:3;
                vuint16_t DIR:1;
                vuint16_t CCS:1;
                  vuint16_t:2;
                vuint16_t ID:6;
            } B;
        } IFCR2;                /* LINFLEX Identifier Filter Control Register 2 */

        int16_t LINFLEX_reserved21;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t DFL:3;
                vuint16_t DIR:1;
                vuint16_t CCS:1;
                  vuint16_t:2;
                vuint16_t ID:6;
            } B;
        } IFCR3;                /* LINFLEX Identifier Filter Control Register 3 */

        int16_t LINFLEX_reserved22;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t DFL:3;
                vuint16_t DIR:1;
                vuint16_t CCS:1;
                  vuint16_t:2;
                vuint16_t ID:6;
            } B;
        } IFCR4;                /* LINFLEX Identifier Filter Control Register 4 */

        int16_t LINFLEX_reserved23;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t DFL:3;
                vuint16_t DIR:1;
                vuint16_t CCS:1;
                  vuint16_t:2;
                vuint16_t ID:6;
            } B;
        } IFCR5;                /* LINFLEX Identifier Filter Control Register 5 */

        int16_t LINFLEX_reserved24;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t DFL:3;
                vuint16_t DIR:1;
                vuint16_t CCS:1;
                  vuint16_t:2;
                vuint16_t ID:6;
            } B;
        } IFCR6;                /* LINFLEX Identifier Filter Control Register 6 */

        int16_t LINFLEX_reserved25;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t DFL:3;
                vuint16_t DIR:1;
                vuint16_t CCS:1;
                  vuint16_t:2;
                vuint16_t ID:6;
            } B;
        } IFCR7;                /* LINFLEX Identifier Filter Control Register 7 */

    };                          /* end of LINFLEX_tag */
/****************************************************************************/
/*                          MODULE : ME                                   */
/****************************************************************************/
    struct ME_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t S_CURRENTMODE:4;
                vuint32_t S_MTRANS:1;
                vuint32_t S_DC:1;
                  vuint32_t:2;
                vuint32_t S_PDO:1;
                  vuint32_t:2;
                vuint32_t S_MVR:1;
                vuint32_t S_DFLA:2;
                vuint32_t S_CFLA:2;
                  vuint32_t:8;
                vuint32_t S_PLL1:1;
                vuint32_t S_PLL0:1;
                vuint32_t S_OSC:1;
                vuint32_t S_RC:1;
                vuint32_t S_SYSCLK:4;
            } B;
        } GS;                   /* Global Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t TARGET_MODE:4;
                  vuint32_t:12;
                vuint32_t KEY:16;
            } B;
        } MCTL;                 /* Mode Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:18;
                vuint32_t STANDBY0:1;
                  vuint32_t:2;
                vuint32_t STOP0:1;
                  vuint32_t:1;
                vuint32_t HALT0:1;
                vuint32_t RUN3:1;
                vuint32_t RUN2:1;
                vuint32_t RUN1:1;
                vuint32_t RUN0:1;
                vuint32_t DRUN:1;
                vuint32_t SAFE:1;
                vuint32_t TEST:1;
                vuint32_t RESET:1;
            } B;
        } MER;                  /* Mode Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t I_CONF:1;
                vuint32_t I_MODE:1;
                vuint32_t I_SAFE:1;
                vuint32_t I_MTC:1;
            } B;
        } IS;                   /* Interrupt Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t M_CONF:1;
                vuint32_t M_MODE:1;
                vuint32_t M_SAFE:1;
                vuint32_t M_TC:1;
            } B;
        } IM;                   /* Interrupt Mask Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:27;
                vuint32_t S_MTI:1;
                vuint32_t S_MRI:1;
                vuint32_t S_DMA:1;
                vuint32_t S_NMA:1;
                vuint32_t S_SEA:1;
            } B;
        } IMTS;                 /* Invalid Mode Transition Status Register */

        int32_t ME_reserved0[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                  vuint32_t:8;
                vuint32_t PLL2ON:1;
                vuint32_t PLL1ON:1;
                vuint32_t XOSC0ON:1;
                vuint32_t IRCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } RESET;                /* Reset Mode Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                  vuint32_t:8;
                vuint32_t PLL2ON:1;
                vuint32_t PLL1ON:1;
                vuint32_t XOSC0ON:1;
                vuint32_t IRCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } TEST;                 /* Test Mode Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                  vuint32_t:8;
                vuint32_t PLL2ON:1;
                vuint32_t PLL1ON:1;
                vuint32_t XOSC0ON:1;
                vuint32_t IRCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } SAFE;                 /* Safe Mode Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                  vuint32_t:8;
                vuint32_t PLL2ON:1;
                vuint32_t PLL1ON:1;
                vuint32_t XOSC0ON:1;
                vuint32_t IRCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } DRUN;                 /* DRUN Mode Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                  vuint32_t:8;
                vuint32_t PLL2ON:1;
                vuint32_t PLL1ON:1;
                vuint32_t XOSC0ON:1;
                vuint32_t IRCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } RUN[4];               /* RUN 0->4 Mode Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                  vuint32_t:8;
                vuint32_t PLL2ON:1;
                vuint32_t PLL1ON:1;
                vuint32_t XOSC0ON:1;
                vuint32_t IRCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } HALT0;                /* HALT0 Mode Configuration Register */

        int32_t ME_reserved1;

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                  vuint32_t:8;
                vuint32_t PLL2ON:1;
                vuint32_t PLL1ON:1;
                vuint32_t XOSC0ON:1;
                vuint32_t IRCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } STOP0;                /* STOP0 Mode Configuration Register */

        int32_t ME_reserved2[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                  vuint32_t:8;
                vuint32_t PLL2ON:1;
                vuint32_t PLL1ON:1;
                vuint32_t XOSC0ON:1;
                vuint32_t IRCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } STANDBY0;             /* STANDBY0 Mode Configuration Register */

        int32_t ME_reserved3[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t PERIPH:32;
            } B;
        } PS[4];                /* Peripheral Status 0->4 Register */

        int32_t ME_reserved4[4];

        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t RUN3:1;
                vuint32_t RUN2:1;
                vuint32_t RUN1:1;
                vuint32_t RUN0:1;
                vuint32_t DRUN:1;
                vuint32_t SAFE:1;
                vuint32_t TEST:1;
                vuint32_t RESET:1;
            } B;
        } RUNPC[8];             /* RUN Peripheral Configuration 0->7 Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:18;
                vuint32_t STANDBY0:1;
                  vuint32_t:2;
                vuint32_t STOP0:1;
                  vuint32_t:1;
                vuint32_t HALT0:1;
                  vuint32_t:8;
            } B;
        } LPPC[8];              /* Low Power Peripheral Configuration 0->7 Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t DBG_F:1;
                vuint8_t LP_CFG:3;
                vuint8_t RUN_CFG:3;
            } B;
        } PCTL[144];            /* Peripheral Control 0->143 Register */

    };                          /* end of ME_tag */
/****************************************************************************/
/*                          MODULE : CGM                                   */
/****************************************************************************/
    struct CGM_tag {

        /* The CGM provides a unified register interface, enabling access to 
           all clock sources:
           
           Clock Type | Starting Address Map | Associated Clock Sources
           ------------------------------------------------------------
           System     | C3FE0000             | OSC_CTL
           "        | -                    | Reserved
           "        | C3FE0040             | LPOSC_CTL
           "        | C3FE0060             | RC_CTL
           "        | C3FE0080             | LPRC_CTL
           "        | C3FE00A0             | FMPLL_0
           "        | C3FE00C0             | FMPLL_1
           "        | -                    | Reserved
           MISC       | C3FE0100             | CMU_0 & CMU_1
           
         */

    /************************************/
        /* OSC_CTL @ CGM base address + 0x0000 */
    /************************************/
        union {
            vuint32_t R;
            struct {
                vuint32_t OSCBYP:1;
                  vuint32_t:7;
                vuint32_t EOCV:8;
                vuint32_t M_OSC:1;
                  vuint32_t:2;
                vuint32_t OSCDIV:5;
                vuint32_t I_OSC:1;
                  vuint32_t:5;
                vuint32_t S_OSC:1;
                vuint32_t OSCON:1;
            } B;
        } OSC_CTL;              /* Main OSC Control Register */

    /************************************/
        /* LPOSC_CTL @ CGM base address + 0x0040 */
    /************************************/
        int32_t CGM_reserved0[15];      /* (0x040 - 0x004)/4 = 0x0F */
        /*int32_t $RESERVED[15]; */

        union {
            vuint32_t R;
            struct {
                vuint32_t OSCBYP:1;
                  vuint32_t:7;
                vuint32_t EOCV:8;
                vuint32_t M_OSC:1;
                  vuint32_t:2;
                vuint32_t OSCDIV:5;
                vuint32_t I_OSC:1;
                  vuint32_t:5;
                vuint32_t S_OSC:1;
                vuint32_t OSCON:1;
            } B;
        } LPOSC_CTL;            /* Low Power OSC Control Register */

    /************************************/
        /* RC_CTL @ CGM base address + 0x0060 */
    /************************************/
        int32_t CGM_reserved1[7];       /* (0x060 - 0x044)/4 = 0x07 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:10;
                vuint32_t RCTRIM:6;
                  vuint32_t:3;
                vuint32_t RCDIV:5;
                  vuint32_t:2;
                vuint32_t S_RC_STDBY:1;
                  vuint32_t:5;
            } B;
        } RC_CTL;               /* RC OSC Control Register */

    /*************************************/
        /* LPRC_CTL @ CGM base address + 0x0080 */
    /*************************************/
        int32_t CGM_reserved2[7];       /* (0x080 - 0x064)/4 = 0x07 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:11;
                vuint32_t LRCTRIM:5;
                  vuint32_t:3;
                vuint32_t LPRCDIV:5;
                  vuint32_t:3;
                vuint32_t S_LPRC:1;
                  vuint32_t:3;
                vuint32_t LPRCON_STDBY:1;
            } B;
        } LPRC_CTL;             /* Low Power RC OSC Control Register */

    /************************************/
        /* FMPLL_0 @ CGM base address + 0x00A0 */
        /* FMPLL_1 @ CGM base address + 0x0100 */
    /************************************/
        int32_t CGM_reserved3[7];       /* (0x0A0 - 0x084)/4 = 0x07 */

        struct {
            union {
                vuint32_t R;
                struct {
                    vuint32_t:2;
                    vuint32_t IDF:4;
                    vuint32_t ODF:2;
                      vuint32_t:1;
                    vuint32_t NDIV:7;
                      vuint32_t:7;
                    vuint32_t EN_PLL_SW:1;
                    vuint32_t MODE:1;
                    vuint32_t UNLOCK_ONCE:1;
                      vuint32_t:1;
                    vuint32_t I_LOCK:1;
                    vuint32_t S_LOCK:1;
                    vuint32_t PLL_FAIL_MASK:1;
                    vuint32_t PLL_FAIL_FLAG:1;
                      vuint32_t:1;
                } B;
            } CR;               /* FMPLL Control Register */

            union {
                vuint32_t R;
                struct {
                    vuint32_t STRB_BYPASS:1;
                      vuint32_t:1;
                    vuint32_t SPRD_SEL:1;
                    vuint32_t MOD_PERIOD:13;
                    vuint32_t FM_EN:1;
                    vuint32_t INC_STEP:15;
                } B;
            } MR;               /* FMPLL Modulation Register */

            int32_t CGM_reserved4[6];   /* (0x0C0 - 0x0A8)/4 = 0x06 */
            /* (0x0E0 - 0x0C8)/4 = 0x06 */

        } FMPLL[2];

    /************************************/
        /* CMU @ CGM base address + 0x0100  */
    /************************************/
        int32_t CGM_reserved5[8];       /* (0x100 - 0x0E0)/4 = 0x08 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t SFM:1;
                  vuint32_t:13;
                vuint32_t CLKSEL1:2;
                  vuint32_t:5;
                vuint32_t RCDIV:2;
                vuint32_t CME_A:1;
            } B;
        } CMU_0_CSR;            /* Control Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t FD:20;
            } B;
        } CMU_0_FDR;            /* Frequency Display Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:20;
                vuint32_t HFREF_A:12;
            } B;
        } CMU_0_HFREFR_A;       /* High Frequency Reference Register PLL_A Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:20;
                vuint32_t LFREF_A:12;
            } B;
        } CMU_0_LFREFR_A;       /* Low Frequency Reference Register PLL_A Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t FLCI_0:1;
                vuint32_t FHHI_0:1;
                vuint32_t FLLI_0:1;
                vuint32_t OLRI:1;
            } B;
        } CMU_0_ISR;            /* Interrupt Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } CMU_0_IMR;            /* Interrupt Mask Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t MD:20;
            } B;
        } CMU_0_MDR;            /* Measurement Duration Register */

        int32_t CGM_reserved5A; /* (0x020 - 0x01C)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t SFM:1;
                  vuint32_t:13;
                vuint32_t CLKSEL1:2;
                  vuint32_t:5;
                vuint32_t RCDIV:2;
                vuint32_t CME_A:1;
            } B;
        } CMU_1_CSR;            /* Control Status Register */

        int32_t CGM_reserved6;  /* (0x028 - 0x024)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:20;
                vuint32_t HFREF_A:12;
            } B;
        } CMU_1_HFREFR_A;       /* High Frequency Reference Register PLL_A Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:20;
                vuint32_t LFREF_A:12;
            } B;
        } CMU_1_LFREFR_A;       /* Low Frequency Reference Register PLL_A Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t FLCI_1:1;
                vuint32_t FHHI_1:1;
                vuint32_t FLLI_1:1;
                  vuint32_t:1;
            } B;
        } CMU_1_ISR;            /* Interrupt Status Register */

    /************************************/
        /* CGM General Registers @ CGM base address + 0x0370 */
    /************************************/
        int32_t CGM_reserved7[143];     /* (0x370 - 0x134)/4 = 0x8F */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t EN:1;
            } B;
        } OCEN;                 /* Output Clock Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:2;
                vuint32_t SELDIV:2;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } OCDSSC;               /* Output Clock Division Select Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELSTAT:4;
                  vuint32_t:24;
            } B;
        } SCSS;                 /* System Clock Select Status */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } SCDC;                 /* GSystem Clock Divider Configuration 0->4 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC0SC;                /* Aux Clock 0 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC0DC;                /* Aux Clock 0 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC1SC;                /* Aux Clock 1 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC1DC;                /* Aux Clock 1 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC2SC;                /* Aux Clock 2 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC2DC;                /* Aux Clock 2 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC3SC;                /* Aux Clock 3 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC3DC;                /* Aux Clock 3 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC4SC;                /* Aux Clock 4 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC4DC;                /* Aux Clock 4 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC5SC;                /* Aux Clock 5 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC5DC;                /* Aux Clock 5 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC6SC;                /* Aux Clock 6 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC6DC;                /* Aux Clock 6 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC7SC;                /* Aux Clock 7 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC7DC;                /* Aux Clock 7 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC8SC;                /* Aux Clock 8 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC8DC;                /* Aux Clock 8 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC9SC;                /* Aux Clock 9 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC9DC;                /* Aux Clock 9 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC10SC;               /* Aux Clock 10 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC10DC;               /* Aux Clock 10 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC11SC;               /* Aux Clock 11 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC11DC;               /* Aux Clock 11 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC12SC;               /* Aux Clock 12 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC12DC;               /* Aux Clock 12 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC13SC;               /* Aux Clock 13 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC13DC;               /* Aux Clock 13 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC14SC;               /* Aux Clock 14 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC14DC;               /* Aux Clock 14 Divider Configuration 0->3 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } AC15SC;               /* Aux Clock 15 Select Control */

        union {
            vuint32_t R;
            struct {
                vuint32_t DE0:1;
                  vuint32_t:3;
                vuint32_t DIV0:4;
                vuint32_t DE1:1;
                  vuint32_t:3;
                vuint32_t DIV1:4;
                vuint32_t DE2:1;
                  vuint32_t:3;
                vuint32_t DIV2:4;
                vuint32_t DE3:1;
                  vuint32_t:3;
                vuint32_t DIV3:4;
            } B;
        } AC15DC;               /* Aux Clock 15 Divider Configuration 0->3 */

    };                          /* end of CGM_tag */
/****************************************************************************/
/*                          MODULE : RGM                                   */
/****************************************************************************/
    struct RGM_tag {

        union {
            vuint16_t R;
            struct {
                vuint16_t F_EXR:1;
                  vuint16_t:3;
                vuint16_t F_CMU1_FHL:1;
                  vuint16_t:1;
                vuint16_t F_PLL1:1;
                vuint16_t F_FLASH:1;
                vuint16_t F_LVD45:1;
                vuint16_t F_CMU0_FHL:1;
                vuint16_t F_CMU0_OLR:1;
                vuint16_t F_PLL0:1;
                vuint16_t F_CHKSTOP:1;
                vuint16_t F_SOFT:1;
                vuint16_t F_CORE:1;
                vuint16_t F_JTAG:1;
            } B;
        } FES;                  /* Functional Event Status */

        union {
            vuint16_t R;
            struct {
                vuint16_t POR:1;
                  vuint16_t:7;
                vuint16_t F_COMP:1;
                vuint16_t F_LVD27_IO:1;
                vuint16_t F_LVD27_FLASH:1;
                vuint16_t F_LVD27_VREG:1;
                vuint16_t F_LVD27:1;
                vuint16_t F_SWT:1;
                vuint16_t F_LVD12_PD1:1;
                vuint16_t F_LVD12_PD0:1;
            } B;
        } DES;                  /* Destructive Event Status */

        union {
            vuint16_t R;
            struct {
                vuint16_t D_EXR:1;
                  vuint16_t:3;
                vuint16_t D_CMU1_FHL:1;
                  vuint16_t:1;
                vuint16_t D_PLL1:1;
                vuint16_t D_FLASH:1;
                vuint16_t D_LVD45:1;
                vuint16_t D_CMU0_FHL:1;
                vuint16_t D_CMU0_OLR:1;
                vuint16_t D_PLL0:1;
                vuint16_t D_CHKSTOP:1;
                vuint16_t D_SOFT:1;
                vuint16_t D_CORE:1;
                vuint16_t D_JTAG:1;
            } B;
        } FERD;                 /* Functional Event Reset Disable */

        union {
            vuint16_t R;
            struct {
                vuint16_t:8;
                vuint16_t D_COMP:1;
                vuint16_t D_LVD27_IO:1;
                vuint16_t D_LVD27_FLASH:1;
                vuint16_t D_LVD27_VREG:1;
                vuint16_t D_LVD27:1;
                vuint16_t D_SWT:1;
                vuint16_t D_LVD12_PD1:1;
                vuint16_t D_LVD12_PD0:1;
            } B;
        } DERD;                 /* Destructive Event Reset Disable */

        int16_t RGM_reserved0[4];

        union {
            vuint16_t R;
            struct {
                vuint16_t:4;
                vuint16_t AR_CMU1_FHL:1;
                  vuint16_t:1;
                vuint16_t AR_PLL1:1;
                vuint16_t AR_FLASH:1;
                vuint16_t AR_LVD45:1;
                vuint16_t AR_CMU0_FHL:1;
                vuint16_t AR_CMU0_OLR:1;
                vuint16_t AR_PLL0:1;
                vuint16_t AR_CHKSTOP:1;
                vuint16_t AR_SOFT:1;
                vuint16_t AR_CORE:1;
                vuint16_t AR_JTAG:1;
            } B;
        } FEAR;                 /* Functional Event Alternate Request */

        union {
            vuint16_t R;
            struct {
                vuint16_t:8;
                vuint16_t AR_COMP:1;
                vuint16_t AR_LVD27_IO:1;
                vuint16_t AR_LVD27_FLASH:1;
                vuint16_t AR_LVD27_VREG:1;
                vuint16_t AR_LVD27:1;
                vuint16_t AR_SWT:1;
                vuint16_t AR_LVD12_PD1:1;
                vuint16_t AR_LVD12_PD0:1;
            } B;
        } DEAR;                 /* Destructive Event Alternate Request */

        int16_t RGM_reserved1[2];

        union {
            vuint16_t R;
            struct {
                vuint16_t:4;
		vuint16_t SS_CMU1_FHL:1;
		vuint16_t:1;
		vuint16_t SS_PLL1:1;
                vuint16_t SS_FLASH:1;
                vuint16_t SS_LVD45:1;
                vuint16_t SS_CMU0_FHL:1;
                vuint16_t SS_CMU0_OLR:1;
                vuint16_t SS_PLL0:1;
                vuint16_t SS_CHKSTOP:1;
                vuint16_t SS_SOFT:1;
                vuint16_t SS_CORE:1;
                vuint16_t SS_JTAG:1;
            } B;
        } FESS;                 /* Functional Event Short Sequence */

        union {
            vuint16_t R;
            struct {
                vuint16_t:8;
                vuint16_t BOOT:1;
                  vuint16_t:4;
                vuint16_t DRUND_FLA:1;
                  vuint16_t:1;
                vuint16_t DRUNC_FLA:1;
            } B;
        } STDBY;                /* STANDBY reset sequence */

        union {
            vuint16_t R;
            struct {
                vuint16_t:4;
		vuint16_t BE_CMU1_FHL:1;
		vuint16_t:1;
		vuint16_t BE_PLL1:1;
                vuint16_t BE_FLASH:1;
                vuint16_t BE_LVD45:1;
                vuint16_t BE_CMU0_FHL:1;
                vuint16_t BE_CMU0_OLR:1;
                vuint16_t BE_PLL0:1;
                vuint16_t BE_CHKSTOP:1;
                vuint16_t BE_SOFT:1;
                vuint16_t BE_CORE:1;
                vuint16_t BE_JTAG:1;
            } B;
        } FBRE;                 /* Functional Bidirectional Reset Enable */

    };                          /* end of RGM_tag */
/****************************************************************************/
/*                          MODULE : PCU                                   */
/****************************************************************************/
    struct PCU_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t:18;
                vuint32_t STBY0:1;
                  vuint32_t:2;
                vuint32_t STOP0:1;
                  vuint32_t:1;
                vuint32_t HALT0:1;
                vuint32_t RUN3:1;
                vuint32_t RUN2:1;
                vuint32_t RUN1:1;
                vuint32_t RUN0:1;
                vuint32_t DRUN:1;
                vuint32_t SAFE:1;
                vuint32_t TEST:1;
                vuint32_t RST:1;
            } B;
        } PCONF[16];            /* Power domain 0-15 configuration register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t PD15:1;
                vuint32_t PD14:1;
                vuint32_t PD13:1;
                vuint32_t PD12:1;
                vuint32_t PD11:1;
                vuint32_t PD10:1;
                vuint32_t PD9:1;
                vuint32_t PD8:1;
                vuint32_t PD7:1;
                vuint32_t PD6:1;
                vuint32_t PD5:1;
                vuint32_t PD4:1;
                vuint32_t PD3:1;
                vuint32_t PD2:1;
                vuint32_t PD1:1;
                vuint32_t PD0:1;
            } B;
        } PSTAT;                /* Power Domain Status Register */

        int32_t PCU_reserved0[15];      /* {0x0080-0x0044}/0x4 = 0xF */

        union {
            vuint32_t R;
            struct {
                vuint32_t:15;
                vuint32_t MASK_LVDHV5:1;
            } B;
        } VCTL;                 /* Voltage Regulator Control Register */

    };                          /* end of PCU_tag */
/****************************************************************************/
/*                          MODULE : FLEXPWM                                   */
/****************************************************************************/
    struct FLEXPWM_SUB_tag {

        union {
            vuint16_t R;
        } CNT;                  /* Counter Register */

        union {
            vuint16_t R;
        } INIT;                 /* Initial Count Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t DBGEN:1;
                vuint16_t WAITEN:1;
                vuint16_t INDEP:1;
                vuint16_t PWMA_INIT:1;
                vuint16_t PWMB_INIT:1;
                vuint16_t PWMX_INIT:1;
                vuint16_t INIT_SEL:2;
                vuint16_t FRCEN:1;
                vuint16_t FORCE:1;
                vuint16_t FORCE_SEL:3;
                vuint16_t RELOAD_SEL:1;
                vuint16_t CLK_SEL:2;
            } B;
        } CTRL2;                /* Control 2 Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t LDFQ:4;
                vuint16_t HALF:1;
                vuint16_t FULL:1;
                vuint16_t DT:2;
                  vuint16_t:1;
                vuint16_t PRSC:3;
                  vuint16_t:3;
                vuint16_t DBLEN:1;
            } B;
        } CTRL;                 /* Control Register */

        union {
            vuint16_t R;
        } VAL[6];               /* Value Register 0->5 */

        union {
            vuint16_t R;
            struct {
                vuint16_t FRACAEN:1;
                  vuint16_t:10;
                vuint16_t FRACADLY:5;
            } B;
        } FRACA;                /* Fractional Delay Register A */

        union {
            vuint16_t R;
            struct {
                vuint16_t FRACBEN:1;
                  vuint16_t:10;
                vuint16_t FRACBDLY:5;
            } B;
        } FRACB;                /* Fractional Delay Register B */

        union {
            vuint16_t R;
            struct {
                vuint16_t PWMA_IN:1;
                vuint16_t PWMB_IN:1;
                vuint16_t PWMX_IN:1;
                  vuint16_t:2;
                vuint16_t POLA:1;
                vuint16_t POLB:1;
                vuint16_t POLX:1;
                  vuint16_t:2;
                vuint16_t PWMAFS:2;
                vuint16_t PWMBFS:2;
                vuint16_t PWMXFS:2;
            } B;
        } OCTRL;                /* Output Control Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:1;
                vuint16_t RUF:1;
                vuint16_t REF:1;
                vuint16_t RF:1;
                vuint16_t CFA1:1;
                vuint16_t CFA0:1;
                vuint16_t CFB1:1;
                vuint16_t CFB0:1;
                vuint16_t CFX1:1;
                vuint16_t CFX0:1;
                vuint16_t CMPF:6;
            } B;
        } STS;                  /* Status Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:2;
                vuint16_t REIE:1;
                vuint16_t RIE:1;
                  vuint16_t:4;
                vuint16_t CX1IE:1;
                vuint16_t CX0IE:1;
                vuint16_t CMPIE:6;
            } B;
        } INTEN;                /* Interrupt Enable Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:6;
                vuint16_t VALDE:1;
                vuint16_t FAND:1;
                vuint16_t CAPTDE:2;
                vuint16_t CA1DE:1;
                vuint16_t CA0DE:1;
                vuint16_t CB1DE:1;
                vuint16_t CB0DE:1;
                vuint16_t CX1DE:1;
                vuint16_t CX0DE:1;
            } B;
        } DMAEN;                /* DMA Enable Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:10;
                vuint16_t OUT_TRIG_EN:6;
            } B;
        } TCTRL;                /* Output Trigger Control Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:4;
                vuint16_t DISX:4;
                vuint16_t DISB:4;
                vuint16_t DISA:4;
            } B;
        } DISMAP;               /* Fault Disable Mapping Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t DTCNT0:11;
            } B;
        } DTCNT0;               /* Deadtime Count Register 0 */

        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t DTCNT1:11;
            } B;
        } DTCNT1;               /* Deadtime Count Register 1 */

        union {
            vuint16_t R;
            struct {
                vuint16_t CA1CNT:3;
                vuint16_t CA0CNT:3;
                vuint16_t CFAWM:2;
                vuint16_t EDGCNTAEN:1;
                vuint16_t INPSELA:1;
                vuint16_t EDGA1:2;
                vuint16_t EDGA0:2;
                vuint16_t ONESHOTA:1;
                vuint16_t ARMA:1;
            } B;
        } CAPTCTRLA;            /* Capture Control Register A */

        union {
            vuint16_t R;
            struct {
                vuint16_t EDGCNTA:8;
                vuint16_t EDGCMPA:8;
            } B;
        } CAPTCOMPA;            /* Capture Compare Register A */

        union {
            vuint16_t R;
            struct {
                vuint16_t CB1CNT:3;
                vuint16_t CB0CNT:3;
                vuint16_t CFBWM:2;
                vuint16_t EDGCNTBEN:1;
                vuint16_t INPSELB:1;
                vuint16_t EDGB1:2;
                vuint16_t EDGB0:2;
                vuint16_t ONESHOTB:1;
                vuint16_t ARMB:1;
            } B;
        } CAPTCTRLB;            /* Capture Control Register B */

        union {
            vuint16_t R;
            struct {
                vuint16_t EDGCNTB:8;
                vuint16_t EDGCMPB:8;
            } B;
        } CAPTCOMPB;            /* Capture Compare Register B */

        union {
            vuint16_t R;
            struct {
                vuint16_t CX1CNT:3;
                vuint16_t CX0CNT:3;
                vuint16_t CFXWM:2;
                vuint16_t EDGCNTX_EN:1;
                vuint16_t INP_SELX:1;
                vuint16_t EDGX1:2;
                vuint16_t EDGX0:2;
                vuint16_t ONESHOTX:1;
                vuint16_t ARMX:1;
            } B;
        } CAPTCTRLX;            /* Capture Control Register B */

        union {
            vuint16_t R;
            struct {
                vuint16_t EDGCNTX:8;
                vuint16_t EDGCMPX:8;
            } B;
        } CAPTCOMPX;            /* Capture Compare Register X */

        union {
            vuint16_t R;
            struct {
                vuint16_t CAPTVAL0:16;
            } B;
        } CVAL0;                /* Capture Value 0 Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t CVAL0CYC:4;
            } B;
        } CVAL0C;               /* Capture Value 0 Cycle Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t CAPTVAL1:16;
            } B;
        } CVAL1;                /* Capture Value 1 Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t CVAL1CYC:4;
            } B;
        } CVAL1C;               /* Capture Value 1 Cycle Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t CAPTVAL2:16;
            } B;
        } CVAL2;                /* Capture Value 2 Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t CVAL2CYC:4;
            } B;
        } CVAL2C;               /* Capture Value 2 Cycle Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t CAPTVAL3:16;
            } B;
        } CVAL3;                /* Capture Value 3 Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t CVAL3CYC:4;
            } B;
        } CVAL3C;               /* Capture Value 3 Cycle Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t CAPTVAL4:16;
            } B;
        } CVAL4;                /* Capture Value 4 Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t CVAL4CYC:4;
            } B;
        } CVAL4C;               /* Capture Value 4 Cycle Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t CAPTVAL5:16;
            } B;
        } CVAL5;                /* Capture Value 5 Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t CVAL5CYC:4;
            } B;
        } CVAL5C;               /* Capture Value 5 Cycle Register */

        uint32_t FLEXPWM_SUB_reserved0; /* (0x04A - 0x050)/4 = 0x01 */

    };                          /* end of FLEXPWM_SUB_tag */

    struct FLEXPWM_tag {

        /* eg. FLEXPWM.SUB<[x]>.CNT.R  {x = 0->3} */
        struct FLEXPWM_SUB_tag SUB[4];

        union {
            vuint16_t R;
            struct {
                vuint16_t:4;
                vuint16_t PWMA_EN:4;
                vuint16_t PWMB_EN:4;
                vuint16_t PWMX_EN:4;
            } B;
        } OUTEN;                /* Output Enable Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:4;
                vuint16_t MASKA:4;
                vuint16_t MASKB:4;
                vuint16_t MASKX:4;
            } B;
        } MASK;                 /* Output Mask Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:8;
                vuint16_t OUTA_3:1;
                vuint16_t OUTB_3:1;
                vuint16_t OUTA_2:1;
                vuint16_t OUTB_2:1;
                vuint16_t OUTA_1:1;
                vuint16_t OUTB_1:1;
                vuint16_t OUTA_0:1;
                vuint16_t OUTB_0:1;
            } B;
        } SWCOUT;               /* Software Controlled Output Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t SELA_3:2;
                vuint16_t SELB_3:2;
                vuint16_t SELA_2:2;
                vuint16_t SELB_2:2;
                vuint16_t SELA_1:2;
                vuint16_t SELB_1:2;
                vuint16_t SELA_0:2;
                vuint16_t SELB_0:2;
            } B;
        } DTSRCSEL;             /* Deadtime Source Select Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t IPOL:4;
                vuint16_t RUN:4;
                vuint16_t CLDOK:4;
                vuint16_t LDOK:4;
            } B;
        } MCTRL;                /* Master Control Register */

        int16_t FLEXPWM_reserved1;

        union {
            vuint16_t R;
            struct {
                vuint16_t FLVL:4;
                vuint16_t FAUTO:4;
                vuint16_t FSAFE:4;
                vuint16_t FIE:4;
            } B;
        } FCTRL;                /* Fault Control Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t FTEST:1;
                vuint16_t FFPIN:4;
                  vuint16_t:4;
                vuint16_t FFLAG:4;
            } B;
        } FSTS;                 /* Fault Status Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t FILT_CNT:3;
                vuint16_t FILT_PER:8;
            } B;
        } FFILT;                /* Fault FilterRegister */

    };                          /* end of FLEXPWM_tag */
/****************************************************************************/
/*                          MODULE : ETIMER                                   */
/****************************************************************************/
    struct ETIMER_CHANNEL_tag {

        union {
            vuint16_t R;
            struct {
                vuint16_t COMP1:16;
            } B;
        } COMP1;                /* Compare Register 1 */

        union {
            vuint16_t R;
            struct {
                vuint16_t COMP2:16;
            } B;
        } COMP2;                /* Compare Register 2 */

        union {
            vuint16_t R;
            struct {
                vuint16_t CAPT1:16;
            } B;
        } CAPT1;                /* Capture Register 1 */

        union {
            vuint16_t R;
            struct {
                vuint16_t CAPT2:16;
            } B;
        } CAPT2;                /* Capture Register 2 */

        union {
            vuint16_t R;
            struct {
                vuint16_t LOAD:16;
            } B;
        } LOAD;                 /* Load Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t HOLD:16;
            } B;
        } HOLD;                 /* Hold Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t CNTR:16;
            } B;
        } CNTR;                 /* Counter Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t CNTMODE:3;
                vuint16_t PRISRC:5;
                vuint16_t ONCE:1;
                vuint16_t LENGTH:1;
                vuint16_t DIR:1;
                vuint16_t SECSRC:5;
            } B;
        } CTRL;                 /* Control Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t OEN:1;
                vuint16_t RDNT:1;
                vuint16_t INPUT:1;
                vuint16_t VAL:1;
                vuint16_t FORCE:1;
                vuint16_t COFRC:1;
                vuint16_t COINIT:2;
                vuint16_t SIPS:1;
                vuint16_t PIPS:1;
                vuint16_t OPS:1;
                vuint16_t MSTR:1;
                vuint16_t OUTMODE:4;
            } B;
        } CTRL2;                /* Control Register 2 */

        union {
            vuint16_t R;
            struct {
                vuint16_t STPEN:1;
                vuint16_t ROC:2;
                vuint16_t FMODE:1;
                vuint16_t FDIS:4;
                vuint16_t C2FCNT:3;
                vuint16_t C1FCNT:3;
                vuint16_t DBGEN:2;
            } B;
        } CTRL3;                /* Control Register 3 */

        union {
            vuint16_t R;
            struct {
                vuint16_t:6;
                vuint16_t WDF:1;
                vuint16_t RCF:1;
                vuint16_t ICF2:1;
                vuint16_t ICF1:1;
                vuint16_t IEHF:1;
                vuint16_t IELF:1;
                vuint16_t TOF:1;
                vuint16_t TCF2:1;
                vuint16_t TCF1:1;
                vuint16_t TCF:1;
            } B;
        } STS;                  /* Status Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t ICF2DE:1;
                vuint16_t ICF1DE:1;
                vuint16_t CMPLD2DE:1;
                vuint16_t CMPLD1DE:1;
                  vuint16_t:2;
                vuint16_t WDFIE:1;
                vuint16_t RCFIE:1;
                vuint16_t ICF2IE:1;
                vuint16_t ICF1IE:1;
                vuint16_t IEHFIE:1;
                vuint16_t IELFIE:1;
                vuint16_t TOFIE:1;
                vuint16_t TCF2IE:1;
                vuint16_t TCF1IE:1;
                vuint16_t TCFIE:1;
            } B;
        } INTDMA;               /* Interrupt and DMA Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t CMPLD1:16;
            } B;
        } CMPLD1;               /* Compare Load Register 1 */

        union {
            vuint16_t R;
            struct {
                vuint16_t CMPLD2:16;
            } B;
        } CMPLD2;               /* Compare Load Register 2 */

        union {
            vuint16_t R;
            struct {
                vuint16_t CLC2:3;
                vuint16_t CLC1:3;
                vuint16_t CMPMODE:2;
                vuint16_t CPT2MODE:2;
                vuint16_t CPT1MODE:2;
                vuint16_t CFWM:2;
                vuint16_t ONESHOT:1;
                vuint16_t ARM:1;
            } B;
        } CCCTRL;               /* Compare and Capture Control Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t FILTCNT:3;
                vuint16_t FILTPER:8;
            } B;
        } FILT;                 /* Input Filter Register */

    };                          /* end of ETIMER_CHANNEL_tag */

    struct ETIMER_tag {

        struct ETIMER_CHANNEL_tag CHANNEL[8];

        union {
            vuint16_t R;
            struct {
                vuint16_t WDTOL:16;
            } B;
        } WDTOL;                /* Watchdog Time-out Low Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t WDTOH:16;
            } B;
        } WDTOH;                /* Watchdog Time-out High Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t FTEST:1;
                vuint16_t FIE:4;
                  vuint16_t:4;
                vuint16_t FLVL:4;
            } B;
        } FCTRL;                /* Fault Control Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:4;
                vuint16_t FFPIN:4;
                  vuint16_t:4;
                vuint16_t FFLAG:4;
            } B;
        } FSTS;                 /* Fault Status Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t FFILTCNT:3;
                vuint16_t FFILTPER:8;
            } B;
        } FFILT;                /* Fault Filter Register */

        int16_t ETIMER_reserved1;

        union {
            vuint16_t R;
            struct {
                vuint16_t:8;
                vuint16_t ENBL:8;
            } B;
        } ENBL;                 /* Channel Enable Register */

        int16_t ETIMER_reserved2;

        union {
            vuint16_t R;
            struct {
                vuint16_t:11;
                vuint16_t DREQ:5;
            } B;
        } DREQ[4];              /* DMA Request 0->3 Select Register */

    };                          /* end of ETIMER_tag */

/****************************************************************************/
/*                          MODULE : CTUL                                   */
/****************************************************************************/
    struct CTUL_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t PRESC_CONF:4;
                  vuint32_t:4;
                vuint32_t TRGIEN:1;
                vuint32_t TRGI:1;
                  vuint32_t:2;
                vuint32_t CNT3_EN:1;
                vuint32_t CNT2_EN:1;
                vuint32_t CNT1_EN:1;
                vuint32_t CNT0_EN:1;
            } B;
        } CSR;                  /* Control Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:23;
                vuint32_t SV:9;
            } B;
        } SVR[7];               /* Start Value Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:23;
                vuint32_t CV:9;
            } B;
        } CVR[4];               /* Current Value Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t TM:1;
                vuint32_t CNT:2;
                vuint32_t DELAY:3;
                  vuint32_t:4;
                vuint32_t CHANNELVALUE:6;
            } B;
        } EVTCFGR[64];          /* Event Configuration Register */

    };                          /* end of CTUL_tag */
/****************************************************************************/
/*                          MODULE : CTU                                   */
/****************************************************************************/
    struct CTU_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t I15_FE:1;
                vuint32_t I15_RE:1;
                vuint32_t I14_FE:1;
                vuint32_t I14_RE:1;
                vuint32_t I13_FE:1;
                vuint32_t I13_RE:1;
                vuint32_t I12_FE:1;
                vuint32_t I12_RE:1;
                vuint32_t I11_FE:1;
                vuint32_t I11_RE:1;
                vuint32_t I10_FE:1;
                vuint32_t I10_RE:1;
                vuint32_t I9_FE:1;
                vuint32_t I9_RE:1;
                vuint32_t I8_FE:1;
                vuint32_t I8_RE:1;
                vuint32_t I7_FE:1;
                vuint32_t I7_RE:1;
                vuint32_t I6_FE:1;
                vuint32_t I6_RE:1;
                vuint32_t I5_FE:1;
                vuint32_t I5_RE:1;
                vuint32_t I4_FE:1;
                vuint32_t I4_RE:1;
                vuint32_t I3_FE:1;
                vuint32_t I3_RE:1;
                vuint32_t I2_FE:1;
                vuint32_t I2_RE:1;
                vuint32_t I1_FE:1;
                vuint32_t I1_RE:1;
                vuint32_t I0_FE:1;
                vuint32_t I0_RE:1;
            } B;
        } TGSISR;               /* -Trigger Generator Subunit Input Selection Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:7;
                vuint16_t ETTM:1;
                vuint16_t PRES:2;
                vuint16_t MRSSM:5;
                vuint16_t TGSM:1;
            } B;
        } TGSCR;                /* Trigger Generator Subunit Control Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t TCRV:16;
            } B;
        } TCR[8];               /* Trigger 0->7 Compare Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t TGSCCV:16;
            } B;
        } TGSCCR;               /* TGS Counter Compare Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t TGSCRV:16;
            } B;
        } TGSCRR;               /* TGS Counter Reload Register */

        uint16_t CTU_reserved0;

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t T3INDEX:5;
                  vuint32_t:3;
                vuint32_t T2INDEX:5;
                  vuint32_t:3;
                vuint32_t T1INDEX:5;
                  vuint32_t:3;
                vuint32_t T0INDEX:5;
            } B;
        } CLCR1;                /* Command List Control Register 1 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t T7INDEX:5;
                  vuint32_t:3;
                vuint32_t T6INDEX:5;
                  vuint32_t:3;
                vuint32_t T5INDEX:5;
                  vuint32_t:3;
                vuint32_t T4INDEX:5;
            } B;
        } CLCR2;                /* Command List Control Register 2 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t T3E:1;
                vuint32_t T3ETE:1;
                vuint32_t T3T1E:1;
                vuint32_t T3T0E:1;
                vuint32_t T3ADCE:1;
                  vuint32_t:3;
                vuint32_t T2E:1;
                vuint32_t T2ETE:1;
                vuint32_t T2T1E:1;
                vuint32_t T2T0E:1;
                vuint32_t T2ADCE:1;
                  vuint32_t:3;
                vuint32_t T1E:1;
                vuint32_t T1ETE:1;
                vuint32_t T1T1E:1;
                vuint32_t T1T0E:1;
                vuint32_t T1ADCE:1;
                  vuint32_t:3;
                vuint32_t T0E:1;
                vuint32_t T0ETE:1;
                vuint32_t T0T1E:1;
                vuint32_t T0T0E:1;
                vuint32_t T0ADCE:1;
            } B;
        } THCR1;                /* Trigger Handler Control Register 1 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t T7E:1;
                vuint32_t T7ETE:1;
                vuint32_t T7T1E:1;
                vuint32_t T7T0E:1;
                vuint32_t T7ADCE:1;
                  vuint32_t:3;
                vuint32_t T6E:1;
                vuint32_t T6ETE:1;
                vuint32_t T6T1E:1;
                vuint32_t T6T0E:1;
                vuint32_t T6ADCE:1;
                  vuint32_t:3;
                vuint32_t T5E:1;
                vuint32_t T5ETE:1;
                vuint32_t T5T1E:1;
                vuint32_t T5T0E:1;
                vuint32_t T5ADCE:1;
                  vuint32_t:3;
                vuint32_t T4E:1;
                vuint32_t T4ETE:1;
                vuint32_t T4T1E:1;
                vuint32_t T4T0E:1;
                vuint32_t T4ADCE:1;
            } B;
        } THCR2;                /* Trigger Handler Control Register 2 */

        /* Single Conversion Mode - Comment for Dual Conversion Mode */
        union {
            vuint16_t R;
            struct {
                vuint16_t CIR:1;
                vuint16_t FC:1;
                vuint16_t CMS:1;
				vuint16_t:1;
                vuint16_t FIFO:2;
                  vuint16_t:4;
                vuint16_t SU:1;
                  vuint16_t:1;
                vuint16_t CH:4;
            } B;
        } CLR[24];              /* Commands List Register x (double-buffered) (x = 1,...,24) */

        /* Uncomment for Dual Conversion Mode */
        /*union {                             
           vuint16_t R;                      
           struct {                          
           vuint16_t CIR:1;                
           vuint16_t FC:1;                 
           vuint16_t CMS:1;        
	vuint16_t:1;
           vuint16_t FIFO:2;      
	vuint16_t:1;
           vuint16_t CHB:4;                
           vuint16_t :1;                   
           vuint16_t CHA:4;                
           } B;                              
           } CLR[24];                                     */
        /* Commands List Register x (double-buffered) (x = 1,...,24) */

		uint16_t CTU_reserved1[8];
		

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
                vuint16_t DMAEN3:1;
                vuint16_t DMAEN2:1;
                vuint16_t DMAEN1:1;
                vuint16_t DMAEN0:1;
            } B;
        } CR;                   /* Control Register */

		   uint16_t CTU_reserved2;
		   
        union {
            vuint32_t R;
            struct {
                vuint32_t FIFO_OVERRUN_EN7:1;
                vuint32_t FIFO_OVERFLOW_EN7:1;
                vuint32_t FIFO_EMPTY_EN7:1;
                vuint32_t FIFO_FULL_EN7:1;
                vuint32_t FIFO_OVERRUN_EN6:1;
                vuint32_t FIFO_OVERFLOW_EN6:1;
                vuint32_t FIFO_EMPTY_EN6:1;
                vuint32_t FIFO_FULL_EN6:1;
                vuint32_t FIFO_OVERRUN_EN5:1;
                vuint32_t FIFO_OVERFLOW_EN5:1;
                vuint32_t FIFO_EMPTY_EN5:1;
                vuint32_t FIFO_FULL_EN5:1;
                vuint32_t FIFO_OVERRUN_EN4:1;
                vuint32_t FIFO_OVERFLOW_EN4:1;
                vuint32_t FIFO_EMPTY_EN4:1;
                vuint32_t FIFO_FULL_EN4:1;
                vuint32_t FIFO_OVERRUN_EN3:1;
                vuint32_t FIFO_OVERFLOW_EN3:1;
                vuint32_t FIFO_EMPTY_EN3:1;
                vuint32_t FIFO_FULL_EN3:1;
                vuint32_t FIFO_OVERRUN_EN2:1;
                vuint32_t FIFO_OVERFLOW_EN2:1;
                vuint32_t FIFO_EMPTY_EN2:1;
                vuint32_t FIFO_FULL_EN2:1;
                vuint32_t FIFO_OVERRUN_EN1:1;
                vuint32_t FIFO_OVERFLOW_EN1:1;
                vuint32_t FIFO_EMPTY_EN1:1;
                vuint32_t FIFO_FULL_EN1:1;
                vuint32_t FIFO_OVERRUN_EN0:1;
                vuint32_t FIFO_OVERFLOW_EN0:1;
                vuint32_t FIFO_EMPTY_EN0:1;
                vuint32_t FIFO_FULL_EN0:1;
            } B;
        } FCR;                  /* CONTROL REGISTER FIFO */

        union {
            vuint32_t R;
            struct {
                vuint32_t THRESHOLD3:8;
                vuint32_t THRESHOLD2:8;
                vuint32_t THRESHOLD1:8;
                vuint32_t THRESHOLD0:8;
            } B;
        } TH1;                  /* Threshold Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t THRESHOLD7:8;
                vuint32_t THRESHOLD6:8;
                vuint32_t THRESHOLD5:8;
                vuint32_t THRESHOLD4:8;
            } B;
        } TH2;                  /* Threshold Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t FIFO_OVERRUN7:1;
                vuint32_t FIFO_OVERFLOW7:1;
                vuint32_t FIFO_EMPTY7:1;
                vuint32_t FIFO_FULL7:1;
                vuint32_t FIFO_OVERRUN6:1;
                vuint32_t FIFO_OVERFLOW6:1;
                vuint32_t FIFO_EMPTY6:1;
                vuint32_t FIFO_FULL6:1;
                vuint32_t FIFO_OVERRUN5:1;
                vuint32_t FIFO_OVERFLOW5:1;
                vuint32_t FIFO_EMPTY5:1;
                vuint32_t FIFO_FULL5:1;
                vuint32_t FIFO_OVERRUN4:1;
                vuint32_t FIFO_OVERFLOW4:1;
                vuint32_t FIFO_EMPTY4:1;
                vuint32_t FIFO_FULL4:1;
                vuint32_t FIFO_OVERRUN3:1;
                vuint32_t FIFO_OVERFLOW3:1;
                vuint32_t FIFO_EMPTY3:1;
                vuint32_t FIFO_FULL3:1;
                vuint32_t FIFO_OVERRUN2:1;
                vuint32_t FIFO_OVERFLOW2:1;
                vuint32_t FIFO_EMPTY2:1;
                vuint32_t FIFO_FULL2:1;
                vuint32_t FIFO_OVERRUN1:1;
                vuint32_t FIFO_OVERFLOW1:1;
                vuint32_t FIFO_EMPTY1:1;
                vuint32_t FIFO_FULL1:1;
                vuint32_t FIFO_OVERRUN0:1;
                vuint32_t FIFO_OVERFLOW0:1;
                vuint32_t FIFO_EMPTY0:1;
                vuint32_t FIFO_FULL0:1;
            } B;
        } STATUS;               /* STATUS REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:11;
                vuint32_t NCH:5;
                  vuint32_t:6;
                vuint32_t DATA:10;
            } B;
        } FRA[8];               /* FIFO RIGHT aligned REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:11;
                vuint32_t NCH:5;
                vuint32_t DATA:10;
                  vuint32_t:6;
            } B;
        } FLA[8];               /* FIFO LEFT aligned REGISTER */

        union {
            vuint16_t R;
            struct {
                vuint16_t:7;
                vuint16_t ETOE:1;
                vuint16_t T1OE:1;
                vuint16_t T0OE:1;
                vuint16_t ADCOE:1;
                vuint16_t TGSOSM:1;
                vuint16_t MRSO:1;
                vuint16_t ICE:1;
                vuint16_t SMTO:1;
                vuint16_t MRSRE:1;
            } B;
        } CTUEFR;               /* Cross Triggering Unit Error Flag Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:6;
                vuint16_t ADC:1;
                vuint16_t T7:1;
                vuint16_t T6:1;
                vuint16_t T5:1;
                vuint16_t T4:1;
                vuint16_t T3:1;
                vuint16_t T2:1;
                vuint16_t T1:1;
                vuint16_t T0:1;
                vuint16_t MRS:1;
            } B;
        } CTUIFR;               /* Cross Triggering Unit Interrupt Flag Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t T7IE:1;
                vuint16_t T6IE:1;
                vuint16_t T5IE:1;
                vuint16_t T4IE:1;
                vuint16_t T3IE:1;
                vuint16_t T2IE:1;
                vuint16_t T1IE:1;
                vuint16_t T0IE:1;
                  vuint16_t:5;
                vuint16_t MRSDMAE:1;
                vuint16_t MRSIE:1;
                vuint16_t IEE:1;
            } B;
        } CTUIR;                /* Cross Triggering Unit Interrupt/DMA Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:8;
                vuint16_t COTR:8;
            } B;
        } COTR;                 /* Control On-Time Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t T7SG:1;
                vuint16_t T6SG:1;
                vuint16_t T5SG:1;
                vuint16_t T4SG:1;
                vuint16_t T3SG:1;
                vuint16_t T2SG:1;
                vuint16_t T1SG:1;
                vuint16_t T0SG:1;
                vuint16_t CTUADCRESET:1;
                vuint16_t CTUODIS:1;
                vuint16_t FILTERENABLE:1;
                vuint16_t CGRE:1;
                vuint16_t FGRE:1;
                vuint16_t MRSSG:1;
                vuint16_t GRE:1;
                vuint16_t TGSISRRE:1;
            } B;
        } CTUCR;                /* Cross Triggering Unit Control Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:8;
                vuint16_t FILTERVALUE:8;
            } B;
        } CTUFILTER;            /* Cross Triggering Unit Digital Filter */

        union {
            vuint16_t R;
            struct {
                vuint16_t:15;
                vuint16_t MDIS:1;
            } B;
        } CTUPCR;               /* Cross Triggering Unit Power Control */

    };                          /* end of CTU_tag */
/****************************************************************************/
/*                          MODULE : FCU                                   */
/****************************************************************************/
    struct FCU_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t MCL:1;
                vuint32_t TM:2;
                  vuint32_t:19;
                vuint32_t PS:2;
                vuint32_t FOM:2;
                vuint32_t FOP:6;
            } B;
        } MCR;                  /* Module Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t SRF0:1;
                vuint32_t SRF1:1;
                vuint32_t SRF2:1;
                vuint32_t SRF3:1;
                vuint32_t SRF4:1;
                vuint32_t SRF5:1;
                vuint32_t SRF6:1;
                vuint32_t SRF7:1;
                vuint32_t SRF8:1;
                vuint32_t SRF9:1;
                vuint32_t SRF10:1;
                vuint32_t SRF11:1;
                vuint32_t SRF12:1;
                vuint32_t SRF13:1;
                vuint32_t SRF14:1;
                vuint32_t SRF15:1;
                vuint32_t HRF15:1;
                vuint32_t HRF14:1;
                vuint32_t HRF13:1;
                vuint32_t HRF12:1;
                vuint32_t HRF11:1;
                vuint32_t HRF10:1;
                vuint32_t HRF9:1;
                vuint32_t HRF8:1;
                vuint32_t HRF7:1;
                vuint32_t HRF6:1;
                vuint32_t HRF5:1;
                vuint32_t HRF4:1;
                vuint32_t HRF3:1;
                vuint32_t HRF2:1;
                vuint32_t HRF1:1;
                vuint32_t HRF0:1;
            } B;
        } FFR;                  /* Fault Flag Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t FRSRF0:1;
                vuint32_t FRSRF1:1;
                vuint32_t FRSRF2:1;
                vuint32_t FRSRF3:1;
                vuint32_t FRSRF4:1;
                vuint32_t FRSRF5:1;
                vuint32_t FRSRF6:1;
                vuint32_t FRSRF7:1;
                vuint32_t FRSRF8:1;
                vuint32_t FRSRF9:1;
                vuint32_t FRSRF10:1;
                vuint32_t FRSRF11:1;
                vuint32_t FRSRF12:1;
                vuint32_t FRSRF13:1;
                vuint32_t FRSRF14:1;
                vuint32_t FRSRF15:1;
                vuint32_t FRHRF15:1;
                vuint32_t FRHRF14:1;
                vuint32_t FRHRF13:1;
                vuint32_t FRHRF12:1;
                vuint32_t FRHRF11:1;
                vuint32_t FRHRF10:1;
                vuint32_t FRHRF9:1;
                vuint32_t FRHRF8:1;
                vuint32_t FRHRF7:1;
                vuint32_t FRHRF6:1;
                vuint32_t FRHRF5:1;
                vuint32_t FRHRF4:1;
                vuint32_t FRHRF3:1;
                vuint32_t FRHRF2:1;
                vuint32_t FRHRF1:1;
                vuint32_t FRHRF0:1;
            } B;
        } FFFR;                 /* Frozen Fault Flag Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:2;
                vuint32_t FSRF2:1;
                vuint32_t FSRF3:1;
                vuint32_t FSRF4:1;
                vuint32_t FSRF5:1;
                vuint32_t FSRF6:1;
                vuint32_t FSRF7:1;
                vuint32_t FSRF8:1;
                vuint32_t FSRF9:1;
                vuint32_t FSRF10:1;
                vuint32_t FSRF11:1;
                vuint32_t FSRF12:1;
                vuint32_t FSRF13:1;
                vuint32_t FSRF14:1;
                vuint32_t FSRF15:1;
                vuint32_t FHRF15:1;
                vuint32_t FHRF14:1;
                vuint32_t FHRF13:1;
                vuint32_t FHRF12:1;
                vuint32_t FHRF11:1;
                vuint32_t FHRF10:1;
                vuint32_t FHRF9:1;
                vuint32_t FHRF8:1;
                vuint32_t FHRF7:1;
                vuint32_t FHRF6:1;
                vuint32_t FHRF5:1;
                vuint32_t FHRF4:1;
                vuint32_t FHRF3:1;
                vuint32_t FHRF2:1;
                vuint32_t FHRF1:1;
                vuint32_t FHRF0:1;
            } B;
        } FFGR;                 /* Fake Fault Generation Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t ESF0:1;
                vuint32_t ESF1:1;
                vuint32_t ESF2:1;
                vuint32_t ESF3:1;
                vuint32_t ESF4:1;
                vuint32_t ESF5:1;
                vuint32_t ESF6:1;
                vuint32_t ESF7:1;
                vuint32_t ESF8:1;
                vuint32_t ESF9:1;
                vuint32_t ESF10:1;
                vuint32_t ESF11:1;
                vuint32_t ESF12:1;
                vuint32_t ESF13:1;
                vuint32_t ESF14:1;
                vuint32_t ESF15:1;
                vuint32_t EHF15:1;
                vuint32_t EHF14:1;
                vuint32_t EHF13:1;
                vuint32_t EHF12:1;
                vuint32_t EHF11:1;
                vuint32_t EHF10:1;
                vuint32_t EHF9:1;
                vuint32_t EHF8:1;
                vuint32_t EHF7:1;
                vuint32_t EHF6:1;
                vuint32_t EHF5:1;
                vuint32_t EHF4:1;
                vuint32_t EHF3:1;
                vuint32_t EHF2:1;
                vuint32_t EHF1:1;
                vuint32_t EHF0:1;
            } B;
        } FER;                  /* Fault Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t KR:32;
            } B;
        } KR;                   /* Fault Collection Unit Key Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t TR:32;
            } B;
        } TR;                   /* Fault Collection Unit Timeout Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t TESF0:1;
                vuint32_t TESF1:1;
                vuint32_t TESF2:1;
                vuint32_t TESF3:1;
                vuint32_t TESF4:1;
                vuint32_t TESF5:1;
                vuint32_t TESF6:1;
                vuint32_t TESF7:1;
                vuint32_t TESF8:1;
                vuint32_t TESF9:1;
                vuint32_t TESF10:1;
                vuint32_t TESF11:1;
                vuint32_t TESF12:1;
                vuint32_t TESF13:1;
                vuint32_t TESF14:1;
                vuint32_t TESF15:1;
                vuint32_t TEHF15:1;
                vuint32_t TEHF14:1;
                vuint32_t TEHF13:1;
                vuint32_t TEHF12:1;
                vuint32_t TEHF11:1;
                vuint32_t TEHF10:1;
                vuint32_t TEHF9:1;
                vuint32_t TEHF8:1;
                vuint32_t TEHF7:1;
                vuint32_t TEHF6:1;
                vuint32_t TEHF5:1;
                vuint32_t TEHF4:1;
                vuint32_t TEHF3:1;
                vuint32_t TEHF2:1;
                vuint32_t TEHF1:1;
                vuint32_t TEHF0:1;
            } B;
        } TER;                  /* Fault Collection Unit Timeout Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t S0:1;
                vuint32_t S1:1;
                vuint32_t S2:1;
                vuint32_t S3:1;
            } B;
        } MSR;                  /* Module state register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t MCPS:4;
                  vuint32_t:12;
                vuint32_t MCAS:4;
            } B;
        } MCSR;                 /* MC state register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t FRMCPS:4;
                  vuint32_t:12;
                vuint32_t FRMCAS:4;
            } B;
        } FMCSR;                /* Frozen MC State Register */

    };                          /* end of FCU_tag */
/****************************************************************************/
/*                          MODULE : SMC - Stepper Motor Control            */
/****************************************************************************/
    struct SMC_tag {

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t MCPRE:2;
                vuint8_t MCSWAI:1;
                  vuint8_t:1;
                vuint8_t DITH:1;
                  vuint8_t:1;
                vuint8_t MCTOIF:1;
            } B;
        } CTL0;                 /* Motor Controller Control Register 0 */

        union {
            vuint8_t R;
            struct {
                vuint8_t RECIRC:1;
                  vuint8_t:6;
                vuint8_t MCTOIE:1;
            } B;
        } CTL1;                 /* Motor Controller Control Register 1 */

        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t P:11;
            } B;
        } PER;                  /* Motor Controller Period Register */

        int32_t SMC_reserved0[3];       /* (0x010 - 0x004)/4 = 0x01 */

        union {
            vuint8_t R;
            struct {
                vuint8_t MCOM:2;
                vuint8_t MCAM:2;
                  vuint8_t:2;
                vuint8_t CD:2;
            } B;
        } CC[12];               /* Motor Controller Channel Control Register 0->11 */

        int32_t SMC_reserved1;  /* (0x020 - 0x01C)/4 = 0x01 */

        union {
            vuint16_t R;
            struct {
                vuint16_t S:5;
                vuint16_t D:11;
            } B;
        } DC[12];               /* Motor Controller Duty Cycle Register 0->11 */

        int8_t SMC_reserved2[8];        /* (0x040 - 0x038) = 0x08 */

        union {
            vuint8_t R;
            struct {
                vuint8_t TOUT:8;
            } B;
        } SDTO;                 /* Shortcut detector time-out register  */

        int8_t SMC_reserved3[3];        /* (0x044 - 0x041) = 0x03 */

        union {
            vuint8_t R;
            struct {
                vuint8_t EN:8;
            } B;
        } SDE[3];               /* Shortcut detector enable register 0->2 */

        int8_t SMC_reserved4;   /* (0x048 - 0x047) = 0x01 */

        union {
            vuint8_t R;
            struct {
                vuint8_t IRQ_EN:8;
            } B;
        } SDIEN[3];             /* Shortcut detector interrupt enable register 0->2 */

        int8_t SMC_reserved5;   /* (0x04C - 0x04B) = 0x01 */

        union {
            vuint8_t R;
            struct {
                vuint8_t IRQ:8;
            } B;
        } SDI[3];               /* Shortcut detector interrupt register 0->2 */

    };                          /* end of SMC_tag */
/****************************************************************************/
/*                          MODULE : SSD - Stepper Stall Detect             */
/****************************************************************************/
    struct SSD_tag {

        union {
            vuint16_t R;
            struct {
                vuint16_t TRIG:1;
                vuint16_t STEP:2;
                vuint16_t RCIR:1;
                vuint16_t ITGDIR:1;
                vuint16_t BLNDCL:1;
                vuint16_t ITGDCL:1;
                vuint16_t RTZE:1;
                  vuint16_t:1;
                vuint16_t BLNST:1;
                vuint16_t ITGST:1;
                  vuint16_t:3;
                vuint16_t SDCPU:1;
                vuint16_t DZDIS:1;
            } B;
        } CONTROL;              /* Control & Status Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t BLNIF:1;
                vuint16_t ITGIF:1;
                  vuint16_t:5;
                vuint16_t ACOVIF:1;
                vuint16_t BLNIE:1;
                vuint16_t ITGIE:1;
                  vuint16_t:5;
                vuint16_t ACOVIE:1;
            } B;
        } IRQ;                  /* Interrupt Flag and Enable Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t ITGACC:16;
            } B;
        } ITGACC;               /* Integrator Accumulator register */

        union {
            vuint16_t R;
            struct {
                vuint16_t DCNT:16;
            } B;
        } DCNT;                 /* Down Counter Count register */

        union {
            vuint16_t R;
            struct {
                vuint16_t BLNCNTLD:16;
            } B;
        } BLNCNTLD;             /* Blanking Counter Load register */

        union {
            vuint16_t R;
            struct {
                vuint16_t ITGCNTLD:16;
            } B;
        } ITGCNTLD;             /* Integration Counter Load register */

        union {
            vuint16_t R;
            struct {
                vuint16_t:1;
                vuint16_t BLNDIV:3;
                  vuint16_t:1;
                vuint16_t ITSSDIV:3;
                  vuint16_t:2;
                vuint16_t OFFCNC:2;
                  vuint16_t:1;
                vuint16_t ACDIV:3;
            } B;
        } PRESCALE;             /* Prescaler register */

        union {
            vuint16_t R;
            struct {
                vuint16_t TMST:1;
                vuint16_t ANLOUT:1;
                vuint16_t ANLIN:1;
                vuint16_t SSDEN:1;
                vuint16_t STEP1:1;
                vuint16_t POL:1;
                vuint16_t ITG:1;
                vuint16_t DACHIZ:1;
                vuint16_t BUFHIZ:1;
                vuint16_t AMPHIZ:1;
                vuint16_t RESSHORT:1;
                vuint16_t ITSSDRV:1;
                vuint16_t ITSSDRVEN:1;
                vuint16_t REFDRV:1;
                vuint16_t REFDRVEN:1;
            } B;
        } FNTEST;               /* Functional Test Mode register */

    };                          /* end of SSD_tag */
/****************************************************************************/
/*                          MODULE : EMIOS                                  */
/****************************************************************************/
    struct EMIOS_CHANNEL_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t CADR:24;
            } B;
        } CADR;                 /* Channel A Data Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t CBDR:24;
            } B;
        } CBDR;                 /* Channel B Data Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t CCNTR:24;
            } B;
        } CCNTR;                /* Channel Counter Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t FREN:1;
                vuint32_t ODIS:1;
                vuint32_t ODISSL:2;
                vuint32_t UCPRE:2;
                vuint32_t UCPEN:1;
                vuint32_t DMA:1;
                  vuint32_t:1;
                vuint32_t IF:4;
                vuint32_t FCK:1;
                vuint32_t FEN:1;
                  vuint32_t:3;
                vuint32_t FORCMA:1;
                vuint32_t FORCMB:1;
                  vuint32_t:1;
                vuint32_t BSL:2;
                vuint32_t EDSEL:1;
                vuint32_t EDPOL:1;
                vuint32_t MODE:7;
            } B;
        } CCR;                  /* Channel Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t OVR:1;
                  vuint32_t:15;
                vuint32_t OVFL:1;
                  vuint32_t:12;
                vuint32_t UCIN:1;
                vuint32_t UCOUT:1;
                vuint32_t FLAG:1;
            } B;
        } CSR;                  /* Channel Status Register */

        union {
            vuint32_t R;        /* Alternate Channel A Data Register */
        } ALTCADR;

        uint32_t emios_channel_reserved[2];

    };                          /* end of EMIOS_CHANNEL_tag */

    struct EMIOS_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:1;
                vuint32_t MDIS:1;
                vuint32_t FRZ:1;
                vuint32_t GTBE:1;
                vuint32_t ETB:1;
                vuint32_t GPREN:1;
                  vuint32_t:6;
                vuint32_t SRV:4;
                vuint32_t GPRE:8;
                  vuint32_t:8;
            } B;
        } MCR;                  /* Module Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t F23:1;
                vuint32_t F22:1;
                vuint32_t F21:1;
                vuint32_t F20:1;
                vuint32_t F19:1;
                vuint32_t F18:1;
                vuint32_t F17:1;
                vuint32_t F16:1;
                vuint32_t F15:1;
                vuint32_t F14:1;
                vuint32_t F13:1;
                vuint32_t F12:1;
                vuint32_t F11:1;
                vuint32_t F10:1;
                vuint32_t F9:1;
                vuint32_t F8:1;
                vuint32_t F7:1;
                vuint32_t F6:1;
                vuint32_t F5:1;
                vuint32_t F4:1;
                vuint32_t F3:1;
                vuint32_t F2:1;
                vuint32_t F1:1;
                vuint32_t F0:1;
            } B;
        } GFR;                  /* Global FLAG Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t OU23:1;
                vuint32_t OU22:1;
                vuint32_t OU21:1;
                vuint32_t OU20:1;
                vuint32_t OU19:1;
                vuint32_t OU18:1;
                vuint32_t OU17:1;
                vuint32_t OU16:1;
                vuint32_t OU15:1;
                vuint32_t OU14:1;
                vuint32_t OU13:1;
                vuint32_t OU12:1;
                vuint32_t OU11:1;
                vuint32_t OU10:1;
                vuint32_t OU9:1;
                vuint32_t OU8:1;
                vuint32_t OU7:1;
                vuint32_t OU6:1;
                vuint32_t OU5:1;
                vuint32_t OU4:1;
                vuint32_t OU3:1;
                vuint32_t OU2:1;
                vuint32_t OU1:1;
                vuint32_t OU0:1;
            } B;
        } OUDR;                 /* Output Update Disable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t CHDIS23:1;
                vuint32_t CHDIS22:1;
                vuint32_t CHDIS21:1;
                vuint32_t CHDIS20:1;
                vuint32_t CHDIS19:1;
                vuint32_t CHDIS18:1;
                vuint32_t CHDIS17:1;
                vuint32_t CHDIS16:1;
                vuint32_t CHDIS15:1;
                vuint32_t CHDIS14:1;
                vuint32_t CHDIS13:1;
                vuint32_t CHDIS12:1;
                vuint32_t CHDIS11:1;
                vuint32_t CHDIS10:1;
                vuint32_t CHDIS9:1;
                vuint32_t CHDIS8:1;
                vuint32_t CHDIS7:1;
                vuint32_t CHDIS6:1;
                vuint32_t CHDIS5:1;
                vuint32_t CHDIS4:1;
                vuint32_t CHDIS3:1;
                vuint32_t CHDIS2:1;
                vuint32_t CHDIS1:1;
                vuint32_t CHDIS0:1;
            } B;
        } UCDIS;                /* Disable Channel Register */

        uint32_t emios_reserved1[4];

        struct EMIOS_CHANNEL_tag CH[28];

    };                          /* end of EMIOS_tag */
/****************************************************************************/
/*                          MODULE : pit                                    */
/****************************************************************************/
    struct PIT_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t FRZ:1;
            } B;
        } PITMCR;

        uint32_t pit_reserved1[63];     /* (0x0100 - 0x0004)/4 = 0x3F */

        struct {
            union {
                vuint32_t R;
                struct {
                    vuint32_t TSV:32;
                } B;
            } LDVAL;

            union {
                vuint32_t R;
                struct {
                    vuint32_t TVL:32;
                } B;
            } CVAL;

            union {
                vuint32_t R;
                struct {
                    vuint32_t:30;
                    vuint32_t TIE:1;
                    vuint32_t TEN:1;
                } B;
            } TCTRL;

            union {
                vuint32_t R;
                struct {
                    vuint32_t:31;
                    vuint32_t TIF:1;
                } B;
            } TFLG;
        } CH[6];

    };                          /* end of PIT_tag */
/****************************************************************************/
/*                          MODULE : i2c                                    */
/****************************************************************************/
    struct I2C_tag {
        union {
            vuint8_t R;
            struct {
                vuint8_t ADR:7;
                  vuint8_t:1;
            } B;
        } IBAD;                 /* Module Bus Address Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t IBC:8;
            } B;
        } IBFD;                 /* Module Bus Frequency Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t MDIS:1;
                vuint8_t IBIE:1;
                vuint8_t MS:1;
                vuint8_t TX:1;
                vuint8_t NOACK:1;
                vuint8_t RSTA:1;
                vuint8_t DMAEN:1;
                vuint8_t IBDOZE:1;
            } B;
        } IBCR;                 /* Module Bus Control Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t TCF:1;
                vuint8_t IAAS:1;
                vuint8_t IBB:1;
                vuint8_t IBAL:1;
                  vuint8_t:1;
                vuint8_t SRW:1;
                vuint8_t IBIF:1;
                vuint8_t RXAK:1;
            } B;
        } IBSR;                 /* Module Status Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t DATA:8;
            } B;
        } IBDR;                 /* Module Data Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t BIIE:1;
                  vuint8_t:7;
            } B;
        } IBIC;                 /* Module Interrupt Configuration Register */

    };                          /* end of I2C_tag */
/****************************************************************************/
/*                          MODULE : MPU                                    */
/****************************************************************************/
    struct MPU_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t SPERR:8;
                  vuint32_t:4;
                vuint32_t HRL:4;
                vuint32_t NSP:4;
                vuint32_t NGRD:4;
                  vuint32_t:7;
                vuint32_t VLD:1;
            } B;
        } CESR;                 /* Module Control/Error Status Register */

        uint32_t mpu_reserved1[3];      /* (0x010 - 0x004)/4 = 0x03 */

        union {
            vuint32_t R;
            struct {
                vuint32_t EADDR:32;
            } B;
        } EAR0;

        union {
            vuint32_t R;
            struct {
                vuint32_t EACD:16;
                vuint32_t EPID:8;
                vuint32_t EMN:4;
                vuint32_t EATTR:3;
                vuint32_t ERW:1;
            } B;
        } EDR0;

        union {
            vuint32_t R;
            struct {
                vuint32_t EADDR:32;
            } B;
        } EAR1;

        union {
            vuint32_t R;
            struct {
                vuint32_t EACD:16;
                vuint32_t EPID:8;
                vuint32_t EMN:4;
                vuint32_t EATTR:3;
                vuint32_t ERW:1;
            } B;
        } EDR1;

        union {
            vuint32_t R;
            struct {
                vuint32_t EADDR:32;
            } B;
        } EAR2;

        union {
            vuint32_t R;
            struct {
                vuint32_t EACD:16;
                vuint32_t EPID:8;
                vuint32_t EMN:4;
                vuint32_t EATTR:3;
                vuint32_t ERW:1;
            } B;
        } EDR2;

        union {
            vuint32_t R;
            struct {
                vuint32_t EADDR:32;
            } B;
        } EAR3;

        union {
            vuint32_t R;
            struct {
                vuint32_t EACD:16;
                vuint32_t EPID:8;
                vuint32_t EMN:4;
                vuint32_t EATTR:3;
                vuint32_t ERW:1;
            } B;
        } EDR3;

        uint32_t mpu_reserved2[244];    /* (0x0400 - 0x0030)/4 = 0x0F4 */

        struct {
            union {
                vuint32_t R;
                struct {
                    vuint32_t SRTADDR:27;
                      vuint32_t:5;
                } B;
            } WORD0;            /* Region Descriptor n Word 0 */

            union {
                vuint32_t R;
                struct {
                    vuint32_t ENDADDR:27;
                      vuint32_t:5;
                } B;
            } WORD1;            /* Region Descriptor n Word 1 */

            union {
                vuint32_t R;
                struct {
                    vuint32_t M7RE:1;
                    vuint32_t M7WE:1;
                    vuint32_t M6RE:1;
                    vuint32_t M6WE:1;
                    vuint32_t M5RE:1;
                    vuint32_t M5WE:1;
                    vuint32_t M4RE:1;
                    vuint32_t M4WE:1;
                    vuint32_t M3PE:1;
                    vuint32_t M3SM:2;
                    vuint32_t M3UM:3;
                    vuint32_t M2PE:1;
                    vuint32_t M2SM:2;
                    vuint32_t M2UM:3;
                    vuint32_t M1PE:1;
                    vuint32_t M1SM:2;
                    vuint32_t M1UM:3;
                    vuint32_t M0PE:1;
                    vuint32_t M0SM:2;
                    vuint32_t M0UM:3;
                } B;
            } WORD2;            /* Region Descriptor n Word 2 */

            union {
                vuint32_t R;
                struct {
                    vuint32_t PID:8;
                    vuint32_t PIDMASK:8;
                      vuint32_t:15;
                    vuint32_t VLD:1;
                } B;
            } WORD3;            /* Region Descriptor n Word 3 */

        } RGD[16];

        uint32_t mpu_reserved3[192];    /* (0x0800 - 0x0500)/4 = 0x0C0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t M7RE:1;
                vuint32_t M7WE:1;
                vuint32_t M6RE:1;
                vuint32_t M6WE:1;
                vuint32_t M5RE:1;
                vuint32_t M5WE:1;
                vuint32_t M4RE:1;
                vuint32_t M4WE:1;
                vuint32_t M3PE:1;
                vuint32_t M3SM:2;
                vuint32_t M3UM:3;
                vuint32_t M2PE:1;
                vuint32_t M2SM:2;
                vuint32_t M2UM:3;
                vuint32_t M1PE:1;
                vuint32_t M1SM:2;
                vuint32_t M1UM:3;
                vuint32_t M0PE:1;
                vuint32_t M0SM:2;
                vuint32_t M0UM:3;
            } B;
        } RGDAAC[16];           /* Region Descriptor Alternate Access Control n */

    };                          /* end of MPU_tag */
/****************************************************************************/
/*                          MODULE : eDMA                                   */
/****************************************************************************/

/*for "standard" format TCD (when EDMA.TCD[x].CITER.E_LINK==BITER.E_LINK=0) */
    struct EDMA_TCD_STD_tag {

        vuint32_t SADDR;        /* source address */

        vuint16_t SMOD:5;       /* source address modulo */
        vuint16_t SSIZE:3;      /* source transfer size */
        vuint16_t DMOD:5;       /* destination address modulo */
        vuint16_t DSIZE:3;      /* destination transfer size */
        vint16_t SOFF;          /* signed source address offset */

        vuint32_t NBYTES;       /* inner (“minor”) byte count */

        vint32_t SLAST;         /* last destination address adjustment, or
                                   scatter/gather address (if e_sg = 1) */

        vuint32_t DADDR;        /* destination address */

        vuint16_t CITERE_LINK:1;
        vuint16_t CITER:15;

        vint16_t DOFF;          /* signed destination address offset */

        vint32_t DLAST_SGA;

        vuint16_t BITERE_LINK:1;        /* beginning ("major") iteration count */
        vuint16_t BITER:15;

        vuint16_t BWC:2;        /* bandwidth control */
        vuint16_t MAJORLINKCH:6;        /* enable channel-to-channel link */
        vuint16_t DONE:1;       /* channel done */
        vuint16_t ACTIVE:1;     /* channel active */
        vuint16_t MAJORE_LINK:1;        /* enable channel-to-channel link */
        vuint16_t E_SG:1;       /* enable scatter/gather descriptor */
        vuint16_t D_REQ:1;      /* disable ipd_req when done */
        vuint16_t INT_HALF:1;   /* interrupt on citer = (biter >> 1) */
        vuint16_t INT_MAJ:1;    /* interrupt on major loop completion */
        vuint16_t START:1;      /* explicit channel start */

    };                          /* end of EDMA_TCD_STD_tag */

/*for "channel link" format TCD (when EDMA.TCD[x].CITER.E_LINK==BITER.E_LINK=1)*/
    struct EDMA_TCD_CHLINK_tag {

        vuint32_t SADDR;        /* source address */

        vuint16_t SMOD:5;       /* source address modulo */
        vuint16_t SSIZE:3;      /* source transfer size */
        vuint16_t DMOD:5;       /* destination address modulo */
        vuint16_t DSIZE:3;      /* destination transfer size */
        vint16_t SOFF;          /* signed source address offset */

        vuint32_t NBYTES;       /* inner (“minor”) byte count */

        vint32_t SLAST;         /* last destination address adjustment, or
                                   scatter/gather address (if e_sg = 1) */

        vuint32_t DADDR;        /* destination address */

        vuint16_t CITERE_LINK:1;
        vuint16_t CITERLINKCH:6;
        vuint16_t CITER:9;

        vint16_t DOFF;          /* signed destination address offset */

        vint32_t DLAST_SGA;

        vuint16_t BITERE_LINK:1;        /* beginning (“major”) iteration count */
        vuint16_t BITERLINKCH:6;
        vuint16_t BITER:9;

        vuint16_t BWC:2;        /* bandwidth control */
        vuint16_t MAJORLINKCH:6;        /* enable channel-to-channel link */
        vuint16_t DONE:1;       /* channel done */
        vuint16_t ACTIVE:1;     /* channel active */
        vuint16_t MAJORE_LINK:1;        /* enable channel-to-channel link */
        vuint16_t E_SG:1;       /* enable scatter/gather descriptor */
        vuint16_t D_REQ:1;      /* disable ipd_req when done */
        vuint16_t INT_HALF:1;   /* interrupt on citer = (biter >> 1) */
        vuint16_t INT_MAJ:1;    /* interrupt on major loop completion */
        vuint16_t START:1;      /* explicit channel start */

    };                          /* end of EDMA_TCD_CHLINK_tag */

    struct EDMA_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:29;
                vuint32_t ERCA:1;
                vuint32_t EDBG:1;
                  vuint32_t:1;
            } B;
        } CR;                   /* Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t VLD:1;
                  vuint32_t:15;
                vuint32_t GPE:1;
                vuint32_t CPE:1;
                vuint32_t ERRCHN:6;
                vuint32_t SAE:1;
                vuint32_t SOE:1;
                vuint32_t DAE:1;
                vuint32_t DOE:1;
                vuint32_t NCE:1;
                vuint32_t SGE:1;
                vuint32_t SBE:1;
                vuint32_t DBE:1;
            } B;
        } ESR;                  /* Error Status Register */

        int16_t EDMA_reserved1[3];      /* (0x0E - 0x08)/2 = 0x03 */

        union {
            vuint16_t R;
            struct {
                vuint16_t ERQ15:1;
                vuint16_t ERQ14:1;
                vuint16_t ERQ13:1;
                vuint16_t ERQ12:1;
                vuint16_t ERQ11:1;
                vuint16_t ERQ10:1;
                vuint16_t ERQ09:1;
                vuint16_t ERQ08:1;
                vuint16_t ERQ07:1;
                vuint16_t ERQ06:1;
                vuint16_t ERQ05:1;
                vuint16_t ERQ04:1;
                vuint16_t ERQ03:1;
                vuint16_t ERQ02:1;
                vuint16_t ERQ01:1;
                vuint16_t ERQ00:1;
            } B;
        } ERQRL;                /* DMA Enable Request Register Low */

        int16_t EDMA_reserved2[3];      /* (0x16 - 0x10)/2 = 0x03 */

        union {
            vuint16_t R;
            struct {
                vuint16_t EEI15:1;
                vuint16_t EEI14:1;
                vuint16_t EEI13:1;
                vuint16_t EEI12:1;
                vuint16_t EEI11:1;
                vuint16_t EEI10:1;
                vuint16_t EEI09:1;
                vuint16_t EEI08:1;
                vuint16_t EEI07:1;
                vuint16_t EEI06:1;
                vuint16_t EEI05:1;
                vuint16_t EEI04:1;
                vuint16_t EEI03:1;
                vuint16_t EEI02:1;
                vuint16_t EEI01:1;
                vuint16_t EEI00:1;
            } B;
        } EEIRL;                /* DMA Enable Error Interrupt Register Low */

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t SERQ:7;
            } B;
        } SERQR;                /* DMA Set Enable Request Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t CERQ:7;
            } B;
        } CERQR;                /* DMA Clear Enable Request Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t SEEI:7;
            } B;
        } SEEIR;                /* DMA Set Enable Error Interrupt Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t CEEI:7;
            } B;
        } CEEIR;                /* DMA Clear Enable Error Interrupt Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t CINT:7;
            } B;
        } CIRQR;                /* DMA Clear Interrupt Request Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t CER:7;
            } B;
        } CERR;                 /* DMA Clear error Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t SSB:7;
            } B;
        } SSBR;                 /* Set Start Bit Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:1;
                vuint8_t CDSB:7;
            } B;
        } CDSBR;                /* Clear Done Status Bit Register */

        int16_t EDMA_reserved3[3];      /* (0x26 - 0x20)/2 = 0x03 */

        union {
            vuint16_t R;
            struct {
                vuint16_t INT15:1;
                vuint16_t INT14:1;
                vuint16_t INT13:1;
                vuint16_t INT12:1;
                vuint16_t INT11:1;
                vuint16_t INT10:1;
                vuint16_t INT09:1;
                vuint16_t INT08:1;
                vuint16_t INT07:1;
                vuint16_t INT06:1;
                vuint16_t INT05:1;
                vuint16_t INT04:1;
                vuint16_t INT03:1;
                vuint16_t INT02:1;
                vuint16_t INT01:1;
                vuint16_t INT00:1;
            } B;
        } IRQRL;                /* DMA Interrupt Request Low */

        int16_t EDMA_reserved4[3];      /* (0x2E - 0x28)/2 = 0x03 */

        union {
            vuint16_t R;
            struct {
                vuint16_t ERR15:1;
                vuint16_t ERR14:1;
                vuint16_t ERR13:1;
                vuint16_t ERR12:1;
                vuint16_t ERR11:1;
                vuint16_t ERR10:1;
                vuint16_t ERR09:1;
                vuint16_t ERR08:1;
                vuint16_t ERR07:1;
                vuint16_t ERR06:1;
                vuint16_t ERR05:1;
                vuint16_t ERR04:1;
                vuint16_t ERR03:1;
                vuint16_t ERR02:1;
                vuint16_t ERR01:1;
                vuint16_t ERR00:1;
            } B;
        } ERL;                  /* DMA Error Low */

        int16_t EDMA_reserved5[3];      /* (0x36 - 0x30)/2 = 0x03 */

        union {
            vuint16_t R;
            struct {
                vuint16_t HRS15:1;
                vuint16_t HRS14:1;
                vuint16_t HRS13:1;
                vuint16_t HRS12:1;
                vuint16_t HRS11:1;
                vuint16_t HRS10:1;
                vuint16_t HRS09:1;
                vuint16_t HRS08:1;
                vuint16_t HRS07:1;
                vuint16_t HRS06:1;
                vuint16_t HRS05:1;
                vuint16_t HRS04:1;
                vuint16_t HRS03:1;
                vuint16_t HRS02:1;
                vuint16_t HRS01:1;
                vuint16_t HRS00:1;
            } B;
        } HRSL;                 /* DMA Hardware Request Status Low */

        uint32_t edma_reserved1[50];    /* (0x100 - 0x038)/4 = 0x32 */

        union {
            vuint8_t R;
            struct {
                vuint8_t ECP:1;
                vuint8_t DPA:1;
                vuint8_t GRPPRI:2;
                vuint8_t CHPRI:4;
            } B;
        } CPR[16];              /* Channel n Priority */

        uint32_t edma_reserved2[956];   /* (0x1000 - 0x0110)/4 = 0x3BC */

        struct EDMA_TCD_STD_tag TCD[16];
        /* struct EDMA_TCD_CHLINK_tag TCD[16]; */

    };                          /* end of EDMA_tag */
/****************************************************************************/
/*                          MODULE : INTC                                   */
/****************************************************************************/
    struct INTC_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:26;
                vuint32_t VTES:1;
                  vuint32_t:4;
                vuint32_t HVEN:1;
            } B;
        } MCR;                  /* Module Configuration Register */

        int32_t INTC_reserved1; /* (0x008 - 0x004)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t PRI:4;
            } B;
        } CPR;                  /* Current Priority Register */

        int32_t INTC_reserved2; /* (0x010 - 0x00C)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t VTBA:21;
                vuint32_t INTVEC:9;
                  vuint32_t:2;
            } B;
        } IACKR;                /* Interrupt Acknowledge Register */

        int32_t INTC_reserved3; /* (0x018 - 0x014)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } EOIR;                 /* End of Interrupt Register */

        int32_t INTC_reserved4; /* (0x020 - 0x01C)/4 = 0x01 */

        union {
            vuint8_t R;
            struct {
                vuint8_t:6;
                vuint8_t SET:1;
                vuint8_t CLR:1;
            } B;
        } SSCIR[8];             /* Software Set/Clear Interruput Register */

        uint32_t intc_reserved5[6];     /* (0x040 - 0x028)/4 = 0x06 */

        union {
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t PRI:4;
            } B;
        } PSR[512];             /* Software Set/Clear Interrupt Register */

    };                          /* end of INTC_tag */
/****************************************************************************/
/*                          MODULE : DSPI                                   */
/****************************************************************************/
    struct DSPI_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t MSTR:1;
                vuint32_t CONT_SCKE:1;
                vuint32_t DCONF:2;
                vuint32_t FRZ:1;
                vuint32_t MTFE:1;
                vuint32_t PCSSE:1;
                vuint32_t ROOE:1;
		vuint32_t PCSIS7:1;
		vuint32_t PCSIS6:1;
                vuint32_t PCSIS5:1;
                vuint32_t PCSIS4:1;
                vuint32_t PCSIS3:1;
                vuint32_t PCSIS2:1;
                vuint32_t PCSIS1:1;
                vuint32_t PCSIS0:1;
                vuint32_t:1;
                vuint32_t MDIS:1;
                vuint32_t DIS_TXF:1;
                vuint32_t DIS_RXF:1;
                vuint32_t CLR_TXF:1;
                vuint32_t CLR_RXF:1;
                vuint32_t SMPL_PT:2;
                  vuint32_t:7;
                vuint32_t HALT:1;
            } B;
        } MCR;                  /* Module Configuration Register */

        uint32_t dspi_reserved1;

        union {
            vuint32_t R;
            struct {
                vuint32_t TCNT:16;
                  vuint32_t:16;
            } B;
        } TCR;

        union {
            vuint32_t R;
            struct {
                vuint32_t DBR:1;
                vuint32_t FMSZ:4;
                vuint32_t CPOL:1;
                vuint32_t CPHA:1;
                vuint32_t LSBFE:1;
                vuint32_t PCSSCK:2;
                vuint32_t PASC:2;
                vuint32_t PDT:2;
                vuint32_t PBR:2;
                vuint32_t CSSCK:4;
                vuint32_t ASC:4;
                vuint32_t DT:4;
                vuint32_t BR:4;
            } B;
        } CTAR[8];              /* Clock and Transfer Attributes Registers */

        union {
            vuint32_t R;
            struct {
                vuint32_t TCF:1;
                vuint32_t TXRXS:1;
                  vuint32_t:1;
                vuint32_t EOQF:1;
                vuint32_t TFUF:1;
                  vuint32_t:1;
                vuint32_t TFFF:1;
                  vuint32_t:5;
                vuint32_t RFOF:1;
                  vuint32_t:1;
                vuint32_t RFDF:1;
                  vuint32_t:1;
                vuint32_t TXCTR:4;
                vuint32_t TXNXTPTR:4;
                vuint32_t RXCTR:4;
                vuint32_t POPNXTPTR:4;
            } B;
        } SR;                   /* Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t TCFRE:1;
                  vuint32_t:2;
                vuint32_t EOQFRE:1;
                vuint32_t TFUFRE:1;
                  vuint32_t:1;
                vuint32_t TFFFRE:1;
                vuint32_t TFFFDIRS:1;
                  vuint32_t:4;
                vuint32_t RFOFRE:1;
                  vuint32_t:1;
                vuint32_t RFDFRE:1;
                vuint32_t RFDFDIRS:1;
                  vuint32_t:16;
            } B;
        } RSER;                 /* DMA/Interrupt Request Select and Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t CONT:1;
                vuint32_t CTAS:3;
                vuint32_t EOQ:1;
                vuint32_t CTCNT:1;
                  vuint32_t:2;
		vuint32_t PCS7:1;
		vuint32_t PCS6:1;
                vuint32_t PCS5:1;
                vuint32_t PCS4:1;
                vuint32_t PCS3:1;
                vuint32_t PCS2:1;
                vuint32_t PCS1:1;
                vuint32_t PCS0:1;
                vuint32_t TXDATA:16;
            } B;
        } PUSHR;                /* PUSH TX FIFO Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RXDATA:16;
            } B;
        } POPR;                 /* POP RX FIFO Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t TXCMD:16;
                vuint32_t TXDATA:16;
            } B;
        } TXFR[5];              /* Transmit FIFO Registers */

        vuint32_t DSPI_reserved_txf[11];

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RXDATA:16;
            } B;
        } RXFR[5];              /* Receive FIFO Registers */

        vuint32_t DSPI_reserved_rxf[12];

        union {
            vuint32_t R;
            struct {
                vuint32_t MTOE:1;
                  vuint32_t:1;
                vuint32_t MTOCNT:6;
                  vuint32_t:4;
                vuint32_t TXSS:1;
                vuint32_t TPOL:1;
                vuint32_t TRRE:1;
                vuint32_t CID:1;
                vuint32_t DCONT:1;
                vuint32_t DSICTAS:3;
                  vuint32_t:6;
                vuint32_t DPCS5:1;
                vuint32_t DPCS4:1;
                vuint32_t DPCS3:1;
                vuint32_t DPCS2:1;
                vuint32_t DPCS1:1;
                vuint32_t DPCS0:1;
            } B;
        } DSICR;                /* DSI Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t SER_DATA:16;
            } B;
        } SDR;                  /* DSI Serialization Data Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t ASER_DATA:16;
            } B;
        } ASDR;                 /* DSI Alternate Serialization Data Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t COMP_DATA:16;
            } B;
        } COMPR;                /* DSI Transmit Comparison Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t DESER_DATA:16;
            } B;
        } DDR;                  /* DSI deserialization Data Register */

    };                          /* end of DSPI_tag */
/****************************************************************************/
/*                          MODULE : FlexCAN                                */
/****************************************************************************/
    struct FLEXCAN_BUF_t {
        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t CODE:4;
                  vuint32_t:1;
                vuint32_t SRR:1;
                vuint32_t IDE:1;
                vuint32_t RTR:1;
                vuint32_t LENGTH:4;
                vuint32_t TIMESTAMP:16;
            } B;
        } CS;

        union {
            vuint32_t R;
            struct {
                vuint32_t PRIO:3;
                vuint32_t STD_ID:11;
                vuint32_t EXT_ID:18;
            } B;
        } ID;

        union {
            /*vuint8_t  B[8]; *//* Data buffer in Bytes (8 bits) */
            /*vuint16_t H[4]; *//* Data buffer in Half-words (16 bits) */
            vuint32_t W[2];     /* Data buffer in words (32 bits) */
            /*vuint32_t R[2]; *//* Data buffer in words (32 bits) */
        } DATA;

    };                          /* end of FLEXCAN_BUF_t */

    struct FLEXCAN_RXFIFO_t {
        union {
            vuint32_t R;
            struct {
                vuint32_t:9;
                vuint32_t SRR:1;
                vuint32_t IDE:1;
                vuint32_t RTR:1;
                vuint32_t LENGTH:4;
                vuint32_t TIMESTAMP:16;
            } B;
        } CS;

        union {
            vuint32_t R;
            struct {
		vuint32_t:3;
                vuint32_t STD_ID:11;
                vuint32_t EXT_ID:18;
            } B;
        } ID;

        union {
            /*vuint8_t  B[8]; *//* Data buffer in Bytes (8 bits) */
            /*vuint16_t H[4]; *//* Data buffer in Half-words (16 bits) */
            vuint32_t W[2];     /* Data buffer in words (32 bits) */
            /*vuint32_t R[2]; *//* Data buffer in words (32 bits) */
        } DATA;

        uint32_t FLEXCAN_RXFIFO_reserved[20];   /* {0x00E0-0x0090}/0x4 = 0x14 */

        union {
            vuint32_t R;
        } IDTABLE[8];

    };                          /* end of FLEXCAN_RXFIFO_t */

    struct FLEXCAN_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t MDIS:1;
                vuint32_t FRZ:1;
                vuint32_t FEN:1;
                vuint32_t HALT:1;
                vuint32_t NOTRDY:1;
                vuint32_t WAKMSK:1;
                vuint32_t SOFTRST:1;
                vuint32_t FRZACK:1;
                vuint32_t SUPV:1;
                vuint32_t SLFWAK:1;
                vuint32_t WRNEN:1;
                vuint32_t LPMACK:1;
                vuint32_t WAKSRC:1;
                vuint32_t:1;
                vuint32_t SRXDIS:1;
                vuint32_t BCC:1;
                  vuint32_t:2;
                vuint32_t LPRIO_EN:1;
                vuint32_t AEN:1;
                  vuint32_t:2;
                vuint32_t IDAM:2;
                  vuint32_t:2;
                vuint32_t MAXMB:6;
            } B;
        } MCR;                  /* Module Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t PRESDIV:8;
                vuint32_t RJW:2;
                vuint32_t PSEG1:3;
                vuint32_t PSEG2:3;
                vuint32_t BOFFMSK:1;
                vuint32_t ERRMSK:1;
                vuint32_t CLKSRC:1;
                vuint32_t LPB:1;
                vuint32_t TWRNMSK:1;
                vuint32_t RWRNMSK:1;
                  vuint32_t:2;
                vuint32_t SMP:1;
                vuint32_t BOFFREC:1;
                vuint32_t TSYN:1;
                vuint32_t LBUF:1;
                vuint32_t LOM:1;
                vuint32_t PROPSEG:3;
            } B;
        } CR;                   /* Control Register */

        union {
            vuint32_t R;
        } TIMER;                /* Free Running Timer */

        uint32_t FLEXCAN_reserved1;

        union {
            vuint32_t R;
            struct {
                vuint32_t MI:32;
            } B;
        } RXGMASK;              /* RX Global Mask */

        union {
            vuint32_t R;
            struct {
                vuint32_t MI:32;
            } B;
        } RX14MASK;             /* RX 14 Mask */

        union {
            vuint32_t R;
            struct {
                vuint32_t MI:32;
            } B;
        } RX15MASK;             /* RX 15 Mask */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RXECNT:8;
                vuint32_t TXECNT:8;
            } B;
        } ECR;                  /* Error Counter Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:14;
                vuint32_t TWRNINT:1;
                vuint32_t RWRNINT:1;
                vuint32_t BIT1ERR:1;
                vuint32_t BIT0ERR:1;
                vuint32_t ACKERR:1;
                vuint32_t CRCERR:1;
                vuint32_t FRMERR:1;
                vuint32_t STFERR:1;
                vuint32_t TXWRN:1;
                vuint32_t RXWRN:1;
                vuint32_t IDLE:1;
                vuint32_t TXRX:1;
                vuint32_t FLTCONF:2;
                  vuint32_t:1;
                vuint32_t BOFFINT:1;
                vuint32_t ERRINT:1;
                vuint32_t WAKINT:1;
            } B;
        } ESR;                  /* Error and Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t BUF63M:1;
                vuint32_t BUF62M:1;
                vuint32_t BUF61M:1;
                vuint32_t BUF60M:1;
                vuint32_t BUF59M:1;
                vuint32_t BUF58M:1;
                vuint32_t BUF57M:1;
                vuint32_t BUF56M:1;
                vuint32_t BUF55M:1;
                vuint32_t BUF54M:1;
                vuint32_t BUF53M:1;
                vuint32_t BUF52M:1;
                vuint32_t BUF51M:1;
                vuint32_t BUF50M:1;
                vuint32_t BUF49M:1;
                vuint32_t BUF48M:1;
                vuint32_t BUF47M:1;
                vuint32_t BUF46M:1;
                vuint32_t BUF45M:1;
                vuint32_t BUF44M:1;
                vuint32_t BUF43M:1;
                vuint32_t BUF42M:1;
                vuint32_t BUF41M:1;
                vuint32_t BUF40M:1;
                vuint32_t BUF39M:1;
                vuint32_t BUF38M:1;
                vuint32_t BUF37M:1;
                vuint32_t BUF36M:1;
                vuint32_t BUF35M:1;
                vuint32_t BUF34M:1;
                vuint32_t BUF33M:1;
                vuint32_t BUF32M:1;
            } B;
        } IMRH;                 /* Interruput Masks Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t BUF31M:1;
                vuint32_t BUF30M:1;
                vuint32_t BUF29M:1;
                vuint32_t BUF28M:1;
                vuint32_t BUF27M:1;
                vuint32_t BUF26M:1;
                vuint32_t BUF25M:1;
                vuint32_t BUF24M:1;
                vuint32_t BUF23M:1;
                vuint32_t BUF22M:1;
                vuint32_t BUF21M:1;
                vuint32_t BUF20M:1;
                vuint32_t BUF19M:1;
                vuint32_t BUF18M:1;
                vuint32_t BUF17M:1;
                vuint32_t BUF16M:1;
                vuint32_t BUF15M:1;
                vuint32_t BUF14M:1;
                vuint32_t BUF13M:1;
                vuint32_t BUF12M:1;
                vuint32_t BUF11M:1;
                vuint32_t BUF10M:1;
                vuint32_t BUF09M:1;
                vuint32_t BUF08M:1;
                vuint32_t BUF07M:1;
                vuint32_t BUF06M:1;
                vuint32_t BUF05M:1;
                vuint32_t BUF04M:1;
                vuint32_t BUF03M:1;
                vuint32_t BUF02M:1;
                vuint32_t BUF01M:1;
                vuint32_t BUF00M:1;
            } B;
        } IMRL;                 /* Interruput Masks Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t BUF63I:1;
                vuint32_t BUF62I:1;
                vuint32_t BUF61I:1;
                vuint32_t BUF60I:1;
                vuint32_t BUF59I:1;
                vuint32_t BUF58I:1;
                vuint32_t BUF57I:1;
                vuint32_t BUF56I:1;
                vuint32_t BUF55I:1;
                vuint32_t BUF54I:1;
                vuint32_t BUF53I:1;
                vuint32_t BUF52I:1;
                vuint32_t BUF51I:1;
                vuint32_t BUF50I:1;
                vuint32_t BUF49I:1;
                vuint32_t BUF48I:1;
                vuint32_t BUF47I:1;
                vuint32_t BUF46I:1;
                vuint32_t BUF45I:1;
                vuint32_t BUF44I:1;
                vuint32_t BUF43I:1;
                vuint32_t BUF42I:1;
                vuint32_t BUF41I:1;
                vuint32_t BUF40I:1;
                vuint32_t BUF39I:1;
                vuint32_t BUF38I:1;
                vuint32_t BUF37I:1;
                vuint32_t BUF36I:1;
                vuint32_t BUF35I:1;
                vuint32_t BUF34I:1;
                vuint32_t BUF33I:1;
                vuint32_t BUF32I:1;
            } B;
        } IFRH;                 /* Interruput Flag Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t BUF31I:1;
                vuint32_t BUF30I:1;
                vuint32_t BUF29I:1;
                vuint32_t BUF28I:1;
                vuint32_t BUF27I:1;
                vuint32_t BUF26I:1;
                vuint32_t BUF25I:1;
                vuint32_t BUF24I:1;
                vuint32_t BUF23I:1;
                vuint32_t BUF22I:1;
                vuint32_t BUF21I:1;
                vuint32_t BUF20I:1;
                vuint32_t BUF19I:1;
                vuint32_t BUF18I:1;
                vuint32_t BUF17I:1;
                vuint32_t BUF16I:1;
                vuint32_t BUF15I:1;
                vuint32_t BUF14I:1;
                vuint32_t BUF13I:1;
                vuint32_t BUF12I:1;
                vuint32_t BUF11I:1;
                vuint32_t BUF10I:1;
                vuint32_t BUF09I:1;
                vuint32_t BUF08I:1;
                vuint32_t BUF07I:1;
                vuint32_t BUF06I:1;
                vuint32_t BUF05I:1;
                vuint32_t BUF04I:1;
                vuint32_t BUF03I:1;
                vuint32_t BUF02I:1;
                vuint32_t BUF01I:1;
                vuint32_t BUF00I:1;
            } B;
        } IFRL;                 /* Interruput Flag Register */

        uint32_t FLEXCAN_reserved2[19]; /* {0x0080-0x0034}/0x4 = 0x13 */

/****************************************************************************/
/* Use either Standard Buffer Structure OR RX FIFO and Buffer Structure     */
/****************************************************************************/
        /* Standard Buffer Structure */
        struct FLEXCAN_BUF_t BUF[64];

        /* RX FIFO and Buffer Structure */
        /*struct FLEXCAN_RXFIFO_t RXFIFO; */
        /*struct FLEXCAN_BUF_t BUF[56];   */
/****************************************************************************/

        uint32_t FLEXCAN_reserved3[256];        /* {0x0880-0x0480}/0x4 = 0x100 */

        union {
            vuint32_t R;
            struct {
                vuint32_t MI:32;
            } B;
        } RXIMR[64];            /* RX Individual Mask Registers */

    };                          /* end of FLEXCAN_tag */
/****************************************************************************/
/*                          MODULE : DMAMUX                                 */
/****************************************************************************/
    struct DMAMUX_tag {
        union {
            vuint8_t R;
            struct {
                vuint8_t ENBL:1;
                vuint8_t TRIG:1;
                vuint8_t SOURCE:6;
            } B;
        } CHCONFIG[16];         /* DMA Channel Configuration Register */

    };                          /* end of DMAMUX_tag */
/****************************************************************************/
/*                          MODULE : FlexRay                                */
/****************************************************************************/

    typedef union uMVR {
        vuint16_t R;
        struct {
            vuint16_t CHIVER:8; /* CHI Version Number */
            vuint16_t PEVER:8;  /* PE Version Number */
        } B;
    } MVR_t;

    typedef union uMCR {
        vuint16_t R;
        struct {
            vuint16_t MEN:1;    /* module enable */
              vuint16_t:1;
            vuint16_t SCMD:1;   /* single channel mode */
            vuint16_t CHB:1;    /* channel B enable */
            vuint16_t CHA:1;    /* channel A enable */
            vuint16_t SFFE:1;   /* synchronization frame filter enable */
              vuint16_t:5;
            vuint16_t CLKSEL:1; /* protocol engine clock source select */
            vuint16_t BITRATE:3;       /* protocol engine clock prescaler */
              vuint16_t:1;
        } B;
    } MCR_t;
    typedef union uSTBSCR {
        vuint16_t R;
        struct {
            vuint16_t WMD:1;    /* write mode */
            vuint16_t STBSSEL:7;        /* strobe signal select */
              vuint16_t:3;
            vuint16_t ENB:1;    /* strobe signal enable */
              vuint16_t:2;
            vuint16_t STBPSEL:2;        /* strobe port select */
        } B;
    } STBSCR_t;
    typedef union uSTBPCR {
        vuint16_t R;
        struct {
            vuint16_t:12;
            vuint16_t STB3EN:1; /* strobe port enable */
            vuint16_t STB2EN:1; /* strobe port enable */
            vuint16_t STB1EN:1; /* strobe port enable */
            vuint16_t STB0EN:1; /* strobe port enable */
        } B;
    } STBPCR_t;

    typedef union uMBDSR {
        vuint16_t R;
        struct {
            vuint16_t:1;
            vuint16_t MBSEG2DS:7;       /* message buffer segment 2 data size */
              vuint16_t:1;
            vuint16_t MBSEG1DS:7;       /* message buffer segment 1 data size */
        } B;
    } MBDSR_t;

    typedef union uMBSSUTR {
        vuint16_t R;
        struct {

            vuint16_t:2;
            vuint16_t LAST_MB_SEG1:6;   /* last message buffer control register for message buffer segment 1 */
              vuint16_t:2;
            vuint16_t LAST_MB_UTIL:6;   /* last message buffer utilized */
        } B;
    } MBSSUTR_t;

    typedef union uPOCR {
        vuint16_t R;
        vuint8_t byte[2];
        struct {
            vuint16_t WME:1;    /* write mode external correction command */
              vuint16_t:3;
            vuint16_t EOC_AP:2; /* external offset correction application */
            vuint16_t ERC_AP:2; /* external rate correction application */
            vuint16_t BSY:1;    /* command write busy / write mode command */
              vuint16_t:3;
            vuint16_t POCCMD:4; /* protocol command */
        } B;
    } POCR_t;
/* protocol commands */
    typedef union uGIFER {
        vuint16_t R;
        struct {
            vuint16_t MIF:1;    /* module interrupt flag */
            vuint16_t PRIF:1;   /* protocol interrupt flag */
            vuint16_t CHIF:1;   /* CHI interrupt flag */
            vuint16_t WKUPIF:1; /* wakeup interrupt flag */
            vuint16_t FNEBIF:1; /* receive FIFO channel B not empty interrupt flag */
            vuint16_t FNEAIF:1; /* receive FIFO channel A not empty interrupt flag */
            vuint16_t RBIF:1;   /* receive message buffer interrupt flag */
            vuint16_t TBIF:1;   /* transmit buffer interrupt flag */
            vuint16_t MIE:1;    /* module interrupt enable */
            vuint16_t PRIE:1;   /* protocol interrupt enable */
            vuint16_t CHIE:1;   /* CHI interrupt enable */
            vuint16_t WKUPIE:1; /* wakeup interrupt enable */
            vuint16_t FNEBIE:1; /* receive FIFO channel B not empty interrupt enable */
            vuint16_t FNEAIE:1; /* receive FIFO channel A not empty interrupt enable */
            vuint16_t RBIE:1;   /* receive message buffer interrupt enable */
            vuint16_t TBIE:1;   /* transmit buffer interrupt enable */
        } B;
    } GIFER_t;
    typedef union uPIFR0 {
        vuint16_t R;
        struct {
            vuint16_t FATLIF:1; /* fatal protocol error interrupt flag */
            vuint16_t INTLIF:1; /* internal protocol error interrupt flag */
            vuint16_t ILCFIF:1; /* illegal protocol configuration flag */
            vuint16_t CSAIF:1;  /* cold start abort interrupt flag */
            vuint16_t MRCIF:1;  /* missing rate correctio interrupt flag */
            vuint16_t MOCIF:1;  /* missing offset correctio interrupt flag */
            vuint16_t CCLIF:1;  /* clock correction limit reached interrupt flag */
            vuint16_t MXSIF:1;  /* max sync frames detected interrupt flag */
            vuint16_t MTXIF:1;  /* media access test symbol received flag */
            vuint16_t LTXBIF:1; /* pdLatestTx violation on channel B interrupt flag */
            vuint16_t LTXAIF:1; /* pdLatestTx violation on channel A interrupt flag */
            vuint16_t TBVBIF:1; /* Transmission across boundary on channel B Interrupt Flag */
            vuint16_t TBVAIF:1; /* Transmission across boundary on channel A Interrupt Flag */
            vuint16_t TI2IF:1;  /* timer 2 expired interrupt flag */
            vuint16_t TI1IF:1;  /* timer 1 expired interrupt flag */
            vuint16_t CYSIF:1;  /* cycle start interrupt flag */
        } B;
    } PIFR0_t;
    typedef union uPIFR1 {
        vuint16_t R;
        struct {
            vuint16_t EMCIF:1;  /* error mode changed interrupt flag */
            vuint16_t IPCIF:1;  /* illegal protocol command interrupt flag */
            vuint16_t PECFIF:1; /* protocol engine communication failure interrupt flag */
            vuint16_t PSCIF:1;  /* Protocol State Changed Interrupt Flag */
            vuint16_t SSI3IF:1; /* slot status counter incremented interrupt flag */
            vuint16_t SSI2IF:1; /* slot status counter incremented interrupt flag */
            vuint16_t SSI1IF:1; /* slot status counter incremented interrupt flag */
            vuint16_t SSI0IF:1; /* slot status counter incremented interrupt flag */
              vuint16_t:2;
            vuint16_t EVTIF:1;  /* even cycle table written interrupt flag */
            vuint16_t ODTIF:1;  /* odd cycle table written interrupt flag */
              vuint16_t:4;
        } B;
    } PIFR1_t;
    typedef union uPIER0 {
        vuint16_t R;
        struct {
            vuint16_t FATLIE:1; /* fatal protocol error interrupt enable */
            vuint16_t INTLIE:1; /* internal protocol error interrupt interrupt enable  */
            vuint16_t ILCFIE:1; /* illegal protocol configuration interrupt enable */
            vuint16_t CSAIE:1;  /* cold start abort interrupt enable */
            vuint16_t MRCIE:1;  /* missing rate correctio interrupt enable */
            vuint16_t MOCIE:1;  /* missing offset correctio interrupt enable */
            vuint16_t CCLIE:1;  /* clock correction limit reached interrupt enable */
            vuint16_t MXSIE:1;  /* max sync frames detected interrupt enable */
            vuint16_t MTXIE:1;  /* media access test symbol received interrupt enable */
            vuint16_t LTXBIE:1; /* pdLatestTx violation on channel B interrupt enable */
            vuint16_t LTXAIE:1; /* pdLatestTx violation on channel A interrupt enable */
            vuint16_t TBVBIE:1; /* Transmission across boundary on channel B Interrupt enable */
            vuint16_t TBVAIE:1; /* Transmission across boundary on channel A Interrupt enable */
            vuint16_t TI2IE:1;  /* timer 2 expired interrupt enable */
            vuint16_t TI1IE:1;  /* timer 1 expired interrupt enable */
            vuint16_t CYSIE:1;  /* cycle start interrupt enable */
        } B;
    } PIER0_t;
    typedef union uPIER1 {
        vuint16_t R;
        struct {
            vuint16_t EMCIE:1;  /* error mode changed interrupt enable */
            vuint16_t IPCIE:1;  /* illegal protocol command interrupt enable */
            vuint16_t PECFIE:1; /* protocol engine communication failure interrupt enable */
            vuint16_t PSCIE:1;  /* Protocol State Changed Interrupt enable */
            vuint16_t SSI3IE:1; /* slot status counter incremented interrupt enable */
            vuint16_t SSI2IE:1; /* slot status counter incremented interrupt enable */
            vuint16_t SSI1IE:1; /* slot status counter incremented interrupt enable */
            vuint16_t SSI0IE:1; /* slot status counter incremented interrupt enable */
              vuint16_t:2;
            vuint16_t EVTIE:1;  /* even cycle table written interrupt enable */
            vuint16_t ODTIE:1;  /* odd cycle table written interrupt enable */
              vuint16_t:4;
        } B;
    } PIER1_t;
    typedef union uCHIERFR {
        vuint16_t R;
        struct {
            vuint16_t FRLBEF:1; /* flame lost channel B error flag */
            vuint16_t FRLAEF:1; /* frame lost channel A error flag */
            vuint16_t PCMIEF:1; /* command ignored error flag */
            vuint16_t FOVBEF:1; /* receive FIFO overrun channel B error flag */
            vuint16_t FOVAEF:1; /* receive FIFO overrun channel A error flag */
            vuint16_t MSBEF:1;  /* message buffer search error flag */
            vuint16_t MBUEF:1;  /* message buffer utilization error flag */
            vuint16_t LCKEF:1;  /* lock error flag */
            vuint16_t DBLEF:1;  /* double transmit message buffer lock error flag */
            vuint16_t SBCFEF:1; /* system bus communication failure error flag */
            vuint16_t FIDEF:1;  /* frame ID error flag */
            vuint16_t DPLEF:1;  /* dynamic payload length error flag */
            vuint16_t SPLEF:1;  /* static payload length error flag */
            vuint16_t NMLEF:1;  /* network management length error flag */
            vuint16_t NMFEF:1;  /* network management frame error flag */
            vuint16_t ILSAEF:1; /* illegal access error flag */
        } B;
    } CHIERFR_t;
    typedef union uMBIVEC {
        vuint16_t R;
        struct {

            vuint16_t:2;
            vuint16_t TBIVEC:6; /* transmit buffer interrupt vector */
              vuint16_t:2;
            vuint16_t RBIVEC:6; /* receive buffer interrupt vector */
        } B;
    } MBIVEC_t;

    typedef union uPSR0 {
        vuint16_t R;
        struct {
            vuint16_t ERRMODE:2;        /* error mode */
            vuint16_t SLOTMODE:2;       /* slot mode */
              vuint16_t:1;
            vuint16_t PROTSTATE:3;      /* protocol state */
            vuint16_t SUBSTATE:4;       /* protocol sub state */
              vuint16_t:1;
            vuint16_t WAKEUPSTATUS:3;   /* wakeup status */
        } B;
    } PSR0_t;

/* protocol states */
/* protocol sub-states */
/* wakeup status */
    typedef union uPSR1 {
        vuint16_t R;
        struct {
            vuint16_t CSAA:1;   /* cold start attempt abort flag */
            vuint16_t CSP:1;    /* cold start path */
              vuint16_t:1;
            vuint16_t REMCSAT:5;        /* remanining coldstart attempts */
            vuint16_t CPN:1;    /* cold start noise path */
            vuint16_t HHR:1;    /* host halt request pending */
            vuint16_t FRZ:1;    /* freeze occured */
            vuint16_t APTAC:5;  /* allow passive to active counter */
        } B;
    } PSR1_t;
    typedef union uPSR2 {
        vuint16_t R;
        struct {
            vuint16_t NBVB:1;   /* NIT boundary violation on channel B */
            vuint16_t NSEB:1;   /* NIT syntax error on channel B */
            vuint16_t STCB:1;   /* symbol window transmit conflict on channel B */
            vuint16_t SBVB:1;   /* symbol window boundary violation on channel B */
            vuint16_t SSEB:1;   /* symbol window syntax error on channel B */
            vuint16_t MTB:1;    /* media access test symbol MTS received on channel B */
            vuint16_t NBVA:1;   /* NIT boundary violation on channel A */
            vuint16_t NSEA:1;   /* NIT syntax error on channel A */
            vuint16_t STCA:1;   /* symbol window transmit conflict on channel A */
            vuint16_t SBVA:1;   /* symbol window boundary violation on channel A */
            vuint16_t SSEA:1;   /* symbol window syntax error on channel A */
            vuint16_t MTA:1;    /* media access test symbol MTS received on channel A */
            vuint16_t CLKCORRFAILCNT:4; /* clock correction failed counter */
        } B;
    } PSR2_t;
    typedef union uPSR3 {
        vuint16_t R;
        struct {
            vuint16_t:2;
            vuint16_t WUB:1;    /* wakeup symbol received on channel B */
            vuint16_t ABVB:1;   /* aggregated boundary violation on channel B */
            vuint16_t AACB:1;   /* aggregated additional communication on channel B */
            vuint16_t ACEB:1;   /* aggregated content error on channel B */
            vuint16_t ASEB:1;   /* aggregated syntax error on channel B */
            vuint16_t AVFB:1;   /* aggregated valid frame on channel B */
              vuint16_t:2;
            vuint16_t WUA:1;    /* wakeup symbol received on channel A */
            vuint16_t ABVA:1;   /* aggregated boundary violation on channel A */
            vuint16_t AACA:1;   /* aggregated additional communication on channel A */
            vuint16_t ACEA:1;   /* aggregated content error on channel A */
            vuint16_t ASEA:1;   /* aggregated syntax error on channel A */
            vuint16_t AVFA:1;   /* aggregated valid frame on channel A */
        } B;
    } PSR3_t;
    typedef union uCIFRR {
        vuint16_t R;
        struct {
            vuint16_t:8;
            vuint16_t MIFR:1;   /* module interrupt flag */
            vuint16_t PRIFR:1;  /* protocol interrupt flag */
            vuint16_t CHIFR:1;  /* CHI interrupt flag */
            vuint16_t WUPIFR:1; /* wakeup interrupt flag */
            vuint16_t FNEBIFR:1;        /* receive fifo channel B no empty interrupt flag */
            vuint16_t FNEAIFR:1;        /* receive fifo channel A no empty interrupt flag */
            vuint16_t RBIFR:1;  /* receive message buffer interrupt flag */
            vuint16_t TBIFR:1;  /* transmit buffer interrupt flag */
        } B;
    } CIFRR_t;
    typedef union uSFCNTR {
        vuint16_t R;
        struct {
            vuint16_t SFEVB:4;  /* sync frames channel B, even cycle */
            vuint16_t SFEVA:4;  /* sync frames channel A, even cycle */
            vuint16_t SFODB:4;  /* sync frames channel B, odd cycle */
            vuint16_t SFODA:4;  /* sync frames channel A, odd cycle */
        } B;
    } SFCNTR_t;

    typedef union uSFTCCSR {
        vuint16_t R;
        struct {
            vuint16_t ELKT:1;   /* even cycle tables lock and unlock trigger */
            vuint16_t OLKT:1;   /* odd cycle tables lock and unlock trigger */
            vuint16_t CYCNUM:6; /* cycle number */
            vuint16_t ELKS:1;   /* even cycle tables lock status */
            vuint16_t OLKS:1;   /* odd cycle tables lock status */
            vuint16_t EVAL:1;   /* even cycle tables valid */
            vuint16_t OVAL:1;   /* odd cycle tables valid */
              vuint16_t:1;
            vuint16_t OPT:1;    /*one pair trigger */
            vuint16_t SDVEN:1;  /* sync frame deviation table enable */
            vuint16_t SIDEN:1;  /* sync frame ID table enable */
        } B;
    } SFTCCSR_t;
    typedef union uSFIDRFR {
        vuint16_t R;
        struct {
            vuint16_t:6;
            vuint16_t SYNFRID:10;       /* sync frame rejection ID */
        } B;
    } SFIDRFR_t;

    typedef union uTICCR {
        vuint16_t R;
        struct {
            vuint16_t:2;
            vuint16_t T2CFG:1;  /* timer 2 configuration */
            vuint16_t T2REP:1;  /* timer 2 repetitive mode */
              vuint16_t:1;
            vuint16_t T2SP:1;   /* timer 2 stop */
            vuint16_t T2TR:1;   /* timer 2 trigger */
            vuint16_t T2ST:1;   /* timer 2 state */
              vuint16_t:3;
            vuint16_t T1REP:1;  /* timer 1 repetitive mode */
              vuint16_t:1;
            vuint16_t T1SP:1;   /* timer 1 stop */
            vuint16_t T1TR:1;   /* timer 1 trigger */
            vuint16_t T1ST:1;   /* timer 1 state */

        } B;
    } TICCR_t;
    typedef union uTI1CYSR {
        vuint16_t R;
        struct {
            vuint16_t:2;
            vuint16_t TI1CYCVAL:6;      /* timer 1 cycle filter value */
              vuint16_t:2;
            vuint16_t TI1CYCMSK:6;      /* timer 1 cycle filter mask */

        } B;
    } TI1CYSR_t;

    typedef union uSSSR {
        vuint16_t R;
        struct {
            vuint16_t WMD:1;    /* write mode */
              vuint16_t:1;
            vuint16_t SEL:2;    /* static slot number */
              vuint16_t:1;
            vuint16_t SLOTNUMBER:11;    /* selector */
        } B;
    } SSSR_t;

    typedef union uSSCCR {
        vuint16_t R;
        struct {
            vuint16_t WMD:1;    /* write mode */
              vuint16_t:1;
            vuint16_t SEL:2;    /* selector */
              vuint16_t:1;
            vuint16_t CNTCFG:2; /* counter configuration */
            vuint16_t MCY:1;    /* multi cycle selection */
            vuint16_t VFR:1;    /* valid frame selection */
            vuint16_t SYF:1;    /* sync frame selection */
            vuint16_t NUF:1;    /* null frame selection  */
            vuint16_t SUF:1;    /* startup frame selection */
            vuint16_t STATUSMASK:4;     /* slot status mask */
        } B;
    } SSCCR_t;
    typedef union uSSR {
        vuint16_t R;
        struct {
            vuint16_t VFB:1;    /* valid frame on channel B */
            vuint16_t SYB:1;    /* valid sync frame on channel B */
            vuint16_t NFB:1;    /* valid null frame on channel B */
            vuint16_t SUB:1;    /* valid startup frame on channel B */
            vuint16_t SEB:1;    /* syntax error on channel B */
            vuint16_t CEB:1;    /* content error on channel B */
            vuint16_t BVB:1;    /* boundary violation on channel B */
            vuint16_t TCB:1;    /* tx conflict on channel B */
            vuint16_t VFA:1;    /* valid frame on channel A */
            vuint16_t SYA:1;    /* valid sync frame on channel A */
            vuint16_t NFA:1;    /* valid null frame on channel A */
            vuint16_t SUA:1;    /* valid startup frame on channel A */
            vuint16_t SEA:1;    /* syntax error on channel A */
            vuint16_t CEA:1;    /* content error on channel A */
            vuint16_t BVA:1;    /* boundary violation on channel A */
            vuint16_t TCA:1;    /* tx conflict on channel A */
        } B;
    } SSR_t;
    typedef union uMTSCFR {
        vuint16_t R;
        struct {
            vuint16_t MTE:1;    /* media access test symbol transmission enable */
              vuint16_t:1;
            vuint16_t CYCCNTMSK:6;      /* cycle counter mask */
              vuint16_t:2;
            vuint16_t CYCCNTVAL:6;      /* cycle counter value */
        } B;
    } MTSCFR_t;

    typedef union uRSBIR {
        vuint16_t R;
        struct {
            vuint16_t WMD:1;    /* write mode */
              vuint16_t:1;
            vuint16_t SEL:2;    /* selector */
              vuint16_t:5;
            vuint16_t RSBIDX:7; /* receive shadow buffer index */
        } B;
    } RSBIR_t;

    typedef union uRFDSR {
        vuint16_t R;
        struct {
            vuint16_t FIFODEPTH:8;      /* fifo depth */
              vuint16_t:1;
            vuint16_t ENTRYSIZE:7;      /* entry size */
        } B;
    } RFDSR_t;

    typedef union uRFRFCFR {
        vuint16_t R;
        struct {
            vuint16_t WMD:1;    /* write mode */
            vuint16_t IBD:1;    /* interval boundary */
            vuint16_t SEL:2;    /* filter number */
              vuint16_t:1;
            vuint16_t SID:11;   /* slot ID */
        } B;
    } RFRFCFR_t;

    typedef union uRFRFCTR {
        vuint16_t R;
        struct {
            vuint16_t:4;
            vuint16_t F3MD:1;   /* filter mode */
            vuint16_t F2MD:1;   /* filter mode */
            vuint16_t F1MD:1;   /* filter mode */
            vuint16_t F0MD:1;   /* filter mode */
              vuint16_t:4;
            vuint16_t F3EN:1;   /* filter enable */
            vuint16_t F2EN:1;   /* filter enable */
            vuint16_t F1EN:1;   /* filter enable */
            vuint16_t F0EN:1;   /* filter enable */
        } B;
    } RFRFCTR_t;
    typedef union uPCR0 {
        vuint16_t R;
        struct {
            vuint16_t ACTION_POINT_OFFSET:6;
            vuint16_t STATIC_SLOT_LENGTH:10;
        } B;
    } PCR0_t;

    typedef union uPCR1 {
        vuint16_t R;
        struct {
            vuint16_t:2;
            vuint16_t MACRO_AFTER_FIRST_STATIC_SLOT:14;
        } B;
    } PCR1_t;

    typedef union uPCR2 {
        vuint16_t R;
        struct {
            vuint16_t MINISLOT_AFTER_ACTION_POINT:6;
            vuint16_t NUMBER_OF_STATIC_SLOTS:10;
        } B;
    } PCR2_t;

    typedef union uPCR3 {
        vuint16_t R;
        struct {
            vuint16_t WAKEUP_SYMBOL_RX_LOW:6;
            vuint16_t MINISLOT_ACTION_POINT_OFFSET:5;
            vuint16_t COLDSTART_ATTEMPTS:5;
        } B;
    } PCR3_t;

    typedef union uPCR4 {
        vuint16_t R;
        struct {
            vuint16_t CAS_RX_LOW_MAX:7;
            vuint16_t WAKEUP_SYMBOL_RX_WINDOW:9;
        } B;
    } PCR4_t;

    typedef union uPCR5 {
        vuint16_t R;
        struct {
            vuint16_t TSS_TRANSMITTER:4;
            vuint16_t WAKEUP_SYMBOL_TX_LOW:6;
            vuint16_t WAKEUP_SYMBOL_RX_IDLE:6;
        } B;
    } PCR5_t;

    typedef union uPCR6 {
        vuint16_t R;
        struct {
            vuint16_t:1;
            vuint16_t SYMBOL_WINDOW_AFTER_ACTION_POINT:8;
            vuint16_t MACRO_INITIAL_OFFSET_A:7;
        } B;
    } PCR6_t;

    typedef union uPCR7 {
        vuint16_t R;
        struct {
            vuint16_t DECODING_CORRECTION_B:9;
            vuint16_t MICRO_PER_MACRO_NOM_HALF:7;
        } B;
    } PCR7_t;

    typedef union uPCR8 {
        vuint16_t R;
        struct {
            vuint16_t MAX_WITHOUT_CLOCK_CORRECTION_FATAL:4;
            vuint16_t MAX_WITHOUT_CLOCK_CORRECTION_PASSIVE:4;
            vuint16_t WAKEUP_SYMBOL_TX_IDLE:8;
        } B;
    } PCR8_t;

    typedef union uPCR9 {
        vuint16_t R;
        struct {
            vuint16_t MINISLOT_EXISTS:1;
            vuint16_t SYMBOL_WINDOW_EXISTS:1;
            vuint16_t OFFSET_CORRECTION_OUT:14;
        } B;
    } PCR9_t;

    typedef union uPCR10 {
        vuint16_t R;
        struct {
            vuint16_t SINGLE_SLOT_ENABLED:1;
            vuint16_t WAKEUP_CHANNEL:1;
            vuint16_t MACRO_PER_CYCLE:14;
        } B;
    } PCR10_t;

    typedef union uPCR11 {
        vuint16_t R;
        struct {
            vuint16_t KEY_SLOT_USED_FOR_STARTUP:1;
            vuint16_t KEY_SLOT_USED_FOR_SYNC:1;
            vuint16_t OFFSET_CORRECTION_START:14;
        } B;
    } PCR11_t;

    typedef union uPCR12 {
        vuint16_t R;
        struct {
            vuint16_t ALLOW_PASSIVE_TO_ACTIVE:5;
            vuint16_t KEY_SLOT_HEADER_CRC:11;
        } B;
    } PCR12_t;

    typedef union uPCR13 {
        vuint16_t R;
        struct {
            vuint16_t FIRST_MINISLOT_ACTION_POINT_OFFSET:6;
            vuint16_t STATIC_SLOT_AFTER_ACTION_POINT:10;
        } B;
    } PCR13_t;

    typedef union uPCR14 {
        vuint16_t R;
        struct {
            vuint16_t RATE_CORRECTION_OUT:11;
            vuint16_t LISTEN_TIMEOUT_H:5;
        } B;
    } PCR14_t;

    typedef union uPCR15 {
        vuint16_t R;
        struct {
            vuint16_t LISTEN_TIMEOUT_L:16;
        } B;
    } PCR15_t;

    typedef union uPCR16 {
        vuint16_t R;
        struct {
            vuint16_t MACRO_INITIAL_OFFSET_B:7;
            vuint16_t NOISE_LISTEN_TIMEOUT_H:9;
        } B;
    } PCR16_t;

    typedef union uPCR17 {
        vuint16_t R;
        struct {
            vuint16_t NOISE_LISTEN_TIMEOUT_L:16;
        } B;
    } PCR17_t;

    typedef union uPCR18 {
        vuint16_t R;
        struct {
            vuint16_t WAKEUP_PATTERN:6;
            vuint16_t KEY_SLOT_ID:10;
        } B;
    } PCR18_t;

    typedef union uPCR19 {
        vuint16_t R;
        struct {
            vuint16_t DECODING_CORRECTION_A:9;
            vuint16_t PAYLOAD_LENGTH_STATIC:7;
        } B;
    } PCR19_t;

    typedef union uPCR20 {
        vuint16_t R;
        struct {
            vuint16_t MICRO_INITIAL_OFFSET_B:8;
            vuint16_t MICRO_INITIAL_OFFSET_A:8;
        } B;
    } PCR20_t;

    typedef union uPCR21 {
        vuint16_t R;
        struct {
            vuint16_t EXTERN_RATE_CORRECTION:3;
            vuint16_t LATEST_TX:13;
        } B;
    } PCR21_t;

    typedef union uPCR22 {
        vuint16_t R;
        struct {
            vuint16_t:1;
            vuint16_t COMP_ACCEPTED_STARTUP_RANGE_A:11;
            vuint16_t MICRO_PER_CYCLE_H:4;
        } B;
    } PCR22_t;

    typedef union uPCR23 {
        vuint16_t R;
        struct {
            vuint16_t micro_per_cycle_l:16;
        } B;
    } PCR23_t;

    typedef union uPCR24 {
        vuint16_t R;
        struct {
            vuint16_t CLUSTER_DRIFT_DAMPING:5;
            vuint16_t MAX_PAYLOAD_LENGTH_DYNAMIC:7;
            vuint16_t MICRO_PER_CYCLE_MIN_H:4;
        } B;
    } PCR24_t;

    typedef union uPCR25 {
        vuint16_t R;
        struct {
            vuint16_t MICRO_PER_CYCLE_MIN_L:16;
        } B;
    } PCR25_t;

    typedef union uPCR26 {
        vuint16_t R;
        struct {
            vuint16_t ALLOW_HALT_DUE_TO_CLOCK:1;
            vuint16_t COMP_ACCEPTED_STARTUP_RANGE_B:11;
            vuint16_t MICRO_PER_CYCLE_MAX_H:4;
        } B;
    } PCR26_t;

    typedef union uPCR27 {
        vuint16_t R;
        struct {
            vuint16_t MICRO_PER_CYCLE_MAX_L:16;
        } B;
    } PCR27_t;

    typedef union uPCR28 {
        vuint16_t R;
        struct {
            vuint16_t DYNAMIC_SLOT_IDLE_PHASE:2;
            vuint16_t MACRO_AFTER_OFFSET_CORRECTION:14;
        } B;
    } PCR28_t;

    typedef union uPCR29 {
        vuint16_t R;
        struct {
            vuint16_t EXTERN_OFFSET_CORRECTION:3;
            vuint16_t MINISLOTS_MAX:13;
        } B;
    } PCR29_t;

    typedef union uPCR30 {
        vuint16_t R;
        struct {
            vuint16_t:12;
            vuint16_t SYNC_NODE_MAX:4;
        } B;
    } PCR30_t;

    typedef struct uMSG_BUFF_CCS {
        union {
            vuint16_t R;
            struct {
                vuint16_t:1;
                vuint16_t MCM:1;        /* message buffer commit mode */
                vuint16_t MBT:1;        /* message buffer type */
                vuint16_t MTD:1;        /* message buffer direction */
                vuint16_t CMT:1;        /* commit for transmission */
                vuint16_t EDT:1;        /* enable / disable trigger */
                vuint16_t LCKT:1;       /* lock request trigger */
                vuint16_t MBIE:1;       /* message buffer interrupt enable */
                  vuint16_t:3;
                vuint16_t DUP:1;        /* data updated  */
                vuint16_t DVAL:1;       /* data valid */
                vuint16_t EDS:1;        /* lock status */
                vuint16_t LCKS:1;       /* enable / disable status */
                vuint16_t MBIF:1;       /* message buffer interrupt flag */
            } B;
        } MBCCSR;
        union {
            vuint16_t R;
            struct {
                vuint16_t MTM:1;        /* message buffer transmission mode */
                vuint16_t CHNLA:1;      /* channel assignement */
                vuint16_t CHNLB:1;      /* channel assignement */
                vuint16_t CCFE:1;       /* cycle counter filter enable */
                vuint16_t CCFMSK:6;     /* cycle counter filter mask */
                vuint16_t CCFVAL:6;     /* cycle counter filter value */
            } B;
        } MBCCFR;
        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t FID:11;       /* frame ID */
            } B;
        } MBFIDR;

        union {
            vuint16_t R;
            struct {
                vuint16_t:9;
                vuint16_t MBIDX:7;      /* message buffer index */
            } B;
        } MBIDXR;
    } MSG_BUFF_CCS_t;
    typedef union uSYSBADHR {
        vuint16_t R;
    } SYSBADHR_t;
    typedef union uSYSBADLR {
        vuint16_t R;
    } SYSBADLR_t;
    typedef union uPADR {
        vuint16_t R;
    } PADR_t;
    typedef union uPDAR {
        vuint16_t R;
    } PDAR_t;
    typedef union uCASERCR {
        vuint16_t R;
    } CASERCR_t;
    typedef union uCBSERCR {
        vuint16_t R;
    } CBSERCR_t;
    typedef union uCYCTR {
        vuint16_t R;
    } CYCTR_t;
    typedef union uMTCTR {
        vuint16_t R;
    } MTCTR_t;
    typedef union uSLTCTAR {
        vuint16_t R;
    } SLTCTAR_t;
    typedef union uSLTCTBR {
        vuint16_t R;
    } SLTCTBR_t;
    typedef union uRTCORVR {
        vuint16_t R;
    } RTCORVR_t;
    typedef union uOFCORVR {
        vuint16_t R;
    } OFCORVR_t;
    typedef union uSFTOR {
        vuint16_t R;
    } SFTOR_t;
    typedef union uSFIDAFVR {
        vuint16_t R;
    } SFIDAFVR_t;
    typedef union uSFIDAFMR {
        vuint16_t R;
    } SFIDAFMR_t;
    typedef union uNMVR {
        vuint16_t R;
    } NMVR_t;
    typedef union uNMVLR {
        vuint16_t R;
    } NMVLR_t;
    typedef union uT1MTOR {
        vuint16_t R;
    } T1MTOR_t;
    typedef union uTI2CR0 {
        vuint16_t R;
    } TI2CR0_t;
    typedef union uTI2CR1 {
        vuint16_t R;
    } TI2CR1_t;
    typedef union uSSCR {
        vuint16_t R;
    } SSCR_t;
    typedef union uRFSR {
        vuint16_t R;
    } RFSR_t;
    typedef union uRFSIR {
        vuint16_t R;
    } RFSIR_t;
    typedef union uRFARIR {
        vuint16_t R;
    } RFARIR_t;
    typedef union uRFBRIR {
        vuint16_t R;
    } RFBRIR_t;
    typedef union uRFMIDAFVR {
        vuint16_t R;
    } RFMIDAFVR_t;
    typedef union uRFMIAFMR {
        vuint16_t R;
    } RFMIAFMR_t;
    typedef union uRFFIDRFVR {
        vuint16_t R;
    } RFFIDRFVR_t;
    typedef union uRFFIDRFMR {
        vuint16_t R;
    } RFFIDRFMR_t;
    typedef union uLDTXSLAR {
        vuint16_t R;
    } LDTXSLAR_t;
    typedef union uLDTXSLBR {
        vuint16_t R;
    } LDTXSLBR_t;

    typedef struct FR_tag {
        volatile MVR_t MVR;     /*module version register *//*0  */
        volatile MCR_t MCR;     /*module configuration register *//*2  */
        volatile SYSBADHR_t SYSBADHR;   /*system memory base address high register *//*4        */
        volatile SYSBADLR_t SYSBADLR;   /*system memory base address low register *//*6         */
        volatile STBSCR_t STBSCR;       /*strobe signal control register *//*8      */
        volatile STBPCR_t STBPCR;       /*strobe port control register *//*A        */
        volatile MBDSR_t MBDSR; /*message buffer data size register *//*C  */
        volatile MBSSUTR_t MBSSUTR;     /*message buffer segment size and utilization register *//*E  */
        volatile PADR_t PADR;   /*PE address register *//*10 */
        volatile PDAR_t PDAR;   /*PE data register *//*12 */
        volatile POCR_t POCR;   /*Protocol operation control register *//*14 */
        volatile GIFER_t GIFER; /*global interrupt flag and enable register *//*16 */
        volatile PIFR0_t PIFR0; /*protocol interrupt flag register 0 *//*18 */
        volatile PIFR1_t PIFR1; /*protocol interrupt flag register 1 *//*1A */
        volatile PIER0_t PIER0; /*protocol interrupt enable register 0 *//*1C */
        volatile PIER1_t PIER1; /*protocol interrupt enable register 1 *//*1E */
        volatile CHIERFR_t CHIERFR;     /*CHI error flag register *//*20 */
        volatile MBIVEC_t MBIVEC;       /*message buffer interrupt vector register *//*22 */
        volatile CASERCR_t CASERCR;     /*channel A status error counter register *//*24 */
        volatile CBSERCR_t CBSERCR;     /*channel B status error counter register *//*26 */
        volatile PSR0_t PSR0;   /*protocol status register 0 *//*28 */
        volatile PSR1_t PSR1;   /*protocol status register 1 *//*2A */
        volatile PSR2_t PSR2;   /*protocol status register 2 *//*2C */
        volatile PSR3_t PSR3;   /*protocol status register 3 *//*2E */
        volatile MTCTR_t MTCTR; /*macrotick counter register *//*30 */
        volatile CYCTR_t CYCTR; /*cycle counter register *//*32 */
        volatile SLTCTAR_t SLTCTAR;     /*slot counter channel A register *//*34 */
        volatile SLTCTBR_t SLTCTBR;     /*slot counter channel B register *//*36 */
        volatile RTCORVR_t RTCORVR;     /*rate correction value register *//*38 */
        volatile OFCORVR_t OFCORVR;     /*offset correction value register *//*3A */
        volatile CIFRR_t CIFRR; /*combined interrupt flag register *//*3C */
        vuint16_t reserved3[1]; /*3E */
        volatile SFCNTR_t SFCNTR;       /*sync frame counter register *//*40 */
        volatile SFTOR_t SFTOR; /*sync frame table offset register *//*42 */
        volatile SFTCCSR_t SFTCCSR;     /*sync frame table configuration, control, status register *//*44 */
        volatile SFIDRFR_t SFIDRFR;     /*sync frame ID rejection filter register *//*46 */
        volatile SFIDAFVR_t SFIDAFVR;   /*sync frame ID acceptance filter value regiater *//*48 */
        volatile SFIDAFMR_t SFIDAFMR;   /*sync frame ID acceptance filter mask register *//*4A */
        volatile NMVR_t NMVR[6];        /*network management vector registers (12 bytes) *//*4C */
        volatile NMVLR_t NMVLR; /*network management vector length register *//*58 */
        volatile TICCR_t TICCR; /*timer configuration and control register *//*5A */
        volatile TI1CYSR_t TI1CYSR;     /*timer 1 cycle set register *//*5C */
        volatile T1MTOR_t T1MTOR;       /*timer 1 macrotick offset register *//*5E */
        volatile TI2CR0_t TI2CR0;       /*timer 2 configuration register 0 *//*60 */
        volatile TI2CR1_t TI2CR1;       /*timer 2 configuration register 1 *//*62 */
        volatile SSSR_t SSSR;   /*slot status selection register *//*64 */
        volatile SSCCR_t SSCCR; /*slot status counter condition register *//*66 */
        volatile SSR_t SSR[8];  /*slot status registers 0-7 *//*68 */
        volatile SSCR_t SSCR[4];        /*slot status counter registers 0-3 *//*78 */
        volatile MTSCFR_t MTSACFR;      /*mts a config register *//*80 */
        volatile MTSCFR_t MTSBCFR;      /*mts b config register *//*82 */
        volatile RSBIR_t RSBIR; /*receive shadow buffer index register *//*84 */
        volatile RFSR_t RFSR;   /*receive fifo selection register *//*86 */
        volatile RFSIR_t RFSIR; /*receive fifo start index register *//*88 */
        volatile RFDSR_t RFDSR; /*receive fifo depth and size register *//*8A */
        volatile RFARIR_t RFARIR;       /*receive fifo a read index register *//*8C */
        volatile RFBRIR_t RFBRIR;       /*receive fifo b read index register *//*8E */
        volatile RFMIDAFVR_t RFMIDAFVR; /*receive fifo message ID acceptance filter value register *//*90 */
        volatile RFMIAFMR_t RFMIAFMR;   /*receive fifo message ID acceptance filter mask register *//*92 */
        volatile RFFIDRFVR_t RFFIDRFVR; /*receive fifo frame ID rejection filter value register *//*94 */
        volatile RFFIDRFMR_t RFFIDRFMR; /*receive fifo frame ID rejection filter mask register *//*96 */
        volatile RFRFCFR_t RFRFCFR;     /*receive fifo range filter configuration register *//*98 */
        volatile RFRFCTR_t RFRFCTR;     /*receive fifo range filter control register *//*9A */
        volatile LDTXSLAR_t LDTXSLAR;   /*last dynamic transmit slot channel A register *//*9C */
        volatile LDTXSLBR_t LDTXSLBR;   /*last dynamic transmit slot channel B register *//*9E */
        volatile PCR0_t PCR0;   /*protocol configuration register 0 *//*A0 */
        volatile PCR1_t PCR1;   /*protocol configuration register 1 *//*A2 */
        volatile PCR2_t PCR2;   /*protocol configuration register 2 *//*A4 */
        volatile PCR3_t PCR3;   /*protocol configuration register 3 *//*A6 */
        volatile PCR4_t PCR4;   /*protocol configuration register 4 *//*A8 */
        volatile PCR5_t PCR5;   /*protocol configuration register 5 *//*AA */
        volatile PCR6_t PCR6;   /*protocol configuration register 6 *//*AC */
        volatile PCR7_t PCR7;   /*protocol configuration register 7 *//*AE */
        volatile PCR8_t PCR8;   /*protocol configuration register 8 *//*B0 */
        volatile PCR9_t PCR9;   /*protocol configuration register 9 *//*B2 */
        volatile PCR10_t PCR10; /*protocol configuration register 10 *//*B4 */
        volatile PCR11_t PCR11; /*protocol configuration register 11 *//*B6 */
        volatile PCR12_t PCR12; /*protocol configuration register 12 *//*B8 */
        volatile PCR13_t PCR13; /*protocol configuration register 13 *//*BA */
        volatile PCR14_t PCR14; /*protocol configuration register 14 *//*BC */
        volatile PCR15_t PCR15; /*protocol configuration register 15 *//*BE */
        volatile PCR16_t PCR16; /*protocol configuration register 16 *//*C0 */
        volatile PCR17_t PCR17; /*protocol configuration register 17 *//*C2 */
        volatile PCR18_t PCR18; /*protocol configuration register 18 *//*C4 */
        volatile PCR19_t PCR19; /*protocol configuration register 19 *//*C6 */
        volatile PCR20_t PCR20; /*protocol configuration register 20 *//*C8 */
        volatile PCR21_t PCR21; /*protocol configuration register 21 *//*CA */
        volatile PCR22_t PCR22; /*protocol configuration register 22 *//*CC */
        volatile PCR23_t PCR23; /*protocol configuration register 23 *//*CE */
        volatile PCR24_t PCR24; /*protocol configuration register 24 *//*D0 */
        volatile PCR25_t PCR25; /*protocol configuration register 25 *//*D2 */
        volatile PCR26_t PCR26; /*protocol configuration register 26 *//*D4 */
        volatile PCR27_t PCR27; /*protocol configuration register 27 *//*D6 */
        volatile PCR28_t PCR28; /*protocol configuration register 28 *//*D8 */
        volatile PCR29_t PCR29; /*protocol configuration register 29 *//*DA */
        volatile PCR30_t PCR30; /*protocol configuration register 30 *//*DC */
        vuint16_t reserved2[17];
        volatile MSG_BUFF_CCS_t MBCCS[128];     /* message buffer configuration, control & status registers 0-31 *//*100 */
    } FR_tag_t;

    typedef union uF_HEADER     /* frame header */
    {
        struct {
            vuint16_t:5;
            vuint16_t HDCRC:11; /* Header CRC */
              vuint16_t:2;
            vuint16_t CYCCNT:6; /* Cycle Count */
              vuint16_t:1;
            vuint16_t PLDLEN:7; /* Payload Length */
              vuint16_t:1;
            vuint16_t PPI:1;    /* Payload Preamble Indicator */
            vuint16_t NUF:1;    /* Null Frame Indicator */
            vuint16_t SYF:1;    /* Sync Frame Indicator */
            vuint16_t SUF:1;    /* Startup Frame Indicator */
            vuint16_t FID:11;   /* Frame ID */
        } B;
        vuint16_t WORDS[3];
    } F_HEADER_t;
    typedef union uS_STSTUS     /* slot status */
    {
        struct {
            vuint16_t VFB:1;    /* Valid Frame on channel B */
            vuint16_t SYB:1;    /* Sync Frame Indicator channel B */
            vuint16_t NFB:1;    /* Null Frame Indicator channel B */
            vuint16_t SUB:1;    /* Startup Frame Indicator channel B */
            vuint16_t SEB:1;    /* Syntax Error on channel B */
            vuint16_t CEB:1;    /* Content Error on channel B */
            vuint16_t BVB:1;    /* Boundary Violation on channel B */
            vuint16_t CH:1;     /* Channel */
            vuint16_t VFA:1;    /* Valid Frame on channel A */
            vuint16_t SYA:1;    /* Sync Frame Indicator channel A */
            vuint16_t NFA:1;    /* Null Frame Indicator channel A */
            vuint16_t SUA:1;    /* Startup Frame Indicator channel A */
            vuint16_t SEA:1;    /* Syntax Error on channel A */
            vuint16_t CEA:1;    /* Content Error on channel A */
            vuint16_t BVA:1;    /* Boundary Violation on channel A */
              vuint16_t:1;
        } RX;
        struct {
            vuint16_t VFB:1;    /* Valid Frame on channel B */
            vuint16_t SYB:1;    /* Sync Frame Indicator channel B */
            vuint16_t NFB:1;    /* Null Frame Indicator channel B */
            vuint16_t SUB:1;    /* Startup Frame Indicator channel B */
            vuint16_t SEB:1;    /* Syntax Error on channel B */
            vuint16_t CEB:1;    /* Content Error on channel B */
            vuint16_t BVB:1;    /* Boundary Violation on channel B */
            vuint16_t TCB:1;    /* Tx Conflict on channel B */
            vuint16_t VFA:1;    /* Valid Frame on channel A */
            vuint16_t SYA:1;    /* Sync Frame Indicator channel A */
            vuint16_t NFA:1;    /* Null Frame Indicator channel A */
            vuint16_t SUA:1;    /* Startup Frame Indicator channel A */
            vuint16_t SEA:1;    /* Syntax Error on channel A */
            vuint16_t CEA:1;    /* Content Error on channel A */
            vuint16_t BVA:1;    /* Boundary Violation on channel A */
            vuint16_t TCA:1;    /* Tx Conflict on channel A */
        } TX;
        vuint16_t R;
    } S_STATUS_t;

    typedef struct uMB_HEADER   /* message buffer header */
    {
        F_HEADER_t FRAME_HEADER;
        vuint16_t DATA_OFFSET;
        S_STATUS_t SLOT_STATUS;
    } MB_HEADER_t;
/****************************************************************************/
/*                          MODULE : LCD                                   */
/****************************************************************************/
    struct LCD_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t LCDEN:1;
                vuint32_t LCDRST:1;
                vuint32_t LCDRCS:1;
                vuint32_t DUTY:3;
                vuint32_t BIAS:1;
                vuint32_t VLCDS:1;
                vuint32_t PWR:2;
                vuint32_t BSTEN:1;
                vuint32_t BSTSEL:1;
                vuint32_t BSTAO:1;
                  vuint32_t:1;
                vuint32_t LCDINT:1;
                vuint32_t EOFF:1;
                vuint32_t NOF:8;
                  vuint32_t:2;
                vuint32_t LCDBPA:1;
                  vuint32_t:2;
                vuint32_t LCDBPS:3;
            } B;
        } CR;                   /* LCD Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t LCLK:4;
                  vuint32_t:24;
            } B;
        } PCR;                  /* LCD Prescaler Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t CCEN:1;
                  vuint32_t:4;
                vuint32_t LCC:11;
                  vuint32_t:16;
            } B;
        } CCR;                  /* LCD Contrast Control Register */

        int32_t LCD_reserved1;  /* (0x10 - 0x0C)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t FP31EN:1;
                vuint32_t FP30EN:1;
                vuint32_t FP29EN:1;
                vuint32_t FP28EN:1;
                vuint32_t FP27EN:1;
                vuint32_t FP26EN:1;
                vuint32_t FP25EN:1;
                vuint32_t FP24EN:1;
                vuint32_t FP23EN:1;
                vuint32_t FP22EN:1;
                vuint32_t FP21EN:1;
                vuint32_t FP20EN:1;
                vuint32_t FP19EN:1;
                vuint32_t FP18EN:1;
                vuint32_t FP17EN:1;
                vuint32_t FP16EN:1;
                vuint32_t FP15EN:1;
                vuint32_t FP14EN:1;
                vuint32_t FP13EN:1;
                vuint32_t FP12EN:1;
                vuint32_t FP11EN:1;
                vuint32_t FP10EN:1;
                vuint32_t FP9EN:1;
                vuint32_t FP8EN:1;
                vuint32_t FP7EN:1;
                vuint32_t FP6EN:1;
                vuint32_t FP5EN:1;
                vuint32_t FP4EN:1;
                vuint32_t FP3EN:1;
                vuint32_t FP2EN:1;
                vuint32_t FP1EN:1;
                vuint32_t FP0EN:1;
            } B;
        } FPENR0;               /* LCD Frontplane Enable Register 0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t FP63EN:1;
                vuint32_t FP62EN:1;
                vuint32_t FP61EN:1;
                vuint32_t FP60EN:1;
                vuint32_t FP59EN:1;
                vuint32_t FP58EN:1;
                vuint32_t FP57EN:1;
                vuint32_t FP56EN:1;
                vuint32_t FP55EN:1;
                vuint32_t FP54EN:1;
                vuint32_t FP53EN:1;
                vuint32_t FP52EN:1;
                vuint32_t FP51EN:1;
                vuint32_t FP50EN:1;
                vuint32_t FP49EN:1;
                vuint32_t FP48EN:1;
                vuint32_t FP47EN:1;
                vuint32_t FP46EN:1;
                vuint32_t FP45EN:1;
                vuint32_t FP44EN:1;
                vuint32_t FP43EN:1;
                vuint32_t FP42EN:1;
                vuint32_t FP41EN:1;
                vuint32_t FP40EN:1;
                vuint32_t FP39EN:1;
                vuint32_t FP38EN:1;
                vuint32_t FP37EN:1;
                vuint32_t FP36EN:1;
                vuint32_t FP35EN:1;
                vuint32_t FP34EN:1;
                vuint32_t FP33EN:1;
                vuint32_t FP32EN:1;
            } B;
        } FPENR1;               /* LCD Frontplane Enable Register 1 */

        int32_t LCD_reserved2[2];       /* (0x20 - 0x18)/4 = 0x02 */

        union {
            vuint32_t R;
        } RAM[16];              /* LCD RAM Register */

    };                          /* end of LCD_tag */
/****************************************************************************/
/*                     MODULE : External Bus Interface (EBI)                */
/****************************************************************************/
    struct EBI_CS_tag {
        union {                 /* Base Register Bank */
            vuint32_t R;
            struct {
                vuint32_t BA:17;
                  vuint32_t:3;
                vuint32_t PS:1;
                  vuint32_t:4;
                vuint32_t BL:1;
                vuint32_t WEBS:1;
                vuint32_t TBDIP:1;
                  vuint32_t:2;
                vuint32_t BI:1;
                vuint32_t V:1;
            } B;
        } BR;

        union {                 /* Option Register Bank */
            vuint32_t R;
            struct {
                vuint32_t AM:17;
                  vuint32_t:7;
                vuint32_t SCY:4;
                  vuint32_t:1;
                vuint32_t BSCY:2;
                  vuint32_t:1;
            } B;
        } OR;
    };                          /* end of EBI_CS_tag */

    struct EBI_tag {
        union {                 /* Module Configuration Register */
            vuint32_t R;
            struct {
                vuint32_t:5;
                vuint32_t SIZEN:1;
                vuint32_t SIZE:2;
                  vuint32_t:8;
                vuint32_t ACGE:1;
                vuint32_t EXTM:1;
                vuint32_t EARB:1;
                vuint32_t EARP:2;
                  vuint32_t:4;
                vuint32_t MDIS:1;
                  vuint32_t:4;
                vuint32_t AD_MUX:1;
                vuint32_t DBM:1;
            } B;
        } MCR;

        uint32_t EBI_reserved1;

        union {                 /* Transfer Error Status Register */
            vuint32_t R;
            struct {
                vuint32_t:30;
                vuint32_t TEAF:1;
                vuint32_t BMTF:1;
            } B;
        } TESR;

        union {                 /* Bus Monitor Control Register */
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t BMT:8;
                vuint32_t BME:1;
                  vuint32_t:7;
            } B;
        } BMCR;

        struct EBI_CS_tag CS[2];

    };                          /* end of EBI_tag */
/****************************************************************************/
/*                     MODULE : DFLASH                                       */
/****************************************************************************/
    struct DFLASH_tag {
        union {                 /* Module Configuration Register */
            vuint32_t R;
            struct {
                vuint32_t EDC:1;
                  vuint32_t:4;
                vuint32_t SIZE:3;
                  vuint32_t:1;
                vuint32_t LAS:3;
                  vuint32_t:3;
                vuint32_t MAS:1;
                vuint32_t EER:1;
                vuint32_t RWE:1;
                  vuint32_t:1;
                  vuint32_t:1;
                vuint32_t PEAS:1;
                vuint32_t DONE:1;
                vuint32_t PEG:1;
                  vuint32_t:4;
                vuint32_t PGM:1;
                vuint32_t PSUS:1;
                vuint32_t ERS:1;
                vuint32_t ESUS:1;
                vuint32_t EHV:1;
            } B;
        } MCR;

        union {                 /* LML Register */
            vuint32_t R;
            struct {
                vuint32_t LME:1;
                  vuint32_t:10;
                vuint32_t TSLK:1;
                  vuint32_t:2;
                vuint32_t MLK:2;
                vuint32_t LLK:16;
            } B;
        } LML;

        union {                 /* HBL Register */
            vuint32_t R;
            struct {
                vuint32_t HBE:1;
                  vuint32_t:23;
                vuint32_t HBLOCK:8;
            } B;
        } HBL;

        union {                 /* SLML Register */
            vuint32_t R;
            struct {
                vuint32_t SLE:1;
                  vuint32_t:10;
                vuint32_t STSLK:1;
                  vuint32_t:2;
                vuint32_t SMK:2;
                vuint32_t SLK:16;
            } B;
        } SLL;

        union {                 /* LMS Register */
            vuint32_t R;
            struct {
                vuint32_t:14;
                vuint32_t MSL:2;
                vuint32_t LSL:16;
            } B;
        } LMS;

        union {                 /* High Address Space Block Select Register */
            vuint32_t R;
            struct {
                vuint32_t:26;
                vuint32_t HSL:6;
            } B;
        } HBS;

        union {                 /* Address Register */
            vuint32_t R;
            struct {
                vuint32_t:9;
                vuint32_t ADD:20;
                  vuint32_t:3;
            } B;
        } ADR;

        int32_t Dflash_reserved0[8];    /* {0x003C-0x001C}/0x4 = 0x08 */

        union {                 /* User Test Register 0 */
            vuint32_t R;
            struct {
                vuint32_t UTE:1;
                  vuint32_t:7;
                vuint32_t DSI:8;
                  vuint32_t:10;
                vuint32_t MRE:1;
                vuint32_t MRV:1;
                vuint32_t EIE:1;
                vuint32_t AIS:1;
                vuint32_t AIE:1;
                vuint32_t AID:1;
            } B;
        } UT0;

        union {                 /* User Test Register 1 */
            vuint32_t R;
            struct {
                vuint32_t DAI:32;
            } B;
        } UT1;

        union {                 /* User Test Register 2 */
            vuint32_t R;
            struct {
                vuint32_t DAI:32;
            } B;
        } UT2;

        union {                 /* User Multiple Input Signature Register 0-4 */
            vuint32_t R;
            struct {
                vuint32_t MS:32;
            } B;
        } UMISR[5];

    };                          /* end of Dflash_tag */
/****************************************************************************/
/*                     MODULE : CFLASH                                       */
/****************************************************************************/
    struct CFLASH_tag {
        union {                 /* Module Configuration Register */
            vuint32_t R;
            struct {
                vuint32_t EDC:1;
                  vuint32_t:4;
                vuint32_t SIZE:3;
                  vuint32_t:1;
                vuint32_t LAS:3;
                  vuint32_t:3;
                vuint32_t MAS:1;
                vuint32_t EER:1;
                vuint32_t RWE:1;
                  vuint32_t:1;
                  vuint32_t:1;
                vuint32_t PEAS:1;
                vuint32_t DONE:1;
                vuint32_t PEG:1;
                  vuint32_t:4;
                vuint32_t PGM:1;
                vuint32_t PSUS:1;
                vuint32_t ERS:1;
                vuint32_t ESUS:1;
                vuint32_t EHV:1;
            } B;
        } MCR;

        union {                 /* LML Register */
            vuint32_t R;
            struct {
                vuint32_t LME:1;
                  vuint32_t:10;
                vuint32_t TSLK:1;
                  vuint32_t:2;
                vuint32_t MLK:2;
                vuint32_t LLK:16;
            } B;
        } LML;

        union {                 /* HBL Register */
            vuint32_t R;
            struct {
                vuint32_t HBE:1;
                  vuint32_t:23;
                vuint32_t HBLOCK:8;
            } B;
        } HBL;

        union {                 /* SLML Register */
            vuint32_t R;
            struct {
                vuint32_t SLE:1;
                  vuint32_t:10;
                vuint32_t STSLK:1;
                  vuint32_t:2;
                vuint32_t SMK:2;
                vuint32_t SLK:16;
            } B;
        } SLL;

        union {                 /* LMS Register */
            vuint32_t R;
            struct {
                vuint32_t:14;
                vuint32_t MSL:2;
                vuint32_t LSL:16;
            } B;
        } LMS;

        union {                 /* High Address Space Block Select Register */
            vuint32_t R;
            struct {
                vuint32_t:26;
                vuint32_t HSL:6;
            } B;
        } HBS;

        union {                 /* Address Register */
            vuint32_t R;
            struct {
                vuint32_t:9;
                vuint32_t ADD:20;
                  vuint32_t:3;
            } B;
        } ADR;

        union {                 /* CFLASH Configuration Register 0 */
            vuint32_t R;
            struct {
                vuint32_t BK0_APC:5;
                vuint32_t BK0_WWSC:5;
                vuint32_t BK0_RWSC:5;
                vuint32_t BK0_RWWC2:1;
                vuint32_t BK0_RWWC1:1;
                vuint32_t B0_P1_BCFG:2;
                vuint32_t B0_P1_DPFE:1;
                vuint32_t B0_P1_IPFE:1;
                vuint32_t B0_P1_PFLM:2;
                vuint32_t B0_P1_BFE:1;
                vuint32_t BK0_RWWC0:1;
                vuint32_t B0_P0_BCFG:2;
                vuint32_t B0_P0_DPFE:1;
                vuint32_t B0_P0_IPFE:1;
                vuint32_t B0_P0_PFLM:2;
                vuint32_t B0_P0_BFE:1;
            } B;
        } PFCR0;

        union {                 /* CFLASH Configuration Register 1 */
            vuint32_t R;
            struct {
                vuint32_t BK1_APC:5;
                vuint32_t BK1_WWSC:5;
                vuint32_t BK1_RWSC:5;
                vuint32_t BK1_RWWC2:1;
                vuint32_t BK1_RWWC1:1;
                  vuint32_t:6;
                vuint32_t B0_P1_BFE:1;
                vuint32_t BK1_RWWC0:1;
                  vuint32_t:6;
                vuint32_t B1_P0_BFE:1;
            } B;
        } PFCR1;

        union {                 /* cflash Access Protection Register */
            vuint32_t R;
            struct {
                vuint32_t:6;
                vuint32_t ARBM:2;
                vuint32_t M7PFD:1;
                vuint32_t M6PFD:1;
                vuint32_t M5PFD:1;
                vuint32_t M4PFD:1;
                vuint32_t M3PFD:1;
                vuint32_t M2PFD:1;
                vuint32_t M1PFD:1;
                vuint32_t M0PFD:1;
                vuint32_t M7AP:2;
                vuint32_t M6AP:2;
                vuint32_t M5AP:2;
                vuint32_t M4AP:2;
                vuint32_t M3AP:2;
                vuint32_t M2AP:2;
                vuint32_t M1AP:2;
                vuint32_t M0AP:2;
            } B;
        } FAPR;

        int32_t CFLASH_reserved0[5];    /* {0x003C-0x0028}/0x4 = 0x05 */

        union {                 /* User Test Register 0 */
            vuint32_t R;
            struct {
                vuint32_t UTE:1;
                  vuint32_t:7;
                vuint32_t DSI:8;
                  vuint32_t:10;
                vuint32_t MRE:1;
                vuint32_t MRV:1;
                vuint32_t EIE:1;
                vuint32_t AIS:1;
                vuint32_t AIE:1;
                vuint32_t AID:1;
            } B;
        } UT0;

        union {                 /* User Test Register 1 */
            vuint32_t R;
            struct {
                vuint32_t DAI:32;
            } B;
        } UT1;

        union {                 /* User Test Register 2 */
            vuint32_t R;
            struct {
                vuint32_t DAI:32;
            } B;
        } UT2;

        union {                 /* User Multiple Input Signature Register 0-4 */
            vuint32_t R;
            struct {
                vuint32_t MS:32;
            } B;
        } UMISR[5];

    };                          /* end of CFLASH_tag */
    
/****************************************************************************/
/*                          MODULE : CRC                                    */
/****************************************************************************/
    struct CRC_SUB_tag {
        union {
            vuint8_t  B[4];  /* Data buffer in Bytes (8 bits) */
            vuint16_t H[2];  /* Data buffer in Half-words (16 bits) */
            vuint32_t W;     /* Data buffer in words (32 bits) */
            struct {
                vuint32_t INV:1;
                vuint32_t SWAP:1;
                vuint32_t POLYG:1;
		vuint32_t:29;
		}BIT;
        } CRC_CFG;                   /* CRC Configuration Register */

	union {
            vuint8_t  B[4];  /* Data buffer in Bytes (8 bits) */
            vuint16_t H[2];  /* Data buffer in Half-words (16 bits) */
            vuint32_t W;     /* Data buffer in words (32 bits) */
        } CRC_INP;	     /* CRC Input Register */

	union {
            vuint8_t  B[4];  /* Data buffer in Bytes (8 bits) */
            vuint16_t H[2];  /* Data buffer in Half-words (16 bits) */
            vuint32_t W;     /* Data buffer in words (32 bits) */
        } CRC_CSTAT;         /*CRC Current Status Register */

	union {
            vuint8_t  B[4];  /* Data buffer in Bytes (8 bits) */
            vuint16_t H[2];  /* Data buffer in Half-words (16 bits) */
            vuint32_t W;     /* Data buffer in words (32 bits) */
        } CRC_OUTP;          /* CRC Output Register */

    };                          /* end of CRC_tag */

    struct CRC_tag {
      struct CRC_SUB_tag CNTX[2];
    };    
    
/****************************************************************** 
| defines and macros (scope: module-local) 
|-----------------------------------------------------------------*/
/* Define instances of modules */
#define ADC_0     (*(volatile struct ADC_tag *)       0xFFE00000UL)
#define ADC_1     (*(volatile struct ADC_tag *)       0xFFE04000UL)
#define CAN_0     (*(volatile struct FLEXCAN_tag *)   0xFFFC0000UL)
#define CAN_1     (*(volatile struct FLEXCAN_tag *)   0xFFFC4000UL)
#define CAN_2     (*(volatile struct FLEXCAN_tag *)   0xFFFC8000UL)
#define CAN_3     (*(volatile struct FLEXCAN_tag *)   0xFFFCC000UL)
#define CAN_4     (*(volatile struct FLEXCAN_tag *)   0xFFFD0000UL)
#define CAN_5     (*(volatile struct FLEXCAN_tag *)   0xFFFD4000UL)
#define CANSP     (*(volatile struct CANSP_tag *)     0xFFE70000UL)
#define CFLASH    (*(volatile struct CFLASH_tag *)    0xC3F88000UL)
#define CGM       (*(volatile struct CGM_tag *)       0xC3FE0000UL)
#define CTU_0     (*(volatile struct CTU_tag *)       0xFFE0C000UL)
#define CTU_1     (*(volatile struct CTU_tag *)       0xFFE10000UL)
#define CTUL      (*(volatile struct CTUL_tag *)      0xFFE64000UL)
#define DCU       (*(volatile struct DCU_tag *)       0xFFE7C000UL)
#define DFLASH    (*(volatile struct DFLASH_tag *)    0xC3F8C000UL)
#define DMAMUX    (*(volatile struct DMAMUX_tag *)    0xFFFDC000UL)
#define DSPI_0    (*(volatile struct DSPI_tag *)      0xFFF90000UL)
#define DSPI_1    (*(volatile struct DSPI_tag *)      0xFFF94000UL)
#define DSPI_2    (*(volatile struct DSPI_tag *)      0xFFF98000UL)
#define DSPI_3    (*(volatile struct DSPI_tag *)      0xFFF9C000UL)
#define EBI       (*(volatile struct EBI_tag *)       0xC3F84000UL)
#define EDMA      (*(volatile struct EDMA_tag *)      0xFFF44000UL)
#define EMIOS_0   (*(volatile struct EMIOS_tag *)     0xC3FA0000UL)
#define EMIOS_1   (*(volatile struct EMIOS_tag *)     0xC3FA4000UL)
#define ETIMER_0  (*(volatile struct ETIMER_tag *)    0xFFE18000UL)
#define ETIMER_1  (*(volatile struct ETIMER_tag *)    0xFFE1C000UL)
#define FCU       (*(volatile struct FCU_tag *)       0xFFE6C000UL)
#define FLEXPWM_0 (*(volatile struct FLEXPWM_tag *)   0xFFE24000UL)
#define FLEXPWM_1 (*(volatile struct FLEXPWM_tag *)   0xFFE28000UL)
#define FR        (*(volatile struct FR_tag *)        0xFFFE0000UL)
#define I2C_0     (*(volatile struct I2C_tag *)       0xFFE30000UL)
#define I2C_1     (*(volatile struct I2C_tag *)       0xFFE34000UL)
#define I2C_2     (*(volatile struct I2C_tag *)       0xFFE38000UL)
#define I2C_3     (*(volatile struct I2C_tag *)       0xFFE3C000UL)
#define INTC      (*(volatile struct INTC_tag *)      0xFFF48000UL)
#define LCD       (*(volatile struct LCD_tag *)       0xFFE74000UL)
#define LINFLEX_0 (*(volatile struct LINFLEX_tag *)   0xFFE40000UL)
#define LINFLEX_1 (*(volatile struct LINFLEX_tag *)   0xFFE44000UL)
#define LINFLEX_2 (*(volatile struct LINFLEX_tag *)   0xFFE48000UL)
#define LINFLEX_3 (*(volatile struct LINFLEX_tag *)   0xFFE4C000UL)
#define MCM       (*(volatile struct MCM_tag *)       0xFFF40000UL)
#define ME        (*(volatile struct ME_tag *)        0xC3FDC000UL)
#define MPU       (*(volatile struct MPU_tag *)       0xFFF10000UL)
#define PCU       (*(volatile struct PCU_tag *)       0xC3FE8000UL)
#define PIT       (*(volatile struct PIT_tag *)       0xC3FF0000UL)
#define RGM       (*(volatile struct RGM_tag *)       0xC3FE4000UL)
#define RTC       (*(volatile struct RTC_tag *)       0xC3FEC000UL)
#define SAFEPORT  (*(volatile struct FLEXCAN_tag *)   0xFFFE8000UL)
#define SIU       (*(volatile struct SIU_tag *)       0xC3F90000UL)
#define SMC       (*(volatile struct SMC_tag *)       0xFFE60000UL)
#define SSCM      (*(volatile struct SSCM_tag *)      0xC3FD8000UL)
#define SSD_0     (*(volatile struct SSD_tag *)       0xFFE61000UL)
#define SSD_1     (*(volatile struct SSD_tag *)       0xFFE61800UL)
#define SSD_2     (*(volatile struct SSD_tag *)       0xFFE62000UL)
#define SSD_3     (*(volatile struct SSD_tag *)       0xFFE62800UL)
#define SSD_4     (*(volatile struct SSD_tag *)       0xFFE63000UL)
#define SSD_5     (*(volatile struct SSD_tag *)       0xFFE63800UL)
#define STM       (*(volatile struct STM_tag *)       0xFFF3C000UL)
#define SWT       (*(volatile struct SWT_tag *)       0xFFF38000UL)
#define WKUP      (*(volatile struct WKUP_tag *)      0xC3F94000UL)
#define CRC       (*(volatile struct CRC_tag *)       0xFFE68000UL)

#ifdef __MWERKS__
#pragma pop
#endif

#ifdef  __cplusplus
}
#endif
#endif                          /* ifdef _JDP_H */
/* End of file */
