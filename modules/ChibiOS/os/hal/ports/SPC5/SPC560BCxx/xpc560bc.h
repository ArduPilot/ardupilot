/***************************************************************** 
 *
 * FILE        : MPC5604B_0M27V_0100.h
 * 
 * DESCRIPTION : This is the header file describing the register
 *               set for:
 *	             MPC5604B, mask set = 0M27V
 *               SPC560B4, mask set = FB50X20B
 * 
 * COPYRIGHT   :(c) 2009, Freescale & STMicroelectronics 
 * 
 * VERSION     : 01.02 
 * DATE        : 08 MAY 2009 
 * AUTHOR      : b04629 
 * HISTORY     : Original source taken from jdp_0100.h. 
 *               Updated to be compatable with 
 *               - MPC5604B Mask ID 0M27V 
 *               - MPC5604B Reference Manual Rev 3 Draft A
 *               - SPC560B4 Mask ID FB50X20B
 *               - SPC560B4 Reference Manual Rev 3 Draft A
 *
 ******************************************************************/  
    
/*>>>>NOTE! this file is auto-generated please do not edit it!<<<<*/ 
    
/***************************************************************** 
* Example instantiation and use:            
*                                           
*  <MODULE>.<REGISTER>.B.<BIT> = 1;         
*  <MODULE>.<REGISTER>.R       = 0x10000000;
*                                           
******************************************************************/ 
    
#ifndef _MPC5604B_H_
#define _MPC5604B_H_
    
#include "typedefs.h"
    
#ifdef  __cplusplus
extern "C" {
    
#endif  /* 
 */
    
#ifdef __MWERKS__
#pragma push
#pragma ANSI_strict off
#endif  /* 
 */
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
                  vuint32_t:4;
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
                  vuint32_t:4;
                vuint32_t PWDN:1;
            } B;
        } MCR;                  /* MAIN CONFIGURATION REGISTER */

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
                  vuint32_t:2;
                vuint32_t ADCSTATUS:3;
            } B;
        } MSR;                  /* MAIN STATUS REGISTER */

        int32_t ADC_reserved1[2];       /* (0x010 - 0x008)/4 = 0x02 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:27;
                vuint32_t EOCTU:1;
                vuint32_t JEOC:1;
                vuint32_t JECH:1;
                vuint32_t EOC:1;
                vuint32_t ECH:1;
            } B;
        } ISR;                  /* INTERRUPT STATUS REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t EOC_CH31:1;
                vuint32_t EOC_CH30:1;
                vuint32_t EOC_CH29:1;
                vuint32_t EOC_CH28:1;
                vuint32_t EOC_CH27:1;
                vuint32_t EOC_CH26:1;
                vuint32_t EOC_CH25:1;
                vuint32_t EOC_CH24:1;
                vuint32_t EOC_CH23:1;
                vuint32_t EOC_CH22:1;
                vuint32_t EOC_CH21:1;
                vuint32_t EOC_CH20:1;
                vuint32_t EOC_CH19:1;
                vuint32_t EOC_CH18:1;
                vuint32_t EOC_CH17:1;
                vuint32_t EOC_CH16:1;
                vuint32_t EOC_CH15:1;
                vuint32_t EOC_CH14:1;
                vuint32_t EOC_CH13:1;
                vuint32_t EOC_CH12:1;
                vuint32_t EOC_CH11:1;
                vuint32_t EOC_CH10:1;
                vuint32_t EOC_CH9:1;
                vuint32_t EOC_CH8:1;
                vuint32_t EOC_CH7:1;
                vuint32_t EOC_CH6:1;
                vuint32_t EOC_CH5:1;
                vuint32_t EOC_CH4:1;
                vuint32_t EOC_CH3:1;
                vuint32_t EOC_CH2:1;
                vuint32_t EOC_CH1:1;
                vuint32_t EOC_CH0:1;
            } B;
        } CEOCFR[3];            /* Channel Pending Register 0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:27;
                vuint32_t MSKEOCTU:1;
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
        } CIMR[3];              /* Channel Interrupt Mask Register 0 */

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
        } WTISR;                /* WATCHDOG INTERRUPT THRESHOLD REGISTER */

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
        } WTIMR;                /* WATCHDOG INTERRUPT MASK REGISTER */

        int32_t ADC_reserved2[6];       /* (0x050 - 0x038)/4 = 0x06 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t THREN:1;
                vuint32_t THRINV:1;
                  vuint32_t:7;
                vuint32_t THRCH:7;
            } B;
        } TRC[4];               /* ADC THRESHOLD REGISTER REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t THRH:12;
                  vuint32_t:4;
                vuint32_t THRL:12;
            } B;
        } THRHLR[4];            /* THRESHOLD REGISTER */

        int32_t ADC_reserved3[4];       /* (0x080 - 0x070)/4 = 0x04 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:25;
                vuint32_t PREVAL2:2;
                vuint32_t PREVAL1:2;
                vuint32_t PREVAL0:2;
                vuint32_t PRECONV:1;
            } B;
        } PSCR;                 /* PRESAMPLING CONTROL REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t PRES31:1;
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
        } PSR[3];               /* PRESAMPLING REGISTER */

        int32_t ADC_reserved4[1];       /* (0x094 - 0x090)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t INPLATCH:1;
                  vuint32_t:4;
                vuint32_t INPCMP:2;
                  vuint32_t:1;
                vuint32_t INPSAMP:8;
            } B;
        } CTR[3];               /* CONVERSION TIMING REGISTER */

        int32_t ADC_reserved5[1];       /* (0x0A4 - 0x0A0)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CH31:1;
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
        } NCMR[3];              /* NORMAL CONVERSION MASK REGISTER */

        int32_t ADC_reserved6[1];       /* (0x0B4 - 0x0B0)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CH31:1;
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
        } JCMR[3];              /* Injected CONVERSION MASK REGISTER */

        int32_t ADC_reserved7[1];       /* (0x0C4 - 0x0C0)/4 = 0x01 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t DSD:8;
            } B;
        } DSDR;                 /* DECODE SIGNALS DELAY REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t PDED:8;
            } B;
        } PDEDR;                /* POWER DOWN DELAY REGISTER */

        int32_t ADC_reserved8[13];      /* (0x100 - 0x0CC)/4 = 0x0D */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t VALID:1;
                vuint32_t OVERW:1;
                vuint32_t RESULT:2;
                  vuint32_t:6;
                vuint32_t CDATA:10;
            } B;
        } CDR[96];              /* Channel 0-95 Data REGISTER */

    };                          /* end of ADC_tag */
/****************************************************************************/
/*                          MODULE : CANSP                                   */
/****************************************************************************/
    struct CANSP_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RX_COMPLETE:1;
                vuint32_t BUSY:1;
                vuint32_t ACTIVE_CK:1;
                  vuint32_t:3;
                vuint32_t MODE:1;
                vuint32_t CAN_RX_SEL:3;
                vuint32_t BRP:5;
                vuint32_t CAN_SMPLR_EN:1;
            } B;
        } CR;                   /* CANSP Control Register */

        union {
            vuint32_t R;
        } SR[12];               /* CANSP Sample Register 0 to 11 */

    };                          /* end of CANSP_tag */
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
                vuint32_t:10;
                vuint32_t ADD:19;
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
/*                          MODULE : CGM                                   */
/****************************************************************************/
    struct CGM_tag {

        /* The CGM provides a unified register interface, enabling access to 

           all clock sources:

           Base Address | Clock Sources

           -----------------------------

           0xC3FE0000   | FXOSC_CTL

           ----------   | Reserved

           0xC3FE0040   | SXOSC_CTL

           0xC3FE0060   | FIRC_CTL

           0xC3FE0080   | SIRC_CTL

           0xC3FE00A0   | FMPLL_0

           ----------   | Reserved 

           0xC3FE0100   | CMU_0

         */
    /************************************/
        /* FXOSC_CTL @ CGM base address + 0x0000 */
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
                  vuint32_t:7;
            } B;
        } FXOSC_CTL;            /* Fast OSC Control Register */

    /************************************/
        /* SXOSC_CTL @ CGM base address + 0x0040 */
    /************************************/
        int32_t CGM_reserved0[15];      /* (0x040 - 0x004)/4 = 0x0F */

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
        } SXOSC_CTL;            /* Slow OSC Control Register */

    /************************************/
        /* FIRC_CTL @ CGM base address + 0x0060 */
    /************************************/
        int32_t CGM_reserved1[7];       /* (0x060 - 0x044)/4 = 0x07 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:10;
                vuint32_t RCTRIM:6;
                  vuint32_t:3;
                vuint32_t RCDIV:5;
                  vuint32_t:8;
            } B;
        } FIRC_CTL;             /* Fast IRC Control Register */

    /****************************************/
        /* SIRC_CTL @ CGM base address + 0x0080 */
    /****************************************/
        int32_t CGM_reserved2[7];       /* (0x080 - 0x064)/4 = 0x07 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:11;
                vuint32_t RCTRIM:5;
                  vuint32_t:3;
                vuint32_t RCDIV:5;
                  vuint32_t:3;
                vuint32_t S_SIRC:1;
                  vuint32_t:3;
                vuint32_t SIRCON_STDBY:1;
            } B;
        } SIRC_CTL;             /* Slow IRC Control Register */

    /*************************************/
        /* FMPLL @ CGM base address + 0x00A0 */
    /*************************************/
        int32_t CGM_reserved3[7];       /* (0x0A0 - 0x084)/4 = 0x07 */

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
        } FMPLL_CR;             /* FMPLL Control Register */

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
        } FMPLL_MR;             /* FMPLL Modulation Register */

    /************************************/
        /* CMU @ CGM base address + 0x0100  */
    /************************************/
        int32_t CGM_reserved5[22];      /* (0x100 - 0x0A8)/4 = 0x16 */

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
        } CMU_CSR;              /* Control Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t FD:20;
            } B;
        } CMU_FDR;              /* Frequency Display Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:20;
                vuint32_t HFREF_A:12;
            } B;
        } CMU_HFREFR_A;         /* High Frequency Reference Register PLL_A Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:20;
                vuint32_t LFREF_A:12;
            } B;
        } CMU_LFREFR_A;         /* Low Frequency Reference Register PLL_A Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t FLCI_A:1;
                vuint32_t FHHI_A:1;
                vuint32_t FLLI_A:1;
                vuint32_t OLRI:1;
            } B;
        } CMU_ISR;              /* Interrupt Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } CMU_IMR;              /* Interrupt Mask Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t MD:20;
            } B;
        } CMU_MDR;              /* Measurement Duration Register */

    /************************************/
        /* CGM General Registers @ CGM base address + 0x0370 */
    /************************************/
        int32_t CGM_reserved7[149];     /* (0x370 - 0x11C)/4 = 0x95 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t EN:1;
            } B;
        } OC_EN;                /* Output Clock Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:2;
                vuint32_t SELDIV:2;
                vuint32_t SELCTL:4;
                  vuint32_t:24;
            } B;
        } OCDS_SC;              /* Output Clock Division Select Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SELSTAT:4;
                  vuint32_t:24;
            } B;
        } SC_SS;                /* System Clock Select Status */

        union {
            vuint8_t R;
            struct {
                vuint8_t DE:1;
                  vuint8_t:3;
                vuint8_t DIV:4;
            } B;
        } SC_DC[3];             /* System Clock Divider Configuration 0->2 */

    };                          /* end of CGM_tag */
/****************************************************************************/
/*                          MODULE : CTU                                   */
/****************************************************************************/
    struct CTU_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t TRGIEN:1;
                vuint32_t TRGI:1;
                  vuint32_t:6;
            } B;
        } CSR;                  /* Control Status Register */

        int32_t CTU_reserved0[11];      /* (0x030 - 0x004)/4 = 0x0B */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t TM:1;
                  vuint32_t:7;
                vuint32_t CLR_FLAG:1;
                  vuint32_t:1;
                vuint32_t CHANNELVALUE:6;
            } B;
        } EVTCFGR[64];          /* Event Configuration Register */

    };                          /* end of CTU_tag */
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
                vuint32_t:10;
                vuint32_t ADD:19;
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
                  vuint32_t:2;
                vuint32_t PCSIS5:1;
                vuint32_t PCSIS4:1;
                vuint32_t PCSIS3:1;
                vuint32_t PCSIS2:1;
                vuint32_t PCSIS1:1;
                vuint32_t PCSIS0:1;
                vuint32_t DOZE:1;
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
                  vuint32_t:4;
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
        } TXFR[4];              /* Transmit FIFO Registers */

        vuint32_t DSPI_reserved_txf[12];

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RXDATA:16;
            } B;
        } RXFR[4];              /* Transmit FIFO Registers */

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
/*                          MODULE : ECSM                                   */
/****************************************************************************/
    struct ECSM_tag {

        union {
            vuint16_t R;
        } PCT;                  /* ECSM Processor Core Type Register */

        union {
            vuint16_t R;
        } REV;                  /* ECSM  Revision Register */

        int32_t ECSM_reserved1;

        union {
            vuint32_t R;
        } IMC;                  /* ECSM IPS Module Configuration Register */

        int8_t ECSM_reserved2[7];

        union {
            vuint8_t R;
            struct {
                vuint8_t ENBWCR:1;
                  vuint8_t:3;
                vuint8_t PRILVL:4;
            } B;
        } MWCR;                 /* ECSM Miscellaneous Wakeup Control Register */

        int32_t ECSM_reserved3[2];
        int8_t ECSM_reserved4[3];

        union {
            vuint8_t R;
            struct {
                vuint8_t FB0AI:1;
                vuint8_t FB0SI:1;
                vuint8_t FB1AI:1;
                vuint8_t FB1SI:1;
                  vuint8_t:4;
            } B;
        } MIR;                  /* ECSM Miscellaneous Interrupt Register */

        int32_t ECSM_reserved5;

        union {
            vuint32_t R;
        } MUDCR;                /* ECSM Miscellaneous User-Defined Control Register */

        int32_t ECSM_reserved6[6];      /* (0x040- 0x028)/4 = 0x06 */
        int8_t ECSM_reserved7[3];

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
        } ECR;                  /* ECSM ECC Configuration Register */

        int8_t ECSM_reserved8[3];

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
        } ESR;                  /* ECSM ECC Status Register */

        int16_t ECSM_reserved9;

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
        } EEGR;                 /* ECSM ECC Error Generation Register */

        int32_t ECSM_reserved10;

        union {
            vuint32_t R;
        } FEAR;                 /* ECSM Flash ECC Address Register */

        int16_t ECSM_reserved11;

        union {
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t FEMR:4;
            } B;
        } FEMR;                 /* ECSM Flash ECC Master Number Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t WRITE:1;
                vuint8_t SIZE:3;
                vuint8_t PROTECTION:4;
            } B;
        } FEAT;                 /* ECSM Flash ECC Attributes Register */

        int32_t ECSM_reserved12;

        union {
            vuint32_t R;
        } FEDR;                 /* ECSM Flash ECC Data Register */

        union {
            vuint32_t R;
        } REAR;                 /* ECSM RAM ECC Address Register */

        int8_t ECSM_reserved13;

        union {
            vuint8_t R;
        } RESR;                 /* ECSM RAM ECC Address Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t REMR:4;
            } B;
        } REMR;                 /* ECSM RAM ECC Master Number Register */

        union {
            vuint8_t R;
            struct {
                vuint8_t WRITE:1;
                vuint8_t SIZE:3;
                vuint8_t PROTECTION:4;
            } B;
        } REAT;                 /* ECSM RAM ECC Attributes Register */

        int32_t ECSM_reserved14;

        union {
            vuint32_t R;
        } REDR;                 /* ECSM RAM ECC Data Register */

    };                          /* end of ECSM_tag */
/****************************************************************************/
/*                          MODULE : EMIOS                                  */
/****************************************************************************/
    struct EMIOS_CHANNEL_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t CADR:16;
            } B;
        } CADR;                 /* Channel A Data Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t CBDR:16;
            } B;
        } CBDR;                 /* Channel B Data Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t CCNTR:16;
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
            vuint8_t B[8];      /* Data buffer in Bytes (8 bits) */
            vuint16_t H[4];     /* Data buffer in Half-words (16 bits) */
            vuint32_t W[2];     /* Data buffer in words (32 bits) */
            vuint32_t R[2];     /* Data buffer in words (32 bits) */
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
            vuint8_t B[8];      /* Data buffer in Bytes (8 bits) */
            vuint16_t H[4];     /* Data buffer in Half-words (16 bits) */
            vuint32_t W[2];     /* Data buffer in words (32 bits) */
            vuint32_t R[2];     /* Data buffer in words (32 bits) */
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
                vuint32_t DOZE:1;
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
/*                          MODULE : LINFLEX                                */
/****************************************************************************/

    struct LINFLEX_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t CCD:1;
                vuint32_t CFD:1;
                vuint32_t LASE:1;
                vuint32_t AWUM:1;
                vuint32_t MBL:4;
                vuint32_t BF:1;
                vuint32_t SLFM:1;
                vuint32_t LBKM:1;
                vuint32_t MME:1;
                vuint32_t SBDT:1;
                vuint32_t RBLM:1;
                vuint32_t SLEEP:1;
                vuint32_t INIT:1;
            } B;
        } LINCR1;               /* LINFLEX LIN Control Register 1 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t SZIE:1;
                vuint32_t OCIE:1;
                vuint32_t BEIE:1;
                vuint32_t CEIE:1;
                vuint32_t HEIE:1;
                  vuint32_t:2;
                vuint32_t FEIE:1;
                vuint32_t BOIE:1;
                vuint32_t LSIE:1;
                vuint32_t WUIE:1;
                vuint32_t DBFIE:1;
                vuint32_t DBEIE:1;
                vuint32_t DRIE:1;
                vuint32_t DTIE:1;
                vuint32_t HRIE:1;
            } B;
        } LINIER;               /* LINFLEX LIN Interrupt Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t LINS:4;
                  vuint32_t:2;
                vuint32_t RMB:1;
                  vuint32_t:1;
                vuint32_t RBSY:1;
                vuint32_t RPS:1;
                vuint32_t WUF:1;
                vuint32_t DBFF:1;
                vuint32_t DBEF:1;
                vuint32_t DRF:1;
                vuint32_t DTF:1;
                vuint32_t HRF:1;
            } B;
        } LINSR;                /* LINFLEX LIN Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t SZF:1;
                vuint32_t OCF:1;
                vuint32_t BEF:1;
                vuint32_t CEF:1;
                vuint32_t SFEF:1;
                vuint32_t SDEF:1;
                vuint32_t IDPEF:1;
                vuint32_t FEF:1;
                vuint32_t BOF:1;
                  vuint32_t:6;
                vuint32_t NF:1;
            } B;
        } LINESR;               /* LINFLEX LIN Error Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:1;
                vuint32_t TDFL:2;
                  vuint32_t:1;
                vuint32_t RDFL:2;
                  vuint32_t:4;
                vuint32_t RXEN:1;
                vuint32_t TXEN:1;
                vuint32_t OP:1;
                vuint32_t PCE:1;
                vuint32_t WL:1;
                vuint32_t UART:1;
            } B;
        } UARTCR;               /* LINFLEX UART Mode Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t SZF:1;
                vuint32_t OCF:1;
                vuint32_t PE:4;
                vuint32_t RMB:1;
                vuint32_t FEF:1;
                vuint32_t BOF:1;
                vuint32_t RPS:1;
                vuint32_t WUF:1;
                  vuint32_t:2;
                vuint32_t DRF:1;
                vuint32_t DTF:1;
                vuint32_t NF:1;
            } B;
        } UARTSR;               /* LINFLEX UART Mode Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:5;
                vuint32_t LTOM:1;
                vuint32_t IOT:1;
                vuint32_t TOCE:1;
                vuint32_t CNT:8;
            } B;
        } LINTCSR;              /* LINFLEX LIN Time-Out Control Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t OC2:8;
                vuint32_t OC1:8;
            } B;
        } LINOCR;               /* LINFLEX LIN Output Compare Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:4;
                vuint32_t RTO:4;
                  vuint32_t:1;
                vuint32_t HTO:7;
            } B;
        } LINTOCR;              /* LINFLEX LIN Output Compare Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:12;
                vuint32_t DIV_F:4;
            } B;
        } LINFBRR;              /* LINFLEX LIN Fractional Baud Rate Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:3;
                vuint32_t DIV_M:13;
            } B;
        } LINIBRR;              /* LINFLEX LIN Integer Baud Rate Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:8;
                vuint32_t CF:8;
            } B;
        } LINCFR;               /* LINFLEX LIN Checksum Field Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:1;
                vuint32_t IOBE:1;
                vuint32_t IOPE:1;
                vuint32_t WURQ:1;
                vuint32_t DDRQ:1;
                vuint32_t DTRQ:1;
                vuint32_t ABRQ:1;
                vuint32_t HTRQ:1;
                  vuint32_t:8;
            } B;
        } LINCR2;               /* LINFLEX LIN Control Register 2 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t DFL:6;
                vuint32_t DIR:1;
                vuint32_t CCS:1;
                  vuint32_t:2;
                vuint32_t ID:6;
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

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:8;
                vuint32_t FACT:8;
            } B;
        } IFER;                 /* LINFLEX Identifier Filter Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:12;
                vuint32_t IFMI:4;
            } B;
        } IFMI;                 /* LINFLEX Identifier Filter Match Index Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:12;
                vuint32_t IFM:4;
            } B;
        } IFMR;                 /* LINFLEX Identifier Filter Mode Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t:3;
                vuint32_t DFL:3;
                vuint32_t DIR:1;
                vuint32_t CCS:1;
                  vuint32_t:2;
                vuint32_t ID:6;
            } B;
        } IFCR[16];             /* LINFLEX Identifier Filter Control Register 0-15 */

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
                  vuint32_t:9;
                vuint32_t S_FMPLL:1;
                vuint32_t S_FXOSC:1;
                vuint32_t S_FIRC:1;
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
                vuint32_t M_MTC:1;
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

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t MPH_BUSY:1;
                  vuint32_t:2;
                vuint32_t PMC_PROG:1;
                vuint32_t CORE_DBG:1;
                  vuint32_t:2;
                vuint32_t SMR:1;
                  vuint32_t:1;
                vuint32_t FMPLL_SC:1;
                vuint32_t FXOSC_SC:1;
                vuint32_t FIRC_SC:1;
                  vuint32_t:1;
                vuint32_t SYSCLK_SW:1;
                vuint32_t DFLASH_SC:1;
                vuint32_t CFLASH_SC:1;
                vuint32_t CDP_PRPH_0_143:1;
                  vuint32_t:3;
                vuint32_t CDP_PRPH_96_127:1;
                vuint32_t CDP_PRPH_64_95:1;
                vuint32_t CDP_PRPH_32_63:1;
                vuint32_t CDP_PRPH_0_31:1;
            } B;
        } DMTS;                 /* Invalid Mode Transition Status Register */

        int32_t ME_reserved0;

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                  vuint32_t:9;
                vuint32_t FMPLLON:1;
                vuint32_t FXOSC0ON:1;
                vuint32_t FIRCON:1;
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
                  vuint32_t:9;
                vuint32_t FMPLLON:1;
                vuint32_t FXOSC0ON:1;
                vuint32_t FIRCON:1;
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
                  vuint32_t:9;
                vuint32_t FMPLLON:1;
                vuint32_t FXOSC0ON:1;
                vuint32_t FIRCON:1;
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
                  vuint32_t:9;
                vuint32_t FMPLLON:1;
                vuint32_t FXOSC0ON:1;
                vuint32_t FIRCON:1;
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
                  vuint32_t:9;
                vuint32_t FMPLLON:1;
                vuint32_t FXOSC0ON:1;
                vuint32_t FIRCON:1;
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
                  vuint32_t:9;
                vuint32_t FMPLLON:1;
                vuint32_t FXOSC0ON:1;
                vuint32_t FIRCON:1;
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
                  vuint32_t:9;
                vuint32_t FMPLLON:1;
                vuint32_t FXOSC0ON:1;
                vuint32_t FIRCON:1;
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
                  vuint32_t:9;
                vuint32_t FMPLLON:1;
                vuint32_t FXOSC0ON:1;
                vuint32_t FIRCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } STANDBY0;             /* STANDBY0 Mode Configuration Register */

        int32_t ME_reserved3[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:10;
                vuint32_t S_FLEXCAN5:1;
                vuint32_t S_FLEXCAN4:1;
                vuint32_t S_FLEXCAN3:1;
                vuint32_t S_FLEXCAN2:1;
                vuint32_t S_FLEXCAN1:1;
                vuint32_t S_FLEXCAN0:1;
                  vuint32_t:9;
                vuint32_t S_DSPI2:1;
                vuint32_t S_DSPI1:1;
                vuint32_t S_DSPI0:1;
                  vuint32_t:4;
            } B;
        } PS0;                  /* Peripheral Status Register 0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t S_CANSAMPLER:1;
                  vuint32_t:2;
                vuint32_t S_CTU:1;
                  vuint32_t:5;
                vuint32_t S_LINFLEX3:1;
                vuint32_t S_LINFLEX2:1;
                vuint32_t S_LINFLEX1:1;
                vuint32_t S_LINFLEX0:1;
                  vuint32_t:3;
                vuint32_t S_I2C:1;
                  vuint32_t:11;
                vuint32_t S_ADC:1;
            } B;
        } PS1;                  /* Peripheral Status Register 1 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t S_PIT_RTI:1;
                vuint32_t S_RTC_API:1;
                  vuint32_t:18;
                vuint32_t S_EMIOS:1;
                  vuint32_t:2;
                vuint32_t S_WKUP:1;
                vuint32_t S_SIU:1;
                  vuint32_t:4;
            } B;
        } PS2;                  /* Peripheral Status Register 2 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:23;
                vuint32_t S_CMU:1;
                  vuint32_t:8;
            } B;
        } PS3;                  /* Peripheral Status Register 3 */

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
        } PCONF[3];             /* Power domain 0-2 configuration register */

        int32_t PCU_reserved0[13];      /* (0x040 - 0x00C)/4 = 0x0D */

        union {
            vuint32_t R;
            struct {
                vuint32_t:29;
                vuint32_t PD2:1;
                vuint32_t PD1:1;
                vuint32_t PD0:1;
            } B;
        } PSTAT;                /* Power Domain Status Register */

        int32_t PCU_reserved1[15];      /* {0x0080-0x0044}/0x4 = 0xF */

        union {
            vuint32_t R;
            struct {
                vuint32_t:15;
                vuint32_t MASK_LVDHV5:1;
            } B;
        } VCTL;                 /* Voltage Regulator Control Register */

    };                          /* end of PCU_tag */
/****************************************************************************/
/*                          MODULE : pit                                    */
/****************************************************************************/
    struct PIT_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:30;
                vuint32_t MDIS:1;
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
/*                          MODULE : RGM                                   */
/****************************************************************************/
    struct RGM_tag {

        union {
            vuint16_t R;
            struct {
                vuint16_t F_EXR:1;
                  vuint16_t:6;
                vuint16_t F_FLASH:1;
                vuint16_t F_LVD45:1;
                vuint16_t F_CMU_FHL:1;
                vuint16_t F_CMU_OLR:1;
                vuint16_t F_FMPLL:1;
                vuint16_t F_CHKSTOP:1;
                vuint16_t F_SOFT:1;
                vuint16_t F_CORE:1;
                vuint16_t F_JTAG:1;
            } B;
        } FES;                  /* Functional Event Status */

        union {
            vuint16_t R;
            struct {
                vuint16_t F_POR:1;
                  vuint16_t:11;
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
                  vuint16_t:6;
                vuint16_t D_FLASH:1;
                vuint16_t D_LVD45:1;
                vuint16_t D_CMU_FHL:1;
                vuint16_t D_CMU_OLR:1;
                vuint16_t D_FMPLL:1;
                vuint16_t D_CHKSTOP:1;
                vuint16_t D_SOFT:1;
                vuint16_t D_CORE:1;
                vuint16_t D_JTAG:1;
            } B;
        } FERD;                 /* Functional Event Reset Disable */

        union {
            vuint16_t R;
            struct {
                vuint16_t D_POR:1;
                  vuint16_t:11;
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
                vuint16_t AR_EXR:1;
                  vuint16_t:6;
                vuint16_t AR_FLASH:1;
                vuint16_t AR_LVD45:1;
                vuint16_t AR_CMU_FHL:1;
                vuint16_t AR_CMU_OLR:1;
                vuint16_t AR_FMPLL:1;
                vuint16_t AR_CHKSTOP:1;
                vuint16_t AR_SOFT:1;
                vuint16_t AR_CORE:1;
                vuint16_t AR_JTAG:1;
            } B;
        } FEAR;                 /* Functional Event Alternate Request */

        union {
            vuint16_t R;
            struct {
                vuint16_t:12;
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
                vuint16_t:8;
                vuint16_t SS_LVD45:1;
                vuint16_t SS_CMU_FHL:1;
                vuint16_t SS_CMU_OLR:1;
                vuint16_t SS_PLL:1;
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
                vuint16_t BOOT_FROM_BKP_RAM:1;
                  vuint16_t:7;
            } B;
        } STDBY;                /* STANDBY reset sequence */

        union {
            vuint16_t R;
            struct {
                vuint16_t BE_EXR:1;
                  vuint16_t:6;
                vuint16_t BE_FLASH:1;
                vuint16_t BE_LVD45:1;
                vuint16_t BE_CMU_FHL:1;
                vuint16_t BE_CMU_OLR:1;
                vuint16_t BE_FMPLL:1;
                vuint16_t BE_CHKSTOP:1;
                vuint16_t BE_SOFT:1;
                vuint16_t BE_CORE:1;
                vuint16_t BE_JTAG:1;
            } B;
        } FBRE;                 /* Functional Bidirectional Reset Enable */

    };                          /* end of RGM_tag */
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
                vuint32_t APIIE:1;
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

        int32_t SIU_reserved0;  /* {0x004-0x000}/4 = 0x01 */

        union {                 /* MCU ID Register 1 */
            vuint32_t R;
            struct {
                vuint32_t PARTNUM:16;
                vuint32_t CSP:1;
                vuint32_t PKG:5;
                  vuint32_t:2;
                vuint32_t MAJOR_MASK:4;
                vuint32_t MINOR_MASK:4;
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
                  vuint32_t:4;
            } B;
        } MIDR2;

        int32_t SIU_reserved1[2];       /* {0x014-0x00C}/4 = 0x02 */

        union {                 /* Interrupt Status Flag Register */
            vuint32_t R;
            struct {
                vuint32_t:16;
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
                vuint32_t:16;
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

        int32_t SIU_reserved2[3];       /* {0x028-0x01C}/4 = 0x03 */

        union {                 /* Interrupt Rising-Edge Event Enable Register */
            vuint32_t R;
            struct {
                vuint32_t:16;
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
                vuint32_t:16;
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
                vuint32_t:16;
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

        int32_t SIU_reserved3[3];       /* {0x040-0x034}/4 = 0x03 */

        union {                 /* Pad Configuration Registers */
            vuint16_t R;
            struct {
                vuint16_t:1;
                vuint16_t SMC:1;
                vuint16_t APC:1;
                  vuint16_t:1;
                vuint16_t PA:2;
                vuint16_t OBE:1;
                vuint16_t IBE:1;
                  vuint16_t:2;
                vuint16_t ODE:1;
                  vuint16_t:2;
                vuint16_t SRC:1;
                vuint16_t WPE:1;
                vuint16_t WPS:1;
            } B;
        } PCR[123];

        int32_t SIU_reserved4[242];     /* {0x500-0x136}/0xF2 */

        union {                 /* Pad Selection for Multiplexed Input Register */
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t PADSEL:4;
            } B;
        } PSMI[32];

        int32_t SIU_reserved5[56];      /* {0x600-0x520}/4 = 0x38 */

        union {                 /* GPIO Pin Data Output Registers */
            vuint8_t R;
            struct {
                vuint8_t:7;
                vuint8_t PDO:1;
            } B;
        } GPDO[124];

        int32_t SIU_reserved6[97];      /* {0x800-0x67C}/4 = 0x61 */

        union {                 /* GPIO Pin Data Input Registers */
            vuint8_t R;
            struct {
                vuint8_t:7;
                vuint8_t PDI:1;
            } B;
        } GPDI[124];

        int32_t SIU_reserved7[225];     /* {0xC00-0x87C}/0x4 = 0xE1 */

        union {                 /* Parallel GPIO Pin Data Output Register */
            vuint32_t R;
            struct {
                vuint32_t PPD0:32;
            } B;
        } PGPDO[4];

        int32_t SIU_reserved8[12];      /* {0xC40-0xC10}/0x4 = 0x0C */

        union {                 /* Parallel GPIO Pin Data Input Register */
            vuint32_t R;
            struct {
                vuint32_t PPDI:32;
            } B;
        } PGPDI[4];

        int32_t SIU_reserved9[12];      /* {0xC80-0xC50}/0x4 = 0x0C */

        union {                 /* Masked Parallel GPIO Pin Data Out Register */
            vuint32_t R;
            struct {
                vuint32_t MASK:16;
                vuint32_t MPPDO:16;
            } B;
        } MPGPDO[8];

        int32_t SIU_reserved10[216];    /* {0x1000-0x0CA0}/4 = 0xD8 */

        union {                 /* Interrupt Filter Maximum Counter Register */
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t MAXCNT:4;
            } B;
        } IFMC[16];

        int32_t SIU_reserved11[16];     /* {0x1080-0x1040}/4 = 0x10 */

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
                  vuint16_t:3;
                vuint16_t BMODE:3;
                  vuint16_t:1;
                vuint16_t ABD:1;
                  vuint16_t:3;
            } B;
        } STATUS;               /* Status Register */

        union {
            vuint16_t R;
            struct {
                vuint16_t SRAM_SIZE:5;
                vuint16_t PRSZ:5;
                vuint16_t PVLB:1;
                vuint16_t DTSZ:4;
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

        int16_t SSCM_reserved1[2];

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
    struct STM_CHANNEL_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR;                  /* STM Channel Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR;                  /* STM Channel Interrupt Register */

        union {
            vuint32_t R;
        } CMP;                  /* STM Channel Compare Register 0 */

        int32_t STM_CHANNEL_reserved;

    };                          /* end of STM_CHANNEL_tag */

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
        } CR;                   /* STM Control Register */

        union {
            vuint32_t R;
        } CNT;                  /* STM Count Register */

        int32_t STM_reserved[2];

        struct STM_CHANNEL_tag CH[4];

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
                  vuint32_t:30;
            } B;
        } NSR;                  /* NMI Status Register */

        int32_t WKUP_reserved;

        union {
            vuint32_t R;
            struct {
                vuint32_t NLOCK:1;
                vuint32_t NDSS:2;
                vuint32_t NWRE:1;
                  vuint32_t:1;
                vuint32_t NREE:1;
                vuint32_t NFEE:1;
                vuint32_t NFE:1;
                  vuint32_t:24;
            } B;
        } NCR;                  /* NMI Configuration Register */

        int32_t WKUP_reserved1[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t EIF:20;
            } B;
        } WISR;                 /* Wakeup/Interrupt Status Flag Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t EIRE:20;
            } B;
        } IRER;                 /* Interrupt Request Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t WRE:20;
            } B;
        } WRER;                 /* Wakeup Request Enable Register */

        int32_t WKUP_reserved2[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t IREE:20;
            } B;
        } WIREER;               /* Wakeup/Interrupt Rising-Edge Event Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t IFEE:20;
            } B;
        } WIFEER;               /* Wakeup/Interrupt Falling-Edge Event Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t IFE:20;
            } B;
        } WIFER;                /* Wakeup/Interrupt Filter Enable Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t IPUE:20;
            } B;
        } WIPUER;               /* Wakeup/Interrupt Pullup Enable Register */

    };                          /* end of WKUP_tag */
/****************************************************************** 
| defines and macros (scope: module-local) 
|-----------------------------------------------------------------*/ 
/* Define instances of modules */ 
#define ADC       (*(volatile struct ADC_tag *)       0xFFE00000UL)
#define CAN_0     (*(volatile struct FLEXCAN_tag *)   0xFFFC0000UL)
#define CAN_1     (*(volatile struct FLEXCAN_tag *)   0xFFFC4000UL)
#define CAN_2     (*(volatile struct FLEXCAN_tag *)   0xFFFC8000UL)
#define CAN_3     (*(volatile struct FLEXCAN_tag *)   0xFFFCC000UL)
#define CAN_4     (*(volatile struct FLEXCAN_tag *)   0xFFFD0000UL)
#define CAN_5     (*(volatile struct FLEXCAN_tag *)   0xFFFD4000UL)
#define CANSP     (*(volatile struct CANSP_tag *)     0xFFE70000UL)
#define CFLASH    (*(volatile struct CFLASH_tag *)    0xC3F88000UL)
#define CGM       (*(volatile struct CGM_tag *)       0xC3FE0000UL)
#define CTU       (*(volatile struct CTU_tag *)       0xFFE64000UL)
#define DFLASH    (*(volatile struct DFLASH_tag *)    0xC3F8C000UL)
#define DSPI_0    (*(volatile struct DSPI_tag *)      0xFFF90000UL)
#define DSPI_1    (*(volatile struct DSPI_tag *)      0xFFF94000UL)
#define DSPI_2    (*(volatile struct DSPI_tag *)      0xFFF98000UL)
#define DSPI_3    (*(volatile struct DSPI_tag *)      0xFFF9C000UL)
#define ECSM      (*(volatile struct ECSM_tag *)      0xFFF40000UL)   
#define EMIOS_0   (*(volatile struct EMIOS_tag *)     0xC3FA0000UL)
#define EMIOS_1   (*(volatile struct EMIOS_tag *)     0xC3FA4000UL)
#define I2C       (*(volatile struct I2C_tag *)       0xFFE30000UL)
#define INTC      (*(volatile struct INTC_tag *)      0xFFF48000UL)  
#define LINFLEX_0 (*(volatile struct LINFLEX_tag *)   0xFFE40000UL)
#define LINFLEX_1 (*(volatile struct LINFLEX_tag *)   0xFFE44000UL) 
#define LINFLEX_2 (*(volatile struct LINFLEX_tag *)   0xFFE48000UL) 
#define LINFLEX_3 (*(volatile struct LINFLEX_tag *)   0xFFE4C000UL) 
#define ME        (*(volatile struct ME_tag *)        0xC3FDC000UL)
#define MPU       (*(volatile struct MPU_tag *)       0xFFF10000UL)
#define PCU       (*(volatile struct PCU_tag *)       0xC3FE8000UL)
#define PIT       (*(volatile struct PIT_tag *)       0xC3FF0000UL)
#define RGM       (*(volatile struct RGM_tag *)       0xC3FE4000UL)
#define RTC       (*(volatile struct RTC_tag *)       0xC3FEC000UL)
#define SIU       (*(volatile struct SIU_tag *)       0xC3F90000UL)
#define SSCM      (*(volatile struct SSCM_tag *)      0xC3FD8000UL) 
#define STM       (*(volatile struct STM_tag *)       0xFFF3C000UL)
#define SWT       (*(volatile struct SWT_tag *)       0xFFF38000UL)
#define WKUP      (*(volatile struct WKUP_tag *)      0xC3F94000UL)  
     
#ifdef __MWERKS__
#pragma pop
#endif  /* 
 */
     
#ifdef  __cplusplus
} 
#endif  /* 
 */
 
#endif  /* ifdef _MPC5604B_H */
 
/* End of file */ 
