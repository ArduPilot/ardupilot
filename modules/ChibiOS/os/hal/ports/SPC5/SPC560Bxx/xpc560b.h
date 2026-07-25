/***************************************************************** 
 * PROJECT     : MPC5607B
 *               
 * FILE        : jdp.h
 * 
 * DESCRIPTION : This is the header file describing the register
 *               set for MPC5607B
 * 
 * COPYRIGHT   :(c) 2008, Freescale & STMicroelectronics 
 * 
 * VERSION     : 01.03 
 * DATE        : 1.20.2010 
 * AUTHOR      : r23668
 * HISTORY     : Hand edited from previous jdp.h file
 * Changes from rev 1.01
 * CGM Section replaced with more accurate section from Bolero 512K
 * eMIOS CADR, CBDR & CCNTR changed from 24 to 16 bit.
 * CAN Sampler section corrected: 0-15 should be resrved not 16-32.
 * Includes additions from Stefan Luellman
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
/*                          MODULE : ADC0                                   */
/****************************************************************************/
    struct ADC0_tag {

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
                vuint32_t ABORT_CHAIN:1;
                vuint32_t ABORT:1;
                vuint32_t ACKO:1;
                vuint32_t:1; //vuint32_t OFFREFRESH:1;
                vuint32_t:1; //vuint32_t OFFCANC:1;
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
                vuint32_t ACKO:1;
                vuint32_t:1; //vuint32_t OFFREFRESH:1;
                vuint32_t:1; //vuint32_t OFFCANC:1;
                vuint32_t ADCSTATUS:3;
            } B;
        } MSR;                 /* MAIN STATUS REGISTER */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC0_reserved0;

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC0_reserved1;
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:25;
                vuint32_t:1; //vuint32_t OFFCANCOVR:1;
                vuint32_t:1; //vuint32_t EOFFSET:1;
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
                vuint32_t :16;
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
        } CE0CFR0;                 /* PRECISE CHANNELS PENDING REGISTERS */
        
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
        } CE0CFR1;                 /* EXTENDED INTERNAL CHANNELS PENDING REGISTERS */
        
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
        } CE0CFR2;                 /* EXTERNAL CHANNELS PENDING REGISTERS */     

        union {
            vuint32_t R;
            struct {
                vuint32_t:25;
                vuint32_t:1; //vuint32_t MSKOFFCANCOVR:1;
                vuint32_t:1; //vuint32_t MSKEOFFSET:1;
                vuint32_t MSKEOCTU:1;
                vuint32_t MSKJEOC:1;
                vuint32_t MSKJECH:1;
                vuint32_t MSKEOC:1;
                vuint32_t MSKECH:1;
            } B;
        } IMR;                 /* INTERRUPT MASK REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
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
        } CIMR0;               /* PRECISE CHANNELS INTERRUPT MASK 0 */

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
        } CIMR1;               /* EXTENDED INTERNAL CHANNELS INTERRUPT MASK 1 */

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
        } CIMR2;               /* EXTERNAL CHANNELS INTERRUPT MASK 2 */

        union {
            vuint32_t R;
            struct {             
                vuint32_t:20;
                vuint32_t WDG5H:1; // non validi
                vuint32_t WDG5L:1; // non validi
                vuint32_t WDG4H:1; // non validi
                vuint32_t WDG4L:1; // non validi
                vuint32_t WDG3H:1; // validi
                vuint32_t WDG3L:1; // validi
                vuint32_t WDG2H:1; // validi
                vuint32_t WDG2L:1; // validi
				vuint32_t WDG1H:1; // validi
				vuint32_t WDG1L:1; // validi
				vuint32_t WDG0H:1; // validi
                vuint32_t WDG0L:1; // validi
            } B;  
        } WTISR;            /* WATCHDOG THRESHOLD INTERRUPT STATUS REGISTER */
        
        union {
            vuint32_t R;
            struct {             
                vuint32_t:20;
                vuint32_t MSKWDG5H:1; // non validi
                vuint32_t MSKWDG5L:1; // non validi
                vuint32_t MSKWDG4H:1; // non validi
                vuint32_t MSKWDG4L:1; // non validi
                vuint32_t MSKWDG3H:1; // validi
                vuint32_t MSKWDG2H:1; // validi
                vuint32_t MSKWDG1H:1; // validi
                vuint32_t MSKWDG0H:1; // validi
				vuint32_t MSKWDG3L:1; // validi
				vuint32_t MSKWDG2L:1; // validi
				vuint32_t MSKWDG1L:1; // validi
                vuint32_t MSKWDG0L:1; // validi
            } B;  
        } WTIMR;            /* WATCHDOG THRESHOLD INTERRUPT MASK REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC0_reserved2;

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC0_reserved3;
               
        union {
            vuint32_t R;
            struct {             
                vuint32_t:30;
                vuint32_t DCLR:1;
                vuint32_t DMAEN:1;
            } B;
        } DMAE;            /* DMA ENABLE REGISTER */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:16;
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
        } DMAR0;            /* PRECISE CHANNELS DMA REGISTER 0 */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t DMA31:1;
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
        } DMAR1;            /* EXTENDED INTERNAL CHANNELS DMA REGISTER 1 */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t DMA31:1;
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
        } DMAR2;            /* EXTERNAL CHANNELS DMA REGISTER 2 */
        
        int32_t ADC0_reserved11[4];
               
        union {
            vuint32_t R;
            struct {
	            vuint32_t:6;
                vuint32_t THRH:10;
                vuint32_t:6;
                vuint32_t THRL:10;
            } B;
        } THRHLR[4];            /* THRESHOLD REGISTER 0-3 */
                        

        int32_t ADC0_reserved12[4];
	
        union {
            vuint32_t R;
            struct {
	            vuint32_t:25;
                vuint32_t PREVAL2:2;
                vuint32_t PREVAL1:2;
                vuint32_t PREVAL0:2;
                vuint32_t PRECONV:1;        
            } B;
        } PSCR;            /* PRESAMPLING CONTROL REGISTER */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
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
        } PSR0;            /* PRECISE CHANNELS PRESAMPLING REGISTER 0 */
        
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
        } PSR1;            /* EXTENDED INTERNAL CHANNELS PRESAMPLING REGISTER 1 */
        
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
        } PSR2;            /* EXTERNAL CHANNELS PRESAMPLING REGISTER 2 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC0_reserved4;
        
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
        } CTR0;            /* PRECISE CHANNELS CONVERSION TIMING REGISTER 0 */
        
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
        } CTR1;            /* EXTENDED INTERNAL CHANNELS CONVERSION TIMING REGISTER 1 */
        
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
        } CTR2;            /* EXTERNAL CHANNELS CONVERSION TIMING REGISTER 2 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC0_reserved5;
        
        union {
            vuint32_t R;
            struct {
                vuint32_t :16;
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
        } NCMR0;            /* PRECISE CHANNELS NORMAL CONVERSION MASK REGISTER 0 */
        
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
        } NCMR1;            /* EXTENDED INTERNAL CHANNELS NORMAL CONVERSION MASK REGISTER 1 */
        
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
        } NCMR2;            /* EXTERNAL CHANNELS NORMAL CONVERSION MASK REGISTER 2 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC0_reserved6;
              
        union {
            vuint32_t R;
            struct {
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
        } JCMR0;            /* PRECISE CHANNELS INJECTED CONVERSION MASK REGISTER 0 */
        
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
        } JCMR1;            /* EXTENDED INTERNAL CHANNELS INJECTED CONVERSION MASK REGISTER 1 */
        
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
        } JCMR2;            /* EXTERNAL CHANNELS INJECTED CONVERSION MASK REGISTER 2 */
               
        
        int32_t ADC0_reserved_OFFWR; /* Digital offset cancellation removed from 1.5M and removed from spec of 512K */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t DSD:8;
            } B;
        } DSDR;            /* DECODE SIGNALS DELAY REGISTER */                
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:24;                
                vuint32_t PDED:8;
            } B;
        } PDEDR;            /* POWER DOWN EXIT DELAY REGISTER */                

    
        
        int32_t ADC0_reserved7[13];     /* {0x100-0x0F0}/0x4 = 4 */
                
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
        } CDR[96];            /* CHANNEL x DATA REGISTER */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:6;
                vuint32_t THRH:10;
                vuint32_t:6;
                vuint32_t THRL:10;
            } B;
        } THRHLR4;            /* THRESHOLD REGISTER 4 */     /* Bolero 1.5M / ADC0 only */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:6;
                vuint32_t THRH:10;
                vuint32_t:6;
                vuint32_t THRL:10;
            } B;
        } THRHLR5;            /* THRESHOLD REGISTER 5 */     /* Bolero 1.5M / ADC0 only */
        
        int32_t ADC0_reserved8[10]; 
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH7:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH6:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH5:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH4:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH3:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH2:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH1:3;
                vuint32_t:1;
                vuint32_t WSEL_CH0:3;
            } B;
        } CWSELR0; /* CHANNEL WATCHDOG SELECTION REGISTERS (PRECISE CHANNELS) */     /* Bolero 1.5M / ADC0 only */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH15:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH14:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH13:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH12:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH11:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH10:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH9:3;
                vuint32_t:1;
                vuint32_t WSEL_CH8:3;
            } B;
        } CWSELR1; /* CHANNEL WATCHDOG SELECTION REGISTERS (PRECISE CHANNELS) */     /* Bolero 1.5M / ADC0 only */
        
       	union {
            vuint32_t R;
            struct {
	            vuint32_t:32;
            } B;
        } CWSELR2; /* reserved (16 precise channels only) */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:32;
            } B;
        } CWSELR3; /* reserved (16 precise channels only) */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH39:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH38:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH37:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH36:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH35:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH34:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH33:3;
                vuint32_t:1;
                vuint32_t WSEL_CH32:3;
            } B;
        } CWSELR4; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTENDED INTERNAL CHANNELS) */     /* Bolero 1.5M / ADC0 only */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH47:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH46:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH45:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH44:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH43:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH42:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH41:3;
                vuint32_t:1;
                vuint32_t WSEL_CH40:3;
            } B;
        } CWSELR5; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTENDED INTERNAL CHANNELS) */     /* Bolero 1.5M / ADC0 only */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH55:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH54:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH53:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH52:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH51:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH50:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH49:3;
                vuint32_t:1;
                vuint32_t WSEL_CH48:3;
            } B;
        } CWSELR6; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTENDED INTERNAL CHANNELS) */     /* Bolero 1.5M / ADC0 only */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH63:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH62:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH61:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH60:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH59:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH58:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH57:3;
                vuint32_t:1;
                vuint32_t WSEL_CH56:3;
            } B;
        } CWSELR7; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTENDED INTERNAL CHANNELS) */     /* Bolero 1.5M / ADC0 only */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH71:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH70:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH69:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH68:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH67:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH66:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH65:3;
                vuint32_t:1;
                vuint32_t WSEL_CH64:3;
            } B;
        } CWSELR8; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTERNAL CHANNELS) */     /* Bolero 1.5M / ADC0 only */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH79:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH78:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH77:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH76:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH75:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH74:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH73:3;
                vuint32_t:1;
                vuint32_t WSEL_CH72:3;
            } B;
        } CWSELR9; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTERNAL CHANNELS) */     /* Bolero 1.5M / ADC0 only */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH87:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH86:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH85:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH84:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH83:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH82:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH81:3;
                vuint32_t:1;
                vuint32_t WSEL_CH80:3;
            } B;
        } CWSELR10; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTERNAL CHANNELS) */     /* Bolero 1.5M / ADC0 only */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH95:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH94:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH93:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH92:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH91:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH90:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH89:3;
                vuint32_t:1;
                vuint32_t WSEL_CH88:3;
            } B;
        } CWSELR11; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTERNAL CHANNELS) */     /* Bolero 1.5M / ADC0 only */
               
        union {
            vuint32_t R;
            struct {
                vuint32_t CWEN31:1;
	            vuint32_t CWEN30:1;
                vuint32_t CWEN29:1;
                vuint32_t CWEN28:1;
                vuint32_t CWEN27:1;
                vuint32_t CWEN26:1;
                vuint32_t CWEN25:1;
                vuint32_t CWEN24:1;
                vuint32_t CWEN23:1;
                vuint32_t CWEN22:1;
                vuint32_t CWEN21:1;
                vuint32_t CWEN20:1;
                vuint32_t CWEN19:1;
                vuint32_t CWEN18:1;
                vuint32_t CWEN17:1;
                vuint32_t CWEN16:1;
                vuint32_t CWEN15:1;
                vuint32_t CWEN14:1;
                vuint32_t CWEN13:1;
                vuint32_t CWEN12:1;
                vuint32_t CWEN11:1;
                vuint32_t CWEN10:1;
                vuint32_t CWEN9:1;
                vuint32_t CWEN8:1;
                vuint32_t CWEN7:1;
                vuint32_t CWEN6:1;
                vuint32_t CWEN5:1;
                vuint32_t CWEN4:1;
                vuint32_t CWEN3:1;
                vuint32_t CWEN2:1;
                vuint32_t CWEN1:1;
                vuint32_t CWEN0:1; 
            } B;
        } CWENR[3]; /* CHANNEL WATCHDOG ENABLE REGISTERS 0-2 */
        
        int32_t ADC0_reserved9;          
        
        union {
            vuint32_t R;
            struct {
                vuint32_t AWORR_CH31:1;
	            vuint32_t AWORR_CH30:1;
                vuint32_t AWORR_CH29:1;
                vuint32_t AWORR_CH28:1;
                vuint32_t AWORR_CH27:1;
                vuint32_t AWORR_CH26:1;
                vuint32_t AWORR_CH25:1;
                vuint32_t AWORR_CH24:1;
                vuint32_t AWORR_CH23:1;
                vuint32_t AWORR_CH22:1;
                vuint32_t AWORR_CH21:1;
                vuint32_t AWORR_CH20:1;
                vuint32_t AWORR_CH19:1;
                vuint32_t AWORR_CH18:1;
                vuint32_t AWORR_CH17:1;
                vuint32_t AWORR_CH16:1;
                vuint32_t AWORR_CH15:1;
                vuint32_t AWORR_CH14:1;
                vuint32_t AWORR_CH13:1;
                vuint32_t AWORR_CH12:1;
                vuint32_t AWORR_CH11:1;
                vuint32_t AWORR_CH10:1;
                vuint32_t AWORR_CH9:1;
                vuint32_t AWORR_CH8:1;
                vuint32_t AWORR_CH7:1;
                vuint32_t AWORR_CH6:1;
                vuint32_t AWORR_CH5:1;
                vuint32_t AWORR_CH4:1;
                vuint32_t AWORR_CH3:1;
                vuint32_t AWORR_CH2:1;
                vuint32_t AWORR_CH1:1;
                vuint32_t AWORR_CH0:1; 
            } B;
        } AWORR[3];            /* ANALOG WATCHDOG OUT OF RANGE REGISTERS 0-2 */
        
        int32_t ADC0_reserved10; 
        
    };                          /* end of ADC0_tag */    
    


/****************************************************************************/
/*                          MODULE : ADC1                                   */
/****************************************************************************/
    struct ADC1_tag {

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
                vuint32_t ABORT_CHAIN:1;
                vuint32_t ABORT:1;
                vuint32_t ACKO:1;
                vuint32_t:1; //vuint32_t OFFREFRESH:1;
                vuint32_t:1; //vuint32_t OFFCANC:1;
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
                vuint32_t ACKO:1;
                vuint32_t:1; //vuint32_t OFFREFRESH:1;
                vuint32_t:1; //vuint32_t OFFCANC:1;
                vuint32_t ADCSTATUS:3;
            } B;
        } MSR;                 /* MAIN STATUS REGISTER */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC1_reserved0;

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC1_reserved1;
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:25;
                vuint32_t:1; //vuint32_t OFFCANCOVR:1;
                vuint32_t:1; //vuint32_t EOFFSET:1;
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
                vuint32_t :16;
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
        } CE0CFR0;                 /* PRECISE CHANNELS PENDING REGISTERS */

        int32_t ADC1_reserved11[2];
        
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:25;
                vuint32_t:1; //vuint32_t MSKOFFCANCOVR:1;
                vuint32_t:1; //vuint32_t MSKEOFFSET:1;
                vuint32_t MSKEOCTU:1;
                vuint32_t MSKJEOC:1;
                vuint32_t MSKJECH:1;
                vuint32_t MSKEOC:1;
                vuint32_t MSKECH:1;
            } B;
        } IMR;                 /* INTERRUPT MASK REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
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
        } CIMR0;               /* PRECISE CHANNELS INTERRUPT MASK 0 */
        
        
         union {
            vuint32_t R;
            struct {
                vuint32_t CIM63:1;
                vuint32_t CIM62:1;
                vuint32_t CIM61:1;
                vuint32_t CIM60:1;
                vuint32_t CIM59:1;
                vuint32_t CIM58:1;
                vuint32_t CIM57:1;
                vuint32_t CIM56:1;
                vuint32_t CIM55:1;
                vuint32_t CIM54:1;
                vuint32_t CIM53:1;
                vuint32_t CIM52:1;
                vuint32_t CIM51:1;
                vuint32_t CIM50:1;
                vuint32_t CIM49:1;
                vuint32_t CIM48:1;
                vuint32_t CIM47:1;
                vuint32_t CIM46:1;
                vuint32_t CIM45:1;
                vuint32_t CIM44:1;
                vuint32_t CIM43:1;
                vuint32_t CIM42:1;
                vuint32_t CIM41:1;
                vuint32_t CIM40:1;
                vuint32_t CIM39:1;
                vuint32_t CIM38:1;
                vuint32_t CIM37:1;
                vuint32_t CIM36:1;
                vuint32_t CIM35:1;
                vuint32_t CIM34:1;
                vuint32_t CIM33:1;
                vuint32_t CIM32:1;
            } B;
        } CIMR1;               /* EXTENDED CHANNELS INTERRUPT MASK 1 */


         union {
            vuint32_t R;
            struct {
                vuint32_t CIM95:1;
                vuint32_t CIM94:1;
                vuint32_t CIM93:1;
                vuint32_t CIM92:1;
                vuint32_t CIM91:1;
                vuint32_t CIM90:1;
                vuint32_t CIM89:1;
                vuint32_t CIM88:1;
                vuint32_t CIM87:1;
                vuint32_t CIM86:1;
                vuint32_t CIM85:1;
                vuint32_t CIM84:1;
                vuint32_t CIM83:1;
                vuint32_t CIM82:1;
                vuint32_t CIM81:1;
                vuint32_t CIM80:1;
                vuint32_t CIM79:1;
                vuint32_t CIM78:1;
                vuint32_t CIM77:1;
                vuint32_t CIM76:1;
                vuint32_t CIM75:1;
                vuint32_t CIM74:1;
                vuint32_t CIM73:1;
                vuint32_t CIM72:1;
                vuint32_t CIM71:1;
                vuint32_t CIM70:1;
                vuint32_t CIM69:1;
                vuint32_t CIM68:1;
                vuint32_t CIM67:1;
                vuint32_t CIM66:1;
                vuint32_t CIM65:1;
                vuint32_t CIM64:1;
            } B;
        } CIMR2;               /* EXTERNAL CHANNELS INTERRUPT MASK 2 */

        union {
            vuint32_t R;
            struct {             
                vuint32_t:20;
                vuint32_t WDG5H:1; // non validi
                vuint32_t WDG5L:1; // non validi
                vuint32_t WDG4H:1; // non validi
                vuint32_t WDG4L:1; // non validi
                vuint32_t WDG3H:1; // validi
                vuint32_t WDG3L:1; // validi
                vuint32_t WDG2H:1; // validi
                vuint32_t WDG2L:1; // validi
				vuint32_t WDG1H:1; // validi
				vuint32_t WDG1L:1; // validi
				vuint32_t WDG0H:1; // validi
                vuint32_t WDG0L:1; // validi
            } B;  
        } WTISR;            /* WATCHDOG THRESHOLD INTERRUPT STATUS REGISTER */
        
        union {
            vuint32_t R;
            struct {             
                vuint32_t:20;
                vuint32_t MSKWDG5H:1; // non validi
                vuint32_t MSKWDG5L:1; // non validi
                vuint32_t MSKWDG4H:1; // non validi
                vuint32_t MSKWDG4L:1; // non validi
                vuint32_t MSKWDG3H:1; // validi
                vuint32_t MSKWDG2H:1; // validi
                vuint32_t MSKWDG1H:1; // validi
                vuint32_t MSKWDG0H:1; // validi
				vuint32_t MSKWDG3L:1; // validi
				vuint32_t MSKWDG2L:1; // validi
				vuint32_t MSKWDG1L:1; // validi
                vuint32_t MSKWDG0L:1; // validi
            } B;  
        } WTIMR;            /* WATCHDOG THRESHOLD INTERRUPT MASK REGISTER */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC1_reserved2;

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC1_reserved3;
               
        union {
            vuint32_t R;
            struct {             
                vuint32_t:30;
                vuint32_t DCLR:1;
                vuint32_t DMAEN:1;
            } B;
        } DMAE;            /* DMA ENABLE REGISTER */
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:16;
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
        } DMAR0;            /* PRECISE CHANNELS DMA REGISTER 0 */
        
        union {
            vuint32_t R;
            struct {
	             vuint32_t DMA63:1;
	             vuint32_t DMA62:1;
	             vuint32_t DMA61:1;
	             vuint32_t DMA60:1;
	             vuint32_t DMA59:1;
	             vuint32_t DMA58:1;
	             vuint32_t DMA57:1;
	             vuint32_t DMA56:1;
	             vuint32_t DMA55:1;
	             vuint32_t DMA54:1;
	             vuint32_t DMA53:1;
	             vuint32_t DMA52:1;
	             vuint32_t DMA51:1;
	             vuint32_t DMA50:1;
	             vuint32_t DMA49:1;
	             vuint32_t DMA48:1;
	             vuint32_t DMA47:1;
	             vuint32_t DMA46:1;
	             vuint32_t DMA45:1;
	             vuint32_t DMA44:1;
	             vuint32_t DMA43:1;
	             vuint32_t DMA42:1;
	             vuint32_t DMA41:1;
	             vuint32_t DMA40:1;
	             vuint32_t DMA39:1;
	             vuint32_t DMA38:1;
	             vuint32_t DMA37:1;
	             vuint32_t DMA36:1;
	             vuint32_t DMA35:1;
	             vuint32_t DMA34:1;
	             vuint32_t DMA33:1;
	             vuint32_t DMA32:1;
            } B;
        } DMAR1;            /* EXTENDED INTERNAL CHANNELS DMA REGISTER 1 */
        
        union {
            vuint32_t R;
            struct {
	             vuint32_t DMA95:1;
	             vuint32_t DMA94:1;
	             vuint32_t DMA93:1;
	             vuint32_t DMA92:1;
	             vuint32_t DMA91:1;
	             vuint32_t DMA90:1;
	             vuint32_t DMA89:1;
	             vuint32_t DMA88:1;
	             vuint32_t DMA87:1;
	             vuint32_t DMA86:1;
	             vuint32_t DMA85:1;
	             vuint32_t DMA84:1;
	             vuint32_t DMA83:1;
	             vuint32_t DMA82:1;
	             vuint32_t DMA81:1;
	             vuint32_t DMA80:1;
	             vuint32_t DMA79:1;
	             vuint32_t DMA78:1;
	             vuint32_t DMA77:1;
	             vuint32_t DMA76:1;
	             vuint32_t DMA75:1;
	             vuint32_t DMA74:1;
	             vuint32_t DMA73:1;
	             vuint32_t DMA72:1;
	             vuint32_t DMA71:1;
	             vuint32_t DMA70:1;
	             vuint32_t DMA69:1;
	             vuint32_t DMA68:1;
	             vuint32_t DMA67:1;
	             vuint32_t DMA66:1;
	             vuint32_t DMA65:1;
	             vuint32_t DMA64:1;
            } B;
        } DMAR2;            /* EXTERNAL CHANNELS DMA REGISTER 2 */
        
        int32_t ADC1_reserved13[4];
               
        union {
            vuint32_t R;
            struct {
	            vuint32_t:4;
                vuint32_t THRH:12;
                vuint32_t:4;
                vuint32_t THRL:12;
            } B;
        } THRHLR[3];            /* THRESHOLD REGISTER 0-2 */
                        
        int32_t ADC1_reserved14[5];
                
        union {
            vuint32_t R;
            struct {
	            vuint32_t:25;
                vuint32_t PREVAL2:2;
                vuint32_t PREVAL1:2;
                vuint32_t PREVAL0:2;
                vuint32_t PRECONV:1;        
            } B;
        } PSCR;            /* PRESAMPLING CONTROL REGISTER */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
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
        } PSR0;            /* PRECISE CHANNELS PRESAMPLING REGISTER 0 */
        
        
         union {
            vuint32_t R;
            struct {
                vuint32_t PRES63:1;
                vuint32_t PRES62:1;
                vuint32_t PRES61:1;
                vuint32_t PRES60:1;
                vuint32_t PRES59:1;
                vuint32_t PRES58:1;
                vuint32_t PRES57:1;
                vuint32_t PRES56:1;
                vuint32_t PRES55:1;
                vuint32_t PRES54:1;
                vuint32_t PRES53:1;
                vuint32_t PRES52:1;
                vuint32_t PRES51:1;
                vuint32_t PRES50:1;
                vuint32_t PRES49:1;
                vuint32_t PRES48:1;
                vuint32_t PRES47:1;
                vuint32_t PRES46:1;
                vuint32_t PRES45:1;
                vuint32_t PRES44:1;
                vuint32_t PRES43:1;
                vuint32_t PRES42:1;
                vuint32_t PRES41:1;
                vuint32_t PRES40:1;
                vuint32_t PRES39:1;
                vuint32_t PRES38:1;
                vuint32_t PRES37:1;
                vuint32_t PRES36:1;
                vuint32_t PRES35:1;
                vuint32_t PRES34:1;
                vuint32_t PRES33:1;
                vuint32_t PRES32:1;
            } B;
        } PSR1;            /* EXTENDED CHANNELS PRESAMPLING REGISTER 1 */
        
         union {
            vuint32_t R;
            struct {
                vuint32_t PRES95:1;
                vuint32_t PRES94:1;
                vuint32_t PRES93:1;
                vuint32_t PRES92:1;
                vuint32_t PRES91:1;
                vuint32_t PRES90:1;
                vuint32_t PRES89:1;
                vuint32_t PRES88:1;
                vuint32_t PRES87:1;
                vuint32_t PRES86:1;
                vuint32_t PRES85:1;
                vuint32_t PRES84:1;
                vuint32_t PRES83:1;
                vuint32_t PRES82:1;
                vuint32_t PRES81:1;
                vuint32_t PRES80:1;
                vuint32_t PRES79:1;
                vuint32_t PRES78:1;
                vuint32_t PRES77:1;
                vuint32_t PRES76:1;
                vuint32_t PRES75:1;
                vuint32_t PRES74:1;
                vuint32_t PRES73:1;
                vuint32_t PRES72:1;
                vuint32_t PRES71:1;
                vuint32_t PRES70:1;
                vuint32_t PRES69:1;
                vuint32_t PRES68:1;
                vuint32_t PRES67:1;
                vuint32_t PRES66:1;
                vuint32_t PRES65:1;
                vuint32_t PRES64:1;
            } B;
        } PSR2;            /* EXTERNAL CHANNELS PRESAMPLING REGISTER 2 */
	
        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC1_reserved4;
        
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
        } CTR0;            /* PRECISE CHANNELS CONVERSION TIMING REGISTER 0 */
        
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
        } CTR1;            /* EXTENDED CHANNELS CONVERSION TIMING REGISTER 1 */
        
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
        } CTR2;            /* EXTERNAL CHANNELS CONVERSION TIMING REGISTER 2 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC1_reserved5;
        
        union {
            vuint32_t R;
            struct {
                vuint32_t :16;
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
        } NCMR0;            /* PRECISE CHANNELS NORMAL CONVERSION MASK REGISTER 0 */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t CH63:1;
                vuint32_t CH62:1;
                vuint32_t CH61:1;
                vuint32_t CH60:1;
                vuint32_t CH59:1;
                vuint32_t CH58:1;
                vuint32_t CH57:1;
                vuint32_t CH56:1;
                vuint32_t CH55:1;
                vuint32_t CH54:1;
                vuint32_t CH53:1;
                vuint32_t CH52:1;
                vuint32_t CH51:1;
                vuint32_t CH50:1;
                vuint32_t CH49:1;
                vuint32_t CH48:1;
                vuint32_t CH47:1;
                vuint32_t CH46:1;
                vuint32_t CH45:1;
                vuint32_t CH44:1;
                vuint32_t CH43:1;
                vuint32_t CH42:1;
                vuint32_t CH41:1;
                vuint32_t CH40:1;
                vuint32_t CH39:1;
                vuint32_t CH38:1;
                vuint32_t CH37:1;
                vuint32_t CH36:1;
                vuint32_t CH35:1;
                vuint32_t CH34:1;
                vuint32_t CH33:1;
                vuint32_t CH32:1;
            } B;
        } NCMR1;            /* EXTENDED CHANNELS NORMAL CONVERSION MASK REGISTER 1 */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t CH95:1;
                vuint32_t CH94:1;
                vuint32_t CH93:1;
                vuint32_t CH92:1;
                vuint32_t CH91:1;
                vuint32_t CH90:1;
                vuint32_t CH89:1;
                vuint32_t CH88:1;
                vuint32_t CH87:1;
                vuint32_t CH86:1;
                vuint32_t CH85:1;
                vuint32_t CH84:1;
                vuint32_t CH83:1;
                vuint32_t CH82:1;
                vuint32_t CH81:1;
                vuint32_t CH80:1;
                vuint32_t CH79:1;
                vuint32_t CH78:1;
                vuint32_t CH77:1;
                vuint32_t CH76:1;
                vuint32_t CH75:1;
                vuint32_t CH74:1;
                vuint32_t CH73:1;
                vuint32_t CH72:1;
                vuint32_t CH71:1;
                vuint32_t CH70:1;
                vuint32_t CH69:1;
                vuint32_t CH68:1;
                vuint32_t CH67:1;
                vuint32_t CH66:1;
                vuint32_t CH65:1;
                vuint32_t CH64:1;
            } B;
        } NCMR2;            /* EXTERNAL CHANNELS NORMAL CONVERSION MASK REGISTER 1 */
	
        union {
            vuint32_t R;
            struct {
                vuint32_t:32;
            } B;
        } ADC1_reserved6;
              
        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
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
        } JCMR0;            /* PRECISE CHANNELS INJECTED CONVERSION MASK REGISTER 0 */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t CH63:1;
                vuint32_t CH62:1;
                vuint32_t CH61:1;
                vuint32_t CH60:1;
                vuint32_t CH59:1;
                vuint32_t CH58:1;
                vuint32_t CH57:1;
                vuint32_t CH56:1;
                vuint32_t CH55:1;
                vuint32_t CH54:1;
                vuint32_t CH53:1;
                vuint32_t CH52:1;
                vuint32_t CH51:1;
                vuint32_t CH50:1;
                vuint32_t CH49:1;
                vuint32_t CH48:1;
                vuint32_t CH47:1;
                vuint32_t CH46:1;
                vuint32_t CH45:1;
                vuint32_t CH44:1;
                vuint32_t CH43:1;
                vuint32_t CH42:1;
                vuint32_t CH41:1;
                vuint32_t CH40:1;
                vuint32_t CH39:1;
                vuint32_t CH38:1;
                vuint32_t CH37:1;
                vuint32_t CH36:1;
                vuint32_t CH35:1;
                vuint32_t CH34:1;
                vuint32_t CH33:1;
                vuint32_t CH32:1;
            } B;
        } JCMR1;            /* EXTENDED CHANNELS INJECTED CONVERSION MASK REGISTER 1 */
        
        union {
            vuint32_t R;
            struct {
                vuint32_t CH95:1;
                vuint32_t CH94:1;
                vuint32_t CH93:1;
                vuint32_t CH92:1;
                vuint32_t CH91:1;
                vuint32_t CH90:1;
                vuint32_t CH89:1;
                vuint32_t CH88:1;
                vuint32_t CH87:1;
                vuint32_t CH86:1;
                vuint32_t CH85:1;
                vuint32_t CH84:1;
                vuint32_t CH83:1;
                vuint32_t CH82:1;
                vuint32_t CH81:1;
                vuint32_t CH80:1;
                vuint32_t CH79:1;
                vuint32_t CH78:1;
                vuint32_t CH77:1;
                vuint32_t CH76:1;
                vuint32_t CH75:1;
                vuint32_t CH74:1;
                vuint32_t CH73:1;
                vuint32_t CH72:1;
                vuint32_t CH71:1;
                vuint32_t CH70:1;
                vuint32_t CH69:1;
                vuint32_t CH68:1;
                vuint32_t CH67:1;
                vuint32_t CH66:1;
                vuint32_t CH65:1;
                vuint32_t CH64:1;
            } B;
        } JCMR2;            /* EXTERNAL CHANNELS INJECTED CONVERSION MASK REGISTER 2 */

         int32_t ADC1_reserved18[1];
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t DSD:8;
            } B;
        } DSDR;            /* DECODE SIGNALS DELAY REGISTER */                
        
        union {
            vuint32_t R;
            struct {
                vuint32_t:24;                
                vuint32_t PDED:8;
            } B;
        } PDEDR;            /* POWER DOWN EXIT DELAY REGISTER */                
       
        
        int32_t ADC1_reserved7[13];     /* {0x100-0x0F0}/0x4 = 4 */
                
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
        } CDR[95];            /* CHANNEL x DATA REGISTER */
        
        
        
        int32_t ADC1_reserved8[13]; 
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH7:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH6:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH5:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH4:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH3:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH2:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH1:3;
                vuint32_t:1;
                vuint32_t WSEL_CH0:3;
            } B;
        } CWSELR0; /* CHANNEL WATCHDOG SELECTION REGISTERS (PRECISE CHANNELS) */   
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH15:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH14:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH13:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH12:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH11:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH10:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH9:3;
                vuint32_t:1;
                vuint32_t WSEL_CH8:3;
            } B;
        } CWSELR1; /* CHANNEL WATCHDOG SELECTION REGISTERS (PRECISE CHANNELS) */    

        int32_t ADC1_reserved19[2];

        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH39:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH38:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH37:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH36:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH35:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH34:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH33:3;
                vuint32_t:1;
                vuint32_t WSEL_CH32:3;
            } B;
        } CWSELR4; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTENDED CHANNELS) */    
        
        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH47:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH46:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH45:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH44:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH43:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH42:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH41:3;
                vuint32_t:1;
                vuint32_t WSEL_CH40:3;
            } B;
        } CWSELR5; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTENDED CHANNELS) */    

        int32_t ADC1_reserved20[2];

        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH71:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH70:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH69:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH68:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH67:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH66:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH65:3;
                vuint32_t:1;
                vuint32_t WSEL_CH64:3;
            } B;
        } CWSELR8; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTERNAL CHANNELS) */    

        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH79:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH78:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH77:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH76:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH75:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH74:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH73:3;
                vuint32_t:1;
                vuint32_t WSEL_CH72:3;
            } B;
        } CWSELR9; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTERNAL CHANNELS) */    

        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH87:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH86:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH85:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH84:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH83:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH82:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH81:3;
                vuint32_t:1;
                vuint32_t WSEL_CH80:3;
            } B;
        } CWSELR10; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTERNAL CHANNELS) */    

        union {
            vuint32_t R;
            struct {
	            vuint32_t:1;
                vuint32_t WSEL_CH95:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH94:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH93:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH92:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH91:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH90:3;
	            vuint32_t:1;
                vuint32_t WSEL_CH89:3;
                vuint32_t:1;
                vuint32_t WSEL_CH88:3;
            } B;
        } CWSELR11; /* CHANNEL WATCHDOG SELECTION REGISTERS (EXTERNAL CHANNELS) */    
               
        union {
            vuint32_t R;
            struct {
                vuint32_t CWEN31:1;
	            vuint32_t CWEN30:1;
                vuint32_t CWEN29:1;
                vuint32_t CWEN28:1;
                vuint32_t CWEN27:1;
                vuint32_t CWEN26:1;
                vuint32_t CWEN25:1;
                vuint32_t CWEN24:1;
                vuint32_t CWEN23:1;
                vuint32_t CWEN22:1;
                vuint32_t CWEN21:1;
                vuint32_t CWEN20:1;
                vuint32_t CWEN19:1;
                vuint32_t CWEN18:1;
                vuint32_t CWEN17:1;
                vuint32_t CWEN16:1;
                vuint32_t CWEN15:1;
                vuint32_t CWEN14:1;
                vuint32_t CWEN13:1;
                vuint32_t CWEN12:1;
                vuint32_t CWEN11:1;
                vuint32_t CWEN10:1;
                vuint32_t CWEN9:1;
                vuint32_t CWEN8:1;
                vuint32_t CWEN7:1;
                vuint32_t CWEN6:1;
                vuint32_t CWEN5:1;
                vuint32_t CWEN4:1;
                vuint32_t CWEN3:1;
                vuint32_t CWEN2:1;
                vuint32_t CWEN1:1;
                vuint32_t CWEN0:1; 
            } B;
        } CWENR[3]; /* CHANNEL WATCHDOG ENABLE REGISTERS 0-2 */
        
        int32_t ADC1_reserved9[1];          
        
        union {
            vuint32_t R;
            struct {
                vuint32_t AWORR_CH31:1;
	            vuint32_t AWORR_CH30:1;
                vuint32_t AWORR_CH29:1;
                vuint32_t AWORR_CH28:1;
                vuint32_t AWORR_CH27:1;
                vuint32_t AWORR_CH26:1;
                vuint32_t AWORR_CH25:1;
                vuint32_t AWORR_CH24:1;
                vuint32_t AWORR_CH23:1;
                vuint32_t AWORR_CH22:1;
                vuint32_t AWORR_CH21:1;
                vuint32_t AWORR_CH20:1;
                vuint32_t AWORR_CH19:1;
                vuint32_t AWORR_CH18:1;
                vuint32_t AWORR_CH17:1;
                vuint32_t AWORR_CH16:1;
                vuint32_t AWORR_CH15:1;
                vuint32_t AWORR_CH14:1;
                vuint32_t AWORR_CH13:1;
                vuint32_t AWORR_CH12:1;
                vuint32_t AWORR_CH11:1;
                vuint32_t AWORR_CH10:1;
                vuint32_t AWORR_CH9:1;
                vuint32_t AWORR_CH8:1;
                vuint32_t AWORR_CH7:1;
                vuint32_t AWORR_CH6:1;
                vuint32_t AWORR_CH5:1;
                vuint32_t AWORR_CH4:1;
                vuint32_t AWORR_CH3:1;
                vuint32_t AWORR_CH2:1;
                vuint32_t AWORR_CH1:1;
                vuint32_t AWORR_CH0:1; 
            } B;
        } AWORR[3];            /* ANALOG WATCHDOG OUT OF RANGE REGISTERS */
        
    };                          /* end of ADC1_tag */

 
    
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
/*                          MODULE : MCM                                   */
/****************************************************************************/
    struct ECSM_tag {

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
                vuint16_t APC:1;       //modified by safdar
                vuint16_t APC0:1;       //added by safdar
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
        } PCR[149];

	int16_t SIU_reserved12[363];
        int32_t SIU_reserved4[48];      /* {0x500-0x440}/0x4 */

        union {                 /* Pad Selection for Multiplexed Input Register */
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t PADSEL:4;
            } B;
        } PSMI[64];

        int32_t SIU_reserved5[48];      /* {0x500-0x440}/0x4 */

        union {                 /* GPIO Pin Data Output Registers */
            vuint8_t R;
            struct {
                vuint8_t:7;
                vuint8_t PDO:1;
            } B;
        } GPDO[152];

        int32_t SIU_reserved6[90];      /* {0x500-0x440}/0x4 */

        union {                 /* GPIO Pin Data Input Registers */
            vuint8_t R;
            struct {
                vuint8_t:7;
                vuint8_t PDI:1;
            } B;
        } GPDI[152];
	int32_t SIU_reserved13[128];
        int32_t SIU_reserved7[90];     /* {0xC00-0xA00}/0x4 */

        union {                 /* Parallel GPIO Pin Data Output Register */
            vuint32_t R;
            struct {
                vuint32_t PPD0:32;
            } B;
        } PGPDO[5];

        int32_t SIU_reserved8[11];     /* {0xC00-0xA00}/0x4 */

        union {                 /* Parallel GPIO Pin Data Input Register */
            vuint32_t R;
            struct {
                vuint32_t PPDI:32;
            } B;
        } PGPDI[5];

       int32_t SIU_reserved9[11];     /* {0xC00-0xA00}/0x4 */


        union {                 /* Masked Parallel GPIO Pin Data Out Register */
            vuint32_t R;
            struct {
                vuint32_t MASK:16;
                vuint32_t MPPDO:16;
            } B;
        } MPGPDO[10];

        int32_t SIU_reserved10[214];     /* {0x1000-0x0D00}/0x4 */

        union {                 /* Interrupt Filter Maximum Counter Register */
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t MAXCNT:4;
            } B;
        } IFMC[24];

        int32_t SIU_reserved11[8];     /* {0x1000-0x0D00}/0x4 */

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
                  vuint16_t:1;
                vuint16_t SEC:1;
                  vuint16_t:1;
                vuint16_t BMODE:3;
                vuint16_t DMID:1;
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
                vuint16_t SLFM:1;
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
                vuint16_t SDEF:1;
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
                vuint16_t RFBM:1;
                vuint16_t TFBM:1;
                vuint16_t WL1:1;
                vuint16_t OP1:1;
                vuint16_t RXEN:1;
                vuint16_t TXEN:1;
                vuint16_t OP0:1; //LCH vuint16_t PARITYODD:1;
                vuint16_t PCE:1;
                vuint16_t WL0:1;
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
        } IFCR[16];  
        
        int32_t LINFLEX_reserved18;//GCR egister commented
        
                       
        int16_t LINFLEX_reserved19;//UARTPTO upper 16 bits reserved
        
         union {
            vuint16_t R;
            struct {
                vuint16_t:4;
            	vuint16_t PTO:12;
                } B;
        }UARTPTO;
        
        int32_t LINFLEX_reserved20;//UARTCTO egister commented
        
        int16_t LINFLEX_reserved21;
        
        
        union {
            vuint16_t R;
            struct {
                vuint16_t DTE15:1;
                vuint16_t DTE14:1;
                vuint16_t DTE13:1;
                vuint16_t DTE12:1;
                vuint16_t DTE11:1;
				vuint16_t DTE10:1;
				vuint16_t DTE9:1;
				vuint16_t DTE8:1;
				vuint16_t DTE7:1;
				vuint16_t DTE6:1;
				vuint16_t DTE5:1;
				vuint16_t DTE4:1;
				vuint16_t DTE3:1;
				vuint16_t DTE2:1;
				vuint16_t DTE1:1;
				vuint16_t DTE0:1;
                } B;
        } DMATXE;
        int16_t LINFLEX_reserved22;

        union {
            vuint16_t R;
            struct {
                vuint16_t DRE15:1;
                vuint16_t DRE14:1;
                vuint16_t DRE13:1;
                vuint16_t DRE12:1;
                vuint16_t DRE11:1;
				vuint16_t DRE10:1;
				vuint16_t DRE9:1;
				vuint16_t DRE8:1;
				vuint16_t DRE7:1;
				vuint16_t DRE6:1;
				vuint16_t DRE5:1;
				vuint16_t DRE4:1;
				vuint16_t DRE3:1;
				vuint16_t DRE2:1;
				vuint16_t DRE1:1;
				vuint16_t DRE0:1;

                } B;
        } DMARXE; 
        
        
        
    };                          /* end of LINFLEX_tag */
        
/****************************************************************************/
/*                          MODULE : ME                                   */
/****************************************************************************/
struct ME_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t CURRENTMODE:4;
                vuint32_t MTRANS:1;
                vuint32_t DC:1;
                  vuint32_t:2;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVR:1;
                vuint32_t DFLA:2;
                vuint32_t CFLA:2;
                vuint32_t SSCLK:9;
                vuint32_t PLL:1;
                vuint32_t OSC:1;
                vuint32_t RC:1;
                vuint32_t SYSCLK:4;
            } B;
        } GS;                   /* Global Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t TARGETMODE:4;
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
                vuint32_t ICONF:1;
                vuint32_t IMODE:1;
                vuint32_t SAFE:1;
                vuint32_t MTC:1;
            } B;
        } IS;                   /* Interrupt Status Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t ICONF:1;
                vuint32_t IMODE:1;
                vuint32_t SAFE:1;
                vuint32_t MTC:1;
            } B;
        } IM;                   /* Interrupt Mask Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:27;
                vuint32_t MTI:1;
                vuint32_t MRI:1;
                vuint32_t DMA:1;
                vuint32_t NMA:1;
                vuint32_t SEA:1;
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
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
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
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
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
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
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
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
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
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
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
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
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
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
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
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } STANDBY0;             /* STANDBY0 Mode Configuration Register */

        int32_t ME_reserved3[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t PERIPH:32;
            } B;
        } PS[5];                /* Peripheral Status 0->4 Register */

        int32_t ME_reserved4[3];

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
                vuint8_t DBGP:1;
                vuint8_t DBGF:1;
                vuint8_t LPCFG:1;
                vuint8_t RUNCFG:1;
            } B;
        } PCTL[144];            /* Peripheral Control 0->143 Register */

    /************************************/
        /* Register Protection              */
    /************************************/
        int32_t ME_reserved5[1964];     /* {0x2000-0x0150}/0x4 = 0x7AC */

        union {
            vuint32_t R;
            struct {
                vuint32_t CURRENTMODE:4;
                vuint32_t MTRANS:1;
                vuint32_t DC:1;
                  vuint32_t:2;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVR:1;
                vuint32_t DFLA:2;
                vuint32_t CFLA:2;
                vuint32_t SSCLK:9;
                vuint32_t PLL:1;
                vuint32_t OSC:1;
                vuint32_t RC:1;
                vuint32_t SYSCLK:4;
            } B;
        } GS_LOCK;              /* Global Status Register Lock */

        union {
            vuint32_t R;
            struct {
                vuint32_t TARGETMODE:4;
                  vuint32_t:12;
                vuint32_t KEY:16;
            } B;
        } MCTL_LOCK;            /* Mode Control Register Lock */

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
        } ME_LOCK;              /* Mode Enable Register Lock */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t ICONF:1;
                vuint32_t IMODE:1;
                vuint32_t SAFE:1;
                vuint32_t MTC:1;
            } B;
        } IS_LOCK;              /* Interrupt Status Register Lock */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t ICONF:1;
                vuint32_t IMODE:1;
                vuint32_t SAFE:1;
                vuint32_t MTC:1;
            } B;
        } IM_LOCK;              /* Interrupt Mask Register Lock */

        union {
            vuint32_t R;
            struct {
                vuint32_t:27;
                vuint32_t MTI:1;
                vuint32_t MRI:1;
                vuint32_t DMA:1;
                vuint32_t NMA:1;
                vuint32_t SEA:1;
            } B;
        } IMTS_LOCK;            /* Invalid Mode Transition Status Register Lock */

        int32_t ME_reserved6[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } RESET_LOCK;           /* Reset Mode Configuration Register Lock */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } TEST_LOCK;            /* Test Mode Configuration Register Lock */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } SAFE_LOCK;            /* Safe Mode Configuration Register Lock */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } DRUN_LOCK;            /* DRUN Mode Configuration Register Lock */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } RUN_LOCK[4];          /* RUN 0->4 Mode Configuration Register Lock */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } HALT0_LOCK;           /* HALT0 Mode Configuration Register Lock */

        int32_t ME_reserved7;

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } STOP0_LOCK;           /* STOP0 Mode Configuration Register Lock */

        int32_t ME_reserved8[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t PDO:1;
                  vuint32_t:2;
                vuint32_t MVRON:1;
                vuint32_t DFLAON:2;
                vuint32_t CFLAON:2;
                vuint32_t SSCLKON:9;
                vuint32_t PLLON:1;
                vuint32_t OSCON:1;
                vuint32_t RCON:1;
                vuint32_t SYSCLK:4;
            } B;
        } STANDBY0_LOCK;        /* STANDBY0 Mode Configuration Register Lock */

        int32_t ME_reserved9[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t PERIPH:32;
            } B;
        } PS_LOCK[5];           /* Peripheral Status 0->4 Register Lock */

        int32_t ME_reserved10[3];

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
        } RUNPC_LOCK[8];        /* RUN Peripheral Configuration 0->7 Register Lock */

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
        } LPPC_LOCK[8];         /* Low Power Peripheral Configuration 0->7 Register Lock */

        union {
            vuint8_t R;
            struct {
                vuint8_t DBGP:1;
                vuint8_t DBGF:1;
                vuint8_t LPCFG:1;
                vuint8_t RUNCFG:1;
            } B;
        } PCTL_LOCK[144];       /* Peripheral Control 0->143 Register Lock */

        int32_t ME_reserved11[1452];    /* {0x3800-0x2150}/0x4 = 0x5AC */

        union {                 /* Soft Lock Bit Register */
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t SLB0:4;
                  vuint32_t:4;
                vuint32_t SLB1:4;
                  vuint32_t:4;
                vuint32_t SLB2:4;
                  vuint32_t:4;
                vuint32_t SLB3:4;
            } B;
        } SLBR[384];

        int32_t ME_reserved12[127];     /* {0x3FFC-0x3E00}/0x4 = 0x07F */

        union {                 /* Global Configuration Register */
            vuint32_t R;
            struct {
                vuint32_t HLB:1;
                  vuint32_t:7;
                vuint32_t SOB:1;
                  vuint32_t:23;
            } B;
        } GCR;

    };                          /* end of ME_tag */

    
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
                vuint16_t:7;
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
                vuint16_t:7;
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
/*                          MODULE : CTUL                                   */
/****************************************************************************/
    struct CTUL_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                  vuint32_t:8;
                vuint32_t TRGIEN:1;
                vuint32_t TRGI:1;
                  vuint32_t:6;
            } B;
        } CSR;                  /* Control Status Register */

        int32_t CTU_reserved0[11];

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t TM:1;
                vuint32_t CLR_FLAG:1;
                vuint32_t:5;
                vuint32_t ADC_SEL:1;
                vuint32_t:1;
                vuint32_t CHANNELVALUE:7;
            } B;
        } EVTCFGR[64];          /* Event Configuration Register */

    };                          /* end of CTUL_tag */

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
                  vuint32_t:3;
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
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t ALTA:16;
            } B;		   
        } ALTCADR;   /* Alternate Channel A Data Register */

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
                  vuint32_t:1;
                vuint32_t GPREN:1;
                  vuint32_t:10;
                vuint32_t GPRE:8;
                  vuint32_t:8;
            } B;
        } MCR;                  /* Module Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t F31:1;				
                vuint32_t F30:1;
                vuint32_t F29:1;
                vuint32_t F28:1;
                vuint32_t F27:1;
                vuint32_t F26:1;
                vuint32_t F25:1;
                vuint32_t F24:1;				
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
                vuint32_t OU31:1;
                vuint32_t OU30:1;
                vuint32_t OU29:1;
                vuint32_t OU28:1;
                vuint32_t OU27:1;
                vuint32_t OU26:1;
                vuint32_t OU25:1;
                vuint32_t OU24:1;				
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
                vuint32_t CHDIS31:1;
                vuint32_t CHDIS30:1;
                vuint32_t CHDIS29:1;
                vuint32_t CHDIS28:1;
                vuint32_t CHDIS27:1;
                vuint32_t CHDIS26:1;
                vuint32_t CHDIS25:1;
                vuint32_t CHDIS24:1;				
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

        struct EMIOS_CHANNEL_tag CH[32];

    };                          /* end of EMIOS_tag */
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
        } CH[8];

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
                vuint32_t MPERR:8;
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

/*for standard format TCD (when EDMA.TCD[x].CITER.E_LINK==BITER.E_LINK=0) */
    struct EDMA_TCD_STD_tag {

        vuint32_t SADDR;        /* source address */

        vuint16_t SMOD:5;       /* source address modulo */
        vuint16_t SSIZE:3;      /* source transfer size */
        vuint16_t DMOD:5;       /* destination address modulo */
        vuint16_t DSIZE:3;      /* destination transfer size */
        vint16_t SOFF;          /* signed source address offset */

		union {
            vuint32_t R;
            struct {
				vuint32_t SMLOE:1;
				vuint32_t DMLOE:1;
				int32_t	  MLOFF:20;
				vuint32_t NBYTES:10;
            } B;
        } NBYTESu;           /* Region Descriptor Alternate Access Control n */
	
        vint32_t SLAST;         /* last destination address adjustment, or
                                   scatter/gather address (if e_sg = 1) */

        vuint32_t DADDR;        /* destination address */

        vuint16_t CITERE_LINK:1;
        vuint16_t CITER:15;

        vint16_t DOFF;          /* signed destination address offset */

        vint32_t DLAST_SGA;

        vuint16_t BITERE_LINK:1;        /* beginning major iteration count */
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

/*for channel link format TCD (when EDMA.TCD[x].CITER.E_LINK==BITER.E_LINK=1)*/
    struct EDMA_TCD_CHLINK_tag {

        vuint32_t SADDR;        /* source address */

        vuint16_t SMOD:5;       /* source address modulo */
        vuint16_t SSIZE:3;      /* source transfer size */
        vuint16_t DMOD:5;       /* destination address modulo */
        vuint16_t DSIZE:3;      /* destination transfer size */
        vint16_t SOFF;          /* signed source address offset */

		union {
            vuint32_t R;
            struct {
				vuint32_t SMLOE:1;
				vuint32_t DMLOE:1;
				int32_t	  MLOFF:20;
				vuint32_t NBYTES:10;
            } B;
        } NBYTESu;           /* Region Descriptor Alternate Access Control n */
        vint32_t SLAST;         /* last destination address adjustment, or
                                   scatter/gather address (if e_sg = 1) */

        vuint32_t DADDR;        /* destination address */

        vuint16_t CITERE_LINK:1;
        vuint16_t CITERLINKCH:6;
        vuint16_t CITER:9;

        vint16_t DOFF;          /* signed destination address offset */

        vint32_t DLAST_SGA;

        vuint16_t BITERE_LINK:1;        /* beginning major iteration count */
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
                vuint32_t:14;
                vuint32_t CX:1;
                vuint32_t ECX:1;
                vuint32_t GRP3PRI:2;
                vuint32_t GRP2PRI:2;
                vuint32_t GRP1PRI:2;
                vuint32_t GRP0PRI:2;
                vuint32_t EMLM:1;
                vuint32_t CLM:1;
                vuint32_t HALT:1;
                vuint32_t HOE:1;
                vuint32_t ERGA:1;
                vuint32_t ERCA:1;
                vuint32_t EDBG:1;
                vuint32_t EBW:1;
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
        } IFRL;                 /* Interrupt Flag Register */

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
                  vuint32_t:25;
                vuint32_t HBLOCK:6;
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
                  vuint32_t:19;
                vuint32_t HBLOCK:12;
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
                vuint32_t:20;
                vuint32_t HSL:12;
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
/****************************************************************** 
| defines and macros (scope: module-local) 
|-----------------------------------------------------------------*/
/* Define instances of modules */

#define ADC_0     (*(volatile struct ADC0_tag *)      0xFFE00000UL)
#define ADC_1     (*(volatile struct ADC1_tag *)      0xFFE04000UL)
#define CAN_0     (*(volatile struct FLEXCAN_tag *)   0xFFFC0000UL)
#define CAN_1     (*(volatile struct FLEXCAN_tag *)   0xFFFC4000UL)
#define CAN_2     (*(volatile struct FLEXCAN_tag *)   0xFFFC8000UL)
#define CAN_3     (*(volatile struct FLEXCAN_tag *)   0xFFFCC000UL)
#define CAN_4     (*(volatile struct FLEXCAN_tag *)   0xFFFD0000UL)
#define CAN_5     (*(volatile struct FLEXCAN_tag *)   0xFFFD4000UL)
#define CANSP     (*(volatile struct CANSP_tag *)     0xFFE70000UL)
#define CFLASH    (*(volatile struct CFLASH_tag *)    0xC3F88000UL)
#define CGM       (*(volatile struct CGM_tag *)       0xC3FE0000UL)
#define CTUL      (*(volatile struct CTUL_tag *)      0xFFE64000UL)
#define DFLASH    (*(volatile struct DFLASH_tag *)    0xC3F8C000UL)
#define DMAMUX    (*(volatile struct DMAMUX_tag *)    0xFFFDC000UL)
#define DSPI_0    (*(volatile struct DSPI_tag *)      0xFFF90000UL)
#define DSPI_1    (*(volatile struct DSPI_tag *)      0xFFF94000UL)
#define DSPI_2    (*(volatile struct DSPI_tag *)      0xFFF98000UL)
#define DSPI_3    (*(volatile struct DSPI_tag *)      0xFFF9C000UL)
#define DSPI_4    (*(volatile struct DSPI_tag *)      0xFFFA0000UL)
#define DSPI_5    (*(volatile struct DSPI_tag *)      0xFFFA4000UL)
#define EDMA      (*(volatile struct EDMA_tag *)      0xFFF44000UL)
#define EMIOS_0   (*(volatile struct EMIOS_tag *)     0xC3FA0000UL)
#define EMIOS_1   (*(volatile struct EMIOS_tag *)     0xC3FA4000UL)
#define I2C_0     (*(volatile struct I2C_tag *)       0xFFE30000UL)
#define INTC      (*(volatile struct INTC_tag *)      0xFFF48000UL)
#define LINFLEX_0 (*(volatile struct LINFLEX_tag *)   0xFFE40000UL)
#define LINFLEX_1 (*(volatile struct LINFLEX_tag *)   0xFFE44000UL)
#define LINFLEX_2 (*(volatile struct LINFLEX_tag *)   0xFFE48000UL)
#define LINFLEX_3 (*(volatile struct LINFLEX_tag *)   0xFFE4C000UL)
#define LINFLEX_4 (*(volatile struct LINFLEX_tag *)   0xFFE50000UL)
#define LINFLEX_5 (*(volatile struct LINFLEX_tag *)   0xFFE54000UL)
#define LINFLEX_6 (*(volatile struct LINFLEX_tag *)   0xFFE58000UL)
#define LINFLEX_7 (*(volatile struct LINFLEX_tag *)   0xFFE5C000UL)
#define LINFLEX_8 (*(volatile struct LINFLEX_tag *)   0xFFFB0000UL)
#define LINFLEX_9 (*(volatile struct LINFLEX_tag *)   0xFFFB4000UL)
#define ECSM      (*(volatile struct ECSM_tag *)      0xFFF40000UL)
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
#endif

#ifdef  __cplusplus
}
#endif
#endif                          /* ifdef _JDP_H */
/* End of file */
