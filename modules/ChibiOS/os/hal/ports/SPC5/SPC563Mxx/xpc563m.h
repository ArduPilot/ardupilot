/**************************************************************************/ 
    
/* FILE NAME: mpc563xm.h                COPYRIGHT (c) Freescale 2008,2009 */ 
/* VERSION:  2.0                                  All Rights Reserved     */ 
/*                                                                        */ 
/* DESCRIPTION:                                                           */ 
/* This file contain all of the register and bit field definitions for    */ 
/* MPC563xM. This version supports revision 1.0 and later.                */ 
/*========================================================================*/ 
/* UPDATE HISTORY                                                         */ 
/* REV      AUTHOR      DATE       DESCRIPTION OF CHANGE                  */ 
/* ---   -----------  ---------    ---------------------                  */ 
/* 1.0   G. Emerson   31/OCT/07    Initial version.                       */ 
/* 1.1   G. Emerson   20/DEC/07    Added SYSDIV HLT HLTACK                */ 
/*                                 Added ESYNCR1 ESYNCR2 SYNFMMR          */ 
/* 1.2   G. Emerson   31/JAN/08    Change eMIOS channels so there are 24. */ 
/*                                 8 channels in the middle of the range  */ 
/*                                 do not exist                           */ 
/* 1.3   G. Emerson   30/JUL/08    FLEXCAN - Supports FIFO and Buffer.    */ 
/*                                 RXIMR added                            */ 
/*                                 FMPLL   - Added FMPLL.SYNFMMR.B.BSY    */ 
/*                                 SIU     - Added SIU.ISEL0-3            */ 
/*                                 EMIOS   - Added EMIOS.CH[x].ALTCADR.R  */ 
/*                                 MCM     - Replaced ECSM with MCM       */ 
/*                                 removing SWT registers as defined at   */ 
/*                                 seperate memory location. PFLASH       */ 
/*                                 registers pre-fixed with P*. Added PCT,*/ 
/*                                 PLREV, PLAMC, PLASC, IOPMC, MRSR, MWCR.*/ 
/*                                 PBRIDGE - Removed as no PBRIDGE        */ 
/*                                 registers.                             */ 
/*                                 INTC    - Updated number of PSR from   */ 
/*                                 358 to 360.                            */ 
/*                                 mpc5500_spr.h - Added RI to MSR and NMI*/ 
/*                                 to MSCR.                               */ 
/* 1.4   G. Emerson   30/SEP/08    Add SIU.MIDR2                          */ 
/*                                 Changes to SIU.MIDR as per RM.         */ 
/* 1.5                May 2009     Changes to match documentation, removed*/ 
/*                                 Not released                           */ 
/* 1.6   K. Odenthal  03/June/09   Update for 1.5M version of the MPC563xM*/ 
/*       & R. Dees                                                        */ 
/*                      INTC - All Processor 0 regs matched to previous   */ 
/*                             version                                    */ 
/*                      INTC - BCR renamed to MCR to match previous       */ 
/*                             version                                    */ 
/*                      INTC - VTES_PRC1 and HVEN_PRC1 added to MCR       */ 
/*                      INTC - CPR_PRC1, IACKR_PRC1 and EOIR_PRC1         */ 
/*                             registers added                            */ 
/*                      INTC - 512 PSR registers instead of 364           */ 
/*                      ECSM - (Internal - mcm -> ecsm in the source files*/ 
/*                              for generating the header file            */ 
/*                      ECSM - All bits and regs got an additional "p" in */ 
/*                             the name in the user manual for "Platform" */ 
/*                             -> deleted to match                        */ 
/*                      ECSM - SWTCR, SWTSR and SWTIR don't exist in      */ 
/*                             MPC563xM -> deleted                        */ 
/*                      ECSM - PROTECTION in the URM is one bitfield,     */ 
/*                             in mop5534 this are four: PROT1-4 ->       */ 
/*                             changed to match                           */ 
/*                      EMCM - removed undocumented registers             */ 
/*                      ECSM - RAM ECC Syndrome is new in MPC563xM -> added */ 
/*                      XBAR - removed AMPR and ASGPCR registers          */ 
/*                      XBAR - removed HPE bits for nonexistant masters   */ 
/*                      EBI - added: D16_31, AD_MUX and SETA bits         */ 
/*                      EBI - Added reserved register at address 0x4.     */ 
/*                      EBI - Corrected number of chip selects in for both*/ 
/*                            the EBI_CS and the CAL_EBI_CS               */ 
/*                      SIU - corrected number of GPDO registers and      */ 
/*                            allowed for maximum PCR registers.          */ 
/*                      SWT - add KEY bit to CR, correct WND (from WNO)   */ 
/*                      SWT - add SK register                             */ 
/*                      PMC - moved bits from CFGR to Status Register (SR)*/ 
/*                      PMC - Added SR                                    */ 
/*                      DECFIL - Added new bits DSEL, IBIE, OBIE, EDME,   */ 
/*                            TORE, & TRFE to MCR. Added IBIC, OBIC,      */ 
/*                            DIVRC, IBIF, OBIF, DIVR to MSR.             */ 
/*                            changed OUTTEG to OUTTAG in OB              */ 
/*                            Change COEF to TAG in TAG register          */ 
/*                      EQADC - removed REDLCCR - not supported           */ 
/*                      FLASH - Aligned register and bit names with legacy*/ 
/* 1.7   K. Odenthal  10/November/09                                      */ 
/*                      SIU    - changed PCR[n].PA from 3 bit to 4 bit    */ 
/*                      eTPU   - changed WDTR_A.WDM from 1 bit to 2 bits  */ 
/*                      DECFIL - changed COEF.R and TAP.R from 16 bit to  */ 
/*                               32 bit                                   */ 
/* 2.0   K. Odenthal  12/February/2010                                    */ 
/*                      TSENS  - Temperature Sensor Module added to       */ 
/*                               header file                              */ 
/*                      ANSI C Compliance - Register structures have a    */ 
/*                               Bitfield Tag ('B') tag only if there is  */ 
/*                               at least one Bitfiels defined. Empty     */ 
/*                               tags like 'vuint32_t:32;' are not        */ 
/*                               allowed.                                 */ 
/*                      DECFIL - removed MXCR register. This register is  */ 
/*                               not supported on this part               */ 
/*                      SIU    - SWT_SEL bit added in SIU DIRER register  */ 
/*                      EDMA   - removed HRSL, HRSH and GPOR registers.   */ 
/*                               Those registers are not supported in     */ 
/*                               that part.                               */ 
/*                      ESCI   - removed LDBG and DSF bits from LCR       */ 
/*                               registers. Those bits are not supported  */ 
/*                               in that part.                            */ 
/*                               Those registers are not supported in     */ 
/*                               that part.                               */ 
/**************************************************************************/ 
/*>>>>NOTE! this file is auto-generated please do not edit it!<<<<*/ 
    
#ifndef _MPC563M_H_
#define _MPC563M_H_
    
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
/*                     MODULE : FMPLL                                       */
/****************************************************************************/
        struct FMPLL_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:1;
                vuint32_t PREDIV:3;
                vuint32_t MFD:5;
                  vuint32_t:1;
                vuint32_t RFD:3;
                vuint32_t LOCEN:1;
                vuint32_t LOLRE:1;
                vuint32_t LOCRE:1;
                  vuint32_t:1;  /* Reserved in MPC563xM 

                                   Deleted for legacy header version [mpc5534.h]: 

                                   <vuint32_t DISCLK:1> */
                vuint32_t LOLIRQ:1;
                vuint32_t LOCIRQ:1;
                  vuint32_t:13; /* Reserved in MPC563xM 

                                   Deleted for legacy header version [mpc5534.h]:

                                   <vuint32_t RATE:1 > 

                                   <vuint32_t DEPTH:2>

                                   <vuint32_t EXP:10 > */
            } B;
        } SYNCR;
        union {
            vuint32_t R;
            struct {
                vuint32_t:22;
                vuint32_t LOLF:1;
                vuint32_t LOC:1;
                vuint32_t MODE:1;
                vuint32_t PLLSEL:1;
                vuint32_t PLLREF:1;
                vuint32_t LOCKS:1;
                vuint32_t LOCK:1;
                vuint32_t LOCF:1;
                  vuint32_t:2;  /* Reserved in MPC563xM

                                   Deleted for legacy header version [mpc5534.h]:

                                   <vuint32_t CALDONE:1>

                                   <vuint32_t CALPASS:1> */
            } B;
        } SYNSR;
        union {
            vuint32_t R;
            struct {
                vuint32_t EMODE:1;
                vuint32_t CLKCFG:3;
                  vuint32_t:8;
                vuint32_t EPREDIV:4;
                  vuint32_t:9;
                vuint32_t EMFD:7;
            } B;
        } ESYNCR1;              /* Enhanced Synthesizer Control Register 1 (ESYNCR1) (new in MPC563xM) Offset 0x0008 */
        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t LOCEN:1;
                vuint32_t LOLRE:1;
                vuint32_t LOCRE:1;
                vuint32_t LOLIRQ:1;
                vuint32_t LOCIRQ:1;
                  vuint32_t:17;
                vuint32_t ERFD:2;
            } B;
        } ESYNCR2;              /* Enhanced Synthesizer Control Register 2 (ESYNCR2) (new in MPC563xM) Offset 0x000C */
        int32_t FMPLL_reserved0[2];
        union {
            vuint32_t R;
            struct {
                vuint32_t BSY:1;
                vuint32_t MODEN:1;
                vuint32_t MODSEL:1;
                vuint32_t MODPERIOD:13;
                  vuint32_t:1;
                vuint32_t INCSTEP:15;
            } B;
        } SYNFMMR;              /* Synthesizer FM Modulation Register (SYNFMMR) (new in MPC563xM) Offset 0x0018 */
    };
/****************************************************************************/
/*                     MODULE : EBI                                      */
/****************************************************************************/
    struct CS_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t BA:17;        /*  */
                  vuint32_t:3;  /*  */
                vuint32_t PS:1; /*  */
                  vuint32_t:3;  /*  */
                vuint32_t AD_MUX:1;     /* new in MPC563xM */
                vuint32_t BL:1; /*  */
                vuint32_t WEBS:1;       /*  */
                vuint32_t TBDIP:1;      /*  */
                  vuint32_t:1;  /*  */
                vuint32_t SETA:1;       /* new in MPC563xM */
                vuint32_t BI:1; /*  */
                vuint32_t V:1;  /*  */
            } B;
        } BR;                   /* <URM>EBI_BR</URM>  */
        union {
            vuint32_t R;
            struct {
                vuint32_t AM:17;        /*  */
                  vuint32_t:7;  /*  */
                vuint32_t SCY:4;        /*  */
                  vuint32_t:1;  /*  */
                vuint32_t BSCY:2;       /*  */
                  vuint32_t:1;  /*  */
            } B;
        } OR;                   /* <URM>EBI_OR</URM> */
    };
    struct CAL_CS_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t BA:17;        /*  */
                  vuint32_t:3;  /*  */
                vuint32_t PS:1; /*  */
                  vuint32_t:3;  /*  */
                vuint32_t AD_MUX:1;     /* new in MPC563xM */
                vuint32_t BL:1; /*  */
                vuint32_t WEBS:1;       /*  */
                vuint32_t TBDIP:1;      /*  */
                  vuint32_t:1;  /*  */
                vuint32_t SETA:1;       /* new in MPC563xM */
                vuint32_t BI:1; /*  */
                vuint32_t V:1;  /*  */
            } B;
        } BR;                   /* <URM>EBI_CAL_BR</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t AM:17;        /*  */
                  vuint32_t:7;  /*  */
                vuint32_t SCY:4;        /*  */
                  vuint32_t:1;  /*  */
                vuint32_t BSCY:2;       /*  */
                  vuint32_t:1;  /*  */
            } B;
        } OR;                   /* <URM>EBI_CAL_OR</URM> */

    };

    struct EBI_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:5;    /*  */
                vuint32_t SIZEEN:1;     /* <URM>SIZEN</URM> */
                vuint32_t SIZE:2;       /*  */
                  vuint32_t:8;  /*  */
                vuint32_t ACGE:1;       /*  */
                vuint32_t EXTM:1;       /*  */
                vuint32_t EARB:1;       /*  */
                vuint32_t EARP:2;       /*  */
                  vuint32_t:4;  /*  */
                vuint32_t MDIS:1;       /*  */
                  vuint32_t:3;  /*  */
                vuint32_t D16_31:1;     /* new in MPC563xM */
                vuint32_t AD_MUX:1;     /* new in MPC563xM */
                vuint32_t DBM:1;        /*  */
            } B;
        } MCR;                  /* EBI Module Configuration Register (MCR) <URM>EBI_MCR</URM> @baseaddress + 0x00 */

        uint32_t EBI_reserved1[1];

        union {
            vuint32_t R;
            struct {
                vuint32_t:30;   /*  */
                vuint32_t TEAF:1;       /*  */
                vuint32_t BMTF:1;       /*  */
            } B;
        } TESR;                 /* EBI Transfer Error Status Register (TESR) <URM>EBI_TESR</URM> @baseaddress + 0x08 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;   /*  */
                vuint32_t BMT:8;        /*  */
                vuint32_t BME:1;        /*  */
                  vuint32_t:7;  /*  */
            } B;
        } BMCR;                 /* <URM>EBI_BMCR</URM> @baseaddress + 0x0C */

        struct CS_tag CS[4];

        uint32_t EBI_reserved2[4];

        /* Calibration registers */
        struct CAL_CS_tag CAL_CS[4];

    };                          /* end of EBI_tag */
/****************************************************************************/
/*                     MODULE : FLASH                                       */
/****************************************************************************/
/* 3 flash modules implemented.                                             */
/*       HBL and HBS not used in Bank 0 / Array 0                           */
/*       LML, SLL, LMS, PFCR1, PFAPR, PFCR2, and PFCR3 not used in          */
/*       Bank 1 / Array 1 or Bank 1 / Array 3                                 */
/****************************************************************************/
    struct FLASH_tag {
        union {                 /* Module Configuration Register (MCR)@baseaddress + 0x00 */
            vuint32_t R;
            struct {
                vuint32_t EDC:1;        /* ECC Data Correction (Read/Clear) */
                  vuint32_t:4;  /* Reserved */
                vuint32_t SIZE:3;       /* Array Size (Read Only) */
                  vuint32_t:1;  /* Reserved */
                vuint32_t LAS:3;        /* Low Address Space (Read Only) */
                  vuint32_t:3;  /* Reserved */
                vuint32_t MAS:1;        /* Mid Address Space (Read Only) */
                vuint32_t EER:1;        /* ECC Event Error (Read/Clear) *//* <LEGACY> BBEPE and EPE </LEGACY> */
                vuint32_t RWE:1;        /* Read While Write Event Error (Read/Clear) */
                  vuint32_t:2;  /* Reserved */
                vuint32_t PEAS:1;       /* Program/Erase Access Space (Read Only) */
                vuint32_t DONE:1;       /* Status (Read Only) */
                vuint32_t PEG:1;        /* Program/Erase Good (Read Only) */
                  vuint32_t:4;  /* Reserved *//* <LEGACY> RSD PEG STOP RSVD </LEGACY> */
                vuint32_t PGM:1;        /* Program (Read/Write) */
                vuint32_t PSUS:1;       /* Program Suspend (Read/Write) */
                vuint32_t ERS:1;        /* Erase (Read/Write) */
                vuint32_t ESUS:1;       /* Erase Suspend (Read/Write) */
                vuint32_t EHV:1;        /* Enable High Voltage (Read/Write) */
            } B;
        } MCR;

        union {                 /* Low/Mid-Address Space Block Locking Register (LML)@baseaddress + 0x04 */
            vuint32_t R;
            struct {
                vuint32_t LME:1;        /* Low/Mid address space block enable (Read Only) */
                  vuint32_t:10; /* Reserved */
                vuint32_t SLOCK:1;      /*<URM>SLK</URM> *//* Shadow address space block lock (Read/Write) */
                  vuint32_t:2;  /* Reserved */
                vuint32_t MLOCK:2;      /*<URM>MLK</URM> *//* Mid address space block lock (Read/Write) */
                  vuint32_t:8;  /* Reserved */
                vuint32_t LLOCK:8;      /*<URM>LLK</URM> *//* Low address space block lock (Read/Write) */
            } B;
        } LMLR;                 /*<URM>LML</URM> */

        union {                 /* High-Address Space Block Locking Register (HBL) - @baseaddress + 0x08 */
            vuint32_t R;
            struct {
                vuint32_t HBE:1;        /* High address space Block Enable (Read Only) */
                  vuint32_t:27; /* Reserved */
                vuint32_t HBLOCK:4;     /* High address space block lock (Read/Write) */
            } B;
        } HLR;                  /*<URM>HBL</URM> */

        union {                 /* Secondary Low/Mid-Address Space Block Locking Register (SLL)@baseaddress + 0x0C */
            vuint32_t R;
            struct {
                vuint32_t SLE:1;        /* Secondary low/mid address space block enable (Read Only) */
                  vuint32_t:10; /* Reserved */
                vuint32_t SSLOCK:1;     /*<URM>SSLK</URM> *//* Secondary shadow address space block lock (Read/Write) */
                  vuint32_t:2;  /* Reserved */
                vuint32_t SMLOCK:2;     /*<URM>SMK</URM> *//* Secondary mid address space block lock (Read/Write) */
                  vuint32_t:8;  /* Reserved */
                vuint32_t SLLOCK:8;     /*<URM>SLK</URM> *//* Secondary low address space block lock (Read/Write) */
            } B;
        } SLMLR;                /*<URM>SLL</URM> */

        union {                 /* Low/Mid-Address Space Block Select Register (LMS)@baseaddress + 0x10 */
            vuint32_t R;
            struct {
                vuint32_t:14;   /* Reserved */
                vuint32_t MSEL:2;       /*<URM>MSL</URM> *//* Mid address space block select (Read/Write) */
                  vuint32_t:8;  /* Reserved */
                vuint32_t LSEL:8;       /*<URM>LSL</URM> *//* Low address space block select (Read/Write) */
            } B;
        } LMSR;                 /*<URM>LMS</URM> */

        union {                 /* High-Address Space Block Select Register (HBS) - not used@baseaddress + 0x14 */
            vuint32_t R;
            struct {
                vuint32_t:28;   /* Reserved */
                vuint32_t HBSEL:4;      /*<URM>HSL</URM> *//* High address space block select (Read/Write) */
            } B;
        } HSR;                  /*<URM>HBS</URM> */

        union {                 /* Address Register (ADR)@baseaddress + 0x18 */
            vuint32_t R;
            struct {
                vuint32_t SAD:1;        /* Shadow address (Read Only) */
                  vuint32_t:10; /* Reserved */
                vuint32_t ADDR:18;      /*<URM>AD</URM> *//* Address 20-3 (Read Only) */
                  vuint32_t:3;  /* Reserved */
            } B;
        } AR;                   /*<URM>ADR</URM> */

        union {                 /* @baseaddress + 0x1C */
            vuint32_t R;
            struct {
                vuint32_t:7;    /* Reserved */
                vuint32_t GCE:1;        /* Global Configuration Enable (Read/Write) */
                  vuint32_t:4;  /* Reserved */
                vuint32_t M3PFE:1;      /* Master 3 Prefetch Enable (Read/Write) */
                vuint32_t M2PFE:1;      /* Master 2 Prefetch Enable (Read/Write) */
                vuint32_t M1PFE:1;      /* Master 1 Prefetch Enable (Read/Write) */
                vuint32_t M0PFE:1;      /* Master 0 Prefetch Enable (Read/Write) */
                vuint32_t APC:3;        /* Address Pipelining Control (Read/Write) */
                vuint32_t WWSC:2;       /* Write Wait State Control (Read/Write) */
                vuint32_t RWSC:3;       /* Read Wait State Control (Read/Write) */
                  vuint32_t:1;  /* Reserved */
                vuint32_t DPFEN:1;      /*<URM>DPFE</URM> *//* Data Prefetch Enable (Read/Write) */
                  vuint32_t:1;  /* Reserved */
                vuint32_t IPFEN:1;      /*<URM>IPFE</URM> *//* Instruction Prefetch Enable (Read/Write) */
                  vuint32_t:1;  /* Reserved */
                vuint32_t PFLIM:2;      /* Prefetch Limit (Read/Write) */
                vuint32_t BFEN:1;       /*<URM>BFE</URM> *//* Buffer Enable (Read/Write) */
            } B;
        } BIUCR;                /*<URM>PFCR1</URM> */

        union {                 /* @baseaddress + 0x20 */
            vuint32_t R;
            struct {
                vuint32_t:24;   /* Reserved */
                vuint32_t M3AP:2;       /* Master 3 Access Protection (Read/Write) */
                vuint32_t M2AP:2;       /* Master 2 Access Protection (Read/Write) */
                vuint32_t M1AP:2;       /* Master 1 Access Protection (Read/Write) */
                vuint32_t M0AP:2;       /* Master 0 Access Protection (Read/Write) */
            } B;
        } BIUAPR;               /*<URM>PFAPR</URM> */

        union {                 /* @baseaddress + 0x24 */
            vuint32_t R;
            struct {
                vuint32_t LBCFG:2;      /* Line Buffer Configuration (Read/Write) */
                  vuint32_t:30; /* Reserved */
            } B;
        } BIUCR2;

        union {                 /* @baseaddress + 0x28 */
            vuint32_t R;
            struct {
                vuint32_t:25;   /* Reserved */
                vuint32_t B1_DPFE:1;    /* Bank1 Data Prefetch Enable (Read/Write) */
                  vuint32_t:1;  /* Reserved */
                vuint32_t B1_IPFE:1;    /* Bank1 Instruction Prefetch Enable (Read/Write) */
                  vuint32_t:1;  /* Reserved */
                vuint32_t B1_PFLIM:2;   /* Bank1 Prefetch Limit (Read/Write) */
                vuint32_t B1_BFE:1;     /* Bank1 Buffer Enable (Read/Write) */
            } B;
        } PFCR3;

        int32_t FLASH_reserverd_89[4];

        union {                 /* User Test 0 (UT0) register@baseaddress + 0x3c */
            vuint32_t R;
            struct {
                vuint32_t UTE:1;        /* User test enable (Read/Clear) */
                vuint32_t SBCE:1;       /* Single bit correction enable (Read/Clear) */
                  vuint32_t:6;  /* Reserved */
                vuint32_t DSI:8;        /* Data syndrome input (Read/Write) */
                  vuint32_t:9;  /* Reserved */
                  vuint32_t:1;  /* Reserved (Read/Write) */
                vuint32_t MRE:1;        /* Margin Read Enable (Read/Write) */
                vuint32_t MRV:1;        /* Margin Read Value (Read/Write) */
                vuint32_t EIE:1;        /* ECC data Input Enable (Read/Write) */
                vuint32_t AIS:1;        /* Array Integrity Sequence (Read/Write) */
                vuint32_t AIE:1;        /* Array Integrity Enable (Read/Write) */
                vuint32_t AID:1;        /* Array Integrity Done (Read Only) */
            } B;
        } UT0;

        union {                 /* User Test 1 (UT1) register@baseaddress + 0x40 */
            vuint32_t R;
            struct {
                vuint32_t DAI:32;       /* Data Array Input (Read/Write) */
            } B;
        } UT1;

        union {                 /* User Test 2 (UT2) register@baseaddress + 0x44 */
            vuint32_t R;
            struct {
                vuint32_t DAI:32;       /* Data Array Input (Read/Write) */
            } B;
        } UT2;

        union {                 /* User Multiple Input Signature Register 0-5 (UMISR[5])@baseaddress + 0x48 */
            vuint32_t R;
            struct {
                vuint32_t MS:32;        /* Multiple input Signature (Read/Write) */
            } B;
        } UMISR[5];

    };                          /* end of FLASH_tag */
/****************************************************************************/
/*                     MODULE : SIU                                      */
/****************************************************************************/
    struct SIU_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t S_F:1;        /* Identifies the Manufacturer <URM>S/F</URM> */
                vuint32_t FLASH_SIZE_1:4;       /* Define major Flash memory size (see Table 15-4 for details) <URM>Flash Size 1</URM> */
                vuint32_t FLASH_SIZE_2:4;       /* Define Flash memory size, small granularity (see Table 15-5 for details) <URM>Flash Size 1</URM> */
                vuint32_t TEMP_RANGE:2; /* Define maximum operating range <URM>Temp Range</URM> */
                  vuint32_t:1;  /* Reserved for future enhancements */
                vuint32_t MAX_FREQ:2;   /* Define maximum device speed <URM>Max Freq</URM> */
                  vuint32_t:1;  /* Reserved for future enhancements */
                vuint32_t SUPPLY:1;     /* Defines if the part is 5V or 3V <URM>Supply</URM> */
                vuint32_t PART_NUMBER:8;        /* Contain the ASCII representation of the character that indicates the product <URM>Part Number</URM> */
                vuint32_t TBD:1;        /* 1-bit field defined by SoC to describe optional feature, e.g., additional SPI */
                  vuint32_t:2;  /* Reserved for future enhancements */
                vuint32_t EE:1; /* Indicates if Data Flash is present */
                  vuint32_t:3;  /* Reserved for future enhancements */
                vuint32_t FR:1; /* Indicates if Data FlexRay is present */
            } B;
        } MIDR2;                /* MCU ID Register 2  <URM>SIU_MIDR2</URM> @baseaddress + 0x4 */

        union {
            vuint32_t R;
            struct {
                vuint32_t PARTNUM:16;   /* Device part number: 0x5633 */
                vuint32_t CSP:1;        /* CSP configuration (new in MPC563xM) */
                vuint32_t PKG:5;        /* Indicate the package the die is mounted in. (new in MPC563xM)  */
                  vuint32_t:2;  /* Reserved */
                vuint32_t MASKNUM:8;    /* MCU major mask number; updated for each complete resynthesis. MCU minor mask number; updated for each mask revision */
            } B;
        } MIDR;                 /* MCU ID Register (MIDR) <URM>SIU_MIDR</URM> @baseaddress + 0x8 */

        union {
            vuint32_t R;
        } TST;                  /* SIU Test Register (SIU_TST) <URM>SIU_TST</URM> @baseaddress + 0xC */

        union {
            vuint32_t R;
            struct {
                vuint32_t PORS:1;       /*  Power-On Reset Status */
                vuint32_t ERS:1;        /*  External Reset Status */
                vuint32_t LLRS:1;       /*  Loss of Lock Reset Status */
                vuint32_t LCRS:1;       /*  Loss of Clock Reset Status */
                vuint32_t WDRS:1;       /*  Watchdog Timer/Debug Reset Status */
                vuint32_t CRS:1;        /*  Checkstop Reset Status */
                vuint32_t SWTRS:1;      /*  Software Watchdog Timer Reset Status (new in MPC563xM) */
                  vuint32_t:7;  /*  */
                vuint32_t SSRS:1;       /*  Software System Reset Status */
                vuint32_t SERF:1;       /*  Software External Reset Flag */
                vuint32_t WKPCFG:1;     /*  Weak Pull Configuration Pin Status */
                  vuint32_t:11; /*  */
                vuint32_t ABR:1;        /*  Auto Baud Rate (new in MPC563xM) */
                vuint32_t BOOTCFG:2;    /*  Reset Configuration Pin Status  */
                vuint32_t RGF:1;        /*  RESET Glitch Flag */
            } B;
        } RSR;                  /* Reset Status Register (SIU_RSR) <URM>SIU_RSR</URM> @baseaddress + 0x10 */

        union {
            vuint32_t R;
            struct {
                vuint32_t SSR:1;        /*  Software System Reset */
                vuint32_t SER:1;        /*  Software External Reset */
                  vuint32_t:14; /*  */
                vuint32_t CRE:1;        /*  Checkstop Reset Enable */
                  vuint32_t:15; /*  */
            } B;
        } SRCR;                 /* System Reset Control Register (SRCR) <URM>SIU_SRCR</URM> @baseaddress + 0x14 */

        union {
            vuint32_t R;
            struct {
                vuint32_t NMI:1;        /* Non-Maskable Interrupt Flag (new in MPC563xM) */
                  vuint32_t:7;  /*  */
                vuint32_t SWT:1;        /* Software Watch Dog Timer Interrupt Flag, from platform (new in MPC563xM) */
                  vuint32_t:7;  /*  */
                vuint32_t EIF15:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF14:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF13:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF12:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF11:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF10:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF9:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF8:1;       /* External Interrupt Request Flag x */
                  vuint32_t:3;  /* (reserved in MPC563xM) */
                vuint32_t EIF4:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF3:1;       /* External Interrupt Request Flag x */
                  vuint32_t:2;  /* (reserved in MPC563xM) */
                vuint32_t EIF0:1;       /* External Interrupt Request Flag x */
            } B;
        } EISR;                 /* SIU External Interrupt Status Register (EISR) <URM>SIU_EISR</URM> @baseaddress + 0x18 */

        union {
            vuint32_t R;
            struct {
                vuint32_t NMI_SEL:1;    /* NMI Interrupt Platform Input Selection (new in MPC563xM) */
                  vuint32_t:7;  /*  */
                vuint32_t SWT_SEL:1;
                  vuint32_t:7;
                vuint32_t EIRE15:1;     /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE14:1;     /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE13:1;     /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE12:1;     /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE11:1;     /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE10:1;     /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE9:1;      /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE8:1;      /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE7:1;      /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE6:1;      /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE5:1;      /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE4:1;      /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE3:1;      /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE2:1;      /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE1:1;      /* External DMA/Interrupt Request Enable x */
                vuint32_t EIRE0:1;      /* External DMA/Interrupt Request Enable x */
            } B;
        } DIRER;                /* DMA/Interrupt Request Enable Register (DIRER) <URM>SIU_DIRER</URM> @baseaddress + 0x1C */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;   /*  */
                vuint32_t DIRS3:1;      /* DMA/Interrupt Request Select x */
                  vuint32_t:2;  /* reserved in MPC563xM */
                vuint32_t DIRS0:1;      /* DMA/Interrupt Request Select x */
            } B;
        } DIRSR;                /* DMA/Interrupt Request Select Register (DIRSR) <URM>SIU_DIRSR</URM> @baseaddress + 0x20 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;   /*  */
                vuint32_t OVF15:1;      /* Overrun Flag x */
                vuint32_t OVF14:1;      /* Overrun Flag x */
                vuint32_t OVF13:1;      /* Overrun Flag x */
                vuint32_t OVF12:1;      /* Overrun Flag x */
                vuint32_t OVF11:1;      /* Overrun Flag x */
                vuint32_t OVF10:1;      /* Overrun Flag x */
                vuint32_t OVF9:1;       /* Overrun Flag x */
                vuint32_t OVF8:1;       /* Overrun Flag x */
                  vuint32_t:3;  /* reserved in MPC563xM */
                vuint32_t OVF4:1;       /* Overrun Flag x */
                vuint32_t OVF3:1;       /* Overrun Flag x */
                  vuint32_t:2;  /* reserved in MPC563xM */
                vuint32_t OVF0:1;       /* Overrun Flag x */
            } B;
        } OSR;                  /* Overrun Status Register (OSR) <URM>SIU_OSR</URM> @baseaddress + 0x24 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;   /*  */
                vuint32_t ORE15:1;      /* Overrun Request Enable x */
                vuint32_t ORE14:1;      /* Overrun Request Enable x */
                vuint32_t ORE13:1;      /* Overrun Request Enable x */
                vuint32_t ORE12:1;      /* Overrun Request Enable x */
                vuint32_t ORE11:1;      /* Overrun Request Enable x */
                vuint32_t ORE10:1;      /* Overrun Request Enable x */
                vuint32_t ORE9:1;       /* Overrun Request Enable x */
                vuint32_t ORE8:1;       /* Overrun Request Enable x */
                  vuint32_t:3;  /* reserved in MPC563xM */
                vuint32_t ORE4:1;       /* Overrun Request Enable x */
                vuint32_t ORE3:1;       /* Overrun Request Enable x */
                  vuint32_t:2;  /* reserved in MPC563xM */
                vuint32_t ORE0:1;       /* Overrun Request Enable x */
            } B;
        } ORER;                 /* Overrun Request Enable Register (ORER) <URM>SIU_ORER</URM> @baseaddress + 0x28 */

        union {
            vuint32_t R;
            struct {
                vuint32_t NMIRE:1;      /* NMI Rising-Edge Event Enable x (new in MPC563xM) */
                  vuint32_t:15; /* reserved in MPC563xM */
                vuint32_t IREE15:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE14:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE13:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE12:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE11:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE10:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE9:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE8:1;      /* IRQ Rising-Edge Event Enable x */
                  vuint32_t:3;  /* reserved in MPC563xM */
                vuint32_t IREE4:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE3:1;      /* IRQ Rising-Edge Event Enable x */
                  vuint32_t:2;  /* reserved in MPC563xM */
                vuint32_t IREE0:1;      /* IRQ Rising-Edge Event Enable x */
            } B;
        } IREER;                /* External IRQ Rising-Edge Event Enable Register (IREER) <URM>SIU_IREER</URM> @baseaddress + 0x2C */

        union {
            vuint32_t R;
            struct {
                vuint32_t NMIFE:1;      /* NMI Falling-Edge Event Enable x (new in MPC563xM) */
                vuint32_t Reserverd:15; /*  */
                vuint32_t IFEE15:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE14:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE13:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE12:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE11:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE10:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE9:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE8:1;      /* IRQ Falling-Edge Event Enable x */
                  vuint32_t:3;  /* reserved in MPC563xM */
                vuint32_t IFEE4:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE3:1;      /* IRQ Falling-Edge Event Enable x */
                  vuint32_t:2;  /* reserved in MPC563xM */
                vuint32_t IFEE0:1;      /* IRQ Falling-Edge Event Enable x */
            } B;
        } IFEER;                /* External IRQ Falling-Edge Event Enable Regi (IFEER) <URM>SIU_IFEER</URM> @baseaddress + 0x30 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;   /*  */
                vuint32_t DFL:4;        /* Digital Filter Length */
            } B;
        } IDFR;                 /* External IRQ Digital Filter Register (IDFR) <URM>SIU_IDFR</URM> @baseaddress + 0x40 */

        int32_t SIU_reserverd_153[3];

        union {
            vuint16_t R;
            struct {
                vuint16_t:2;    /*  */
                vuint16_t PA:4; /*  */
                vuint16_t OBE:1;        /*  */
                vuint16_t IBE:1;        /*  */
                vuint16_t DSC:2;        /*  */
                vuint16_t ODE:1;        /*  */
                vuint16_t HYS:1;        /*  */
                vuint16_t SRC:2;        /*  */
                vuint16_t WPE:1;        /*  */
                vuint16_t WPS:1;        /*  */
            } B;
        } PCR[512];             /* Pad Configuration Register  (PCR) <URM>SIU_PCR</URM> @baseaddress + 0x600 */

        int32_t SIU_reserverd_164[112];

        union {
            vuint8_t R;
            struct {
                vuint8_t:7;     /*  */
                vuint8_t PDO:1; /*  */
            } B;
        } GPDO[512];            /* GPIO Pin Data Output Register (GPDO) <URM>SIU_GDPO</URM> @baseaddress + 0x800 */

        union {
            vuint8_t R;
            struct {
                vuint8_t:7;     /*  */
                vuint8_t PDI:1; /*  */
            } B;
        } GPDI[256];            /* GPIO Pin Data Input Register (GDPI) <URM>SIU_GDPI</URM> @baseaddress + 0x900 */

        union {
            vuint32_t R;
            struct {
                vuint32_t TSEL5:2;      /* eQADC Trigger 5 Input */
                vuint32_t TSEL4:2;      /* eQADC Trigger 4 Input */
                vuint32_t TSEL3:2;      /* eQADC Trigger 3 Input */
                vuint32_t TSEL2:2;      /* eQADC Trigger 4 Input */
                vuint32_t TSEL1:2;      /* eQADC Trigger 1 Input */
                vuint32_t TSEL0:2;      /* eQADC Trigger 0 Input */
                  vuint32_t:20; /*  */
            } B;
        } ETISR;                /* eQADC Trigger Input Select Register (ETISR) <URM>SIU_ETISR</URM> @baseaddress + 0x904 */

        union {
            vuint32_t R;
            struct {
                vuint32_t ESEL15:2;     /* External IRQ Input Select x */
                vuint32_t ESEL14:2;     /* External IRQ Input Select x */
                vuint32_t ESEL13:2;     /* External IRQ Input Select x */
                vuint32_t ESEL12:2;     /* External IRQ Input Select x */
                vuint32_t ESEL11:2;     /* External IRQ Input Select x */
                vuint32_t ESEL10:2;     /* External IRQ Input Select x */
                vuint32_t ESEL9:2;      /* External IRQ Input Select x */
                vuint32_t ESEL8:2;      /* External IRQ Input Select x */
                vuint32_t ESEL7:2;      /* External IRQ Input Select x */
                vuint32_t ESEL6:2;      /* External IRQ Input Select x */
                vuint32_t ESEL5:2;      /* External IRQ Input Select x */
                vuint32_t ESEL4:2;      /* External IRQ Input Select x */
                vuint32_t ESEL3:2;      /* External IRQ Input Select x */
                vuint32_t ESEL2:2;      /* External IRQ Input Select x */
                vuint32_t ESEL1:2;      /* External IRQ Input Select x */
                vuint32_t ESEL0:2;      /* External IRQ Input Select x */
            } B;
        } EIISR;                /* External IRQ Input Select Register (EIISR) <URM>SIU_EIISR</URM> @baseaddress + 0x908 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;    /* reserved in MPC563xM */
                vuint32_t SINSELB:2;    /* DSPI_B Data Input Select <URM>SIN-SELB</URM> */
                vuint32_t SSSELB:2;     /* DSPI_B Slave Select Input Select <URM>SS-SELB</URM> */
                vuint32_t SCKSELB:2;    /* DSPI_B Clock Input Select <URM>SCK-SELB</URM> */
                vuint32_t TRIGSELB:2;   /* DSPI_B Trigger Input Select <URM>TRIG-SELB</URM> */
                vuint32_t SINSELC:2;    /* DSPI_C Data Input Select <URM>SIN-SELC</URM> */
                vuint32_t SSSELC:2;     /* DSPI_C Slave Select Input Select <URM>SSSELC</URM> */
                vuint32_t SCKSELC:2;    /* DSPI_C Clock Input Select <URM>SCK-SELC</URM> */
                vuint32_t TRIGSELC:2;   /* DSPI_C Trigger Input Select <URM>TRIG-SELC</URM> */
                  vuint32_t:8;  /* reserved in MPC563xM */
            } B;
        } DISR;                 /* DSPI Input Select Register (DISR) <URM>SIU_DISR</URM> @baseaddress + 0x90c */

        union {
            vuint32_t R;
            struct {
                vuint32_t:2;    /*  */
                vuint32_t ETSEL5:5;     /* eQADC queue X Enhanced Trigger Selection <URM>eTSEL5</URM> */
                vuint32_t ETSEL4:5;     /* eQADC queue X Enhanced Trigger Selection <URM>eTSEL4</URM> */
                vuint32_t ETSEL3:5;     /* eQADC queue X Enhanced Trigger Selection <URM>eTSEL3</URM> */
                vuint32_t ETSEL2:5;     /* eQADC queue X Enhanced Trigger Selection <URM>eTSEL2</URM> */
                vuint32_t ETSEL1:5;     /* eQADC queue X Enhanced Trigger Selection <URM>eTSEL1</URM> */
                vuint32_t ETSEL0:5;     /* eQADC queue X Enhanced Trigger Selection <URM>eTSEL0</URM> */
            } B;
        } ISEL3;                /* MUX Select Register 3 (ISEL3) (new in MPC563xM) <URM>SIU_ISEL3</URM> @baseaddress + 0x920 */

        int32_t SIU_reserverd_214[4];

        union {
            vuint32_t R;
            struct {
                vuint32_t:11;   /*  */
                vuint32_t ESEL5:1;      /* <URM>eSEL5</URM> */
                  vuint32_t:3;  /*  */
                vuint32_t ESEL4:1;      /* <URM>eSEL4</URM> */
                  vuint32_t:3;  /*  */
                vuint32_t ESEL3:1;      /* <URM>eSEL3</URM> */
                  vuint32_t:3;  /*  */
                vuint32_t ESEL2:1;      /* <URM>eSEL2</URM> */
                  vuint32_t:3;  /*  */
                vuint32_t ESEL1:1;      /* <URM>eSEL1</URM> */
                  vuint32_t:3;  /*  */
                vuint32_t ESEL0:1;      /* <URM>eSEL0</URM> */
            } B;
        } ISEL8;                /* MUX Select Register 8 (ISEL8) (new in MPC563xM) <URM>SIU_ISEL8</URM> @baseaddress + 0x924 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:27;   /*  */
                vuint32_t ETSEL0A:5;    /* <URM>eTSEL0A</URM> */
            } B;
        } ISEL9;                /* MUX Select Register 9(ISEL9) <URM>SIU_ISEL9</URM> @baseaddress + 0x980 */

        int32_t SIU_reserverd_230[22];

        union {
            vuint32_t R;
            struct {
                vuint32_t:14;   /*  */
                vuint32_t MATCH:1;      /*  Compare Register Match */
                vuint32_t DISNEX:1;     /*  Disable Nexus */
                  vuint32_t:14; /*  */
                vuint32_t CRSE:1;       /*  Calibration Reflection Suppression Enable (new in MPC563xM) */
                  vuint32_t:1;  /*  */
            } B;
        } CCR;                  /* Chip Configuration Register (CCR) <URM>SIU_CCR</URM> @baseaddress + 0x984 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;   /* The ENGDIV bit is reserved in MPC563xM */
                vuint32_t EBTS:1;       /* External Bus Tap Select */
                  vuint32_t:1;  /*  */
                vuint32_t EBDF:2;       /* External Bus Division Factor */
            } B;
        } ECCR;                 /* External Clock Control Register (ECCR) <URM>SIU_ECCR</URM> @baseaddress + 0x988 */

        union {
            vuint32_t R;
        } CARH;                 /* Compare A High Register (CARH) <URM>SIU_CMPAH</URM> @baseaddress + 0x98C */

        union {
            vuint32_t R;
        } CARL;                 /* Compare A Low Register (CARL) <URM>SIU_CMPAL</URM> @baseaddress + 0x990 */

        union {
            vuint32_t R;
        } CBRH;                 /* Compare B High Register (CBRH) <URM>SIU_CMPBH</URM> @baseaddress + 0x994 */

        union {
            vuint32_t R;
        } CBRL;                 /* Compare B Low Register (CBRL)  <URM>SIU_CMPBL</URM> @baseaddress + 0x9A0 */

        int32_t SIU_reserverd_250[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:27;   /* Reserved */
                vuint32_t BYPASS:1;     /* Bypass bit <URM>BY-PASS</URM> */
                vuint32_t SYSCLKDIV:2;  /* System Clock Divide <URM>SYS-CLKDIV</URM> */
                  vuint32_t:2;  /* Reserved */
            } B;
        } SYSDIV;               /* System Clock Register (SYSDIV) (new in MPC563xM) <URM>SIU_SYSDIV</URM>  @baseaddress + 0x9A4 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CPUSTP:1;     /* CPU stop request. When asserted, a stop request is sent to the following modules: */
                  vuint32_t:2;  /* Reserved */
                vuint32_t SWTSTP:1;     /* SWT stop request. When asserted, a stop request is sent to the Software Watchdog */
                  vuint32_t:1;  /* Reserved */
                vuint32_t TPUSTP:1;     /* eTPU stop request. When asserted, a stop request is sent to the eTPU module. */
                vuint32_t NPCSTP:1;     /* Nexus stop request. When asserted, a stop request is sent to the Nexus Controller. */
                vuint32_t EBISTP:1;     /* EBI stop request. When asserted, a stop request is sent to the external bus */
                vuint32_t ADCSTP:1;     /* eQADC stop request. When asserted, a stop request is sent to the eQADC module. */
                  vuint32_t:1;  /* Reserved */
                vuint32_t MIOSSTP:1;    /* Stop mode request */
                vuint32_t DFILSTP:1;    /* Decimation filter stop request. When asserted, a stop request is sent to the */
                  vuint32_t:1;  /* Reserved */
                vuint32_t PITSTP:1;     /* PIT stop request. When asserted, a stop request is sent to the periodical internal */
                  vuint32_t:3;  /* Reserved */
                vuint32_t CNCSTP:1;     /* FlexCAN C stop request. When asserted, a stop request is sent to the FlexCAN C */
                  vuint32_t:1;  /* Reserved */
                vuint32_t CNASTP:1;     /* FlexCAN A stop request. When asserted, a stop request is sent to the FlexCAN A */
                  vuint32_t:1;  /* Reserved */
                vuint32_t SPICSTP:1;    /* DSPI C stop request. When asserted, a stop request is sent to the DSPI C. */
                vuint32_t SPIBSTP:1;    /* DSPI B stop request. When asserted, a stop request is sent to the DSPI B. */
                  vuint32_t:7;  /* Reserved */
                vuint32_t SCIBSTP:1;    /* eSCI B stop request. When asserted, a stop request is sent to the eSCI B module. */
                vuint32_t SCIASTP:1;    /* eSCI A stop request. When asserted, a stop request is sent to the eSCIA module. */
            } B;
        } HLT;                  /* Halt Register (HLT) (new in MPC563xM) <URM>SIU_HLT</URM> @baseaddress + 0x9A8 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CPUACK:1;     /* CPU stop acknowledge. When asserted, indicates that a stop acknowledge was */
                  vuint32_t:2;  /* Reserved */
                vuint32_t SWTACK:1;     /* SWT stop acknowledge. When asserted, indicates that a stop acknowledge was */
                  vuint32_t:1;  /* Reserved */
                vuint32_t TPUACK:1;     /* eTPU stop acknowledge. When asserted, indicates that a stop acknowledge was */
                vuint32_t NPCACK:1;     /* Nexus stop acknowledge. When asserted, indicates that a stop acknowledge was */
                vuint32_t EBIACK:1;     /* EBI stop acknowledge. When asserted, indicates that a stop acknowledge was */
                vuint32_t ADCACK:1;     /* eQADC stop acknowledge. When asserted, indicates that a stop acknowledge was */
                  vuint32_t:1;  /* Reserved */
                vuint32_t MIOSACK:1;    /* eMIOS stop acknowledge. When asserted, indicates that a stop acknowledge was */
                vuint32_t DFILACK:1;    /* Decimation filter stop acknowledge. When asserted, indicates that a stop */
                  vuint32_t:1;  /* Reserved */
                vuint32_t PITACK:1;     /* PIT stop acknowledge. When asserted, indicates that a stop acknowledge was */
                  vuint32_t:3;  /* Reserved */
                vuint32_t CNCACK:1;     /* FlexCAN C stop acknowledge. When asserted, indicates that a stop acknowledge */
                  vuint32_t:1;  /* Reserved */
                vuint32_t CNAACK:1;     /* FlexCAN A stop acknowledge. When asserted, indicates that a stop acknowledge */
                  vuint32_t:1;  /* Reserved */
                vuint32_t SPICACK:1;    /* DSPI C stop acknowledge. When asserted, indicates that a stop acknowledge was */
                vuint32_t SPIBACK:1;    /* DSPI B stop acknowledge. When asserted, indicates that a stop acknowledge was */
                  vuint32_t:7;  /* Reserved */
                vuint32_t SCIBACK:1;    /* eSCI B stop acknowledge */
                vuint32_t SCIAACK:1;    /* eSCI A stop acknowledge. */
            } B;
        } HLTACK;               /* Halt Acknowledge Register (HLTACK) (new in MPC563xM) <URM>SIU_HLTACK</URM> @baseaddress + 0x9ac */

        int32_t SIU_reserved3[21];

    };                          /* end of SIU_tag */
/****************************************************************************/
/*                     MODULE : EMIOS                                      */
/****************************************************************************/
    struct EMIOS_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t DOZEEN:1;     /* new in MPC563xM */
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
        } MCR;                  /* Module Configuration Register <URM>EMIOSMCR</URM> */

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
        } GFR;                  /* Global FLAG Register  <URM>EMIOSGFLAG</URM> */

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
        } OUDR;                 /* Output Update Disable Register  <URM>EMIOSOUDIS</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;    /*  */
                vuint32_t CHDIS23:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS22:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS21:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS20:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS19:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS18:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS17:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS16:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS15:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS14:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS13:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS12:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS11:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS10:1;    /* Enable Channel [n] bit */
                vuint32_t CHDIS9:1;     /* Enable Channel [n] bit */
                vuint32_t CHDIS8:1;     /* Enable Channel [n] bit */
                vuint32_t CHDIS7:1;     /* Enable Channel [n] bit */
                vuint32_t CHDIS6:1;     /* Enable Channel [n] bit */
                vuint32_t CHDIS5:1;     /* Enable Channel [n] bit */
                vuint32_t CHDIS4:1;     /* Enable Channel [n] bit */
                vuint32_t CHDIS3:1;     /* Enable Channel [n] bit */
                vuint32_t CHDIS2:1;     /* Enable Channel [n] bit */
                vuint32_t CHDIS1:1;     /* Enable Channel [n] bit */
                vuint32_t CHDIS0:1;     /* Enable Channel [n] bit */
            } B;
        } UCDIS;                /* Disable Channel (EMIOSUCDIS) <URM>EMIOSUCDIS</URM> (new in MPC563xM) @baseaddress + 0x0C */

        int32_t EMIOS_reserverd_30[4];

        struct {
            union {
                vuint32_t R;    /* Channel A Data Register */
            } CADR;             /* <URM>EMIOSA</URM> */

            union {
                vuint32_t R;    /* Channel B Data Register */
            } CBDR;             /* <URM>EMIOSB</URM> */

            union {
                vuint32_t R;    /* Channel Counter Register */
            } CCNTR;            /* <URM>EMIOSCNT</URM> */

            union {
                vuint32_t R;
                struct {
                    vuint32_t FREN:1;
                    vuint32_t ODIS:1;
                    vuint32_t ODISSL:2;
                    vuint32_t UCPRE:2;
                    vuint32_t UCPREN:1;
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
            } CCR;              /* Channel Control Register  <URM>EMIOSC</URM> */

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
            } CSR;              /* Channel Status Register <URM>EMIOSS</URM> */

            union {
                vuint32_t R;    /* Alternate Channel A Data Register */
            } ALTA;             /* new in MPC563xM <URM>EMIOSALTA</URM> */

            uint32_t emios_channel_reserved[2];

        } CH[24];

    };                          /* end of EMIOS_tag */
/****************************************************************************/
/*                     MODULE : ETPU                                      */
/****************************************************************************/
    struct ETPU_tag {           /* offset 0x0000 */
        union {                 /* eTPU module configuration register@baseaddress + 0x00 */
            vuint32_t R;
            struct {
                vuint32_t GEC:1;        /*  Global Exception Clear  */
                vuint32_t SDMERR:1;     /*  */
                vuint32_t WDTOA:1;      /*  */
                vuint32_t WDTOB:1;      /*  */
                vuint32_t MGE1:1;       /* <URM>MGEA</URM> */
                vuint32_t MGE2:1;       /* <URM>MGEB</URM> */
                vuint32_t ILF1:1;       /* Invalid instruction flag eTPU A. <URM>ILFFA</URM> */
                vuint32_t ILF2:1;       /* Invalid instruction flag eTPU B. <URM>ILFFB</URM> */
                vuint32_t SCMERR:1;     /* . */
                  vuint32_t:2;  /*  */
                vuint32_t SCMSIZE:5;    /*  Shared Code Memory size  */
                  vuint32_t:4;  /*  */
                vuint32_t SCMMISC:1;    /*  SCM MISC Flag  */
                vuint32_t SCMMISF:1;    /*  SCM MISC Flag  */
                vuint32_t SCMMISEN:1;   /*  SCM MISC Enable  */
                  vuint32_t:2;  /*  */
                vuint32_t VIS:1;        /*  SCM Visability  */
                  vuint32_t:5;  /*  */
                vuint32_t GTBE:1;       /*  Global Time Base Enable  */
            } B;
        } MCR;                  /* <URM>ETPU_MCR</URM> */

        /* offset 0x0004 */
        union {                 /* eTPU coherent dual-parameter controller register@baseaddress + 0x04 */
            vuint32_t R;
            struct {
                vuint32_t STS:1;        /*  Start Status bit  */
                vuint32_t CTBASE:5;     /*  Channel Transfer Base  */
                vuint32_t PBASE:10;     /*  Parameter Buffer Base Address  <URM>PBBASE</URM> */
                vuint32_t PWIDTH:1;     /*  Parameter Width  */
                vuint32_t PARAM0:7;     /*  Channel Parameter 0  <URM>PARM0</URM> */
                vuint32_t WR:1; /*  */
                vuint32_t PARAM1:7;     /*  Channel Parameter 1  <URM>PARM1</URM> */
            } B;
        } CDCR;                 /*<URM>ETPU_CDCR</URM> */

        vuint32_t ETPU_reserved_0;

        /* offset 0x000C */
        union {                 /* eTPU MISC Compare Register@baseaddress + 0x0c */
            vuint32_t R;
            struct {
                vuint32_t ETPUMISCCMP:32;       /* Expected multiple input signature calculator compare register value. <URM>EMISCCMP</URM> */
            } B;
        } MISCCMPR /*<URM>ETPU_MISCCMPR</URM> */ ;

        /* offset 0x0010 */
        union {                 /* eTPU SCM Off-Range Data Register@baseaddress + 0x10 */
            vuint32_t R;
            struct {
                vuint32_t ETPUSCMOFFDATA:32;    /* SCM Off-range read data value. */
            } B;
        } SCMOFFDATAR;          /*<URM>ETPU_SCMOFFDATAR</URM> */

        /* offset 0x0014 */
        union {                 /* eTPU Engine Configuration Register (ETPUA_ECR)@baseaddress + 0x14 */
            vuint32_t R;
            struct {
                vuint32_t FEND:1;       /*  Force END  */
                vuint32_t MDIS:1;       /*  Low power Stop  */
                  vuint32_t:1;  /*  */
                vuint32_t STF:1;        /*  Stop Flag  */
                  vuint32_t:4;  /*  */
                vuint32_t HLTF:1;       /*  Halt Mode Flag  */
                  vuint32_t:3;  /*  */
                vuint32_t FCSS:1;
                vuint32_t FPSCK:3;      /*  Filter Prescaler Clock Control  */
                vuint32_t CDFC:2;       /*  */
                  vuint32_t:1;  /*  */
                vuint32_t ERBA:5;       /*  */
                vuint32_t SPPDIS:1;     /*  */
                  vuint32_t:2;  /*  */
                vuint32_t ETB:5;        /*  Entry Table Base  */
            } B;
        } ECR_A;                /*<URM>ETPU_ECR</URM> */

        vuint32_t ETPU_reserved_1[2];

        /* offset 0x0020 */
        union {                 /* eTPU Time Base Configuration Register (ETPU_TBCR)@baseaddress + 0x20 */
            vuint32_t R;
            struct {
                vuint32_t TCR2CTL:3;    /*  TCR2 Clock/Gate Control  */
                vuint32_t TCRCF:2;      /*  TCRCLK Signal Filter Control  */
                vuint32_t AM:2; /*  Angle Mode  */
                  vuint32_t:3;  /*  */
                vuint32_t TCR2P:6;      /*  TCR2 Prescaler Control  */
                vuint32_t TCR1CTL:2;    /*  TCR1 Clock/Gate Control  */
                vuint32_t TCR1CS:1;     /*  */
                  vuint32_t:5;  /*  */
                vuint32_t TCR1P:8;      /*  TCR1 Prescaler Control  */
            } B;
        } TBCR_A;               /*<URM>ETPU_TBCR</URM> */

        /* offset 0x0024 */
        union {                 /* eTPU Time Base 1 (TCR1) Visibility Register (ETPU_TB1R)@baseaddress + 0x24 */
            vuint32_t R;
            struct {
                vuint32_t:8;    /*  */
                vuint32_t TCR1:24;      /* TCR1 value. Used on matches and captures. For more information, see the eTPU reference manual. */
            } B;
        } TB1R_A;               /*<URM>ETPU_TB1R</URM> */

        /* offset 0x0028 */
        union {                 /* eTPU Time Base 2 (TCR2) Visibility Register (ETPU_TB2R)@baseaddress + 0x28 */
            vuint32_t R;
            struct {
                vuint32_t:8;    /*  */
                vuint32_t TCR2:24;      /* TCR2 value. Used on matches and captures. For information on TCR2, see the eTPU reference manual. */
            } B;
        } TB2R_A;               /*<URM>ETPU_TB2R</URM> */

        /* offset 0x002C */
        union {                 /* STAC Bus Configuration Register (ETPU_STACCR)@baseaddress + 0x2c */
            vuint32_t R;
            struct {
                vuint32_t REN1:1;       /*  Resource Enable TCR1  */
                vuint32_t RSC1:1;       /*  Resource Control TCR1  */
                  vuint32_t:2;  /*  */
                vuint32_t SERVER_ID1:4; /*  */
                  vuint32_t:4;  /*  */
                vuint32_t SRV1:4;       /*  Resource Server Slot  */
                vuint32_t REN2:1;       /*  Resource Enable TCR2  */
                vuint32_t RSC2:1;       /*  Resource Control TCR2  */
                  vuint32_t:2;  /*  */
                vuint32_t SERVER_ID2:4; /*  */
                  vuint32_t:4;  /*  */
                vuint32_t SRV2:4;       /*  Resource Server Slot  */
            } B;
        } REDCR_A;              /*<URM>ETPU_REDCR</URM> */

        vuint32_t ETPU_reserved_2[12];

        /* offset 0x0060 */
        union {                 /* ETPU1 WDTR Register */
            vuint32_t R;
            struct {
                vuint32_t WDM:2;
                  vuint32_t:14;
                vuint32_t WDCNT:16;
            } B;
        } WDTR_A;

        vuint32_t ETPU1_reserved_3;

        /* offset 0x0068 */
        union {                 /* ETPU1 IDLE Register */
            vuint32_t R;
            struct {
                vuint32_t IDLE_CNT:31;
                vuint32_t ICLR:1;
            } B;
        } IDLE_A;

        vuint32_t ETPU_reserved_4[101];

        /* offset 0x0200 */
        union {                 /* eTPU Channel Interrupt Status Register (ETPU_CISR)@baseaddress + 0x200 */
            vuint32_t R;
            struct {
                vuint32_t CIS31:1;      /*  Channel 31 Interrut Status  */
                vuint32_t CIS30:1;      /*  Channel 30 Interrut Status  */
                vuint32_t CIS29:1;      /*  Channel 29 Interrut Status  */
                vuint32_t CIS28:1;      /*  Channel 28 Interrut Status  */
                vuint32_t CIS27:1;      /*  Channel 27 Interrut Status  */
                vuint32_t CIS26:1;      /*  Channel 26 Interrut Status  */
                vuint32_t CIS25:1;      /*  Channel 25 Interrut Status  */
                vuint32_t CIS24:1;      /*  Channel 24 Interrut Status  */
                vuint32_t CIS23:1;      /*  Channel 23 Interrut Status  */
                vuint32_t CIS22:1;      /*  Channel 22 Interrut Status  */
                vuint32_t CIS21:1;      /*  Channel 21 Interrut Status  */
                vuint32_t CIS20:1;      /*  Channel 20 Interrut Status  */
                vuint32_t CIS19:1;      /*  Channel 19 Interrut Status  */
                vuint32_t CIS18:1;      /*  Channel 18 Interrut Status  */
                vuint32_t CIS17:1;      /*  Channel 17 Interrut Status  */
                vuint32_t CIS16:1;      /*  Channel 16 Interrut Status  */
                vuint32_t CIS15:1;      /*  Channel 15 Interrut Status  */
                vuint32_t CIS14:1;      /*  Channel 14 Interrut Status  */
                vuint32_t CIS13:1;      /*  Channel 13 Interrut Status  */
                vuint32_t CIS12:1;      /*  Channel 12 Interrut Status  */
                vuint32_t CIS11:1;      /*  Channel 11 Interrut Status  */
                vuint32_t CIS10:1;      /*  Channel 10 Interrut Status  */
                vuint32_t CIS9:1;       /*  Channel 9 Interrut Status  */
                vuint32_t CIS8:1;       /*  Channel 8 Interrut Status  */
                vuint32_t CIS7:1;       /*  Channel 7 Interrut Status  */
                vuint32_t CIS6:1;       /*  Channel 6 Interrut Status  */
                vuint32_t CIS5:1;       /*  Channel 5 Interrut Status  */
                vuint32_t CIS4:1;       /*  Channel 4 Interrut Status  */
                vuint32_t CIS3:1;       /*  Channel 3 Interrut Status  */
                vuint32_t CIS2:1;       /*  Channel 2 Interrut Status  */
                vuint32_t CIS1:1;       /*  Channel 1 Interrut Status  */
                vuint32_t CIS0:1;       /*  Channel 0 Interrut Status  */
            } B;
        } CISR_A;               /* <URM>ETPU_CISR</URM> */

        int32_t ETPU_reserved_5[3];

        /* offset 0x0210 */
        union {                 /* @baseaddress + 0x210 */
            vuint32_t R;
            struct {
                vuint32_t DTRS31:1;     /*  Channel 31 Data Transfer Request Status  */
                vuint32_t DTRS30:1;     /*  Channel 30 Data Transfer Request Status  */
                vuint32_t DTRS29:1;     /*  Channel 29 Data Transfer Request Status  */
                vuint32_t DTRS28:1;     /*  Channel 28 Data Transfer Request Status  */
                vuint32_t DTRS27:1;     /*  Channel 27 Data Transfer Request Status  */
                vuint32_t DTRS26:1;     /*  Channel 26 Data Transfer Request Status  */
                vuint32_t DTRS25:1;     /*  Channel 25 Data Transfer Request Status  */
                vuint32_t DTRS24:1;     /*  Channel 24 Data Transfer Request Status  */
                vuint32_t DTRS23:1;     /*  Channel 23 Data Transfer Request Status  */
                vuint32_t DTRS22:1;     /*  Channel 22 Data Transfer Request Status  */
                vuint32_t DTRS21:1;     /*  Channel 21 Data Transfer Request Status  */
                vuint32_t DTRS20:1;     /*  Channel 20 Data Transfer Request Status  */
                vuint32_t DTRS19:1;     /*  Channel 19 Data Transfer Request Status  */
                vuint32_t DTRS18:1;     /*  Channel 18 Data Transfer Request Status  */
                vuint32_t DTRS17:1;     /*  Channel 17 Data Transfer Request Status  */
                vuint32_t DTRS16:1;     /*  Channel 16 Data Transfer Request Status  */
                vuint32_t DTRS15:1;     /*  Channel 15 Data Transfer Request Status  */
                vuint32_t DTRS14:1;     /*  Channel 14 Data Transfer Request Status  */
                vuint32_t DTRS13:1;     /*  Channel 13 Data Transfer Request Status  */
                vuint32_t DTRS12:1;     /*  Channel 12 Data Transfer Request Status  */
                vuint32_t DTRS11:1;     /*  Channel 11 Data Transfer Request Status  */
                vuint32_t DTRS10:1;     /*  Channel 10 Data Transfer Request Status  */
                vuint32_t DTRS9:1;      /*  Channel 9 Data Transfer Request Status  */
                vuint32_t DTRS8:1;      /*  Channel 8 Data Transfer Request Status  */
                vuint32_t DTRS7:1;      /*  Channel 7 Data Transfer Request Status  */
                vuint32_t DTRS6:1;      /*  Channel 6 Data Transfer Request Status  */
                vuint32_t DTRS5:1;      /*  Channel 5 Data Transfer Request Status  */
                vuint32_t DTRS4:1;      /*  Channel 4 Data Transfer Request Status  */
                vuint32_t DTRS3:1;      /*  Channel 3 Data Transfer Request Status  */
                vuint32_t DTRS2:1;      /*  Channel 2 Data Transfer Request Status  */
                vuint32_t DTRS1:1;      /*  Channel 1 Data Transfer Request Status  */
                vuint32_t DTRS0:1;      /*  Channel 0 Data Transfer Request Status  */
            } B;
        } CDTRSR_A;             /* <URM>ETPU_CDTRSR</URM> */

        int32_t ETPU_reserved_6[3];

        /* offset 0x0220 */
        union {                 /* eTPU Channel Interrupt Overflow Status Register (ETPU_CIOSR)@baseaddress + 0x220 */
            vuint32_t R;
            struct {
                vuint32_t CIOS31:1;     /*  Channel 31 Interruput Overflow Status  */
                vuint32_t CIOS30:1;     /*  Channel 30 Interruput Overflow Status  */
                vuint32_t CIOS29:1;     /*  Channel 29 Interruput Overflow Status  */
                vuint32_t CIOS28:1;     /*  Channel 28 Interruput Overflow Status  */
                vuint32_t CIOS27:1;     /*  Channel 27 Interruput Overflow Status  */
                vuint32_t CIOS26:1;     /*  Channel 26 Interruput Overflow Status  */
                vuint32_t CIOS25:1;     /*  Channel 25 Interruput Overflow Status  */
                vuint32_t CIOS24:1;     /*  Channel 24 Interruput Overflow Status  */
                vuint32_t CIOS23:1;     /*  Channel 23 Interruput Overflow Status  */
                vuint32_t CIOS22:1;     /*  Channel 22 Interruput Overflow Status  */
                vuint32_t CIOS21:1;     /*  Channel 21 Interruput Overflow Status  */
                vuint32_t CIOS20:1;     /*  Channel 20 Interruput Overflow Status  */
                vuint32_t CIOS19:1;     /*  Channel 19 Interruput Overflow Status  */
                vuint32_t CIOS18:1;     /*  Channel 18 Interruput Overflow Status  */
                vuint32_t CIOS17:1;     /*  Channel 17 Interruput Overflow Status  */
                vuint32_t CIOS16:1;     /*  Channel 16 Interruput Overflow Status  */
                vuint32_t CIOS15:1;     /*  Channel 15 Interruput Overflow Status  */
                vuint32_t CIOS14:1;     /*  Channel 14 Interruput Overflow Status  */
                vuint32_t CIOS13:1;     /*  Channel 13 Interruput Overflow Status  */
                vuint32_t CIOS12:1;     /*  Channel 12 Interruput Overflow Status  */
                vuint32_t CIOS11:1;     /*  Channel 11 Interruput Overflow Status  */
                vuint32_t CIOS10:1;     /*  Channel 10 Interruput Overflow Status  */
                vuint32_t CIOS9:1;      /*  Channel 9 Interruput Overflow Status  */
                vuint32_t CIOS8:1;      /*  Channel 8 Interruput Overflow Status  */
                vuint32_t CIOS7:1;      /*  Channel 7 Interruput Overflow Status  */
                vuint32_t CIOS6:1;      /*  Channel 6 Interruput Overflow Status  */
                vuint32_t CIOS5:1;      /*  Channel 5 Interruput Overflow Status  */
                vuint32_t CIOS4:1;      /*  Channel 4 Interruput Overflow Status  */
                vuint32_t CIOS3:1;      /*  Channel 3 Interruput Overflow Status  */
                vuint32_t CIOS2:1;      /*  Channel 2 Interruput Overflow Status  */
                vuint32_t CIOS1:1;      /*  Channel 1 Interruput Overflow Status  */
                vuint32_t CIOS0:1;      /*  Channel 0 Interruput Overflow Status  */
            } B;
        } CIOSR_A;              /* <URM>ETPU_CIOSR</URM> */

        int32_t ETPU_reserved_7[3];

        /* offset 0x0230 */
        union {                 /* eTPU Channel Data Transfer Request Overflow Status Register@baseaddress + 0x230 */
            vuint32_t R;
            struct {
                vuint32_t DTROS31:1;    /*  Channel 31 Data Transfer Overflow Status  */
                vuint32_t DTROS30:1;    /*  Channel 30 Data Transfer Overflow Status  */
                vuint32_t DTROS29:1;    /*  Channel 29 Data Transfer Overflow Status  */
                vuint32_t DTROS28:1;    /*  Channel 28 Data Transfer Overflow Status  */
                vuint32_t DTROS27:1;    /*  Channel 27 Data Transfer Overflow Status  */
                vuint32_t DTROS26:1;    /*  Channel 26 Data Transfer Overflow Status  */
                vuint32_t DTROS25:1;    /*  Channel 25 Data Transfer Overflow Status  */
                vuint32_t DTROS24:1;    /*  Channel 24 Data Transfer Overflow Status  */
                vuint32_t DTROS23:1;    /*  Channel 23 Data Transfer Overflow Status  */
                vuint32_t DTROS22:1;    /*  Channel 22 Data Transfer Overflow Status  */
                vuint32_t DTROS21:1;    /*  Channel 21 Data Transfer Overflow Status  */
                vuint32_t DTROS20:1;    /*  Channel 20 Data Transfer Overflow Status  */
                vuint32_t DTROS19:1;    /*  Channel 19 Data Transfer Overflow Status  */
                vuint32_t DTROS18:1;    /*  Channel 18 Data Transfer Overflow Status  */
                vuint32_t DTROS17:1;    /*  Channel 17 Data Transfer Overflow Status  */
                vuint32_t DTROS16:1;    /*  Channel 16 Data Transfer Overflow Status  */
                vuint32_t DTROS15:1;    /*  Channel 15 Data Transfer Overflow Status  */
                vuint32_t DTROS14:1;    /*  Channel 14 Data Transfer Overflow Status  */
                vuint32_t DTROS13:1;    /*  Channel 13 Data Transfer Overflow Status  */
                vuint32_t DTROS12:1;    /*  Channel 12 Data Transfer Overflow Status  */
                vuint32_t DTROS11:1;    /*  Channel 11 Data Transfer Overflow Status  */
                vuint32_t DTROS10:1;    /*  Channel 10 Data Transfer Overflow Status  */
                vuint32_t DTROS9:1;     /*  Channel 9 Data Transfer Overflow Status  */
                vuint32_t DTROS8:1;     /*  Channel 8 Data Transfer Overflow Status  */
                vuint32_t DTROS7:1;     /*  Channel 7 Data Transfer Overflow Status  */
                vuint32_t DTROS6:1;     /*  Channel 6 Data Transfer Overflow Status  */
                vuint32_t DTROS5:1;     /*  Channel 5 Data Transfer Overflow Status  */
                vuint32_t DTROS4:1;     /*  Channel 4 Data Transfer Overflow Status  */
                vuint32_t DTROS3:1;     /*  Channel 3 Data Transfer Overflow Status  */
                vuint32_t DTROS2:1;     /*  Channel 2 Data Transfer Overflow Status  */
                vuint32_t DTROS1:1;     /*  Channel 1 Data Transfer Overflow Status  */
                vuint32_t DTROS0:1;     /*  Channel 0 Data Transfer Overflow Status  */
            } B;
        } CDTROSR_A;            /* <URM>ETPU_CDTROSR</URM> */

        int32_t ETPU_reserved_8[3];

        /* offset 0x0240 */
        union {                 /* eTPU Channel Interrupt Enable Register (ETPU_CIER)@baseaddress + 0x240 */
            vuint32_t R;
            struct {
                vuint32_t CIE31:1;      /*  Channel 31 Interruput Enable  */
                vuint32_t CIE30:1;      /*  Channel 30 Interruput Enable  */
                vuint32_t CIE29:1;      /*  Channel 29 Interruput Enable  */
                vuint32_t CIE28:1;      /*  Channel 28 Interruput Enable  */
                vuint32_t CIE27:1;      /*  Channel 27 Interruput Enable  */
                vuint32_t CIE26:1;      /*  Channel 26 Interruput Enable  */
                vuint32_t CIE25:1;      /*  Channel 25 Interruput Enable  */
                vuint32_t CIE24:1;      /*  Channel 24 Interruput Enable  */
                vuint32_t CIE23:1;      /*  Channel 23 Interruput Enable  */
                vuint32_t CIE22:1;      /*  Channel 22 Interruput Enable  */
                vuint32_t CIE21:1;      /*  Channel 21 Interruput Enable  */
                vuint32_t CIE20:1;      /*  Channel 20 Interruput Enable  */
                vuint32_t CIE19:1;      /*  Channel 19 Interruput Enable  */
                vuint32_t CIE18:1;      /*  Channel 18 Interruput Enable  */
                vuint32_t CIE17:1;      /*  Channel 17 Interruput Enable  */
                vuint32_t CIE16:1;      /*  Channel 16 Interruput Enable  */
                vuint32_t CIE15:1;      /*  Channel 15 Interruput Enable  */
                vuint32_t CIE14:1;      /*  Channel 14 Interruput Enable  */
                vuint32_t CIE13:1;      /*  Channel 13 Interruput Enable  */
                vuint32_t CIE12:1;      /*  Channel 12 Interruput Enable  */
                vuint32_t CIE11:1;      /*  Channel 11 Interruput Enable  */
                vuint32_t CIE10:1;      /*  Channel 10 Interruput Enable  */
                vuint32_t CIE9:1;       /*  Channel 9 Interruput Enable  */
                vuint32_t CIE8:1;       /*  Channel 8 Interruput Enable  */
                vuint32_t CIE7:1;       /*  Channel 7 Interruput Enable  */
                vuint32_t CIE6:1;       /*  Channel 6 Interruput Enable  */
                vuint32_t CIE5:1;       /*  Channel 5 Interruput Enable  */
                vuint32_t CIE4:1;       /*  Channel 4 Interruput Enable  */
                vuint32_t CIE3:1;       /*  Channel 3 Interruput Enable  */
                vuint32_t CIE2:1;       /*  Channel 2 Interruput Enable  */
                vuint32_t CIE1:1;       /*  Channel 1 Interruput Enable  */
                vuint32_t CIE0:1;       /*  Channel 0 Interruput Enable  */
            } B;
        } CIER_A;               /* <URM>ETPU_CIER</URM> */

        int32_t ETPU_reserved_9[3];

        /* offset 0x0250 */
        union {                 /* eTPU Channel Data Transfer Request Enable Register (ETPU_CDTRER)@baseaddress + 0x250 */
            vuint32_t R;
            struct {
                vuint32_t DTRE31:1;     /*  Channel 31 Data Transfer Request Enable  */
                vuint32_t DTRE30:1;     /*  Channel 30 Data Transfer Request Enable  */
                vuint32_t DTRE29:1;     /*  Channel 29 Data Transfer Request Enable  */
                vuint32_t DTRE28:1;     /*  Channel 28 Data Transfer Request Enable  */
                vuint32_t DTRE27:1;     /*  Channel 27 Data Transfer Request Enable  */
                vuint32_t DTRE26:1;     /*  Channel 26 Data Transfer Request Enable  */
                vuint32_t DTRE25:1;     /*  Channel 25 Data Transfer Request Enable  */
                vuint32_t DTRE24:1;     /*  Channel 24 Data Transfer Request Enable  */
                vuint32_t DTRE23:1;     /*  Channel 23 Data Transfer Request Enable  */
                vuint32_t DTRE22:1;     /*  Channel 22 Data Transfer Request Enable  */
                vuint32_t DTRE21:1;     /*  Channel 21 Data Transfer Request Enable  */
                vuint32_t DTRE20:1;     /*  Channel 20 Data Transfer Request Enable  */
                vuint32_t DTRE19:1;     /*  Channel 19 Data Transfer Request Enable  */
                vuint32_t DTRE18:1;     /*  Channel 18 Data Transfer Request Enable  */
                vuint32_t DTRE17:1;     /*  Channel 17 Data Transfer Request Enable  */
                vuint32_t DTRE16:1;     /*  Channel 16 Data Transfer Request Enable  */
                vuint32_t DTRE15:1;     /*  Channel 15 Data Transfer Request Enable  */
                vuint32_t DTRE14:1;     /*  Channel 14 Data Transfer Request Enable  */
                vuint32_t DTRE13:1;     /*  Channel 13 Data Transfer Request Enable  */
                vuint32_t DTRE12:1;     /*  Channel 12 Data Transfer Request Enable  */
                vuint32_t DTRE11:1;     /*  Channel 11 Data Transfer Request Enable  */
                vuint32_t DTRE10:1;     /*  Channel 10 Data Transfer Request Enable  */
                vuint32_t DTRE9:1;      /*  Channel 9 Data Transfer Request Enable  */
                vuint32_t DTRE8:1;      /*  Channel 8 Data Transfer Request Enable  */
                vuint32_t DTRE7:1;      /*  Channel 7 Data Transfer Request Enable  */
                vuint32_t DTRE6:1;      /*  Channel 6 Data Transfer Request Enable  */
                vuint32_t DTRE5:1;      /*  Channel 5 Data Transfer Request Enable  */
                vuint32_t DTRE4:1;      /*  Channel 4 Data Transfer Request Enable  */
                vuint32_t DTRE3:1;      /*  Channel 3 Data Transfer Request Enable  */
                vuint32_t DTRE2:1;      /*  Channel 2 Data Transfer Request Enable  */
                vuint32_t DTRE1:1;      /*  Channel 1 Data Transfer Request Enable  */
                vuint32_t DTRE0:1;      /*  Channel 0 Data Transfer Request Enable  */
            } B;
        } CDTRER_A;             /* <URM>ETPU_CDTRER</URM> */

        int32_t ETPU_reserved_10[3];

        /* offset 0x0260 */
        union {                 /* ETPUWDSR - eTPU Watchdog Status Register */
            vuint32_t R;
            struct {
                vuint32_t WDS31:1;      /*  Channel 31 Data Transfer Request Enable  */
                vuint32_t WDS30:1;      /*  Channel 30 Data Transfer Request Enable  */
                vuint32_t WDS29:1;      /*  Channel 29 Data Transfer Request Enable  */
                vuint32_t WDS28:1;      /*  Channel 28 Data Transfer Request Enable  */
                vuint32_t WDS27:1;      /*  Channel 27 Data Transfer Request Enable  */
                vuint32_t WDS26:1;      /*  Channel 26 Data Transfer Request Enable  */
                vuint32_t WDS25:1;      /*  Channel 25 Data Transfer Request Enable  */
                vuint32_t WDS24:1;      /*  Channel 24 Data Transfer Request Enable  */
                vuint32_t WDS23:1;      /*  Channel 23 Data Transfer Request Enable  */
                vuint32_t WDS22:1;      /*  Channel 22 Data Transfer Request Enable  */
                vuint32_t WDS21:1;      /*  Channel 21 Data Transfer Request Enable  */
                vuint32_t WDS20:1;      /*  Channel 20 Data Transfer Request Enable  */
                vuint32_t WDS19:1;      /*  Channel 19 Data Transfer Request Enable  */
                vuint32_t WDS18:1;      /*  Channel 18 Data Transfer Request Enable  */
                vuint32_t WDS17:1;      /*  Channel 17 Data Transfer Request Enable  */
                vuint32_t WDS16:1;      /*  Channel 16 Data Transfer Request Enable  */
                vuint32_t WDS15:1;      /*  Channel 15 Data Transfer Request Enable  */
                vuint32_t WDS14:1;      /*  Channel 14 Data Transfer Request Enable  */
                vuint32_t WDS13:1;      /*  Channel 13 Data Transfer Request Enable  */
                vuint32_t WDS12:1;      /*  Channel 12 Data Transfer Request Enable  */
                vuint32_t WDS11:1;      /*  Channel 11 Data Transfer Request Enable  */
                vuint32_t WDS10:1;      /*  Channel 10 Data Transfer Request Enable  */
                vuint32_t WDS9:1;       /*  Channel 9 Data Transfer Request Enable  */
                vuint32_t WDS8:1;       /*  Channel 8 Data Transfer Request Enable  */
                vuint32_t WDS7:1;       /*  Channel 7 Data Transfer Request Enable  */
                vuint32_t WDS6:1;       /*  Channel 6 Data Transfer Request Enable  */
                vuint32_t WDS5:1;       /*  Channel 5 Data Transfer Request Enable  */
                vuint32_t WDS4:1;       /*  Channel 4 Data Transfer Request Enable  */
                vuint32_t WDS3:1;       /*  Channel 3 Data Transfer Request Enable  */
                vuint32_t WDS2:1;       /*  Channel 2 Data Transfer Request Enable  */
                vuint32_t WDS1:1;       /*  Channel 1 Data Transfer Request Enable  */
                vuint32_t WDS0:1;       /*  Channel 0 Data Transfer Request Enable  */
            } B;
        } WDSR_A;

        int32_t ETPU_reserved_11[7];

        /* offset 0x0280 */
        union {                 /* ETPUCPSSR - eTPU Channel Pending Service Status Register */
            vuint32_t R;
            struct {
                vuint32_t SR31:1;       /*  Channel 31 Data Transfer Request Enable  */
                vuint32_t SR30:1;       /*  Channel 30 Data Transfer Request Enable  */
                vuint32_t SR29:1;       /*  Channel 29 Data Transfer Request Enable  */
                vuint32_t SR28:1;       /*  Channel 28 Data Transfer Request Enable  */
                vuint32_t SR27:1;       /*  Channel 27 Data Transfer Request Enable  */
                vuint32_t SR26:1;       /*  Channel 26 Data Transfer Request Enable  */
                vuint32_t SR25:1;       /*  Channel 25 Data Transfer Request Enable  */
                vuint32_t SR24:1;       /*  Channel 24 Data Transfer Request Enable  */
                vuint32_t SR23:1;       /*  Channel 23 Data Transfer Request Enable  */
                vuint32_t SR22:1;       /*  Channel 22 Data Transfer Request Enable  */
                vuint32_t SR21:1;       /*  Channel 21 Data Transfer Request Enable  */
                vuint32_t SR20:1;       /*  Channel 20 Data Transfer Request Enable  */
                vuint32_t SR19:1;       /*  Channel 19 Data Transfer Request Enable  */
                vuint32_t SR18:1;       /*  Channel 18 Data Transfer Request Enable  */
                vuint32_t SR17:1;       /*  Channel 17 Data Transfer Request Enable  */
                vuint32_t SR16:1;       /*  Channel 16 Data Transfer Request Enable  */
                vuint32_t SR15:1;       /*  Channel 15 Data Transfer Request Enable  */
                vuint32_t SR14:1;       /*  Channel 14 Data Transfer Request Enable  */
                vuint32_t SR13:1;       /*  Channel 13 Data Transfer Request Enable  */
                vuint32_t SR12:1;       /*  Channel 12 Data Transfer Request Enable  */
                vuint32_t SR11:1;       /*  Channel 11 Data Transfer Request Enable  */
                vuint32_t SR10:1;       /*  Channel 10 Data Transfer Request Enable  */
                vuint32_t SR9:1;        /*  Channel 9 Data Transfer Request Enable  */
                vuint32_t SR8:1;        /*  Channel 8 Data Transfer Request Enable  */
                vuint32_t SR7:1;        /*  Channel 7 Data Transfer Request Enable  */
                vuint32_t SR6:1;        /*  Channel 6 Data Transfer Request Enable  */
                vuint32_t SR5:1;        /*  Channel 5 Data Transfer Request Enable  */
                vuint32_t SR4:1;        /*  Channel 4 Data Transfer Request Enable  */
                vuint32_t SR3:1;        /*  Channel 3 Data Transfer Request Enable  */
                vuint32_t SR2:1;        /*  Channel 2 Data Transfer Request Enable  */
                vuint32_t SR1:1;        /*  Channel 1 Data Transfer Request Enable  */
                vuint32_t SR0:1;        /*  Channel 0 Data Transfer Request Enable  */
            } B;
        } CPSSR_A;              /* <URM>ETPU_CPSSR</URM> */

        int32_t ETPU_reserved_12[3];

        /* offset 0x0290 */
        union {                 /* ETPUCSSR - eTPU Channel Service Status Register */
            vuint32_t R;
            struct {
                vuint32_t SS31:1;       /*  Channel 31 Data Transfer Request Enable  */
                vuint32_t SS30:1;       /*  Channel 30 Data Transfer Request Enable  */
                vuint32_t SS29:1;       /*  Channel 29 Data Transfer Request Enable  */
                vuint32_t SS28:1;       /*  Channel 28 Data Transfer Request Enable  */
                vuint32_t SS27:1;       /*  Channel 27 Data Transfer Request Enable  */
                vuint32_t SS26:1;       /*  Channel 26 Data Transfer Request Enable  */
                vuint32_t SS25:1;       /*  Channel 25 Data Transfer Request Enable  */
                vuint32_t SS24:1;       /*  Channel 24 Data Transfer Request Enable  */
                vuint32_t SS23:1;       /*  Channel 23 Data Transfer Request Enable  */
                vuint32_t SS22:1;       /*  Channel 22 Data Transfer Request Enable  */
                vuint32_t SS21:1;       /*  Channel 21 Data Transfer Request Enable  */
                vuint32_t SS20:1;       /*  Channel 20 Data Transfer Request Enable  */
                vuint32_t SS19:1;       /*  Channel 19 Data Transfer Request Enable  */
                vuint32_t SS18:1;       /*  Channel 18 Data Transfer Request Enable  */
                vuint32_t SS17:1;       /*  Channel 17 Data Transfer Request Enable  */
                vuint32_t SS16:1;       /*  Channel 16 Data Transfer Request Enable  */
                vuint32_t SS15:1;       /*  Channel 15 Data Transfer Request Enable  */
                vuint32_t SS14:1;       /*  Channel 14 Data Transfer Request Enable  */
                vuint32_t SS13:1;       /*  Channel 13 Data Transfer Request Enable  */
                vuint32_t SS12:1;       /*  Channel 12 Data Transfer Request Enable  */
                vuint32_t SS11:1;       /*  Channel 11 Data Transfer Request Enable  */
                vuint32_t SS10:1;       /*  Channel 10 Data Transfer Request Enable  */
                vuint32_t SS9:1;        /*  Channel 9 Data Transfer Request Enable  */
                vuint32_t SS8:1;        /*  Channel 8 Data Transfer Request Enable  */
                vuint32_t SS7:1;        /*  Channel 7 Data Transfer Request Enable  */
                vuint32_t SS6:1;        /*  Channel 6 Data Transfer Request Enable  */
                vuint32_t SS5:1;        /*  Channel 5 Data Transfer Request Enable  */
                vuint32_t SS4:1;        /*  Channel 4 Data Transfer Request Enable  */
                vuint32_t SS3:1;        /*  Channel 3 Data Transfer Request Enable  */
                vuint32_t SS2:1;        /*  Channel 2 Data Transfer Request Enable  */
                vuint32_t SS1:1;        /*  Channel 1 Data Transfer Request Enable  */
                vuint32_t SS0:1;        /*  Channel 0 Data Transfer Request Enable  */
            } B;
        } CSSR_A;               /* <URM>ETPU_CSSR</URM> */

        int32_t ETPU_reserved_13[3];
        int32_t ETPU_reserved_14[88];

/***************************** Channels ********************************/
/* Note not all devices implement all channels or even 2 engines       */
/* Each eTPU engine can implement 64 channels, however most devcies    */
/* only implemnet 32 channels. The eTPU block can implement 1 or 2     */
/* engines per instantiation                                           */
/***********************************************************************/

        struct {
            union {             /* eTPU Channel n Configuration Register (ETPU_CnCR)@baseaddress + 0x400 */
                vuint32_t R;
                struct {
                    vuint32_t CIE:1;    /*  Channel Interruput Enable  */
                    vuint32_t DTRE:1;   /*  Data Transfer Request Enable  */
                    vuint32_t CPR:2;    /*  Channel Priority  */
                      vuint32_t:2;      /*  */
                    vuint32_t ETPD:1;   /*  This bit selects which channel signal, input or output, is used in the entry point selection */
                    vuint32_t ETCS:1;   /*  Entry Table Condition Select  */
                      vuint32_t:3;      /*  */
                    vuint32_t CFS:5;    /*  Channel Function Select  */
                    vuint32_t ODIS:1;   /*  Output disable  */
                    vuint32_t OPOL:1;   /*  output polarity  */
                      vuint32_t:3;      /*  */
                    vuint32_t CPBA:11;  /*  Channel Parameter Base Address  */
                } B;
            } CR;               /* <URM>ETPU_CnCR</URM> */

            union {             /* eTPU Channel n Status Control Register (ETPU_CnSCR)@baseaddress + 0x404 */
                vuint32_t R;
                struct {
                    vuint32_t CIS:1;    /*  Channel Interruput Status  */
                    vuint32_t CIOS:1;   /*  Channel Interruput Overflow Status  */
                      vuint32_t:6;      /*  */
                    vuint32_t DTRS:1;   /*  Data Transfer Status  */
                    vuint32_t DTROS:1;  /*  Data Transfer Overflow Status  */
                      vuint32_t:6;      /*  */
                    vuint32_t IPS:1;    /*  Input Pin State  */
                    vuint32_t OPS:1;    /*  Output Pin State  */
                    vuint32_t OBE:1;    /*  Output Pin State  */
                      vuint32_t:11;     /*  */
                    vuint32_t FM1:1;    /* Function mode */
                    vuint32_t FM0:1;    /* Function mode */
                } B;
            } SCR;              /* <URM>ETPU_CnSCR</URM> */

            union {             /* eTPU channel host service request register (ETPU_CnHSRR)@baseaddress + 0x408 */
                vuint32_t R;
                struct {
                    vuint32_t:29;       /*  Host Service Request  */
                    vuint32_t HSR:3;    /*  */
                } B;
            } HSRR;             /* <URM>ETPU_CnHSRR</URM> */
            int32_t ETPU_reserved_18;

        } CHAN[127];
                 /**** Note: Not all channels implemented on all devices. Up 64 can be implemented on */
    };                          /* end of ETPU_tag */
/****************************************************************************/
/*                     MODULE : XBAR                                      */
/****************************************************************************/
    struct XBAR_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:4;    /* Master 7 Priority - Not implemented */
                vuint32_t:4;    /* Master 6 Priority - Not implemented */
                vuint32_t:4;    /* Master 5 Priority - Not implemented */
                vuint32_t:1;    /*  */
                vuint32_t MSTR4:3;      /* Master 4 Priority - Core load/store & Nexus port */
                  vuint32_t:4;  /* Master 3 Priority - Not implemented */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR2:3;      /* Master 2 Priority - Unused implemented master port */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR1:3;      /* Master 1 Priority - eDMA */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR0:3;      /* Master 0 Priority - e200z335 core Instruction */
            } B;
        } MPR0;                 /* Master Priority Register for Slave port 0 @baseaddress + 0x00  - Flash */

        int32_t XBAR_reserverd_35[3];

        union {
            vuint32_t R;
            struct {
                vuint32_t RO:1; /* Read Only */
                vuint32_t HLP:1;        /* Halt Low Priority (new in MPC563xM) */
                  vuint32_t:6;  /* Slave General Purpose Control Register Reserved */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                vuint32_t HPE4:1;       /* High Priority Enable (new in MPC563xM) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                vuint32_t HPE2:1;       /* High Priority Enable (new in MPC563xM) */
                vuint32_t HPE1:1;       /* High Priority Enable (new in MPC563xM) */
                vuint32_t HPE0:1;       /* High Priority Enable (new in MPC563xM) */
                  vuint32_t:6;  /*  */
                vuint32_t ARB:2;        /* Arbitration Mode */
                  vuint32_t:2;  /*  */
                vuint32_t PCTL:2;       /* Parking Control */
                  vuint32_t:1;  /*  */
                vuint32_t PARK:3;       /* PARK */
            } B;
        } SGPCR0;               /* Slave General Purpose Control Register 0 @baseaddress + 0x10 */

        int32_t XBAR_reserverd_71[59];

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;    /* Master 7 Priority - Not implemented */
                vuint32_t:4;    /* Master 6 Priority - Not implemented */
                vuint32_t:4;    /* Master 5 Priority - Not implemented */
                vuint32_t:1;    /*  */
                vuint32_t MSTR4:3;      /* Master 4 Priority - Core load/store & Nexus port */
                  vuint32_t:4;  /* Master 3 Priority - Not implemented */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR2:3;      /* Master 2 Priority - Unused implemented master port */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR1:3;      /* Master 1 Priority - eDMA */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR0:3;      /* Master 0 Priority - e200z335 core Instruction */
            } B;
        } MPR1;                 /* Master Priority Register for Slave port 1 @baseaddress + 0x100 */

        int32_t XBAR_reserverd_105[3];

        union {
            vuint32_t R;
            struct {
                vuint32_t RO:1; /* Read Only */
                vuint32_t HLP:1;        /* Halt Low Priority (new in MPC563xM) */
                  vuint32_t:6;  /* Slave General Purpose Control Register Reserved */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                vuint32_t HPE4:1;       /* High Priority Enable (new in MPC563xM) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                vuint32_t HPE2:1;       /* High Priority Enable (new in MPC563xM) */
                vuint32_t HPE1:1;       /* High Priority Enable (new in MPC563xM) */
                vuint32_t HPE0:1;       /* High Priority Enable (new in MPC563xM) */
                  vuint32_t:6;  /*  */
                vuint32_t ARB:2;        /* Arbitration Mode */
                  vuint32_t:2;  /*  */
                vuint32_t PCTL:2;       /* Parking Control */
                  vuint32_t:1;  /*  */
                vuint32_t PARK:3;       /* PARK */
            } B;
        } SGPCR1;               /* Slave General Purpose Control Register 1 @baseaddress + 0x110 */

        int32_t XBAR_reserverd_141[59];

/* Slave General Purpose Control Register 2 @baseaddress + 0x210  - not implemented */

        int32_t XBAR_reserverd_211[64];

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;    /* Master 7 Priority - Not implemented */
                vuint32_t:4;    /* Master 6 Priority - Not implemented */
                vuint32_t:4;    /* Master 5 Priority - Not implemented */
                vuint32_t:1;    /*  */
                vuint32_t MSTR4:3;      /* Master 4 Priority - Core load/store & Nexus port */
                  vuint32_t:4;  /* Master 3 Priority - Not implemented */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR2:3;      /* Master 2 Priority - Unused implemented master port */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR1:3;      /* Master 1 Priority - eDMA */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR0:3;      /* Master 0 Priority - e200z335 core Instruction */
            } B;
        } MPR3;                 /* Master Priority Register for Slave port 3 @baseaddress + 0x300 */

        int32_t XBAR_reserverd_245[3];

        union {
            vuint32_t R;
            struct {
                vuint32_t RO:1; /* Read Only */
                vuint32_t HLP:1;        /* Halt Low Priority (new in MPC563xM) */
                  vuint32_t:6;  /* Slave General Purpose Control Register Reserved */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                vuint32_t HPE4:1;       /* High Priority Enable (new in MPC563xM) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                vuint32_t HPE2:1;       /* High Priority Enable (new in MPC563xM) */
                vuint32_t HPE1:1;       /* High Priority Enable (new in MPC563xM) */
                vuint32_t HPE0:1;       /* High Priority Enable (new in MPC563xM) */
                  vuint32_t:6;  /*  */
                vuint32_t ARB:2;        /* Arbitration Mode */
                  vuint32_t:2;  /*  */
                vuint32_t PCTL:2;       /* Parking Control */
                  vuint32_t:1;  /*  */
                vuint32_t PARK:3;       /* PARK */
            } B;
        } SGPCR3;               /* Slave General Purpose Control Register 3 @baseaddress + 0x310 */

        int32_t XBAR_reserverd_281[59];

        /* Slave General Purpose Control Register 4 @baseaddress + 0x410 - not implemented */

        int32_t XBAR_reserverd_351[64];

        /* Slave XBAR Port 5 Not implemented @baseaddress + 0x510 */

        int32_t XBAR_reserverd_421[64];

        /* Slave Port 6 not implemented @baseaddress + 0x610 */

        int32_t XBAR_reserverd_491[64];

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;    /* Master 7 Priority - Not implemented */
                vuint32_t:4;    /* Master 6 Priority - Not implemented */
                vuint32_t:4;    /* Master 5 Priority - Not implemented */
                vuint32_t:1;    /*  */
                vuint32_t MSTR4:3;      /* Master 4 Priority - Core load/store & Nexus port */
                  vuint32_t:4;  /* Master 3 Priority - Not implemented */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR2:3;      /* Master 2 Priority - Unused implemented master port */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR1:3;      /* Master 1 Priority - eDMA */
                  vuint32_t:1;  /*  */
                vuint32_t MSTR0:3;      /* Master 0 Priority - e200z335 core Instruction */
            } B;
        } MPR7;                 /* Master Priority Register for Slave port 7 @baseaddress + 0x700 */

        int32_t XBAR_reserverd_525[3];

        union {
            vuint32_t R;
            struct {
                vuint32_t RO:1; /* Read Only */
                vuint32_t HLP:1;        /* Halt Low Priority (new in MPC563xM) */
                  vuint32_t:6;  /* Slave General Purpose Control Register Reserved */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                vuint32_t HPE4:1;       /* High Priority Enable (new in MPC563xM) */
                  vuint32_t:1;  /* High Priority Enable (new in MPC563xM - not implemented) */
                vuint32_t HPE2:1;       /* High Priority Enable (new in MPC563xM) */
                vuint32_t HPE1:1;       /* High Priority Enable (new in MPC563xM) */
                vuint32_t HPE0:1;       /* High Priority Enable (new in MPC563xM) */
                  vuint32_t:6;  /*  */
                vuint32_t ARB:2;        /* Arbitration Mode */
                  vuint32_t:2;  /*  */
                vuint32_t PCTL:2;       /* Parking Control */
                  vuint32_t:1;  /*  */
                vuint32_t PARK:3;       /* PARK */
            } B;
        } SGPCR7;               /* Slave General Purpose Control Register 7 @baseaddress + 0x710 */

        int32_t XBAR_reserverd_561[59];

        union {
            vuint32_t R;
            struct {
                vuint32_t:29;   /*  */
                vuint32_t AULB:3;       /* Arbitrate on Undefined Length Bursts */
            } B;
        } MGPCR0;               /* Master General Purpose Control Register 0 @baseaddress + 0x800 */

        int32_t XBAR_reserverd_564[63];

        union {
            vuint32_t R;
            struct {
                vuint32_t:29;   /*  */
                vuint32_t AULB:3;       /* Arbitrate on Undefined Length Bursts */
            } B;
        } MGPCR1;               /* Master General Purpose Control Register 1 @baseaddress + 0x900 */

        int32_t XBAR_reserverd_567[63];

        union {
            vuint32_t R;
            struct {
                vuint32_t:29;   /*  */
                vuint32_t AULB:3;       /* Arbitrate on Undefined Length Bursts */
            } B;
        } MGPCR2;               /* Master General Purpose Control Register 2 @baseaddress + 0xA00 */

        int32_t XBAR_reserverd_570[63];

        /* Master General Purpose Control Register 3 not implemented @baseaddress + 0xB00 */

        int32_t XBAR_reserverd_573[64];

        union {
            vuint32_t R;
            struct {
                vuint32_t:29;   /*  */
                vuint32_t AULB:3;       /* Arbitrate on Undefined Length Bursts */
            } B;
        } MGPCR4;               /* Master General Purpose Control Register 4 @baseaddress + 0xC00 */

        int32_t XBAR_reserverd_576[64];

        /* Master General Purpose Control Register 5 not implemented @baseaddress + 0xD00 */

        int32_t XBAR_reserverd_579[64];

        /* Master General Purpose Control Register 6 not implemented @baseaddress + 0xE00 */

        int32_t XBAR_reserverd_582[64];

        /* Master General Purpose Control Register 7 not implemented @baseaddress + 0xF00 */

    };                          /* end of XBAR_tag */
/****************************************************************************/
/*                     MODULE : ECSM                                       */
/****************************************************************************/
    struct ECSM_tag {
        /* SWTCR, SWTSR and SWTIR don't exist in MPC563xM */
        uint32_t ecsm_reserved1[16];

        uint8_t ecsm_reserved3[3];      /* base + 0x40 */

        union {
            vuint8_t R;
            struct {
                vuint8_t:6;
                vuint8_t ERNCR:1;       /* <URM>EPRNCR</URM> */
                vuint8_t EFNCR:1;       /* <URM>EPFNCR</URM> */
            } B;
        } ECR;                  /* ECC Configuration Register */

        uint8_t ecsm_reserved4[3];      /* base + 0x44 */

        union {
            vuint8_t R;
            struct {
                vuint8_t:6;
                vuint8_t RNCE:1;        /* <URM>PRNCE</URM> */
                vuint8_t FNCE:1;        /* <URM>PFNCE</URM> */
            } B;
        } ESR;                  /* ECC Status Register */

        /* EEGR don't exist in MPC563xM */
        uint32_t ecsm_reserved4a[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t FEAR:32;      /* <URM>PFEAR</URM> */
            } B;
        } FEAR;                 /* Flash ECC Address Register <URM>PFEAR</URM> - 0x50 */

        uint16_t ecsm_reserved4b;

        union {
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t FEMR:4;        /* <URM>PFEMR</URM> */
            } B;
        } FEMR;                 /* Flash ECC Master Register <URM>PFEMR</URM> */

        union {
            vuint8_t R;
            struct {
                vuint8_t WRITE:1;
                vuint8_t SIZE:3;
                vuint8_t PROT0:1;       /* <URM>PROTECTION</URM> */
                vuint8_t PROT1:1;       /* <URM>PROTECTION</URM> */
                vuint8_t PROT2:1;       /* <URM>PROTECTION</URM> */
                vuint8_t PROT3:1;       /* <URM>PROTECTION</URM> */
            } B;
        } FEAT;                 /* Flash ECC Attributes Register <URM>PFEAT</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t FEDH:32;      /* <URM>PFEDR</URM> */
            } B;
        } FEDRH;                /* Flash ECC Data High Register <URM>PFEDRH</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t FEDL:32;      /* <URM>PFEDR</URM> */
            } B;
        } FEDRL;                /* Flash ECC Data Low Register <URM>PFEDRL</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t REAR:32;      /* <URM>PREAR</URM> */
            } B;
        } REAR;                 /* RAM ECC Address <URM>PREAR</URM> */

        uint8_t ecsm_reserved5;

        union {
            vuint8_t R;
            struct {
                vuint8_t PRESR:8;
            } B;
        } PRESR;                /* RAM ECC Syndrome (new in MPC563xM) */

        union {
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t REMR:4;        /* <URM>PREMR</URM> */
            } B;
        } REMR;                 /* RAM ECC Master <URM>PREMR</URM> */

        union {
            vuint8_t R;
            struct {
                vuint8_t WRITE:1;
                vuint8_t SIZE:3;
                vuint8_t PROT0:1;       /* <URM>PROTECTION</URM> */
                vuint8_t PROT1:1;       /* <URM>PROTECTION</URM> */
                vuint8_t PROT2:1;       /* <URM>PROTECTION</URM> */
                vuint8_t PROT3:1;       /* <URM>PROTECTION</URM> */
            } B;
        } REAT;                 /*  RAM ECC Attributes Register <URM>PREAT</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t REDH:32;      /* <URM>PREDR</URM> */
            } B;
        } REDRH;                /* RAM ECC Data High Register <URM>PREDRH</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t REDL:32;      /* <URM>PREDR</URM> */
            } B;
        } REDRL;                /* RAMECC Data Low Register <URM>PREDRL</URM> */

    };
/****************************************************************************/
/*                     MODULE : EDMA                                      */
/****************************************************************************/
    struct EDMA_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:14;   /* Reserved */
                vuint32_t CX:1; /* Cancel Transfer (new in MPC563xM) */
                vuint32_t ECX:1;        /* Error Cancel Transfer (new in MPC563xM) */
                vuint32_t GRP3PRI:2;    /* Channel Group 3 Priority (new in MPC563xM) */
                vuint32_t GRP2PRI:2;    /* Channel Group 2 Priority (new in MPC563xM) */
                vuint32_t GRP1PRI:2;    /* Channel Group 1 Priority */
                vuint32_t GRP0PRI:2;    /* Channel Group 0 Priority */
                vuint32_t EMLM:1;       /* Enable Minor Loop Mapping (new in MPC563xM) */
                vuint32_t CLM:1;        /* Continuous Link Mode (new in MPC563xM) */
                vuint32_t HALT:1;       /* Halt DMA Operations (new in MPC563xM) */
                vuint32_t HOE:1;        /* Halt On Error (new in MPC563xM) */
                vuint32_t ERGA:1;       /* Enable Round Robin Group Arbitration */
                vuint32_t ERCA:1;       /* Enable Round Robin Channel Arbitration */
                vuint32_t EDBG:1;       /* Enable Debug */
                vuint32_t EBW:1;        /* Enable Buffered Writes */
            } B;
        } CR;                   /* DMA Control Register <URM>DMACR</URM> @baseaddress + 0x0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t VLD:1;        /* Logical OR of all DMAERRH */

                  vuint32_t:14; /* Reserved */
                vuint32_t ECX:1;        /* (new in MPC563xM) */
                vuint32_t GPE:1;        /* Group Priority Error */
                vuint32_t CPE:1;        /* Channel Priority Error  */
                vuint32_t ERRCHN:6;     /* ERRCHN[5:0] Error Channel Number or The channel number of the last recorded error */
                vuint32_t SAE:1;        /* Source Address Error 0  */
                vuint32_t SOE:1;        /* Source Offset Error */
                vuint32_t DAE:1;        /* Destination Address Error */
                vuint32_t DOE:1;        /* Destination Offset Error */
                vuint32_t NCE:1;        /* Nbytes/Citer Configuration Error */
                vuint32_t SGE:1;        /* Scatter/Gather Configuration Error */
                vuint32_t SBE:1;        /* Source Bus Error */
                vuint32_t DBE:1;        /* Destination Bus Error  */

            } B;
        } ESR;                  /* <URM>DMAES</URM> Error Status Register */

        uint32_t edma_reserved_erqrh;

        union {
            vuint32_t R;
            struct {
                vuint32_t ERQ31:1;
                vuint32_t ERQ30:1;
                vuint32_t ERQ29:1;
                vuint32_t ERQ28:1;
                vuint32_t ERQ27:1;
                vuint32_t ERQ26:1;
                vuint32_t ERQ25:1;
                vuint32_t ERQ24:1;
                vuint32_t ERQ23:1;
                vuint32_t ERQ22:1;
                vuint32_t ERQ21:1;
                vuint32_t ERQ20:1;
                vuint32_t ERQ19:1;
                vuint32_t ERQ18:1;
                vuint32_t ERQ17:1;
                vuint32_t ERQ16:1;
                vuint32_t ERQ15:1;
                vuint32_t ERQ14:1;
                vuint32_t ERQ13:1;
                vuint32_t ERQ12:1;
                vuint32_t ERQ11:1;
                vuint32_t ERQ10:1;
                vuint32_t ERQ09:1;
                vuint32_t ERQ08:1;
                vuint32_t ERQ07:1;
                vuint32_t ERQ06:1;
                vuint32_t ERQ05:1;
                vuint32_t ERQ04:1;
                vuint32_t ERQ03:1;
                vuint32_t ERQ02:1;
                vuint32_t ERQ01:1;
                vuint32_t ERQ00:1;
            } B;
        } ERQRL;                /* <URM>DMAERQL</URM> ,DMA Enable Request Register Low */

        uint32_t edma_reserved_eeirh;

        union {
            vuint32_t R;
            struct {
                vuint32_t EEI31:1;
                vuint32_t EEI30:1;
                vuint32_t EEI29:1;
                vuint32_t EEI28:1;
                vuint32_t EEI27:1;
                vuint32_t EEI26:1;
                vuint32_t EEI25:1;
                vuint32_t EEI24:1;
                vuint32_t EEI23:1;
                vuint32_t EEI22:1;
                vuint32_t EEI21:1;
                vuint32_t EEI20:1;
                vuint32_t EEI19:1;
                vuint32_t EEI18:1;
                vuint32_t EEI17:1;
                vuint32_t EEI16:1;
                vuint32_t EEI15:1;
                vuint32_t EEI14:1;
                vuint32_t EEI13:1;
                vuint32_t EEI12:1;
                vuint32_t EEI11:1;
                vuint32_t EEI10:1;
                vuint32_t EEI09:1;
                vuint32_t EEI08:1;
                vuint32_t EEI07:1;
                vuint32_t EEI06:1;
                vuint32_t EEI05:1;
                vuint32_t EEI04:1;
                vuint32_t EEI03:1;
                vuint32_t EEI02:1;
                vuint32_t EEI01:1;
                vuint32_t EEI00:1;
            } B;
        } EEIRL;                /* <URM>DMAEEIL</URM> , DMA Enable Error Interrupt Register Low */

        union {
            vuint8_t R;
            vuint8_t B;         /* <URM>NOP:1 SERQ:7</URM> */
        } SERQR;                /* <URM>DMASERQ</URM> , DMA Set Enable Request Register */

        union {
            vuint8_t R;
            vuint8_t B;         /* <URM>NOP:1 CERQ:7</URM> */
        } CERQR;                /* <URM>DMACERQ</URM> , DMA Clear Enable Request Register */

        union {
            vuint8_t R;
            vuint8_t B;         /* <URM>NOP:1 SEEI:7</URM> */
        } SEEIR;                /* <URM>DMASEEI</URM> , DMA Set Enable Error Interrupt Register */

        union {
            vuint8_t R;
            vuint8_t B;         /* <URM>NOP:1 CEEI:7</URM> */
        } CEEIR;                /* <URM>DMACEEI</URM> , DMA Clear Enable Error Interrupt Register */

        union {
            vuint8_t R;
            vuint8_t B;         /* <URM>NOP:1 CINT:7</URM> */
        } CIRQR;                /* <URM>DMACINT</URM> , DMA Clear Interrupt Request Register */

        union {
            vuint8_t R;
            vuint8_t B;         /* <URM>NOP:1 CERR:7</URM> */
        } CER;                  /* <URM>DMACERR</URM> , DMA Clear error Register */

        union {
            vuint8_t R;
            vuint8_t B;         /* <URM>NOP:1 SSRT:7</URM> */
        } SSBR;                 /* <URM>DMASSRT</URM> , Set Start Bit Register */

        union {
            vuint8_t R;
            vuint8_t B;         /* <URM>NOP:1 CDNE:7</URM> */
        } CDSBR;                /* <URM>DMACDNE</URM> , Clear Done Status Bit Register */

        uint32_t edma_reserved_irqrh;

        union {
            vuint32_t R;
            struct {
                vuint32_t INT31:1;
                vuint32_t INT30:1;
                vuint32_t INT29:1;
                vuint32_t INT28:1;
                vuint32_t INT27:1;
                vuint32_t INT26:1;
                vuint32_t INT25:1;
                vuint32_t INT24:1;
                vuint32_t INT23:1;
                vuint32_t INT22:1;
                vuint32_t INT21:1;
                vuint32_t INT20:1;
                vuint32_t INT19:1;
                vuint32_t INT18:1;
                vuint32_t INT17:1;
                vuint32_t INT16:1;
                vuint32_t INT15:1;
                vuint32_t INT14:1;
                vuint32_t INT13:1;
                vuint32_t INT12:1;
                vuint32_t INT11:1;
                vuint32_t INT10:1;
                vuint32_t INT09:1;
                vuint32_t INT08:1;
                vuint32_t INT07:1;
                vuint32_t INT06:1;
                vuint32_t INT05:1;
                vuint32_t INT04:1;
                vuint32_t INT03:1;
                vuint32_t INT02:1;
                vuint32_t INT01:1;
                vuint32_t INT00:1;
            } B;
        } IRQRL;                /* <URM>DMAINTL</URM> , DMA Interrupt Request Low */

        uint32_t edma_reserved_erh;

        union {
            vuint32_t R;
            struct {
                vuint32_t ERR31:1;
                vuint32_t ERR30:1;
                vuint32_t ERR29:1;
                vuint32_t ERR28:1;
                vuint32_t ERR27:1;
                vuint32_t ERR26:1;
                vuint32_t ERR25:1;
                vuint32_t ERR24:1;
                vuint32_t ERR23:1;
                vuint32_t ERR22:1;
                vuint32_t ERR21:1;
                vuint32_t ERR20:1;
                vuint32_t ERR19:1;
                vuint32_t ERR18:1;
                vuint32_t ERR17:1;
                vuint32_t ERR16:1;
                vuint32_t ERR15:1;
                vuint32_t ERR14:1;
                vuint32_t ERR13:1;
                vuint32_t ERR12:1;
                vuint32_t ERR11:1;
                vuint32_t ERR10:1;
                vuint32_t ERR09:1;
                vuint32_t ERR08:1;
                vuint32_t ERR07:1;
                vuint32_t ERR06:1;
                vuint32_t ERR05:1;
                vuint32_t ERR04:1;
                vuint32_t ERR03:1;
                vuint32_t ERR02:1;
                vuint32_t ERR01:1;
                vuint32_t ERR00:1;
            } B;
        } ERL;                  /* <URM>DMAERRL</URM> , DMA Error Low */

        int32_t edma_reserverd_hrsh[1];

        int32_t edma_reserverd_hrsl[1];

        int32_t edma_reserverd_gpor[1];

        int32_t EDMA_reserverd_223[49];

        union {
            vuint8_t R;
            struct {
                vuint8_t ECP:1;
                vuint8_t DPA:1;
                vuint8_t GRPPRI:2;
                vuint8_t CHPRI:4;
            } B;
        } CPR[64];              /* <URM>DCHPRI [32]</URM> , Channel n Priority */

        uint32_t edma_reserved2[944];

/****************************************************************************/
/*       DMA2 Transfer Control Descriptor                                   */
/****************************************************************************/

        struct tcd_t {          /*for "standard" format TCDs (when EDMA.TCD[x].CITER.E_LINK==BITER.E_LINK=0 && EDMA.EMLM=0 ) */
            vuint32_t SADDR;    /* source address */

            vuint16_t SMOD:5;   /* source address modulo */
            vuint16_t SSIZE:3;  /* source transfer size */
            vuint16_t DMOD:5;   /* destination address modulo */
            vuint16_t DSIZE:3;  /* destination transfer size */
            vint16_t SOFF;      /* signed source address offset */
            vuint32_t NBYTES;   /* inner (“minor”) byte count */
            vint32_t SLAST;     /* last destination address adjustment, or

                                   scatter/gather address (if e_sg = 1) */
            vuint32_t DADDR;    /* destination address */
            vuint16_t CITERE_LINK:1;
            vuint16_t CITER:15;
            vint16_t DOFF;      /* signed destination address offset */
            vint32_t DLAST_SGA;
            vuint16_t BITERE_LINK:1;    /* beginning ("major") iteration count */
            vuint16_t BITER:15;
            vuint16_t BWC:2;    /* bandwidth control */
            vuint16_t MAJORLINKCH:6;    /* enable channel-to-channel link */
            vuint16_t DONE:1;   /* channel done */
            vuint16_t ACTIVE:1; /* channel active */
            vuint16_t MAJORE_LINK:1;    /* enable channel-to-channel link */
            vuint16_t E_SG:1;   /* enable scatter/gather descriptor */
            vuint16_t D_REQ:1;  /* disable ipd_req when done */
            vuint16_t INT_HALF:1;       /* interrupt on citer = (biter >> 1) */
            vuint16_t INT_MAJ:1;        /* interrupt on major loop completion */
            vuint16_t START:1;  /* explicit channel start */
        } TCD[64];              /* <URM>TCD [32]</URM> , transfer_control_descriptor */
    };

    struct EDMA_TCD_alt1_tag {  /*for alternate format TCDs (when EDMA.TCD[x].CITER.E_LINK==BITER.E_LINK=1 ) */

        struct tcd_alt1_t {
            vuint32_t SADDR;    /* source address */

            vuint16_t SMOD:5;   /* source address modulo */
            vuint16_t SSIZE:3;  /* source transfer size */
            vuint16_t DMOD:5;   /* destination address modulo */
            vuint16_t DSIZE:3;  /* destination transfer size */
            vint16_t SOFF;      /* signed source address offset */
            vuint32_t NBYTES;   /* inner (“minor”) byte count */
            vint32_t SLAST;     /* last destination address adjustment, or

                                   scatter/gather address (if e_sg = 1) */
            vuint32_t DADDR;    /* destination address */
            vuint16_t CITERE_LINK:1;
            vuint16_t CITERLINKCH:6;
            vuint16_t CITER:9;
            vint16_t DOFF;      /* signed destination address offset */
            vint32_t DLAST_SGA;
            vuint16_t BITERE_LINK:1;    /* beginning (“major”) iteration count */
            vuint16_t BITERLINKCH:6;
            vuint16_t BITER:9;
            vuint16_t BWC:2;    /* bandwidth control */
            vuint16_t MAJORLINKCH:6;    /* enable channel-to-channel link */
            vuint16_t DONE:1;   /* channel done */
            vuint16_t ACTIVE:1; /* channel active */
            vuint16_t MAJORE_LINK:1;    /* enable channel-to-channel link */
            vuint16_t E_SG:1;   /* enable scatter/gather descriptor */
            vuint16_t D_REQ:1;  /* disable ipd_req when done */
            vuint16_t INT_HALF:1;       /* interrupt on citer = (biter >> 1) */
            vuint16_t INT_MAJ:1;        /* interrupt on major loop completion */
            vuint16_t START:1;  /* explicit channel start */
        } TCD[64];              /* <URM>TCD [32]</URM> , transfer_control_descriptor */
    };

/****************************************************************************/
/*                     MODULE : INTC                                      */
/****************************************************************************/
    struct INTC_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:18;   /* Reserved */
                vuint32_t VTES_PRC1:1;  /* Vector Table Entry Size for PRC1 (new in MPC563xM) */
                  vuint32_t:4;  /* Reserved */
                vuint32_t HVEN_PRC1:1;  /* Hardware Vector Enable for PRC1 (new in MPC563xM) */
                  vuint32_t:2;  /* Reserved */
                vuint32_t VTES:1;       /* Vector Table Entry Size for PRC0 <URM>VTES_PRC0</URM> */
                  vuint32_t:4;  /* Reserved */
                vuint32_t HVEN:1;       /* Hardware Vector Enable for PRC0 <URM>HVEN_PRC0</URM> */
            } B;
        } MCR;                  /* INTC Module Configuration Register (MCR) <URM>INTC_BCR</URM> @baseaddress + 0x00 */
        int32_t INTC_reserverd_10[1];

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;   /* Reserved */
                vuint32_t PRI:4;        /* Priority */
            } B;
        } CPR;                  /* INTC Current Priority Register for Processor 0 (CPR) <URM>INTC_CPR_PRC0</URM> @baseaddress + 0x08 */

        int32_t INTC_reserved_1;        /* CPR_PRC1 - INTC Current Priority Register for Processor 1 (CPR_PRC1) <URM>INTC_CPR_PRC1</URM> @baseaddress + 0x0c */

        union {
            vuint32_t R;
            struct {
                vuint32_t VTBA:21;      /* Vector Table Base Address <URM>VTBA_PRC0</URM> */
                vuint32_t INTVEC:9;     /* Interrupt Vector <URM>INTVEC_PRC0</URM> */
                  vuint32_t:2;  /* Reserved */
            } B;
        } IACKR;                /* INTC Interrupt Acknowledge Register for Processor 0 (IACKR) <URM>INTC_IACKR_PRC0</URM> @baseaddress + 0x10 */

        int32_t INTC_reserverd_2;       /* IACKR_PRC1 - INTC Interrupt Acknowledge Register for Processor 1 (IACKR_PRC1) <URM>INTC_IACKR_PRC1</URM> @baseaddress + 0x14 */

        union {
            vuint32_t R;
        } EOIR;                 /* INTC End of Interrupt Register for Processor 0 (EOIR) <URM>INTC_EOIR_PRC0</URM> @baseaddress + 0x18 */

        int32_t INTC_reserverd_3;       /* EOIR_PRC1 -  INTC End of Interrupt Register for Processor 1 (EOIR_PRC1) <URM>INTC_EOIR_PRC1</URM> @baseaddress + 0x1C */

        union {
            vuint8_t R;
            struct {
                vuint8_t:6;     /* Reserved */
                vuint8_t SET:1; /* Set Flag bits */
                vuint8_t CLR:1; /* Clear Flag bits */
            } B;
        } SSCIR[8];             /* INTC Software Set/Clear Interrupt Registers (SSCIR) <URM>INTC_SSCIRn</URM> @baseaddress + 0x20 */

        int32_t INTC_reserverd_32[6];

        union {
            vuint8_t R;
            struct {
                vuint8_t PRC_SEL:2;     /* Processor Select (new in MPC563xM) */
                  vuint8_t:2;   /* Reserved */
                vuint8_t PRI:4; /* Priority Select */
            } B;
        } PSR[512];             /* INTC Priority Select Registers (PSR) <URM>INTC_PSR</URM> @baseaddress + 0x40 */

    };                          /* end of INTC_tag */
/****************************************************************************/
/*                          MODULE : EQADC                                  */
/****************************************************************************/
    struct EQADC_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:24;
                vuint32_t ICEA0:1;
                vuint32_t ICEA1:1;
                  vuint32_t:1;
                vuint32_t ESSIE:2;
                  vuint32_t:1;
                vuint32_t DBG:2;
            } B;
        } MCR;                  /* Module Configuration Register <URM>EQADC_MCR</URM> */

        int32_t EQADC_reserved00;

        union {
            vuint32_t R;
            struct {
                vuint32_t:6;
                vuint32_t NMF:26;
            } B;
        } NMSFR;                /* Null Message Send Format Register <URM>EQADC_NMSFR</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t DFL:4;
            } B;
        } ETDFR;                /* External Trigger Digital Filter Register  <URM>EQADC_ETDFR</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t CFPUSH:32;    /* <URM>CF_PUSH</URM> */
            } B;
        } CFPR[6];              /* CFIFO Push Registers <URM>EQADC_CFPR</URM> */

        uint32_t eqadc_reserved1;

        uint32_t eqadc_reserved2;

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RFPOP:16;     /* <URM>RF_POP</URM> */
            } B;
        } RFPR[6];              /* Result FIFO Pop Registers <URM>EQADC_RFPR</URM> */

        uint32_t eqadc_reserved3;

        uint32_t eqadc_reserved4;

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t CFEE0:1;
                vuint16_t STRME0:1;
                vuint16_t SSE:1;
                vuint16_t CFINV:1;
                  vuint16_t:1;
                vuint16_t MODE:4;
                vuint16_t AMODE0:4;     /* CFIFO0 only */
            } B;
        } CFCR[6];              /* CFIFO Control Registers <URM>EQADC_CFCR</URM> */

        uint32_t eqadc_reserved5;

        union {
            vuint16_t R;
            struct {
                vuint16_t NCIE:1;
                vuint16_t TORIE:1;
                vuint16_t PIE:1;
                vuint16_t EOQIE:1;
                vuint16_t CFUIE:1;
                  vuint16_t:1;
                vuint16_t CFFE:1;
                vuint16_t CFFS:1;
                  vuint16_t:4;
                vuint16_t RFOIE:1;
                  vuint16_t:1;
                vuint16_t RFDE:1;
                vuint16_t RFDS:1;
            } B;
        } IDCR[6];              /* Interrupt and DMA Control Registers <URM>EQADC_IDCR</URM> */

        uint32_t eqadc_reserved6;

        union {
            vuint32_t R;
            struct {
                vuint32_t NCF:1;
                vuint32_t TORF:1;
                vuint32_t PF:1;
                vuint32_t EOQF:1;
                vuint32_t CFUF:1;
                vuint32_t SSS:1;
                vuint32_t CFFF:1;
                  vuint32_t:5;
                vuint32_t RFOF:1;
                  vuint32_t:1;
                vuint32_t RFDF:1;
                  vuint32_t:1;
                vuint32_t CFCTR:4;
                vuint32_t TNXTPTR:4;
                vuint32_t RFCTR:4;
                vuint32_t POPNXTPTR:4;
            } B;
        } FISR[6];              /* FIFO and Interrupt Status Registers <URM>EQADC_FISR</URM> */

        uint32_t eqadc_reserved7;

        uint32_t eqadc_reserved8;

        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t TCCF:11;      /* <URM>TC_CF</URM> */
            } B;
        } CFTCR[6];             /* CFIFO Transfer Counter Registers <URM>EQADC_CFTCR</URM> */

        uint32_t eqadc_reserved9;

        union {
            vuint32_t R;
            struct {
                vuint32_t CFS0:2;       /* <URM>CFS0_TCB0</URM> */
                vuint32_t CFS1:2;       /* <URM>CFS1_TCB0</URM> */
                vuint32_t CFS2:2;       /* <URM>CFS2_TCB0</URM> */
                vuint32_t CFS3:2;       /* <URM>CFS3_TCB0</URM> */
                vuint32_t CFS4:2;       /* <URM>CFS4_TCB0</URM> */
                vuint32_t CFS5:2;       /* <URM>CFS5_TCB0</URM> */
                  vuint32_t:5;
                vuint32_t LCFTCB0:4;
                vuint32_t TC_LCFTCB0:11;
            } B;
        } CFSSR0;               /* CFIFO Status Register 0 <URM>EQADC_CFSSR0</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t CFS0:2;       /* <URM>CFS0_TCB1</URM> */
                vuint32_t CFS1:2;       /* <URM>CFS1_TCB1</URM> */
                vuint32_t CFS2:2;       /* <URM>CFS2_TCB1</URM> */
                vuint32_t CFS3:2;       /* <URM>CFS3_TCB1</URM> */
                vuint32_t CFS4:2;       /* <URM>CFS4_TCB1</URM> */
                vuint32_t CFS5:2;       /* <URM>CFS5_TCB1</URM> */
                  vuint32_t:5;
                vuint32_t LCFTCB1:4;
                vuint32_t TC_LCFTCB1:11;
            } B;
        } CFSSR1;               /* CFIFO Status Register 1 <URM>EQADC_CFSSR1</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t CFS0:2;       /* <URM>CFS0_TSSI</URM> */
                vuint32_t CFS1:2;       /* <URM>CFS1_TSSI</URM> */
                vuint32_t CFS2:2;       /* <URM>CFS2_TSSI</URM> */
                vuint32_t CFS3:2;       /* <URM>CFS3_TSSI</URM> */
                vuint32_t CFS4:2;       /* <URM>CFS4_TSSI</URM> */
                vuint32_t CFS5:2;       /* <URM>CFS5_TSSI</URM> */
                  vuint32_t:4;
                vuint32_t ECBNI:1;
                vuint32_t LCFTSSI:4;
                vuint32_t TC_LCFTSSI:11;
            } B;
        } CFSSR2;               /* CFIFO Status Register 2 <URM>EQADC_CFSSR2</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t CFS0:2;
                vuint32_t CFS1:2;
                vuint32_t CFS2:2;
                vuint32_t CFS3:2;
                vuint32_t CFS4:2;
                vuint32_t CFS5:2;
                  vuint32_t:20;
            } B;
        } CFSR;                 /* <URM>EQADC_CFSR</URM> */

        uint32_t eqadc_reserved11;

        union {
            vuint32_t R;
            struct {
                vuint32_t:21;
                vuint32_t MDT:3;
                  vuint32_t:4;
                vuint32_t BR:4;
            } B;
        } SSICR;                /* SSI Control Register <URM>EQADC_SSICR</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t RDV:1;
                  vuint32_t:5;
                vuint32_t RDATA:26;
            } B;
        } SSIRDR;               /* SSI Recieve Data Register <URM>EQADC_SSIRDR</URM> @ baseaddress + 0xB8 */

        uint32_t eqadc_reserved11b[5];

        uint32_t eqadc_reserved15;      /* EQADC Red Line Client Configuration Register @ baseaddress + 0xD0  */
        /* REDLCCR is not implemented in the MPC563xM */

        uint32_t eqadc_reserved12[11];

        struct {
            union {
                vuint32_t R;

                /*<URM>B.CFIFOx_DATAw</URM> */

            } R[4];             /*<URM>EQADC_CFxRw<URM> */

            union {
                vuint32_t R;
                /*<URM>B.CFIFOx_EDATAw</URM> */
            } EDATA[4];         /*<URM>EQADC_CFxERw</URM> (new in MPC563xM) */

            uint32_t eqadc_reserved13[8];

        } CF[6];

        uint32_t eqadc_reserved14[32];

        struct {
            union {
                vuint32_t R;
                /*<URM>RFIFOx_DATAw</URM> */
            } R[4];             /*<URM>EQADC_RFxRw</URM> */

            uint32_t eqadc_reserved15[12];

        } RF[6];

    };
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
                vuint32_t PCSIS7:1;     /* new in MPC563xM */
                vuint32_t PCSIS6:1;     /* new in MPC563xM */
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
        } MCR;                  /* Module Configuration Register <URM>DSPI_MCR</URM> @baseaddress + 0x00 */

        uint32_t dspi_reserved1;

        union {
            vuint32_t R;
            struct {
                vuint32_t TCNT:16;      /* <URM>SPI_TCNT</URM> */
                  vuint32_t:16;
            } B;
        } TCR;                  /* DSPI Transfer Count Register  <URM>DSPI_TCR</URM> @baseaddress + 0x08 */

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
        } CTAR[8];              /* Clock and Transfer Attributes Registers <URM>DSPI_CTARx</URM> @baseaddress + 0x0C - 0x28 */

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
        } SR;                   /* Status Register <URM>DSPI_SR</URM> @baseaddress + 0x2C */

        union {
            vuint32_t R;
            struct {
                vuint32_t TCFRE:1;      /*<URM>TCF_RE</URM> */
                  vuint32_t:2;
                vuint32_t EOQFRE:1;     /*<URM>EQQF_RE</URM> */
                vuint32_t TFUFRE:1;     /*<URM>TFUF_RE</URM> */
                  vuint32_t:1;
                vuint32_t TFFFRE:1;     /*<URM>TFFF_RE</URM> */
                vuint32_t TFFFDIRS:1;   /*<URM>TFFF_DIRS</URM> */
                  vuint32_t:4;
                vuint32_t RFOFRE:1;     /*<URM>RFOF_RE</URM> */
                  vuint32_t:1;
                vuint32_t RFDFRE:1;     /*<URM>RFDF_RE</URM> */
                vuint32_t RFDFDIRS:1;   /*<URM>RFDF_DIRS</URM> */
                  vuint32_t:16;
            } B;
        } RSER;                 /* DMA/Interrupt Request Select and Enable Register <URM>DSPI_RSER</URM> @baseaddress + 0x30 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CONT:1;
                vuint32_t CTAS:3;
                vuint32_t EOQ:1;
                vuint32_t CTCNT:1;
                  vuint32_t:2;
                vuint32_t PCS7:1;       /* new in MPC563xM */
                vuint32_t PCS6:1;       /* new in MPC563xM */
                vuint32_t PCS5:1;
                vuint32_t PCS4:1;
                vuint32_t PCS3:1;
                vuint32_t PCS2:1;
                vuint32_t PCS1:1;
                vuint32_t PCS0:1;
                vuint32_t TXDATA:16;
            } B;
        } PUSHR;                /* PUSH TX FIFO Register <URM>DSPI_PUSHR</URM> @baseaddress + 0x34 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RXDATA:16;
            } B;
        } POPR;                 /* POP RX FIFO Register <URM>DSPI_POPR</URM> @baseaddress + 0x38 */

        union {
            vuint32_t R;
            struct {
                vuint32_t TXCMD:16;
                vuint32_t TXDATA:16;
            } B;
        } TXFR[4];              /* Transmit FIFO Registers <URM>DSPI_TXFRx</URM> @baseaddress + 0x3c - 0x78 */

        vuint32_t DSPI_reserved_txf[12];

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RXDATA:16;
            } B;
        } RXFR[4];              /* Transmit FIFO Registers <URM>DSPI_RXFRx</URM> @baseaddress + 0x7c - 0xB8 */

        vuint32_t DSPI_reserved_rxf[12];

        union {
            vuint32_t R;
            struct {
                vuint32_t MTOE:1;
                  vuint32_t:1;
                vuint32_t MTOCNT:6;
                  vuint32_t:3;
                vuint32_t TSBC:1;
                vuint32_t TXSS:1;
                vuint32_t TPOL:1;
                vuint32_t TRRE:1;
                vuint32_t CID:1;
                vuint32_t DCONT:1;
                vuint32_t DSICTAS:3;
                  vuint32_t:4;
                vuint32_t DPCS7:1;
                vuint32_t DPCS6:1;
                vuint32_t DPCS5:1;
                vuint32_t DPCS4:1;
                vuint32_t DPCS3:1;
                vuint32_t DPCS2:1;
                vuint32_t DPCS1:1;
                vuint32_t DPCS0:1;
            } B;
        } DSICR;                /* DSI Configuration Register <URM>DSPI_DSICR</URM> @baseaddress + 0xBC */

        union {
            vuint32_t R;
            struct {
                vuint32_t SER_DATA:32;  /* 32bit instead of 16 in MPC563xM */
            } B;
        } SDR;                  /* DSI Serialization Data Register <URM>DSPI_SDR</URM> @baseaddress + 0xC0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t ASER_DATA:32; /* 32bit instead of 16 in MPC563xM */
            } B;
        } ASDR;                 /* DSI Alternate Serialization Data Register <URM>DSPI_ASDR</URM> @baseaddress + 0xC4 */

        union {
            vuint32_t R;
            struct {
                vuint32_t COMP_DATA:32; /* 32bit instead of 16 in MPC563xM */
            } B;
        } COMPR;                /* DSI Transmit Comparison Register <URM>DSPI_COMPR</URM> @baseaddress + 0xC8 */

        union {
            vuint32_t R;
            struct {
                vuint32_t DESER_DATA:32;        /* 32bit instead of 16 in MPC563xM */
            } B;
        } DDR;                  /* DSI deserialization Data Register <URM>DSPI_DDR</URM> @baseaddress + 0xCC */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t TSBCNT:5;
                  vuint32_t:16;
                vuint32_t DPCS1_7:1;
                vuint32_t DPCS1_6:1;
                vuint32_t DPCS1_5:1;
                vuint32_t DPCS1_4:1;
                vuint32_t DPCS1_3:1;
                vuint32_t DPCS1_2:1;
                vuint32_t DPCS1_1:1;
                vuint32_t DPCS1_0:1;
            } B;
        } DSICR1;               /* DSI Configuration Register 1 <URM>DSPI_DSICR1</URM> @baseaddress + 0xD0 */

    };
/****************************************************************************/
/*                          MODULE : eSCI                                   */
/****************************************************************************/
    struct ESCI_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t SBR:13;
                vuint32_t LOOPS:1;
                  vuint32_t:1;  /* Reserved in MPC563xM */
                vuint32_t RSRC:1;
                vuint32_t M:1;
                vuint32_t WAKE:1;
                vuint32_t ILT:1;
                vuint32_t PE:1;
                vuint32_t PT:1;
                vuint32_t TIE:1;
                vuint32_t TCIE:1;
                vuint32_t RIE:1;
                vuint32_t ILIE:1;
                vuint32_t TE:1;
                vuint32_t RE:1;
                vuint32_t RWU:1;
                vuint32_t SBK:1;
            } B;
        } CR1;                  /* Control Register 1 <URM>SCIBDH, SCIBDL, SCICR1, SCICR2</URM> @baseaddress + 0x00 */

        union {
            vuint16_t R;
            struct {
                vuint16_t MDIS:1;
                vuint16_t FBR:1;
                vuint16_t BSTP:1;
                vuint16_t IEBERR:1;     /* <URM>BERIE</URM> */
                vuint16_t RXDMA:1;
                vuint16_t TXDMA:1;
                vuint16_t BRK13:1;      /* <URM>BRCL</URM> */
                vuint16_t TXDIR:1;
                vuint16_t BESM13:1;     /* <URM>BESM</URM> */
                vuint16_t SBSTP:1;      /* <URM>BESTP</URM> */
                vuint16_t RXPOL:1;
                vuint16_t PMSK:1;
                vuint16_t ORIE:1;
                vuint16_t NFIE:1;
                vuint16_t FEIE:1;
                vuint16_t PFIE:1;
            } B;
        } CR2;                  /* Control Register 2 <URM>SCICR3, SCICR4</URM>  @baseaddress + 0x04 */

        union {
            vuint16_t R;
            struct {
                vuint16_t R8:1; /* <URM>RN</URM> */
                vuint16_t T8:1; /* <URM>TN</URM> */
                vuint16_t ERR:1;
                  vuint16_t:1;
                vuint16_t R:4;
                vuint8_t D;
            } B;
        } DR;                   /* Data Register <URM>SCIDRH, SCIDRL</URM>  @baseaddress + 0x06 */

        union {
            vuint32_t R;
            struct {
                vuint32_t TDRE:1;
                vuint32_t TC:1;
                vuint32_t RDRF:1;
                vuint32_t IDLE:1;
                vuint32_t OR:1;
                vuint32_t NF:1;
                vuint32_t FE:1;
                vuint32_t PF:1;
                  vuint32_t:3;
                vuint32_t BERR:1;
                  vuint32_t:2;
                vuint32_t TACT:1;
                vuint32_t RAF:1;        /* <URM>RACT</URM> */
                vuint32_t RXRDY:1;
                vuint32_t TXRDY:1;
                vuint32_t LWAKE:1;
                vuint32_t STO:1;
                vuint32_t PBERR:1;
                vuint32_t CERR:1;
                vuint32_t CKERR:1;
                vuint32_t FRC:1;
                  vuint32_t:6;
                vuint32_t UREQ:1;
                vuint32_t OVFL:1;
            } B;
        } SR;                   /* Status Register <URM>SCISR1, SCIRSR2, LINSTAT1, LINSTAT2 </URM>  @baseaddress + 0x08 */

        union {
            vuint32_t R;
            struct {
                vuint32_t LRES:1;
                vuint32_t WU:1;
                vuint32_t WUD0:1;
                vuint32_t WUD1:1;
                  vuint32_t:2;  /* reserved: LDBG and DSF not longer supported */
                vuint32_t PRTY:1;
                vuint32_t LIN:1;
                vuint32_t RXIE:1;
                vuint32_t TXIE:1;
                vuint32_t WUIE:1;
                vuint32_t STIE:1;
                vuint32_t PBIE:1;
                vuint32_t CIE:1;
                vuint32_t CKIE:1;
                vuint32_t FCIE:1;
                  vuint32_t:6;
                vuint32_t UQIE:1;
                vuint32_t OFIE:1;
                  vuint32_t:8;
            } B;
        } LCR;                  /* LIN Control Register <URM>LINCTRL1, LINCTRL2, LINCTRL3 </URM>  @baseaddress + 0x0C  */

        union {
            vuint32_t R;
        } LTR;                  /* LIN Transmit Register <URM>LINTX</URM>  @baseaddress + 0x10 */

        union {
            vuint32_t R;
        } LRR;                  /* LIN Recieve Register <URM>LINRX</URM>  @baseaddress + 0x14 */

        union {
            vuint32_t R;
            struct {
                vuint32_t P:16;
                  vuint32_t:3;
                vuint32_t SYNM:1;
                vuint32_t EROE:1;
                vuint32_t ERFE:1;
                vuint32_t ERPE:1;
                vuint32_t M2:1;
                  vuint32_t:8;
            } B;
        } LPR;                  /* LIN CRC Polynom Register  <URM>LINCRCP1, LINCRCP2, SCICR5</URM>  @baseaddress + 0x18 */

    };
/****************************************************************************/
/*                          MODULE : eSCI                                   */
/****************************************************************************/
    struct ESCI_12_13_bit_tag {
        union {
            vuint16_t R;
            struct {
                vuint16_t R8:1;
                vuint16_t T8:1;
                vuint16_t ERR:1;
                  vuint16_t:1;
                vuint16_t D:12;
            } B;
        } DR;                   /* Data Register */
    };
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
            /*vuint8_t  B[8]; *//* Data buffer in Bytes (8 bits) *//* Not used in MPC563xM */
            /*vuint16_t H[4]; *//* Data buffer in Half-words (16 bits) *//* Not used in MPC563xM */
            vuint32_t W[2];     /* Data buffer in words (32 bits) */
            /*vuint32_t R[2]; *//* Data buffer in words (32 bits) *//* Not used in MPC563xM */
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
            /*vuint8_t  B[8]; *//* Data buffer in Bytes (8 bits) *//* Not used in MPC563xM */
            /*vuint16_t H[4]; *//* Data buffer in Half-words (16 bits) *//* Not used in MPC563xM */
            vuint32_t W[2];     /* Data buffer in words (32 bits) */
            /*vuint32_t R[2]; *//* Data buffer in words (32 bits) *//* Not used in MPC563xM */
        } DATA;

        uint32_t FLEXCAN_RXFIFO_reserved[20];   /* {0x00E0-0x0090}/0x4 = 0x14 */

        union {
            vuint32_t R;
        } IDTABLE[8];

    };                          /* end of FLEXCAN_RXFIFO_t */

    struct FLEXCAN2_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t MDIS:1;
                vuint32_t FRZ:1;
                vuint32_t FEN:1;        /* new in MPC563xM */
                vuint32_t HALT:1;
                vuint32_t NOTRDY:1;     /* <URM>NOT_RDY</URM> */
                vuint32_t WAK_MSK:1;    /* new in MPC563xM */
                vuint32_t SOFTRST:1;    /* <URM>SOFT_RST</URM> */
                vuint32_t FRZACK:1;     /* <URM>FRZ_ACK</URM> */
                vuint32_t SUPV:1;       /* new in MPC563xM */
                vuint32_t SLF_WAK:1;    /* new in MPC563xM */

                vuint32_t WRNEN:1;      /* <URM>WRN_EN</URM> */

                vuint32_t MDISACK:1;    /* <URM>LPM_ACK</URM> */
                vuint32_t WAK_SRC:1;    /* new in MPC563xM */
                vuint32_t DOZE:1;       /* new in MPC563xM */

                vuint32_t SRXDIS:1;     /* <URM>SRX_DIS</URM> */
                vuint32_t MBFEN:1;      /* <URM>BCC</URM> */
                  vuint32_t:2;

                vuint32_t LPRIO_EN:1;   /* new in MPC563xM */
                vuint32_t AEN:1;        /* new in MPC563xM */
                  vuint32_t:2;
                vuint32_t IDAM:2;       /* new in MPC563xM */
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
                vuint32_t BOFFMSK:1;    /* <URM>BOFF_MSK</URM> */
                vuint32_t ERRMSK:1;     /* <URM>ERR_MSK</URM> */
                vuint32_t CLKSRC:1;     /* <URM>CLK_SRC</URM> */
                vuint32_t LPB:1;
                vuint32_t TWRNMSK:1;    /* <URM>TWRN_MSK</URM> */
                vuint32_t RWRNMSK:1;    /* <URM>RWRN_MSK</URM> */
                  vuint32_t:2;
                vuint32_t SMP:1;
                vuint32_t BOFFREC:1;    /* <URM>BOFF_REC</URM> */
                vuint32_t TSYN:1;
                vuint32_t LBUF:1;
                vuint32_t LOM:1;
                vuint32_t PROPSEG:3;
            } B;                /* Control Register */
        } CR;                   /* <URM>CTRL</URM> */

        union {
            vuint32_t R;
        } TIMER;                /* Free Running Timer */

        int32_t FLEXCAN_reserved00;

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t MI:29;
            } B;
        } RXGMASK;              /* RX Global Mask */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t MI:29;
            } B;
        } RX14MASK;             /* RX 14 Mask */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t MI:29;
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
                vuint32_t TWRNINT:1;    /* <URM>TWRN_INT</URM> */
                vuint32_t RWRNINT:1;    /* <URM>RWRN_INT</URM> */
                vuint32_t BIT1ERR:1;    /* <URM>BIT1_ERR</URM> */
                vuint32_t BIT0ERR:1;    /* <URM>BIT0_ERR</URM> */
                vuint32_t ACKERR:1;     /* <URM>ACK_ERR</URM> */
                vuint32_t CRCERR:1;     /* <URM>CRC_ERR</URM> */
                vuint32_t FRMERR:1;     /* <URM>FRM_ERR</URM> */
                vuint32_t STFERR:1;     /* <URM>STF_ERR</URM> */
                vuint32_t TXWRN:1;      /* <URM>TX_WRN</URM> */
                vuint32_t RXWRN:1;      /* <URM>RX_WRN</URM> */
                vuint32_t IDLE:1;
                vuint32_t TXRX:1;
                vuint32_t FLTCONF:2;    /* <URM>FLT_CONF</URM> */
                  vuint32_t:1;
                vuint32_t BOFFINT:1;    /* <URM>BOFF_INT</URM> */
                vuint32_t ERRINT:1;     /* <URM>ERR_INT</URM> */
                vuint32_t WAK_INT:1;    /* new in MPC563xM */
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
            } B;                /* Interruput Masks Register */
        } IMRH;                 /* <URM>IMASK2</URM> */

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
            } B;                /* Interruput Masks Register */
        } IMRL;                 /* <URM>IMASK1</URM> */

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
            } B;                /* Interruput Flag Register */
        } IFRH;                 /* <URM>IFLAG2</URM> */

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
            } B;                /* Interruput Flag Register */
        } IFRL;                 /* <URM>IFLAG1</URM> */

        uint32_t flexcan2_reserved2[19];

/****************************************************************************/
/* Use either Standard Buffer Structure OR RX FIFO and Buffer Structure     */
/****************************************************************************/
        /* Standard Buffer Structure */
        struct FLEXCAN_BUF_t BUF[64];

        /* RX FIFO and Buffer Structure *//* New options in MPC563xM */
        /*struct FLEXCAN_RXFIFO_t RXFIFO; */
        /*struct FLEXCAN_BUF_t BUF[56];   */
/****************************************************************************/

        uint32_t FLEXCAN_reserved3[256];        /* {0x0880-0x0480}/0x4 = 0x100 *//* (New in MPC563xM) Address Base + 0x0034 */

        union {
            vuint32_t R;
            struct {
                vuint32_t MI:32;
            } B;                /* RX Individual Mask Registers */
        } RXIMR[64];            /* (New in MPC563xM) Address Base + 0x0880 */

    };                          /* end of FLEXCAN_tag */
/****************************************************************************/
/*                     MODULE : Decimation Filter (DECFIL)                  */
/****************************************************************************/
    struct DECFIL_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t MDIS:1;
                vuint32_t FREN:1;
                  vuint32_t:1;
                vuint32_t FRZ:1;
                vuint32_t SRES:1;
                  vuint32_t:2;  /* CASCD not supported in MPC563xM */
                vuint32_t IDEN:1;
                vuint32_t ODEN:1;
                vuint32_t ERREN:1;
                  vuint32_t:1;
                vuint32_t FTYPE:2;
                  vuint32_t:1;
                vuint32_t SCAL:2;
                  vuint32_t:1;
                vuint32_t SAT:1;
                vuint32_t ISEL:1;
                  vuint32_t:1;  /* MIXM does not appear to be implemented on the MPC563xM */
                vuint32_t DEC_RATE:4;
                  vuint32_t:1;  /* SDIE not supported in MPC563xM */
                vuint32_t DSEL:1;
                vuint32_t IBIE:1;
                vuint32_t OBIE:1;
                vuint32_t EDME:1;
                vuint32_t TORE:1;
                vuint32_t TMODE:2;      /* the LSB of TMODE is always 0 on the MPC563xM */
            } B;
        } MCR;                  /* Configuration Register <URM>DECFILTER_MCR</URM> @baseaddress + 0x00 */

        union {
            vuint32_t R;
            struct {
                vuint32_t BSY:1;
                  vuint32_t:1;
                vuint32_t DEC_COUNTER:4;
                vuint32_t IDFC:1;
                vuint32_t ODFC:1;
                vuint32_t SDFC:1;       /* SDFC not supported in MPC563xM */
                vuint32_t IBIC:1;
                vuint32_t OBIC:1;
                vuint32_t SVRC:1;       /* SVRC not supported in MPC563xM */
                vuint32_t DIVRC:1;
                vuint32_t OVFC:1;
                vuint32_t OVRC:1;
                vuint32_t IVRC:1;
                  vuint32_t:6;
                vuint32_t IDF:1;
                vuint32_t ODF:1;
                vuint32_t SDF:1;        /* SDF not supported in MPC563xM */
                vuint32_t IBIF:1;
                vuint32_t OBIF:1;
                vuint32_t SVR:1;        /* SVR not supported in MPC563xM */
                vuint32_t DIVR:1;
                vuint32_t OVF:1;
                vuint32_t OVR:1;
                vuint32_t IVR:1;
            } B;
        } MSR;                  /* Status Register <URM>DECFILTER_MSR</URM> @baseaddress + 0x04 */

        /* Module Extended Config.Register - not siupported on the MPC563xM <URM>DECFILTER_MXCR</URM> @baseaddress + 0x08  */

        uint32_t decfil_reserved1[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:4;
                vuint32_t INTAG:4;
                  vuint32_t:6;
                vuint32_t PREFILL:1;
                vuint32_t FLUSH:1;
                vuint32_t INPBUF:16;
            } B;
        } IB;                   /* Interface Input Buffer <URM>DECFILTER_IB</URM> @baseaddress + 0x10  */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t OUTTAG:4;
                vuint32_t OUTBUF:16;
            } B;
        } OB;                   /* Interface Output Buffer <URM>DECFILTER_OB</URM> @baseaddress + 0x14  */

        uint32_t decfil_reserved2[2];

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t COEF:24;
            } B;
        } COEF[9];              /* Filter Coefficient Registers <URM>DECFILTER_COEFx</URM> @baseaddress + 0x20 - 0x40  */

        uint32_t decfil_reserved3[13];

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t TAP:24;
            } B;
        } TAP[8];               /* Filter TAP Registers <URM>DECFILTER_TAPx</URM> @baseaddress + 0x78 - 0x94 */

        uint32_t decfil_reserved4[14];

        /* 0x0D0 */
        union {
            vuint16_t R;
            struct {
                vuint32_t:16;
                vuint32_t SAMP_DATA:16;
            } B;
        } EDID;                 /* Filter EDID Registers <URM>DECFILTER_EDID</URM> @baseaddress + 0xD0 */

        uint32_t decfil_reserved5[3];

        /* 0x0E0 */
        uint32_t decfil_reserved6;
        /* Filter FINTVAL Registers - Not supported on MPC563xM <URM>DECFILTER_FINTVAL</URM> @baseaddress + 0xE0 */

        /* 0x0E4 */
        uint32_t decfil_reserved7;
        /* Filter FINTCNT Registers - Not supported on MPC563xM <URM>DECFILTER_FINTCNT</URM> @baseaddress + 0xE4 */

        /* 0x0E8 */
        uint32_t decfil_reserved8;
        /* Filter CINTVAL Registers - Not supported on MPC563xM <URM>DECFILTER_CINTVAL</URM> @baseaddress + 0xE8 */

        /* 0x0EC */
        uint32_t decfil_reserved9;
        /* Filter CINTCNT Registers - Not supported on MPC563xM <URM>DECFILTER_CINTCNT</URM> @baseaddress + 0xEC */

    };
/****************************************************************************/
/*                     MODULE : Periodic Interval Timer (PIT)               */
/****************************************************************************/
    struct PIT_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t:29;
                vuint32_t MDIS_RTI:1;
                vuint32_t MDIS:1;
                vuint32_t FRZ:1;
            } B;
        } PITMCR;               /* PIT Module Control Register */

        uint32_t pit_reserved1[59];

        struct {
            union {
                vuint32_t R;    /* <URM>TSVn</URM> */
            } LDVAL;            /* Timer Load Value Register */

            union {
                vuint32_t R;    /* <URM>TVLn</URM> */
            } CVAL;             /* Current Timer Value Register */

            union {
                vuint32_t R;
                struct {
                    vuint32_t:30;
                    vuint32_t TIE:1;
                    vuint32_t TEN:1;
                } B;
            } TCTRL;            /* Timer Control Register */

            union {
                vuint32_t R;
                struct {
                    vuint32_t:31;
                    vuint32_t TIF:1;
                } B;
            } TFLG;             /* Timer Flag Register */
        } RTI;                  /* RTI Channel */

        struct {
            union {
                vuint32_t R;
            } LDVAL;            /* Timer Load Value Register */

            union {
                vuint32_t R;
            } CVAL;             /* Current Timer Value Register */

            union {
                vuint32_t R;
                struct {
                    vuint32_t:30;
                    vuint32_t TIE:1;
                    vuint32_t TEN:1;
                } B;
            } TCTRL;            /* Timer Control Register */

            union {
                vuint32_t R;
                struct {
                    vuint32_t:31;
                    vuint32_t TIF:1;
                } B;
            } TFLG;             /* Timer Flag Register */
        } TIMER[4];             /* Timer Channels */

    };
/****************************************************************************/
/*                     MODULE : System Timer Module (STM)                   */
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
        } CR;                   /* STM Control Register <URM>STM_CR</URM> (new in MPC563xM) Offset 0x0000 */

        union {
            vuint32_t R;
        } CNT;                  /* STM Count Register <URM>STM_CNT</URM> (new in MPC563xM) Offset Offset 0x0004 */

        uint32_t stm_reserved1[2];      /* Reserved (new in MPC563xM) Offset Offset 0x0008 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR0;                 /* STM Channel Control Register <URM>STM_CCR0</URM> (new in MPC563xM) Offset 0x0010 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR0;                 /* STM Channel Interrupt Register <URM>STM_CIR0</URM> (new in MPC563xM) Offset 0x0014 */

        union {
            vuint32_t R;
        } CMP0;                 /* STM Channel Compare Register <URM>STM_CMP0</URM> (new in MPC563xM) Offset Offset 0x0018 */

        uint32_t stm_reserved2; /* Reserved (new in MPC563xM) Offset Offset 0x001C */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR1;                 /* STM Channel Control Register <URM>STM_CCR1</URM> (new in MPC563xM) Offset 0x0020 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR1;                 /* STM Channel Interrupt Register <URM>STM_CIR1</URM> (new in MPC563xM) Offset 0x0024 */

        union {
            vuint32_t R;
        } CMP1;                 /* STM Channel Compare Register <URM>STM_CMP1</URM> (new in MPC563xM) Offset Offset 0x0028 */

        uint32_t stm_reserved3; /* Reserved (new in MPC563xM) Offset Offset 0x002C */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR2;                 /* STM Channel Control Register <URM>STM_CCR2</URM> (new in MPC563xM) Offset 0x0030 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR2;                 /* STM Channel Interrupt Register <URM>STM_CIR2</URM> (new in MPC563xM) Offset 0x0034 */

        union {
            vuint32_t R;
        } CMP2;                 /* STM Channel Compare Register <URM>STM_CMP2</URM> (new in MPC563xM) Offset Offset 0x0038 */

        uint32_t stm_reserved4; /* Reserved (new in MPC563xM) Offset Offset 0x003C */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR3;                 /* STM Channel Control Register <URM>STM_CCR3</URM> (new in MPC563xM) Offset 0x0040 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR3;                 /* STM Channel Interrupt Register <URM>STM_CIR3</URM> (new in MPC563xM) Offset 0x0044 */

        union {
            vuint32_t R;
        } CMP3;                 /* STM Channel Compare Register <URM>STM_CMP3</URM> (new in MPC563xM) Offset Offset 0x0048 */

        uint32_t stm_reserved5; /* Reserved (new in MPC563xM) Offset Offset 0x004C */
    };

/****************************************************************************/
/*                     MODULE : SWT                                         */
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
                  vuint32_t:14;
                vuint32_t KEY:1;
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
        } MCR;                  /*<URM>SWT_CR</URM> *//* Module Configuration Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t TIF:1;
            } B;
        } IR;                   /* Interrupt register <URM>SWT_IR</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t WTO:32;
            } B;
        } TO;                   /* Timeout register <URM>SWT_TO</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t WST:32;

            } B;
        } WN;                   /* Window register <URM>SWT_WN</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t WSC:16;
            } B;
        } SR;                   /* Service register <URM>SWT_SR</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t CNT:32;
            } B;
        } CO;                   /* Counter output register <URM>SWT_CO</URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t SK:16;
            } B;
        } SK;                   /* Service key register <URM>SWT_SK</URM> */
    };
/****************************************************************************/
/*                     MODULE : Power Management Controller (PMC)           */
/****************************************************************************/
    struct PMC_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t LVRER:1;      /* <URM> LVIRR </URM> */
                vuint32_t LVREH:1;      /* <URM> LVIHR </URM> */
                vuint32_t LVRE50:1;     /* <URM> LVI5R </URM> */
                vuint32_t LVRE33:1;     /* <URM> LVI3R </URM> */
                vuint32_t LVREC:1;      /* <URM> LVI1R </URM> */
                  vuint32_t:3;
                vuint32_t LVIER:1;      /* <URM> LVIRE </URM> */
                vuint32_t LVIEH:1;      /* <URM> LVIHE </URM> */
                vuint32_t LVIE50:1;     /* <URM> LVI5E </URM> */
                vuint32_t LVIE33:1;     /* <URM> LVI3E </URM> */
                vuint32_t LVIC:1;       /* <URM> LVI1E </URM> */
                  vuint32_t:2;
                vuint32_t TLK:1;
                  vuint32_t:16;
            } B;
        } MCR;                  /* Module Configuration register <URM> CFGR </URM> */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t LVDREGTRIM:4; /* <URM> LVI50TRIM </URM> */
                vuint32_t VDD33TRIM:4;  /* <URM> BV33TRIM </URM> */
                vuint32_t LVD33TRIM:4;  /* <URM> LVI33TRIM </URM> */
                vuint32_t VDDCTRIM:4;   /* <URM> V12TRIM </URM> */
                vuint32_t LVDCTRIM:4;   /* <URM> LVI33TRIM </URM> */
            } B;
        } TRIMR;                /* Trimming register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:5;
                vuint32_t LVFVSTBY:1;
                vuint32_t BGRDY:1;      /* <URM> BGS1 </URM> */
                vuint32_t BGTS:1;       /* <URM> BGS2 </URM> */
                  vuint32_t:5;
                vuint32_t LVFCSTBY:1;
                  vuint32_t:1;
                vuint32_t V33DIS:1;     /* 3.3V Regulator Disable <URM> V33S </URM> */
                vuint32_t LVFCR:1;      /* Clear LVFR <URM> LVIRC </URM> */
                vuint32_t LVFCH:1;      /* Clear LVFH <URM> LVIHC </URM> */
                vuint32_t LVFC50:1;     /* Clear LVF5 <URM> LVI5 </URM> */
                vuint32_t LVFC33:1;     /* Clear LVF3 <URM> LVI3 </URM> */
                vuint32_t LVFCC:1;      /* Clear LVFC <URM> LVI1 </URM> */
                  vuint32_t:3;
                vuint32_t LVFR:1;       /* Low Voltage Flag Reset Supply <URM> LVIRF </URM> */
                vuint32_t LVFH:1;       /* Low Voltage Flag VDDEH Supply <URM> LVIHF </URM> */
                vuint32_t LVF50:1;      /* Low Voltage Flag 5V Supply <URM> LVI5F </URM> */
                vuint32_t LVF33:1;      /* Low Voltage Flag 3.3V Supply <URM> LVI3F </URM> */
                vuint32_t LVFC:1;       /* Low Voltage Flag Core (1.2V) <URM> LVI1F </URM> */
                  vuint32_t:3;

            } B;
        } SR;                   /* status register */
    };
/****************************************************************************/
/*                          MODULE : TSENS (Temperature Sensor)             */
/****************************************************************************/

    struct TSENS_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t TSCV2:16;
                vuint32_t TSCV1:16;
            } B;
        } TCCR0;                /* Temperature Sensor Calibration B @baseaddress + 0x00 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t TSCV3:16;
            } B;
        } TCCR1;                /* Temperature Sensor Calibration A @baseaddress + 0x04 */

        uint32_t TSENS_reserved0008[16382];     /* 0x0008-0xFFFF */

    };
     
/* Define memories */ 
/* Comments need to be moved for different memory sizes */ 
     
#define SRAM_START    0x40000000
        /*#define SRAM_SIZE     0xC000        48K SRAM */ 
        /*#define SRAM_SIZE     0x10000       64K SRAM */ 
#define SRAM_SIZE     0x17800        /* 94K SRAM */
        /*#define SRAM_END      0x4000BFFF    48K SRAM */ 
        /*#define SRAM_END      0x4000FFFF    64K SRAM */ 
#define SRAM_END      0x400177FF     /* 94K SRAM */
     
#define FLASH_START   0x0
        /*#define FLASH_SIZE  0x100000          1M Flash */ 
#define FLASH_SIZE    0x180000       /* 1.5M Flash */
        /*#define FLASH_END   0xFFFFF           1M Flash */ 
#define FLASH_END     0x17FFFF       /* 1.5M Flash */
     
/* Shadow Flash start and end address */ 
#define FLASH_SHADOW_START 0x00FFC000  
#define FLASH_SHADOW_SIZE  0x4000
#define FLASH_SHADOW_END 0x00FFFFFF
     
/* Define instances of modules */ 
#define FMPLL     (*( volatile struct FMPLL_tag *)     0xC3F80000)
#define EBI       (*( volatile struct EBI_tag *)       0xC3F84000)
#define CFLASH0   (*( volatile struct FLASH_tag *)     0xC3F88000)
#define CFLASH1   (*( volatile struct FLASH_tag *)     0xC3FB0000)
#define CFLASH2   (*( volatile struct FLASH_tag *)     0xC3FB4000)
#define SIU       (*( volatile struct SIU_tag *)       0xC3F90000)
     
#define EMIOS     (*( volatile struct EMIOS_tag *)     0xC3FA0000)
#define PMC       (*( volatile struct PMC_tag *)       0xC3FBC000)
#define ETPU      (*( volatile struct ETPU_tag *)      0xC3FC0000)
#define ETPU_DATA_RAM  (*( uint32_t *)                 0xC3FC8000)
#define ETPU_DATA_RAM_EXT  (*( uint32_t *)             0xC3FCC000)
#define ETPU_DATA_RAM_END                 0xC3FC8BFC
#define CODE_RAM       (*( uint32_t *)                 0xC3FD0000)
#define ETPU_CODE_RAM  (*( uint32_t *)                 0xC3FD0000)
#define PIT       (*( volatile struct PIT_tag *)       0xC3FF0000)
     
#define XBAR      (*( volatile struct XBAR_tag *)      0xFFF04000)
#define SWT       (*( volatile struct SWT_tag *)       0xFFF38000)
#define STM       (*( volatile struct STM_tag *)       0xFFF3C000)
#define ECSM       (*( volatile struct ECSM_tag *)     0xFFF40000)
#define EDMA      (*( volatile struct EDMA_tag *)      0xFFF44000)
#define INTC      (*( volatile struct INTC_tag *)      0xFFF48000)
     
#define EQADC     (*( volatile struct EQADC_tag *)     0xFFF80000)
#define DECFIL    (*( volatile struct DECFIL_tag *)    0xFFF88000)
     
#define DSPI_B    (*( volatile struct DSPI_tag *)      0xFFF94000)
#define DSPI_C    (*( volatile struct DSPI_tag *)      0xFFF98000)
     
#define ESCI_A    (*( volatile struct ESCI_tag *)      0xFFFB0000)
#define ESCI_A_12_13    (*( volatile struct ESCI_12_13_bit_tag *)      0xFFFB0006)
#define ESCI_B    (*( volatile struct ESCI_tag *)      0xFFFB4000)
#define ESCI_B_12_13    (*( volatile struct ESCI_12_13_bit_tag *)      0xFFFB4006)
     
#define CAN_A     (*( volatile struct FLEXCAN2_tag *)  0xFFFC0000)
#define CAN_C     (*( volatile struct FLEXCAN2_tag *)  0xFFFC8000)
     
#define TSENS    (*( volatile struct TSENS_tag *)      0xFFFEC000)
     
#ifdef __MWERKS__
#pragma pop
#endif  /* 
 */
     
#ifdef  __cplusplus
} 
#endif  /* 
 */
 
#endif  /* ifdef _MPC563M_H */
/*********************************************************************
 *
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
 ********************************************************************/ 
 
