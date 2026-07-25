/**************************************************************************/
/* FILE NAME: MPC5644A.h      COPYRIGHT(c) Freescale & STMicroelectronics */
/* VERSION:  0.5                           2010 - All Rights Reserved     */
/*                                                                        */
/* DESCRIPTION:                                                           */
/* This file contains all of the register and bit field definitions for   */
/* MPC5644A.                                                              */
/*========================================================================*/
/* UPDATE HISTORY                                                         */
/* REV      AUTHOR      DATE       DESCRIPTION OF CHANGE                  */
/* ---   -----------  ---------    ---------------------                  */
/* 0.1   R. MORAN     10/Aug/09    Initial Alpha version.                 */
/* 0.2   R. Moran     09/Nov/09    Several Updates:                       */
/*                                 - XBAR - MPR/SGPCR'3' changed to '2'   */
/*                                 - CFCR, IDCR, CFTCR array sizes altered*/
/*                                 - IDIS,CASCD added to DEC_Filter MCR   */
/*                                 - WDM fields changed in eTPU_WDTR reg  */
/*                                 - Additional ECSM registers added      */
/* 0.3   R. Moran     14/Jan/10    Several Updates:                       */
/*                                 - Flash User Test Register implemented */
/*                                 - MPU.EDR[3] register removed          */
/*                                 - Minor Loop TCD bits implemented      */                                   
/*                                 - Temperature Sensor implemented       */                                   
/*                                 - DSPI.MCR.B.PES implemented           */ 
/* 0.4   R. Moran     30/Mar/10    - Added DTS Module Registers           */
/*                                 - Added Reaction Module                */ 
/*								   - eQADC REDLCCR register				  */            
/*								   - Temp Sensor TCCR0 corrected 		  */
/*								   - CFTCR definition changed             */
/*                                 - EBI CAL_BR/OR updated                */
/*                                 - XBAR MPR0,1,3,7 fixed master fields  */
/*                                 - Decimation filter updated to support */
/*                                   Integration filter and rev2 changes  */
/* 0.5	 I. Harris	  25/May/10    - Updated ECSM_ESR with 1bit cor. fld  */
/*								   - Updated ECSM_ECR with 1bit cor. ena  */
/*								   - Corrected ECSM_MUDR endianness		  */
/*								   - Corrected ECSM_MWCR ENBWCR field name*/
/*								   - Updated SIU_IREEx fields			  */
/*                                 - Added EBI_MCR DBM field              */
/*								   - Added PBRIDGE Registers			  */
/*                                 - Added SIU EMPCR0 Register            */
/*                                 - Included FlexCAN RXFIFO structure    */
/**************************************************************************/

/**************************************************************************/
/* Example instantiation and use:                                         */
/*                                                                        */
/*  <MODULE>.<REGISTER>.B.<BIT> = 1;                                      */
/*  <MODULE>.<REGISTER>.R       = 0x10000000;                             */
/*                                                                        */
/**************************************************************************/

#ifndef _MPC5644_H_
#define _MPC5644_H_

#include "typedefs.h"

#ifdef  __cplusplus
extern "C" {
#endif

#ifdef __MWERKS__
#pragma push
#pragma ANSI_strict off
#endif

/****************************************************************************/
/*       DMA2 Transfer Control Descriptor                                   */
/****************************************************************************/

    struct EDMA_TCD_STD_tag {          /* for "standard" format TCDs (when EDMA.TCD[x].CITER.E_LINK==BITER.E_LINK=0 && EDMA.EMLM=0 ) */
            /* 00 */
            vuint32_t SADDR;    /* Source Address */
    
            /* 04 */            /* Transfer Attributes */
            vuint16_t SMOD:5;   /* Source Address Modulo */
            vuint16_t SSIZE:3;  /* Source Data Transfer Size */
            vuint16_t DMOD:5;   /* Destination Address Modulo */
            vuint16_t DSIZE:3;  /* Destination Data Transfer Size */
    
            /* 06 */
            vint16_t SOFF;      /* Signed Source Address Offset */
    
            /* 08 */
            vuint32_t NBYTES;   /* Inner ("Minor") Byte Transfer Count */
    
            /* 0C */
            vint32_t SLAST;     /* Last Source Address Adjustment */
    
            /* 10 */
            vuint32_t DADDR;    /* Destination Address */
    
            /* 14 */
            vuint16_t CITERE_LINK:1;     /* Enable Channel-to-Channel        */
                                         /* Linking on Minor Loop Completion */
            vuint16_t CITER:15;          /* Current Major Iteration Count    */
    
            /* 16 */
            vint16_t DOFF;      /* Signed Destination Address Offset */
    
            /* 18 */
            vint32_t DLAST_SGA; /* Last Destination Address Adjustment, or  */
                                /* Scatter/Gather Address (if E_SG = 1)     */
    
            /* 1C */
            vuint16_t BITERE_LINK:1;    /* Enable Channel-to-Channel           */
                                        /* Linking on Minor Loop Complete      */
            vuint16_t BITER:15;         /* Starting ("Major") Iteration Count */
    
            /* 1E */                    /* Channel Control/Status */
            vuint16_t BWC:2;            /* Bandwidth Control */
            vuint16_t MAJORLINKCH:6;    /* Link Channel Number */
            vuint16_t DONE:1;           /* Channel Done */
            vuint16_t ACTIVE:1;
            vuint16_t MAJORE_LINK:1;    /* Enable Channel-to-Channel Link */
            vuint16_t E_SG:1;           /* Enable Scatter/Gather Descriptor */
            vuint16_t D_REQ:1;          /* Disable IPD_REQ When Done */
            vuint16_t INT_HALF:1;       /* Interrupt on CITER = (BITER >> 1) */
            vuint16_t INT_MAJ:1;        /* Interrupt on Major Loop Completion */
            vuint16_t START:1;          /* Explicit Channel Start */
    };

    

    struct EDMA_TCD_alt1_tag {  /*for alternate format TCDs (when EDMA.TCD[x].CITER.E_LINK==BITER.E_LINK=1 ) */

            /* 00 */
            vuint32_t SADDR;    /* Source Address */

            /* 04 */            /* Transfer Attributes */
            vuint16_t SMOD:5;   /* Source Address Modulo */
            vuint16_t SSIZE:3;  /* Source Data Transfer Size */
            vuint16_t DMOD:5;   /* Destination Address Modulo */
            vuint16_t DSIZE:3;  /* Destination Data Transfer Size */

            /* 06 */
            vint16_t SOFF;      /* Signed Source Address Offset */

            /* 08 */
            vuint32_t NBYTES;   /* Inner ("Minor") Byte Transfer Count */

            /* 0C */
            vint32_t SLAST;     /* Last Source Address Adjustment */

            /* 10 */
            vuint32_t DADDR;    /* Destination Address */

            /* 14 */
            vuint16_t CITERE_LINK:1;     /* Enable Channel-to-Channel        */
                                         /* Linking on Minor Loop Completion */
            vuint16_t CITERLINKCH:6;     /* Link Channel Number              */
            vuint16_t CITER:9;           /* Current Major Iteration Count    */

            /* 16 */
            vint16_t DOFF;      /* Signed Destination Address Offset */

            /* 18 */
            vint32_t DLAST_SGA; /* Last Destination Address Adjustment, or  */
                                /* Scatter/Gather Address (if E_SG = 1)     */

            /* 1C */
            vuint16_t BITERE_LINK:1;    /* Enable Channel-to-Channel           */
                                        /* Linking on Minor Loop Complete      */
            vuint16_t BITERLINKCH:6;    /* Link Channel Number                 */
            vuint16_t BITER:9;          /* Starting ("Major") Iteration Count  */

            /* 1E */                    /* Channel Control/Status */
            vuint16_t BWC:2;            /* Bandwidth Control */
            vuint16_t MAJORLINKCH:6;    /* Link Channel Number */
            vuint16_t DONE:1;           /* Channel Done */
            vuint16_t ACTIVE:1;         /* Channel Active */
            vuint16_t MAJORE_LINK:1;    /* Enable Channel-to-Channel Link */
            vuint16_t E_SG:1;           /* Enable Scatter/Gather Descriptor */
            vuint16_t D_REQ:1;          /* Disable IPD_REQ When Done */
            vuint16_t INT_HALF:1;       /* Interrupt on CITER = (BITER >> 1) */
            vuint16_t INT_MAJ:1;        /* Interrupt on Major Loop Completion */
            vuint16_t START:1;          /* Explicit Channel Start */
    };

    struct EDMA_TCD_alt2_tag {  /*       for alternate format TCDs (when EDMA.EMLM=1) */

            vuint32_t SADDR;    /* source address */

            vuint16_t SMOD:5;   /* source address modulo */
            vuint16_t SSIZE:3;  /* source transfer size */
            vuint16_t DMOD:5;   /* destination address modulo */
            vuint16_t DSIZE:3;  /* destination transfer size */
            vint16_t SOFF;      /* signed source address offset */

            vuint16_t SMLOE:1;  /* Source minor loop offset enable */
            vuint16_t DMLOE:1;  /* Destination minor loop offset enable */
            vuint32_t MLOFF:20; /* Minor loop Offset */
            vuint16_t NBYTES:10;        /* inner (“minor”) byte count */

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
    };
    

/****************************************************************************/
/*                          MODULE : eDMA                                   */
/****************************************************************************/

    struct EDMA_tag {

        union {
            vuint32_t R;
            struct {
                vuint32_t:14;           /* Reserved */
                vuint32_t CX:1;         /* Cancel Transfer */
                vuint32_t ECX:1;        /* Error Cancel Transfer  */
                vuint32_t GRP3PRI:2;    /* Channel Group 3 Priority  */
                vuint32_t GRP2PRI:2;    /* Channel Group 2 Priority  */
                vuint32_t GRP1PRI:2;    /* Channel Group 1 Priority  */
                vuint32_t GRP0PRI:2;    /* Channel Group 0 Priority  */
                vuint32_t EMLM:1;       /* Enable Minor Loop Mapping */
                vuint32_t CLM:1;        /* Continuous Link Mode */
                vuint32_t HALT:1;       /* Halt DMA Operations  */
                vuint32_t HOE:1;        /* Halt On Error */
                vuint32_t ERGA:1;       /* Enable Round Robin Group Arbitration */
                vuint32_t ERCA:1;       /* Enable Round Robin Channel Arbitration */
                vuint32_t EDBG:1;       /* Enable Debug */
                vuint32_t:1;            /* Enable Buffered Writes */
            } B;
        } CR;                /* DMA Control Register @baseaddress + 0x0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t VLD:1;        /* Logical OR of all DMAERRH */
                vuint32_t:14;           /* Reserved */
                vuint32_t ECX:1;        /* Transfer Canceled */
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
        } ESR;               /* Error Status Register @baseaddress + 0x4 */

        union {
            vuint32_t R;
            struct {
                vuint32_t ERQ63:1;
                vuint32_t ERQ62:1;
                vuint32_t ERQ61:1;
                vuint32_t ERQ60:1;
                vuint32_t ERQ59:1;
                vuint32_t ERQ58:1;
                vuint32_t ERQ57:1;
                vuint32_t ERQ56:1;
                vuint32_t ERQ55:1;
                vuint32_t ERQ54:1;
                vuint32_t ERQ53:1;
                vuint32_t ERQ52:1;
                vuint32_t ERQ51:1;
                vuint32_t ERQ50:1;
                vuint32_t ERQ49:1;
                vuint32_t ERQ48:1;
                vuint32_t ERQ47:1;
                vuint32_t ERQ46:1;
                vuint32_t ERQ45:1;
                vuint32_t ERQ44:1;
                vuint32_t ERQ43:1;
                vuint32_t ERQ42:1;
                vuint32_t ERQ41:1;
                vuint32_t ERQ40:1;
                vuint32_t ERQ39:1;
                vuint32_t ERQ38:1;
                vuint32_t ERQ37:1;
                vuint32_t ERQ36:1;
                vuint32_t ERQ35:1;
                vuint32_t ERQ34:1;
                vuint32_t ERQ33:1;
                vuint32_t ERQ32:1;
            } B;
        } ERQRH;              /* DMA Enable Request Register High @baseaddress + 0x8*/

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
        } ERQRL;              /* DMA Enable Request Register Low @baseaddress + 0xC*/

        union {
            vuint32_t R;
            struct {

                vuint32_t EEI63:1;
                vuint32_t EEI62:1;
                vuint32_t EEI61:1;
                vuint32_t EEI60:1;
                vuint32_t EEI59:1;
                vuint32_t EEI58:1;
                vuint32_t EEI57:1;
                vuint32_t EEI56:1;
                vuint32_t EEI55:1;
                vuint32_t EEI54:1;
                vuint32_t EEI53:1;
                vuint32_t EEI52:1;
                vuint32_t EEI51:1;
                vuint32_t EEI50:1;
                vuint32_t EEI49:1;
                vuint32_t EEI48:1;
                vuint32_t EEI47:1;
                vuint32_t EEI46:1;
                vuint32_t EEI45:1;
                vuint32_t EEI44:1;
                vuint32_t EEI43:1;
                vuint32_t EEI42:1;
                vuint32_t EEI41:1;
                vuint32_t EEI40:1;
                vuint32_t EEI39:1;
                vuint32_t EEI38:1;
                vuint32_t EEI37:1;
                vuint32_t EEI36:1;
                vuint32_t EEI35:1;
                vuint32_t EEI34:1;
                vuint32_t EEI33:1;
                vuint32_t EEI32:1;
            } B;
        } EEIRH;              /* DMA Enable Error Interrupt Register High @baseaddress + 0x10*/

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
        } EEIRL;              /* DMA Enable Error Interrupt Register Low @baseaddress + 0x14*/

        union {
            vuint8_t R;
            struct {
                vuint8_t NOP:1;
                vuint8_t SERQ:7;
            } B;
        } SERQR;              /* DMA Set Enable Request Register @baseaddress + 0x18*/

        union {
            vuint8_t R;
            struct {
                vuint8_t NOP:1;
                vuint8_t CERQ:7;
            } B;
        } CERQR;              /* DMA Clear Enable Request Register @baseaddress + 0x19*/

        union {
            vuint8_t R;
            struct {
                vuint8_t NOP:1;
                vuint8_t SEEI:7;
            } B;
        } SEEIR;              /* DMA Set Enable Error Interrupt Register @baseaddress + 0x1A*/

        union {
            vuint8_t R;
            struct {
                vuint8_t NOP:1;
                vuint8_t CEEI:7;
            } B;
        } CEEIR;              /* DMA Clear Enable Error Interrupt Register @baseaddress + 0x1B*/

        union {
            vuint8_t R;
            struct {
                vuint8_t NOP:1;
                vuint8_t CINT:7;
            } B;
        } CIRQR;              /* DMA Clear Interrupt Request Register @baseaddress + 0x1C */

        union {
            vuint8_t R;
            struct {
                vuint8_t NOP:1;
                vuint8_t CERR:7;
            } B;
        } CER;                /* DMA Clear error Register @baseaddress + 0x1D */

        union {
            vuint8_t R;
            struct {
                vuint8_t NOP:1;
                vuint8_t SSB:7;
            } B;
        } SSBR;               /* Set Start Bit Register @baseaddress + 0x1E */

        union {
            vuint8_t R;
            struct {
                vuint8_t NOP:1;
                vuint8_t CDSB:7;
            } B;
        } CDSBR;              /* Clear Done Status Bit Register @baseaddress + 0x1F */

        union {
            vuint32_t R;
            struct {
                vuint32_t INT63:1;
                vuint32_t INT62:1;
                vuint32_t INT61:1;
                vuint32_t INT60:1;
                vuint32_t INT59:1;
                vuint32_t INT58:1;
                vuint32_t INT57:1;
                vuint32_t INT56:1;
                vuint32_t INT55:1;
                vuint32_t INT54:1;
                vuint32_t INT53:1;
                vuint32_t INT52:1;
                vuint32_t INT51:1;
                vuint32_t INT50:1;
                vuint32_t INT49:1;
                vuint32_t INT48:1;
                vuint32_t INT47:1;
                vuint32_t INT46:1;
                vuint32_t INT45:1;
                vuint32_t INT44:1;
                vuint32_t INT43:1;
                vuint32_t INT42:1;
                vuint32_t INT41:1;
                vuint32_t INT40:1;
                vuint32_t INT39:1;
                vuint32_t INT38:1;
                vuint32_t INT37:1;
                vuint32_t INT36:1;
                vuint32_t INT35:1;
                vuint32_t INT34:1;
                vuint32_t INT33:1;
                vuint32_t INT32:1;
            } B;
        } IRQRH;              /* DMA Interrupt Request High @baseaddress + 0x20 */

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
        } IRQRL;              /* DMA Interrupt Request Low @baseaddress + 0x24 */

        union {
            vuint32_t R;
            struct {
                vuint32_t ERR63:1;
                vuint32_t ERR62:1;
                vuint32_t ERR61:1;
                vuint32_t ERR60:1;
                vuint32_t ERR59:1;
                vuint32_t ERR58:1;
                vuint32_t ERR57:1;
                vuint32_t ERR56:1;
                vuint32_t ERR55:1;
                vuint32_t ERR54:1;
                vuint32_t ERR53:1;
                vuint32_t ERR52:1;
                vuint32_t ERR51:1;
                vuint32_t ERR50:1;
                vuint32_t ERR49:1;
                vuint32_t ERR48:1;
                vuint32_t ERR47:1;
                vuint32_t ERR46:1;
                vuint32_t ERR45:1;
                vuint32_t ERR44:1;
                vuint32_t ERR43:1;
                vuint32_t ERR42:1;
                vuint32_t ERR41:1;
                vuint32_t ERR40:1;
                vuint32_t ERR39:1;
                vuint32_t ERR38:1;
                vuint32_t ERR37:1;
                vuint32_t ERR36:1;
                vuint32_t ERR35:1;
                vuint32_t ERR34:1;
                vuint32_t ERR33:1;
                vuint32_t ERR32:1;
            } B;
        } ERH;                /* DMA Error High @baseaddress + 0x28 */

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
        } ERL;                /* DMA Error Low @baseaddress + 0x2C */

        union {
            vuint32_t R;
            struct {
                vuint32_t HRS63:1;
                vuint32_t HRS62:1;
                vuint32_t HRS61:1;
                vuint32_t HRS60:1;
                vuint32_t HRS59:1;
                vuint32_t HRS58:1;
                vuint32_t HRS57:1;
                vuint32_t HRS56:1;
                vuint32_t HRS55:1;
                vuint32_t HRS54:1;
                vuint32_t HRS53:1;
                vuint32_t HRS52:1;
                vuint32_t HRS51:1;
                vuint32_t HRS50:1;
                vuint32_t HRS49:1;
                vuint32_t HRS48:1;
                vuint32_t HRS47:1;
                vuint32_t HRS46:1;
                vuint32_t HRS45:1;
                vuint32_t HRS44:1;
                vuint32_t HRS43:1;
                vuint32_t HRS42:1;
                vuint32_t HRS41:1;
                vuint32_t HRS40:1;
                vuint32_t HRS39:1;
                vuint32_t HRS38:1;
                vuint32_t HRS37:1;
                vuint32_t HRS36:1;
                vuint32_t HRS35:1;
                vuint32_t HRS34:1;
                vuint32_t HRS33:1;
                vuint32_t HRS32:1;
            } B;
        } HRSH;               /* hardware request status high @baseaddress + 0x30 */

        union {
            vuint32_t R;
            struct {
                vuint32_t HRS31:1;
                vuint32_t HRS30:1;
                vuint32_t HRS29:1;
                vuint32_t HRS28:1;
                vuint32_t HRS27:1;
                vuint32_t HRS26:1;
                vuint32_t HRS25:1;
                vuint32_t HRS24:1;
                vuint32_t HRS23:1;
                vuint32_t HRS22:1;
                vuint32_t HRS21:1;
                vuint32_t HRS20:1;
                vuint32_t HRS19:1;
                vuint32_t HRS18:1;
                vuint32_t HRS17:1;
                vuint32_t HRS16:1;
                vuint32_t HRS15:1;
                vuint32_t HRS14:1;
                vuint32_t HRS13:1;
                vuint32_t HRS12:1;
                vuint32_t HRS11:1;
                vuint32_t HRS10:1;
                vuint32_t HRS09:1;
                vuint32_t HRS08:1;
                vuint32_t HRS07:1;
                vuint32_t HRS06:1;
                vuint32_t HRS05:1;
                vuint32_t HRS04:1;
                vuint32_t HRS03:1;
                vuint32_t HRS02:1;
                vuint32_t HRS01:1;
                vuint32_t HRS00:1;
            } B;
        } HRSL;               /* hardware request status low @baseaddress + 0x34 */

        uint32_t eDMA_reserved0038[50];  /* 0x0038-0x00FF */

        union {
            vuint8_t R;
            struct {
                vuint8_t ECP:1;
                vuint8_t DPA:1;
                vuint8_t GRPPRI:2;
                vuint8_t CHPRI:4;
            } B;
        } CPR[64];            /* Channel n Priority @baseaddress + 0x100 */

        uint32_t eDMA_reserved0140[944];  /* 0x0140-0x0FFF */
        
        /* Select one of the following declarations depending on the DMA mode chosen */
        struct EDMA_TCD_STD_tag TCD[64];              /* Standard Format   */
        /* struct EDMA_TCD_alt1_tag TCD[64]; */       /* CITER/BITER Link  */
        /* struct EDMA_TCD_alt2_tag TCD[64]; */       /* Minor Loop Offset */
        
        };



/****************************************************************************/
/*                     MODULE : XBAR                                      */
/****************************************************************************/
    struct XBAR_tag {
        union {
            vuint32_t R;
            struct {
                  vuint32_t:1;
                vuint32_t MSTR7:3;      /* Master 7 Priority - Reserved */
                  vuint32_t:1;          
                vuint32_t MSTR6:3;      /* Master 6 Priority - FlexRay */  
                  vuint32_t:4;			/* Master 5 Priority - Not implemented */
                  vuint32_t:1;
                vuint32_t MSTR4:3;      /* Master 4 Priority - eDMA */
                  vuint32_t:4;          /* Master 3 Priority - Not implemented */
                  vuint32_t:4;          /* Master 2 Priority - Not implemented */
                  vuint32_t:1;
                vuint32_t MSTR1:3;      /* Master 1 Priority - e200z4 Core load/store & Nexus port  */
                  vuint32_t:1;
                vuint32_t MSTR0:3;      /* Master 0 Priority - e200z4 core Instruction */
            } B;
        } MPR0;                 /* Master Priority Register for Slave port 0 (Flash) @baseaddress + 0x00 */

        int32_t XBAR_reserved_0004[3];      /* 0x0004 - 0x000F */

        union {
            vuint32_t R;
            struct {
                vuint32_t RO:1;
                vuint32_t HLP:1;        /* Halt Low Priority */
                  vuint32_t:6;
                vuint32_t HPE7:1;       /* High Priority Enable */
                vuint32_t HPE6:1;       /* High Priority Enable */
                  vuint32_t:1;
                vuint32_t HPE4:1;       /* High Priority Enable */
                  vuint32_t:1;
                  vuint32_t:1;
                vuint32_t HPE1:1;       /* High Priority Enable */
                vuint32_t HPE0:1;       /* High Priority Enable */
                  vuint32_t:6;
                vuint32_t ARB:2;
                vuint32_t:2;
                vuint32_t PCTL:2;
                vuint32_t:1;
                vuint32_t PARK:3;
            } B;
        } SGPCR0;               /* Slave General Purpose Control Register 0 (Flash) @baseaddress + 0x10 */

        int32_t XBAR_reserved_0014[59];     /* 0x0014 - 0x01FF */

        union {
            vuint32_t R;
            struct {
                  vuint32_t:1;
                vuint32_t MSTR7:3;      /* Master 7 Priority - Reserved */
                  vuint32_t:1;          
                vuint32_t MSTR6:3;      /* Master 6 Priority - FlexRay */  
                  vuint32_t:4;			/* Master 5 Priority - Not implemented */
                  vuint32_t:1;
                vuint32_t MSTR4:3;      /* Master 4 Priority - eDMA */
                  vuint32_t:4;          /* Master 3 Priority - Not implemented */
                  vuint32_t:4;          /* Master 2 Priority - Not implemented */
                  vuint32_t:1;
                vuint32_t MSTR1:3;      /* Master 1 Priority - e200z4 Core load/store & Nexus port  */
                  vuint32_t:1;
                vuint32_t MSTR0:3;      /* Master 0 Priority - e200z4 core Instruction */
            } B;
        } MPR1;                 /* Master Priority Register for Slave port 1 (EBI) @baseaddress + 0x100 */

        int32_t XBAR_reserved_0100[3];      /* 0x0100 - 0x010F */

        union {
            vuint32_t R;
            struct {
                vuint32_t RO:1;
                vuint32_t HLP:1;        /* Halt Low Priority */
                  vuint32_t:6;
                vuint32_t HPE7:1;       /* High Priority Enable */
                vuint32_t HPE6:1;       /* High Priority Enable */
                  vuint32_t:1;
                vuint32_t HPE4:1;       /* High Priority Enable */
                  vuint32_t:1;
                  vuint32_t:1;
                vuint32_t HPE1:1;       /* High Priority Enable */
                vuint32_t HPE0:1;       /* High Priority Enable */
                  vuint32_t:6;
                vuint32_t ARB:2;
                vuint32_t:2;
                vuint32_t PCTL:2;
                vuint32_t:1;
                vuint32_t PARK:3;
            } B;
        } SGPCR1;               /* Slave General Purpose Control Register 1 (EBI) @baseaddress + 0x110 */

        int32_t XBAR_reserved_0114[59]; /* 0x0114 - 0x01FF */

        union {
            vuint32_t R;
            struct {
                  vuint32_t:1;
                vuint32_t MSTR7:3;      /* Master 7 Priority - Reserved */
                  vuint32_t:1;          
                vuint32_t MSTR6:3;      /* Master 6 Priority - FlexRay */  
                  vuint32_t:4;			/* Master 5 Priority - Not implemented */
                  vuint32_t:1;
                vuint32_t MSTR4:3;      /* Master 4 Priority - eDMA */
                  vuint32_t:4;          /* Master 3 Priority - Not implemented */
                  vuint32_t:4;          /* Master 2 Priority - Not implemented */
                  vuint32_t:1;
                vuint32_t MSTR1:3;      /* Master 1 Priority - e200z4 Core load/store & Nexus port  */
                  vuint32_t:1;
                vuint32_t MSTR0:3;      /* Master 0 Priority - e200z4 core Instruction */
            } B;
        } MPR2;                 /* Master Priority Register for Slave port 2 (RAM) @baseaddress + 0x200 */

        int32_t XBAR_reserved_0204[3];      /* 0x0204 - 0x020F */

        union {
            vuint32_t R;
            struct {
                vuint32_t RO:1;
                vuint32_t HLP:1;        /* Halt Low Priority */
                  vuint32_t:6;
                vuint32_t HPE7:1;       /* High Priority Enable */
                vuint32_t HPE6:1;       /* High Priority Enable */
                  vuint32_t:1;
                vuint32_t HPE4:1;       /* High Priority Enable */
                  vuint32_t:1;
                  vuint32_t:1;
                vuint32_t HPE1:1;       /* High Priority Enable */
                vuint32_t HPE0:1;       /* High Priority Enable */
                  vuint32_t:6;
                vuint32_t ARB:2;
                vuint32_t:2;
                vuint32_t PCTL:2;
                vuint32_t:1;
                vuint32_t PARK:3;
            } B;
        } SGPCR2;               /* Slave General Purpose Control Register 2 (RAM)@baseaddress + 0x210 */

        int32_t XBAR_reserved_0214[59];     /* 0x0214 - 0x02FF */

        /* Slave General Purpose Control Register 3 @baseaddress + 0x310 - not implemented */

        int32_t XBAR_reserved_0300[64];     /* 0x0300 - 0x03FF */

        /* Slave General Purpose Control Register 4 @baseaddress + 0x410 - not implemented */

        int32_t XBAR_reserved_0400[64];     /* 0x0400 - 0x04FF */

        /* Slave XBAR Port 5 Not implemented @baseaddress + 0x510 */

        int32_t XBAR_reserved_0500[64];     /* 0x0500 - 0x05FF */

        /* Slave Port 6 not implemented @baseaddress + 0x610 */

        int32_t XBAR_reserved_0600[64];     /* 0x0600 - 0x06FF */

        union {
            vuint32_t R;
            struct {
                  vuint32_t:1;
                vuint32_t MSTR7:3;      /* Master 7 Priority - Reserved */
                  vuint32_t:1;          
                vuint32_t MSTR6:3;      /* Master 6 Priority - FlexRay */  
                  vuint32_t:4;			/* Master 5 Priority - Not implemented */
                  vuint32_t:1;
                vuint32_t MSTR4:3;      /* Master 4 Priority - eDMA */
                  vuint32_t:4;          /* Master 3 Priority - Not implemented */
                  vuint32_t:4;          /* Master 2 Priority - Not implemented */
                  vuint32_t:1;
                vuint32_t MSTR1:3;      /* Master 1 Priority - e200z4 Core load/store & Nexus port  */
                  vuint32_t:1;
                vuint32_t MSTR0:3;      /* Master 0 Priority - e200z4 core Instruction */
            } B;
        } MPR7;                 /* Master Priority Register for Slave port 7 @baseaddress + 0x700 */

        int32_t XBAR_reserved_0704[3];          /* 0x0704 - 0x070F */

        union {
            vuint32_t R;
            struct {
                vuint32_t RO:1;
                vuint32_t HLP:1;        /* Halt Low Priority */
                  vuint32_t:6;
                vuint32_t HPE7:1;       /* High Priority Enable */
                vuint32_t HPE6:1;       /* High Priority Enable */
                  vuint32_t:1;
                vuint32_t HPE4:1;       /* High Priority Enable */
                  vuint32_t:1;
                  vuint32_t:1;
                vuint32_t HPE1:1;       /* High Priority Enable */
                vuint32_t HPE0:1;       /* High Priority Enable */
                  vuint32_t:6;
                vuint32_t ARB:2;
                vuint32_t:2;
                vuint32_t PCTL:2;
                vuint32_t:1;
                vuint32_t PARK:3;
            } B;
        } SGPCR7;               /* Slave General Purpose Control Register 7 @baseaddress + 0x710 */

        int32_t XBAR_reserved_0714[59];         /* 0x0714 - 0x07FF */

        int32_t XBAR_reserved_0800[3584];       /* 0x0800-0x3FFF */

    };


/****************************************************************************/
/*                          MODULE : PBRIDGE Peripheral Bridge            */
/****************************************************************************/

	struct PBRIDGE_tag {
		
		union {                               /* Master Privilege Registers 0-7 @baseaddress + 0x00*/
            vuint32_t R;
			struct {
				 vuint32_t:1;      
				vuint32_t  MTR0:1;      /* z4 core: Master 0 Trusted for Reads */
				vuint32_t  MTW0:1;      /* z4 core: Master 0 Trusted for Writes */
				vuint32_t  MPL0:1;      /* z4 core: Master 0 Priviledge Level */
				 vuint32_t:13;      
				vuint32_t  MTR4:1;      /* DMA: Master 4 Trusted for Reads */
				vuint32_t  MTW4:1;      /* DMA: Master 4 Trusted for Writes */
				vuint32_t  MPL4:1;      /* DMA: Master 4 Priviledge Level */
				 vuint32_t:5;      
				vuint32_t  MTR6:1;      /* FlexRay: Master 6 Trusted for Reads */
				vuint32_t  MTW6:1;      /* FlexRay: Master 6 Trusted for Writes */
				vuint32_t  MPL6:1;      /* FlexRay: Master 6 Priviledge Level */
				 vuint32_t:1;      
				vuint32_t  MTR7:1;      /* EBI: Master 7 Trusted for Reads */
				vuint32_t  MTW7:1;      /* EBI: Master 7 Trusted for Writes */
				vuint32_t  MPL7:1;      /* EBI: Master 7 Priviledge Level */
            } B;
        } MPCR;
		
		union {                               /* Master Privilege Registers 8-15 @baseaddress + 0x04*/
            vuint32_t R;
			struct {
                 vuint32_t:1;      
				vuint32_t  MTR0:1;      /* Nexus: Master 8 Trusted for Reads */
				vuint32_t  MTW0:1;      /* Nexus: Master 8 Trusted for Writes */
				vuint32_t  MPL0:1;      /* Nexus: Master 8 Priviledge Level */
				 vuint32_t:28;      
            } B;
        } MPCR1;
		
		uint32_t PRIDGE_reserved0008[6];        /* 0x0008-0x001F */
		
		union {                             /* Peripheral Access Conrol Registers 0-7 @baseaddress + 0x20*/
            vuint32_t R;
			struct {
				 vuint32_t:5;    
				vuint32_t  SP1:1;        /* Crossbar: Supervisor Protect */
				vuint32_t  WP1:1;        /* Crossbar: Write Protect */
				vuint32_t  TP1:1;        /* Crossbar: Trusted Protect */
				 vuint32_t:9;      
				vuint32_t  SP4:1;        /* MPU: Supervisor Protect */
				vuint32_t  WP4:1;        /* MPU: Write Protect */
				vuint32_t  TP4:1;        /* MPU: Trusted Protect */
				 vuint32_t:12;  
            } B;
        } PACR0;
		
		union {                             /* Peripheral Access Conrol Registers 8-15 @baseaddress + 0x24*/
            vuint32_t R;
			struct {
				vuint32_t:25;    
				vuint32_t  SP6:1;        /* SWT: Supervisor Protect */
				vuint32_t  WP6:1;        /* SWT: Write Protect */
				vuint32_t  TP6:1;        /* SWT: Trusted Protect */
				 vuint32_t:1;      
				vuint32_t  SP7:1;        /* STM: Supervisor Protect */
				vuint32_t  WP7:1;        /* STM: Write Protect */
				vuint32_t  TP7:1;        /* STM: Trusted Protect */ 
            } B;
        } PACR1;
		
		union {                             /* Peripheral Access Conrol Registers 16-23 @baseaddress + 0x28*/
            vuint32_t R;
			struct {
				vuint32_t:1;    
				vuint32_t  SP0:1;        /* ECSM: Supervisor Protect */
				vuint32_t  WP0:1;        /* ECSM: Write Protect */
				vuint32_t  TP0:1;        /* ECSM: Trusted Protect */
				 vuint32_t:1;      
				vuint32_t  SP1:1;        /* DMA: Supervisor Protect */
				vuint32_t  WP1:1;        /* DMA: Write Protect */
				vuint32_t  TP1:1;        /* DMA: Trusted Protect */ 
				 vuint32_t:1;      
				vuint32_t  SP2:1;        /* INTC: Supervisor Protect */
				vuint32_t  WP2:1;        /* INTC: Write Protect */
				vuint32_t  TP2:1;        /* INTC: Trusted Protect */ 
				 vuint32_t:20; 
            } B;
        } PACR2;
		
		union {                             /* Peripheral Access Conrol Registers 24-31 @baseaddress + 0x2C*/
            vuint32_t R;
			struct {
				 vuint32_t:32;    
			} B;
        } PACR3;
		
		uint32_t PRIDGE_reserved0030[4];        /* 0x0030-0x003C */
		
		union {                             /* Off-Platform Access Control Registers 0-7 @baseaddress + 0x40*/
            vuint32_t R;
			struct {
                 vuint32_t:1;      
				vuint32_t  SP0:1;       /* eQADC: Supervisor Protect */
				vuint32_t  WP0:1;       /* eQADC: Write Protect */
				vuint32_t  TP0:1;       /* eQADC: Trusted Protect */
				 vuint32_t:5; 
				vuint32_t  SP2:1;       /* Dec Filter A: Supervisor Protect */
				vuint32_t  WP2:1;       /* Dec Filter A: Write Protect */
				vuint32_t  TP2:1;       /* Dec Filter A: Trusted Protect */
				vuint32_t:1;       
				vuint32_t  SP3:1;       /* Dec Filter B: Supervisor Protect */
				vuint32_t  WP3:1;       /* Dec Filter B: Write Protect */
				vuint32_t  TP3:1;       /* Dec Filter B: Trusted Protect */
				vuint32_t:5;       
				vuint32_t  SP5:1;       /* DSPIB: Supervisor Protect */
				vuint32_t  WP5:1;       /* DSPIB: Write Protect */
				vuint32_t  TP5:1;       /* DSPIB: Trusted Protect */
				vuint32_t:1;     
				vuint32_t  SP6:1;       /* DSPIC: Supervisor Protect */
				vuint32_t  WP6:1;       /* DSPIC: Write Protect */
				vuint32_t  TP6:1;       /* DSPIC: Trusted Protect */
				vuint32_t:1; 
				vuint32_t  SP7:1;       /* DSPID: Supervisor Protect */
				vuint32_t  WP7:1;       /* DSPID: Write Protect */
				vuint32_t  TP7:1;       /* DSPID: Trusted Protect */
            } B;
        } OPACR0;
		
		union {   							/* Off-Platform Access Control Registers 8-15 @baseaddress + 0x44*/
			vuint32_t R;
			struct {
				 vuint32_t:17;      
				vuint32_t  SP4:1;      /* eSCIA: Supervisor Protect */
				vuint32_t  WP4:1;      /* eSCIA: Write Protect */
				vuint32_t  TP4:1;      /* eSCIA: Trusted Protect */
				 vuint32_t:1;    
				vuint32_t  SP5:1;      /* eSCIB: Supervisor Protect */
				vuint32_t  WP5:1;      /* eSCIB: Write Protect */
				vuint32_t  TP5:1;      /* eSCIB: Trusted Protect */
				 vuint32_t:1;   
				vuint32_t  SP6:1;      /* eSCIC: Supervisor Protect */
				vuint32_t  WP6:1;      /* eSCIC: Write Protect */
				vuint32_t  TP6:1;      /* eSCIC: Trusted Protect */
				 vuint32_t:4;
			} B;
        } OPACR1;	 
		   
		union {   							/* Off-Platform Access Control Registers 16-23 @baseaddress + 0x48*/
			vuint32_t R;
			struct {
				 vuint32_t:1;      
				vuint32_t  SP0:1;      /* FlexCANA: Supervisor Protect */
				vuint32_t  WP0:1;      /* FlexCANA: Write Protect */
				vuint32_t  TP0:1;      /* FlexCANA: Trusted Protect */
				 vuint32_t:1;    
				vuint32_t  SP1:1;      /* FlexCANB: Supervisor Protect */
				vuint32_t  WP1:1;      /* FlexCANB: Write Protect */
				vuint32_t  TP1:1;      /* FlexCANB: Trusted Protect */
				 vuint32_t:1;   
				vuint32_t  SP2:1;      /* FlexCANC: Supervisor Protect */
				vuint32_t  WP2:1;      /* FlexCANC: Write Protect */
				vuint32_t  TP2:1;      /* FlexCANC: Trusted Protect */
				 vuint32_t:20;
			} B;
		} OPACR2;
		   
		union {   							/* Off-Platform Access Control Registers 24-31 @baseaddress + 0x4C*/
			vuint32_t R;
			struct {
				 vuint32_t:1;      
				vuint32_t  SP0:1;      /* FlexRay: Supervisor Protect */
				vuint32_t  WP0:1;      /* FlexRay: Write Protect */
				vuint32_t  TP0:1;      /* FlexRay: Trusted Protect */
				 vuint32_t:9;    
				vuint32_t  SP3:1;      /* SIM: Supervisor Protect */
				vuint32_t  WP3:1;      /* SIM: Write Protect */
				vuint32_t  TP3:1;      /* SIM: Trusted Protect */
				 vuint32_t:13;   
				vuint32_t  SP7:1;      /* BAM: Supervisor Protect */
				vuint32_t  WP7:1;      /* BAM: Write Protect */
				vuint32_t  TP7:1;      /* BAM: Trusted Protect */
			} B;
		} OPACR3;
		
		union {   							/* Off-Platform Access Control Registers 32-39 @baseaddress + 0x50*/
			vuint32_t R;
			struct {
				 vuint32_t:32;      
			} B;
		} OPACR4;
		
		union {   							/* Off-Platform Access Control Registers 40-47 @baseaddress + 0x54*/
			vuint32_t R;
			struct {
				 vuint32_t:32;      
			} B;
		} OPACR5;
		
		union {   							/* Off-Platform Access Control Registers 48-55 @baseaddress + 0x58*/
			vuint32_t R;
			struct {
				 vuint32_t:32;      
			} B;
		} OPACR6;
		
		union {   							/* Off-Platform Access Control Registers 56-63 @baseaddress + 0x5C*/
			vuint32_t R;
			struct {
				 vuint32_t:9;  
				vuint32_t  SP2:1;      /* CRC: Supervisor Protect */
				vuint32_t  WP2:1;      /* CRC: Write Protect */
				vuint32_t  TP2:1;      /* CRC: Trusted Protect */
				 vuint32_t:20; 
			} B;
		} OPACR7;
		
		union {   							/* Off-Platform Access Control Registers 64-71 @baseaddress + 0x60*/
			vuint32_t R;
			struct {
				 vuint32_t:1;  
				vuint32_t  SP0:1;      /* FMPLL: Supervisor Protect */
				vuint32_t  WP0:1;      /* FMPLL: Write Protect */
				vuint32_t  TP0:1;      /* FMPLL: Trusted Protect */
				 vuint32_t:1;  
				vuint32_t  SP1:1;      /* EBI: Supervisor Protect */
				vuint32_t  WP1:1;      /* EBI: Write Protect */
				vuint32_t  TP1:1;      /* EBI: Trusted Protect */
				 vuint32_t:1;  
				vuint32_t  SP2:1;      /* FlashA: Supervisor Protect */
				vuint32_t  WP2:1;      /* FlashA: Write Protect */
				vuint32_t  TP2:1;      /* FlashA: Trusted Protect */
				 vuint32_t:1;  
				vuint32_t  SP3:1;      /* FlashB: Supervisor Protect */
				vuint32_t  WP3:1;      /* FlashB: Write Protect */
				vuint32_t  TP3:1;      /* FlashB: Trusted Protect */
				 vuint32_t:1;  
				vuint32_t  SP4:1;      /* SIU: Supervisor Protect */
				vuint32_t  WP4:1;      /* SIU: Write Protect */
				vuint32_t  TP4:1;      /* SIU: Trusted Protect */
				 vuint32_t:9;  
				vuint32_t  SP7:1;      /* DTS: Supervisor Protect */
				vuint32_t  WP7:1;      /* DTS: Write Protect */
				vuint32_t  TP7:1;      /* DTS: Trusted Protect */
			} B;
		} OPACR8;
		
		union {   							/* Off-Platform Access Control Registers 72-79 @baseaddress + 0x64*/
			vuint32_t R;
			struct {
				 vuint32_t:1;  
				vuint32_t  SP0:1;      /* eMIOS: Supervisor Protect */
				vuint32_t  WP0:1;      /* eMIOS: Write Protect */
				vuint32_t  TP0:1;      /* eMIOS: Trusted Protect */
				 vuint32_t:25;  
				vuint32_t  SP7:1;      /* PMC: Supervisor Protect */
				vuint32_t  WP7:1;      /* PMC: Write Protect */
				vuint32_t  TP7:1;      /* PMC: Trusted Protect */
			} B;
		} OPACR9;
		
		union {   							/* Off-Platform Access Control Registers 80-87 @baseaddress + 0x68*/
			vuint32_t R;
			struct {
				 vuint32_t:1;  
				vuint32_t  SP0:1;      /* eTPU2: Supervisor Protect */
				vuint32_t  WP0:1;      /* eTPU2: Write Protect */
				vuint32_t  TP0:1;      /* eTPU2: Trusted Protect */
				 vuint32_t:1;  
				vuint32_t  SP1:1;      /* REACM: Supervisor Protect */
				vuint32_t  WP1:1;      /* REACM: Write Protect */
				vuint32_t  TP1:1;      /* REACM: Trusted Protect */
				 vuint32_t:1;  
				vuint32_t  SP2:1;      /* eTPU PRAM: Supervisor Protect */
				vuint32_t  WP2:1;      /* eTPU PRAM: Write Protect */
				vuint32_t  TP2:1;      /* eTPU PRAM: Trusted Protect */
				 vuint32_t:1;  
				vuint32_t  SP3:1;      /* eTPU PRAM mirror: Supervisor Protect */
				vuint32_t  WP3:1;      /* eTPU PRAM mirror: Write Protect */
				vuint32_t  TP3:1;      /* eTPU PRAM mirror: Trusted Protect */
				 vuint32_t:1;  
				vuint32_t  SP4:1;      /* eTPU CRAM: Supervisor Protect */
				vuint32_t  WP4:1;      /* eTPU CRAM: Write Protect */
				vuint32_t  TP4:1;      /* eTPU CRAM: Trusted Protect */
				 vuint32_t:12;  
			} B;
		} OPACR10;
		
		union {   							/* Off-Platform Access Control Registers 88-95 @baseaddress + 0x6C*/
			vuint32_t R;
			struct {
				 vuint32_t:17;  
				vuint32_t  SP4:1;      /* PIT: Supervisor Protect */
				vuint32_t  WP4:1;      /* PIT: Write Protect */
				vuint32_t  TP4:1;      /* PIT: Trusted Protect */
				 vuint32_t:12; 
			} B;
		} OPACR11;
		
		uint32_t PRIDGE_reserved0070[4068];        /* 0x0070-0x3FFF */
		
	};

/****************************************************************************/
/*                     MODULE : FLASH                                       */
/****************************************************************************/

    struct FLASH_tag {

        union {                             /* Module Configuration Register @baseaddress + 0x00*/
            vuint32_t R;
            struct {
                vuint32_t:5;
                vuint32_t SIZE:3;
                vuint32_t:1;
                vuint32_t LAS:3;
                vuint32_t:3;
                vuint32_t MAS:1;
                vuint32_t EER:1;
                vuint32_t RWE:1;
                vuint32_t SBC:1;
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

        union {
            vuint32_t R;
            struct {
                vuint32_t LME:1;
                vuint32_t:10;
                vuint32_t SLOCK:1;
                vuint32_t:2;
                vuint32_t MLOCK:2;
                vuint32_t:6;
                vuint32_t LLOCK:10;
            } B;                            /* Low/Mid Address Space Block Locking Register @baseaddress + 0x04*/
        } LMLR;

        union {
            vuint32_t R;
            struct {
                vuint32_t HBE:1;
                vuint32_t:25;
                vuint32_t HBLOCK:6;
            } B;
        } HLR;                              /* High Address Space Block Locking Register @baseaddress + 0x08*/

        union {
            vuint32_t R;
            struct {
                vuint32_t SLE:1;
                vuint32_t:10;
                vuint32_t SSLOCK:1;
                vuint32_t:2;
                vuint32_t SMLOCK:2;
                vuint32_t:6;
                vuint32_t SLLOCK:10;
            } B;
        } SLMLR;                            /* Secondary Low/Mid Block Locking Register @baseaddress + 0x0C*/

        union {
            vuint32_t R;
            struct {
                vuint32_t:14;
                vuint32_t MSEL:2;
                vuint32_t:6;
                vuint32_t LSEL:10;
            } B;
        } LMSR;                             /* Low/Mid Address Space Block Select Register @baseaddress + 0x10*/

        union {
            vuint32_t R;
            struct {
                vuint32_t:26;
                vuint32_t HBSEL:6;
            } B;
        } HSR;                              /* High Address Space Block Select Register @baseaddress + 0x14*/

        union {
            vuint32_t R;
            struct {
                vuint32_t SAD:1;
                vuint32_t:13;
                vuint32_t ADDR:15;
                vuint32_t:3;
            } B;
        } AR;                               /* Address Register @baseaddress + 0x18*/

        union {
            vuint32_t R;
            struct {
                vuint32_t:7;
                vuint32_t:1;           /* Reserved */
                vuint32_t:1;           /* EBI Testing - Reserved */
                vuint32_t M6PFE:1;     /* FlexRay  */
                vuint32_t:1;           /* Reserved */
                vuint32_t M4PFE:1;     /* eDMA     */
                vuint32_t:1;           /* Reserved */
                vuint32_t:1;           /* Reserved */
                vuint32_t M1PFE:1;     /* z4 Core Load/Store  */
                vuint32_t M0PFE:1;     /* z4 Core Instruction */
                vuint32_t APC:3;
                vuint32_t WWSC:2;
                vuint32_t RWSC:3;
                vuint32_t:1;
                vuint32_t DPFEN:1;
                vuint32_t:1;
                vuint32_t IPFEN:1;
                vuint32_t:1;
                vuint32_t PFLIM:2;
                vuint32_t BFEN:1;
            } B;
        } BIUCR;                            /* Bus Interface Unit Configuration Register 1 @baseaddress + 0x1C*/

        union {
            vuint32_t R;
            struct {
                vuint32_t:14;
                vuint32_t:2;          /* Reserved */
                vuint32_t:2;          /* EBI Testing - Reserved */
                vuint32_t M6AP:2;     /* FlexRay  */
                vuint32_t:2;          /* Reserved */
                vuint32_t M4AP:2;     /* eDMA_A   */
                vuint32_t:2;          /* Reserved */
                vuint32_t:2;          /* Reserved */
                vuint32_t M1AP:2;     /* z4 Core Load/Store  */
                vuint32_t M0AP:2;     /* z4 Core Instruction */
            } B;
        } BIUAPR;                           /*Bus Interface Unit Access Protection Register @baseaddress + 0x20*/

        union {
            vuint32_t R;
            struct {
                vuint32_t LBCFG:2;
                vuint32_t:30;
            } B;
        } BIUCR2;                           /* Bus Interface Unit Configuration Register 2 @baseaddress + 0x24*/

        uint32_t FLASH_reserved0028[5];  /* 0x0028-0x003B */
        
        union {                 /* User Test 0 (UT0) register@baseaddress + 0x3C */
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
        
        uint32_t FLASH_reserved005C[4073];  /* 0x005C-0x3FFF */
    };

    
    

/****************************************************************************/
/*                     MODULE : EBI                                         */
/****************************************************************************/
    struct CS_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t BA:17;
                vuint32_t:3;
                vuint32_t PS:1;
                vuint32_t:3;
                vuint32_t AD_MUX:1;
                vuint32_t BL:1;
                vuint32_t WEBS:1;
                vuint32_t TBDIP:1;
                vuint32_t:1;
                vuint32_t SETA:1;
                vuint32_t BI:1;
                vuint32_t V:1;
            } B;
        } BR;                           /* EBI Base Registers (BR) @baseaddress + 0x10 - 0x28 */
        union {
            vuint32_t R;
            struct {
                vuint32_t AM:17;
                vuint32_t:7;
                vuint32_t SCY:4;
                vuint32_t:1;
                vuint32_t BSCY:2;
                vuint32_t:1;
            } B;
        } OR;                           /* EBI Option Registers (OR) @baseaddress + 0x14 - 0x2C */
    };
    struct CAL_CS_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t BA:17;
                vuint32_t:3;
                vuint32_t PS:1;
                vuint32_t:3;
                vuint32_t AD_MUX:1;
                vuint32_t BL:1;
                vuint32_t WEBS:1;
                vuint32_t TBDIP:1;
                vuint32_t:1;
                vuint32_t SETA:1;
                vuint32_t BI:1;	
                vuint32_t V:1;
            } B;
        } BR;                           /* EBI CAL Base Registers (CAL_BR) @baseaddress + 0x40 - 0x58 */
        union {
            vuint32_t R;
            struct {
                vuint32_t AM:17;
                vuint32_t:7;
                vuint32_t SCY:4;
                vuint32_t:1;
                vuint32_t BSCY:2;
                vuint32_t:1;

            } B;
        } OR;                           /* EBI CAL Option Registers (CAL_OR) @baseaddress + 0x44 - 0x5C */
    };


    struct EBI_tag {
        union {
            vuint32_t R;
            struct {
                 vuint32_t:16;
                vuint32_t ACGE:1;
                 vuint32_t:8;
                vuint32_t MDIS:1;
                 vuint32_t:3;
                vuint32_t D16_31:1;
                vuint32_t AD_MUX:1;
                vuint32_t DBM:1;
            } B;
        } MCR;                          /* EBI Module Configuration Register (MCR) @baseaddress + 0x00 */
        
		uint32_t EBI_reserved0004[1];   /* 0x0004-0x0008 */
        
		union {
            vuint32_t R;
            struct {
                 vuint32_t:31;
                vuint32_t BMTF:1;
            } B;
        } TESR;                         /* EBI Transfer Error Status Register (TESR) @baseaddress + 0x08 */
        
		union {
            vuint32_t R;
            struct {
                 vuint32_t:16;
                vuint32_t BMT:8;
                vuint32_t BME:1;
                  vuint32_t:7;
            } B;
        } BMCR;                         /* EBI Bus Montior Control Register (BMCR) @baseaddress + 0x0C */

        struct CS_tag CS[4];            /* EBI CS Registers (BR / OR) @baseaddress + 0x10 - 0x2C */

        uint32_t EBI_reserved0030[4];   /* 0x0030 - 0x003C */

        struct CAL_CS_tag CAL_CS[4];    /* EBI CAL_CS Registers (CAL_BR / CAL_OR) @baseaddress + 0x40 - 0x5C */
    };


/****************************************************************************/
/*                     MODULE : INTC                                      */
/****************************************************************************/
    struct INTC_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:18;   /* Reserved */
                  vuint32_t:1;  /* Reserved */
                  vuint32_t:4;  /* Reserved */
                  vuint32_t:1;  /* Reserved */
                  vuint32_t:2;  /* Reserved */
                vuint32_t VTES:1;       /* Vector Table Entry Size */
                  vuint32_t:4;  /* Reserved */
                vuint32_t HVEN:1;       /* Hardware Vector Enable */
            } B;
        } MCR;                  /* INTC Module Configuration Register (MCR) @baseaddress + 0x00 */

        int32_t INTC_Reserved_0004[1];     /* 0x0004 - 0x0007 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;   /* Reserved */
                vuint32_t PRI:4;        /* Priority */
            } B;
        } CPR;                  /* INTC Current Priority Register (CPR) @baseaddress + 0x08 */

        int32_t INTC_reserved_000C;        /* 0x000C - 0x000F */

        union {
            vuint32_t R;
            struct {
                vuint32_t VTBA:21;      /* Vector Table Base Address */
                vuint32_t INTVEC:9;     /* Interrupt Vector */
                  vuint32_t:2;          /* Reserved */
            } B;
        } IACKR;                /* INTC Interrupt Acknowledge Register (IACKR) @baseaddress + 0x10 */

        int32_t INTC_Reserved_0014;       /* 0x0014 - 0x00017 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:32;   /* Reserved */
            } B;
        } EOIR;                 /* INTC End of Interrupt Register (EOIR) @baseaddress + 0x18 */

        int32_t INTC_Reserved_001C;       /* 0x001C - 0x001F */

        union {
            vuint8_t R;
            struct {
                vuint8_t:6;     /* Reserved */
                vuint8_t SET:1; /* Set Flag bits */
                vuint8_t CLR:1; /* Clear Flag bits */
            } B;
        } SSCIR[8];             /* INTC Software Set/Clear Interrupt Registers (SSCIR) @baseaddress + 0x20 */

        int32_t INTC_Reserved_0028[6];     /* 0x0028 - 0x003F */

        union {
            vuint8_t R;
            struct {
                  vuint8_t:2;   /* Reserved */
                  vuint8_t:2;   /* Reserved */
                vuint8_t PRI:4; /* Priority Select */
            } B;
        } PSR[512];             /* INTC Priority Select Registers (PSR) @baseaddress + 0x40 */

    };


/****************************************************************************/
/*                     MODULE : SIU                                      */
/****************************************************************************/
    struct SIU_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t S_F:1;                /* Identifies the Manufacturer */
                vuint32_t FLASH_SIZE_1:4;       /* Define major Flash memory size (see Table 15-4 for details) */
                vuint32_t FLASH_SIZE_2:4;       /* Define Flash memory size, small granularity  */
                vuint32_t TEMP_RANGE:2;         /* Define maximum operating range  */
                  vuint32_t:1;                  /* Reserved for future enhancements */
                vuint32_t MAX_FREQ:2;           /* Define maximum device speed  */
                  vuint32_t:1;                  /* Reserved for future enhancements */
                vuint32_t SUPPLY:1;             /* Defines if the part is 5V or 3V  */
                vuint32_t PART_NUMBER:8;        /* Contain the ASCII representation of the character that indicates the product  */
                vuint32_t TBD:1;                /* 1-bit field defined by SoC to describe optional feature, e.g., additional SPI */
                  vuint32_t:2;                  /* Reserved for future enhancements */
                vuint32_t EE:1;                 /* Indicates if Data Flash is present */
                  vuint32_t:3;                  /* Reserved for future enhancements */
                vuint32_t FR:1;                 /* Indicates if Data FlexRay is present */
            } B;
        } MIDR2;                /* MCU ID Register 2  @baseaddress + 0x0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t PARTNUM:16;   /* Device part number */
                vuint32_t CSP:1;        /* CSP configuration */
                vuint32_t PKG:5;        /* Indicate the package the die is mounted in. */
                  vuint32_t:2;          /* Reserved */
                vuint32_t MASKNUM:8;    /* MCU major mask number; updated for each complete resynthesis. MCU minor mask number; updated for each mask revision */
            } B;
        } MIDR;                 /* MCU ID Register (MIDR) @baseaddress + 0x4 */

        int32_t SIU_Reserved_0008;     /* 0x0008 - 0x000B */

        union {
            vuint32_t R;
            struct {
                vuint32_t PORS:1;       /*  Power-On Reset Status */
                vuint32_t ERS:1;        /*  External Reset Status */
                vuint32_t LLRS:1;       /*  Loss of Lock Reset Status */
                vuint32_t LCRS:1;       /*  Loss of Clock Reset Status */
                vuint32_t WDRS:1;       /*  Watchdog Timer/Debug Reset Status */
                vuint32_t :1;
                vuint32_t SWTRS:1;      /*  Software Watchdog Timer Reset Status */
                  vuint32_t:7;
                vuint32_t SSRS:1;       /*  Software System Reset Status */
                vuint32_t SERF:1;       /*  Software External Reset Flag */
                vuint32_t WKPCFG:1;     /*  Weak Pull Configuration Pin Status */
                  vuint32_t:11;
                vuint32_t ABR:1;        /*  Auto Baud Rate */
                vuint32_t BOOTCFG:2;    /*  Reset Configuration Pin Status  */
                vuint32_t RGF:1;        /*  RESET Glitch Flag */
            } B;
        } RSR;                  /* Reset Status Register (SIU_RSR) @baseaddress + 0xC */

        union {
            vuint32_t R;
            struct {
                vuint32_t SSR:1;        /*  Software System Reset */
                vuint32_t SER:1;        /*  Software External Reset */
                  vuint32_t:14;
                  vuint32_t:1;
                  vuint32_t:15;
            } B;
        } SRCR;                 /* System Reset Control Register (SRCR) @baseaddress + 0x10 */

        union {
            vuint32_t R;
            struct {
                vuint32_t NMI:1;        /* Non-Maskable Interrupt Flag */
                  vuint32_t:7;  /*  */
                vuint32_t SWT:1;        /* Software Watch Dog Timer Interrupt Flag, from platform */
                  vuint32_t:7;  /*  */
                vuint32_t EIF15:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF14:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF13:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF12:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF11:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF10:1;      /* External Interrupt Request Flag x */
                vuint32_t EIF9:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF8:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF7:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF6:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF5:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF4:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF3:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF2:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF1:1;       /* External Interrupt Request Flag x */
                vuint32_t EIF0:1;       /* External Interrupt Request Flag x */
            } B;
        } EISR;                 /* SIU External Interrupt Status Register (EISR) @baseaddress + 0x14 */

        union {
            vuint32_t R;
            struct {
                vuint32_t NMI_SEL:1;    /* NMI Interrupt Platform Input Selection */
                  vuint32_t:7;
                vuint32_t NMISEL0:1;    /* SWT Interrupt Platform Input Selection */
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
        } DIRER;                /* DMA/Interrupt Request Enable Register (DIRER) @baseaddress + 0x18 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;   /*  */
                vuint32_t DIRS3:1;      /* DMA/Interrupt Request Select x */
                vuint32_t DIRS2:1;      /* DMA/Interrupt Request Select x */
                vuint32_t DIRS1:1;      /* DMA/Interrupt Request Select x */
                vuint32_t DIRS0:1;      /* DMA/Interrupt Request Select x */
            } B;
        } DIRSR;                /* DMA/Interrupt Request Select Register (DIRSR) @baseaddress + 0x1C */

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
                vuint32_t OVF7:1;       /* Overrun Flag x */
                vuint32_t OVF6:1;       /* Overrun Flag x */
                vuint32_t OVF5:1;       /* Overrun Flag x */
                vuint32_t OVF4:1;       /* Overrun Flag x */
                vuint32_t OVF3:1;       /* Overrun Flag x */
                vuint32_t OVF2:1;       /* Overrun Flag x */
                vuint32_t OVF1:1;       /* Overrun Flag x */
                vuint32_t OVF0:1;       /* Overrun Flag x */
            } B;
        } OSR;                  /* Overrun Status Register (OSR) @baseaddress + 0x20 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t ORE15:1;      /* Overrun Request Enable x */
                vuint32_t ORE14:1;      /* Overrun Request Enable x */
                vuint32_t ORE13:1;      /* Overrun Request Enable x */
                vuint32_t ORE12:1;      /* Overrun Request Enable x */
                vuint32_t ORE11:1;      /* Overrun Request Enable x */
                vuint32_t ORE10:1;      /* Overrun Request Enable x */
                vuint32_t ORE9:1;       /* Overrun Request Enable x */
                vuint32_t ORE8:1;       /* Overrun Request Enable x */
                vuint32_t ORE7:1;       /* Overrun Request Enable x */
                vuint32_t ORE6:1;       /* Overrun Request Enable x */
                vuint32_t ORE5:1;       /* Overrun Request Enable x */
                vuint32_t ORE4:1;       /* Overrun Request Enable x */
                vuint32_t ORE3:1;       /* Overrun Request Enable x */
                vuint32_t ORE2:1;       /* Overrun Request Enable x */
                vuint32_t ORE1:1;       /* Overrun Request Enable x */
                vuint32_t ORE0:1;       /* Overrun Request Enable x */
            } B;
        } ORER;                 /* Overrun Request Enable Register (ORER) @baseaddress + 0x24 */

        union {
            vuint32_t R;
            struct {
                vuint32_t NMIRE:1;      /* NMI Rising-Edge Event Enable x  */
                 vuint32_t:7; 
				vuint32_t NMIRE0:1;     /* NMI Falling-Edge Event Enable (SWT) x */
                 vuint32_t:7; 
				vuint32_t IREE15:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE14:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE13:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE12:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE11:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE10:1;     /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE9:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE8:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE7:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE6:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE5:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE4:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE3:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE2:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE1:1;      /* IRQ Rising-Edge Event Enable x */
                vuint32_t IREE0:1;      /* IRQ Rising-Edge Event Enable x */
            } B;
        } IREER;                /* External IRQ Rising-Edge Event Enable Register (IREER)  @baseaddress + 0x28 */

        union {
            vuint32_t R;
            struct {
                vuint32_t NMIFE:1;      /* NMI Falling-Edge Event Enable (NMI Input) x */
                 vuint32_t:7; 
				vuint32_t NMIFE0:1;     /* NMI Falling-Edge Event Enable (SWT) x */
                 vuint32_t:7; 
				vuint32_t IFEE15:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE14:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE13:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE12:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE11:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE10:1;     /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE9:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE8:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE7:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE6:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE5:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE4:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE3:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE2:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE1:1;      /* IRQ Falling-Edge Event Enable x */
                vuint32_t IFEE0:1;      /* IRQ Falling-Edge Event Enable x */
            } B;
        } IFEER;                /* External IRQ Falling-Edge Event Enable Regi (IFEER) @baseaddress + 0x2C */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t DFL:4;        /* Digital Filter Length */
            } B;
        } IDFR;                 /* External IRQ Digital Filter Register (IDFR) @baseaddress + 0x30 */

        int32_t SIU_Reserved_0034[3];      /* 0x0034 - 0x003F */

        union {
            vuint16_t R;
            struct {
                vuint16_t:3;
                vuint16_t PA:3;
                vuint16_t OBE:1;
                vuint16_t IBE:1;
                vuint16_t DSC:2;
                vuint16_t ODE:1;
                vuint16_t HYS:1;
                vuint16_t SRC:2;
                vuint16_t WPE:1;
                vuint16_t WPS:1;
            } B;
        } PCR[512];             /* Pad Configuration Register  (PCR) @baseaddress + 0x40 */

        int32_t SIU_Reserved_0374[112];    /* 0x0374 - 0x05FF */

        union {
            vuint8_t R;
            struct {
                vuint8_t:7;
                vuint8_t PDO:1;
            } B;
        } GPDO[512];            /* GPIO Pin Data Output Register (GPDO) @baseaddress + 0x600 */

        union {
            vuint8_t R;
            struct {
                vuint8_t:7;
                vuint8_t PDI:1;
            } B;
        } GPDI[256];            /* GPIO Pin Data Input Register (GDPI) @baseaddress + 0x800 */

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
        } ETISR;                /* eQADC Trigger Input Select Register (ETISR) @baseaddress + 0x900 */

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
        } EIISR;                /* External IRQ Input Select Register (EIISR) @baseaddress + 0x904 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t SINSELB:2;    /* DSPI_B Data Input Select  */
                vuint32_t SSSELB:2;     /* DSPI_B Slave Select Input Select */
                vuint32_t SCKSELB:2;    /* DSPI_B Clock Input Select */
                vuint32_t TRIGSELB:2;   /* DSPI_B Trigger Input Select */
                vuint32_t SINSELC:2;    /* DSPI_C Data Input Select */
                vuint32_t SSSELC:2;     /* DSPI_C Slave Select Input Select */
                vuint32_t SCKSELC:2;    /* DSPI_C Clock Input Select */
                vuint32_t TRIGSELC:2;   /* DSPI_C Trigger Input Select  */
                vuint32_t SINSELD:2;    /* DSPI_D Data Input Select */
                vuint32_t SSSELD:2;     /* DSPI_D Slave Select Input Select */
                vuint32_t SCKSELD:2;    /* DSPI_D Clock Input Select */
                vuint32_t TRIGSELD:2;   /* DSPI_D Trigger Input Select  */
            } B;
        } DISR;                 /* DSPI Input Select Register (DISR) @baseaddress + 0x908 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:2;    /*  */
                vuint32_t ETSEL5:5;     /* eQADC queue X Enhanced Trigger Selection */
                vuint32_t ETSEL4:5;     /* eQADC queue X Enhanced Trigger Selection */
                vuint32_t ETSEL3:5;     /* eQADC queue X Enhanced Trigger Selection */
                vuint32_t ETSEL2:5;     /* eQADC queue X Enhanced Trigger Selection */
                vuint32_t ETSEL1:5;     /* eQADC queue X Enhanced Trigger Selection */
                vuint32_t ETSEL0:5;     /* eQADC queue X Enhanced Trigger Selection */
            } B;
        } ISEL3;                /* MUX Select Register 3 (ISEL3) @baseaddress + 0x90C */

        int32_t SIU_Reserved_0910[4];      /* 0x0910 - 0x091F */

        union {
            vuint32_t R;
            struct {
                vuint32_t:11;
                vuint32_t ESEL5:1;
                  vuint32_t:3;
                vuint32_t ESEL4:1;
                  vuint32_t:3;
                vuint32_t ESEL3:1;
                  vuint32_t:3;
                vuint32_t ESEL2:1;
                  vuint32_t:3;
                vuint32_t ESEL1:1;
                  vuint32_t:3;
                vuint32_t ESEL0:1;
            } B;
        } ISEL8;                /* MUX Select Register 8 (ISEL8) @baseaddress + 0x920 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:27;
                vuint32_t ETSEL0A:5;
            } B;
        } ISEL9;                /* MUX Select Register 9(ISEL9) @baseaddress + 0x924 */

        int32_t SIU_Reserved_0928[22];     /* 0x0928 - 0x097F */

        union {
            vuint32_t R;
            struct {
                vuint32_t:14;
                vuint32_t MATCH:1;      /*  Compare Register Match */
                vuint32_t DISNEX:1;     /*  Disable Nexus */
                  vuint32_t:14;
                vuint32_t CRSE:1;       /*  Calibration Reflection Suppression Enable */
                  vuint32_t:1;
            } B;
        } CCR;                  /* Chip Configuration Register (CCR) @baseaddress + 0x980 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:18;
                vuint32_t ENGDIV:6;
                vuint32_t ENGSSE:1;
                  vuint32_t:3;
                vuint32_t EBTS:1;
                  vuint32_t:1;
                vuint32_t EBDF:2;
            } B;
        } ECCR;                 /* External Clock Control Register (ECCR) @baseaddress + 0x984 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CMPAH:32;
            } B;
        } CARH;                 /* Compare A High Register (CARH) @baseaddress + 0x988 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CMPAL:32;
            } B;
        } CARL;                 /* Compare A Low Register (CARL) @baseaddress + 0x98C */

        union {
            vuint32_t R;
            struct {
                vuint32_t CMPBH:32;
            } B;
        } CBRH;                 /* Compare B High Register (CBRH) @baseaddress + 0x990 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CMPBL:32;
            } B;
        } CBRL;                 /* Compare B Low Register (CBRL) @baseaddress + 0x994 */

        int32_t SIU_Reserved_0998[2];      /* 0x0998 - 0x099F */

        union {
            vuint32_t R;
            struct {
                vuint32_t:15;
                vuint32_t CAN_SRC:1;    /* CAN 2:1 Mode */
                vuint32_t:11;
                vuint32_t BYPASS:1;     /* Bypass bit  */
                vuint32_t SYSCLKDIV:2;  /* System Clock Divide */
                  vuint32_t:2;
            } B;
        } SYSDIV;               /* System Clock Register (SYSDIV) @baseaddress + 0x9A0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CPUSTP:1;     /* CPU stop request. When asserted, a stop request is sent to the following modules: */
                  vuint32_t:2;  /* Reserved */
                  vuint32_t:1;
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
                vuint32_t CNBSTP:1;     /* FlexCAN B stop request. When asserted, a stop request is sent to the FlexCAN B */
                vuint32_t CNASTP:1;     /* FlexCAN A stop request. When asserted, a stop request is sent to the FlexCAN A */
                vuint32_t SPIDSTP:1;    /* DSPI D stop request. When asserted, a stop request is sent to the DSPI D. */
                vuint32_t SPICSTP:1;    /* DSPI C stop request. When asserted, a stop request is sent to the DSPI C. */
                vuint32_t SPIBSTP:1;    /* DSPI B stop request. When asserted, a stop request is sent to the DSPI B. */
                  vuint32_t:6;  /* Reserved */
                vuint32_t SCICSTP:1;    /* eSCI C stop request. When asserted, a stop request is sent to the eSCI C module. */
                vuint32_t SCIBSTP:1;    /* eSCI B stop request. When asserted, a stop request is sent to the eSCI B module. */
                vuint32_t SCIASTP:1;    /* eSCI A stop request. When asserted, a stop request is sent to the eSCIA module. */
            } B;
        } HLT;                  /* Halt Register (HLT) @baseaddress + 0x9A4 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CPUACK:1;     /* CPU stop acknowledge. When asserted, indicates that a stop acknowledge was */
                  vuint32_t:2;  /* Reserved */
                  vuint32_t:1;
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
                vuint32_t CNBACK:1;     /* FlexCAN B stop acknowledge. When asserted, indicates that a stop acknowledge */
                vuint32_t CNAACK:1;     /* FlexCAN A stop acknowledge. When asserted, indicates that a stop acknowledge */
                vuint32_t SPIDACK:1;    /* DSPI D stop acknowledge. When asserted, indicates that a stop acknowledge was */
                vuint32_t SPICACK:1;    /* DSPI C stop acknowledge. When asserted, indicates that a stop acknowledge was */
                vuint32_t SPIBACK:1;    /* DSPI B stop acknowledge. When asserted, indicates that a stop acknowledge was */
                  vuint32_t:6;  /* Reserved */
                vuint32_t SCICACK:1;    /* eSCI C stop acknowledge */
                vuint32_t SCIBACK:1;    /* eSCI B stop acknowledge */
                vuint32_t SCIAACK:1;    /* eSCI A stop acknowledge. */
            } B;
        } HLTACK;               /* Halt Acknowledge Register (HLTACK) @baseaddress + 0x9A8 */

        int32_t SIU_reserved09AC[2];       /* 0x09AC - 0x09B0 */
		
		 union {
            vuint32_t R;
            struct {
				vuint32_t EXT_PID_EN:1;     /* External PID Selection Enable */
                vuint32_t EXT_PID_SYNC0:1;  /* External PID Synchronization 0 */
                 vuint32_t:28;              /* Reserved */
				vuint32_t EXT_PID6:1;       /* EXT_PID6 */
                vuint32_t EXT_PID7:1;       /* EXT_PID7 */
			} B;
		} EMPCR0;				/* Core MMU PID Control Register (EMPCR0) @baseaddress + 0x9B4 */
		
		int32_t SIU_reserved09B8[19];       /* 0x09B8 - 0x09B0 */
		
    };

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
                  vuint32_t:1;
                vuint32_t LOLIRQ:1;
                vuint32_t LOCIRQ:1;
                  vuint32_t:13;
            } B;
        } SYNCR;                /* Synthesizer Control Register (SYNCR) @baseaddress + 0x0000 */

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
                  vuint32_t:2;
            } B;
        } SYNSR;                /* Synthesizer Status Register (SYNSR) @baseaddress + 0x0004 */

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
        } ESYNCR1;              /* Enhanced Synthesizer Control Register 1 (ESYNCR1) @baseaddress + 0x0008 */

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
        } ESYNCR2;              /* Enhanced Synthesizer Control Register 2 (ESYNCR2) @baseaddress + 0x000C */

        int32_t FMPLL_reserved0010[2];      /* 0x0010-0x0017 */

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
        } SYNFMMR;              /* Synthesizer FM Modulation Register (SYNFMMR) @baseaddress + 0x0018 */
    };


/****************************************************************************/
/*                     MODULE : ECSM                                       */
/****************************************************************************/
    struct ECSM_tag {

        union {                /* Processor core type */
            vuint16_t R;
        } PCT;
        
        union {                /* Platform revision */
            vuint16_t R;
        } REV;
        
        union {                /* AXBS Master Configuration */
            vuint16_t R;
        } AMC;
        
        union {                /* AXBS Slave Configuration */
            vuint16_t R;
        } ASC;
        
        union {                 /* IPS Module Configuration */
            vuint32_t R;
        } IMC;
        
        uint8_t ECSM_reserved000C[3];  /* 0x000C-0x000E */
        
        union {                 /* Miscellaneous Reset Status Register */
            vuint8_t R;
            struct {
                vuint8_t POR:1;
                vuint8_t DIR:1;
                vuint8_t SWTR:1;
                 vuint8_t:5;
            } B;
        } MRSR;

        uint8_t ECSM_reserved0010[3];   /* 0x0010-0x0012 */
        
        union {                         /* Miscellaneous Wakeup Control */
            vuint8_t R;
            struct {
                vuint8_t ENBWCR:1;
                 vuint8_t:3;
                vuint8_t PRILVL:4;
            } B;
        } MWCR;
                
        uint32_t ecsm_reserved0014[4];  /* 0x0014 - 0x0023 */
        
        union {                         /* Miscellaneous User Defined Control */
            vuint32_t R;
            struct {
                 vuint32_t:1;
                vuint32_t SWSC:1;
                 vuint32_t:30;
            } B;
        } MUDCR;

        uint32_t ecsm_reserved0028[6];      /* 0x0028 - 0x003C*/
        
        uint8_t ecsm_reserved0040[3];       /* 0x0040 - 0x0042*/

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
        } ECR;                  /* ECC Configuration Register @baseaddress + 0x43 */

        uint8_t ecsm_reserved0044[3];      /* 0x0044 - 0x0046*/

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
        } ESR;                  /* ECC Status Register @baseaddress + 0x47 */

        uint8_t ecsm_reserved0048[2];      /* 0x0048 - 0x0049*/
        
        union {
            vuint16_t R;
            struct {
                vuint16_t FRCAP:1;
                vuint16_t:1;
                vuint16_t FRC1BI:1;
                vuint16_t FR11BI:1;
                vuint16_t:2;
                vuint16_t FRCNCI:1;
                vuint16_t FR1NCI:1;
                vuint16_t:1;
                vuint16_t ERRBIT:7;
            } B;
        } EEGR;                 /* ECC Error Generation Register @baseaddress + 0x4A */
        
        uint32_t ecsm_reserved004C;      /* 0x004C - 0x004F*/
                
        union {
            vuint32_t R;
            struct {
                vuint32_t FEAR:32;
            } B;
        } FEAR;                 /* Flash ECC Address Register @baseaddress + 0x50 */

        uint16_t ecsm_reserved0054;          /* 0x0054 - 0x0055*/

        union {
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t FEMR:4;
            } B;
        } FEMR;                 /* Flash ECC Master Register @baseaddress + 0x56 */

        union {
            vuint8_t R;
            struct {
                vuint8_t WRITE:1;
                vuint8_t SIZE:3;
                vuint8_t PROT0:1;
                vuint8_t PROT1:1;
                vuint8_t PROT2:1;
                vuint8_t PROT3:1;
            } B;
        } FEAT;                 /* Flash ECC Attributes Register @baseaddress + 0x57 */

        union {
            vuint32_t R;
            struct {
                vuint32_t FEDH:32;
            } B;
        } FEDRH;                /* Flash ECC Data High Register @baseaddress + 0x58 */

        union {
            vuint32_t R;
            struct {
                vuint32_t FEDL:32;
            } B;
        } FEDRL;                /* Flash ECC Data Low Register @baseaddress + 0x5C */

        union {
            vuint32_t R;
            struct {
                vuint32_t REAR:32;
            } B;
        } REAR;                 /* RAM ECC Address @baseaddress + 0x60 */

        uint8_t ecsm_reserved0064;          /* 0x0064 - 0x0065*/

        union {
            vuint8_t R;
            struct {
                vuint8_t PRESR:8;
            } B;
        } PRESR;                /* RAM ECC Syndrome @baseaddress + 0x65 */

        union {
            vuint8_t R;
            struct {
                vuint8_t:4;
                vuint8_t REMR:4;
            } B;
        } REMR;                 /* RAM ECC Master @baseaddress + 0x66  */

        union {
            vuint8_t R;
            struct {
                vuint8_t WRITE:1;
                vuint8_t SIZE:3;
                vuint8_t PROT0:1;
                vuint8_t PROT1:1;
                vuint8_t PROT2:1;
                vuint8_t PROT3:1;
            } B;
        } REAT;                 /*  RAM ECC Attributes Register @baseaddress + 0x67 */

        union {
            vuint32_t R;
            struct {
                vuint32_t REDH:32;
            } B;
        } REDRH;                /* RAM ECC Data High Register @baseaddress + 0x68 */

        union {
            vuint32_t R;
            struct {
                vuint32_t REDL:32;
            } B;
        } REDRL;                /* RAMECC Data Low Register @baseaddress + 0x6C */

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
        } CR;                   /* STM Control Register @baseaddress + 0x0000 */

        union {
            vuint32_t R;
        } CNT;                  /* STM Count Register @baseaddress + 0x0004 */

        uint32_t stm_reserved0008[2];      /* 0x0008 - 0x000F */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR0;                 /* STM Channel Control Register @baseaddress + 0x0010 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR0;                 /* STM Channel Interrupt Register @baseaddress + 0x0014 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CMP;
            } B;
        } CMP0;                 /* STM Channel Compare Register @baseaddress + 0x0018 */

        uint32_t stm_reserved001C; /* 0x001C - 0x001F*/

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR1;                 /* STM Channel Control Register @baseaddress + 0x0020 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR1;                 /* STM Channel Interrupt Register @baseaddress + 0x0024 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CMP;
            } B;
        } CMP1;                 /* STM Channel Compare Register @baseaddress + 0x0028 */

        uint32_t stm_reserved002C; /* 0x002C - 0x002F */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR2;                 /* STM Channel Control Register @baseaddress + 0x0030 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR2;                 /* STM Channel Interrupt Register @baseaddress + 0x0034 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CMP;
            } B;
        } CMP2;                 /* STM Channel Compare Register @baseaddress + 0x0038 */

        uint32_t stm_reserved003C; /* 0x003C - 0x003F */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CEN:1;
            } B;
        } CCR3;                 /* STM Channel Control Register @baseaddress + 0x0040 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t CIF:1;
            } B;
        } CIR3;                 /* STM Channel Interrupt Register @baseaddress + 0x0044 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CMP;
            } B;
        } CMP3;                 /* STM Channel Compare Register @baseaddress + 0x0048 */

        uint32_t stm_reserved004C; /* 0x004C - 0x004F */
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
        } MCR;                  /* Module Configuration Register @baseaddress + 0x00 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:31;
                vuint32_t TIF:1;
            } B;
        } IR;                   /* Interrupt register @baseaddress + 0x04 */

        union {
            vuint32_t R;
            struct {
                vuint32_t WTO:32;
            } B;
        } TO;                   /* Timeout register @baseaddress + 0x08 */

        union {
            vuint32_t R;
            struct {
                vuint32_t WST:32;

            } B;
        } WN;                   /* Window register @baseaddress + 0x0C */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t WSC:16;
            } B;
        } SR;                   /* Service register @baseaddress + 0x10 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CNT:32;
            } B;
        } CO;                   /* Counter output register @baseaddress + 0x14 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t SK:16;
            } B;
        } SK;                   /* Service key register @baseaddress + 0x18 */
    };

/****************************************************************************/
/*                     MODULE : EMIOS                                      */
/****************************************************************************/
    struct EMIOS_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t DOZEEN:1;
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
        } MCR;                  /* Module Configuration Register @baseaddress + 0x00 */

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
        } GFR;                  /* Global FLAG Register @baseaddress + 0x04 */

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
        } OUDR;                 /* Output Update Disable Register @baseaddress + 0x08 */

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
        } UCDIS;                /* Disable Channel (EMIOSUCDIS) @baseaddress + 0x0C */

        int32_t EMIOS_Reserved_0010[4];    /* 0x0010 - 0x001F */

        struct {
            union {
                vuint32_t R;
                struct {
                    vuint32_t A;
                }B;
            } CADR;             /* Channel A Data Register */

            union {
                vuint32_t R;
                struct {
                    vuint32_t B;
                }B;
            } CBDR;             /* Channel B Data Register */

            union {
                vuint32_t R;
                struct {
                    vuint32_t C;
                }B;
            } CCNTR;            /* Channel Counter Register */

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
            } CCR;              /* Channel Control Register */

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
            } CSR;              /* Channel Status Register */

            union {
                vuint32_t R;
                struct {
                    vuint32_t ALTA;
                } B;
            } ALTA;             /* Alternate Channel A Data Register */

            uint32_t emios_channel_reserved[2];

        } CH[24];

    };

/****************************************************************************/
/*                     MODULE : ETPU                                      */
/****************************************************************************/
    struct ETPU_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t GEC:1;        /*  Global Exception Clear  */
                vuint32_t SDMERR:1;     /*  */
                vuint32_t WDTOA:1;      /*  */
                vuint32_t WDTOB:1;      /*  */
                vuint32_t MGE1:1;       /* */
                vuint32_t MGE2:1;       /* */
                vuint32_t ILF1:1;       /* Invalid instruction flag eTPU A.  */
                vuint32_t ILF2:1;       /* Invalid instruction flag eTPU B.  */
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
        } MCR;                  /* eTPU module configuration register@baseaddress + 0x00 */


        union {
            vuint32_t R;
            struct {
                vuint32_t STS:1;        /*  Start Status bit  */
                vuint32_t CTBASE:5;     /*  Channel Transfer Base  */
                vuint32_t PBASE:10;     /*  Parameter Buffer Base Address */
                vuint32_t PWIDTH:1;     /*  Parameter Width  */
                vuint32_t PARAM0:7;     /*  Channel Parameter 0  */
                vuint32_t WR:1; /*  */
                vuint32_t PARAM1:7;     /*  Channel Parameter 1  */
            } B;
        } CDCR;                 /* eTPU coherent dual-parameter controller register@baseaddress + 0x04 */

        vuint32_t ETPU_reserved_0008;   /* 0x0008 - 0x000B */


        union {
            vuint32_t R;
            struct {
                vuint32_t ETPUMISCCMP:32;       /* Expected multiple input signature calculator compare register value.  */
            } B;
        } MISCCMPR;             /* eTPU MISC Compare Register@baseaddress + 0x0c */


        union {
            vuint32_t R;
            struct {
                vuint32_t ETPUSCMOFFDATA:32;    /* SCM Off-range read data value. */
            } B;
        } SCMOFFDATAR;          /* eTPU SCM Off-Range Data Register@baseaddress + 0x10 */


        union {
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
        } ECR_A;               /* eTPU Engine Configuration Register (ETPUA_ECR)@baseaddress + 0x14 */

        vuint32_t ETPU_reserved_0018[2];        /* 0x0018 - 0x001B */

        union {
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
        } TBCR_A;                       /* eTPU Time Base Configuration Register (ETPU_TBCR)@baseaddress + 0x20 */

        /* offset 0x0024 */
        union {
            vuint32_t R;
            struct {
                vuint32_t:8;    /*  */
                vuint32_t TCR1:24;      /* TCR1 value. Used on matches and captures. For more information, see the eTPU reference manual. */
            } B;
        } TB1R_A;                /* eTPU Time Base 1 (TCR1) Visibility Register (ETPU_TB1R)@baseaddress + 0x24 */

        /* offset 0x0028 */
        union {
            vuint32_t R;
            struct {
                vuint32_t:8;    /*  */
                vuint32_t TCR2:24;      /* TCR2 value. Used on matches and captures. For information on TCR2, see the eTPU reference manual. */
            } B;
        } TB2R_A;               /* eTPU Time Base 2 (TCR2) Visibility Register (ETPU_TB2R)@baseaddress + 0x28 */


        union {
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
        } REDCR_A;                 /* STAC Bus Configuration Register (ETPU_STACCR)@baseaddress + 0x2c */

        vuint32_t ETPU_reserved_0030[12];       /* 0x0030 - 0x005F */


        union {
            vuint32_t R;
            struct {
                vuint32_t WDM:2;
                  vuint32_t:14;
                vuint32_t WDCNT:16;
            } B;
        } WDTR_A;                 /* ETPU1 WDTR Register @baseaddress + 0x60 */

        vuint32_t ETPU1_reserved_0064;  /* 0x0064 - 0x0067 */


        union {
            vuint32_t R;
            struct {
                vuint32_t IDLE_CNT:31;
                vuint32_t ICLR:1;
            } B;
        } IDLE_A;                  /* ETPU1 IDLE Register @baseaddress + 0x68 */

        vuint32_t ETPU_reserved_006C[101];      /* 0x006C - 0x01FF */


        union {
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
        } CISR_A;               /* eTPU Channel Interrupt Status Register (ETPU_CISR)@baseaddress + 0x200 */

        int32_t ETPU_reserved_0204[3];      /* 0x0204 - 0x20F */

        union {
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
        } CDTRSR_A;             /* eTPU Channel Data Transfer Request Status Register (ETPU_CDTRSR) @baseaddress + 0x210 */

        int32_t ETPU_reserved_0214[3];      /* 0x0214 - 0x021F */


        union {
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
        } CIOSR_A;              /* eTPU Channel Interrupt Overflow Status Register (ETPU_CIOSR)@baseaddress + 0x220 */

        int32_t ETPU_reserved_0224[3];      /* 0x0224 - 0x022F */


        union {
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
        } CDTROSR_A;            /* eTPU Channel Data Transfer Request Overflow Status Register@baseaddress + 0x230 */

        int32_t ETPU_reserved_0234[3];      /* 0x0234 - 0x023F */


        union {
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
        } CIER_A;               /* eTPU Channel Interrupt Enable Register (ETPU_CIER)@baseaddress + 0x240 */

        int32_t ETPU_reserved_0244[3];      /* 0x0244 - 0x25F */


        union {
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
        } CDTRER_A;             /* eTPU Channel Data Transfer Request Enable Register (ETPU_CDTRER)@baseaddress + 0x250 */

        int32_t ETPU_reserved_0254[3];      /* 0x0254 - 0x025F */


        union {
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
        } WDSR_A;              /* ETPUWDSR - eTPU Watchdog Status Register @baseaddress + 0x260 */

        int32_t ETPU_reserved_0264[7];      /* 0x0264 - 0x027F */


        union {
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
        } CPSSR_A;              /* ETPUCPSSR - eTPU Channel Pending Service Status Register @baseaddress + 0x280  */

        int32_t ETPU_reserved_0x0284[3];        /* 0x0284 - 0x028F  */


        union {
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
        } CSSR_A;               /* ETPUCSSR - eTPU Channel Service Status Register @baseaddress + 0x290  */

        int32_t ETPU_reserved_0294[91];         /* 0x0294 - 0x03FF */


/***************************** Channels ********************************/
/* Note not all devices implement all channels or even 2 engines       */
/* Each eTPU engine can implement 64 channels, however most devcies    */
/* only implemnet 32 channels. The eTPU block can implement 1 or 2     */
/* engines per instantiation                                           */
/***********************************************************************/

        struct {
            union {
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
            } CR;               /* eTPU Channel n Configuration Register (ETPU_CnCR)@baseaddress + 0x400 */

            union {
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
            } SCR;              /* eTPU Channel n Status Control Register (ETPU_CnSCR)@baseaddress + 0x404 */

            union {
                vuint32_t R;
                struct {
                    vuint32_t:29;       /*  Host Service Request  */
                    vuint32_t HSR:3;    /*  */
                } B;
            } HSRR;             /* eTPU channel host service request register (ETPU_CnHSRR)@baseaddress + 0x408 */

            int32_t ETPU_reserved_0C;       /* CHAN Base + 0x0C */

        } CHAN[127];
                 /**** Note: Not all channels implemented on all devices. *******/
    };

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
        } MCR;                  /* Module Configuration Register */

        int32_t EQADC_reserved0004;     /* 0x0004 - 0x0007 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:6;
                vuint32_t NMF:26;
            } B;
        } NMSFR;                /* Null Message Send Format Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t:28;
                vuint32_t DFL:4;
            } B;
        } ETDFR;                /* External Trigger Digital Filter Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t CFPUSH:32;
            } B;
        } CFPR[6];              /* CFIFO Push Registers */

        uint32_t eqadc_reserved1;

        uint32_t eqadc_reserved2;

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RFPOP:16;
            } B;
        } RFPR[6];              /* Result FIFO Pop Registers*/

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
        } CFCR[6];              /* CFIFO Control Registers */

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
        } IDCR[6];              /* Interrupt and DMA Control Registers */

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
        } FISR[6];              /* FIFO and Interrupt Status Registers */

        uint32_t eqadc_reserved7;

        uint32_t eqadc_reserved8;

        union {
            vuint16_t R;
            struct {
                vuint16_t:5;
                vuint16_t TCCF:11;
            } B;
        } CFTCR[6];             /* CFIFO Transfer Counter Registers */

        uint32_t eqadc_reserved9;

        union {
            vuint32_t R;
            struct {
                vuint32_t CFS0:2;
                vuint32_t CFS1:2;
                vuint32_t CFS2:2;
                vuint32_t CFS3:2;
                vuint32_t CFS4:2;
                vuint32_t CFS5:2;
                  vuint32_t:5;
                vuint32_t LCFTCB0:4;
                vuint32_t TC_LCFTCB0:11;
            } B;
        } CFSSR0;               /* CFIFO Status Register 0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CFS0:2;
                vuint32_t CFS1:2;
                vuint32_t CFS2:2;
                vuint32_t CFS3:2;
                vuint32_t CFS4:2;
                vuint32_t CFS5:2;
                  vuint32_t:5;
                vuint32_t LCFTCB1:4;
                vuint32_t TC_LCFTCB1:11;
            } B;
        } CFSSR1;               /* CFIFO Status Register 1 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CFS0:2;
                vuint32_t CFS1:2;
                vuint32_t CFS2:2;
                vuint32_t CFS3:2;
                vuint32_t CFS4:2;
                vuint32_t CFS5:2;
                  vuint32_t:4;
                vuint32_t ECBNI:1;
                vuint32_t LCFTSSI:4;
                vuint32_t TC_LCFTSSI:11;
            } B;
        } CFSSR2;               /* CFIFO Status Register 2 */

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
        } CFSR;

        uint32_t eqadc_reserved11;

        union {
            vuint32_t R;
            struct {
                vuint32_t:21;
                vuint32_t MDT:3;
                  vuint32_t:4;
                vuint32_t BR:4;
            } B;
        } SSICR;                /* SSI Control Register */

        union {
            vuint32_t R;
            struct {
                vuint32_t RDV:1;
                  vuint32_t:5;
                vuint32_t RDATA:26;
            } B;
        } SSIRDR;               /* SSI Recieve Data Register @ baseaddress + 0xB8 */

        uint32_t eqadc_reserved11b[5];

        union {
            vuint32_t R;
            struct {
                 vuint32_t:16;
                vuint32_t REDBS2:4;
                vuint32_t SRV2:4;
                vuint32_t REDBS1:4;
                vuint32_t SRV1:4;
            } B;
        } REDLCCR;               /* STAC Bus Clent Configuration Register @ baseaddress + 0xD0 */


        uint32_t eqadc_reserved12[11];

        struct {
            union {
                vuint32_t R;
                struct {
                    vuint32_t:32;
                } B;
            } R[4];

            union {
                vuint32_t R;
                struct {
                    vuint32_t:32;
                } B;
            } EDATA[4];

            uint32_t eqadc_reserved13[8];

        } CF[6];

        uint32_t eqadc_reserved14[32];

        struct {
            union {
                vuint32_t R;
                struct {
                    vuint32_t:32;
                } B;
            } R[4];

            uint32_t eqadc_reserved15[12];

        } RF[6];

    };

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
                vuint32_t CASCD:2;
                vuint32_t IDEN:1;
                vuint32_t ODEN:1;
                vuint32_t ERREN:1;
                  vuint32_t:1;
                vuint32_t FTYPE:2;
                  vuint32_t:1;
                vuint32_t SCAL:2;
                vuint32_t IDIS:1;
                vuint32_t SAT:1;
                vuint32_t ISEL:1;
                vuint32_t MIXM:1;
                vuint32_t DEC_RATE:4;
                vuint32_t SDIE:1;
                vuint32_t DSEL:1;
                vuint32_t IBIE:1;
                vuint32_t OBIE:1;
                vuint32_t EDME:1;
                vuint32_t TORE:1;
                vuint32_t TMODE:2;
            } B;
        } MCR;                  /* Configuration Register @baseaddress + 0x00 */

        union {
            vuint32_t R;
            struct {
                vuint32_t BSY:1;
                  vuint32_t:1;
                vuint32_t DEC_COUNTER:4;
                vuint32_t IDFC:1;
                vuint32_t ODFC:1;
                  vuint32_t:1;
                vuint32_t IBIC:1;
                vuint32_t OBIC:1;
                  vuint32_t:1;
                vuint32_t DIVRC:1;
                vuint32_t OVFC:1;
                vuint32_t OVRC:1;
                vuint32_t IVRC:1;
                  vuint32_t:6;
                vuint32_t IDF:1;
                vuint32_t ODF:1;
                  vuint32_t:1;
                vuint32_t IBIF:1;
                vuint32_t OBIF:1;
                  vuint32_t:1;
                vuint32_t DIVR:1;
                vuint32_t OVF:1;
                vuint32_t OVR:1;
                vuint32_t IVR:1;
            } B;
        } MSR;                  /* Status Register @baseaddress + 0x04 */

 		union {
            vuint32_t R;
            struct {
                vuint32_t SDMAE:1;
                vuint32_t SSIG:1;
                vuint32_t SSAT:1;
                vuint32_t SCSAT:1;
                  vuint32_t:10;
                vuint32_t SRQ:1;
                vuint32_t SZRO:1;  
                vuint32_t SISEL:1;  
                  vuint32_t:1;
                vuint32_t SZROSEL:2;    
                  vuint32_t:2;
                vuint32_t SHLTSEL:2;      
                  vuint32_t:1;
                vuint32_t SRQSEL:3;        
                  vuint32_t:2;
                vuint32_t SENSEL:2;          
            } B;
        } MXCR;                   /* Extended Config Register @baseaddress + 0x8  */
        
        union {
            vuint32_t R;
            struct {
                  vuint32_t:7;
                vuint32_t SDFC:1;
                  vuint32_t:2;
                vuint32_t SSEC:1;
                vuint32_t SCEC:1;
                  vuint32_t:1;
                vuint32_t SSOVFC:1;
                vuint32_t SCOVFC:1;
                vuint32_t SVRC:1;  
                  vuint32_t:7;
                vuint32_t SDF:1;
                  vuint32_t:2;
                vuint32_t SSE:1;
                vuint32_t SCE:1;
                  vuint32_t:1;
                vuint32_t SSOVF:1;
                vuint32_t SCOVF:1;
                vuint32_t SVR:1;    
            } B;
        } MXSR;                   /* Extended Status Register @baseaddress + 0xC  */       
        

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
        } IB;                   /* Interface Input Buffer @baseaddress + 0x10  */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t OUTTAG:4;
                vuint32_t OUTBUF:16;
            } B;
        } OB;                   /* Interface Output Buffer @baseaddress + 0x14  */

        uint32_t decfil_reserved0018[2];        /* 0x0018 - 0x001F */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t COEF:24;
            } B;
        } COEF[9];              /* Filter Coefficient Registers @baseaddress + 0x20 - 0x40  */

        uint32_t decfil_reserved0044[13];       /* 0x0044 - 0x0077 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:8;
                vuint32_t TAP:24;
            } B;
        } TAP[8];               /* Filter TAP Registers @baseaddress + 0x78 - 0x94 */

        uint32_t decfil_reserved00D0[14];       /* 0x00D0 - 0x00D3 */

        /* 0x0D0 */
        union {
            vuint16_t R;
            struct {
                vuint32_t:16;
                vuint32_t SAMP_DATA:16;
            } B;
        } EDID;                 /* Filter EDID Registers @baseaddress + 0xD0 */

        uint32_t decfil_reserved00D4[3];    /* 0x00D4 - 0x00DF */

        union {
            vuint32_t R;
            struct {
                vuint32_t SUM_VALUE:1;
            } B;
        } FINTVAL;                   /* Final Integr. Value Register @baseaddress + 0xE0  */       

       union {
            vuint32_t R;
            struct {
                vuint32_t COUNT:1;
            } B;
        } FINTCNT;                   /* Final Integr. Count Register @baseaddress + 0xE0  */       

       union {
            vuint32_t R;
            struct {
                vuint32_t SUM_VALUE:1;
            } B;
        } CINTVAL;                   /* Current Integr. Value Register @baseaddress + 0xE0  */       

       union {
            vuint32_t R;
            struct {
                vuint32_t COUNT:1;
            } B;
        } CINTCNT;                   /* Current Integr. Count Register @baseaddress + 0xE0  */       

    };

/****************************************************************************/
/*                          MODULE : CRC                                   */
/****************************************************************************/
   struct CRC_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t:29;
                vuint32_t POLY:1;
                vuint32_t SWAP:1;
                vuint32_t INV:1;
            } B;
        } CFG;                      /* Configuration Register @baseaddress + 0x00 */

        union {
            vuint32_t R;
            struct{
                vuint32_t INP:32;
            } B;
        } INP;                      /* Input Register @baseaddress + 0x04 */

        union {
            vuint32_t R;
            struct{
                vuint32_t CSTAT:32;
            } B;
        } CSTAT;                    /* Current Status Register @baseaddress + 0x08 */

        union {
            vuint32_t R;
            struct {
                vuint32_t OUTP:32;
            } B;
        } OUTP;                     /* Output Register @baseaddress + 0x0C */
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
                vuint32_t PCSIS7:1;
                vuint32_t PCSIS6:1;
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
                  vuint32_t:6;
                vuint32_t PES:1;  
                vuint32_t HALT:1;
            } B;
        } MCR;                  /* Module Configuration Register @baseaddress + 0x00 */

        uint32_t dspi_reserved0004;         /* 0x0004-0x008 */

        union {
            vuint32_t R;
            struct {
                vuint32_t TCNT:16;
                  vuint32_t:16;
            } B;
        } TCR;                  /* DSPI Transfer Count Register  @baseaddress + 0x08 */

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
        } CTAR[8];              /* Clock and Transfer Attributes Registers @baseaddress + 0x0C - 0x28 */

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
                  vuint32_t:2;
                vuint32_t DPEF:1;
                vuint32_t SPEF:1;
                vuint32_t DDIF:1;
                vuint32_t RFOF:1;
                  vuint32_t:1;
                vuint32_t RFDF:1;
                  vuint32_t:1;
                vuint32_t TXCTR:4;
                vuint32_t TXNXTPTR:4;
                vuint32_t RXCTR:4;
                vuint32_t POPNXTPTR:4;
            } B;
        } SR;                   /* Status Register @baseaddress + 0x2C */

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
                  vuint32_t:1;
                vuint32_t DPEFRE:1;
                vuint32_t SPEFRE:1;
                vuint32_t DDIFRE:1;  
                vuint32_t RFOFRE:1;
                  vuint32_t:1;
                vuint32_t RFDFRE:1;
                vuint32_t RFDFDIRS:1;
                  vuint32_t:16;
            } B;
        } RSER;                 /* DMA/Interrupt Request Select and Enable Register @baseaddress + 0x30 */

        union {
            vuint32_t R;
            struct {
                vuint32_t CONT:1;
                vuint32_t CTAS:3;
                vuint32_t EOQ:1;
                vuint32_t CTCNT:1;
                vuint32_t PE:1;
                vuint32_t PP:1;
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
        } PUSHR;                /* PUSH TX FIFO Register @baseaddress + 0x34 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RXDATA:16;
            } B;
        } POPR;                 /* POP RX FIFO Register @baseaddress + 0x38 */

        union {
            vuint32_t R;
            struct {
                vuint32_t TXCMD:16;
                vuint32_t TXDATA:16;
            } B;
        } TXFR[4];              /* Transmit FIFO Registers @baseaddress + 0x3c - 0x78 */

        vuint32_t DSPI_reserved_004C[12];   /* 0x004C-0x0078 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RXDATA:16;
            } B;
        } RXFR[4];              /* Transmit FIFO Registers @baseaddress + 0x7c - 0xB8 */

        vuint32_t DSPI_reserved_008C[12];   /* 0x008C-0x00B8 */

        union {
            vuint32_t R;
            struct {
                vuint32_t MTOE:1;
                vuint32_t FMSZ4:1;
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
        } DSICR;                /* DSI Configuration Register @baseaddress + 0xBC */

        union {
            vuint32_t R;
            struct {
                vuint32_t SER_DATA:32;
            } B;
        } SDR;                  /* DSI Serialization Data Register @baseaddress + 0xC0 */

        union {
            vuint32_t R;
            struct {
                vuint32_t ASER_DATA:32;
            } B;
        } ASDR;                 /* DSI Alternate Serialization Data Register @baseaddress + 0xC4 */

        union {
            vuint32_t R;
            struct {
                vuint32_t COMP_DATA:32;
            } B;
        } COMPR;                /* DSI Transmit Comparison Register @baseaddress + 0xC8 */

        union {
            vuint32_t R;
            struct {
                vuint32_t DESER_DATA:32;
            } B;
        } DDR;                  /* DSI deserialization Data Register @baseaddress + 0xCC */

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
        } DSICR1;               /* DSI Configuration Register 1 @baseaddress + 0xD0 */

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
                  vuint32_t:1;
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
        } CR1;                  /* Control Register 1  @baseaddress + 0x00 */

        union {
            vuint16_t R;
            struct {
                vuint16_t MDIS:1;
                vuint16_t FBR:1;
                vuint16_t BSTP:1;
                vuint16_t IEBERR:1;
                vuint16_t RXDMA:1;
                vuint16_t TXDMA:1;
                vuint16_t BRK13:1;
                vuint16_t TXDIR:1;
                vuint16_t BESM13:1;
                vuint16_t SBSTP:1;
                vuint16_t RXPOL:1;
                vuint16_t PMSK:1;
                vuint16_t ORIE:1;
                vuint16_t NFIE:1;
                vuint16_t FEIE:1;
                vuint16_t PFIE:1;
            } B;
        } CR2;                  /* Control Register 2 @baseaddress + 0x04 */

        union {
            vuint16_t R;
            struct {
                vuint16_t R8:1;
                vuint16_t T8:1;
                vuint16_t ERR:1;
                  vuint16_t:1;
                vuint16_t R:4;
                vuint8_t D;
            } B;
        } DR;                   /* Data Register @baseaddress + 0x06 */

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
                vuint32_t RAF:1;
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
        } SR;                   /* Status Register   @baseaddress + 0x08 */

        union {
            vuint32_t R;
            struct {
                vuint32_t LRES:1;
                vuint32_t WU:1;
                vuint32_t WUD0:1;
                vuint32_t WUD1:1;
                vuint32_t LDBG:1;
                vuint32_t DSF:1;
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
        } LCR;                  /* LIN Control Register @baseaddress + 0x0C  */

        union {
            vuint32_t R;
        } LTR;                  /* LIN Transmit Register @baseaddress + 0x10 */

        union {
            vuint32_t R;
        } LRR;                  /* LIN Recieve Register @baseaddress + 0x14 */

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
        } LPR;                  /* LIN CRC Polynom Register  @baseaddress + 0x18 */

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
                vuint32_t FEN:1;
                vuint32_t HALT:1;
                vuint32_t NOTRDY:1;
                vuint32_t WAK_MSK:1;
                vuint32_t SOFTRST:1;
                vuint32_t FRZACK:1;
                vuint32_t SUPV:1;
                vuint32_t SLF_WAK:1;

                vuint32_t WRNEN:1;

                vuint32_t MDISACK:1;
                vuint32_t WAK_SRC:1;
                vuint32_t DOZE:1;

                vuint32_t SRXDIS:1;
                vuint32_t MBFEN:1;
                  vuint32_t:2;

                vuint32_t LPRIO_EN:1;
                vuint32_t AEN:1;
                  vuint32_t:2;
                vuint32_t IDAM:2;
                  vuint32_t:2;

                vuint32_t MAXMB:6;
            } B;
        } MCR;                  /* Module Configuration Register @baseaddress + 0x00 */

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
            } B;                /* Control Register @baseaddress + 0x04 */
        } CR;

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t TIMER:16;
            } B;
        } TIMER;                /* Free Running Timer @baseaddress + 0x08 */

        int32_t FLEXCAN_reserved00;

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t MI:29;
            } B;
        } RXGMASK;              /* RX Global Mask @baseaddress + 0x0C */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t MI:29;
            } B;
        } RX14MASK;             /* RX 14 Mask @baseaddress + 0x10 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:3;
                vuint32_t MI:29;
            } B;
        } RX15MASK;             /* RX 15 Mask @baseaddress + 0x14 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:16;
                vuint32_t RXECNT:8;
                vuint32_t TXECNT:8;
            } B;
        } ECR;                  /* Error Counter Register @baseaddress + 0x18 */

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
                vuint32_t WAK_INT:1;
            } B;
        } ESR;                  /* Error and Status Register @baseaddress + 0x1C */

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
            } B;                /* Interruput Masks Register @baseaddress + 0x20 */
        } IMRH;

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
            } B;                /* Interruput Masks Register @baseaddress + 0x24 */
        } IMRL;

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
            } B;                /* Interruput Flag Register @baseaddress + 0x28 */
        } IFRH;

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
            } B;                /* Interruput Flag Register @baseaddress + 0x2C */
        } IFRL;

        uint32_t flexcan2_reserved2[19];

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
            } B;                /* RX Individual Mask Registers @baseaddress + 0x0880 */
        } RXIMR[64];

    };                          /* end of FLEXCAN_tag */

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
        } PITMCR;               /* PIT Module Control Register @baseaddress + 0x00 */

        uint32_t pit_reserved1[59];

        struct {
            union {
                vuint32_t R;
            } LDVAL;            /* Timer Load Value Register @baseaddress + 0xF0 */

            union {
                vuint32_t R;
            } CVAL;             /* Current Timer Value Register @baseaddress + 0xF4 */

            union {
                vuint32_t R;
                struct {
                    vuint32_t:30;
                    vuint32_t TIE:1;
                    vuint32_t TEN:1;
                } B;
            } TCTRL;            /* Timer Control Register @baseaddress + 0xF8 */

            union {
                vuint32_t R;
                struct {
                    vuint32_t:31;
                    vuint32_t TIF:1;
                } B;
            } TFLG;             /* Timer Flag Register */
        } RTI;                  /* RTI Channel @baseaddress + 0xFC */

        struct {
            union {
                vuint32_t R;
            } LDVAL;            /* Timer Load Value Register @baseaddress + CH + 0x0 */

            union {
                vuint32_t R;
            } CVAL;             /* Current Timer Value Register @baseaddress + CH + 0x4 */

            union {
                vuint32_t R;
                struct {
                    vuint32_t:30;
                    vuint32_t TIE:1;
                    vuint32_t TEN:1;
                } B;
            } TCTRL;            /* Timer Control Register @baseaddress + CH + 0x8 */

            union {
                vuint32_t R;
                struct {
                    vuint32_t:31;
                    vuint32_t TIF:1;
                } B;
            } TFLG;             /* Timer Flag Register @baseaddress + CH + 0xC */
        } TIMER[4];             /* Timer Channels @baseaddress + 0x100 */

    };

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
            vuint16_t PRESCALE:3;       /* protocol engine clock prescaler */
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

            vuint16_t:1;
            vuint16_t LAST_MB_SEG1:7;   /* last message buffer control register for message buffer segment 1 */
              vuint16_t:1;
            vuint16_t LAST_MB_UTIL:7;   /* last message buffer utilized */
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

            vuint16_t:1;
            vuint16_t TBIVEC:7; /* transmit buffer interrupt vector */
              vuint16_t:1;
            vuint16_t RBIVEC:7; /* receive buffer interrupt vector */
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
            vuint16_t SCP:1;    /* cold start path */
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
              vuint16_t:4;
            vuint16_t RSBIDX:8; /* receive shadow buffer index */
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
                vuint16_t:8;
                vuint16_t MBIDX:8;      /* message buffer index */
            } B;
        } MBIDXR;
    } MSG_BUFF_CCS_t;
    typedef union uSYSBADHR {
        vuint16_t R;
    } SYSBADHR_t;
    typedef union uSYSBADLR {
        vuint16_t R;
    } SYSBADLR_t;
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
        vuint16_t reserved3a[1];        /*10 */
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
/*                     MODULE : Power Management Controller (PMC)           */
/****************************************************************************/
    struct PMC_tag {
        union {
            vuint32_t R;
            struct {
                vuint32_t LVRER:1;
                vuint32_t LVREH:1;
                vuint32_t LVRE50:1;
                vuint32_t LVRE33:1;
                vuint32_t LVREC:1;
                  vuint32_t:3;
                vuint32_t LVIER:1;
                vuint32_t LVIEH:1;
                vuint32_t LVIE50:1;
                vuint32_t LVIE33:1;
                vuint32_t LVIC:1;
                  vuint32_t:2;
                vuint32_t TLK:1;
                  vuint32_t:16;
            } B;
        } MCR;                  /* Module Configuration register @baseaddress + 0x00 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:12;
                vuint32_t LVDREGTRIM:4;
                vuint32_t VDD33TRIM:4;
                vuint32_t LVD33TRIM:4;
                vuint32_t VDDCTRIM:4;
                vuint32_t LVDCTRIM:4;
            } B;
        } TRIMR;                /* Trimming register @baseaddress + 0x00 */

        union {
            vuint32_t R;
            struct {
                vuint32_t:5;
                vuint32_t LVFVSTBY:1;
                vuint32_t BGRDY:1;
                vuint32_t BGTS:1;
                  vuint32_t:5;
                vuint32_t LVFCSTBY:1;
                  vuint32_t:1;
                vuint32_t V33DIS:1;
                vuint32_t LVFCR:1;
                vuint32_t LVFCH:1;
                vuint32_t LVFC50:1;
                vuint32_t LVFC33:1;
                vuint32_t LVFCC:1;
                  vuint32_t:3;
                vuint32_t LVFR:1;
                vuint32_t LVFH:1;
                vuint32_t LVF50:1;
                vuint32_t LVF33:1;
                vuint32_t LVFC:1;
                  vuint32_t:3;

            } B;
        } SR;                   /* status register @baseaddress + 0x00 */
    };

/****************************************************************************/
/*                          MODULE : MPU                                    */
/****************************************************************************/

    struct MPU_tag {

        union {                 /* Module Control/Error Status Register */
            vuint32_t R;
            struct {
                vuint32_t SPERR:8;
                  vuint32_t:4;
                vuint32_t HRL:4;
                vuint32_t NSP:4;
                vuint32_t NRGD:4;
                  vuint32_t:7;
                vuint32_t VLD:1;
            } B;
        } CESR;

        uint32_t MPU_reserved0004[3]; /* 0x0004-0x000F */

        struct {
            union {             /* MPU Error Address Registers */
                vuint32_t R;
                struct {
                    vuint32_t EADDR:32;
                } B;
            } EAR;

            union {             /* MPU Error Detail Registers */
                vuint32_t R;
                struct {
                    vuint32_t EACD:16;
                    vuint32_t EPID:8;
                    vuint32_t EMN:4;
                    vuint32_t EATTR:3;
                    vuint32_t ERW:1;
                } B;
            } EDR;
        } PORT[2];

        uint32_t MPU_reserved0020[248];  /* 0x0020-0x03FF */

        struct {
            union {            /* Region Descriptor n Word 0 */
                vuint32_t R;
                struct {
                    vuint32_t SRTADDR:27;
                    vuint32_t:5;
                } B;
            } WORD0;

            union {            /* Region Descriptor n Word 1 */
                vuint32_t R;
                struct {
                    vuint32_t ENDADDR:27;
                    vuint32_t:5;
                } B;
            } WORD1;

            union {            /* Region Descriptor n Word 2 */
                vuint32_t R;
                struct {
                    vuint32_t M7RE:1;
                    vuint32_t M7WE:1;
                    vuint32_t M6RE:1;
                    vuint32_t M6WE:1;
                    vuint32_t:2;
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
            } WORD2;

            union {            /* Region Descriptor n Word 3 */
                vuint32_t R;
                struct {
                    vuint32_t PID:8;
                    vuint32_t PIDMASK:8;
                      vuint32_t:15;
                    vuint32_t VLD:1;
                } B;
            } WORD3;
        } RGD[16];

        uint32_t MPU_reserved0500[192];  /* 0x0500-0x07FF */

        union {           /* Region Descriptor Alternate Access Control n */
            vuint32_t R;
            struct {
                vuint32_t M7RE:1;
                vuint32_t M7WE:1;
                vuint32_t M6RE:1;
                vuint32_t M6WE:1;
                vuint32_t:2;
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
        } RGDAAC[16];

        uint32_t MPU_reserved0840[3568];  /* 0x0840-0x3FFF */

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
        } TCCR0;                 /* Temperature Sensor Calibration B @baseaddress + 0x00 */

        union {                
            vuint32_t R;
            struct {
                 vuint32_t:16;
                vuint32_t TSCV3:16;
            } B;    
        } TCCR1;                 /* Temperature Sensor Calibration A @baseaddress + 0x04 */
        
        uint32_t TSENS_reserved0008[16382]; /* 0x0008-0xFFFF */

    };

/****************************************************************************/
/*					   MODULE : DTS (Development Trigger Semaphor)          */
/****************************************************************************/
    struct DTS_tag {
	    
	  union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:31;
	      	  vuint32_t DTS_EN:1;
		  }B;  
  	  } ENABLE; 					/* DTS Output Enable Register @baseaddress + 0x0 */

	  union {
		  vuint32_t R;
		  struct{
			  vuint32_t AD31:1;
			  vuint32_t AD30:1;
			  vuint32_t AD29:1;
			  vuint32_t AD28:1;
			  vuint32_t AD27:1;
			  vuint32_t AD26:1;
			  vuint32_t AD25:1;
			  vuint32_t AD24:1;
			  vuint32_t AD23:1;
			  vuint32_t AD22:1;
			  vuint32_t AD21:1;
			  vuint32_t AD20:1;
			  vuint32_t AD19:1;
			  vuint32_t AD18:1;
			  vuint32_t AD17:1;
			  vuint32_t AD16:1;
			  vuint32_t AD15:1;
			  vuint32_t AD14:1;
			  vuint32_t AD13:1;
			  vuint32_t AD12:1;
			  vuint32_t AD11:1;
			  vuint32_t AD10:1;
			  vuint32_t AD9:1;
			  vuint32_t AD8:1;
			  vuint32_t AD7:1;
			  vuint32_t AD6:1;
			  vuint32_t AD5:1;
			  vuint32_t AD4:1;
			  vuint32_t AD3:1;
			  vuint32_t AD2:1;
			  vuint32_t AD1:1;
			  vuint32_t AD0:1;
		  }B;  
  	  } STARTUP;					/* DTS Startup Register @baseaddress + 0x4 */
	  
	  union {
		  vuint32_t R;
		  struct {
			  vuint32_t ST31:1;
			  vuint32_t ST30:1;
			  vuint32_t ST29:1;
			  vuint32_t ST28:1;
			  vuint32_t ST27:1;
			  vuint32_t ST26:1;
			  vuint32_t ST25:1;
			  vuint32_t ST24:1;
			  vuint32_t ST23:1;
			  vuint32_t ST22:1;
			  vuint32_t ST21:1;
			  vuint32_t ST20:1;
			  vuint32_t ST19:1;
			  vuint32_t ST18:1;
			  vuint32_t ST17:1;
			  vuint32_t ST16:1;
			  vuint32_t ST15:1;
			  vuint32_t ST14:1;
			  vuint32_t ST13:1;
			  vuint32_t ST12:1;
			  vuint32_t ST11:1;
			  vuint32_t ST10:1;
			  vuint32_t ST9:1;
			  vuint32_t ST8:1;
			  vuint32_t ST7:1;
			  vuint32_t ST6:1;
			  vuint32_t ST5:1;
			  vuint32_t ST4:1;
			  vuint32_t ST3:1;
			  vuint32_t ST2:1;
			  vuint32_t ST1:1;
			  vuint32_t ST0:1;
		  }B;  
      } SEMAPHORE;				/* DTS Semaphore Register @baseaddress + 0x8 */
	  
	  uint32_t DTS_reserved000C[16381]; /* 0x000C-0xFFFF */	    
   
    };
    
/****************************************************************************/
/*					   MODULE : REACM (Reaction Module)          			*/
/****************************************************************************/
    struct REACM_tag {
	    
	  union {
		  vuint32_t R;
		  struct {
		  	  vuint32_t OVRC:1;
		  	  vuint32_t MDIS:1;
		  	  vuint32_t FRZ:1;
			   vuint32_t:1;
			  vuint32_t FREN:1;
			  vuint32_t TPREN:1;
			  vuint32_t HPREN:1;
			  vuint32_t GIEN:1;
			  vuint32_t OVREN:1; 
	      	   vuint32_t:23;
		  } B;  
      } MCR; 					/* REACM Module Configuration @baseaddress + 0x0 */
    
      union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:4;
	      	  vuint32_t HPRE:12;
	      	   vuint32_t:8;
	      	  vuint32_t TPRE:8;
		  } B;  
      } TCR; 					/* REACM Timer Configuration @baseaddress + 0x4 */
      
      union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:6;
	      	  vuint32_t WREN1:1;
	      	  vuint32_t WREN0:1;
	      	   vuint32_t:12;
	      	  vuint32_t THRADC1:4; 
	      	   vuint32_t:4;
	      	  vuint32_t THRADC0:4; 
		  } B;  
      } THRR; 					/* REACM Threshold Router @baseaddress + 0x8 */
      
      uint32_t REACM_reserved000C[1]; /* 0x000C-0x000F */
      
      union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:12;
		  	  vuint32_t ADC_TAG:4; 
	      	  vuint32_t ADC_RESULT:16;
		  } B;  
      } SINR; 					/* REACM ADC Sensor Input Register @baseaddress + 0x10 */
      
      uint32_t REACM_reserved0014[3]; /* 0x0014-0x0001F */
      
      union {
		  vuint32_t R;
		  struct {
		  	  vuint32_t OVR:1;
			   vuint32_t:26;
	      	  vuint32_t EF4:1;
	      	  vuint32_t EF3:1;
	      	  vuint32_t EF2:1;
	      	  vuint32_t EF1:1;
	      	  vuint32_t EF0:1;
		  } B;  
      } GEFR; 					/* REACM Global Error Flag @baseaddress + 0x20 */
      
	  uint32_t REACM_reserved0024[55]; /* 0x0024-0x00FF */      
	  
	  struct {
	  	  union {
			  vuint32_t R;
			  struct {
			  	  vuint32_t CHEN:2;
			  	  vuint32_t SWMC:1;
			  	  vuint32_t MAXLEN:1;
			  	  vuint32_t OCDFEN:1;
			  	  vuint32_t SCDFEN:1;
			  	  vuint32_t TAEREN:1;
			  	  vuint32_t SQEREN:1;
			  	  vuint32_t RAEREN:1;
				   vuint32_t:1;
		      	  vuint32_t CHOFF:1;
		      	   vuint32_t:2;
		      	  vuint32_t DOFF:3;
		      	   vuint32_t:5;
		      	  vuint32_t BSB:3;
		      	   vuint32_t:2;
		      	  vuint32_t MODULATION_ADDR:6;
			  } B;  
	      } CR; 					/* REACM Channel n Configuration @baseaddress + 0x100 + (n*0x10) + 0x0 */
	      
		  union {
			  vuint32_t R;
			  struct {
			  	   vuint32_t:2;
		      	  vuint32_t MODACT:1;
		      	  vuint32_t MAXL:1;
		      	  vuint32_t OCDF:1;
		      	  vuint32_t SCDF:1;
		      	  vuint32_t TAER:1;
		      	  vuint32_t SQER:1;
		      	  vuint32_t RAER:1;
		      	  vuint32_t CHOUT:3;
		      	   vuint32_t:7;
		      	  vuint32_t MAXLC:1;
		      	  vuint32_t OCDFC:1;
		      	  vuint32_t SCDFC:1;
		      	  vuint32_t TAERC:1;
		      	  vuint32_t SQERC:1;
		      	  vuint32_t RAERC:1; 
		      	   vuint32_t:1;
		      	  vuint32_t MODULATION_POINTER:6; 
			  } B;  
	      } SR; 					/* REACM Channel n Status @baseaddress + 0x100 + (n*0x10) + 0x4 */
		  
	      union {
			  vuint32_t R;
			  struct {
		      	   vuint32_t:12;
		      	  vuint32_t ADCR:4;
		      	   vuint32_t:12;
		      	  vuint32_t CHIR:4;
			  } B;  
	      } RR; 					/* REACM Channel n Router @baseaddress + 0x100 + (n*0x10) + 0x8 */
	      
	      uint32_t REACM_reserved01xC; /* 0x01xC-0x01xF */      
	      
      } CH[6];

      uint32_t REACM_reserved0160[104]; /* 0x0160-0x02FF */      
      
      union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:16;
	      	  vuint32_t SHARED_TIMER:16;
		  } B;  
      } STBK[16]; 					/* REACM Shared Timer Bank @baseaddress + 0x300 */
      
      uint32_t REACM_reserved0340[16]; /* 0x0340-0x037F */      
      
      union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:20;
	      	  vuint32_t HOLD_OFF:12;
		  } B;  
      } HOTBK[16]; 					/* REACM Hold-off Timer Bank @baseaddress + 0x380 */
      
      uint32_t REACM_reserved03C0[16]; /* 0x03C0-0x03FF */      
      
      union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:16;
	      	  vuint32_t THRESHOLD_VALUE:16;
		  } B;  
      } THBK[32]; 					/* REACM Threshold Timer Bank @baseaddress + 0x400 */
      
      uint32_t REACM_reserved0480[96]; /* 0x0480-0x05FF */      
      
      union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:16;
	      	  vuint32_t ADC_MAX_LIMIT:16;
		  } B;  
      } ADCMAX; 					/* REACM ADC Result Max Limit Check @baseaddress + 0x600 */
      
      uint32_t REACM_reserved0604[31]; /* 0x0604-0x067F */      
      
      union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:20;
	      	  vuint32_t RANGE_PWD:12;
		  } B;  
      } RANGEPWD; 					/* REACM Modulation Range Pulse Width @baseaddress + 0x680 */
      
      uint32_t REACM_reserved0684[15]; /* 0x0684-0x06BF */      
      
      union {
		  vuint32_t R;
		  struct {
		  	   vuint32_t:20;
	      	  vuint32_t MIN_PWD:12;
		  } B;  
      } MINPWD; 					/* REACM Modulation Minimum Pulse Width @baseaddress + 0x6C0 */
      
      uint32_t REACM_reserved06C4[15]; /* 0x06C4-0x06FF */      
      
      union {
		  vuint32_t R;
		  struct {
		  	  vuint32_t LOOP:1;
		  	  vuint32_t IOSS:1;
			   vuint32_t:1;
	      	  vuint32_t MM:2;
	      	   vuint32_t:1;
	      	  vuint32_t SM:2;
	      	   vuint32_t:1;
	      	  vuint32_t HOD:3;
	      	   vuint32_t:1;
	      	  vuint32_t LOD:3;
	      	   vuint32_t:1;
	      	  vuint32_t THRESPT:6;
	      	  vuint32_t STPT:4;
	      	   vuint32_t:1;
	      	  vuint32_t HDOFFTPT:4;
		  } B;  
      } MWBK[64]; 					/* REACM Modulation Control Word Bank @baseaddress + 0x700 */
     
	};



    
/* Define memories */

#define SRAM_START    0x40000000
#define SRAM_SIZE        0x30000
#define SRAM_END      0x4002FFFF

#define FLASH_START   0x00000000
#define FLASH_SIZE      0x400000
#define FLASH_END     0x003FFFFF

/* Define instances of modules */
#define FMPLL     (*( volatile struct FMPLL_tag *)      0xC3F80000)
#define EBI       (*( volatile struct EBI_tag *)        0xC3F84000)
#define FLASH_A   (*( volatile struct FLASH_tag *)      0xC3F88000)
#define FLASH_B   (*( volatile struct FLASH_tag *)      0xC3F8C000)
#define SIU       (*( volatile struct SIU_tag *)        0xC3F90000)
#define DTS       (*( volatile struct DTS_tag *)        0xC3F9C000)

#define EMIOS     (*( volatile struct EMIOS_tag *)      0xC3FA0000)
#define PMC       (*( volatile struct PMC_tag *)        0xC3FBC000)

#define ETPU      (*( volatile struct ETPU_tag *)       0xC3FC0000)
#define ETPU_DATA_RAM      (*( uint32_t *)              0xC3FC8000)
#define ETPU_DATA_RAM_END                               0xC3FC8BFC
#define ETPU_DATA_RAM_EXT  (*( uint32_t *)              0xC3FCC000)
#define CODE_RAM           (*( uint32_t *)              0xC3FD0000)
#define ETPU_CODE_RAM      (*( uint32_t *)              0xC3FD0000)

#define REACM	  (*( volatile struct REACM_tag *)      0xC3FC7000)

#define PIT       (*( volatile struct PIT_tag *)        0xC3FF0000)

#define CRC       (*( volatile struct CRC_tag *)        0xFFE68000)

#define PBRIDGE   (*( volatile struct PBRIDGE_tag *)    0xFFF00000)
#define XBAR      (*( volatile struct XBAR_tag *)       0xFFF04000)
#define MPU       (*( volatile struct MPU_tag *)        0xFFF10000)
#define SWT       (*( volatile struct SWT_tag *)        0xFFF38000)
#define STM       (*( volatile struct STM_tag *)        0xFFF3C000)
#define ECSM      (*( volatile struct ECSM_tag *)       0xFFF40000)
#define EDMA      (*( volatile struct EDMA_tag *)       0xFFF44000)
#define INTC      (*( volatile struct INTC_tag *)       0xFFF48000)

#define EQADC     (*( volatile struct EQADC_tag *)      0xFFF80000)

#define DECFIL_A   (*( volatile struct DECFIL_tag *)      0xFFF88000)
#define DECFIL_B   (*( volatile struct DECFIL_tag *)      0xFFF8C000)

#define DSPI_B    (*( volatile struct DSPI_tag *)       0xFFF94000)
#define DSPI_C    (*( volatile struct DSPI_tag *)       0xFFF98000)
#define DSPI_D    (*( volatile struct DSPI_tag *)       0xFFF9C000)

#define ESCI_A    (*( volatile struct ESCI_tag *)       0xFFFB0000)
#define ESCI_B    (*( volatile struct ESCI_tag *)       0xFFFB4000)
#define ESCI_C    (*( volatile struct ESCI_tag *)       0xFFFB8000)

#define CAN_A     (*( volatile struct FLEXCAN2_tag *)   0xFFFC0000)
#define CAN_B     (*( volatile struct FLEXCAN2_tag *)   0xFFFC4000)
#define CAN_C     (*( volatile struct FLEXCAN2_tag *)   0xFFFC8000)

#define FR        (*( volatile struct FR_tag *)         0xFFFE0000)
#define TSENS     (*( volatile struct TSENS_tag *)      0xFFFEC000)


#ifdef __MWERKS__
#pragma pop
#endif

#ifdef  __cplusplus
}
#endif
#endif                          /* ifdef _MPC5644_H */

/*********************************************************************
 *
 * Copyright:
 *  Freescale Semiconductor, INC. & STMicroelectronics All Rights Reserved.
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
 *  Semiconductor & STMicroelectronics DISCLAIMS ALL WARRANTIES WHETHER 
 *  EXPRESS OR IMPLIED,INCLUDING IMPLIED WARRANTIES OF MERCHANTABILITY OR 
 *  FITNESS FOR A PARTICULAR PURPOSE AND ANY WARRANTY AGAINST INFRINGEMENT WITH
 *  REGARD TO THE SOFTWARE (INCLUDING ANY MODIFIED VERSIONS THEREOF)
 *  AND ANY ACCOMPANYING WRITTEN MATERIALS.
 *
 *  To the maximum extent permitted by applicable law, IN NO EVENT
 *  SHALL Freescale Semiconductor or STMicroelectronics BE LIABLE FOR ANY 
 *  DAMAGES WHATSOEVER (INCLUDING WITHOUT LIMITATION, DAMAGES FOR LOSS OF 
 *  BUSINESS PROFITS, BUSINESS INTERRUPTION, LOSS OF BUSINESS INFORMATION, 
 *  OR OTHER PECUNIARY LOSS) ARISING OF THE USE OR INABILITY TO USE THE SOFTWARE.
 *
 *  Freescale Semiconductor & STMicroelectronics assumes no responsibility 
 *  for the maintenance and support of this software
 *
 ********************************************************************/
