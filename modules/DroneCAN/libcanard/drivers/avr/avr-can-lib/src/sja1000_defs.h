// ----------------------------------------------------------------------------
/*
 * Copyright (c) 2007 Fabian Greif, Roboterclub Aachen e.V.
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id$
 */
// ----------------------------------------------------------------------------

#ifndef	SJA1000_DEFS_H
#define	SJA1000_DEFS_H

/**
 * \name	Adressen der Register des SJA1000 im Intel Mode
 *
 * PeliCAN Modus
 */
/*@{*/

#define MOD			0x00 // mode
#define CR			0x00 // control

#define CMR			0x01 // command
#define SR			0x02 // status
#define IR			0x03 // interrupt

#define BTR0		0x06 // bit timing 0
#define BTR1		0x07 // bit timing 1
#define OCR			0x08 // output control
#define CDR			0x1F // clock divider

#define IER			0x04 // interrupt enable

#define ALC			0x0B // arbitration lost capture
#define ECC			0x0C // error code capture
#define EWL			0x0D // error_warning_limit
#define RXERR		0x0E // RX error counter
#define TXERR		0x0F // TX error counter
	
#define RMC			0x1D // RX message counter
#define RBSA		0x1E // RX_buffer_start_adress
	
#define ACR0		0x10 // acceptance code0
#define ACR1		0x11 // acceptance code1
#define ACR2		0x12 // acceptance code2
#define ACR3		0x13 // acceptance code3
#define AMR0		0x14 // acceptance mask0
#define AMR1		0x15 // acceptance mask1
#define AMR2		0x16 // acceptance mask2
#define AMR3		0x17 // acceptance mask3

#define RX_INFO		0x10
#define RX_ID1		0x11
#define RX_ID0		0x12
#define RX_DATA0	0x13
#define RX_DATA1	0x14
#define RX_DATA2	0x15
#define RX_DATA3	0x16
#define RX_DATA4	0x17
#define RX_DATA5	0x18
#define RX_DATA6	0x19
#define RX_DATA7	0x1A

#define TX_INFO		0x10
#define TX_ID1		0x11
#define TX_ID0		0x12
#define TX_DATA0	0x13
#define TX_DATA1	0x14
#define TX_DATA2	0x15
#define TX_DATA3	0x16
#define TX_DATA4	0x17
#define TX_DATA5	0x18
#define TX_DATA6	0x19
#define TX_DATA7	0x1A

/*@}*/

/**
 * \name	Bitdefinition der verschiedenen Register
 */
/*@{*/

/**
 * \brief	Bitdefinition von MOD
 */
#define SM			4
#define AFM			3
#define STM			2
#define LOM			1
#define RM			0

/**
 * \brief	Bitdefinition von CMR
 */
#define SRR			4
#define CDO			3
#define RRB			2
#define AT			1
#define TR			0

/**
 * \brief	Bitdefinition von SR
 */
#define BS			7
#define ES			6
#define TS			5
#define RS			4
#define TCS			3
#define TBS			2
#define DOS			1
#define RBS			0

/**
 * \brief	Bitdefinition von IR
 */
#define BEI			7
#define ALI			6
#define EPI			5
#define WUI			4 
#define DOI			3
#define EI			2
#define TI			1
#define RI			0

/**
* \brief	Bitdefinition von IER / CR
 */
#define BEIE		7
#define ALIE		6
#define EPIE		5
#define WUIE		4
#define DOIE		3
#define EIE			2
#define TIE			1
#define RIE			0

/**
 * \brief	Bitdefinition von BTR0
 */
#define _SJW1		7
#define _SJW0		6
#define _BRP5		5
#define _BRP4		4
#define _BRP3		3
#define _BRP2		2
#define _BRP1		1
#define _BRP0		0

/**
 * \brief	Bitdefinition von BTR1
 */
#define SAM			7
#define TSEG22		6
#define TSEG21		5
#define TSEG20		4
#define TSEG13		3
#define TSEG12		2
#define TSEG11		1
#define TSEG10		0

/**
 * \brief	Bitdefinition von OCR
 */
#define OCTP1		7
#define OCTN1		6
#define OCPOL1		5
#define OCTP0		4
#define OCTN0		3
#define OCPOL0		2
#define OCMODE1		1
#define OCMODE0		0

/**
 * \brief	Bitdefinition von ALC
 */
#define BITNO4		4
#define BITNO3		3
#define BITNO2		2
#define BITNO1		1
#define BITNO0		0

/**
 * \brief	Bitdefinition von ECC
 */
#define ERRC1		7
#define ERRC0		6
#define DIR			5
#define SEG4		4
#define SEG3		3
#define SEG2		2
#define SEG1		1
#define SEG0		0

/**
 * \brief	Bitdefinition von EWL
 */
#define ERRC1		7
#define ERRC0		6
#define DIR			5
#define SEG4		4
#define SEG3		3
#define SEG2		2
#define SEG1		1
#define SEG0		0

/**
 * \brief	Bitdefinition von CDR
 */
#define CANMODE		7
#define CBP			6
#define RXINTEN		5
#define CLKOFF		3
#define CD2			2
#define CD1			1
#define CD0			0

/**
 * \brief   Bitdefinition von RX_INFO und TX_INFO
 */
#define	FF			7
#define	RTR			6	

/*@}*/

#endif	// SJA1000_DEFS_H
