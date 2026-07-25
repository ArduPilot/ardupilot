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

#ifndef	SJA1000_PRIVATE_H
#define	SJA1000_PRIVATE_H

// ----------------------------------------------------------------------------
/**
 * \file	sja1000_private.h
 * \brief	SJA1000 Interface
 *
 * \author	Henning Schepker
 * \author	Fabian Greif <fabian.greif@rwth-aachen.de>
 *
 * \version	$Id$
 */
// ----------------------------------------------------------------------------

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "can_private.h"

#include "can.h"
#include "utils.h"

#if (BUILD_FOR_SJA1000 == 1)
	// set some default-values
	#ifndef SJA1000_MEMORY_MAPPED
		#define SJA1000_MEMORY_MAPPED   0
	#endif
	
	// check if the settings are correct
	#if SUPPORT_EXTENDED_CANID == 0
		#error	Extended CANIDs need to be supported!
	#endif
	
	#ifndef	SJA1000_INT
		#error	SJA1000_INT	is not defined!
	#endif
	
	#ifndef SJA1000_CLKOUT_PRESCALER
		#define	SJA1000_CLOCK_REGISTER		(1<<CLKOFF)
	#else
		#if SJA1000_CLKOUT_PRESCALER == 1
			#define	SJA1000_CLOCK_REGISTER		((1<<CD2)|(1<<CD1)|(1<<CD0))
		#elif SJA1000_CLKOUT_PRESCALER == 2
			#define	SJA1000_CLOCK_REGISTER		0
		#elif SJA1000_CLKOUT_PRESCALER == 4
			#define	SJA1000_CLOCK_REGISTER		(1<<CD0)
		#elif SJA1000_CLKOUT_PRESCALER == 6
			#define	SJA1000_CLOCK_REGISTER		(1<<CD1)
		#elif SJA1000_CLKOUT_PRESCALER == 8
			#define	SJA1000_CLOCK_REGISTER		((1<<CD1)|(1<<CD0))
		#elif SJA1000_CLKOUT_PRESCALER == 10
			#define	SJA1000_CLOCK_REGISTER		(1<<CD2)
		#elif SJA1000_CLKOUT_PRESCALER == 12
			#define	SJA1000_CLOCK_REGISTER		((1<<CD2)|(1<<CD0))
		#elif SJA1000_CLKOUT_PRESCALER == 14
			#define	SJA1000_CLOCK_REGISTER		((1<<CD2)|(1<<CD1))
		#endif
	#endif
	
	#if SJA1000_MEMORY_MAPPED
		#ifndef	SJA1000_BASE_ADDR
			#error	SJA1000_BASE_ADDR is not defined!
		#endif
		
		#define	SUPPORT_FOR_SJA1000__		1
		
		// write to a register
		static inline void sja1000_write(uint8_t address, uint8_t data) {
			(*((uint8_t *) (SJA1000_BASE_ADDR + address))) = data;
		}

		// read a register
		static inline uint8_t sja1000_read(uint8_t address) {
			return (*((uint8_t *) (SJA1000_BASE_ADDR + address)));
		}

	#else
		#warning    not tested yet!

		#if !defined(SJA1000_WR) || !defined(SJA1000_RD) || \
			!defined(SJA1000_CS) || !defined(SJA1000_DATA) || !defined(SJA1000_ALE)
			#error in definition of SJA1000-pins (check SJA1000_WR, SJA1000_RD, SJA1000_CS, SJA1000_DATA and SJA1000_ALE)!
		#endif
		
		#define	SUPPORT_FOR_SJA1000__		1
		extern void sja1000_write(uint8_t address, uint8_t data);
		extern uint8_t sja1000_read(uint8_t address);
	#endif	// SJA1000_MEMORY_MAPPED

	#ifdef  SUPPORT_FOR_SJA1000__
		#include "sja1000_defs.h"
	#endif
#endif	// SUPPORT_SJA1000

#endif	// SJA1000_PRIVATE_H
