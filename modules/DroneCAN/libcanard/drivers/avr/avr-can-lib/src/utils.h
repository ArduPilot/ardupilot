// coding: utf-8
// ----------------------------------------------------------------------------
/*
 * Copyright (c) 2007 Roboterclub Aachen e.V.
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
 */
// ----------------------------------------------------------------------------

#ifndef	UTILS_H
#define	UTILS_H

// ----------------------------------------------------------------------------
/**
 * \defgroup utils_h Gruppe nützlicher Makros und Inline-Funktionen
 * \brief	Nützliche Makros und Funktionen.
 *
 * \version	$Id: utils.h 8541 2010-03-15 22:01:12Z fabian $
 */
// ----------------------------------------------------------------------------

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

//#include "config.h"

// ----------------------------------------------------------------------------

#ifndef	TRUE
	#define	TRUE	(1==1)
#elif !TRUE
	#error	fehlerhafte Definition fuer TRUE
#endif

#ifndef FALSE
	#define	FALSE	(1!=1)
#elif FALSE
	#error	fehlerhafte Definition fuer FALSE
#endif

#ifndef NULL
	#define NULL ((void*)0)		//!< Nullzeiger
#endif

// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \name	Nützliches
 */
//@{

#define	DEGREE_TO_RAD(x)	((x * M_PI) / 180)

//@}
// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \name	Nützliches
 */
//@{

#define	LOW_BYTE(x)		((uint8_t) (x & 0xff))
#define	HIGH_BYTE(x)	((uint8_t) (x >> 8))
#define LOW_WORD(x)		((uint16_t) (x & 0xffff))
#define HIGH_WORD(x)    ((uint16_t) (x >> 16))

//@}
// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 */
typedef struct {
	uint8_t b4;		// lsb
	uint8_t b3;
	uint8_t b2;
	uint8_t b1;		// msb
} long_to_byte_t;

// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \name	Kritische Sektionen (Interrupts sperren)
 */
//@{

#if defined(__DOXYGEN__)

#define	ENTER_CRITICAL_SECTION
#define	LEAVE_CRITICAL_SECTION

#else /* !DOXYGEN */

#if __AVR_LIBC_VERSION__ >= 10600 && !defined (__cplusplus)

	#include <util/atomic.h>
	
	#define	ENTER_CRITICAL_SECTION		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	#define	LEAVE_CRITICAL_SECTION		}
	
	#define	IRQ_LOCK					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)

#else

	#define	ENTER_CRITICAL_SECTION		do { unsigned char sreg_ = SREG; cli();
	#define	LEAVE_CRITICAL_SECTION		SREG = sreg_; } while (0);

#endif
#endif /* DOXYGEN */

//@}
// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \brief	atomare Operationen
 */
static inline uint8_t read_and_replace_atomar(volatile uint8_t *data, uint8_t new_data)
{
	uint8_t old_data;
	
	ENTER_CRITICAL_SECTION
	
	// Daten tauschen
	old_data = *data;
	*data = new_data;
	
	LEAVE_CRITICAL_SECTION
	
	return old_data;
}

// ----------------------------------------------------------------------------
/**
 * \ingroup utils_h
 * \name    Volatile Zugriff auf Variablen
 */
//@{

#define	vu8(x)	(*(volatile  uint8_t*)&(x))
#define	vs8(x)	(*(volatile   int8_t*)&(x))
#define	vu16(x)	(*(volatile uint16_t*)&(x))
#define	vs16(x)	(*(volatile  int16_t*)&(x))
#define	vu32(x)	(*(volatile uint32_t*)&(x))
#define	vs32(x)	(*(volatile  int32_t*)&(x))

//@}
// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \name	Port-Makros
 *
 * Die Makros RESET(), SET(), SET_OUTPUT(), SET_INPUT() und IS_SET()
 * beziehen sich immer auf ein bestimmtes Bit eines Ports und helfen somit
 * den Code sehr portabel zu gestalten.
 *
 * Beispiel:
 * \code
 * #define LED   D,5		// PORTD, Pin 5
 *
 * SET_OUTPUT(LED);		// Pin als Ausgang schalten (wird z.B. zu DDRD |= (1<<5);)
 * 
 * SET(LED);				// LED aktivieren
 * \endcode
 *
 * oder auch:
 *
 * \code
 * #define SCHALTER   B,1		// PORTB, Pin 1
 * 
 * SET_INPUT_WITH_PULLUP(SCHALTER);
 * 
 * if (IS_SET(SCHALTER)) {
 * 		...
 * }
 * \endcode
 * 
 * Somit muss nur ein Define geändert werden sobald ein anderer Pin verwendet
 * werden soll. Außerdem muss nicht immer noch ein extra Define für den 
 * entsprechenden Port angelegt werden wie es bisher immer der Fall war.
 */
//@{
#if defined(__DOXYGEN__)

#define RESET(x)		//!< Einzelnes Bit eines bestimmten Ports setzen
#define SET(x)			//!< Bit löschen
#define	TOGGLE(x)		//!< Bit umschalten

#define	SET_OUTPUT(x)	//!< Einzeles Bit ein Port als Ausgang schalten
#define	SET_INPUT(x)	//!< Bit als Eingang schalten
#define	SET_PULLUP(x)	//!< aktiviert den Pullup eines Pins (nur falls dieser als Eingang geschaltet ist)

#define	SET_INPUT_WITH_PULLUP(x)	//!< Set den Pin als Eingang mit akiviertem Pullup

#define	IS_SET(x)		//!< Zustand eines Eingangs abfragen

#else /* !DOXYGEN */

/* Warum hier zum Teil so seltsame Konstrukte notwendig sind wird zum Beispiel
 * in http://www.mikrocontroller.net/forum/read-1-324854.html#324980 erklärt.
 */
#define	PORT(x)			_port2(x)
#define	DDR(x)			_ddr2(x)
#define	PIN(x)			_pin2(x)
#define	REG(x)			_reg(x)
#define	PIN_NUM(x)		_pin_num(x)

#define	RESET(x)		RESET2(x)
#define	SET(x)			SET2(x)
#define	TOGGLE(x)		TOGGLE2(x)
#define	SET_OUTPUT(x)	SET_OUTPUT2(x)
#define	SET_INPUT(x)	SET_INPUT2(x)
#define	SET_PULLUP(x)	SET2(x)
#define	IS_SET(x)		IS_SET2(x)

#define	SET_INPUT_WITH_PULLUP(x)	SET_INPUT_WITH_PULLUP2(x)

#define	_port2(x)	PORT ## x
#define	_ddr2(x)	DDR ## x
#define	_pin2(x)	PIN ## x

#define	_reg(x,y)		x
#define	_pin_num(x,y)	y

#define	RESET2(x,y)		PORT(x) &= ~(1<<y)
#define	SET2(x,y)		PORT(x) |= (1<<y)
#define	TOGGLE2(x,y)	PORT(x) ^= (1<<y)

#define	SET_OUTPUT2(x,y)	DDR(x) |= (1<<y)
#define	SET_INPUT2(x,y)		DDR(x) &= ~(1<<y)
#define	SET_INPUT_WITH_PULLUP2(x,y)	SET_INPUT2(x,y);SET2(x,y)

#define	IS_SET2(x,y)	((PIN(x) & (1<<y)) != 0)

#endif /* DOXYGEN */
//@}

// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \name	weitere nuetzliche Makros
 */
//@{
#if defined(__DOXYGEN__)

#define	_bit_is_set(pin, bit)	//!< Überprüft ob das entsprechende Bit gesetzt ist
#define	_bit_is_clear(pin, bit)	//!< Überprüft ob das entsprechende Bit gelöscht ist

#define	STRING(x)		//!< erstellt einen String aus dem übergebenen Wert

#else /* !DOXYGEN */

#define	_bit_is_set(pin, bit)	(pin & (1<<bit))
#define	_bit_is_clear(pin, bit)	(!(pin & (1<<bit)))

#define	STRING(x)	_STRING(x)
#define	_STRING(x)	#x

#endif /* DOXYGEN */
//@}

// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \brief	Dreht die beiden Nibble in einem Byte um.
 *
 * \param	x	Byte das verarbeitet werden soll.
 */
static inline uint8_t swap (uint8_t x)
{
	if (__builtin_constant_p(x))
		x = (x << 4) | (x >> 4);
	else
		asm volatile ("swap %0" : "=r" (x) : "0" (x));
	
	return x;
}

// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \brief	
 */
#if defined(DEBUG_LEVEL) && DEBUG_LEVEL
	#include <stdio.h>
	#define	DEBUG_PRINT(s, ...)	do { static const char __s[] PROGMEM = (s); \
				printf_P(__s, ## __VA_ARGS__); } while (0)
#else
	#define	DEBUG_PRINT(s, ...)

#endif

// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \brief	Zählt die Anzahl der gesetzten Bits in einem Byte
 *
 * \param	n	Byte das verarbeitet werden soll.
 * \return	Anzahl der gesetzten Bits (0..8)
 *
 * \see		http://infolab.stanford.edu/~manku/bitcount/bitcount.html
 */
static inline uint8_t bit_count8(uint8_t n)
{
	n = ((n >> 1) & 0x55) + (n & 0x55);
	n = ((n >> 2) & 0x33) + (n & 0x33);
	n = ((n >> 4) + n) & 0xf;
	
	return n;
}

#define MASK_01010101 (((uint32_t)(-1))/3)
#define MASK_00110011 (((uint32_t)(-1))/5)
#define MASK_00001111 (((uint32_t)(-1))/17)

// ----------------------------------------------------------------------------
/**
 * \ingroup	utils_h
 * \brief	Zählt die Anzahl der gesetzten Bits in einem Byte
 * 
 * \param	n	Wert der verarbeitet werden soll.
 * \return	Anzahl der gesetzten Bits (0..32)
 */
static inline uint8_t bit_count32(uint32_t n)
{
	n = (n & MASK_01010101) + ((n >> 1) & MASK_01010101);
	n = (n & MASK_00110011) + ((n >> 2) & MASK_00110011);
	n = (n & MASK_00001111) + ((n >> 4) & MASK_00001111);
	
	return n % 255 ;
}


#define	START_TIMED_BLOCK(time, gettime) \
	do { \
		static uint16_t last_time__; \
		uint16_t current_time__ = gettime; \
		if ((uint16_t) (current_time__ - last_time__) > time) { \
			last_time__ = current_time__;

#define END_TIMED_BLOCK \
		} \
	} while (0); 

// ----------------------------------------------------------------------------
#define	TO_DEG(x)		(x * 180.0 / M_PI)
#define	TO_RAD(x)		(x * M_PI / 180.0)

/**
 * \brief	Macro to supress "unused argument" warnings from the compiler
 * 
 * \todo	 __attribute__((unused)) for the gcc?
 */
#define	USE_IT(x)	(void) x

#endif	// UTILS_H
