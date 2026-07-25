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
 * $Id: at90can.c 7538 2009-04-16 13:07:33Z fabian $
 */
// ----------------------------------------------------------------------------

#include "at90can_private.h"
#ifdef	SUPPORT_FOR_AT90CAN__

#include <avr/pgmspace.h>
#include "can_buffer.h"

// ----------------------------------------------------------------------------

const uint8_t _at90can_cnf[8][3] PROGMEM = {
	// 10 kbps
	{	0x7E,
		0x6E,
		0x7F
	},
	// 20 kbps
	{	0x62,
		0x0C,
		0x37
	},
	// 50 kbps
	{	0x26,
		0x0C,
		0x37
	},
	// 100 kbps
	{	0x12,
		0x0C,
		0x37
	},
	// 125 kbps
	{	0x0E,
		0x0C,
		0x37
	},
	// 250 kbps
	{	0x06,
		0x0C,
		0x37
	},
	// 500 kbps
	{	0x02,
		0x0C,
		0x37
	},
	// 1 Mbps
	{	0x00,
		0x0C,
		0x36
	}
};

// ----------------------------------------------------------------------------

#if CAN_RX_BUFFER_SIZE > 0
can_buffer_t can_rx_buffer;
can_t can_rx_list[CAN_RX_BUFFER_SIZE];
#else
volatile uint8_t _messages_waiting;
#endif

#if CAN_TX_BUFFER_SIZE > 0
can_buffer_t can_tx_buffer;
can_t can_tx_list[CAN_TX_BUFFER_SIZE];

volatile uint8_t _transmission_in_progress = 0;
#else
volatile uint8_t _free_buffer;			//!< Stores the numer of currently free MObs
#endif

// ----------------------------------------------------------------------------
// get next free MOb

uint8_t _find_free_mob(void)
{
	#if CAN_TX_BUFFER_SIZE == 0
	if (_free_buffer == 0)
		return 0xff;
	#elif CAN_FORCE_TX_ORDER
	if (_transmission_in_progress)
		return 0xff;
	#endif
	
	uint8_t i;
	for (i = 0;i < 15;i++)
	{
		// load MOb page
		CANPAGE = i << 4;
		
		// check if MOb is in use
		if ((CANCDMOB & ((1 << CONMOB1) | (1 << CONMOB0))) == 0)
			return i;
	}
	
	return 0xff;
}

// ----------------------------------------------------------------------------
// disable interrupt of corresponding MOb

void _disable_mob_interrupt(uint8_t mob)
{
	if (mob < 8)
		CANIE2 &= ~(1 << mob);
	else
		CANIE1 &= ~(1 << (mob - 8));
}

// ----------------------------------------------------------------------------
// enable interrupt of corresponding MOb

void _enable_mob_interrupt(uint8_t mob)
{
	if (mob < 8)
		CANIE2 |= (1 << mob);
	else
		CANIE1 |= (1 << (mob - 8));
}

// ----------------------------------------------------------------------------

bool at90can_init(uint8_t bitrate)
{
	if (bitrate >= 8)
		return false;
	
	// switch CAN controller to reset mode
	CANGCON |= (1 << SWRES);
	
	// set CAN Bit Timing
	// (see datasheet page 260)
	CANBT1 = pgm_read_byte(&_at90can_cnf[bitrate][0]);
	CANBT2 = pgm_read_byte(&_at90can_cnf[bitrate][1]);
	CANBT3 = pgm_read_byte(&_at90can_cnf[bitrate][2]);
	
	// activate CAN transmit- and receive-interrupt
	CANGIT = 0;
	CANGIE = (1 << ENIT) | (1 << ENRX) | (1 << ENTX);
	
	// set timer prescaler to 199 which results in a timer
	// frequency of 10 kHz (at 16 MHz)
	CANTCON = 199;
	
	// disable all filters
	at90can_disable_filter( 0xff );
	
	#if CAN_RX_BUFFER_SIZE > 0
	can_buffer_init( &can_rx_buffer, CAN_RX_BUFFER_SIZE, can_rx_list );
	#endif
	
	#if CAN_TX_BUFFER_SIZE > 0
	can_buffer_init( &can_tx_buffer, CAN_TX_BUFFER_SIZE, can_tx_list );
	#endif
	
	// activate CAN controller
	CANGCON = (1 << ENASTB);
	
	return true;
}

// ----------------------------------------------------------------------------
// The CANPAGE register have to be restored after usage, otherwise it
// could cause trouble in the application programm.

ISR(CANIT_vect)
{
	uint8_t canpage;
	uint8_t mob;
	
	if ((CANHPMOB & 0xF0) != 0xF0)
	{
		// save MOb page register
		canpage = CANPAGE;
		
		// select MOb page with the highest priority
		CANPAGE = CANHPMOB & 0xF0;
		mob = (CANHPMOB >> 4);
		
		// a interrupt is only generated if a message was transmitted or received
		if (CANSTMOB & (1 << TXOK))
		{
			// clear MOb
			CANSTMOB &= 0;
			CANCDMOB = 0;
			
			#if CAN_TX_BUFFER_SIZE > 0
			can_t *buf = can_buffer_get_dequeue_ptr(&can_tx_buffer);
			
			// check if there are any another messages waiting 
			if (buf != NULL)
			{
				at90can_copy_message_to_mob( buf );
				can_buffer_dequeue(&can_tx_buffer);
				
				// enable transmission
				CANCDMOB |= (1<<CONMOB0);
			}
			else {
				// buffer underflow => no more messages to send
				_disable_mob_interrupt(mob);
				_transmission_in_progress = 0;
			}
			#else
			_free_buffer++;
			
			// reset interrupt
			if (mob < 8)
				CANIE2 &= ~(1 << mob);
			else
				CANIE1 &= ~(1 << (mob - 8));
			#endif
			
			CAN_INDICATE_TX_TRAFFIC_FUNCTION;
		}
		else {
			// a message was received successfully
			#if CAN_RX_BUFFER_SIZE > 0
			can_t *buf = can_buffer_get_enqueue_ptr(&can_rx_buffer);
			
			if (buf != NULL)
			{
				// read message
				at90can_copy_mob_to_message( buf );
				
				// push it to the list
				can_buffer_enqueue(&can_rx_buffer);
			}
			else {
				// buffer overflow => reject message
				// FIXME inform the user
			}
			
			// clear flags
			CANSTMOB &= 0;
			CANCDMOB = (1 << CONMOB1) | (CANCDMOB & (1 << IDE));
			#else
			_messages_waiting++;
			
			// reset interrupt
			if (mob < 8)
				CANIE2 &= ~(1 << mob);
			else
				CANIE1 &= ~(1 << (mob - 8));
			#endif
			
			CAN_INDICATE_RX_TRAFFIC_FUNCTION;
		}
		
		// restore MOb page register
		CANPAGE = canpage;
	}
	else
	{
		// no MOb matches with the interrupt => general interrupt
		CANGIT |= 0;
	}
}

// ----------------------------------------------------------------------------
// Overflow of CAN timer
ISR(OVRIT_vect) {}

#endif	// SUPPORT_FOR_AT90CAN__
