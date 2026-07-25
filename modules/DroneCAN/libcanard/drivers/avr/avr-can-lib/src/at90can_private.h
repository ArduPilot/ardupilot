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
 * $Id: at90can_private.h 6910 2008-11-30 21:13:14Z fabian $
 */
// ----------------------------------------------------------------------------

#ifndef	AT90CAN_PRIVATE_H
#define	AT90CAN_PRIVATE_H

// ----------------------------------------------------------------------------

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdlib.h>

#include "can_private.h"
#include "can.h"
#include "can_buffer.h"
#include "utils.h"

// ----------------------------------------------------------------------------

#if (defined (__AVR_AT90CAN32__) || \
	 defined (__AVR_AT90CAN64__) || \
	 defined (__AVR_AT90CAN128__)) && \
	 BUILD_FOR_AT90CAN == 1

#if F_CPU != 16000000UL
	#error	only 16 MHz for F_CPU supported!
#endif

#define	SUPPORT_FOR_AT90CAN__		1

// ----------------------------------------------------------------------------

#if CAN_RX_BUFFER_SIZE > 0
extern can_buffer_t can_rx_buffer;
#else
// Indicates the number of MObs which have currently one received
// message stored in them.
extern volatile uint8_t _messages_waiting;
#endif

#if CAN_TX_BUFFER_SIZE > 0
extern can_buffer_t can_tx_buffer;
#else
extern volatile uint8_t _free_buffer;
#endif

#if CAN_FORCE_TX_ORDER
extern volatile uint8_t _transmission_in_progress ;
#endif

// ----------------------------------------------------------------------------
extern uint8_t _find_free_mob(void);

// ----------------------------------------------------------------------------
extern void _disable_mob_interrupt(uint8_t mob);

// ----------------------------------------------------------------------------
extern void _enable_mob_interrupt(uint8_t mob);


// ----------------------------------------------------------------------------
extern uint8_t at90can_send_message(const can_t *msg);

// ----------------------------------------------------------------------------
extern uint8_t at90can_get_message(can_t *msg);

// ----------------------------------------------------------------------------
/**
 * Copy data form a message in RAM to the actual registers.
 *
 * \warning this function assumes CANPAGE to be set properly
 */
extern void at90can_copy_message_to_mob(const can_t *msg);

// ----------------------------------------------------------------------------
/**
 * Copy data form a message the registers to RAM.
 *
 * \return true  if the message should be retrieved,
 *         false if no valid message was available (only for
 *               SUPPORT_EXTENDED_CANID = 0)
 *
 * \warning this function assumes CANPAGE to be set properly
 */
extern bool at90can_copy_mob_to_message(can_t *msg);

// ----------------------------------------------------------------------------
// enter standby mode => messages are not transmitted nor received

extern __attribute__ ((gnu_inline)) inline void _enter_standby_mode(void)
{
	// request abort
	CANGCON = (1 << ABRQ);
	
	// wait until receiver is not busy
	while (CANGSTA & (1 << RXBSY))
		;
	
	// request standby mode
	CANGCON = 0;
	
	// wait until the CAN Controller has entered standby mode
	while (CANGSTA & (1 << ENFG))
		;
}

// ----------------------------------------------------------------------------
// leave standby mode => CAN Controller is connected to CAN Bus

extern __attribute__ ((gnu_inline)) inline void _leave_standby_mode(void)
{
	// save CANPAGE register
	uint8_t canpage = CANPAGE;
	
	// reenable all MObs
	for (uint8_t i=0;i<15;i++) {
		CANPAGE = i << 4;
		CANCDMOB = CANCDMOB;
	}
	
	// restore CANPAGE
	CANPAGE = canpage;
	
	// request normal mode
	CANGCON = (1 << ENASTB);
	
	// wait until the CAN Controller has left standby mode
	while ((CANGSTA & (1 << ENFG)) == 0)
		;
}

#endif

// ----------------------------------------------------------------------------

#endif	// AT90CAN_PRIVATE_H
