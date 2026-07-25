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
 * $Id: can_private.h 8086 2009-07-14 14:08:25Z fabian $
 */
// ----------------------------------------------------------------------------

#ifndef	CAN_PRIVATE_H
#define	CAN_PRIVATE_H

#include "can.h"

#ifndef	CAN_FORCE_TX_ORDER
	#define	CAN_FORCE_TX_ORDER		0
#endif
	
// settings for buffered operation (only possible for the AT90CAN...)
#if !defined(CAN_TX_BUFFER_SIZE) || CAN_TX_BUFFER_SIZE == 0
	#define	CAN_TX_BUFFER_SIZE		0
	
	// forced order is only possible with buffered transmission
	#undef	CAN_FORCE_TX_ORDER
	#define	CAN_FORCE_TX_ORDER		0
#endif

#ifndef	CAN_RX_BUFFER_SIZE
	#define	CAN_RX_BUFFER_SIZE		0
#endif


#if defined(SUPPORT_MCP2515) && (SUPPORT_MCP2515 == 1)
	#define	BUILD_FOR_MCP2515	1
#else
	#define BUILD_FOR_MCP2515	0
#endif

#if defined(SUPPORT_AT90CAN) && (SUPPORT_AT90CAN == 1)
	#define	BUILD_FOR_AT90CAN	1
#else
	#define BUILD_FOR_AT90CAN	0
#endif

#if defined(SUPPORT_SJA1000) && (SUPPORT_SJA1000 == 1)
	#define	BUILD_FOR_SJA1000	1
#else
	#define BUILD_FOR_SJA1000	0
#endif

#if ((BUILD_FOR_MCP2515 + BUILD_FOR_AT90CAN + BUILD_FOR_SJA1000) <= 1)
	#if (BUILD_FOR_MCP2515 == 1)

		#define mcp2515_init(...)					can_init(__VA_ARGS__)
		#define mcp2515_sleep(...)					can_sleep(__VA_ARGS__)
		#define mcp2515_wakeup(...)					can_wakeup(__VA_ARGS__)
		#define mcp2515_check_free_buffer(...)		can_check_free_buffer(__VA_ARGS__)
		#define mcp2515_check_message(...)			can_check_message(__VA_ARGS__)
		#define mcp2515_get_filter(...)				can_get_filter(__VA_ARGS__)
		#define mcp2515_static_filter(...)			can_static_filter(__VA_ARGS__)
		#define mcp2515_set_filter(...)				can_set_filter(__VA_ARGS__)
		#define mcp2515_get_message(...)			can_get_message(__VA_ARGS__)
		#define mcp2515_send_message(...)			can_send_message(__VA_ARGS__)
		#define	mcp2515_read_error_register(...)	can_read_error_register(__VA_ARGS__)
		#define	mcp2515_set_mode(...)				can_set_mode(__VA_ARGS__)

	#elif (BUILD_FOR_AT90CAN == 1)

		#define at90can_init(...)					can_init(__VA_ARGS__)
		#define at90can_check_free_buffer(...)		can_check_free_buffer(__VA_ARGS__)
		#define at90can_check_message(...)			can_check_message(__VA_ARGS__)
		#define at90can_get_filter(...)				can_get_filter(__VA_ARGS__)
		#define at90can_set_filter(...)				can_set_filter(__VA_ARGS__)
		#define at90can_disable_filter(...)			can_disable_filter(__VA_ARGS__)
		
		#if CAN_RX_BUFFER_SIZE == 0
			#define at90can_get_message(...)			can_get_message(__VA_ARGS__)
		#else
			#define	at90can_get_buffered_message(...)	can_get_message(__VA_ARGS__)
		#endif
		
		#if CAN_TX_BUFFER_SIZE == 0
			#define at90can_send_message(...)			can_send_message(__VA_ARGS__)
		#else
			#define	at90can_send_buffered_message(...)	can_send_message(__VA_ARGS__)
		#endif
		
		#define	at90can_read_error_register(...)	can_read_error_register(__VA_ARGS__)
		#define	at90can_set_mode(...)				can_set_mode(__VA_ARGS__)

	#elif (BUILD_FOR_SJA1000 == 1)

		#define	sja1000_init(...)					can_init(__VA_ARGS__)
		#define sja1000_check_free_buffer(...)		can_check_free_buffer(__VA_ARGS__)
		#define sja1000_check_message(...)			can_check_message(__VA_ARGS__)
		#define sja1000_disable_filter(...)			can_disable_filter(__VA_ARGS__)
		#define sja1000_get_message(...)			can_get_message(__VA_ARGS__)
		#define sja1000_send_message(...)			can_send_message(__VA_ARGS__)
		#define	sja1000_read_error_register(...)	can_read_error_register(__VA_ARGS__)
		#define	sja1000_check_bus_off(...)			can_check_bus_off(__VA_ARGS__)
		#define	sja1000_reset_bus_off(...)			can_reset_bus_off(__VA_ARGS__)
		#define	sja1000_set_mode(...)				can_set_mode(__VA_ARGS__)

	#else

		#error	No CAN-interface specified!

	#endif
#endif

#ifndef	CAN_INDICATE_TX_TRAFFIC_FUNCTION
	#define	CAN_INDICATE_TX_TRAFFIC_FUNCTION
#endif

#ifndef	CAN_INDICATE_RX_TRAFFIC_FUNCTION
	#define	CAN_INDICATE_RX_TRAFFIC_FUNCTION
#endif

#ifdef	CAN_DEBUG_LEVEL
	#include <avr/pgmspace.h>
	#include <stdio.h>
	
	#define	DEBUG_INFO(format, ...)		printf_P(PSTR(format), ##__VA_ARGS__)
#else
	#define	DEBUG_INFO(format, ...)
#endif


#endif	// CAN_PRIVATE_H
