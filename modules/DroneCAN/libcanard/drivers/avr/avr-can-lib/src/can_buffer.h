// -----------------------------------------------------------------------------
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
 * $Id: can_buffer.h 6837 2008-11-16 19:05:15Z fabian $
 */
// -----------------------------------------------------------------------------

#ifndef	CAN_BUFFER_H
#define	CAN_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "can.h"

// -----------------------------------------------------------------------------
typedef struct {
	can_t *buf;
	uint8_t size;
	
	uint8_t used;
	uint8_t head;
	uint8_t tail;
} can_buffer_t;

// -----------------------------------------------------------------------------
extern void can_buffer_init(can_buffer_t *buf, uint8_t size, can_t *list);

// -----------------------------------------------------------------------------
extern bool can_buffer_empty(can_buffer_t *buf);

// -----------------------------------------------------------------------------
extern bool can_buffer_full(can_buffer_t *buf);

// -----------------------------------------------------------------------------
extern can_t *can_buffer_get_enqueue_ptr(can_buffer_t *buf);

// -----------------------------------------------------------------------------
/**
 * 
 * \warning	
 */
extern void can_buffer_enqueue(can_buffer_t *buf);

// -----------------------------------------------------------------------------
extern can_t *can_buffer_get_dequeue_ptr(can_buffer_t *buf);

// -----------------------------------------------------------------------------
/**
 * 
 * \warning	
 */
extern void can_buffer_dequeue(can_buffer_t *buf);

#endif	// CAN_BUFFER_H
