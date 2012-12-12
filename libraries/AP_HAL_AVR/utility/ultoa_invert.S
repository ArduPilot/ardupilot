/* Copyright (c) 2005,2007  Dmitry Xmelkov
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

/* $Id: ultoa_invert.S 1944 2009-04-01 23:12:20Z arcanum $	*/

#ifndef	__DOXYGEN__
#include <AP_HAL_Boards.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2

#include "macros.inc"
#include "ntz.h"
#include "xtoa_fast.h"

/* --------------------------------------------------------------------
   char * __ultoa_invert (unsigned long val, char * str, int base)

   This function is intended for usage as internal printf's one.
   It differs from others of `xtoa_fast' family:
       * srt[] will NOT 0 terminated.
       * Sequence of digits is inverted.
       * It returns pointer to first byte after a string.
       * Only `XTOA_UPPER' flag is operated.
   Notes:
       * base: check only 8 and 16, all others are treated as 10.
       (internal printf's function).
*/

    /* Input	*/
#define v_lo	r22
#define	v_hi	r23
#define	v_hlo	r24
#define	v_hhi	r25
#define	str_lo	r20
#define	str_hi	r21
#define	base	r18
#define	flags	r19

    /* Used	*/
#define	v_fifth	r26	/* val: bits 39..32			*/
#define	t_lo	r18	/* temporary for shifted `val'		*/
#define	t_hi	r19
#define	t_hlo	r20
#define	t_hhi	r21
#define	symb	r20	/* write to string			*/
#define	cnt	r27	/* shift loop counter, local arg	*/

    /* Fixed	*/
#define	rzero	r1

/*	ASSEMBLY_CLIB_SECTION */
	.global	__ultoa_invert
	.type	__ultoa_invert, "function"

__ultoa_invert:
	X_movw	ZL, str_lo
	clr	v_fifth			; needed for all (ultoa_lsr)
	cpi	base, 8
	breq	.L_oct
	cpi	base, 16
	breq	.L_hex

  ; decimal format
	clt				; flag of val == 0
.L_dec_loop:
	push	v_lo			; to calculate remander
  ; val &= ~1
	andi	v_lo, ~1
  ; val += 2
	subi	v_lo, lo8(-2)
	sbci	v_hi, hi8(-2)
	sbci	v_hlo, hlo8(-2)
	sbci	v_hhi, hhi8(-2)
	sbci	v_fifth, hhi8(-2)
  ; val += val/2
	ldi	cnt, 1
	rcall	.L_div_add
  ; val += val/16
	ldi	cnt, 4
	rcall	.L_div_add
  ; val += val/256
	add	v_lo, v_hi
	adc	v_hi, v_hlo
	adc	v_hlo, v_hhi
	adc	v_hhi, v_fifth
	adc	v_fifth, rzero
  ; val += val/65536
	add	v_lo, v_hlo
	adc	v_hi, v_hhi
	adc	v_hlo, v_fifth
	adc	v_hhi, rzero
	adc	v_fifth, rzero
  ; val += val >> 32
	add	v_lo, v_fifth
	adc	v_hi, rzero
	adc	v_hlo, rzero
	adc	v_hhi, rzero
	adc	v_fifth, rzero
  ; division result:  val /= 16
	rcall	.L_lsr_4		; v_fitth := 0
	brne	1f
	set				; T := Z flag
1:
  ; rem:  val_original - 10*val
	pop	t_hi
#if  defined(__AVR_ENHANCED__) && __AVR_ENHANCED__
	ldi	t_lo, 10
	mul	t_lo, v_lo
	clr	r1
#else
	mov	r0, v_lo
	lsl	r0
	sub	t_hi, r0
	lsl	r0
	lsl	r0
#endif
	sub	t_hi, r0
  ; output digit
	subi	t_hi, lo8(-'0')
	st	Z+, t_hi
  ; quotient == 0 ?
	brtc	.L_dec_loop
  ; end of string
.L_eos:
	X_movw	r24, ZL
	ret

  ; octal format
.L_oct:
	mov	symb, v_lo
	andi	symb, 7
	subi	symb, lo8(-'0')
	st	Z+, symb
	ldi	cnt, 3
	rcall	.L_lsr
	brne	.L_oct
	rjmp	.L_eos

  ; hex format
.L_hex:
	mov	symb, v_lo
	andi	symb, 0x0f
	subi	symb, lo8(-'0')
	cpi	symb, '9' + 1
	brlo	3f
	subi	symb, lo8('9' + 1 - 'a')
	sbrc	flags, ntz(XTOA_UPPER) - 8
	subi	symb, lo8('a' - 'A')
3:	st	Z+, symb
	rcall	.L_lsr_4
	brne	.L_hex
	rjmp	.L_eos
	
.L_lsr_4:
	ldi	cnt, 4
.L_lsr:
	lsr	v_fifth
	ror	v_hhi
	ror	v_hlo
	ror	v_hi
	ror	v_lo
	dec	cnt
	brne	.L_lsr
  ; tst
	sbiw	v_hlo, 0		; only Z flag is needed
	cpc	v_lo, rzero
	cpc	v_hi, rzero
	ret

.L_div_add:
  ; copy to temporary
	X_movw	t_lo, v_lo
	X_movw	t_hlo, v_hlo
	mov	r0, v_fifth
  ; lsr temporary
7:	lsr	r0
	ror	t_hhi
	ror	t_hlo
	ror	t_hi
	ror	t_lo
	dec	cnt
	brne	7b
  ; add
	add	v_lo, t_lo
	adc	v_hi, t_hi
	adc	v_hlo, t_hlo
	adc	v_hhi, t_hhi
	adc	v_fifth, r0		; here r0 == 0
	ret

	.size	__ultoa_invert, . - __ultoa_invert
	.end

#endif	/* !__DOXYGEN__ */
#endif
	