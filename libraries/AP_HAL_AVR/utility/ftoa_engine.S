/* Copyright (c) 2005, Dmitry Xmelkov
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

/* $Id: ftoa_engine.S,v 1.3 2009/04/01 23:11:00 arcanum Exp $ */

#ifndef	__DOXYGEN__

#include "macros.inc"
#include "ftoa_engine.h"

#if  defined(__AVR_HAVE_LPMX__) && __AVR_HAVE_LPMX__
#  define AVR_ENH_LPM	1
#else
#  define AVR_ENH_LPM	0
#endif

/*
   int __ftoa_engine (double val, char *buf,
                      unsigned char prec, unsigned char maxdgs)
 Input:
    val    - value to convert
    buf    - output buffer address
    prec   - precision: number of decimal digits is 'prec + 1'
    maxdgs - (0 if unused) precision restriction for "%f" specification

 Output:
    return     - decimal exponent of first digit
    buf[0]     - flags (FTOA_***)
    buf[1],... - decimal digits
    Number of digits:
	maxdgs == 0 ? prec+1 :
	(buf[0] & FTOA_CARRY) == 0 || buf[1] != '1' ?
	    aver(1, maxdgs+exp, prec+1) :
	    aver(1, masdgs+exp-1, prec+1)

 Notes:
    * Output string is not 0-terminated. For possibility of user's buffer
    usage in any case.
    * If used, 'maxdgs' is a number of digits for value with zero exponent.
*/

    /* Input */
#define maxdgs	r16
#define	prec	r18
#define	buf_lo	r20
#define	buf_hi	r21
#define	val_lo	r22
#define	val_hi	r23
#define	val_hlo	r24
#define	val_hhi	r25

    /* Float value parse	*/
#define	flag	r19

    /* Multiplication of mantisses	*/
#define	exp_sv	r17
#define	mlt_1	r19	/* lowest result byte	*/
#define mlt_2	r14
#define	mlt_3	r15
#define	mlt_4	r20
#define	mlt_5	r21
#define	mlt_6	r28
#define	mlt_7	r29

    /* Conversion to string	*/
#define	pwr_2	r1	/* lowest byte of 'powr10' element	*/
#define	pwr_3	r17
#define	pwr_4	r19
#define	pwr_5	r22
#define	pwr_6	r25
#define	pwr_7	r0
#define	digit	r23
#define	exp10	r24

    /* Fixed */
#define	zero	r1

/*    ASSEMBLY_CLIB_SECTION */
	
    .global	__ftoa_engine
    .type	__ftoa_engine, "function"
__ftoa_engine:

/* --------------------------------------------------------------------
   Float value parse.
*/
  ; limit 'prec'
	cpi	prec, 8
	brlo	1f
	ldi	prec, 7
1:
  ; init.
	clr	flag
	X_movw	XL, buf_lo
  ; val_hhi := exponent, sign test and remove
#if  FTOA_MINUS != 1
#  error  FTOA_MINUS must be 1:  add with carry used
#endif
	lsl	val_hhi
	adc	flag, zero		; FTOA_MINUS
	sbrc	val_hlo, 7
	ori	val_hhi, 1
  ; zero test
	adiw	val_hlo, 0
	cpc	val_lo, zero
	cpc	val_hi, zero
	brne	3f
  ; return 0
	ori	flag, FTOA_ZERO
	subi	prec, -2
2:	st	X+, flag
	ldi	flag, '0'
	dec	prec
	brne	2b
	ret				; r24,r25 == 0
3:
  ; infinity, NaN ?
#if  FTOA_NAN != 2 * FTOA_INF
#  error  Must: FTOA_NAN == 2*FTOA_INF: 'rjmp' is absent
#endif
	cpi	val_hhi, 0xff
	brlo	6f
	cpi	val_hlo, 0x80
	cpc	val_hi, zero
	cpc	val_lo, zero
	breq	5f
	subi	flag, -FTOA_INF		; FTOA_NAN
5:	subi	flag, -FTOA_INF
6:
  ; write flags byte
	st	X+, flag
  ; hidden bit
	cpi	val_hhi, 1
	brlo	7f			; if subnormal value
	ori	val_hlo, 0x80
7:	adc	val_hhi, zero
  ; pushes
	push	r29
	push	r28
	push	r17
	push	r16
	push	r15
	push	r14

/* --------------------------------------------------------------------
   Multiplication of mantisses (val and table).
   At the begin:
	val_hlo .. val_lo  - input value mantisse
	val_hhi            - input value exponent
	X                  - second byte address (string begin)
   At the end:
	mlt_7 .. mlt_2     - multiplication result
	exp10              - decimal exponent
*/

  ; save
	mov	exp_sv, val_hhi
  ; Z := & base10[exp / 8]	(sizeof(base10[0]) == 5)
	andi	val_hhi, ~7
	lsr	val_hhi			; (exp/8) * 4
	mov	ZL, val_hhi
	lsr	val_hhi
	lsr	val_hhi			; exp/8
	add	ZL, val_hhi		; (exp/8) * 5
	clr	ZH
	subi	ZL, lo8(-(.L_base10))
	sbci	ZH, hi8(-(.L_base10))
  ; highest mantissa byte  (mult. shifting prepare)
	clr	val_hhi
  ; result initializ.
	clr	mlt_1
	clr	mlt_2
	clr	mlt_3
	X_movw	mlt_4, mlt_2
	X_movw	mlt_6, mlt_2

  ; multiply to 1-st table byte
#if  AVR_ENH_LPM
	lpm	r0, Z+
#else
	lpm
	adiw	ZL, 1
#endif
	sec			; for loop end control
	ror	r0
  ; addition
10:	brcc	11f
	add	mlt_1, val_lo
	adc	mlt_2, val_hi
	adc	mlt_3, val_hlo
	adc	mlt_4, val_hhi
	adc	mlt_5, zero
  ; arg shift
11:	lsl	val_lo
	rol	val_hi
	rol	val_hlo
	rol	val_hhi
  ; next bit
	lsr	r0
	brne	10b

  ; second table byte
#if  AVR_ENH_LPM
	lpm	r0, Z+		; C flag is stay 1
#else
	lpm
	adiw	ZL, 1
	sec
#endif
	ror	r0
  ; addition
12:	brcc	13f
	add	mlt_2, val_hi		; val_hi is the least byte now
	adc	mlt_3, val_hlo
	adc	mlt_4, val_hhi
	adc	mlt_5, val_lo
	adc	mlt_6, zero
  ; arg shift
13:	lsl	val_hi
	rol	val_hlo
	rol	val_hhi
	rol	val_lo
  ; next bit
	lsr	r0
	brne	12b

  ; 3-t table byte
#if  AVR_ENH_LPM
	lpm	r0, Z+		; C flag is stay 1
#else
	lpm
	adiw	ZL, 1
	sec
#endif
	ror	r0
  ; addition
14:	brcc	15f
	add	mlt_3, val_hlo		; val_hlo is the least byte now
	adc	mlt_4, val_hhi
	adc	mlt_5, val_lo
	adc	mlt_6, val_hi
	adc	mlt_7, zero
  ; arg shift
15:	lsl	val_hlo
	rol	val_hhi
	rol	val_lo
	rol	val_hi
  ; next bit
	lsr	r0
	brne	14b

  ; 4-t table byte
#if  AVR_ENH_LPM
	lpm	r0, Z+		; C flag is stay 1
#else
	lpm
#endif
	ror	r0
  ; addition
16:	brcc	17f
	add	mlt_4, val_hhi		; val_hhi is the least byte now
	adc	mlt_5, val_lo
	adc	mlt_6, val_hi
	adc	mlt_7, val_hlo
  ; arg shift
17:	lsl	val_hhi
	rol	val_lo
	rol	val_hi
	rol	val_hlo
  ; next bit
	lsr	r0
	brne	16b

  ; decimal exponent
#if  AVR_ENH_LPM
	lpm	exp10, Z
#else
	adiw	ZL, 1
	lpm
	mov	exp10, r0
#endif

  ; result shift:  mlt_7..2 >>= (~exp & 7)
	com	exp_sv
	andi	exp_sv, 7
	breq	19f
18:	lsr	mlt_7
	ror	mlt_6
	ror	mlt_5
	ror	mlt_4
	ror	mlt_3
	ror	mlt_2
	dec	exp_sv
	brne	18b
19:

/* --------------------------------------------------------------------
   Conversion to string.

   Registers usage:
      mlt_7 .. mlt_2	- new mantissa (multiplication result)
      pwr_7 .. pwr_2	- 'powr10' table element
      Z			- 'powr10' table pointer
      X			- output string pointer
      maxdgs		- number of digits
      prec		- number of digits stays to output
      exp10		- decimal exponent
      digit		- conversion process

   At the end:
      X			- end of buffer (nonfilled byte)
      exp10		- corrected dec. exponent
      mlt_7 .. mlt_2	- remainder
      pwr_7 .. pwr_2	- last powr10[] element

   Notes:
     * It is possible to leave out powr10'x table with subnormal value.
      Result: accuracy degrease on the rounding phase.  No matter: high
      precision with subnormals is not needed. (Now 0x00000001 is converted
      exactly on prec = 5, i.e. 6 digits.)
*/

  ; to find first digit
	ldi	ZL, lo8(.L_powr10)
	ldi	ZH, hi8(.L_powr10)
	set
  ; 'pwr10' element reading
.L_digit:
	X_lpm	pwr_2, Z+
	X_lpm	pwr_3, Z+
	X_lpm	pwr_4, Z+
	X_lpm	pwr_5, Z+
	X_lpm	pwr_6, Z+
	X_lpm	pwr_7, Z+
  ; 'digit' init.
	ldi	digit, '0' - 1
  ; subtraction loop
20:	inc	digit
	sub	mlt_2, pwr_2
	sbc	mlt_3, pwr_3
	sbc	mlt_4, pwr_4
	sbc	mlt_5, pwr_5
	sbc	mlt_6, pwr_6
	sbc	mlt_7, pwr_7
	brsh	20b
  ; restore mult
	add	mlt_2, pwr_2
	adc	mlt_3, pwr_3
	adc	mlt_4, pwr_4
	adc	mlt_5, pwr_5
	adc	mlt_6, pwr_6
	adc	mlt_7, pwr_7
  ; analisys
	brtc	25f
	cpi	digit, '0'
	brne	21f		; this is the first digit finded
	dec	exp10
	rjmp	.L_digit
  ; now is the first digit
21:	clt
  ; number of digits
	subi	maxdgs, 1
	brlo	23f			; maxdgs was 0
	add	maxdgs, exp10
	brpl	22f
	clr	maxdgs
22:	cp	maxdgs, prec
	brsh	23f
	mov	prec, maxdgs
23:	inc	prec
	mov	maxdgs, prec	
  ; operate digit
25:	cpi	digit, '0' + 10
	brlo	27f
  ; overflow, digit > '9'
	ldi	digit, '9'
26:	st	X+, digit
	dec	prec
	brne	26b
	rjmp	.L_up
  ; write digit
27:	st	X+, digit
	dec	prec
	brne	.L_digit

/* --------------------------------------------------------------------
    Rounding.
*/
.L_round:
  ; pwr10 /= 2
	lsr	pwr_7
	ror	pwr_6
	ror	pwr_5
	ror	pwr_4
	ror	pwr_3
	ror	pwr_2
  ; mult -= pwr10  (half of last 'pwr10' value)
	sub	mlt_2, pwr_2
	sbc	mlt_3, pwr_3
	sbc	mlt_4, pwr_4
	sbc	mlt_5, pwr_5
	sbc	mlt_6, pwr_6
	sbc	mlt_7, pwr_7
  ; rounding direction?
	brlo	.L_rest
  ; round to up
.L_up:
	inc	prec
	ld	digit, -X
	inc	digit
	cpi	digit, '9' + 1
	brlo	31f
	ldi	digit, '0'
31:	st	X, digit
	cpse	prec, maxdgs
	brsh	.L_up
  ; it was a carry to master digit
	ld	digit, -X		; flags
	ori	digit, FTOA_CARRY	; 'C' is not changed
	st	X+, digit
	brlo	.L_rest			; above comparison
  ; overflow
	inc	exp10
	ldi	digit, '1'
32:	st	X+, digit
	ldi	digit, '0'
	dec	prec
	brne	32b
  ; restore
.L_rest:
	clr	zero
	pop	r14
	pop	r15
	pop	r16
	pop	r17
	pop	r28
	pop	r29
  ; return
	clr	r25
	sbrc	exp10, 7		; high byte
	com	r25
	ret

    .size  __ftoa_engine, . - __ftoa_engine

/* --------------------------------------------------------------------
    Tables.  '.L_powr10' is placed first -- for subnormals stability.
*/
    .section .progmem.data,"a",@progbits

    .type .L_powr10, "object"
.L_powr10:
	.byte	0, 64, 122, 16, 243, 90	; 100000000000000
	.byte	0, 160, 114, 78, 24, 9	; 10000000000000
	.byte	0, 16, 165, 212, 232, 0	; 1000000000000
	.byte	0, 232, 118, 72, 23, 0	; 100000000000
	.byte	0, 228, 11, 84, 2, 0	; 10000000000
	.byte	0, 202, 154, 59, 0, 0	; 1000000000
	.byte	0, 225, 245, 5, 0, 0	; 100000000
	.byte	128, 150, 152, 0, 0, 0	; 10000000
	.byte	64, 66, 15, 0, 0, 0	; 1000000
	.byte	160, 134, 1, 0, 0, 0	; 100000
	.byte	16, 39, 0, 0, 0, 0	; 10000
	.byte	232, 3, 0, 0, 0, 0	; 1000
	.byte	100, 0, 0, 0, 0, 0	; 100
	.byte	10, 0, 0, 0, 0, 0	; 10
	.byte	1, 0, 0, 0, 0, 0	; 1
    .size .L_powr10, . - .L_powr10

    .type	.L_base10, "object"
.L_base10:
	.byte	44, 118, 216, 136, -36	; 2295887404
	.byte	103, 79, 8, 35, -33	; 587747175
	.byte	193, 223, 174, 89, -31	; 1504632769
	.byte	177, 183, 150, 229, -29	; 3851859889
	.byte	228, 83, 198, 58, -26	; 986076132
	.byte	81, 153, 118, 150, -24	; 2524354897
	.byte	230, 194, 132, 38, -21	; 646234854
	.byte	137, 140, 155, 98, -19	; 1654361225
	.byte	64, 124, 111, 252, -17	; 4235164736
	.byte	188, 156, 159, 64, -14	; 1084202172
	.byte	186, 165, 111, 165, -12	; 2775557562
	.byte	144, 5, 90, 42, -9	; 710542736
	.byte	92, 147, 107, 108, -7	; 1818989404
	.byte	103, 109, 193, 27, -4	; 465661287
	.byte	224, 228, 13, 71, -2	; 1192092896
	.byte	245, 32, 230, 181, 0	; 3051757813
	.byte	208, 237, 144, 46, 3	; 781250000
	.byte	0, 148, 53, 119, 5	; 2000000000
	.byte	0, 128, 132, 30, 8	; 512000000
	.byte	0, 0, 32, 78, 10	; 1310720000
	.byte	0, 0, 0, 200, 12	; 3355443200
	.byte	51, 51, 51, 51, 15	; 858993459
	.byte	152, 110, 18, 131, 17	; 2199023256
	.byte	65, 239, 141, 33, 20	; 562949953
	.byte	137, 59, 230, 85, 22	; 1441151881
	.byte	207, 254, 230, 219, 24	; 3689348815
	.byte	209, 132, 75, 56, 27	; 944473297
	.byte	247, 124, 29, 144, 29	; 2417851639
	.byte	164, 187, 228, 36, 32	; 618970020
	.byte	50, 132, 114, 94, 34	; 1584563250
	.byte	129, 0, 201, 241, 36	; 4056481921
	.byte	236, 161, 229, 61, 39	; 1038459372
    .size .L_base10, . - .L_base10

	.end
#endif	/* !__DOXYGEN__ */