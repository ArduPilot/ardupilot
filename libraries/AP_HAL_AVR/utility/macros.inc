/* Copyright (c) 2002, 2005, 2006, 2007 Marek Michalkiewicz
   Copyright (c) 2006 Dmitry Xmelkov
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

#include <AP_HAL_Boards.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2

/*
   macros.inc - macros for use in assembler sources

   Contributors:
     Created by Marek Michalkiewicz <marekm@linux.org.pl>
 */

#include <avr/io.h>
//#include "sectionname.h"

/* if not defined, assume old version with underscores */
#ifndef __USER_LABEL_PREFIX__
#define __USER_LABEL_PREFIX__ _
#endif

#ifndef __REGISTER_PREFIX__
#define __REGISTER_PREFIX__
#endif

/* the assembler line separator (just in case it ever changes) */
#define _L $

#define CONCAT1(a, b) CONCAT2(a, b)
#define CONCAT2(a, b) a ## b

#define _U(x) CONCAT1(__USER_LABEL_PREFIX__, x)

#define _R(x) CONCAT1(__REGISTER_PREFIX__, x)

/* these should help to fix the "can't have function named r1()" bug
   which may require adding '%' in front of register names.  */

#define r0 _R(r0)
#define r1 _R(r1)
#define r2 _R(r2)
#define r3 _R(r3)
#define r4 _R(r4)
#define r5 _R(r5)
#define r6 _R(r6)
#define r7 _R(r7)
#define r8 _R(r8)
#define r9 _R(r9)
#define r10 _R(r10)
#define r11 _R(r11)
#define r12 _R(r12)
#define r13 _R(r13)
#define r14 _R(r14)
#define r15 _R(r15)
#define r16 _R(r16)
#define r17 _R(r17)
#define r18 _R(r18)
#define r19 _R(r19)
#define r20 _R(r20)
#define r21 _R(r21)
#define r22 _R(r22)
#define r23 _R(r23)
#define r24 _R(r24)
#define r25 _R(r25)
#define r26 _R(r26)
#define r27 _R(r27)
#define r28 _R(r28)
#define r29 _R(r29)
#define r30 _R(r30)
#define r31 _R(r31)

#ifndef __tmp_reg__
#define __tmp_reg__ r0
#endif

#ifndef __zero_reg__
#define __zero_reg__ r1
#endif

#if __AVR_MEGA__
  #define XJMP jmp
  #define XCALL call
#else
  #define XJMP rjmp
  #define XCALL rcall
#endif

/* used only by fplib/strtod.S - libgcc internal function calls */
#define PROLOGUE_SAVES(offset) XJMP (__prologue_saves__ + 2 * (offset))
#define EPILOGUE_RESTORES(offset) XJMP (__epilogue_restores__ + 2 * (offset))

#if FLASHEND > 0x10000  /* ATmega103 */
  #define BIG_CODE 1
#else
  #define BIG_CODE 0
#endif

#ifndef __AVR_HAVE_MOVW__
#  if  defined(__AVR_ENHANCED__) && __AVR_ENHANCED__
#   define __AVR_HAVE_MOVW__ 1
#  endif
#endif

#ifndef __AVR_HAVE_LPMX__
# if  defined(__AVR_ENHANCED__) && __AVR_ENHANCED__
#  define __AVR_HAVE_LPMX__ 1
# endif
#endif

#ifndef __AVR_HAVE_MUL__
# if  defined(__AVR_ENHANCED__) && __AVR_ENHANCED__
#  define __AVR_HAVE_MUL__ 1
# endif
#endif

/*
   Smart version of movw:
    - uses "movw" if possible (supported by MCU, and both registers even)
    - handles overlapping register pairs correctly
    - no instruction generated if source and destination are the same
   (may expand to 0, 1 or 2 instructions).
 */

.macro  X_movw dst src
	.L_movw_dst = -1
	.L_movw_src = -1
	.L_movw_n = 0
	.irp  reg,	r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, \
			r10,r11,r12,r13,r14,r15,r16,r17,r18,r19, \
			r20,r21,r22,r23,r24,r25,r26,r27,r28,r29, \
			r30,r31
		.ifc  \reg,\dst
			.L_movw_dst = .L_movw_n
		.endif
		.ifc  \reg,\src
			.L_movw_src = .L_movw_n
		.endif
		.L_movw_n = .L_movw_n + 1
	.endr
	.L_movw_n = 0
	.irp  reg,	R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, \
			R10,R11,R12,R13,R14,R15,R16,R17,R18,R19, \
			R20,R21,R22,R23,R24,R25,R26,R27,R28,R29, \
			R30,R31
		.ifc  \reg,\dst
			.L_movw_dst = .L_movw_n
		.endif
		.ifc  \reg,\src
			.L_movw_src = .L_movw_n
		.endif
		.L_movw_n = .L_movw_n + 1
	.endr
	.if   .L_movw_dst < 0
		.L_movw_n = 0
		.rept   32
			.if \dst == .L_movw_n
				.L_movw_dst = .L_movw_n
			.endif
			.L_movw_n = .L_movw_n + 1
		.endr
	.endif
	.if   .L_movw_src < 0
		.L_movw_n = 0
		.rept   32
			.if \src == .L_movw_n
				.L_movw_src = .L_movw_n
			.endif
			.L_movw_n = .L_movw_n + 1
		.endr
	.endif
	.if   (.L_movw_dst < 0) || (.L_movw_src < 0)
		.err    ; Invalid 'X_movw' arg.
	.endif
                
	.if ((.L_movw_src) - (.L_movw_dst))  /* different registers */
		.if (((.L_movw_src) | (.L_movw_dst)) & 0x01)
			.if (((.L_movw_src)-(.L_movw_dst)) & 0x80) /* src < dest */
				mov     (.L_movw_dst)+1, (.L_movw_src)+1
				mov     (.L_movw_dst), (.L_movw_src)
			.else                                      /* src > dest */
				mov     (.L_movw_dst), (.L_movw_src)
				mov     (.L_movw_dst)+1, (.L_movw_src)+1
			.endif
		.else  /* both even -> overlap not possible */
#if  defined(__AVR_HAVE_MOVW__) && __AVR_HAVE_MOVW__
			movw    \dst, \src
#else
			mov     (.L_movw_dst), (.L_movw_src)
			mov     (.L_movw_dst)+1, (.L_movw_src)+1
#endif
		.endif
	.endif
.endm

/* Macro 'X_lpm' extends enhanced lpm instruction for classic chips.
   Usage:
	X_lpm	reg, dst
   where
	reg	is 0..31, r0..r31 or R0..R31
	dst	is z, Z, z+ or Z+
   It is possible to omit both arguments.

   Possible results for classic chips:
	lpm
	lpm / mov Rd,r0
	lpm / adiw ZL,1
	lpm / mov Rd,r0 / adiw ZL,1
	
   For enhanced chips it is one instruction always.

   ATTENTION:  unlike enhanced chips SREG (S,V,N,Z,C) flags are
   changed in case of 'Z+' dst.  R0 is scratch.
 */
.macro	X_lpm	dst=r0, src=Z

  /* dst evaluation	*/
  .L_lpm_dst = -1

  .L_lpm_n = 0
  .irp  reg,  r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, \
	     r10,r11,r12,r13,r14,r15,r16,r17,r18,r19, \
	     r20,r21,r22,r23,r24,r25,r26,r27,r28,r29, \
	     r30,r31
    .ifc  \reg,\dst
      .L_lpm_dst = .L_lpm_n
    .endif
    .L_lpm_n = .L_lpm_n + 1
  .endr

  .L_lpm_n = 0
  .irp  reg,  R0, R1, R2, R3, R4, R5, R6, R7, R8, R9, \
	     R10,R11,R12,R13,R14,R15,R16,R17,R18,R19, \
	     R20,R21,R22,R23,R24,R25,R26,R27,R28,R29, \
	     R30,R31
    .ifc  \reg,\dst
      .L_lpm_dst = .L_lpm_n
    .endif
    .L_lpm_n = .L_lpm_n + 1
  .endr

  .if  .L_lpm_dst < 0
    .L_lpm_n = 0
    .rept 32
      .if  \dst == .L_lpm_n
	.L_lpm_dst = .L_lpm_n
      .endif
      .L_lpm_n = .L_lpm_n + 1
    .endr
  .endif

  .if  (.L_lpm_dst < 0)
    .err	; Invalid dst arg of 'X_lpm' macro.
  .endif

  /* src evaluation	*/    
  .L_lpm_src = -1
  .L_lpm_n = 0
  .irp  reg,  z,Z,z+,Z+
    .ifc  \reg,\src
      .L_lpm_src = .L_lpm_n
    .endif
    .L_lpm_n = .L_lpm_n + 1
  .endr

  .if  (.L_lpm_src < 0)
    .err	; Invalid src arg of 'X_lpm' macro.
  .endif

  /* instruction(s)	*/    
  .if  .L_lpm_src < 2
    .if  .L_lpm_dst == 0
	lpm
    .else
#if  defined(__AVR_HAVE_LPMX__) && __AVR_HAVE_LPMX__
	lpm	.L_lpm_dst, Z
#else
	lpm
	mov	.L_lpm_dst, r0
#endif
    .endif
  .else
    .if  (.L_lpm_dst >= 30)
      .err	; Registers 30 and 31 are inhibited as 'X_lpm *,Z+' dst.
    .endif
#if  defined(__AVR_HAVE_LPMX__) && __AVR_HAVE_LPMX__
	lpm	.L_lpm_dst, Z+
#else
	lpm
    .if  .L_lpm_dst
	mov	.L_lpm_dst, r0
    .endif
	adiw	r30, 1
#endif
  .endif
.endm

/*
   LPM_R0_ZPLUS_INIT is used before the loop to initialize RAMPZ
   for future devices with RAMPZ:Z auto-increment - [e]lpm r0, Z+.

   LPM_R0_ZPLUS_NEXT is used inside the loop to load a byte from
   the program memory at [RAMPZ:]Z to R0, and increment [RAMPZ:]Z.

   The argument in both macros is a register that contains the
   high byte (bits 23-16) of the address, bits 15-0 should be in
   the Z (r31:r30) register.  It can be any register except for:
   r0, r1 (__zero_reg__ - assumed to always contain 0), r30, r31.
 */

	.macro	LPM_R0_ZPLUS_INIT hhi
#if __AVR_ENHANCED__
  #if BIG_CODE
	out	AVR_RAMPZ_ADDR, \hhi
  #endif
#endif
	.endm

	.macro	LPM_R0_ZPLUS_NEXT hhi
#if __AVR_ENHANCED__
  #if BIG_CODE
    /* ELPM with RAMPZ:Z post-increment, load RAMPZ only once */
	elpm	r0, Z+
  #else
    /* LPM with Z post-increment, max 64K, no RAMPZ (ATmega83/161/163/32) */
	lpm	r0, Z+
  #endif
#else
  #if BIG_CODE
    /* ELPM without post-increment, load RAMPZ each time (ATmega103) */
	out	AVR_RAMPZ_ADDR, \hhi
	elpm
	adiw	r30,1
	adc	\hhi, __zero_reg__
  #else
    /* LPM without post-increment, max 64K, no RAMPZ (AT90S*) */
	lpm
	adiw	r30,1
  #endif
#endif
	.endm
#endif
