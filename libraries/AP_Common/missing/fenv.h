#pragma once

#include_next <fenv.h>

#ifndef HAVE_FEENABLEEXCEPT
#if defined(__APPLE__) && defined(__MACH__)
#if defined __i386__ || defined __x86_64__

// Public domain polyfill for feenableexcept on OS X
// http://www-personal.umich.edu/~williams/archive/computation/fe-handling-example.c

inline int feenableexcept(unsigned int excepts)
{
    static fenv_t fenv;
    unsigned int new_excepts = excepts & FE_ALL_EXCEPT;
    // previous masks
    unsigned int old_excepts;

    if (fegetenv(&fenv)) {
        return -1;
    }
    old_excepts = fenv.__control & FE_ALL_EXCEPT;

    // unmask
    fenv.__control &= ~new_excepts;
    fenv.__mxcsr   &= ~(new_excepts << 7);

    return fesetenv(&fenv) ? -1 : old_excepts;
}

inline int fedisableexcept(unsigned int excepts)
{
    static fenv_t fenv;
    unsigned int new_excepts = excepts & FE_ALL_EXCEPT;
    // all previous masks
    unsigned int old_excepts;

    if (fegetenv(&fenv)) {
        return -1;
    }
    old_excepts = fenv.__control & FE_ALL_EXCEPT;

    // mask
    fenv.__control |= new_excepts;
    fenv.__mxcsr   |= new_excepts << 7;

    return fesetenv(&fenv) ? -1 : old_excepts;
}

#elif defined __arm64__

/*
  Yoinked and adapted this ARM64 implementation of floating point exceptions from
  https://android.googlesource.com/platform/bionic/+/a147a1d/libm/arm64/fenv.c
 */

/*-
 * Copyright (c) 2004 David Schultz <das@FreeBSD.ORG>
 * All rights reserved.
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
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: libm/aarch64/fenv.c $
 */


#define FPCR_EXCEPT_SHIFT 8

// FPCR, Floating-point Control Register.
#define __get_fpcr(__fpcr) __asm__ __volatile__("mrs %0,fpcr" : "=r" (__fpcr))
#define __set_fpcr(__fpcr) __asm__ __volatile__("msr fpcr,%0" : :"ri" (__fpcr))

inline int feenableexcept(int mask) {
    unsigned long long old_fpcr, new_fpcr;
    __get_fpcr(old_fpcr);
    new_fpcr = old_fpcr | ((mask & FE_ALL_EXCEPT) << FPCR_EXCEPT_SHIFT);
    if (new_fpcr != old_fpcr) {
        __set_fpcr(new_fpcr);
    }
    return ((old_fpcr >> FPCR_EXCEPT_SHIFT) & FE_ALL_EXCEPT);
}

inline int fedisableexcept(int mask) {
    unsigned long long old_fpcr, new_fpcr;
    __get_fpcr(old_fpcr);
    new_fpcr = old_fpcr & ~((mask & FE_ALL_EXCEPT) << FPCR_EXCEPT_SHIFT);
    if (new_fpcr != old_fpcr) {
        __set_fpcr(new_fpcr);
    }
    return ((old_fpcr >> FPCR_EXCEPT_SHIFT) & FE_ALL_EXCEPT);
}

#endif

#else
inline int feenableexcept(unsigned int excepts)
{
    #pragma STDC FENV_ACCESS ON
    fexcept_t flags;
    /* Save current exception flags. */
    fegetexceptflag(&flags, FE_ALL_EXCEPT);

    feclearexcept(FE_ALL_EXCEPT);   /* clear all fp exception conditions */
    return fesetexceptflag(&flags, excepts) != 0 ? -1 : flags; /* set new flags */

}

inline int fedisableexcept(unsigned int excepts)
{
    #pragma STDC FENV_ACCESS ON
    fexcept_t flags;
    /* Save current exception flags. */
    fegetexceptflag(&flags, FE_ALL_EXCEPT);

    feclearexcept(FE_ALL_EXCEPT);   /* clear all fp exception conditions */
    return fesetexceptflag(&flags, ~excepts) != 0 ? -1 : flags; /* set new flags */
}

#endif
#endif
