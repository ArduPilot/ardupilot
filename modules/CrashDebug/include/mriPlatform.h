/*  Copyright (C) 2017  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef _MRI_PLATFORM_H_
#define _MRI_PLATFORM_H_

#include <IComm.h>
#include <IMemory.h>
#include <try_catch.h>

/* Register names / indices into the RegisterContext::R array of registers. */
#define R0              0
#define R1              1
#define R2              2
#define R3              3
#define R4              4
#define R5              5
#define R6              6
#define R7              7
#define R8              8
#define R9              9
#define R10             10
#define R11             11
#define R12             12
#define SP              13
#define LR              14
#define PC              15
#define XPSR            16
#define MSP             17
#define PSP             18
#define TOTAL_REG_COUNT (PSP + 1)

/* Register names /indices into the RegisterContext::FPR array of floating point registers. */
#define S0                  0
#define S1                  1
#define S2                  2
#define S3                  3
#define S4                  4
#define S5                  5
#define S6                  6
#define S7                  7
#define S8                  8
#define S9                  9
#define S10                 10
#define S11                 11
#define S12                 12
#define S13                 13
#define S14                 14
#define S15                 15
#define S16                 16
#define S17                 17
#define S18                 18
#define S19                 19
#define S20                 20
#define S21                 21
#define S22                 22
#define S23                 23
#define S24                 24
#define S25                 25
#define S26                 26
#define S27                 27
#define S28                 28
#define S29                 29
#define S30                 30
#define S31                 31
#define FPSCR               32
#define TOTAL_FPREG_COUNT   (FPSCR + 1)

typedef struct RegisterContext
{
    uint32_t flags;
    uint32_t R[TOTAL_REG_COUNT];
    uint32_t exceptionPSR;
    uint32_t FPR[TOTAL_FPREG_COUNT];
} RegisterContext;

/* Default value to be placed in MSP and PSP if crash dump (ie. version 2.0) doesn't contain specific values. */
#define DEFAULT_SP_VALUE 0xBAADBAAD


__throws void mriPlatform_Init(RegisterContext* pContext, IMemory* pMem);
         void mriPlatform_Run(IComm* pComm);

#endif /* _MRI_PLATFORM_H_ */
