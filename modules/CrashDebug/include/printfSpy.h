/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Module for spying on printf output from code under test. */
#ifndef _PRINTF_SPY_H_
#define _PRINTF_SPY_H_

#include <stdio.h>


/* Pointer to *printf routines which can intercepted by this module. */
extern int (*hook_printf)(const char* pFormat, ...);
extern int (*hook_fprintf)(FILE* pFile, const char* pFormat, ...);

void        printfSpy_Hook(size_t BufferSize);
void        printfSpy_Unhook(void);

const char* printfSpy_GetLastOutput(void);
const char* printfSpy_GetPreviousOutput(void);
const char* printfSpy_GetNthOutput(size_t n);
const char* printfSpy_GetLastErrorOutput(void);
const char* printfSpy_GetPreviousErrorOutput(void);
const char* printfSpy_GetNthErrorOutput(size_t n);
size_t      printfSpy_GetCallCount(void);


#undef  printf
#define printf hook_printf

#undef fprintf
#define fprintf hook_fprintf


#endif /* _PRINTF_SPY_H_ */
