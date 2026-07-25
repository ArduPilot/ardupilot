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
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define OUTPUT_COUNT    4

char*   g_stdoutBuffers[OUTPUT_COUNT];
char*   g_stderrBuffers[OUTPUT_COUNT];
size_t  g_bufferSize = 0;
size_t  g_callCount = 0;
size_t  g_bufferIndex = 0;
int (*hook_printf)(const char* pFormat, ...) = printf;
int (*hook_fprintf)(FILE* pFile, const char* pFormat, ...) = fprintf;

void printfSpy_Unhook(void);

static void _AllocateAndInitBuffers(size_t BufferSize)
{
    g_bufferSize = BufferSize + 1;
    g_bufferIndex = 0;
}

static void _FreeBuffer(void)
{
    size_t i = 0;
    for (i = 0 ; i < OUTPUT_COUNT ; i++)
    {
        free(g_stdoutBuffers[i]);
        free(g_stderrBuffers[i]);
        g_stdoutBuffers[i] = NULL;
        g_stderrBuffers[i] = NULL;
    }
    g_bufferSize = 0;
    g_bufferIndex = 0;
}

static size_t _nextIndex(void)
{
    return (g_bufferIndex + 1) & (OUTPUT_COUNT - 1);
}

static size_t _prevIndex(size_t n)
{
    return (g_bufferIndex - n) & (OUTPUT_COUNT - 1);
}

static int mock_common(FILE* pFile, const char* pFormat, va_list valist)
{
    int     WrittenSize = -1;
    char**  ppBuffers = NULL;
    char*   pBuffer = NULL;

    assert (pFile == stdout || pFile == stderr);
    if (pFile == stdout)
    {
        ppBuffers = g_stdoutBuffers;
    }
    else
    {
        ppBuffers = g_stderrBuffers;
    }

    if (ppBuffers[g_bufferIndex] == NULL)
    {
        ppBuffers[g_bufferIndex] = malloc(g_bufferSize);
        assert(ppBuffers[g_bufferIndex]);
    }
    pBuffer = ppBuffers[g_bufferIndex];

    WrittenSize = vsnprintf(pBuffer,
                            g_bufferSize,
                            pFormat,
                            valist);
    g_bufferIndex = _nextIndex();
    g_callCount++;
    return WrittenSize;
}

static int mock_printf(const char* pFormat, ...)
{
    va_list valist;
    va_start(valist, pFormat);
    return mock_common(stdout, pFormat, valist);
}

static int mock_fprintf(FILE* pFile, const char* pFormat, ...)
{
    va_list valist;
    va_start(valist, pFormat);
    return mock_common(pFile, pFormat, valist);
}

static void setHookFunctionPointers(void)
{
    hook_printf = mock_printf;
    hook_fprintf = mock_fprintf;
}

static void restoreHookFunctionPointers(void)
{
    hook_printf = printf;
    hook_fprintf = fprintf;
}


/********************/
/* Public routines. */
/********************/
void printfSpy_Hook(size_t BufferSize)
{
    printfSpy_Unhook();

    _AllocateAndInitBuffers(BufferSize);
    g_callCount = 0;
    setHookFunctionPointers();
}

void printfSpy_Unhook(void)
{
    restoreHookFunctionPointers();
    _FreeBuffer();
}

const char* printfSpy_GetNthOutput(size_t n)
{
    assert(n <= OUTPUT_COUNT);
    return g_stdoutBuffers[_prevIndex(n)];
}

const char* printfSpy_GetLastOutput(void)
{
    return printfSpy_GetNthOutput(1);
}

const char* printfSpy_GetPreviousOutput(void)
{
    return printfSpy_GetNthOutput(2);
}

const char* printfSpy_GetNthErrorOutput(size_t n)
{
    assert(n <= OUTPUT_COUNT);
    return g_stderrBuffers[_prevIndex(n)];
}

const char* printfSpy_GetLastErrorOutput(void)
{
    return printfSpy_GetNthErrorOutput(1);
}

const char* printfSpy_GetPreviousErrorOutput(void)
{
    return printfSpy_GetNthErrorOutput(2);
}

size_t printfSpy_GetCallCount(void)
{
    return g_callCount;
}
