/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#include <Console.h>
#include <mockConsole.h>


static int         g_hasStdInDataToReadException = noException;
static int         g_hasStdInDataToReadReturn = 0;
static int         g_readStdInException = noException;
static const char* g_pReadStdInCurr = NULL;
static const char* g_pReadStdInEnd = NULL;
static int         g_writeStdOutException = noException;
static char*       g_pWriteStdOutStart = NULL;
static char*       g_pWriteStdOutCurr = NULL;
static char*       g_pWriteStdOutEnd = NULL;


void ConsoleMock_Uninit(void)
{
    g_hasStdInDataToReadException = noException;
    g_hasStdInDataToReadReturn = 0;
    g_readStdInException = noException;
    g_pReadStdInCurr = g_pReadStdInEnd = NULL;
    g_writeStdOutException = noException;
    free(g_pWriteStdOutStart);
    g_pWriteStdOutStart = g_pWriteStdOutCurr = g_pWriteStdOutEnd = NULL;
}


void ConsoleMock_HasStdInDataToRead_SetException(int exceptionToThrow)
{
    g_hasStdInDataToReadException = exceptionToThrow;
}


void ConsoleMock_HasStdInDataToRead_SetReturn(int returnValue)
{
    g_hasStdInDataToReadReturn = returnValue;
}


void ConsoleMock_ReadStdIn_SetException(int exceptionToThrow)
{
    g_readStdInException = exceptionToThrow;
}


void ConsoleMock_ReadStdIn_SetBuffer(const char* pBuffer, size_t bufferSize)
{
    g_pReadStdInCurr = pBuffer;
    g_pReadStdInEnd = pBuffer + bufferSize;
}

void ConsoleMock_WriteStdOut_SetException(int exceptionToThrow)
{
    g_writeStdOutException = exceptionToThrow;
}

void ConsoleMock_WriteStdOut_SetCaptureBufferSize(size_t bufferSize)
{
    g_pWriteStdOutStart = malloc(bufferSize + 1);
    g_pWriteStdOutCurr = g_pWriteStdOutStart;
    g_pWriteStdOutEnd = g_pWriteStdOutStart + bufferSize;
}

const char* ConsoleMock_WriteStdOut_GetCapturedText(void)
{
    *g_pWriteStdOutCurr = '\0';
    return g_pWriteStdOutStart;
}



__throws int  Console_HasStdInDataToRead(void)
{
    if (g_hasStdInDataToReadException)
        __throw(g_hasStdInDataToReadException);
    return g_hasStdInDataToReadReturn;
}

__throws int Console_ReadStdIn(void)
{
    if (g_readStdInException)
        __throw(g_readStdInException);
    if (g_pReadStdInCurr < g_pReadStdInEnd)
        return (int)*(g_pReadStdInCurr++);
    else
        return 0;
}

__throws void Console_WriteStdOut(int character)
{
    if (g_writeStdOutException)
        __throw(g_writeStdOutException);
    if (g_pWriteStdOutCurr < g_pWriteStdOutEnd)
        *g_pWriteStdOutCurr++ = (char)character;
}
