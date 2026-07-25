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
/* Module for spying on printf output from code under test. */
#ifndef _MOCK_CONSOLE_H
#define _MOCK_CONSOLE_H


void ConsoleMock_Uninit(void);

void ConsoleMock_HasStdInDataToRead_SetException(int exceptionToThrow);
void ConsoleMock_HasStdInDataToRead_SetReturn(int returnValue);

void ConsoleMock_ReadStdIn_SetException(int exceptionToThrow);
void ConsoleMock_ReadStdIn_SetBuffer(const char* pBuffer, size_t bufferSize);

void ConsoleMock_WriteStdOut_SetException(int exceptionToThrow);
void ConsoleMock_WriteStdOut_SetCaptureBufferSize(size_t bufferSize);
const char* ConsoleMock_WriteStdOut_GetCapturedText(void);


#endif /* _MOCK_CONSOLE_H */
