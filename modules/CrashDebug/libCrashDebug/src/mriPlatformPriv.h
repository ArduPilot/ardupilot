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
#ifndef _MRI_PLATFORM_PRIV_H_
#define _MRI_PLATFORM_PRIV_H_
#include <buffer.h>

void mriPlatform_setWaitForGdbConnect(int setting);
int hasFPURegisters(void);
void writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount);
void* throwingZeroedMalloc(size_t size);
#endif /* _MRI_PLATFORM_PRIV_H_ */
