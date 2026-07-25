/* Copyright 2017 Adam Green (http://mbed.org/users/AdamGreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* Routines to read/write memory and detect any faults that might occur while attempting to do so. */
#ifndef _MEMORY_H_
#define _MEMORY_H_

#include <stdint.h>
#include "buffer.h"

/* Real name of functions are in __mri namespace. */
uint32_t __mriMem_ReadMemoryIntoHexBuffer(Buffer* pBuffer, const void* pvMemory, uint32_t readByteCount);
int      __mriMem_WriteHexBufferToMemory(Buffer* pBuffer, void* pvMemory, uint32_t writeByteCount);
int      __mriMem_WriteBinaryBufferToMemory(Buffer* pBuffer, void* pvMemory, uint32_t writeByteCount);

/* Macroes which allow code to drop the __mri namespace prefix. */
#define ReadMemoryIntoHexBuffer     __mriMem_ReadMemoryIntoHexBuffer
#define WriteHexBufferToMemory      __mriMem_WriteHexBufferToMemory
#define WriteBinaryBufferToMemory   __mriMem_WriteBinaryBufferToMemory

#endif /* _MEMORY_H_ */
