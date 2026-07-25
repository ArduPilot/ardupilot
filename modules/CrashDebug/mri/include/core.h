/* Copyright 2014 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Core mri functionality exposed to other modules within the debug monitor.  These are the private routines exposed
   from within mri.c.  The public functionality is exposed via mri.h. */
#ifndef _CORE_H_
#define _CORE_H_

#include <stdint.h>
#include "buffer.h"

/* Real name of functions are in __mri namespace. */
void    __mriCore_InitBuffer(void);
Buffer* __mriCore_GetBuffer(void);
Buffer* __mriCore_GetInitializedBuffer(void);
void    __mriCore_PrepareStringResponse(const char* pErrorString);
#define PrepareEmptyResponseForUnknownCommand() __mriCore_PrepareStringResponse("")

int     __mriCore_WasControlCFlagSentFromGdb(void);
void    __mriCore_RecordControlCFlagSentFromGdb(int controlCFlag);
int     __mriCore_WasSemihostCallCancelledByGdb(void);
void    __mriCore_FlagSemihostCallAsHandled(void);
int     __mriCore_IsFirstException(void);
int     __mriCore_WasSuccessfullyInit(void);
int     __mriCore_IsWaitingForGdbToConnect(void);

void    __mriCore_SetSignalValue(uint8_t signalValue);
uint8_t __mriCore_GetSignalValue(void);
void    __mriCore_SetSemihostReturnValues(int semihostReturnCode, int semihostErrNo);
int     __mriCore_GetSemihostReturnCode(void);
int     __mriCore_GetSemihostErrno(void);

void    __mriCore_SendPacketToGdb(void);
void    __mriCore_GdbCommandHandlingLoop(void);

/* Macroes which allow code to drop the __mri namespace prefix. */
#define InitBuffer                      __mriCore_InitBuffer
#define GetBuffer                       __mriCore_GetBuffer
#define GetInitializedBuffer            __mriCore_GetInitializedBuffer
#define PrepareStringResponse           __mriCore_PrepareStringResponse
#define WasControlCFlagSentFromGdb      __mriCore_WasControlCFlagSentFromGdb
#define RecordControlCFlagSentFromGdb   __mriCore_RecordControlCFlagSentFromGdb
#define WasSemihostCallCancelledByGdb   __mriCore_WasSemihostCallCancelledByGdb
#define FlagSemihostCallAsHandled       __mriCore_FlagSemihostCallAsHandled
#define IsFirstException                __mriCore_IsFirstException
#define WasSuccessfullyInit             __mriCore_WasSuccessfullyInit
#define IsWaitingForGdbToConnect        __mriCore_IsWaitingForGdbToConnect
#define SetSignalValue                  __mriCore_SetSignalValue
#define GetSignalValue                  __mriCore_GetSignalValue
#define SetSemihostReturnValues         __mriCore_SetSemihostReturnValues
#define GetSemihostReturnCode           __mriCore_GetSemihostReturnCode
#define GetSemihostErrno                __mriCore_GetSemihostErrno
#define SendPacketToGdb                 __mriCore_SendPacketToGdb
#define GdbCommandHandlingLoop          __mriCore_GdbCommandHandlingLoop

/* Macro to convert 32-bit addresses sent from GDB to pointer. */
#if _LP64
    /* When unit testing on 64-bit, address will be from stack so grab upper 32-bit from stack address. */
    /* NOTE: This is for unit testing only.  It would never work on real 64-bit systems. */
    #define ADDR32_TO_POINTER(X) (void*)((size_t)(X) | ((size_t)(&pBuffer) & 0xFFFFFFFF00000000ULL))
#else
    #define ADDR32_TO_POINTER(X) (void*)(X)
#endif /* _LP64 */

#endif /* _CORE_H_ */
