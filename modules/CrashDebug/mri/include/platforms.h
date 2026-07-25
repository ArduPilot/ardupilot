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
/* Declaration of routines that need to be provided for a specific target hardware platform before mri can be used to
   as a debug conduit for it. */
#ifndef _PLATFORMS_H_
#define _PLATFORMS_H_

#include <stdint.h>
#include "token.h"
#include "buffer.h"
#include "try_catch.h"

void      __mriPlatform_Init(Token* pParameterTokens);
char*     __mriPlatform_GetPacketBuffer(void);
uint32_t  __mriPlatform_GetPacketBufferSize(void);
void      __mriPlatform_EnteringDebugger(void);
void      __mriPlatform_LeavingDebugger(void);

uint32_t  __mriPlatform_MemRead32(const void* pv);
uint16_t  __mriPlatform_MemRead16(const void* pv);
uint8_t   __mriPlatform_MemRead8(const void* pv);
void      __mriPlatform_MemWrite32(void* pv, uint32_t value);
void      __mriPlatform_MemWrite16(void* pv, uint16_t value);
void      __mriPlatform_MemWrite8(void* pv, uint8_t value);

uint32_t  __mriPlatform_CommHasReceiveData(void);
int       __mriPlatform_CommReceiveChar(void);
void      __mriPlatform_CommSendChar(int character);
int       __mriPlatform_CommCausedInterrupt(void);
void      __mriPlatform_CommClearInterrupt(void);
int       __mriPlatform_CommShouldWaitForGdbConnect(void);
int       __mriPlatform_CommSharingWithApplication(void);
void      __mriPlatform_CommPrepareToWaitForGdbConnection(void);
int       __mriPlatform_CommIsWaitingForGdbToConnect(void);
void      __mriPlatform_CommWaitForReceiveDataToStop(void);
int       __mriPlatform_CommUartIndex(void);

uint8_t   __mriPlatform_DetermineCauseOfException(void);
void      __mriPlatform_DisplayFaultCauseToGdbConsole(void);
void      __mriPlatform_EnableSingleStep(void);
void      __mriPlatform_DisableSingleStep(void);
int       __mriPlatform_IsSingleStepping(void);
uint32_t  __mriPlatform_GetProgramCounter(void);
void      __mriPlatform_SetProgramCounter(uint32_t newPC);
void      __mriPlatform_AdvanceProgramCounterToNextInstruction(void);
int       __mriPlatform_WasProgramCounterModifiedByUser(void);
int       __mriPlatform_WasMemoryFaultEncountered(void);

void      __mriPlatform_WriteTResponseRegistersToBuffer(Buffer* pBuffer);
void      __mriPlatform_CopyContextToBuffer(Buffer* pBuffer);
void      __mriPlatform_CopyContextFromBuffer(Buffer* pBuffer);

uint32_t     __mriPlatform_GetDeviceMemoryMapXmlSize(void);
const char*  __mriPlatform_GetDeviceMemoryMapXml(void);
uint32_t     __mriPlatform_GetTargetXmlSize(void);
const char*  __mriPlatform_GetTargetXml(void);

typedef enum
{
    MRI_PLATFORM_WRITE_WATCHPOINT = 0,
    MRI_PLATFORM_READ_WATCHPOINT,
    MRI_PLATFORM_READWRITE_WATCHPOINT
}  PlatformWatchpointType;

__throws void  __mriPlatform_SetHardwareBreakpoint(uint32_t address, uint32_t kind);
__throws void  __mriPlatform_ClearHardwareBreakpoint(uint32_t address, uint32_t kind);
__throws void  __mriPlatform_SetHardwareWatchpoint(uint32_t address, uint32_t size,  PlatformWatchpointType type);
__throws void  __mriPlatform_ClearHardwareWatchpoint(uint32_t address, uint32_t size,  PlatformWatchpointType type);

typedef enum
{
    MRI_PLATFORM_INSTRUCTION_OTHER = 0,
    MRI_PLATFORM_INSTRUCTION_MBED_SEMIHOST_CALL,
    MRI_PLATFORM_INSTRUCTION_NEWLIB_SEMIHOST_CALL,
    MRI_PLATFORM_INSTRUCTION_HARDCODED_BREAKPOINT,
}  PlatformInstructionType;

typedef struct
{
    uint32_t    parameter1;
    uint32_t    parameter2;
    uint32_t    parameter3;
    uint32_t    parameter4;
}  PlatformSemihostParameters;

PlatformInstructionType     __mriPlatform_TypeOfCurrentInstruction(void);
PlatformSemihostParameters  __mriPlatform_GetSemihostCallParameters(void);
void                        __mriPlatform_SetSemihostCallReturnAndErrnoValues(int returnValue, int err);

const uint8_t* __mriPlatform_GetUid(void);
uint32_t       __mriPlatform_GetUidSize(void);

void __mriPlatform_SetSymbolAddress(Buffer* pBuffer, uint32_t address);
int __mriPlatform_SetSymbolRequest(Buffer* pBuffer);
uint32_t __mriPlatform_RtosGetFirstThreadId(void);
uint32_t __mriPlatform_RtosGetNextThreadId(void);
const char* __mriPlatform_RtosGetExtraThreadInfo(uint32_t thread_id);
int __mriPlatform_RtosGetThreadContext(Buffer* pBuffer, uint32_t thread_id);

/* Macroes which allow code to drop the __mri namespace prefix. */
#define Platform_Init                                       __mriPlatform_Init
#define Platform_GetPacketBuffer                            __mriPlatform_GetPacketBuffer
#define Platform_GetPacketBufferSize                        __mriPlatform_GetPacketBufferSize
#define Platform_EnteringDebugger                           __mriPlatform_EnteringDebugger
#define Platform_LeavingDebugger                            __mriPlatform_LeavingDebugger
#define Platform_MemRead32                                  __mriPlatform_MemRead32
#define Platform_MemRead16                                  __mriPlatform_MemRead16
#define Platform_MemRead8                                   __mriPlatform_MemRead8
#define Platform_MemWrite32                                 __mriPlatform_MemWrite32
#define Platform_MemWrite16                                 __mriPlatform_MemWrite16
#define Platform_MemWrite8                                  __mriPlatform_MemWrite8
#define Platform_CommHasReceiveData                         __mriPlatform_CommHasReceiveData
#define Platform_CommReceiveChar                            __mriPlatform_CommReceiveChar
#define Platform_CommSendChar                               __mriPlatform_CommSendChar
#define Platform_CommCausedInterrupt                        __mriPlatform_CommCausedInterrupt
#define Platform_CommClearInterrupt                         __mriPlatform_CommClearInterrupt
#define Platform_CommShouldWaitForGdbConnect                __mriPlatform_CommShouldWaitForGdbConnect
#define Platform_CommSharingWithApplication                 __mriPlatform_CommSharingWithApplication
#define Platform_CommPrepareToWaitForGdbConnection          __mriPlatform_CommPrepareToWaitForGdbConnection
#define Platform_CommIsWaitingForGdbToConnect               __mriPlatform_CommIsWaitingForGdbToConnect
#define Platform_CommWaitForReceiveDataToStop               __mriPlatform_CommWaitForReceiveDataToStop
#define Platform_CommUartIndex                              __mriPlatform_CommUartIndex
#define Platform_DetermineCauseOfException                  __mriPlatform_DetermineCauseOfException
#define Platform_DisplayFaultCauseToGdbConsole              __mriPlatform_DisplayFaultCauseToGdbConsole
#define Platform_EnableSingleStep                           __mriPlatform_EnableSingleStep
#define Platform_DisableSingleStep                          __mriPlatform_DisableSingleStep
#define Platform_IsSingleStepping                           __mriPlatform_IsSingleStepping
#define Platform_GetProgramCounter                          __mriPlatform_GetProgramCounter
#define Platform_SetProgramCounter                          __mriPlatform_SetProgramCounter
#define Platform_AdvanceProgramCounterToNextInstruction     __mriPlatform_AdvanceProgramCounterToNextInstruction
#define Platform_WasProgramCounterModifiedByUser            __mriPlatform_WasProgramCounterModifiedByUser
#define Platform_WasMemoryFaultEncountered                  __mriPlatform_WasMemoryFaultEncountered
#define Platform_WriteTResponseRegistersToBuffer            __mriPlatform_WriteTResponseRegistersToBuffer
#define Platform_CopyContextToBuffer                        __mriPlatform_CopyContextToBuffer
#define Platform_CopyContextFromBuffer                      __mriPlatform_CopyContextFromBuffer
#define Platform_GetDeviceMemoryMapXmlSize                  __mriPlatform_GetDeviceMemoryMapXmlSize
#define Platform_GetTargetXmlSize                           __mriPlatform_GetTargetXmlSize
#define Platform_GetTargetXml                               __mriPlatform_GetTargetXml
#define Platform_GetDeviceMemoryMapXml                      __mriPlatform_GetDeviceMemoryMapXml
#define Platform_SetHardwareBreakpoint                      __mriPlatform_SetHardwareBreakpoint
#define Platform_ClearHardwareBreakpoint                    __mriPlatform_ClearHardwareBreakpoint
#define Platform_SetHardwareWatchpoint                      __mriPlatform_SetHardwareWatchpoint
#define Platform_ClearHardwareWatchpoint                    __mriPlatform_ClearHardwareWatchpoint
#define Platform_TypeOfCurrentInstruction                   __mriPlatform_TypeOfCurrentInstruction
#define Platform_GetSemihostCallParameters                  __mriPlatform_GetSemihostCallParameters
#define Platform_SetSemihostCallReturnAndErrnoValues        __mriPlatform_SetSemihostCallReturnAndErrnoValues
#define Platform_GetUid                                     __mriPlatform_GetUid
#define Platform_GetUidSize                                 __mriPlatform_GetUidSize
#define Platform_SetSymbolAddress                           __mriPlatform_SetSymbolAddress
#define Platform_SetSymbolRequest                           __mriPlatform_SetSymbolRequest
#define Platform_RtosGetFirstThreadId                       __mriPlatform_RtosGetFirstThreadId
#define Platform_RtosGetNextThreadId                        __mriPlatform_RtosGetNextThreadId
#define Platform_RtosGetExtraThreadInfo                     __mriPlatform_RtosGetExtraThreadInfo
#define Platform_RtosGetThreadContext                       __mriPlatform_RtosGetThreadContext

#endif /* _PLATFORMS_H_ */
