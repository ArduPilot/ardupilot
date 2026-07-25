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
#ifndef _PLATFORM_MOCK_H_
#define _PLATFORM_MOCK_H_

#include <token.h>
#include <platforms.h>

#define INITIAL_PC 0x10000000

void        platformMock_Init(void);
void        platformMock_Uninit(void);

void        platformMock_CommInitReceiveData(const char* pDataToReceive1, const char* pDataToReceive2 = NULL);
void        platformMock_CommInitReceiveChecksummedData(const char* pDataToReceive1, const char* pDataToReceive2 = NULL);
void        platformMock_CommInitTransmitDataBuffer(size_t Size);
int         platformMock_CommDoesTransmittedDataEqual(const char* thisString);
void        platformMock_CommSetInterruptBit(int setValue);
void        platformMock_CommSetShouldWaitForGdbConnect(int setValue);
void        platformMock_CommSetIsWaitingForGdbToConnectIterations(int iterations);
int         platformMock_GetCommWaitForReceiveDataToStopCalls(void);
int         platformMock_GetCommPrepareToWaitForGdbConnectionCalls(void);
void        platformMock_SetCommSharingWithApplication(int setValue);

void        platformMock_SetInitException(int exceptionToThrow);
int         platformMock_GetInitCount(void);
Token*      platformMock_GetInitTokenCopy(void);

int         platformMock_GetEnteringDebuggerCalls(void);
int         platformMock_GetLeavingDebuggerCalls(void);

void        platformMock_SetIsDebuggeeMakingSemihostCall(int setValue);
int         platformMock_GetHandleSemihostRequestCalls(void);

int         platformMock_DisplayFaultCauseToGdbConsoleCalls(void);

void        platformMock_SetPacketBufferSize(uint32_t setValue);

void        platformMock_SetTypeOfCurrentInstruction(PlatformInstructionType setValue);
int         platformMock_AdvanceProgramCounterToNextInstructionCalls(void);
int         platformMock_SetProgramCounterCalls(void);
uint32_t    platformMock_GetProgramCounterValue(void);

void        platformMock_FaultOnSpecificMemoryCall(int callToFail);

uint32_t*   platformMock_GetContext(void);

int         platformMock_SetHardwareBreakpointCalls(void);
uint32_t    platformMock_SetHardwareBreakpointAddressArg(void);
uint32_t    platformMock_SetHardwareBreakpointKindArg(void);
void        platformMock_SetHardwareBreakpointException(uint32_t exceptionToThrow);

int         platformMock_ClearHardwareBreakpointCalls(void);
uint32_t    platformMock_ClearHardwareBreakpointAddressArg(void);
uint32_t    platformMock_ClearHardwareBreakpointKindArg(void);
void        platformMock_ClearHardwareBreakpointException(uint32_t exceptionToThrow);

int                    platformMock_SetHardwareWatchpointCalls(void);
uint32_t               platformMock_SetHardwareWatchpointAddressArg(void);
uint32_t               platformMock_SetHardwareWatchpointSizeArg(void);
PlatformWatchpointType platformMock_SetHardwareWatchpointTypeArg(void);
void                   platformMock_SetHardwareWatchpointException(uint32_t exceptionToThrow);

int                    platformMock_ClearHardwareWatchpointCalls(void);
uint32_t               platformMock_ClearHardwareWatchpointAddressArg(void);
uint32_t               platformMock_ClearHardwareWatchpointSizeArg(void);
PlatformWatchpointType platformMock_ClearHardwareWatchpointTypeArg(void);
void                   platformMock_ClearHardwareWatchpointException(uint32_t exceptionToThrow);

int platformMock_GetSemihostCallReturnValue(void);
int platformMock_GetSemihostCallErrno(void);

#endif /* _PLATFORM_MOCK_H_ */
