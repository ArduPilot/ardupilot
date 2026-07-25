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
#include <assert.h>
#include <signal.h>
#include <string.h>

extern "C"
{
#include "platforms.h"
#include "posix4win.h"
#include "semihost.h"
#include "buffer.h"
#include "try_catch.h"
#include "hex_convert.h"
#include "memory.h"
}
#include "platformMock.h"

#define TRUE 1
#define FALSE 0
#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))


// Forward Function Declarations.
static char*    allocateAndCopyChecksummedData(const char* pData);
static size_t   countPoundSigns(const char* p);
static void     copyChecksummedData(char* pDest, const char* pSrc);
static void     commUninitTransmitDataBuffer();
static uint32_t isReceiveBufferEmpty();
static void     waitForReceiveData();
static size_t   getTransmitDataBufferSize();



// Platform_Init Instrumentation
static int   g_initException;
static int   g_initCount;
static Token g_initTokenCopy;

void platformMock_SetInitException(int exceptionToThrow)
{
    g_initException = exceptionToThrow;
}

int platformMock_GetInitCount()
{
    return g_initCount;
}

Token* platformMock_GetInitTokenCopy()
{
    return &g_initTokenCopy;
}

// Platform_Init stub called by MRI core.
void Platform_Init(Token* pParameterTokens)
{
    g_initCount++;
    Token_Copy(&g_initTokenCopy, pParameterTokens);
    
    if (g_initException)
        __throw(g_initException);
}



// Platform_Comm* Instrumentation
static const char  g_emptyPacket[] = "$#00";
static Buffer      g_receiveBuffers[2];
static size_t      g_receiveIndex;
static char*       g_pAlloc1;
static char*       g_pAlloc2;
static char*       g_pTransmitDataBufferStart;
static char*       g_pTransmitDataBufferEnd;
static char*       g_pTransmitDataBufferCurr;
static int         g_commInterruptBit;
static int         g_commShouldWaitForGdbConnect;
static int         g_commIsWaitingForGdbToConnect;
static int         g_commWaitForReceiveDataToStopCount;
static int         g_commPrepareToWaitForGdbConnectionCount;
static int         g_commSharingWithApplication;

void platformMock_CommInitReceiveData(const char* pDataToReceive1, const char* pDataToReceive2 /*= NULL*/)
{
    Buffer_Init(&g_receiveBuffers[0], (char*)pDataToReceive1, strlen(pDataToReceive1));
    if (pDataToReceive2)
        Buffer_Init(&g_receiveBuffers[1], (char*)pDataToReceive2, strlen(pDataToReceive2));
    else
        Buffer_Init(&g_receiveBuffers[1], (char*)g_emptyPacket, strlen(g_emptyPacket));
    g_receiveIndex = 0;
}

void platformMock_CommInitReceiveChecksummedData(const char* pDataToReceive1, const char* pDataToReceive2 /*= NULL*/)
{
    g_pAlloc1 = allocateAndCopyChecksummedData(pDataToReceive1);
    Buffer_Init(&g_receiveBuffers[0], g_pAlloc1, strlen(g_pAlloc1));
    if (pDataToReceive2)
    {
        g_pAlloc2 = allocateAndCopyChecksummedData(pDataToReceive2);
        Buffer_Init(&g_receiveBuffers[1], g_pAlloc2, strlen(g_pAlloc2));
    }
    else
    {
        Buffer_Init(&g_receiveBuffers[1], (char*)g_emptyPacket, strlen(g_emptyPacket));
    }
    g_receiveIndex = 0;
}

static char* allocateAndCopyChecksummedData(const char* pData)
{
    size_t len = strlen(pData) + 2 * countPoundSigns(pData) + 1;
    char*  pAlloc = (char*) malloc(len);
    copyChecksummedData(pAlloc, pData);
    return pAlloc;
}

static size_t countPoundSigns(const char* p)
{
    size_t count = 0;
    while (*p)
    {
        if (*p++ == '#')
            count++;
    }
    return count;
}

static void copyChecksummedData(char* pDest, const char* pSrc)
{
    char checksum = 0;
    
    while (*pSrc)
    {
        char curr = *pSrc++;
        
        *pDest++ = curr;
        switch (curr)
        {
        case '$':
            checksum = 0;
            break;
        case '#':
            *pDest++ = NibbleToHexChar[EXTRACT_HI_NIBBLE(checksum)];
            *pDest++ = NibbleToHexChar[EXTRACT_LO_NIBBLE(checksum)];
            break;
        default:
            checksum += curr;
            break;
        }
    }
    *pDest++ = '\0';
}

void platformMock_CommInitTransmitDataBuffer(size_t Size)
{
    commUninitTransmitDataBuffer();
    g_pTransmitDataBufferStart = (char*)malloc(Size);
    g_pTransmitDataBufferCurr = g_pTransmitDataBufferStart;
    g_pTransmitDataBufferEnd = g_pTransmitDataBufferStart + Size;
}

static void commUninitTransmitDataBuffer()
{
    free(g_pTransmitDataBufferStart);
    g_pTransmitDataBufferStart = NULL;
    g_pTransmitDataBufferCurr = NULL;
    g_pTransmitDataBufferEnd = NULL;
}

int platformMock_CommDoesTransmittedDataEqual(const char* thisString)
{
    size_t stringLength = strlen(thisString);

    if (getTransmitDataBufferSize() != stringLength)
        return 0;
    return !memcmp(thisString, g_pTransmitDataBufferStart, stringLength);
}

static size_t getTransmitDataBufferSize()
{
    return g_pTransmitDataBufferCurr - g_pTransmitDataBufferStart;
}

void platformMock_CommSetInterruptBit(int setValue)
{
    g_commInterruptBit = setValue;
}

void platformMock_CommSetShouldWaitForGdbConnect(int setValue)
{
    g_commShouldWaitForGdbConnect = setValue;
}

void platformMock_CommSetIsWaitingForGdbToConnectIterations(int iterations)
{
    g_commIsWaitingForGdbToConnect = iterations;
}

int platformMock_GetCommWaitForReceiveDataToStopCalls(void)
{
    return g_commWaitForReceiveDataToStopCount;
}

int platformMock_GetCommPrepareToWaitForGdbConnectionCalls(void)
{
    return g_commPrepareToWaitForGdbConnectionCount;
}

void platformMock_SetCommSharingWithApplication(int setValue)
{
    g_commSharingWithApplication = setValue;
}

// Platform_Comm* stubs called by MRI core.
uint32_t Platform_CommHasReceiveData(void)
{
    if (isReceiveBufferEmpty())
    {
        if (g_receiveIndex < ARRAY_SIZE(g_receiveBuffers))
            g_receiveIndex++;
        return 0;
    }
    
    return 1;
}

static uint32_t isReceiveBufferEmpty()
{
    if (g_receiveIndex >= ARRAY_SIZE(g_receiveBuffers))
        return TRUE;
    return (uint32_t)(Buffer_BytesLeft(&g_receiveBuffers[g_receiveIndex]) == 0);
}

int Platform_CommReceiveChar(void)
{
    waitForReceiveData();

    int character = Buffer_ReadChar(&g_receiveBuffers[g_receiveIndex]);

    clearExceptionCode();

    return character;
}

static void waitForReceiveData()
{
    while (!Platform_CommHasReceiveData())
    {
    }
}

void Platform_CommSendChar(int character)
{
    if (g_pTransmitDataBufferCurr < g_pTransmitDataBufferEnd)
        *g_pTransmitDataBufferCurr++ = (char)character;
}

int __mriPlatform_CommCausedInterrupt(void)
{
    return g_commInterruptBit;
}

void __mriPlatform_CommClearInterrupt(void)
{
    g_commInterruptBit = FALSE;
}

int __mriPlatform_CommShouldWaitForGdbConnect(void)
{
    return g_commShouldWaitForGdbConnect;
}

int __mriPlatform_CommIsWaitingForGdbToConnect(void)
{
    int returnValue = g_commIsWaitingForGdbToConnect;
    if (returnValue)
        g_commIsWaitingForGdbToConnect--;
    return returnValue;
}

void __mriPlatform_CommWaitForReceiveDataToStop(void)
{
    g_commWaitForReceiveDataToStopCount++;
}

void __mriPlatform_CommPrepareToWaitForGdbConnection(void)
{
    g_commPrepareToWaitForGdbConnectionCount++;
}

int __mriPlatform_CommSharingWithApplication(void)
{
    return g_commSharingWithApplication;
}



// Instrumentation to entering and leaving of debugger.
static int g_enteringDebuggerCount;
static int g_leavingDebuggerCount;

int platformMock_GetEnteringDebuggerCalls(void)
{
    return g_enteringDebuggerCount;
}

int platformMock_GetLeavingDebuggerCalls(void)
{
    return g_leavingDebuggerCount;
}

// Stubs called by MRI core.
void __mriPlatform_EnteringDebugger(void)
{
    g_enteringDebuggerCount++;
}

void __mriPlatform_LeavingDebugger(void)
{
    g_leavingDebuggerCount++;
}



// Packet Buffer Instrumentation.
// NOTE: Packet must be big enough for g/G packets to hold 16 general purpose registers + PSR with 2 hex digits per
//       byte + 1 more byte for the g/G command character.
static char     g_packetBuffer[1 + (16 + 1) * (sizeof(uint32_t) * 2)];
static uint32_t g_packetBufferSize;

void        platformMock_SetPacketBufferSize(uint32_t setValue)
{
    assert ( setValue <= sizeof(g_packetBuffer) );
    g_packetBufferSize = setValue;
}

// Packet Buffer stubs called by MRI core.
char* __mriPlatform_GetPacketBuffer(void)
{
    return g_packetBuffer;
}

uint32_t  __mriPlatform_GetPacketBufferSize(void)
{
    return g_packetBufferSize;
}



// Semihost Instrumentation
static int g_isDebuggeeMakingSemihostCall;
static int g_getHandleSemihostRequestCount;

void platformMock_SetIsDebuggeeMakingSemihostCall(int setValue)
{
    g_isDebuggeeMakingSemihostCall = setValue;
}

int platformMock_GetHandleSemihostRequestCalls(void)
{
    return g_getHandleSemihostRequestCount;
}

// Semihost stubs called by MRI core.
int __mriSemihost_IsDebuggeeMakingSemihostCall(void)
{
    return g_isDebuggeeMakingSemihostCall;
}

int __mriSemihost_HandleSemihostRequest(void)
{
    g_getHandleSemihostRequestCount++;
    return TRUE;
}



// Fault/Exception Related Instrumentation
static int g_displayFaultCauseToGdbConsoleCount;

int platformMock_DisplayFaultCauseToGdbConsoleCalls(void)
{
    return g_displayFaultCauseToGdbConsoleCount;
}

// Fault/Semihost stubs called by MRI core.
void __mriPlatform_DisplayFaultCauseToGdbConsole(void)
{
    g_displayFaultCauseToGdbConsoleCount++;
}



// Current Instruction Related Instrumentation
PlatformInstructionType g_instructionType;
int                     g_advanceProgramCounterToNextInstruction;
int                     g_setProgramCounterCalls;
uint32_t                g_programCounter;

void platformMock_SetTypeOfCurrentInstruction(PlatformInstructionType setValue)
{
    g_instructionType = setValue;
}

int platformMock_AdvanceProgramCounterToNextInstructionCalls(void)
{
    return g_advanceProgramCounterToNextInstruction;
}

int platformMock_SetProgramCounterCalls(void)
{
    return g_setProgramCounterCalls;
}

uint32_t platformMock_GetProgramCounterValue(void)
{
    return g_programCounter;
}

// Stubs called by MRI core.
PlatformInstructionType __mriPlatform_TypeOfCurrentInstruction(void)
{
    return g_instructionType;
}

void __mriPlatform_AdvanceProgramCounterToNextInstruction(void)
{
    g_advanceProgramCounterToNextInstruction++;
    g_programCounter += 4;
}

void __mriPlatform_SetProgramCounter(uint32_t newPC)
{
    g_programCounter = newPC;
    g_setProgramCounterCalls++;
}

int __mriPlatform_WasProgramCounterModifiedByUser(void)
{
    return FALSE;
}



// Single Stepping stubs called by MRI core.
static int g_singleStepping;

void __mriPlatform_EnableSingleStep(void)
{
    g_singleStepping = TRUE;
}

int __mriPlatform_IsSingleStepping(void)
{
    return g_singleStepping;
}


// Memory Fault Test Instrumentation.
static int g_callToFail;

void platformMock_FaultOnSpecificMemoryCall(int callToFail)
{
    g_callToFail = callToFail;
}

// Stub called by MRI core.
int __mriPlatform_WasMemoryFaultEncountered(void)
{
    if (g_callToFail == 0)
        return FALSE;
    g_callToFail--;
    if (g_callToFail == 0)
        return TRUE;
    return FALSE;
}



// Context Related Instrumentation.
static uint32_t g_context[4];

uint32_t* platformMock_GetContext(void)
{
    return g_context;
}

// Stubs called from MRI core.
void __mriPlatform_CopyContextToBuffer(Buffer* pBuffer)
{
    ReadMemoryIntoHexBuffer(pBuffer, &g_context, sizeof(g_context));
}

void __mriPlatform_CopyContextFromBuffer(Buffer* pBuffer)
{
    WriteHexBufferToMemory(pBuffer, &g_context, sizeof(g_context));
}

void __mriPlatform_WriteTResponseRegistersToBuffer(Buffer* pBuffer)
{
    Buffer_WriteString(pBuffer, "responseT");
}



// Hardware Breakpoint / Watchpoint Instrumentation.
int                    g_setHardwareBreakpointCalls;
uint32_t               g_setHardwareBreakpointAddressArg;
uint32_t               g_setHardwareBreakpointKindArg;
uint32_t               g_setHardwareBreakpointException;
int                    g_clearHardwareBreakpointCalls;
uint32_t               g_clearHardwareBreakpointAddressArg;
uint32_t               g_clearHardwareBreakpointKindArg;
uint32_t               g_clearHardwareBreakpointException;
int                    g_setHardwareWatchpointCalls;
uint32_t               g_setHardwareWatchpointAddressArg;
uint32_t               g_setHardwareWatchpointSizeArg;
PlatformWatchpointType g_setHardwareWatchpointTypeArg;
uint32_t               g_setHardwareWatchpointException;
int                    g_clearHardwareWatchpointCalls;
uint32_t               g_clearHardwareWatchpointAddressArg;
uint32_t               g_clearHardwareWatchpointSizeArg;
PlatformWatchpointType g_clearHardwareWatchpointTypeArg;
uint32_t               g_clearHardwareWatchpointException;

int platformMock_SetHardwareBreakpointCalls(void)
{
    return g_setHardwareBreakpointCalls;
}

uint32_t platformMock_SetHardwareBreakpointAddressArg(void)
{
    return g_setHardwareBreakpointAddressArg;
}

uint32_t platformMock_SetHardwareBreakpointKindArg(void)
{
    return g_setHardwareBreakpointKindArg;
}

void platformMock_SetHardwareBreakpointException(uint32_t exceptionToThrow)
{
    g_setHardwareBreakpointException = exceptionToThrow;    
}

int platformMock_ClearHardwareBreakpointCalls(void)
{
    return g_clearHardwareBreakpointCalls;
}

uint32_t platformMock_ClearHardwareBreakpointAddressArg(void)
{
    return g_clearHardwareBreakpointAddressArg;
}

uint32_t platformMock_ClearHardwareBreakpointKindArg(void)
{
    return g_clearHardwareBreakpointKindArg;
}

void platformMock_ClearHardwareBreakpointException(uint32_t exceptionToThrow)
{
    g_clearHardwareBreakpointException = exceptionToThrow;    
}

int platformMock_SetHardwareWatchpointCalls(void)
{
    return g_setHardwareWatchpointCalls;
}

uint32_t platformMock_SetHardwareWatchpointAddressArg(void)
{
    return g_setHardwareWatchpointAddressArg;
}

uint32_t platformMock_SetHardwareWatchpointSizeArg(void)
{
    return g_setHardwareWatchpointSizeArg;
}

PlatformWatchpointType platformMock_SetHardwareWatchpointTypeArg(void)
{
    return g_setHardwareWatchpointTypeArg;
}

void platformMock_SetHardwareWatchpointException(uint32_t exceptionToThrow)
{
    g_setHardwareWatchpointException = exceptionToThrow;    
}

int platformMock_ClearHardwareWatchpointCalls(void)
{
    return g_clearHardwareWatchpointCalls;
}

uint32_t platformMock_ClearHardwareWatchpointAddressArg(void)
{
    return g_clearHardwareWatchpointAddressArg;
}

uint32_t platformMock_ClearHardwareWatchpointSizeArg(void)
{
    return g_clearHardwareWatchpointSizeArg;
}

PlatformWatchpointType platformMock_ClearHardwareWatchpointTypeArg(void)
{
    return g_clearHardwareWatchpointTypeArg;
}

void platformMock_ClearHardwareWatchpointException(uint32_t exceptionToThrow)
{
    g_clearHardwareWatchpointException = exceptionToThrow;    
}

// Stubs called from MRI core.
__throws void  __mriPlatform_SetHardwareBreakpoint(uint32_t address, uint32_t kind)
{
    g_setHardwareBreakpointCalls++;
    g_setHardwareBreakpointAddressArg = address;
    g_setHardwareBreakpointKindArg = kind;
    if (g_setHardwareBreakpointException)
        __throw(g_setHardwareBreakpointException);
}

__throws void  __mriPlatform_ClearHardwareBreakpoint(uint32_t address, uint32_t kind)
{
    g_clearHardwareBreakpointCalls++;
    g_clearHardwareBreakpointAddressArg = address;
    g_clearHardwareBreakpointKindArg = kind;
    if (g_clearHardwareBreakpointException)
        __throw(g_clearHardwareBreakpointException);
}

__throws void  __mriPlatform_SetHardwareWatchpoint(uint32_t address, uint32_t size,  PlatformWatchpointType type)
{
    g_setHardwareWatchpointCalls++;
    g_setHardwareWatchpointAddressArg = address;
    g_setHardwareWatchpointSizeArg = size;
    g_setHardwareWatchpointTypeArg = type;
    if (g_setHardwareWatchpointException)
        __throw(g_setHardwareWatchpointException);
}

__throws void  __mriPlatform_ClearHardwareWatchpoint(uint32_t address, uint32_t size,  PlatformWatchpointType type)
{
    g_clearHardwareWatchpointCalls++;
    g_clearHardwareWatchpointAddressArg = address;
    g_clearHardwareWatchpointSizeArg = size;
    g_clearHardwareWatchpointTypeArg = type;
    if (g_clearHardwareWatchpointException)
        __throw(g_clearHardwareWatchpointException);
}



// Query memory map and feature XML test instrumentation.
static char g_deviceMemoryMapXml[] = "TEST";
static char g_targetXml[] = "test!";

// Stubs called by MRI core.
uint32_t __mriPlatform_GetDeviceMemoryMapXmlSize(void)
{
    return sizeof(g_deviceMemoryMapXml) - 1;
}

const char*  __mriPlatform_GetDeviceMemoryMapXml(void)
{
    return g_deviceMemoryMapXml;
}

uint32_t __mriPlatform_GetTargetXmlSize(void)
{
    return sizeof(g_targetXml) - 1;
}

const char*  __mriPlatform_GetTargetXml(void)
{
    return g_targetXml;
}



// Semihost Test Instrumentation.
static int g_semihostCallReturnValue;
static int g_semihostCallErrno;

int platformMock_GetSemihostCallReturnValue(void)
{
    return g_semihostCallReturnValue;
}

int platformMock_GetSemihostCallErrno(void)
{
    return g_semihostCallErrno;
}

// Stubs called by MRI core.
void __mriPlatform_SetSemihostCallReturnAndErrnoValues(int returnValue, int err)
{
    g_semihostCallReturnValue = returnValue;
    g_semihostCallErrno = err;
}

void __mriPlatform_SetSymbolAddress(Buffer* pBuffer, uint32_t address)
{
    // Do nothing.
}

int __mriPlatform_SetSymbolRequest(Buffer* pBuffer)
{
    return 0;
}


// Mock Setup and Cleanup APIs.
void platformMock_Init(void)
{
    platformMock_CommInitReceiveData(g_emptyPacket);
    platformMock_CommInitTransmitDataBuffer(2 * sizeof(g_packetBuffer));
    platformMock_SetInitException(noException);
    memset(&g_initTokenCopy, 0, sizeof(g_initTokenCopy));
    g_commInterruptBit = FALSE;
    g_commShouldWaitForGdbConnect = FALSE;
    g_commIsWaitingForGdbToConnect = 0;
    g_commWaitForReceiveDataToStopCount = 0;
    g_commPrepareToWaitForGdbConnectionCount = 0;
    g_commSharingWithApplication = FALSE;
    g_initCount = 0;
    g_enteringDebuggerCount = 0;
    g_leavingDebuggerCount = 0;
    g_isDebuggeeMakingSemihostCall = FALSE;
    g_getHandleSemihostRequestCount = 0;
    g_displayFaultCauseToGdbConsoleCount = 0;
    g_packetBufferSize = sizeof(g_packetBuffer);
    g_instructionType = MRI_PLATFORM_INSTRUCTION_OTHER;
    g_advanceProgramCounterToNextInstruction = 0;
    g_setProgramCounterCalls = 0;
    g_programCounter = INITIAL_PC;
    g_singleStepping = FALSE;
    g_callToFail = 0;
    memset(&g_context, 0xff, sizeof(g_context));
    g_setHardwareBreakpointCalls = 0;
    g_setHardwareBreakpointAddressArg = 0;
    g_setHardwareBreakpointKindArg = 0;
    g_setHardwareBreakpointException = noException;
    g_clearHardwareBreakpointCalls = 0;
    g_clearHardwareBreakpointAddressArg = 0;
    g_clearHardwareBreakpointKindArg = 0;
    g_clearHardwareBreakpointException = noException;
    g_setHardwareWatchpointCalls = 0;
    g_setHardwareWatchpointAddressArg = 0;
    g_setHardwareWatchpointSizeArg = 0;
    g_setHardwareWatchpointTypeArg = MRI_PLATFORM_WRITE_WATCHPOINT;
    g_setHardwareWatchpointException = noException;
    g_clearHardwareWatchpointCalls = 0;
    g_clearHardwareWatchpointAddressArg = 0;
    g_clearHardwareWatchpointSizeArg = 0;
    g_clearHardwareWatchpointTypeArg = MRI_PLATFORM_WRITE_WATCHPOINT;
    g_clearHardwareWatchpointException = noException;
    g_semihostCallReturnValue = 0;
}

void platformMock_Uninit(void)
{
    free(g_pAlloc1);
    free(g_pAlloc2);
    g_pAlloc1 = NULL;
    g_pAlloc2 = NULL;
    commUninitTransmitDataBuffer();
}



// Stubs for Platform APIs that act as NOPs when called from mriCore during testing.
uint8_t __mriPlatform_DetermineCauseOfException(void)
{
    return SIGTRAP;
}

extern "C" void __mriPlatform_EnteringDebuggerHook(void)
{
}

extern "C" void __mriPlatform_LeavingDebuggerHook(void)
{
}
