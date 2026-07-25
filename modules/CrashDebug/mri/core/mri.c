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
/* Monitor for Remote Inspection - Provides core mri routines to initialize the debug monitor, query its state, and
   invoke it into action when a debug event occurs on the target hardware. */
#include <string.h>
#include <signal.h>
#include <errno.h>
#include "mri.h"
#include "buffer.h"
#include "hex_convert.h"
#include "try_catch.h"
#include "packet.h"
#include "token.h"
#include "core.h"
#include "platforms.h"
#include "posix4win.h"
#include "semihost.h"
#include "cmd_common.h"
#include "cmd_file.h"
#include "cmd_registers.h"
#include "cmd_memory.h"
#include "cmd_continue.h"
#include "cmd_query.h"
#include "cmd_break_watch.h"
#include "cmd_step.h"
#include "cmd_thread.h"
#include "memory.h"


typedef struct
{
    Packet      packet;
    Buffer      buffer;
    uint32_t    flags;
    int         semihostReturnCode;
    int         semihostErrno;
    uint8_t     signalValue;
} MriCore;

static MriCore g_mri;

/* MriCore::flags bit definitions. */
#define MRI_FLAGS_SUCCESSFUL_INIT   1
#define MRI_FLAGS_FIRST_EXCEPTION   2
#define MRI_FLAGS_SEMIHOST_CTRL_C   4

/* Calculates the number of items in a static array at compile time. */
#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))

/* These two routines can be provided by the debuggee to get notified on debugger entry/exit.  Can be used to safely
   turn off some external hardware so that it doesn't keep running while sitting at a breakpoint. */
void __mriPlatform_EnteringDebuggerHook(void) __attribute__((weak));
void __mriPlatform_LeavingDebuggerHook(void) __attribute__((weak));
#define Platform_EnteringDebuggerHook __mriPlatform_EnteringDebuggerHook
#define Platform_LeavingDebuggerHook  __mriPlatform_LeavingDebuggerHook

static void clearCoreStructure(void);
static void initializePlatformSpecificModulesWithDebuggerParameters(const char* pDebuggerParameters);
static void setFirstExceptionFlag(void);
static void setSuccessfulInitFlag(void);
void __mriInit(const char* pDebuggerParameters)
{
    clearCoreStructure();
    
    __try
        initializePlatformSpecificModulesWithDebuggerParameters(pDebuggerParameters);
    __catch
        return;

    setFirstExceptionFlag();
    setSuccessfulInitFlag();
}

static void clearCoreStructure(void)
{
    memset(&g_mri, 0, sizeof(g_mri));
}

static void initializePlatformSpecificModulesWithDebuggerParameters(const char* pDebuggerParameters)
{
    Token    tokens;

    Token_Init(&tokens);
    __try
    {
        __throwing_func( Token_SplitString(&tokens, pDebuggerParameters) );
        __throwing_func( Platform_Init(&tokens) );
    }
    __catch
        __rethrow;
}

static void setFirstExceptionFlag(void)
{
    g_mri.flags |= MRI_FLAGS_FIRST_EXCEPTION;
}

static void setSuccessfulInitFlag(void)
{
    g_mri.flags |= MRI_FLAGS_SUCCESSFUL_INIT;
}


static void blockIfGdbHasNotConnected(void);
static void waitForGdbToConnect(void);
static void waitForFirstCharFromHost(void);
static int  didHostSendGdbAckChar(void);
static void determineSignalValue(void);
static int  isDebugTrap(void);
static void prepareForDebuggerExit(void);
static void clearFirstExceptionFlag(void);
void __mriDebugException(void)
{
    int wasWaitingForGdbToConnect = IsWaitingForGdbToConnect();
    int justSingleStepped = Platform_IsSingleStepping();
    
    if (Platform_CommCausedInterrupt() && !Platform_CommHasReceiveData())
    {
        Platform_CommClearInterrupt();
        return;
    }

    Platform_EnteringDebuggerHook();
    blockIfGdbHasNotConnected();
    Platform_EnteringDebugger();
    determineSignalValue();
    
    if (isDebugTrap() && 
        Semihost_IsDebuggeeMakingSemihostCall() && 
        Semihost_HandleSemihostRequest() &&
        !justSingleStepped )
    {
        prepareForDebuggerExit();
        return;
    }
    
    if (!wasWaitingForGdbToConnect)
    {
        Platform_DisplayFaultCauseToGdbConsole();
        Send_T_StopResponse();
    }
    
    GdbCommandHandlingLoop();

    prepareForDebuggerExit();
}

static void blockIfGdbHasNotConnected(void)
{
    if (IsWaitingForGdbToConnect())
        waitForGdbToConnect();
}

static void waitForGdbToConnect(void)
{
    for (;;)
    {
        waitForFirstCharFromHost();
        if (didHostSendGdbAckChar())
            return;
        Platform_CommWaitForReceiveDataToStop();
        Platform_CommPrepareToWaitForGdbConnection();
    }
}

static void waitForFirstCharFromHost(void)
{
    while (Platform_CommIsWaitingForGdbToConnect())
    {
    }
}

static int didHostSendGdbAckChar(void)
{
    return ('+' == Platform_CommReceiveChar());
    
}

static void determineSignalValue(void)
{
    g_mri.signalValue = Platform_DetermineCauseOfException();
}

static int isDebugTrap(void)
{
    return g_mri.signalValue == SIGTRAP;
}

static void prepareForDebuggerExit(void)
{
    Platform_LeavingDebugger();
    Platform_LeavingDebuggerHook();
    clearFirstExceptionFlag();
}

static void clearFirstExceptionFlag(void)
{
    g_mri.flags &= ~MRI_FLAGS_FIRST_EXCEPTION;
}


/*********************************************/
/* Routines to manipulate MRI state objects. */
/*********************************************/
static int handleGDBCommand(void);
static void getPacketFromGDB(void);
void GdbCommandHandlingLoop(void)
{
    int startDebuggeeUpAgain;
    
    do
    {
        startDebuggeeUpAgain = handleGDBCommand();
    } while (!startDebuggeeUpAgain);
}

static int handleGDBCommand(void)
{
    Buffer*         pBuffer = GetBuffer();
    uint32_t        handlerResult = 0;
    char            commandChar;
    size_t          i;
    static const struct
    {
        uint32_t     (*Handler)(void);
        char         commandChar;
    } commandTable[] =
    {
        {Send_T_StopResponse,                       '?'},
        {HandleContinueCommand,                     'c'},
        {HandleContinueWithSignalCommand,           'C'},
        {HandleFileIOCommand,                       'F'},
        {HandleRegisterReadCommand,                 'g'},
        {HandleRegisterWriteCommand,                'G'},
        {HandleThreadContextCommand,                'H'},
        {HandleMemoryReadCommand,                   'm'},
        {HandleMemoryWriteCommand,                  'M'},
        {HandleQueryCommand,                        'q'},
        {HandleSingleStepCommand,                   's'},
        {HandleSingleStepWithSignalCommand,         'S'},
        {HandleIsThreadActiveCommand,               'T'},
        {HandleBinaryMemoryWriteCommand,            'X'},
        {HandleBreakpointWatchpointRemoveCommand,   'z'},
        {HandleBreakpointWatchpointSetCommand,      'Z'}
    };
    
    getPacketFromGDB();
    
    commandChar = Buffer_ReadChar(pBuffer);
    for (i = 0 ; i < ARRAY_SIZE(commandTable) ; i++)
    {
        if (commandTable[i].commandChar == commandChar)
        {
            handlerResult = commandTable[i].Handler();
            if (handlerResult & HANDLER_RETURN_RETURN_IMMEDIATELY)
            {
                return handlerResult & HANDLER_RETURN_RESUME_PROGRAM;
            }
            else
            {
                break;
            }
        }
    }
    if (ARRAY_SIZE(commandTable) == i)
        PrepareEmptyResponseForUnknownCommand();

    SendPacketToGdb();
    return (handlerResult & HANDLER_RETURN_RESUME_PROGRAM);
}

static void getPacketFromGDB(void)
{
    InitBuffer();
    Packet_GetFromGDB(&g_mri.packet, &g_mri.buffer);
}


void InitBuffer(void)
{
    Buffer_Init(&g_mri.buffer, Platform_GetPacketBuffer(), Platform_GetPacketBufferSize());
}


void PrepareStringResponse(const char* pErrorString)
{
    InitBuffer();
    Buffer_WriteString(&g_mri.buffer, pErrorString);
}


int WasSuccessfullyInit(void)
{
    return (int)(g_mri.flags & MRI_FLAGS_SUCCESSFUL_INIT);
}


int WasControlCFlagSentFromGdb(void)
{
    return (int)(g_mri.flags & MRI_FLAGS_SEMIHOST_CTRL_C);
}


int IsWaitingForGdbToConnect(void)
{
    return IsFirstException() && Platform_CommShouldWaitForGdbConnect();
}



void RecordControlCFlagSentFromGdb(int controlCFlag)
{
    if (controlCFlag)
        g_mri.flags |= MRI_FLAGS_SEMIHOST_CTRL_C;
    else
        g_mri.flags &= ~MRI_FLAGS_SEMIHOST_CTRL_C;
}


int WasSemihostCallCancelledByGdb(void)
{
    return g_mri.semihostErrno == EINTR;
}


void FlagSemihostCallAsHandled(void)
{
    Platform_AdvanceProgramCounterToNextInstruction();
    Platform_SetSemihostCallReturnAndErrnoValues(g_mri.semihostReturnCode, g_mri.semihostErrno);
}


int IsFirstException(void)
{
    return (int)(g_mri.flags & MRI_FLAGS_FIRST_EXCEPTION);
}


void SetSignalValue(uint8_t signalValue)
{
    g_mri.signalValue = signalValue;
}


uint8_t GetSignalValue(void)
{
    return g_mri.signalValue;
}


void SetSemihostReturnValues(int semihostReturnCode, int semihostErrNo)
{
    g_mri.semihostReturnCode = semihostReturnCode;
    g_mri.semihostErrno = semihostErrNo;
}


int GetSemihostReturnCode(void)
{
    return g_mri.semihostReturnCode;
}


int GetSemihostErrno(void)
{
    return g_mri.semihostErrno;
}


Buffer* GetBuffer(void)
{
    return &g_mri.buffer;
}


Buffer* GetInitializedBuffer(void)
{
    InitBuffer();
    return &g_mri.buffer;
}


void SendPacketToGdb(void)
{
    if (Buffer_OverrunDetected(&g_mri.buffer))
    {
        InitBuffer();
        Buffer_WriteString(&g_mri.buffer, MRI_ERROR_BUFFER_OVERRUN);
    }

    Buffer_SetEndOfBuffer(&g_mri.buffer);
    Packet_SendToGDB(&g_mri.packet, &g_mri.buffer);
}
