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
#include <common.h>
#include <CrashCatcher.h>
#include <gdb_console.h>
#include <IMemory.h>
#include <signal.h>
#include <string.h>
#include <MemorySim.h>
#include <mri.h>
#include <mriPlatform.h>
#include <platforms.h>
#include <posix4win.h>
#include <printfSpy.h>
#include <semihost.h>
#include "mriPlatformPriv.h"
#include <MallocFailureInject.h>



/* NOTE: This is the original version of the following XML which has had things stripped to reduce the amount of
         FLASH consumed by the debug monitor.  This includes the removal of the copyright comment.
<?xml version="1.0"?>
<!-- Copyright (C) 2010, 2011 Free Software Foundation, Inc.

     Copying and distribution of this file, with or without modification,
     are permitted in any medium without royalty provided the copyright
     notice and this notice are preserved.  -->

<!DOCTYPE feature SYSTEM "gdb-target.dtd">
<feature name="org.gnu.gdb.arm.m-profile">
  <reg name="r0" bitsize="32"/>
  <reg name="r1" bitsize="32"/>
  <reg name="r2" bitsize="32"/>
  <reg name="r3" bitsize="32"/>
  <reg name="r4" bitsize="32"/>
  <reg name="r5" bitsize="32"/>
  <reg name="r6" bitsize="32"/>
  <reg name="r7" bitsize="32"/>
  <reg name="r8" bitsize="32"/>
  <reg name="r9" bitsize="32"/>
  <reg name="r10" bitsize="32"/>
  <reg name="r11" bitsize="32"/>
  <reg name="r12" bitsize="32"/>
  <reg name="sp" bitsize="32" type="data_ptr"/>
  <reg name="lr" bitsize="32"/>
  <reg name="pc" bitsize="32" type="code_ptr"/>
  <reg name="xpsr" bitsize="32" regnum="25"/>
</feature>
*/
static const char g_targetXML[] =
    "<?xml version=\"1.0\"?>\n"
    "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
    "<target>\n"
    "<feature name=\"org.gnu.gdb.arm.m-profile\">\n"
    "<reg name=\"r0\" bitsize=\"32\"/>\n"
    "<reg name=\"r1\" bitsize=\"32\"/>\n"
    "<reg name=\"r2\" bitsize=\"32\"/>\n"
    "<reg name=\"r3\" bitsize=\"32\"/>\n"
    "<reg name=\"r4\" bitsize=\"32\"/>\n"
    "<reg name=\"r5\" bitsize=\"32\"/>\n"
    "<reg name=\"r6\" bitsize=\"32\"/>\n"
    "<reg name=\"r7\" bitsize=\"32\"/>\n"
    "<reg name=\"r8\" bitsize=\"32\"/>\n"
    "<reg name=\"r9\" bitsize=\"32\"/>\n"
    "<reg name=\"r10\" bitsize=\"32\"/>\n"
    "<reg name=\"r11\" bitsize=\"32\"/>\n"
    "<reg name=\"r12\" bitsize=\"32\"/>\n"
    "<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>\n"
    "<reg name=\"lr\" bitsize=\"32\"/>\n"
    "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>\n"
    "<reg name=\"xpsr\" bitsize=\"32\" regnum=\"25\"/>\n"
    "</feature>\n"
    "<feature name=\"org.gnu.gdb.arm.m-system\">\n"
    "<reg name=\"msp\" bitsize=\"32\" regnum=\"26\"/>\n"
    "<reg name=\"psp\" bitsize=\"32\" regnum=\"27\"/>\n"
    "</feature>\n"
    "</target>\n";

static const char g_targetFpuXML[] =
    "<?xml version=\"1.0\"?>\n"
    "<!DOCTYPE feature SYSTEM \"gdb-target.dtd\">\n"
    "<target>\n"
    "<feature name=\"org.gnu.gdb.arm.m-profile\">\n"
    "<reg name=\"r0\" bitsize=\"32\"/>\n"
    "<reg name=\"r1\" bitsize=\"32\"/>\n"
    "<reg name=\"r2\" bitsize=\"32\"/>\n"
    "<reg name=\"r3\" bitsize=\"32\"/>\n"
    "<reg name=\"r4\" bitsize=\"32\"/>\n"
    "<reg name=\"r5\" bitsize=\"32\"/>\n"
    "<reg name=\"r6\" bitsize=\"32\"/>\n"
    "<reg name=\"r7\" bitsize=\"32\"/>\n"
    "<reg name=\"r8\" bitsize=\"32\"/>\n"
    "<reg name=\"r9\" bitsize=\"32\"/>\n"
    "<reg name=\"r10\" bitsize=\"32\"/>\n"
    "<reg name=\"r11\" bitsize=\"32\"/>\n"
    "<reg name=\"r12\" bitsize=\"32\"/>\n"
    "<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>\n"
    "<reg name=\"lr\" bitsize=\"32\"/>\n"
    "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>\n"
    "<reg name=\"xpsr\" bitsize=\"32\" regnum=\"25\"/>\n"
    "</feature>\n"
    "<feature name=\"org.gnu.gdb.arm.m-system\">\n"
    "<reg name=\"msp\" bitsize=\"32\" regnum=\"26\"/>\n"
    "<reg name=\"psp\" bitsize=\"32\" regnum=\"27\"/>\n"
    "</feature>\n"
    "<feature name=\"org.gnu.gdb.arm.vfp\">\n"
    "<reg name=\"d0\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d1\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d2\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d3\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d4\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d5\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d6\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d7\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d8\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d9\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d10\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d11\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d12\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d13\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d14\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"d15\" bitsize=\"64\" type=\"ieee_double\"/>\n"
    "<reg name=\"fpscr\" bitsize=\"32\" type=\"int\" group=\"float\"/>\n"
    "</feature>\n"
    "</target>\n";


/* Addresses of fault status registers in System Control Block. */
#define CFSR  0xE000ED28
#define HFSR  0xE000ED2C
#define MMFAR 0xE000ED34
#define BFAR  0xE000ED38


static RegisterContext* g_pContext;
static IMemory*         g_pMemory;
static IComm*           g_pComm;
static char             g_packetBuffer[16 * 1024];
static int              g_memoryFaultEncountered;
static int              g_shouldWaitForGdbConnect = TRUE;


/* Core MRI function not exposed in public header since typically called by ASM. */
void __mriDebugException(void);

/* Forward static function declarations. */
static uint32_t getCurrentlyExecutingExceptionNumber(void);
static void displayHardFaultCauseToGdbConsole(void);
static void displayMemFaultCauseToGdbConsole(void);
static void displayBusFaultCauseToGdbConsole(void);
static void displayUsageFaultCauseToGdbConsole(void);
static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint32_t registerValue);
int hasFPURegisters();
static void readBytesFromBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount);

/* Calculates the number of items in a static array at compile time. */
#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))


__throws void mriPlatform_Init(RegisterContext* pContext, IMemory* pMem)
{
    g_pContext = pContext;
    g_pMemory = pMem;
    g_memoryFaultEncountered = FALSE;

    __mriInit("");
}

void mriPlatform_Run(IComm* pComm)
{
    g_pComm = pComm;
    do
    {
        __mriDebugException();
    } while (!IComm_ShouldStopRun(pComm));
}

/* This routine is just used for unit testing so that it can run tests where a T response packet and/or exception
   information returned to GDB is expected.  This only happens on first mriPlatform_Run() when it doesn't need to wait 
   for GDB connection. 
*/
void mriPlatform_setWaitForGdbConnect(int setting)
{
    g_shouldWaitForGdbConnect = setting;
}




void Platform_Init(Token* pParameterTokens)
{
}

char* Platform_GetPacketBuffer(void)
{
    return g_packetBuffer;
}

uint32_t  Platform_GetPacketBufferSize(void)
{
    return sizeof(g_packetBuffer);
}

void Platform_EnteringDebugger(void)
{
    Platform_DisableSingleStep();
}

void Platform_LeavingDebugger(void)
{
}

uint32_t Platform_MemRead32(const void* pv)
{
    uint32_t retVal = 0;
    __try
        retVal = IMemory_Read32(g_pMemory, (uint32_t)(unsigned long)pv);
    __catch
        g_memoryFaultEncountered++;
    return retVal;
}

uint16_t Platform_MemRead16(const void* pv)
{
    uint16_t retVal = 0;
    __try
        retVal = IMemory_Read16(g_pMemory, (uint32_t)(unsigned long)pv);
    __catch
        g_memoryFaultEncountered++;
    return retVal;
}

uint8_t Platform_MemRead8(const void* pv)
{
    uint8_t retVal = 0;
    __try
        retVal = IMemory_Read8(g_pMemory, (uint32_t)(unsigned long)pv);
    __catch
        g_memoryFaultEncountered++;
    return retVal;
}

void Platform_MemWrite32(void* pv, uint32_t value)
{
    __try
        IMemory_Write32(g_pMemory, (uint32_t)(unsigned long)pv, value);
    __catch
        g_memoryFaultEncountered++;
}

void Platform_MemWrite16(void* pv, uint16_t value)
{
    __try
        IMemory_Write16(g_pMemory, (uint32_t)(unsigned long)pv, value);
    __catch
        g_memoryFaultEncountered++;
}

void Platform_MemWrite8(void* pv, uint8_t value)
{
    __try
        IMemory_Write8(g_pMemory, (uint32_t)(unsigned long)pv, value);
    __catch
        g_memoryFaultEncountered++;
}

uint32_t Platform_CommHasReceiveData(void)
{
    return IComm_HasReceiveData(g_pComm);
}

int Platform_CommReceiveChar(void)
{
    return IComm_ReceiveChar(g_pComm);
}

void Platform_CommSendChar(int character)
{
    IComm_SendChar(g_pComm, character);
}

int Platform_CommCausedInterrupt(void)
{
    return FALSE;
}

void Platform_CommClearInterrupt(void)
{
}

int Platform_CommShouldWaitForGdbConnect(void)
{
    return g_shouldWaitForGdbConnect;
}

int Platform_CommSharingWithApplication(void)
{
    return FALSE;
}

void Platform_CommPrepareToWaitForGdbConnection(void)
{
}

int Platform_CommIsWaitingForGdbToConnect(void)
{
    return !IComm_IsGdbConnected(g_pComm);
}

void Platform_CommWaitForReceiveDataToStop(void)
{
}

uint8_t Platform_DetermineCauseOfException(void)
{
    uint32_t exceptionNumber = getCurrentlyExecutingExceptionNumber();

    switch(exceptionNumber)
    {
    case 2:
        /* NMI */
        return SIGINT;
    case 3:
        /* HardFault */
        return SIGSEGV;
    case 4:
        /* MemManage */
        return SIGSEGV;
    case 5:
        /* BusFault */
        return SIGBUS;
    case 6:
        /* UsageFault */
        return SIGILL;
    case 12:
        /* Debug Monitor */
        return SIGTRAP;
    default:
        /* NOTE: Catch all signal will be SIGSTOP. */
        return SIGSTOP;
    }
}

static uint32_t getCurrentlyExecutingExceptionNumber(void)
{
    return (g_pContext->exceptionPSR & 0xFF);
}

void Platform_DisplayFaultCauseToGdbConsole(void)
{
    switch (getCurrentlyExecutingExceptionNumber())
    {
    case 3:
        /* HardFault */
        displayHardFaultCauseToGdbConsole();
        break;
    case 4:
        /* MemManage */
        displayMemFaultCauseToGdbConsole();
        break;
    case 5:
        /* BusFault */
        displayBusFaultCauseToGdbConsole();
        break;
    case 6:
        /* UsageFault */
        displayUsageFaultCauseToGdbConsole();
        break;
    default:
        return;
    }
    WriteStringToGdbConsole("\n");
}

static void displayHardFaultCauseToGdbConsole(void)
{
    static const uint32_t debugEventBit = 1 << 31;
    static const uint32_t forcedBit = 1 << 30;
    static const uint32_t vectorTableReadBit = 1 << 1;
    volatile uint32_t     hardFaultStatusRegister = 0;

    WriteStringToGdbConsole("\n**Hard Fault**");

    __try
        hardFaultStatusRegister = IMemory_Read32(g_pMemory, HFSR);
    __catch
        return;
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(hardFaultStatusRegister);

    if (hardFaultStatusRegister & debugEventBit)
        WriteStringToGdbConsole("\n    Debug Event");

    if (hardFaultStatusRegister & vectorTableReadBit)
        WriteStringToGdbConsole("\n    Vector Table Read");

    if (hardFaultStatusRegister & forcedBit)
    {
        WriteStringToGdbConsole("\n    Forced");
        displayMemFaultCauseToGdbConsole();
        displayBusFaultCauseToGdbConsole();
        displayUsageFaultCauseToGdbConsole();
    }
}

static void displayMemFaultCauseToGdbConsole(void)
{
    static const uint32_t MMARValidBit = 1 << 7;
    static const uint32_t FPLazyStatePreservationBit = 1 << 5;
    static const uint32_t stackingErrorBit = 1 << 4;
    static const uint32_t unstackingErrorBit = 1 << 3;
    static const uint32_t dataAccess = 1 << 1;
    static const uint32_t instructionFetch = 1;
    uint32_t volatile     memManageFaultStatusRegister = 0;

    /* Check to make sure that there is a memory fault to display. */
    __try
        memManageFaultStatusRegister = IMemory_Read32(g_pMemory, CFSR) & 0xFF;
    __catch
        return;
    if (memManageFaultStatusRegister == 0)
        return;

    WriteStringToGdbConsole("\n**MPU Fault**");
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(memManageFaultStatusRegister);

    if (memManageFaultStatusRegister & MMARValidBit)
    {
        __try
        {
            uint32_t memManageFaultAddressRegister = IMemory_Read32(g_pMemory, MMFAR);
            WriteStringToGdbConsole("\n    Fault Address: ");
            WriteHexValueToGdbConsole(memManageFaultAddressRegister);
        }
        __catch
        {
        }
    }
    if (memManageFaultStatusRegister & FPLazyStatePreservationBit)
        WriteStringToGdbConsole("\n    FP Lazy Preservation");
    if (memManageFaultStatusRegister & stackingErrorBit)
        WriteStringToGdbConsole("\n    Stacking Error");
    if (memManageFaultStatusRegister & unstackingErrorBit)
        WriteStringToGdbConsole("\n    Unstacking Error");
    if (memManageFaultStatusRegister & dataAccess)
        WriteStringToGdbConsole("\n    Data Access");
    if (memManageFaultStatusRegister & instructionFetch)
        WriteStringToGdbConsole("\n    Instruction Fetch");
}

static void displayBusFaultCauseToGdbConsole(void)
{
    static const uint32_t BFARValidBit = 1 << 7;
    static const uint32_t FPLazyStatePreservationBit = 1 << 5;
    static const uint32_t stackingErrorBit = 1 << 4;
    static const uint32_t unstackingErrorBit = 1 << 3;
    static const uint32_t impreciseDataAccessBit = 1 << 2;
    static const uint32_t preciseDataAccessBit = 1 << 1;
    static const uint32_t instructionPrefetch = 1;
    uint32_t volatile     busFaultStatusRegister = 0;

    __try
        busFaultStatusRegister = (IMemory_Read32(g_pMemory, CFSR) >> 8) & 0xFF;
    __catch
        return;

    /* Check to make sure that there is a bus fault to display. */
    if (busFaultStatusRegister == 0)
        return;

    WriteStringToGdbConsole("\n**Bus Fault**");
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(busFaultStatusRegister);

    if (busFaultStatusRegister & BFARValidBit)
    {
        __try
        {
            uint32_t busFaultAddressRegister = IMemory_Read32(g_pMemory, BFAR);
            WriteStringToGdbConsole("\n    Fault Address: ");
            WriteHexValueToGdbConsole(busFaultAddressRegister);
        }
        __catch
        {
        }
    }
    if (busFaultStatusRegister & FPLazyStatePreservationBit)
        WriteStringToGdbConsole("\n    FP Lazy Preservation");
    if (busFaultStatusRegister & stackingErrorBit)
        WriteStringToGdbConsole("\n    Stacking Error");
    if (busFaultStatusRegister & unstackingErrorBit)
        WriteStringToGdbConsole("\n    Unstacking Error");
    if (busFaultStatusRegister & impreciseDataAccessBit)
        WriteStringToGdbConsole("\n    Imprecise Data Access");
    if (busFaultStatusRegister & preciseDataAccessBit)
        WriteStringToGdbConsole("\n    Precise Data Access");
    if (busFaultStatusRegister & instructionPrefetch)
        WriteStringToGdbConsole("\n    Instruction Prefetch");
}

static void displayUsageFaultCauseToGdbConsole(void)
{
    static const uint32_t divideByZeroBit = 1 << 9;
    static const uint32_t unalignedBit = 1 << 8;
    static const uint32_t coProcessorAccessBit = 1 << 3;
    static const uint32_t invalidPCBit = 1 << 2;
    static const uint32_t invalidStateBit = 1 << 1;
    static const uint32_t undefinedInstructionBit = 1;
    volatile uint32_t     usageFaultStatusRegister = 0;

    /* Make sure that there is a usage fault to display. */
    __try
        usageFaultStatusRegister = (IMemory_Read32(g_pMemory, CFSR) >> 16) & 0xFFFF;
    __catch
        return;
    if (usageFaultStatusRegister == 0)
        return;

    WriteStringToGdbConsole("\n**Usage Fault**");
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(usageFaultStatusRegister);

    if (usageFaultStatusRegister & divideByZeroBit)
        WriteStringToGdbConsole("\n    Divide by Zero");
    if (usageFaultStatusRegister & unalignedBit)
        WriteStringToGdbConsole("\n    Unaligned Access");
    if (usageFaultStatusRegister & coProcessorAccessBit)
        WriteStringToGdbConsole("\n    Coprocessor Access");
    if (usageFaultStatusRegister & invalidPCBit)
        WriteStringToGdbConsole("\n    Invalid Exception Return State");
    if (usageFaultStatusRegister & invalidStateBit)
        WriteStringToGdbConsole("\n    Invalid State");
    if (usageFaultStatusRegister & undefinedInstructionBit)
        WriteStringToGdbConsole("\n    Undefined Instruction");
}

void Platform_EnableSingleStep(void)
{
}

void Platform_DisableSingleStep(void)
{
}

int Platform_IsSingleStepping(void)
{
    /* Can't really single step so just return FALSE here. */
    return FALSE;
}

void Platform_SetProgramCounter(uint32_t newPC)
{
    /* This is just called when user tries to set new PC value from continue command which we don't really support
       so just ignore this request. */
}

void Platform_AdvanceProgramCounterToNextInstruction(void)
{
    /* This is just called to advance past hardcoded breakpoints before continuing execution so just ignore. */
}

int Platform_WasProgramCounterModifiedByUser(void)
{
    /* Pretend that PC was always modified so that mriCore doesn't check for hardcoded breakpoint on resume. */
    return TRUE;
}

int Platform_WasMemoryFaultEncountered(void)
{
    int memoryFaultEncountered = g_memoryFaultEncountered;
    g_memoryFaultEncountered = 0;
    return memoryFaultEncountered;
}

void Platform_WriteTResponseRegistersToBuffer(Buffer* pBuffer)
{
    sendRegisterForTResponse(pBuffer, 12, g_pContext->R[12]);
    sendRegisterForTResponse(pBuffer, 13, g_pContext->R[13]);
    sendRegisterForTResponse(pBuffer, 14, g_pContext->R[14]);
    sendRegisterForTResponse(pBuffer, 15, g_pContext->R[15]);
}

static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint32_t registerValue)
{
    Buffer_WriteByteAsHex(pBuffer, registerOffset);
    Buffer_WriteChar(pBuffer, ':');
    writeBytesToBufferAsHex(pBuffer, &registerValue, sizeof(registerValue));
    Buffer_WriteChar(pBuffer, ';');
}

void writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount)
{
    uint8_t* pByte = (uint8_t*)pBytes;
    size_t   i;

    for (i = 0 ; i < byteCount ; i++)
        Buffer_WriteByteAsHex(pBuffer, *pByte++);
}

void Platform_CopyContextToBuffer(Buffer* pBuffer)
{
    writeBytesToBufferAsHex(pBuffer, g_pContext->R, sizeof(g_pContext->R));
    if (hasFPURegisters())
        writeBytesToBufferAsHex(pBuffer, g_pContext->FPR, sizeof(g_pContext->FPR));
}

int hasFPURegisters()
{
    return g_pContext->flags & CRASH_CATCHER_FLAGS_FLOATING_POINT;
}

void Platform_CopyContextFromBuffer(Buffer* pBuffer)
{
    readBytesFromBufferAsHex(pBuffer, g_pContext->R, sizeof(g_pContext->R));
    if (hasFPURegisters())
        readBytesFromBufferAsHex(pBuffer, g_pContext->FPR, sizeof(g_pContext->FPR));
}

static void readBytesFromBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount)
{
    uint8_t* pByte = (uint8_t*)pBytes;
    size_t   i;

    for (i = 0 ; i < byteCount; i++)
        *pByte++ = Buffer_ReadByteAsHex(pBuffer);
}

uint32_t Platform_GetDeviceMemoryMapXmlSize(void)
{
    return strlen(MemorySim_GetMemoryMapXML(g_pMemory));
}

const char* Platform_GetDeviceMemoryMapXml(void)
{
    return MemorySim_GetMemoryMapXML(g_pMemory);
}

uint32_t Platform_GetTargetXmlSize(void)
{
    if (hasFPURegisters())
        return sizeof(g_targetFpuXML) - 1;
    else
        return sizeof(g_targetXML) - 1;
}

const char* Platform_GetTargetXml(void)
{
    if (hasFPURegisters())
        return g_targetFpuXML;
    else
        return g_targetXML;
}

__throws void Platform_SetHardwareBreakpoint(uint32_t address, uint32_t kind)
{
    /* Ignore during post-mortem debugging. */
}

__throws void Platform_ClearHardwareBreakpoint(uint32_t address, uint32_t kind)
{
    /* Ignore during post-mortem debugging. */
}

__throws void Platform_SetHardwareWatchpoint(uint32_t address, uint32_t size, PlatformWatchpointType type)
{
    /* Ignore during post-mortem debugging. */
}

__throws void Platform_ClearHardwareWatchpoint(uint32_t address, uint32_t size,  PlatformWatchpointType type)
{
    /* Ignore during post-mortem debugging. */
}

PlatformInstructionType Platform_TypeOfCurrentInstruction(void)
{
    /* Ignore semihost and hard coded breakpoint instructions during post-mortem debugging. */
    return MRI_PLATFORM_INSTRUCTION_OTHER;
}

PlatformSemihostParameters Platform_GetSemihostCallParameters(void)
{
    /* Ignore during post-mortem debugging. */
    PlatformSemihostParameters parameters;
    memset(&parameters, 0, sizeof(parameters));
    return parameters;
}

void Platform_SetSemihostCallReturnAndErrnoValues(int returnValue, int err)
{
    /* Ignore during post-mortem debugging. */
}

int Semihost_IsDebuggeeMakingSemihostCall(void)
{
    /* Ignore semihost instructions during post-mortem debugging. */
    return FALSE;
}

int Semihost_HandleSemihostRequest(void)
{
    /* Ignore semihost requests during post-mortem debugging. */
    return FALSE;
}

void __mriPlatform_EnteringDebuggerHook(void)
{
}

void __mriPlatform_LeavingDebuggerHook(void)
{
}

struct symbols_t {
    const char *name;
    uint32_t value;
};

static struct symbols_t chibios_symbol_list[] = {
    {"ch", 0},       /* System data structure */
    {"ch_debug", 0}, /* Memory Signature containing offsets of fields in rlist */
};
#define CHIBIOS_VAL_CH 0
#define CHIBIOS_VAL_CH_DEBUG 1

static uint8_t curr_symbol = 0;
int __mriPlatform_SetSymbolRequest(Buffer* pBuffer)
{
    if (curr_symbol >= 2) // based on number of vars in chibios_symbol_list
        return FALSE;
    uint8_t i;
    for (i=0; i<strlen(chibios_symbol_list[curr_symbol].name); i++)
        Buffer_WriteByteAsHex(pBuffer, chibios_symbol_list[curr_symbol].name[i]);
    return TRUE;
}

void __mriPlatform_SetSymbolAddress(Buffer* pBuffer, uint32_t address)
{
    if (curr_symbol >= 2)
        return;
    char symbolname[20] = {0};
    uint8_t i = 0;
    while (!Buffer_IsNextCharEqualTo(pBuffer, ':') && i < sizeof(symbolname)) {
        symbolname[i++] = Buffer_ReadByteAsHex(pBuffer);
    }
    symbolname[i++] = '\0';
    if (strcmp(symbolname, chibios_symbol_list[curr_symbol].name) == 0) {
        chibios_symbol_list[curr_symbol].value = address;
        curr_symbol++;
    }
}

/***************************************************************************
 *   Copyright (C) 2012 by Matthias Blaicher                               *
 *   Matthias Blaicher - matthias@blaicher.com                             *
 *                                                                         *
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

struct stack_register_offset {
	unsigned short number;		/* register number */
	signed short offset;		/* offset in bytes from stack head, or -1 to indicate
					 * register is not stacked, or -2 to indicate this is the
					 * stack pointer register */
	unsigned short width_bits;
};

struct rtos_register_stacking {
	unsigned char stack_registers_size;
	signed char stack_growth_direction;
	unsigned char num_output_registers;
	const struct stack_register_offset *register_offsets;
};

void read_address_to_mem(void *dest, uint32_t source, uint32_t size);
int rtos_generic_stack_read(Buffer* pBuffer, const struct rtos_register_stacking *stacking, int64_t stack_ptr);

void read_address_to_mem(void *dest, uint32_t source, uint32_t size)
{
    uint32_t i;
    for (i = 0; i < size; i++)
        ((uint8_t *)dest)[i] = Platform_MemRead8((void*)(uint64_t)(source+i));
}

int rtos_generic_stack_read(Buffer* pBuffer, const struct rtos_register_stacking *stacking, int64_t stack_ptr)
{
	if (stack_ptr == 0 || g_pContext == NULL) {
		WriteStringToGdbConsole("Error: null stack pointer in thread\n");
		return FALSE;
	}
	/* Read the stack */
	uint8_t *stack_data = throwingZeroedMalloc(stacking->stack_registers_size);
	uint32_t address = stack_ptr;

	if (stacking->stack_growth_direction == 1)
		address -= stacking->stack_registers_size;
    read_address_to_mem(stack_data, address, stacking->stack_registers_size);

#if 0
		LOG_OUTPUT("Stack Data :");
		for (i = 0; i < stacking->stack_registers_size; i++)
			LOG_OUTPUT("%02X", stack_data[i]);
		LOG_OUTPUT("\r\n");
#endif

	uint32_t new_stack_ptr = stack_ptr - stacking->stack_growth_direction * stacking->stack_registers_size;
    int i;
	for (i = 0; i < stacking->num_output_registers; ++i) {
		int offset = stacking->register_offsets[i].offset;
		if (offset == -2) {
            g_pContext->R[i] = new_stack_ptr;
        } else if (offset != -1) {
            memcpy(&g_pContext->R[i], stack_data + offset, stacking->register_offsets[i].width_bits / 8);
        }
	}
    // writeBytesToBufferAsHex(pBuffer, regs, TOTAL_REG_COUNT * sizeof(uint32_t));
	free(stack_data);
/*	LOG_OUTPUT("Output register string: %s\r\n", *hex_reg_list); */
	return TRUE;
}

static const struct stack_register_offset rtos_chibios_arm_v7m_stack_offsets[TOTAL_REG_COUNT-2] = {
	{ R0,   -1,   32 },		/* r0   */
	{ R1,   -1,   32 },		/* r1   */
	{ R2,   -1,   32 },		/* r2   */
	{ R3,   -1,   32 },		/* r3   */
	{ R4,   0x00, 32 },		/* r4   */
	{ R5,   0x04, 32 },		/* r5   */
	{ R6,   0x08, 32 },		/* r6   */
	{ R7,   0x0c, 32 },		/* r7   */
	{ R8,   0x10, 32 },		/* r8   */
	{ R9,   0x14, 32 },		/* r9   */
	{ R10,  0x18, 32 },		/* r10  */
	{ R11,  0x1c, 32 },		/* r11  */
	{ R12,  -1,   32 },		/* r12  */
	{ SP,  -2,   32 },		/* sp   */
	{ LR,  -1,   32 },		/* lr   */
	{ PC,   0x20, 32 },		/* pc   */
	{ XPSR, -1,   32 },		/* xPSR */
};

const struct rtos_register_stacking rtos_chibios_arm_v7m_stacking = {
	0x24,					/* stack_registers_size */
	-1,						/* stack_growth_direction */
	TOTAL_REG_COUNT-2,	/* num_output_registers */
	rtos_chibios_arm_v7m_stack_offsets	/* register_offsets */
};

static const struct stack_register_offset rtos_chibios_arm_v7m_stack_offsets_w_fpu[TOTAL_REG_COUNT-2] = {
	{ R0,   -1,   32 },		/* r0   */
	{ R1,   -1,   32 },		/* r1   */
	{ R2,   -1,   32 },		/* r2   */
	{ R3,   -1,   32 },		/* r3   */
	{ R4,   0x40, 32 },		/* r4   */
	{ R5,   0x44, 32 },		/* r5   */
	{ R6,   0x48, 32 },		/* r6   */
	{ R7,   0x4c, 32 },		/* r7   */
	{ R8,   0x50, 32 },		/* r8   */
	{ R9,   0x54, 32 },		/* r9   */
	{ R10,  0x58, 32 },		/* r10  */
	{ R11,  0x5c, 32 },		/* r11  */
	{ R12,  -1,   32 },		/* r12  */
	{ SP,  -2,   32 },		/* sp   */
	{ LR,  -1,   32 },		/* lr   */
	{ PC,   0x60, 32 },		/* pc   */
	{ XPSR, -1,   32 },		/* xPSR */
};

const struct rtos_register_stacking rtos_chibios_arm_v7m_stacking_w_fpu = {
	0x64,										/* stack_registers_size */
	-1,											/* stack_growth_direction */
	TOTAL_REG_COUNT-2,      						/* num_output_registers */
	rtos_chibios_arm_v7m_stack_offsets_w_fpu	/* register_offsets */
};

// ChibiOS Specific Debugging
struct chibios_chdebug {
    char      ch_identifier[4];       
    uint8_t   ch_zero;                
    uint8_t   ch_size;                
    uint16_t  ch_version;             
    uint8_t   ch_ptrsize;             
    uint8_t   ch_timesize;            
    uint8_t   ch_threadsize;          
    uint8_t   cf_off_prio;            
    uint8_t   cf_off_ctx;             
    uint8_t   cf_off_newer;           
    uint8_t   cf_off_older;           
    uint8_t   cf_off_name;            
    uint8_t   cf_off_stklimit;        
    uint8_t   cf_off_state;           
    uint8_t   cf_off_flags;           
    uint8_t   cf_off_refs;            
    uint8_t   cf_off_preempt;         
    uint8_t   cf_off_time;            
};
  
#define GET_CH_KERNEL_MAJOR(coded_version) ((coded_version >> 11) & 0x1f)
#define GET_CH_KERNEL_MINOR(coded_version) ((coded_version >> 6) & 0x1f)
#define GET_CH_KERNEL_PATCH(coded_version) ((coded_version >> 0) & 0x3f)

static const char * const chibios_thread_states[] = { "READY", "CURRENT",
"WTSTART", "SUSPENDED", "QUEUED", "WTSEM", "WTMTX", "WTCOND", "SLEEPING",
"WTEXIT", "WTOREVT", "WTANDEVT", "SNDMSGQ", "SNDMSG", "WTMSG", "FINAL"
};

#define CHIBIOS_NUM_STATES ARRAY_SIZE(chibios_thread_states)

/* Maximum ChibiOS thread name. There is no real limit set by ChibiOS but 64
* chars ought to be enough.
*/
#define CHIBIOS_THREAD_NAME_STR_SIZE (64)

struct thread_detail {
	int64_t threadid;
	uint8_t exists;
	char thread_name_str[CHIBIOS_THREAD_NAME_STR_SIZE];
	char extra_info_str[100];
};

struct chibios_chdebug *chibios_signature = 0;
const struct rtos_register_stacking *stacking_info = 0;
struct thread_detail thread_details[50] = {0};
uint16_t thread_count = 0;
uint32_t current_thread = 0;
/* Offset of the rlist structure within the system data structure (ch) */
#define CH_RLIST_OFFSET 0x00

static int chibios_update_memory_signature()
{
    struct chibios_chdebug *signature;
    if (chibios_signature != NULL) {
        free(chibios_signature);
    }
    chibios_signature = NULL;
    signature = throwingZeroedMalloc(sizeof(*signature));
    if (!signature) {
        WriteStringToGdbConsole("\nCould not allocate space for ChibiOS/RT memory signature\n");
        return FALSE;
    }

    read_address_to_mem(signature, chibios_symbol_list[CHIBIOS_VAL_CH_DEBUG].value, sizeof(*signature));

    if (strncmp(signature->ch_identifier, "main", 4) != 0) {
        WriteStringToGdbConsole("Memory signature identifier does not contain magic bytes.\n");
        goto errfree;
    }

    if (signature->ch_size < sizeof(*signature)) {
        WriteStringToGdbConsole("ChibiOS/RT memory signature claims to be smaller "
                "than expected\n");
        goto errfree;
    }

    if (signature->ch_size > sizeof(*signature)) {
        WriteStringToGdbConsole("ChibiOS/RT memory signature claims to be bigger than"
                    " expected. Assuming compatibility...\n");
    }

    // WriteStringToGdbConsole("Successfully loaded memory map of ChibiOS/RT target \n");

    /* Currently, we have the inherent assumption that all address pointers
     * are 32 bit wide. */
    if (signature->ch_ptrsize != sizeof(uint32_t)) {
        WriteStringToGdbConsole("ChibiOS/RT target memory signature claims an address "
                  "width unequal to 32 bits!\n");
        free(signature);
        return FALSE;
    }

    chibios_signature = signature;
    return TRUE;

errfree:
    /* Error reading the ChibiOS memory structure */
    free(signature);
    chibios_signature = 0;
    return FALSE;
}

#define FPU_CPACR	0xE000ED88
static int chibios_update_stacking()
{
	/* Sometimes the stacking can not be determined only by looking at the
	 * target name but only a runtime.
	 *
	 * For example, this is the case for Cortex-M4 targets and ChibiOS which
	 * only stack the FPU registers if it is enabled during ChibiOS build.
	 *
	 * Terminating which stacking is used is target depending.
	 *
	 * Assumptions:
	 *  - Once ChibiOS is actually initialized, the stacking is fixed.
	 *  - During startup code, the FPU might not be initialized and the
	 *    detection might fail.
	 *  - Since no threads are running during startup, the problem is solved
	 *    by delaying stacking detection until there are more threads
	 *    available than the current execution. In which case
	 *    chibios_get_thread_reg_list is called.
	 */

	if (!chibios_signature)
		return FALSE;

	/* Check for armv7m with *enabled* FPU, i.e. a Cortex-M4  */
    if (hasFPURegisters()) {
        /* Found ARM v7m target which includes a FPU */
        uint32_t cpacr;

        cpacr = Platform_MemRead32((void*)(uint64_t)(FPU_CPACR));

        /* Check if CP10 and CP11 are set to full access.
            * In ChibiOS this is done in ResetHandler() in crt0.c */
        if (cpacr & 0x00F00000) {
            WriteStringToGdbConsole("\nEnabled FPU detected.\n");
            stacking_info = &rtos_chibios_arm_v7m_stacking_w_fpu;
            return TRUE;
        }
    }

    /* Found ARM v7m target with no or disabled FPU */
    stacking_info = &rtos_chibios_arm_v7m_stacking;
    return TRUE;
}

static int chibios_update_threads()
{
    int tasks_found = 0;
    int symbols = TRUE;
    uint8_t i;
    for (i=0; i<ARRAY_SIZE(chibios_symbol_list); i++) {
        if (chibios_symbol_list[i].value == 0) {
            symbols=FALSE;
        }
    }
    if (!symbols) {
        WriteStringToGdbConsole("No symbols for ChibiOS\n");
        return FALSE;
    }
    
    /* Update the memory signature saved in the target memory */
    if (!chibios_signature) {
        if (!chibios_update_memory_signature()) {
            WriteStringToGdbConsole("Reading the memory signature of ChibiOS/RT failed\n");
            return FALSE;
        }
    }

    /* ChibiOS does not save the current thread count. We have to first
     * parse the double linked thread list to check for errors and the number of
     * threads. */
    const uint32_t rlist = chibios_symbol_list[CHIBIOS_VAL_CH].value + CH_RLIST_OFFSET;
    const struct chibios_chdebug *signature = chibios_signature;
    uint32_t current;
    uint32_t previous;
    uint32_t older;
    uint32_t rtos_valid = 1;
    
    current = rlist;
    previous = rlist;
    while (1) {
        current = Platform_MemRead32((void*)(uint64_t)(current + signature->cf_off_newer));

        /* Could be NULL if the kernel is not initialized yet or if the
         * registry is corrupted. */
        if (current == 0) {
            WriteStringToGdbConsole("ChibiOS registry integrity check failed, NULL pointer\n");

            rtos_valid = 0;
            break;
        }
        /* Fetch previous thread in the list as a integrity check. */
        older = Platform_MemRead32((void*)(uint64_t)(current + signature->cf_off_older));
        if ((older == 0) || (older != previous)) {
            WriteStringToGdbConsole("ChibiOS registry integrity check failed, "
                        "double linked list violation\n");
            rtos_valid = 0;
            break;
        }
        /* Check for full iteration of the linked list. */
        if (current == rlist)
            break;
        tasks_found++;
        previous = current;
    }

	if (!rtos_valid) {
		/* No RTOS, there is always at least the current execution, though */
		WriteStringToGdbConsole("Only showing current execution because of a broken "
				"ChibiOS thread registry.\n");

		const char tmp_thread_name[] = "Current Execution";
		const char tmp_thread_extra_info[] = "No RTOS thread";

		thread_details[0].threadid = 1;
		thread_details[0].exists = 1;

		strcpy(thread_details[0].extra_info_str, tmp_thread_extra_info);

		strcpy(thread_details[0].thread_name_str, tmp_thread_name);

	    current_thread = 1;
		thread_count = 1;
		return TRUE;
	}
    thread_count = tasks_found;
    /* Loop through linked list. */
    size_t thd_idx;
    for (thd_idx=0; thd_idx<thread_count && thd_idx<ARRAY_SIZE(thread_details); thd_idx++) {
        uint32_t name_ptr = 0;
        char tmp_str[CHIBIOS_THREAD_NAME_STR_SIZE];

        current = Platform_MemRead32((void*)(uint64_t)(current + signature->cf_off_newer));

        /* Check for full iteration of the linked list. */
        if (current == rlist) {
            break;
        }

        /* Save the thread pointer */
        thread_details[thd_idx].threadid = current;

        /* read the name pointer */
        name_ptr = Platform_MemRead32((void*)(uint64_t)(current + signature->cf_off_name));

        /* Read the thread name */
        read_address_to_mem(&tmp_str, name_ptr, CHIBIOS_THREAD_NAME_STR_SIZE);
        tmp_str[CHIBIOS_THREAD_NAME_STR_SIZE - 1] = '\x00';

        if (tmp_str[0] == '\x00')
            strcpy(tmp_str, "No Name");
        strcpy(thread_details[thd_idx].thread_name_str, tmp_str);

        /* State info */
        uint8_t thread_state;
        const char *state_desc;

        thread_state = Platform_MemRead8((void*)(uint64_t)(current + signature->cf_off_state));

        if (thread_state < CHIBIOS_NUM_STATES)
            state_desc = chibios_thread_states[thread_state];
        else
            state_desc = "Unknown";

        snprintf(thread_details[thd_idx].extra_info_str,
            sizeof(thread_details[thd_idx].extra_info_str), "Name: %s State: %s", 
                thread_details[thd_idx].thread_name_str, state_desc);

        thread_details[thd_idx].exists = 1;
    }

    uint32_t current_thrd;
    /* NOTE: By design, cf_off_name equals readylist_current_offset */
    current_thrd = Platform_MemRead32((void*)(uint64_t)(rlist + signature->cf_off_name));

    current_thread = current_thrd;

    return TRUE;
}

static uint32_t current_thread_idx = 0;
uint32_t Platform_RtosGetFirstThreadId()
{
    if (chibios_update_threads()) {
        current_thread_idx = 1;
        return thread_details[0].threadid;
    }
    return 0;
}

uint32_t Platform_RtosGetNextThreadId()
{
    if (current_thread_idx == 0) {
        // need to fetch first thread first
        return 0;
    }
    if (current_thread_idx < thread_count) {
        return thread_details[current_thread_idx++].threadid;
    }
    return 0;
}

const char* Platform_RtosGetExtraThreadInfo(uint32_t thread_id)
{
    int i;
    for (i=0; i<thread_count; i++) {
        if (thread_details[i].threadid == thread_id) {
            return thread_details[i].extra_info_str;
        }
    }
    return NULL;
}

int Platform_RtosGetThreadContext(Buffer* pBuffer, uint32_t thread_id)
{
    uint32_t stack_ptr = 0;

    if ((thread_id == 0) || !chibios_signature)
        return FALSE;

    /* Update stacking if it can only be determined from runtime information */
    if ((stacking_info == 0) &&
        (!chibios_update_stacking())) {
        WriteStringToGdbConsole("Failed to determine exact stacking\n");
        return FALSE;
    }

    /* Read the stack pointer */
    stack_ptr = Platform_MemRead32((void*)(uint64_t)(thread_id + chibios_signature->cf_off_ctx));

    return rtos_generic_stack_read(pBuffer, stacking_info, (int64_t)stack_ptr);
}
