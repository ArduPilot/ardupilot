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
/* Routines to expose the Cortex-M functionality to the mri debugger. */
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <platforms.h>
#include <gdb_console.h>
#include "debug_cm3.h"
#include "armv7-m.h"

/* Disable any macro used for errno and use the int global instead. */
#undef errno
extern int errno;

/* Fake stack used when task encounters stacking/unstacking fault. */
const uint32_t  __mriCortexMFakeStack[8] = { 0xDEADDEAD, 0xDEADDEAD, 0xDEADDEAD, 0xDEADDEAD,
                                             0xDEADDEAD, 0xDEADDEAD, 0xDEADDEAD, 0xDEADDEAD };
CortexMState    __mriCortexMState;

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
static const char g_targetXml[] = 
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
    "<reg name=\"primask\" bitsize=\"32\" regnum=\"28\"/>\n"
    "<reg name=\"basepri\" bitsize=\"32\" regnum=\"29\"/>\n"
    "<reg name=\"faultmask\" bitsize=\"32\" regnum=\"30\"/>\n"
    "<reg name=\"control\" bitsize=\"32\" regnum=\"31\"/>\n"
    "</feature>\n"
#if MRI_DEVICE_HAS_FPU
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
#endif
    "</target>\n";

/* Macro to provide index for specified register in the SContext structure. */
#define CONTEXT_MEMBER_INDEX(MEMBER) (offsetof(Context, MEMBER)/sizeof(uint32_t))


/* Reference this handler in the ASM module to make sure that it gets linked in. */
void __mriExceptionHandler(void);


static void clearState(void);
static void configureDWTandFPB(void);
static void defaultSvcAndSysTickInterruptsToPriority1(void);
void __mriCortexMInit(Token* pParameterTokens)
{

    /* Reference routine in ASM module to make sure that is gets linked in. */
    void (* volatile dummyReference)(void) = __mriExceptionHandler;
    (void)dummyReference;
    (void)pParameterTokens;

    clearState();
    configureDWTandFPB();
    defaultSvcAndSysTickInterruptsToPriority1();
    Platform_DisableSingleStep();
    clearMonitorPending();
    enableDebugMonitorAtPriority0();
}

static void clearState(void)
{
    memset(&__mriCortexMState, 0, sizeof(__mriCortexMState));
}

static void configureDWTandFPB(void)
{
    enableDWTandITM();
    initDWT();
    initFPB();
}

static void defaultSvcAndSysTickInterruptsToPriority1(void)
{
    NVIC_SetPriority(SVCall_IRQn, 1);
    NVIC_SetPriority(PendSV_IRQn, 1);
    NVIC_SetPriority(SysTick_IRQn, 1);
}


static void clearSingleSteppingFlag(void);
void Platform_DisableSingleStep(void)
{
    disableSingleStep();
    clearSingleSteppingFlag();
}

static void clearSingleSteppingFlag(void)
{
    __mriCortexMState.flags &= ~CORTEXM_FLAGS_SINGLE_STEPPING;
}


static int      doesPCPointToSVCInstruction(void);
static void     setHardwareBreakpointOnSvcHandler(void);
static uint32_t getNvicVector(IRQn_Type irq);
static void     setSvcStepFlag(void);
static void     setSingleSteppingFlag(void);
static void     setSingleSteppingFlag(void);
static void     recordCurrentBasePriorityAndSwitchToPriority1(void);
static int      doesPCPointToBASEPRIUpdateInstruction(void);
static uint16_t getFirstHalfWordOfCurrentInstruction(void);
static uint16_t getSecondHalfWordOfCurrentInstruction(void);
static uint16_t throwingMemRead16(uint32_t address);
static int      isFirstHalfWordOfMSR(uint16_t instructionHalfWord0);
static int      isSecondHalfWordOfMSRModifyingBASEPRI(uint16_t instructionHalfWord1);
static int      isSecondHalfWordOfMSR_BASEPRI(uint16_t instructionHalfWord1);
static int      isSecondHalfWordOfMSR_BASEPRI_MAX(uint16_t instructionHalfWord1);
static void     recordCurrentBasePriority(void);
static void     setRestoreBasePriorityFlag(void);
static uint32_t calculateBasePriorityForThisCPU(uint32_t basePriority);
void Platform_EnableSingleStep(void)
{
    if (!doesPCPointToSVCInstruction())
    {
        setSingleSteppingFlag();
        recordCurrentBasePriorityAndSwitchToPriority1();
        enableSingleStep();
        return;
    }
    
    __try
    {
        __throwing_func( setHardwareBreakpointOnSvcHandler() );
        setSvcStepFlag();
    }
    __catch
    {
        /* Failed to set hardware breakpoint so single step without modifying priority since the priority
           elevation leads SVC to escalate to Hard Fault. */
        clearExceptionCode();
        setSingleSteppingFlag();
        enableSingleStep();
    }
    return;
}

static int doesPCPointToSVCInstruction(void)
{
    static const uint16_t svcMachineCodeMask = 0xff00;
    static const uint16_t svcMachineCode = 0xdf00;
    uint16_t              instructionWord;
    
    __try
    {
        instructionWord = getFirstHalfWordOfCurrentInstruction();
    }
    __catch
    {
        clearExceptionCode();
        return 0;
    }
    
    return ((instructionWord & svcMachineCodeMask) == svcMachineCode);
}

static void setHardwareBreakpointOnSvcHandler(void)
{
    Platform_SetHardwareBreakpoint(getNvicVector(SVCall_IRQn) & ~1, 2);
}

static uint32_t getNvicVector(IRQn_Type irq)
{
    const uint32_t           nvicBaseVectorOffset = 16;
    volatile const uint32_t* pVectors = (volatile const uint32_t*)SCB->VTOR;
    return pVectors[irq + nvicBaseVectorOffset];
}

static void setSvcStepFlag(void)
{
    __mriCortexMState.flags |= CORTEXM_FLAGS_SVC_STEP;
}

static void setSingleSteppingFlag(void)
{
    __mriCortexMState.flags |= CORTEXM_FLAGS_SINGLE_STEPPING;
}

static void recordCurrentBasePriorityAndSwitchToPriority1(void)
{
    if (!doesPCPointToBASEPRIUpdateInstruction())
        recordCurrentBasePriority();
    __set_BASEPRI(calculateBasePriorityForThisCPU(1));
}

static int doesPCPointToBASEPRIUpdateInstruction(void)
{
    uint16_t firstWord = 0;
    uint16_t secondWord = 0;
    
    __try
    {
        __throwing_func( firstWord = getFirstHalfWordOfCurrentInstruction() );
        __throwing_func( secondWord = getSecondHalfWordOfCurrentInstruction() );
    }
    __catch
    {
        clearExceptionCode();
        return 0;
    }
    
    return isFirstHalfWordOfMSR(firstWord) && isSecondHalfWordOfMSRModifyingBASEPRI(secondWord);
}

static uint16_t getFirstHalfWordOfCurrentInstruction(void)
{
    return throwingMemRead16(__mriCortexMState.context.PC);
}

static uint16_t getSecondHalfWordOfCurrentInstruction(void)
{
    return throwingMemRead16(__mriCortexMState.context.PC + sizeof(uint16_t));
}

static uint16_t throwingMemRead16(uint32_t address)
{
    uint16_t instructionWord = Platform_MemRead16((const uint16_t*)address);
    if (Platform_WasMemoryFaultEncountered())
        __throw_and_return(memFaultException, 0);
    return instructionWord;
}

static int isFirstHalfWordOfMSR(uint16_t instructionHalfWord0)
{
    static const unsigned short MSRMachineCode = 0xF380;
    static const unsigned short MSRMachineCodeMask = 0xFFF0;

    return MSRMachineCode == (instructionHalfWord0 & MSRMachineCodeMask);
}

static int isSecondHalfWordOfMSRModifyingBASEPRI(uint16_t instructionHalfWord1)
{
    return isSecondHalfWordOfMSR_BASEPRI(instructionHalfWord1) ||
           isSecondHalfWordOfMSR_BASEPRI_MAX(instructionHalfWord1);
}

static int isSecondHalfWordOfMSR_BASEPRI(uint16_t instructionHalfWord1)
{
    static const unsigned short BASEPRIMachineCode = 0x8811;

    return instructionHalfWord1 == BASEPRIMachineCode;
}

static int isSecondHalfWordOfMSR_BASEPRI_MAX(uint16_t instructionHalfWord1)
{
    static const unsigned short BASEPRI_MAXMachineCode = 0x8812;

    return instructionHalfWord1 == BASEPRI_MAXMachineCode;
}

static void recordCurrentBasePriority(void)
{
    __mriCortexMState.originalBasePriority = __get_BASEPRI();
    setRestoreBasePriorityFlag();
}

static void setRestoreBasePriorityFlag(void)
{
    __mriCortexMState.flags |= CORTEXM_FLAGS_RESTORE_BASEPRI;
}

static uint32_t calculateBasePriorityForThisCPU(uint32_t basePriority)
{
    /* Different Cortex-M3 chips support different number of bits in the priority register. */
    return ((basePriority << (8 - __NVIC_PRIO_BITS)) & 0xff);
}


int Platform_IsSingleStepping(void)
{
    return __mriCortexMState.flags & CORTEXM_FLAGS_SINGLE_STEPPING;
}


char* Platform_GetPacketBuffer(void)
{
    return __mriCortexMState.packetBuffer;
}


uint32_t Platform_GetPacketBufferSize(void)
{
    return sizeof(__mriCortexMState.packetBuffer);
}


static uint8_t  determineCauseOfDebugEvent(void);
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
        return determineCauseOfDebugEvent();
    case 21:
    case 22:
    case 23:
    case 24:
        /* UART* */
        return SIGINT;
    default:
        /* NOTE: Catch all signal will be SIGSTOP. */
        return SIGSTOP;
    }
}

static uint8_t determineCauseOfDebugEvent(void)
{
    static struct
    {
        uint32_t        statusBit;
        unsigned char   signalToReturn;
    } const debugEventToSignalMap[] =
    {
        {SCB_DFSR_EXTERNAL, SIGSTOP},
        {SCB_DFSR_DWTTRAP, SIGTRAP},
        {SCB_DFSR_BKPT, SIGTRAP},
        {SCB_DFSR_HALTED, SIGTRAP}
    };
    uint32_t debugFaultStatus = SCB->DFSR;
    size_t   i;
    
    for (i = 0 ; i < sizeof(debugEventToSignalMap)/sizeof(debugEventToSignalMap[0]) ; i++)
    {
        if (debugFaultStatus & debugEventToSignalMap[i].statusBit)
        {
            SCB->DFSR = debugEventToSignalMap[i].statusBit;
            return debugEventToSignalMap[i].signalToReturn;
        }
    }
    
    /* NOTE: Default catch all signal is SIGSTOP. */
    return SIGSTOP;
}


static void displayHardFaultCauseToGdbConsole(void);
static void displayMemFaultCauseToGdbConsole(void);
static void displayBusFaultCauseToGdbConsole(void);
static void displayUsageFaultCauseToGdbConsole(void);
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
    uint32_t              hardFaultStatusRegister = SCB->HFSR;
    
    WriteStringToGdbConsole("\n**Hard Fault**");
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
    uint32_t              memManageFaultStatusRegister = SCB->CFSR & 0xFF;
    
    /* Check to make sure that there is a memory fault to display. */
    if (memManageFaultStatusRegister == 0)
        return;
    
    WriteStringToGdbConsole("\n**MPU Fault**");
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(memManageFaultStatusRegister);
    
    if (memManageFaultStatusRegister & MMARValidBit)
    {
        WriteStringToGdbConsole("\n    Fault Address: ");
        WriteHexValueToGdbConsole(SCB->MMFAR);
    }
    if (memManageFaultStatusRegister & FPLazyStatePreservationBit)
        WriteStringToGdbConsole("\n    FP Lazy Preservation");

    if (memManageFaultStatusRegister & stackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Stacking Error w/ SP = ");
        WriteHexValueToGdbConsole(__mriCortexMState.taskSP);
    }
    if (memManageFaultStatusRegister & unstackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Unstacking Error w/ SP = ");
        WriteHexValueToGdbConsole(__mriCortexMState.taskSP);
    }
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
    uint32_t              busFaultStatusRegister = (SCB->CFSR >> 8) & 0xFF;
    
    /* Check to make sure that there is a bus fault to display. */
    if (busFaultStatusRegister == 0)
        return;
    
    WriteStringToGdbConsole("\n**Bus Fault**");
    WriteStringToGdbConsole("\n  Status Register: ");
    WriteHexValueToGdbConsole(busFaultStatusRegister);
    
    if (busFaultStatusRegister & BFARValidBit)
    {
        WriteStringToGdbConsole("\n    Fault Address: ");
        WriteHexValueToGdbConsole(SCB->BFAR);
    }
    if (busFaultStatusRegister & FPLazyStatePreservationBit)
        WriteStringToGdbConsole("\n    FP Lazy Preservation");

    if (busFaultStatusRegister & stackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Stacking Error w/ SP = ");
        WriteHexValueToGdbConsole(__mriCortexMState.taskSP);
    }
    if (busFaultStatusRegister & unstackingErrorBit)
    {
        WriteStringToGdbConsole("\n    Unstacking Error w/ SP = ");
        WriteHexValueToGdbConsole(__mriCortexMState.taskSP);
    }
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
    uint32_t              usageFaultStatusRegister = SCB->CFSR >> 16;
    
    /* Make sure that there is a usage fault to display. */
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


static void     clearMemoryFaultFlag(void);
static void     configureMpuToAccessAllMemoryWithNoCaching(void);
static void     saveOriginalMpuConfiguration(void);
static void     configureHighestMpuRegionToAccessAllMemoryWithNoCaching(void);
static void     cleanupIfSingleStepping(void);
static void     restoreBasePriorityIfNeeded(void);
static uint32_t shouldRestoreBasePriority(void);
static void     clearRestoreBasePriorityFlag(void);
static void     removeHardwareBreakpointOnSvcHandlerIfNeeded(void);
static int      shouldRemoveHardwareBreakpointOnSvcHandler(void);
static void     clearSvcStepFlag(void);
static void     clearHardwareBreakpointOnSvcHandler(void);
void Platform_EnteringDebugger(void)
{
    clearMemoryFaultFlag();
    __mriCortexMState.originalPC = __mriCortexMState.context.PC;
    configureMpuToAccessAllMemoryWithNoCaching();
    cleanupIfSingleStepping();
}

static void clearMemoryFaultFlag(void)
{
    __mriCortexMState.flags &= ~CORTEXM_FLAGS_FAULT_DURING_DEBUG;
}

static void configureMpuToAccessAllMemoryWithNoCaching(void)
{
    saveOriginalMpuConfiguration();
    disableMPU();
    configureHighestMpuRegionToAccessAllMemoryWithNoCaching();    
    enableMPUWithHardAndNMIFaults();
}

static void saveOriginalMpuConfiguration(void)
{
    __mriCortexMState.originalMPUControlValue = getMPUControlValue();
    __mriCortexMState.originalMPURegionNumber = getCurrentMPURegionNumber();
    prepareToAccessMPURegion(getHighestMPUDataRegionIndex());
    __mriCortexMState.originalMPURegionAddress = getMPURegionAddress();
    __mriCortexMState.originalMPURegionAttributesAndSize = getMPURegionAttributeAndSize();
}

static void configureHighestMpuRegionToAccessAllMemoryWithNoCaching(void)
{
    static const uint32_t regionToStartAtAddress0 = 0U;
    static const uint32_t regionReadWrite = 1  << MPU_RASR_AP_SHIFT;
    static const uint32_t regionSizeAt4GB = 31 << MPU_RASR_SIZE_SHIFT; /* 4GB = 2^(31+1) */
    static const uint32_t regionEnable    = MPU_RASR_ENABLE;
    static const uint32_t regionSizeAndAttributes = regionReadWrite | regionSizeAt4GB | regionEnable;
    
    prepareToAccessMPURegion(getHighestMPUDataRegionIndex());
    setMPURegionAddress(regionToStartAtAddress0);
    setMPURegionAttributeAndSize(regionSizeAndAttributes);
}

static void cleanupIfSingleStepping(void)
{
    restoreBasePriorityIfNeeded();
    removeHardwareBreakpointOnSvcHandlerIfNeeded();
    Platform_DisableSingleStep();
}

static void restoreBasePriorityIfNeeded(void)
{
    if (shouldRestoreBasePriority())
    {
        clearRestoreBasePriorityFlag();
        __set_BASEPRI(__mriCortexMState.originalBasePriority);
        __mriCortexMState.originalBasePriority = 0;
    }
}

static uint32_t shouldRestoreBasePriority(void)
{
    return __mriCortexMState.flags & CORTEXM_FLAGS_RESTORE_BASEPRI;
}

static void clearRestoreBasePriorityFlag(void)
{
    __mriCortexMState.flags &= ~CORTEXM_FLAGS_RESTORE_BASEPRI;
}

static void removeHardwareBreakpointOnSvcHandlerIfNeeded(void)
{
    if (shouldRemoveHardwareBreakpointOnSvcHandler())
    {
        clearSvcStepFlag();
        clearHardwareBreakpointOnSvcHandler();
    }
}

static int shouldRemoveHardwareBreakpointOnSvcHandler(void)
{
    return __mriCortexMState.flags & CORTEXM_FLAGS_SVC_STEP;
}

static void clearSvcStepFlag(void)
{
    __mriCortexMState.flags &= ~CORTEXM_FLAGS_SVC_STEP;
}

static void clearHardwareBreakpointOnSvcHandler(void)
{
    Platform_ClearHardwareBreakpoint(getNvicVector(SVCall_IRQn) & ~1, 2);
}


static void restoreMPUConfiguration(void);
static void checkStack(void);
void Platform_LeavingDebugger(void)
{
    restoreMPUConfiguration();
    checkStack();
    clearMonitorPending();
}

static void restoreMPUConfiguration(void)
{
    disableMPU();
    prepareToAccessMPURegion(getHighestMPUDataRegionIndex());
    setMPURegionAddress(__mriCortexMState.originalMPURegionAddress);
    setMPURegionAttributeAndSize(__mriCortexMState.originalMPURegionAttributesAndSize);
    prepareToAccessMPURegion(__mriCortexMState.originalMPURegionNumber);
    setMPUControlValue(__mriCortexMState.originalMPUControlValue);
}

static void checkStack(void)
{
    uint32_t* pCurr = (uint32_t*)__mriCortexMState.debuggerStack;
    uint8_t*  pEnd = (uint8_t*)__mriCortexMState.debuggerStack + sizeof(__mriCortexMState.debuggerStack);
    int       spaceUsed;
    
    while ((uint8_t*)pCurr < pEnd && *pCurr == CORTEXM_DEBUGGER_STACK_FILL)
        pCurr++;

    spaceUsed = pEnd - (uint8_t*)pCurr;
    if (spaceUsed > __mriCortexMState.maxStackUsed)
        __mriCortexMState.maxStackUsed = spaceUsed;
}


uint32_t Platform_GetProgramCounter(void)
{
    return __mriCortexMState.context.PC;
}


void Platform_SetProgramCounter(uint32_t newPC)
{
    __mriCortexMState.context.PC = newPC;
}


static int isInstruction32Bit(uint16_t firstWordOfInstruction);
void Platform_AdvanceProgramCounterToNextInstruction(void)
{
    uint16_t  firstWordOfCurrentInstruction;
    
    __try
    {
        firstWordOfCurrentInstruction = getFirstHalfWordOfCurrentInstruction();
    }
    __catch
    {
        /* Will get here if PC isn't pointing to valid memory so don't bother to advance. */
        clearExceptionCode();
        return;
    }
    
    if (isInstruction32Bit(firstWordOfCurrentInstruction))
    {
        /* 32-bit Instruction. */
        __mriCortexMState.context.PC += 4;
    }
    else
    {
        /* 16-bit Instruction. */
        __mriCortexMState.context.PC += 2;
    }
}

static int isInstruction32Bit(uint16_t firstWordOfInstruction)
{
    uint16_t maskedOffUpper5BitsOfWord = firstWordOfInstruction & 0xF800;
    
    /* 32-bit instructions start with 0b11101, 0b11110, 0b11111 according to page A5-152 of the 
       ARMv7-M Architecture Manual. */
    return  (maskedOffUpper5BitsOfWord == 0xE800 ||
             maskedOffUpper5BitsOfWord == 0xF000 ||
             maskedOffUpper5BitsOfWord == 0xF800);
}


int Platform_WasProgramCounterModifiedByUser(void)
{
    return __mriCortexMState.context.PC != __mriCortexMState.originalPC;
}


static int isInstructionMbedSemihostBreakpoint(uint16_t instruction);
static int isInstructionNewlibSemihostBreakpoint(uint16_t instruction);
static int isInstructionHardcodedBreakpoint(uint16_t instruction);
PlatformInstructionType Platform_TypeOfCurrentInstruction(void)
{
    uint16_t currentInstruction;
    
    __try
    {
        currentInstruction = getFirstHalfWordOfCurrentInstruction();
    }
    __catch
    {
        /* Will get here if PC isn't pointing to valid memory so treat as other. */
        clearExceptionCode();
        return MRI_PLATFORM_INSTRUCTION_OTHER;
    }
    
    if (isInstructionMbedSemihostBreakpoint(currentInstruction))
        return MRI_PLATFORM_INSTRUCTION_MBED_SEMIHOST_CALL;
    else if (isInstructionNewlibSemihostBreakpoint(currentInstruction))
        return MRI_PLATFORM_INSTRUCTION_NEWLIB_SEMIHOST_CALL;
    else if (isInstructionHardcodedBreakpoint(currentInstruction))
        return MRI_PLATFORM_INSTRUCTION_HARDCODED_BREAKPOINT;
    else
        return MRI_PLATFORM_INSTRUCTION_OTHER;
}

static int isInstructionMbedSemihostBreakpoint(uint16_t instruction)
{
    static const uint16_t mbedSemihostBreakpointMachineCode = 0xbeab;

    return mbedSemihostBreakpointMachineCode == instruction;
}

static int isInstructionNewlibSemihostBreakpoint(uint16_t instruction)
{
    static const uint16_t newlibSemihostBreakpointMachineCode = 0xbeff;

    return (newlibSemihostBreakpointMachineCode == instruction);
}

static int isInstructionHardcodedBreakpoint(uint16_t instruction)
{
    static const uint16_t hardCodedBreakpointMachineCode = 0xbe00;

    return (hardCodedBreakpointMachineCode == instruction);
}


PlatformSemihostParameters Platform_GetSemihostCallParameters(void)
{
    PlatformSemihostParameters parameters;
    
    parameters.parameter1 = __mriCortexMState.context.R0;
    parameters.parameter2 = __mriCortexMState.context.R1;
    parameters.parameter3 = __mriCortexMState.context.R2;
    parameters.parameter4 = __mriCortexMState.context.R3;
    
    return parameters;
}


void Platform_SetSemihostCallReturnAndErrnoValues(int returnValue, int err)
{
    __mriCortexMState.context.R0 = returnValue;
    if (returnValue < 0)
        errno = err;
}


int Platform_WasMemoryFaultEncountered(void)
{
    int wasFaultEncountered;
    
    __DSB();
    wasFaultEncountered = __mriCortexMState.flags & CORTEXM_FLAGS_FAULT_DURING_DEBUG;
    clearMemoryFaultFlag();
    
    return wasFaultEncountered;    
}


static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint32_t registerValue);
static void writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount);
void Platform_WriteTResponseRegistersToBuffer(Buffer* pBuffer)
{
    sendRegisterForTResponse(pBuffer, CONTEXT_MEMBER_INDEX(R7), __mriCortexMState.context.R7);
    sendRegisterForTResponse(pBuffer, CONTEXT_MEMBER_INDEX(SP), __mriCortexMState.context.SP);
    sendRegisterForTResponse(pBuffer, CONTEXT_MEMBER_INDEX(LR), __mriCortexMState.context.LR);
    sendRegisterForTResponse(pBuffer, CONTEXT_MEMBER_INDEX(PC), __mriCortexMState.context.PC);
}

static void sendRegisterForTResponse(Buffer* pBuffer, uint8_t registerOffset, uint32_t registerValue)
{
    Buffer_WriteByteAsHex(pBuffer, registerOffset);
    Buffer_WriteChar(pBuffer, ':');
    writeBytesToBufferAsHex(pBuffer, &registerValue, sizeof(registerValue));
    Buffer_WriteChar(pBuffer, ';');
}

static void writeBytesToBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount)
{
    uint8_t* pByte = (uint8_t*)pBytes;
    size_t   i;
    
    for (i = 0 ; i < byteCount ; i++)
        Buffer_WriteByteAsHex(pBuffer, *pByte++);
}


void Platform_CopyContextToBuffer(Buffer* pBuffer)
{
    writeBytesToBufferAsHex(pBuffer, &__mriCortexMState.context, sizeof(__mriCortexMState.context));
}


static void readBytesFromBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount);
void Platform_CopyContextFromBuffer(Buffer* pBuffer)
{
    readBytesFromBufferAsHex(pBuffer, &__mriCortexMState.context, sizeof(__mriCortexMState.context));
}

static void readBytesFromBufferAsHex(Buffer* pBuffer, void* pBytes, size_t byteCount)
{
    uint8_t* pByte = (uint8_t*)pBytes;
    size_t   i;
    
    for (i = 0 ; i < byteCount; i++)
        *pByte++ = Buffer_ReadByteAsHex(pBuffer);
}


static int doesKindIndicate32BitInstruction(uint32_t kind);
void Platform_SetHardwareBreakpoint(uint32_t address, uint32_t kind)
{
    uint32_t* pFPBBreakpointComparator;
    int       is32BitInstruction;
    
    __try
        is32BitInstruction = doesKindIndicate32BitInstruction(kind);
    __catch
        __rethrow;
        
    pFPBBreakpointComparator = enableFPBBreakpoint(address, is32BitInstruction);
    if (!pFPBBreakpointComparator)
        __throw(exceededHardwareResourcesException);
}

static int doesKindIndicate32BitInstruction(uint32_t kind)
{
    switch (kind)
    {
    case 2:
        return 0;
    case 3:
    case 4:
        return 1;
    default:
        __throw_and_return(invalidArgumentException, -1);
    }
}


void Platform_ClearHardwareBreakpoint(uint32_t address, uint32_t kind)
{
    int       is32BitInstruction;
    
    __try
        is32BitInstruction = doesKindIndicate32BitInstruction(kind);
    __catch
        __rethrow;
        
    disableFPBBreakpointComparator(address, is32BitInstruction);
}


static uint32_t convertWatchpointTypeToCortexMType(PlatformWatchpointType type);
void Platform_SetHardwareWatchpoint(uint32_t address, uint32_t size, PlatformWatchpointType type)
{
    uint32_t       nativeType = convertWatchpointTypeToCortexMType(type);
    DWT_COMP_Type* pComparator;
    
    if (!isValidDWTComparatorSetting(address, size, nativeType))
        __throw(invalidArgumentException);
    
    pComparator = enableDWTWatchpoint(address, size, nativeType);
    if (!pComparator)
        __throw(exceededHardwareResourcesException);
}

static uint32_t convertWatchpointTypeToCortexMType(PlatformWatchpointType type)
{
    switch (type)
    {
    case MRI_PLATFORM_WRITE_WATCHPOINT:
        return DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE;
    case MRI_PLATFORM_READ_WATCHPOINT:
        return DWT_COMP_FUNCTION_FUNCTION_DATA_READ;
    case MRI_PLATFORM_READWRITE_WATCHPOINT:
        return DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE;
    default:
        return 0;
    }
}


void Platform_ClearHardwareWatchpoint(uint32_t address, uint32_t size, PlatformWatchpointType type)
{
    uint32_t nativeType = convertWatchpointTypeToCortexMType(type);
    
    if (!isValidDWTComparatorSetting(address, size, nativeType))
        __throw(invalidArgumentException);
    
    disableDWTWatchpoint(address, size, nativeType);
}

uint32_t Platform_GetTargetXmlSize(void)
{
    return sizeof(g_targetXml) - 1;
}


const char* Platform_GetTargetXml(void)
{
    return g_targetXml;
}
