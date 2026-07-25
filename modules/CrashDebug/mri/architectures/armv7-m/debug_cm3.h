/* Copyright 2015 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Declares registers, bit fields, and inline routines to utilize the debug hardware on the Cortex-M architecture. */
#ifndef _DEBUG_CM3_H_
#define _DEBUG_CM3_H_

#include <cmsis.h>
#include <stdio.h>
#include <try_catch.h>

/* Data Watchpoint and Trace Registers */
typedef struct
{
    /* Comparator register. */
    __IO uint32_t   COMP;
    /* Comparator Mask register. */
    __IO uint32_t   MASK;
    /* Comparator Function register. */
    __IO uint32_t   FUNCTION;
    /* Reserved 4 bytes to pad struct size out to 16 bytes. */
    __I  uint32_t   Reserved;
} DWT_COMP_Type;

/* Flash Patch and Breakpoint Registers */
typedef struct
{
    /* FlashPatch Control Register. */
    __IO uint32_t   CTRL;
    /* FlashPatch Remap Register. */
    __IO uint32_t   REMAP;
} FPB_Type;

/* Memory mapping of Cortex-M3 Debug Hardware */
#define DWT_COMP_BASE   (0xE0001020)
#define DWT_COMP_ARRAY  ((DWT_COMP_Type*) DWT_COMP_BASE)
#define FPB_BASE        (0xE0002000)
#define FPB_COMP_BASE   (0xE0002008)
#define FPB             ((FPB_Type*) FPB_BASE)
#define FPB_COMP_ARRAY  ((uint32_t*) FPB_COMP_BASE)

/* Debug Halting Control and Status Register Bits */
/*  Enable halt mode debug.  If set to 1 then JTAG debugging is being used. */
#define CoreDebug_DHCSR_C_DEBUGEN   (1 << 0)

/* Debug Exception and Monitor Control Registers Bits */
/*  Global enable for all DWT and ITM features. */
#define CoreDebug_DEMCR_TRCENA      (1 << 24)
/*  Monitor Single Step.  Set to 1 to single step instruction when exiting monitor. */
#define CoreDebug_DEMCR_MON_STEP    (1 << 18)
/* Monitor Pending.  Set to 1 to pend a monitor exception. */
#define CoreDebug_DEMCR_MON_PEND    (1 << 17)
/* Monitor Enable.  Set to 1 to enable the debug monitor exception. */
#define CoreDebug_DEMCR_MON_END     (1 << 16)

/* Debug Fault Status Register Bits.  Clear a bit by writing a 1 to it. */
/* Indicates that EDBGRQ was asserted. */
#define SCB_DFSR_EXTERNAL     (1 << 4)
/* Indicates that a vector catch was triggered. */
#define SCB_DFSR_VCATCH       (1 << 3)
/* Indicates that a DWT debug event was triggered. */
#define SCB_DFSR_DWTTRAP      (1 << 2)
/* Indicates a BKPT instruction or FPB match was encountered. */
#define SCB_DFSR_BKPT         (1 << 1)
/* Indicates that a single step has occurred. */
#define SCB_DFSR_HALTED       1

static __INLINE int isDebuggerAttached(void)
{
    return (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN);
}

static __INLINE void waitForDebuggerToDetach(uint32_t timeOut)
{
    while (timeOut-- > 0 && isDebuggerAttached())
    {
    }
    
    if (isDebuggerAttached())
        __throw(timeoutException);
}

static __INLINE void enableDebugMonitorAtPriority0(void)
{
    NVIC_SetPriority(DebugMonitor_IRQn, 0);
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_MON_END;
}

static __INLINE void enableDWTandITM(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA;
}

static __INLINE void disableSingleStep(void)
{
    CoreDebug->DEMCR &=  ~CoreDebug_DEMCR_MON_STEP;
}

static __INLINE void enableSingleStep(void)
{
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_MON_STEP;
}

static __INLINE void clearMonitorPending(void)
{
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_MON_PEND;
}


/* Data Watchpoint and Trace Comparator Function Bits. */
/*  Matched.  Read-only.  Set to 1 to indicate that this comparator has been matched.  Cleared on read. */
#define DWT_COMP_FUNCTION_MATCHED               (1 << 24)
/*  Data Address Linked Index 1. */
#define DWT_COMP_FUNCTION_DATAVADDR1            (0xF << 16)
/*  Data Address Linked Index 0. */
#define DWT_COMP_FUNCTION_DATAVADDR0            (0xF << 12)
/*  Selects size for data value matches. */
#define DWT_COMP_FUNCTION_DATAVSIZE_MASK        (3 << 10)
/*      Byte */
#define DWT_COMP_FUNCTION_DATAVSIZE_BYTE        (0 << 10)
/*      Halfword */
#define DWT_COMP_FUNCTION_DATAVSIZE_HALFWORD    (1 << 10)
/*      Word */
#define DWT_COMP_FUNCTION_DATAVSIZE_WORD        (2 << 10)
/*  Data Value Match.  Set to 0 for address compare and 1 for data value compare. */
#define DWT_COMP_FUNCTION_DATAVMATCH            (1 << 8)
/*  Cycle Count Match.  Set to 1 for enabling cycle count match and 0 otherwise.  Only valid on comparator 0. */
#define DWT_COMP_FUNCTION_CYCMATCH              (1 << 7)
/*  Enable Data Trace Address offset packets.  0 to disable. */
#define DWT_COMP_FUNCTION_EMITRANGE             (1 << 5)
/*  Selects action to be taken on match. */
#define DWT_COMP_FUNCTION_FUNCTION_MASK             0xF
/*      Disabled */
#define DWT_COMP_FUNCTION_FUNCTION_DISABLED         0x0
/*      Instruction Watchpoint */
#define DWT_COMP_FUNCTION_FUNCTION_INSTRUCTION      0x4
/*      Data Read Watchpoint */
#define DWT_COMP_FUNCTION_FUNCTION_DATA_READ        0x5
/*      Data Write Watchpoint */
#define DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE       0x6
/*      Data Read/Write Watchpoint */
#define DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE   0x7

/* DWT - Data Watchpoint Trace Routines */
static __INLINE uint32_t getDWTComparatorCount(void)
{
    return (DWT->CTRL >> 28);
}

static __INLINE void clearDWTComparator(DWT_COMP_Type* pComparatorStruct)
{
    pComparatorStruct->COMP = 0;
    pComparatorStruct->MASK = 0;
    pComparatorStruct->FUNCTION &= ~(DWT_COMP_FUNCTION_DATAVMATCH | 
                                     DWT_COMP_FUNCTION_CYCMATCH |
                                     DWT_COMP_FUNCTION_EMITRANGE |
                                     DWT_COMP_FUNCTION_FUNCTION_MASK);
}

static __INLINE void clearDWTComparators(void)
{
    DWT_COMP_Type*  pComparatorStruct = DWT_COMP_ARRAY;
    uint32_t        comparatorCount;
    uint32_t        i;
    
    comparatorCount = getDWTComparatorCount();
    for (i = 0 ; i < comparatorCount ; i++)
    {
        clearDWTComparator(pComparatorStruct);
        pComparatorStruct++;
    }
}

static __INLINE void initDWT(void)
{
    clearDWTComparators();
}

static __INLINE uint32_t maskOffDWTFunctionBits(uint32_t functionValue)
{
    return functionValue & (DWT_COMP_FUNCTION_DATAVADDR1 |
                            DWT_COMP_FUNCTION_DATAVADDR0 |
                            DWT_COMP_FUNCTION_DATAVSIZE_MASK |
                            DWT_COMP_FUNCTION_DATAVMATCH |
                            DWT_COMP_FUNCTION_CYCMATCH |
                            DWT_COMP_FUNCTION_EMITRANGE |
                            DWT_COMP_FUNCTION_FUNCTION_MASK);
                            
}

static __INLINE int doesDWTComparatorAddressMatch(DWT_COMP_Type* pComparator, uint32_t address)
{
    return pComparator->COMP == address;
}

static __INLINE uint32_t calculateLog2(uint32_t value)
{
    uint32_t log2 = 0;
    
    while (value > 1)
    {
        value >>= 1;
        log2++;
    }
    
    return log2;
}

static __INLINE int doesDWTComparatorMaskMatch(DWT_COMP_Type* pComparator, uint32_t size)
{
    return pComparator->MASK == calculateLog2(size);
}

static __INLINE int doesDWTComparatorFunctionMatch(DWT_COMP_Type* pComparator, uint32_t function)
{
    uint32_t importantFunctionBits = maskOffDWTFunctionBits(pComparator->FUNCTION);

    return importantFunctionBits == function;
}


static __INLINE int doesDWTComparatorMatch(DWT_COMP_Type* pComparator, 
                                           uint32_t       address, 
                                           uint32_t       size, 
                                           uint32_t       function)
{
    return doesDWTComparatorFunctionMatch(pComparator, function) &&
           doesDWTComparatorAddressMatch(pComparator, address) &&
           doesDWTComparatorMaskMatch(pComparator, size);
}

static __INLINE DWT_COMP_Type* findDWTComparator(uint32_t watchpointAddress, 
                                                 uint32_t watchpointSize, 
                                                 uint32_t watchpointType)
{
    DWT_COMP_Type* pCurrentComparator = DWT_COMP_ARRAY;
    uint32_t       comparatorCount;
    uint32_t       i;
    
    comparatorCount = getDWTComparatorCount();
    for (i = 0 ; i < comparatorCount ; i++)
    {
        if (doesDWTComparatorMatch(pCurrentComparator, watchpointAddress, watchpointSize, watchpointType))
            return pCurrentComparator;

        pCurrentComparator++;
    }
    
    /* Return NULL if no DWT comparator is already enabled for this watchpoint. */
    return NULL;
}

static __INLINE int isDWTComparatorFree(DWT_COMP_Type* pComparator)
{
    return (pComparator->FUNCTION & DWT_COMP_FUNCTION_FUNCTION_MASK) == DWT_COMP_FUNCTION_FUNCTION_DISABLED;
}

static __INLINE DWT_COMP_Type* findFreeDWTComparator(void)
{
    DWT_COMP_Type* pCurrentComparator = DWT_COMP_ARRAY;
    uint32_t       comparatorCount;
    uint32_t       i;
    
    comparatorCount = getDWTComparatorCount();
    for (i = 0 ; i < comparatorCount ; i++)
    {
        if (isDWTComparatorFree(pCurrentComparator))
        {
            return pCurrentComparator;
        }
        pCurrentComparator++;
    }
    
    /* Return NULL if there are no free DWT comparators. */
    return NULL;
}

static __INLINE int isPowerOf2(uint32_t value)
{
    return (value & (value - 1)) == 0;
}

static __INLINE int isAddressAlignedToSize(uint32_t address, uint32_t size)
{
    uint32_t addressMask = ~(size - 1);
    return address == (address & addressMask);
}

static __INLINE int isValidDWTComparatorSize(uint32_t watchpointSize)
{
    return isPowerOf2(watchpointSize);
}

static __INLINE int isValidDWTComparatorAddress(uint32_t watchpointAddress, uint32_t watchpointSize)
{
    return isAddressAlignedToSize(watchpointAddress, watchpointSize);
}

static __INLINE int isValidDWTComparatorType(uint32_t watchpointType)
{
    return (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_READ) ||
           (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_WRITE) ||
           (watchpointType == DWT_COMP_FUNCTION_FUNCTION_DATA_READWRITE);
}

static __INLINE int isValidDWTComparatorSetting(uint32_t watchpointAddress, 
                                                uint32_t watchpointSize, 
                                                uint32_t watchpointType)
{
    return isValidDWTComparatorSize(watchpointSize) &&
           isValidDWTComparatorAddress(watchpointAddress, watchpointSize) &&
           isValidDWTComparatorType(watchpointType);
}

static __INLINE int attemptToSetDWTComparatorMask(DWT_COMP_Type* pComparator, uint32_t watchpointSize)
{
    uint32_t maskBitCount;
    
    maskBitCount = calculateLog2(watchpointSize);
    pComparator->MASK = maskBitCount;

    /* Processor may limit number of bits to be masked off so check. */
    return pComparator->MASK == maskBitCount;
}

static __INLINE int attemptToSetDWTComparator(DWT_COMP_Type* pComparator,
                                              uint32_t       watchpointAddress,
                                              uint32_t       watchpointSize,
                                              uint32_t       watchpointType)
{
    if (!attemptToSetDWTComparatorMask(pComparator, watchpointSize))
        return 0;
    
    pComparator->COMP = watchpointAddress;
    pComparator->FUNCTION = watchpointType;
    return 1;
}

static __INLINE DWT_COMP_Type* enableDWTWatchpoint(uint32_t watchpointAddress, 
                                                   uint32_t watchpointSize, 
                                                   uint32_t watchpointType)
{
    DWT_COMP_Type* pComparator = NULL;
    
    pComparator = findDWTComparator(watchpointAddress, watchpointSize, watchpointType);
    if (pComparator)
    {
        /* This watchpoint has already been set so return a pointer to it. */
        return pComparator;
    }
    
    pComparator = findFreeDWTComparator();
    if (!pComparator)
    {
        /* There are no free comparators left. */
        return NULL;
    }
    
    if (!attemptToSetDWTComparator(pComparator, watchpointAddress, watchpointSize, watchpointType))
    {
        /* Failed set due to the size being larger than supported by CPU. */
        return NULL;
    }
    
    /* Successfully configured a free comparator for this watchpoint. */
    return pComparator;
}

static __INLINE DWT_COMP_Type* disableDWTWatchpoint(uint32_t watchpointAddress, 
                                                    uint32_t watchpointSize, 
                                                    uint32_t watchpointType)
{
    DWT_COMP_Type* pComparator = NULL;
    
    pComparator = findDWTComparator(watchpointAddress, watchpointSize, watchpointType);
    if (!pComparator)
    {
        /* This watchpoint not set so return NULL. */
        return NULL;
    }
    
    clearDWTComparator(pComparator);
    return pComparator;
}


/* FlashPatch Control Register Bits. */
/*  Most significant bits of number of instruction address comparators.  Read-only */
#define FP_CTRL_NUM_CODE_MSB_SHIFT  12
#define FP_CTRL_NUM_CODE_MSB_MASK   (0x7 << FP_CTRL_NUM_CODE_MSB_SHIFT)
/*  Least significant bits of number of instruction address comparators.  Read-only */
#define FP_CTRL_NUM_CODE_LSB_SHIFT  4
#define FP_CTRL_NUM_CODE_LSB_MASK   (0xF << FP_CTRL_NUM_CODE_LSB_SHIFT)
/*  Number of instruction literal address comparators.  Read only */
#define FP_CTRL_NUM_LIT_SHIFT       8
#define FP_CTRL_NUM_LIT_MASK        (0xF << FP_CTRL_NUM_LIT_SHIFT)
/*  This Key field must be set to 1 when writing or the write will be ignored. */
#define FP_CTRL_KEY                 (1 << 1)
/*  Enable bit for the FPB.  Set to 1 to enable FPB. */
#define FP_CTRL_ENABLE              1

/* FlashPatch Comparator Register Bits. */
/*  Defines the behaviour for code address comparators. */
#define FP_COMP_REPLACE_SHIFT       30
#define FP_COMP_REPLACE_MASK        (0x3U << FP_COMP_REPLACE_SHIFT)
/*      Remap to specified address in SRAM. */
#define FP_COMP_REPLACE_REMAP       (0x0U << FP_COMP_REPLACE_SHIFT)
/*      Breakpoint on lower halfword. */
#define FP_COMP_REPLACE_BREAK_LOWER (0x1U << FP_COMP_REPLACE_SHIFT)
/*      Breakpoint on upper halfword. */
#define FP_COMP_REPLACE_BREAK_UPPER (0x2U << FP_COMP_REPLACE_SHIFT)
/*      Breakpoint on word. */
#define FP_COMP_REPLACE_BREAK       (0x3U << FP_COMP_REPLACE_SHIFT)
/*  Specified bits 28:2 of the address to be use for match on this comparator. */
#define FP_COMP_COMP_SHIFT          2
#define FP_COMP_COMP_MASK           (0x07FFFFFF << FP_COMP_COMP_SHIFT)
/*  Enables this comparator.  Set to 1 to enable. */
#define FP_COMP_ENABLE              1

/* FPB - Flash Patch Breakpoint Routines. */
static __INLINE uint32_t getFPBCodeComparatorCount(void)
{
    uint32_t    controlValue = FPB->CTRL;
    return (((controlValue & FP_CTRL_NUM_CODE_MSB_MASK) >> 8) |
            ((controlValue & FP_CTRL_NUM_CODE_LSB_MASK) >> 4));
}

static __INLINE uint32_t getFPBLiteralComparatorCount(void)
{
    uint32_t    controlValue = FPB->CTRL;
    return ((controlValue & FP_CTRL_NUM_LIT_MASK) >> FP_CTRL_NUM_LIT_SHIFT);
}

static __INLINE void clearFPBComparator(uint32_t* pComparator)
{
    *pComparator = 0;
}

static __INLINE int isAddressInUpperHalfGig(uint32_t address)
{
    return (int)(address & 0xE0000000);
}

static __INLINE int isAddressOdd(uint32_t address)
{
    return (int)(address & 0x1);
}

static __INLINE int isBreakpointAddressInvalid(uint32_t breakpointAddress)
{
     return (isAddressInUpperHalfGig(breakpointAddress) || isAddressOdd(breakpointAddress));
}

static __INLINE int isAddressInUpperHalfword(uint32_t address)
{
    return (int)(address & 0x2);
}

static __INLINE uint32_t calculateFPBComparatorReplaceValue(uint32_t breakpointAddress, int32_t is32BitInstruction)
{
    if (is32BitInstruction)
        return FP_COMP_REPLACE_BREAK;
    else if (isAddressInUpperHalfword(breakpointAddress))
        return FP_COMP_REPLACE_BREAK_UPPER;
    else
        return FP_COMP_REPLACE_BREAK_LOWER;
}

static __INLINE uint32_t calculateFPBComparatorValue(uint32_t breakpointAddress, int32_t is32BitInstruction)
{
    uint32_t    comparatorValue;
    
    if (isBreakpointAddressInvalid(breakpointAddress))
    {
        /* Can only set a breakpoint on addresses where the upper 3-bits are all 0 (upper 0.5GB is off limits) and 
           the address is half-word aligned */
        return (uint32_t)~0U;
    }
    
    comparatorValue = (breakpointAddress & FP_COMP_COMP_MASK);
    comparatorValue |= FP_COMP_ENABLE;
    comparatorValue |= calculateFPBComparatorReplaceValue(breakpointAddress, is32BitInstruction);
    
    return comparatorValue;
}

static __INLINE uint32_t maskOffFPBComparatorReservedBits(uint32_t comparatorValue)
{
    return (comparatorValue & (FP_COMP_REPLACE_MASK | FP_COMP_COMP_MASK | FP_COMP_ENABLE));
}

static __INLINE int isFPBComparatorEnabled(uint32_t comparator)
{
    return (int)(comparator & FP_COMP_ENABLE);
}

static __INLINE uint32_t* findFPBBreakpointComparator(uint32_t breakpointAddress, int32_t is32BitInstruction)
{
    uint32_t*    pCurrentComparator = FPB_COMP_ARRAY;
    uint32_t     comparatorValueForThisBreakpoint;
    uint32_t     codeComparatorCount;
    uint32_t     i;
    
    comparatorValueForThisBreakpoint = calculateFPBComparatorValue(breakpointAddress, is32BitInstruction);
    codeComparatorCount = getFPBCodeComparatorCount();
    
    for (i = 0 ; i < codeComparatorCount ; i++)
    {
        uint32_t maskOffReservedBits;
        
        maskOffReservedBits = maskOffFPBComparatorReservedBits(*pCurrentComparator);
        if (comparatorValueForThisBreakpoint == maskOffReservedBits)
            return pCurrentComparator;

        pCurrentComparator++;
    }
    
    /* Return NULL if no FPB comparator is already enabled for this breakpoint. */
    return NULL;
}

static __INLINE uint32_t* findFreeFPBBreakpointComparator(void)
{
    uint32_t* pCurrentComparator = FPB_COMP_ARRAY;
    uint32_t  codeComparatorCount;
    uint32_t  i;
    
    codeComparatorCount = getFPBCodeComparatorCount();
    for (i = 0 ; i < codeComparatorCount ; i++)
    {
        if (!isFPBComparatorEnabled(*pCurrentComparator))
            return pCurrentComparator;

        pCurrentComparator++;
    }
    
    /* Return NULL if no FPB breakpoint comparators are free. */
    return NULL;
}

static __INLINE uint32_t* enableFPBBreakpoint(uint32_t breakpointAddress, int32_t is32BitInstruction)
{
    uint32_t* pExistingFPBBreakpoint;
    uint32_t* pFreeFPBBreakpointComparator;
    
    pExistingFPBBreakpoint = findFPBBreakpointComparator(breakpointAddress, is32BitInstruction);
    if (pExistingFPBBreakpoint)
    {
        /* This breakpoint is already set to just return pointer to existing comparator. */
        return pExistingFPBBreakpoint;
    }
    
    pFreeFPBBreakpointComparator = findFreeFPBBreakpointComparator();
    if (!pFreeFPBBreakpointComparator)
    {
        /* All FPB breakpoint comparator slots are used so return NULL as error indicator. */
        return NULL;
    }
    
    
    *pFreeFPBBreakpointComparator = calculateFPBComparatorValue(breakpointAddress, is32BitInstruction);
    return pFreeFPBBreakpointComparator;
}

static __INLINE uint32_t* disableFPBBreakpointComparator(uint32_t breakpointAddress, int32_t is32BitInstruction)
{
    uint32_t* pExistingFPBBreakpoint;
    
    pExistingFPBBreakpoint = findFPBBreakpointComparator(breakpointAddress, is32BitInstruction);
    if (pExistingFPBBreakpoint)
        clearFPBComparator(pExistingFPBBreakpoint);
 
    return pExistingFPBBreakpoint;
}

static __INLINE void clearFPBComparators(void)
{
    uint32_t* pCurrentComparator = FPB_COMP_ARRAY;
    uint32_t  codeComparatorCount;
    uint32_t  literalComparatorCount;
    uint32_t  totalComparatorCount;
    uint32_t  i;
    
    codeComparatorCount = getFPBCodeComparatorCount();
    literalComparatorCount = getFPBLiteralComparatorCount();
    totalComparatorCount = codeComparatorCount + literalComparatorCount;
    for (i = 0 ; i < totalComparatorCount ; i++)
    {
        clearFPBComparator(pCurrentComparator);
        pCurrentComparator++;
    }
}

static __INLINE void enableFPB(void)
{
    FPB->CTRL |= (FP_CTRL_KEY | FP_CTRL_ENABLE);
}

static __INLINE void initFPB(void)
{
    clearFPBComparators();
    enableFPB();
}


/* Memory Protection Unit Type Register Bits. */
/* Number of instruction regions supported by MPU.  0 for Cortex-M3 */
#define MPU_TYPE_IREGION_SHIFT      16
#define MPU_TYPE_IREGION_MASK       (0xFF << MPU_TYPE_IREGION_SHIFT)
/* Number of data regions supported by MPU. */
#define MPU_TYPE_DREGION_SHIFT      8
#define MPU_TYPE_DREGION_MASK       (0xFF << MPU_TYPE_DREGION_SHIFT)
/* Are instruction and data regions configured separately?  1 for yes and 0 otherwise. */
#define MPU_TYPE_SEPARATE           0x1

/* Memory Protection Unit Control Register Bits. */
/* Default memory map as background region for privileged access. 1 enables. */
#define MPU_CTRL_PRIVDEFENA         (1 << 2)
/* Hard fault and NMI exceptions to use MPU. 0 disables MPU for these handlers. */
#define MPU_CTRL_HFNMIENA           (1 << 1)
/* MPU Enable.  1 enables and disabled otherwise. */
#define MPU_CTRL_ENABLE             1

/* Memory Protection Unit Region Region Number Register Bits. */
#define MPU_RNR_REGION_MASK         0xFF

/* Memory Protection Unit Region Base Address Register Bits. */
/* Base address of this region. */
#define MPU_RBAR_ADDR_SHIFT         5
#define MPU_RBAR_ADDR_MASK          (0x7FFFFFF << MPU_RBAR_ADDR_SHIFT)
/* Are the region bits in this register valid or should RNR be used instead. */
#define MPU_RBAR_VALID              (1 << 4)
/* The region number.  Only used when MPU_RBAR_VALID is one. */
#define MPU_RBAR_REGION_MASK        0xF

/* Memory Protection Unit Region Attribute and Size Register Bits. */
/* eXecute Never bit.  1 means code can't execute from this region. */
#define MPU_RASR_XN                 (1 << 28)
/* Access permission bits. */
#define MPU_RASR_AP_SHIFT           24
#define MPU_RASR_AP_MASK            (0x7 << MPU_RASR_AP_SHIFT)
/* TEX, C, and B bits together determine memory type. */
#define MPU_RASR_TEX_SHIFT          19
#define MPU_RASR_TEX_MASK           (0x7 << MPU_RASR_TEX_SHIFT)
#define MPU_RASR_S                  (1 << 18)
#define MPU_RASR_C                  (1 << 17)
#define MPU_RASR_B                  (1 << 16)
/* Sub-region disable bits. */
#define MPU_RASR_SRD_SHIFT          8
#define MPU_RASR_SRD_MASK           (0xff << MPU_RASR_SRD_SHIFT)
/* Region size in 2^(value + 1) */
#define MPU_RASR_SIZE_SHIFT         1
#define MPU_RASR_SIZE_MASK          (0x1F << MPU_RASR_SIZE_SHIFT)
/* Region enable.  1 enables. */
#define MPU_RASR_ENABLE             1


/* MPU - Memory Protection Unit Routines. */
static __INLINE uint32_t getMPUDataRegionCount(void)
{
    return (MPU->TYPE & MPU_TYPE_DREGION_MASK) >> MPU_TYPE_DREGION_SHIFT;
}

static __INLINE uint32_t getHighestMPUDataRegionIndex(void)
{
    return getMPUDataRegionCount() - 1;
}

static __INLINE int isMPURegionNumberValid(uint32_t regionNumber)
{
    return regionNumber < getMPUDataRegionCount();
}

static __INLINE int isMPUNotPresent(void)
{
    return getMPUDataRegionCount() == 0;
}

static __INLINE uint32_t getMPUControlValue(void)
{
    if (isMPUNotPresent())
        return ~0U;

    return (MPU->CTRL);

}

static __INLINE void setMPUControlValue(uint32_t newControlValue)
{
    if (isMPUNotPresent())
        return;
    
    MPU->CTRL = newControlValue;
    __DSB();
    __ISB();
}

static __INLINE void disableMPU(void)
{
    if (isMPUNotPresent())
        return;
    
    MPU->CTRL &= ~MPU_CTRL_ENABLE;
    __DSB();
    __ISB();
}

static __INLINE void enableMPU(void)
{
    if (isMPUNotPresent())
        return;
    
    MPU->CTRL |= MPU_CTRL_ENABLE;
    __DSB();
    __ISB();
}

static __INLINE void enableMPUWithHardAndNMIFaults(void)
{
    if (isMPUNotPresent())
        return;
    
    MPU->CTRL |= MPU_CTRL_ENABLE | MPU_CTRL_HFNMIENA;
    __DSB();
    __ISB();
}

static __INLINE int prepareToAccessMPURegion(uint32_t regionNumber)
{
    if (!isMPURegionNumberValid(regionNumber))
        return 0;

    MPU->RNR = regionNumber;
    return 1;
}

static __INLINE uint32_t getCurrentMPURegionNumber(void)
{
    return MPU->RNR;
}

static __INLINE void setMPURegionAddress(uint32_t address)
{
    if (isMPUNotPresent())
        return;

    MPU->RBAR = address << MPU_RBAR_ADDR_SHIFT;
}

static __INLINE uint32_t getMPURegionAddress(void)
{
    if (isMPUNotPresent())
        return 0;

    return MPU->RBAR >> MPU_RBAR_ADDR_SHIFT;
}

static __INLINE void setMPURegionAttributeAndSize(uint32_t attributeAndSize)
{
    if (isMPUNotPresent())
        return;

    MPU->RASR = attributeAndSize;
}

static __INLINE uint32_t getMPURegionAttributeAndSize(void)
{
    if (isMPUNotPresent())
        return 0;

    return MPU->RASR;
}

static __INLINE uint32_t getCurrentlyExecutingExceptionNumber(void)
{
    return (__get_IPSR() & 0xFF);
}


static __INLINE uint32_t getCurrentSysTickControlValue(void)
{
    return SysTick->CTRL;
}

static __INLINE uint32_t getCurrentSysTickReloadValue(void)
{
    return SysTick->LOAD;
}

static __INLINE void setSysTickControlValue(uint32_t controlValue)
{
    SysTick->CTRL = controlValue;
}

static __INLINE void setSysTickReloadValue(uint32_t reloadValue)
{
    SysTick->LOAD = reloadValue & SysTick_LOAD_RELOAD_Msk;
}

static __INLINE uint32_t getSysTick10MillisecondInterval(void)
{
    return SysTick->CALIB & SysTick_CALIB_TENMS_Msk;
}

static __INLINE void disableSysTick(void)
{
    SysTick->CTRL = 0;
}

static __INLINE void enableSysTickWithCClkNoInterrupt(void)
{
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
}

static __INLINE void start10MillisecondSysTick(void)
{
    if (getSysTick10MillisecondInterval() == 0)
        return;
        
     disableSysTick();
     setSysTickReloadValue(getSysTick10MillisecondInterval());
     enableSysTickWithCClkNoInterrupt();
}

static __INLINE int has10MillisecondSysTickExpired(void)
{
    if (getSysTick10MillisecondInterval() == 0)
        return 1;

    return SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;
}


/* Program Status Register Bits. */
/*  Was the stack 8-byte aligned during auto stacking. */
#define PSR_STACK_ALIGN     (1 << 9)


#endif /* _DEBUG_CM3_H_ */
