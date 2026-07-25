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
#include <signal.h>
#include "mriPlatformBaseTest.h"

/* Addresses of fault status registers in System Control Block. */
#define CFSR  0xE000ED28
#define HFSR  0xE000ED2C
#define MMFAR 0xE000ED34
#define BFAR  0xE000ED38

TEST_GROUP_BASE(mriPlatformException, mriPlatformBase)
{
    void setup()
    {
        mriPlatformBase::setup();
    }

    void teardown()
    {
        mriPlatformBase::teardown();
    }

    void setFaultRegister(uint32_t regAddress, uint32_t regValue)
    {
        MemorySim_CreateRegion(m_pMemory, regAddress, sizeof(uint32_t));
        IMemory_Write32(m_pMemory, regAddress, regValue);
    }
};


TEST(mriPlatformException, SetExceptionCodeToNMI_ShouldReturn_SIGINT_TPacket)
{
    uint32_t ExceptionCode = 2;
    setIPSR(ExceptionCode);
    mockIComm_InitReceiveChecksummedData("+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGINT, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToHardFault_ShouldReturn_SIGSEGV_TPacket)
{
    uint32_t ExceptionCode = 3;
    setIPSR(ExceptionCode);
    mockIComm_InitReceiveChecksummedData("+++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Hard Fault**");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToHardFault_StatusRegisterSetTo0_VerifyOutput)
{
    uint32_t ExceptionCode = 3;
    setIPSR(ExceptionCode);
    setFaultRegister(HFSR, 0);
    mockIComm_InitReceiveChecksummedData("+++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Hard Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x00");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToHardFault_DebugEvent_VerifyOutput)
{
    uint32_t ExceptionCode = 3;
    setIPSR(ExceptionCode);
    setFaultRegister(HFSR, 1 << 31);
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Hard Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x80000000");
    appendExpectedOPacket("\n    Debug Event");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToHardFault_VectorTableReadError_VerifyOutput)
{
    uint32_t ExceptionCode = 3;
    setIPSR(ExceptionCode);
    setFaultRegister(HFSR, 1 << 1);
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Hard Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x02");
    appendExpectedOPacket("\n    Vector Table Read");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToMemManageFault_ShouldReturn_SIGSEGV_TPacket)
{
    uint32_t ExceptionCode = 4;
    setIPSR(ExceptionCode);
    mockIComm_InitReceiveChecksummedData("++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToMemManageFault_StatusTo0_VerifyOutput)
{
    uint32_t ExceptionCode = 4;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 0);
    mockIComm_InitReceiveChecksummedData("++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToMemManageFault_MMARValid_VerifyOutput)
{
    uint32_t ExceptionCode = 4;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << 7);
    setFaultRegister(MMFAR, 0xBAADFEED);
    mockIComm_InitReceiveChecksummedData("+++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**MPU Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x80");
    appendExpectedOPacket("\n    Fault Address: ");
    appendExpectedOPacket("0xbaadfeed");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToMemManageFault_MMARValidButMMFARAccessThrows_VerifyOutput)
{
    uint32_t ExceptionCode = 4;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << 7);
    mockIComm_InitReceiveChecksummedData("+++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**MPU Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x80");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToMemManageFault_FPLazyStatePreservation_VerifyOutput)
{
    uint32_t ExceptionCode = 4;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << 5);
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**MPU Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x20");
    appendExpectedOPacket("\n    FP Lazy Preservation");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToMemManageFault_StackingError_VerifyOutput)
{
    uint32_t ExceptionCode = 4;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << 4);
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**MPU Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x10");
    appendExpectedOPacket("\n    Stacking Error");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToMemManageFault_UnstackingError_VerifyOutput)
{
    uint32_t ExceptionCode = 4;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << 3);
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**MPU Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x08");
    appendExpectedOPacket("\n    Unstacking Error");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToMemManageFault_DataAccess_VerifyOutput)
{
    uint32_t ExceptionCode = 4;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << 1);
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**MPU Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x02");
    appendExpectedOPacket("\n    Data Access");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToMemManageFault_InstructionFetch_VerifyOutput)
{
    uint32_t ExceptionCode = 4;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << 0);
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**MPU Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x01");
    appendExpectedOPacket("\n    Instruction Fetch");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToHardFault_ForceToMemManageFault_StackingError_VerifyOutput)
{
    uint32_t ExceptionCode = 3;
    setIPSR(ExceptionCode);
    setFaultRegister(HFSR, 1 << 30);
    setFaultRegister(CFSR, 1 << 4);
    mockIComm_InitReceiveChecksummedData("++++++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Hard Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x40000000");
    appendExpectedOPacket("\n    Forced");
    appendExpectedOPacket("\n**MPU Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x10");
    appendExpectedOPacket("\n    Stacking Error");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_ShouldReturn_SIGBUS_TPacket)
{
    uint32_t BusFaultExceptionCode = 5;
    setIPSR(BusFaultExceptionCode);
    mockIComm_InitReceiveChecksummedData("++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_StatusTo0_VerifyOutput)
{
    uint32_t ExceptionCode = 5;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 0);
    mockIComm_InitReceiveChecksummedData("++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_BFARValid_VerifyOutput)
{
    uint32_t ExceptionCode = 5;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (7 + 8));
    setFaultRegister(BFAR, 0xBAADFEED);
    mockIComm_InitReceiveChecksummedData("+++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Bus Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x80");
    appendExpectedOPacket("\n    Fault Address: ");
    appendExpectedOPacket("0xbaadfeed");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_BARValidButBFARAccessThrows_VerifyOutput)
{
    uint32_t ExceptionCode = 5;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (7 + 8));
    mockIComm_InitReceiveChecksummedData("+++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Bus Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x80");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_FPLazyStatePreservation_VerifyOutput)
{
    uint32_t ExceptionCode = 5;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (5 + 8));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Bus Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x20");
    appendExpectedOPacket("\n    FP Lazy Preservation");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_StackingError_VerifyOutput)
{
    uint32_t ExceptionCode = 5;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (4 + 8));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Bus Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x10");
    appendExpectedOPacket("\n    Stacking Error");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_UnstackingError_VerifyOutput)
{
    uint32_t ExceptionCode = 5;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (3 + 8));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Bus Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x08");
    appendExpectedOPacket("\n    Unstacking Error");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_ImpreciseAccess_VerifyOutput)
{
    uint32_t ExceptionCode = 5;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (2 + 8));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Bus Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x04");
    appendExpectedOPacket("\n    Imprecise Data Access");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_PreciseAccess_VerifyOutput)
{
    uint32_t ExceptionCode = 5;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (1 + 8));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Bus Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x02");
    appendExpectedOPacket("\n    Precise Data Access");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToBusFault_InstructionPrefetch_VerifyOutput)
{
    uint32_t ExceptionCode = 5;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (0 + 8));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Bus Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x01");
    appendExpectedOPacket("\n    Instruction Prefetch");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGBUS, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToHardFault_ForceToBusFault_StackingError_VerifyOutput)
{
    uint32_t ExceptionCode = 3;
    setIPSR(ExceptionCode);
    setFaultRegister(HFSR, 1 << 30);
    setFaultRegister(CFSR, 1 << (4 + 8));
    mockIComm_InitReceiveChecksummedData("++++++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Hard Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x40000000");
    appendExpectedOPacket("\n    Forced");
    appendExpectedOPacket("\n**Bus Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x10");
    appendExpectedOPacket("\n    Stacking Error");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToUsageFault_ShouldReturn_SIGILL_TPacket)
{
    uint32_t ExceptionCode = 6;
    setIPSR(ExceptionCode);
    mockIComm_InitReceiveChecksummedData("++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGILL, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToUsageFault_StatusTo0_VerifyOutput)
{
    uint32_t ExceptionCode = 6;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 0);
    mockIComm_InitReceiveChecksummedData("++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGILL, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToUsageFault_DivideByZero_VerifyOutput)
{
    uint32_t ExceptionCode = 6;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (9 + 16));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Usage Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x0200");
    appendExpectedOPacket("\n    Divide by Zero");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGILL, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToUsageFault_Unaligned_VerifyOutput)
{
    uint32_t ExceptionCode = 6;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (8 + 16));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Usage Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x0100");
    appendExpectedOPacket("\n    Unaligned Access");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGILL, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToUsageFault_CoprocessorAccess_VerifyOutput)
{
    uint32_t ExceptionCode = 6;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (3 + 16));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Usage Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x08");
    appendExpectedOPacket("\n    Coprocessor Access");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGILL, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToUsageFault_InvalidPC_VerifyOutput)
{
    uint32_t ExceptionCode = 6;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (2 + 16));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Usage Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x04");
    appendExpectedOPacket("\n    Invalid Exception Return State");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGILL, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToUsageFault_InvalidState_VerifyOutput)
{
    uint32_t ExceptionCode = 6;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (1 + 16));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Usage Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x02");
    appendExpectedOPacket("\n    Invalid State");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGILL, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToUsageFault_UndefinedInstruction_VerifyOutput)
{
    uint32_t ExceptionCode = 6;
    setIPSR(ExceptionCode);
    setFaultRegister(CFSR, 1 << (0 + 16));
    mockIComm_InitReceiveChecksummedData("++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Usage Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x01");
    appendExpectedOPacket("\n    Undefined Instruction");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGILL, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToHardFault_ForceToUsageFault_UnalignedAccess_VerifyOutput)
{
    uint32_t ExceptionCode = 3;
    setIPSR(ExceptionCode);
    setFaultRegister(HFSR, 1 << 30);
    setFaultRegister(CFSR, 1 << (8 + 16));
    mockIComm_InitReceiveChecksummedData("++++++++++$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedOPacket("\n**Hard Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x40000000");
    appendExpectedOPacket("\n    Forced");
    appendExpectedOPacket("\n**Usage Fault**");
    appendExpectedOPacket("\n  Status Register: ");
    appendExpectedOPacket("0x0100");
    appendExpectedOPacket("\n    Unaligned Access");
    appendExpectedOPacket("\n");
    appendExpectedTPacket(SIGSEGV, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToHardFault_ForceToUsageFault_UnalignedAccess_WithGdbWaitVerifyNoOutput)
{
    uint32_t ExceptionCode = 3;
    mriPlatform_setWaitForGdbConnect(TRUE);
    setIPSR(ExceptionCode);
    setFaultRegister(HFSR, 1 << 30);
    setFaultRegister(CFSR, 1 << (8 + 16));
    mockIComm_InitReceiveChecksummedData("+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(mriPlatformException, SetExceptionCodeToDebugMonitorTrap_ShouldReturn_SIGTRAP_TPacket)
{
    uint32_t DebugMonitorExceptionCode = 12;
    setIPSR(DebugMonitorExceptionCode);
    mockIComm_InitReceiveChecksummedData("+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}
