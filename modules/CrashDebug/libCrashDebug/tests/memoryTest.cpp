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
#include <mri.h>
#include "mriPlatformBaseTest.h"

TEST_GROUP_BASE(memoryTests, mriPlatformBase)
{
    void setup()
    {
        mriPlatformBase::setup();
        // Set IPSR to DebugMonitor exception to generate SIGTRAP exception code.
        setIPSR(12);
    }

    void teardown()
    {
        mriPlatformBase::teardown();
    }
};


TEST(memoryTests, ReadByte)
{
    IMemory_Write8(m_pMemory, INITIAL_SP - 1, 0x5a);
    char command[64];
    snprintf(command, sizeof(command), "+$m%x,1#", INITIAL_SP - 1);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$5a#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(memoryTests, ReadHalfWord)
{
    IMemory_Write16(m_pMemory, INITIAL_SP - 2, 0xF00D);
    char command[64];
    snprintf(command, sizeof(command), "+$m%x,2#", INITIAL_SP - 2);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$0df0#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(memoryTests, ReadWord)
{
    IMemory_Write32(m_pMemory, INITIAL_SP - 4, 0xBAADF00D);
    char command[64];
    snprintf(command, sizeof(command), "+$m%x,4#", INITIAL_SP - 4);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$0df0adba#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(memoryTests, WriteByte)
{
    char command[64];
    snprintf(command, sizeof(command), "+$M%x,1:5A#", INITIAL_SP - 1);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(0x5A, IMemory_Read8(m_pMemory, INITIAL_SP - 1));
}

TEST(memoryTests, WriteHalfWord)
{
    char command[64];
    snprintf(command, sizeof(command), "+$M%x,2:0DF0#", INITIAL_SP - 2);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(0xF00D, IMemory_Read16(m_pMemory, INITIAL_SP - 2));
}

TEST(memoryTests, WriteWord)
{
    char command[64];
    snprintf(command, sizeof(command), "+$M%x,4:0DF0ADBA#", INITIAL_SP - 4);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(0xBAADF00D, IMemory_Read32(m_pMemory, INITIAL_SP - 4));
}

TEST(memoryTests, ReadByte_InvalidAddress_ShouldSendErrorBack)
{
    char command[64];
    snprintf(command, sizeof(command), "+$m%x,1#", INITIAL_SP);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$E03#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(memoryTests, ReadHalfWord_InvalidAddress_ShouldSendErrorBack)
{
    char command[64];
    snprintf(command, sizeof(command), "+$m%x,2#", INITIAL_SP);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$E03#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(memoryTests, ReadWord_InvalidAddress_ShouldSendErrorBack)
{
    char command[64];
    snprintf(command, sizeof(command), "+$m%x,4#", INITIAL_SP);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$E03#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(memoryTests, ReadBytes_InvalidAddressForLastByte_ShouldSendPartialDataBack)
{
    char command[64];
    snprintf(command, sizeof(command), "+$m%x,8#", INITIAL_SP - 7);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$00000000000000#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(memoryTests, WriteByte_InvalidAddress_ShouldSendErrorBack)
{
    char command[64];
    snprintf(command, sizeof(command), "+$M%x,1:5A#", INITIAL_SP);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(memoryTests, WriteHalfWord_InvalidAddress_ShouldSendErrorBack)
{
    char command[64];
    snprintf(command, sizeof(command), "+$M%x,2:BAAD#", INITIAL_SP);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(memoryTests, WriteWord_InvalidAddress_ShouldSendErrorBack)
{
    char command[64];
    snprintf(command, sizeof(command), "+$M%x,4:BAADF00D#", INITIAL_SP);
    mockIComm_InitReceiveChecksummedData(command, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}