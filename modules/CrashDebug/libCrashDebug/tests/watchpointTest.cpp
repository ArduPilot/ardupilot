/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
extern "C"
{
    #include <MallocFailureInject.h>
    #include <MemorySim.h>
    #include <mri.h>
    #include <signal.h>
}
#include "mriPlatformBaseTest.h"

TEST_GROUP_BASE(watchpointTests, mriPlatformBase)
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
        MallocFailureInject_Restore();
    }
};


TEST(watchpointTests, Set4ByteReadWatchpoint_ShouldBeIgnored)
{
    char commands[64];
    snprintf(commands, sizeof(commands), "+$Z3,%x,4#", INITIAL_SP - 4);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
    IMemory_Write32(m_pMemory, INITIAL_SP - 4, 0xBAADF00D);
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);

    uint32_t value = IMemory_Read32(m_pMemory, INITIAL_SP - 4);
    CHECK_EQUAL(0xBAADF00D, value);
    CHECK_FALSE(MemorySim_WasWatchpointEncountered(m_pMemory));
}

TEST(watchpointTests, Set4ByteWriteWatchpoint_ShouldBeIgnored)
{
    char commands[64];
    snprintf(commands, sizeof(commands), "+$Z2,%x,4#", INITIAL_SP - 4);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);

    IMemory_Write32(m_pMemory, INITIAL_SP - 4, 0xBAADF00D);
    CHECK_FALSE(MemorySim_WasWatchpointEncountered(m_pMemory));
}

TEST(watchpointTests, Set4ByteReadWriteWatchpoint_ShouldBeIgnored)
{
    char commands[64];
    snprintf(commands, sizeof(commands), "+$Z4,%x,4#", INITIAL_SP - 4);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);

    IMemory_Write32(m_pMemory, INITIAL_SP - 4, 0xBAADF00D);
    CHECK_FALSE(MemorySim_WasWatchpointEncountered(m_pMemory));

    uint32_t value = IMemory_Read32(m_pMemory, INITIAL_SP - 4);
    CHECK_EQUAL(0xBAADF00D, value);
    CHECK_FALSE(MemorySim_WasWatchpointEncountered(m_pMemory));
}

TEST(watchpointTests, Clear4ByteReadWatchpoint_ShouldBeIgnored)
{
    char commands[64];
    snprintf(commands, sizeof(commands), "+$z3,%x,4#", INITIAL_SP - 4);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
    IMemory_Write32(m_pMemory, INITIAL_SP - 4, 0xBAADF00D);
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}
