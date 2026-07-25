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
    #include <mri.h>
    #include <MallocFailureInject.h>
}
#include "mriPlatformBaseTest.h"

TEST_GROUP_BASE(breakpointTests, mriPlatformBase)
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


TEST(breakpointTests, Set16bitBreakpoint_ShouldBeIgnored)
{
    char commands[64];
    snprintf(commands, sizeof(commands), "+$Z1,%x,2#", INITIAL_PC + 6);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(breakpointTests, Set32bitBreakpoint_ShouldBeIgnored)
{
    char commands[64];
    snprintf(commands, sizeof(commands), "+$Z1,%x,3#", INITIAL_PC + 6);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(breakpointTests, SetBreakpointHit_ClearBreakpoint_ShouldBeIgnored)
{
    char commands[64];
    snprintf(commands, sizeof(commands), "+$Z1,%x,2#", INITIAL_PC + 4);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());

    mockIComm_InitTransmitDataBuffer(1024);
    snprintf(commands, sizeof(commands), "+$z1,%x,2#", INITIAL_PC + 4);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
    mockIComm_DelayReceiveData(3);
        mriPlatform_Run(mockIComm_Get());
    resetExpectedBuffer();
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(breakpointTests, Set32bitBreakpoint_Clear32bitBreakpoint_ShouldBeIgnored)
{
    char commands[64];
    snprintf(commands, sizeof(commands), "+$Z1,%x,3#", INITIAL_PC + 6);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());

    mockIComm_InitTransmitDataBuffer(1024);
    snprintf(commands, sizeof(commands), "+$z1,%x,3#", INITIAL_PC + 6);
    mockIComm_InitReceiveChecksummedData(commands, "+$c#");
    mockIComm_DelayReceiveData(3);
        mriPlatform_Run(mockIComm_Get());
    resetExpectedBuffer();
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}
