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
#include "mriPlatformBaseTest.h"

TEST_GROUP_BASE(continueTests, mriPlatformBase)
{
    void setup()
    {
        mriPlatformBase::setup();
    }

    void teardown()
    {
        mriPlatformBase::teardown();
    }
};


TEST(continueTests, AttemptToContinue_ShouldRemainWhereItWas)
{
    setIPSR(0);
    mockIComm_InitReceiveChecksummedData("+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGSTOP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(continueTests, AttemptToContinueWithNewPC_ShouldRemainWhereItWas)
{
    setIPSR(0);
    mockIComm_InitReceiveChecksummedData("+$cbaadfeed#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGSTOP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}

TEST(continueTests, AttemptToContinueAndSkipHardcodedBreakpoint_ShouldRemainWhereItWas)
{
    setIPSR(0);
    emitBKPT(0);
    mockIComm_InitReceiveChecksummedData("+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGSTOP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}
