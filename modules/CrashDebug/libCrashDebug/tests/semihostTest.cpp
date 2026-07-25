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
}
#include <signal.h>
#include "mriPlatformBaseTest.h"

#define NEWLIB_WRITE    0xFE

TEST_GROUP_BASE(semihostTests, mriPlatformBase)
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


TEST(semihostTests, WriteCall_VerifyThatItIsIgnored)
{
    const char buffer[] = "Test\n";
    m_context.R[0] = 0xbaadfeed;
    m_context.R[1] = INITIAL_SP - sizeof(buffer) + 1;
    m_context.R[2] = sizeof(buffer) - 1;

    emitBKPT(NEWLIB_WRITE);
    emitBKPT(0);

    mockIComm_InitReceiveChecksummedData("+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, INITIAL_SP, INITIAL_LR, INITIAL_PC);
    appendExpectedString("+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
    CHECK_EQUAL(0xbaadfeed, m_context.R[0]);
    CHECK_EQUAL(INITIAL_PC, m_context.R[PC]);
}
