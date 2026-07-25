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
/* NOTE: These are tests for Platform_* APIs which must be included in library for things to link but the
         functionality won't be called from MRI core because of the code paths which end up being enabled. */
#include "mriPlatformBaseTest.h"
extern "C"
{
    #include <platforms.h>
    #include <semihost.h>
}

TEST_GROUP_BASE(otherTests, mriPlatformBase)
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


TEST(otherTests, CallAPIsNotUsedByMRI_GuaranteeCoverage)
{
    Platform_CommClearInterrupt();
    Platform_CommPrepareToWaitForGdbConnection();
    CHECK_FALSE(Platform_CommIsWaitingForGdbToConnect());
    Platform_CommWaitForReceiveDataToStop();

    Platform_AdvanceProgramCounterToNextInstruction();
    CHECK_EQUAL(MRI_PLATFORM_INSTRUCTION_OTHER, Platform_TypeOfCurrentInstruction());

    PlatformSemihostParameters parameters;
    parameters = Platform_GetSemihostCallParameters();
    CHECK_EQUAL(0x00000000, parameters.parameter1);
    CHECK_EQUAL(0x00000000, parameters.parameter2);
    CHECK_EQUAL(0x00000000, parameters.parameter3);
    CHECK_EQUAL(0x00000000, parameters.parameter4);

    Platform_SetSemihostCallReturnAndErrnoValues(-1 , -1);
    CHECK_EQUAL(0x00000000, m_context.R[0]);
    CHECK_EQUAL(0x11111111, m_context.R[1]);

    Semihost_HandleSemihostRequest();
}
