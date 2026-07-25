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
// Include headers from C modules under test.
extern "C"
{
    #include <common.h>
    #include <core.h>
    #include <mriPlatform.h>
    #include <MemorySim.h>
    #include <platforms.h>
}

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


TEST_GROUP(mriPlatformInit)
{
    IMemory*        m_pMem;
    RegisterContext m_context;

    void setup()
    {
        m_pMem = MemorySim_Init();
    }

    void teardown()
    {
        CHECK_EQUAL(noException, getExceptionCode());
        MemorySim_Uninit(m_pMem);
    }
};


TEST(mriPlatformInit, ValidateSuccessfulInit_NoRegistersModifiedInContext)
{
    size_t   i = 0;
    uint32_t fillValue = 0;
    for (i = 0 ; i < ARRAY_SIZE(m_context.R) ; i++, fillValue += 0x11111111)
        m_context.R[i] = fillValue;

    mriPlatform_Init(&m_context, m_pMem);

    for (i = 0, fillValue = 0 ; i < ARRAY_SIZE(m_context.R) ; i++, fillValue += 0x11111111)
        CHECK_EQUAL(fillValue, m_context.R[i]);
    CHECK_FALSE(Platform_WasMemoryFaultEncountered());
    CHECK_TRUE(IsFirstException());
    CHECK_TRUE(WasSuccessfullyInit());
}
