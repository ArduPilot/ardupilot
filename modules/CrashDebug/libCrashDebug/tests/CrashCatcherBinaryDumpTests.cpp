/*  Copyright (C) 2015  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#include "DumpBaseTest.h"


TEST_GROUP_BASE(CrashCatcherBinaryDump, DumpBaseTest)
{
    void setup()
    {
        m_pTestFilename = "CrashCatcherBinaryDumpTest.dmp";
        DumpBaseTest::setup();
    }

    void teardown()
    {
        DumpBaseTest::teardown();
    }

    void createTestDumpFile(const void* pData, size_t dataSize)
    {
        FILE* pFile = fopen(m_pTestFilename, "wb");
        fwrite(pData, 1, dataSize, pFile);
        fclose(pFile);
    }
};


// Pull in shared binary/hex shared tests from header file.
#define DUMP_TEST(X) TEST(CrashCatcherBinaryDump, X)
#define CRASH_CATCHER_DUMP_READ_FUNC CrashCatcherDump_ReadBinary

#include "CrashCatcherDumpTests.h"
