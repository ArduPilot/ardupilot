/* Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

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

// Include headers from C modules under test.
extern "C"
{
    #include <DumpMocks.h>
}

// Include C++ headers for test harness.
#include <CppUTest/TestHarness.h>


TEST_GROUP(DumpMocks)
{
    void setup()
    {
        DumpMocks_Init(4);
    }

    void teardown()
    {
        DumpMocks_Uninit();
    }
};


TEST(DumpMocks, GetRamRegions_ShouldReturnNullByDefault)
{
    const CrashCatcherMemoryRegion* pRegions = CrashCatcher_GetMemoryRegions();
    POINTERS_EQUAL(NULL, pRegions);
}

TEST(DumpMocks, GetRamRegions_SetToReturnValidPointer_Verify)
{
    const CrashCatcherMemoryRegion regions[] = { {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE} };
    DumpMocks_SetMemoryRegions(regions);
    const CrashCatcherMemoryRegion* pRegions = CrashCatcher_GetMemoryRegions();
    POINTERS_EQUAL(regions, pRegions);
}

TEST(DumpMocks, getc_ReturnSpace)
{
    const int testData = ' ';
    DumpMocks_SetGetcData(&testData);
    int val = CrashCatcher_getc();
    CHECK_EQUAL(' ', val);
}

TEST(DumpMocks, getc_ReturnA)
{
    const int testData = 'A';
    DumpMocks_SetGetcData(&testData);
    int val = CrashCatcher_getc();
    CHECK_EQUAL('A', val);
}

TEST(DumpMocks, getc_Return2Chars)
{
    const int testData[] = {'a', 'z'};
    int val;

    DumpMocks_SetGetcData(testData);
    val = CrashCatcher_getc();
    CHECK_EQUAL('a', val);
    val = CrashCatcher_getc();
    CHECK_EQUAL('z', val);
}

TEST(DumpMocks, putc_OneSpace_ValidateData)
{
    CrashCatcher_putc(' ');
    STRCMP_EQUAL(" ", DumpMocks_GetPutCData());
}

TEST(DumpMocks, putc_OneA_ValidateData)
{
    CrashCatcher_putc('A');
    STRCMP_EQUAL("A", DumpMocks_GetPutCData());
}

TEST(DumpMocks, putc_TwoCharacters_ValidateData)
{
    CrashCatcher_putc('a');
    CrashCatcher_putc('z');
    STRCMP_EQUAL("az", DumpMocks_GetPutCData());
}

TEST(DumpMocks, putc_FourCharacters_MaximumBasedOnDumpMocksInitValue_ValidateData)
{
    CrashCatcher_putc('1');
    CrashCatcher_putc('2');
    CrashCatcher_putc('3');
    CrashCatcher_putc('4');
    STRCMP_EQUAL("1234", DumpMocks_GetPutCData());
}

TEST(DumpMocks, putc_FiveCharacters_MaximumPlusOneBasedOnDumpMocksInitValue_ValidateTruncatedData)
{
    CrashCatcher_putc('1');
    CrashCatcher_putc('2');
    CrashCatcher_putc('3');
    CrashCatcher_putc('4');
    CrashCatcher_putc('5');
    STRCMP_EQUAL("1234", DumpMocks_GetPutCData());
}
