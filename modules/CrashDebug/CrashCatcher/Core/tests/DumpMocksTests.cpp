/* Copyright (C) 2018  Adam Green (https://github.com/adamgreen)

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
    #include <CrashCatcherPriv.h>
    #include <DumpMocks.h>
}

// Include C++ headers for test harness.
#include <CppUTest/TestHarness.h>


TEST_GROUP(DumpMocks)
{
    CrashCatcherInfo m_dummyInfo;
    
    void setup()
    {
        memset(&m_dummyInfo, 0, sizeof(m_dummyInfo));
        DumpMocks_Init();
    }

    void teardown()
    {
        DumpMocks_Uninit();
    }
};


TEST(DumpMocks, GetDumpStartCallCount_MakeNoCalls_ShouldReturn0)
{
    CHECK_EQUAL(0, DumpMocks_GetDumpStartCallCount());
}

TEST(DumpMocks, GetDumpStartCallCount_MakeOneCall_ShouldReturn1)
{
    CrashCatcher_DumpStart(&m_dummyInfo);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
}

TEST(DumpMocks, GetDumpStartCallCount_MakeTwoCalls_ShouldReturn2)
{
    CrashCatcher_DumpStart(&m_dummyInfo);
    CrashCatcher_DumpStart(&m_dummyInfo);
    CHECK_EQUAL(2, DumpMocks_GetDumpStartCallCount());
}

TEST(DumpMocks, GetDumpStartInfo_VerifyDeepCopyWithDefaultValues)
{
    CrashCatcher_DumpStart(&m_dummyInfo);

    const CrashCatcherInfo* pInfo = DumpMocks_GetDumpStartInfo();
    CHECK_FALSE(pInfo == &m_dummyInfo);
    CHECK_TRUE(0 == memcmp(&m_dummyInfo, pInfo, sizeof(m_dummyInfo)));
}

TEST(DumpMocks, GetDumpStartInfo_VerifyDeepCopyWithNonDefaultValues)
{
    CrashCatcherInfo info;
    info.sp = 0xBAADF00D;
    info.isBKPT = 1;

    CrashCatcher_DumpStart(&info);

    const CrashCatcherInfo* pInfo = DumpMocks_GetDumpStartInfo();
    CHECK_FALSE(pInfo == &info);
    CHECK_TRUE(0 == memcmp(&info, pInfo, sizeof(info)));
}

TEST(DumpMocks, DumpEndCall_ShouldIndicateExitIsWantedOnFirstCall)
{
    CHECK_EQUAL(CRASH_CATCHER_EXIT, CrashCatcher_DumpEnd());
}

TEST(DumpMocks, DumpEndLoops_SetTo1_ShouldIndicateOneTryAgainAndThenExit)
{
    DumpMocks_SetDumpEndLoops(1);
    CHECK_EQUAL(CRASH_CATCHER_TRY_AGAIN, CrashCatcher_DumpEnd());
    CHECK_EQUAL(CRASH_CATCHER_EXIT, CrashCatcher_DumpEnd());
}

TEST(DumpMocks, GetDumpEndCallCount_MakeNoCalls_ShouldReturn0)
{
    CHECK_EQUAL(0, DumpMocks_GetDumpEndCallCount());
}

TEST(DumpMocks, GetDumpEndCallCount_MakeOneCall_ShouldReturn1)
{
    CrashCatcher_DumpEnd();
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(DumpMocks, GetDumpEndCallCount_MakeTwoCalls_ShouldReturn2)
{
    CrashCatcher_DumpEnd();
    CrashCatcher_DumpEnd();
    CHECK_EQUAL(2, DumpMocks_GetDumpEndCallCount());
}

TEST(DumpMocks, GetDumpMemoryCallCount_NoCalls_ShouldReturn0)
{
    CHECK_EQUAL(0, DumpMocks_GetDumpMemoryCallCount());
}

TEST(DumpMocks, VerifyDumpMemory_MismatchElementSize_ShouldReturnFalse)
{
    const uint8_t byte = 0x5a;
    CrashCatcher_DumpMemory(&byte, CRASH_CATCHER_BYTE, 1);
    CHECK_EQUAL(1, DumpMocks_GetDumpMemoryCallCount());
    CHECK_FALSE(DumpMocks_VerifyDumpMemoryItem(0, &byte, CRASH_CATCHER_WORD, 1));
}

TEST(DumpMocks, VerifyDumpMemory_MismatchElementCount_ShouldReturnFalse)
{
    const uint8_t byte = 0x5a;
    CrashCatcher_DumpMemory(&byte, CRASH_CATCHER_BYTE, 1);
    CHECK_EQUAL(1, DumpMocks_GetDumpMemoryCallCount());
    CHECK_FALSE(DumpMocks_VerifyDumpMemoryItem(0, &byte, CRASH_CATCHER_BYTE, 2));
}

TEST(DumpMocks, VerifyDumpMemory_MismatchContent_ShouldReturnFalse)
{
    const uint8_t byte = 0x5a;
    const uint8_t checkByte = 0x5b;
    CrashCatcher_DumpMemory(&byte, CRASH_CATCHER_BYTE, 1);
    CHECK_EQUAL(1, DumpMocks_GetDumpMemoryCallCount());
    CHECK_FALSE(DumpMocks_VerifyDumpMemoryItem(0, &checkByte, CRASH_CATCHER_BYTE, 1));
}

TEST(DumpMocks, VerifyDumpMemory_1Byte_Match_ShouldReturnTrue)
{
    const uint8_t byte = 0x5a;
    CrashCatcher_DumpMemory(&byte, CRASH_CATCHER_BYTE, 1);
    CHECK_EQUAL(1, DumpMocks_GetDumpMemoryCallCount());
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(0, &byte, CRASH_CATCHER_BYTE, 1));
}

TEST(DumpMocks, VerifyDumpMemory_2Bytes_Match_ShouldReturnTrue)
{
    const uint8_t bytes[2] = {0x5a, 0xa5};
    CrashCatcher_DumpMemory(&bytes, CRASH_CATCHER_BYTE, 2);
    CHECK_EQUAL(1, DumpMocks_GetDumpMemoryCallCount());
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(0, &bytes, CRASH_CATCHER_BYTE, 2));
}

TEST(DumpMocks, VerifyDumpMemory_1HalfWord_Match_ShouldReturnTrue)
{
    const uint16_t halfWord = 0x5a5a;
    CrashCatcher_DumpMemory(&halfWord, CRASH_CATCHER_HALFWORD, 1);
    CHECK_EQUAL(1, DumpMocks_GetDumpMemoryCallCount());
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(0, &halfWord, CRASH_CATCHER_HALFWORD, 1));
}

TEST(DumpMocks, VerifyDumpMemory_2HalfWords_Match_ShouldReturnTrue)
{
    const uint16_t halfWords[2] = {0x5a5a, 0xa5a5};
    CrashCatcher_DumpMemory(&halfWords, CRASH_CATCHER_HALFWORD, 2);
    CHECK_EQUAL(1, DumpMocks_GetDumpMemoryCallCount());
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(0, &halfWords, CRASH_CATCHER_HALFWORD, 2));
}

TEST(DumpMocks, VerifyDumpMemory_1Word_Match_ShouldReturnTrue)
{
    const uint32_t word = 0x5a5aa5a5;
    CrashCatcher_DumpMemory(&word, CRASH_CATCHER_WORD, 1);
    CHECK_EQUAL(1, DumpMocks_GetDumpMemoryCallCount());
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(0, &word, CRASH_CATCHER_WORD, 1));
}

TEST(DumpMocks, VerifyDumpMemory_2Words_Match_ShouldReturnTrue)
{
    const uint32_t words[2] = {0x5a5aa5a5, 0xa5a55a5a};
    CrashCatcher_DumpMemory(&words, CRASH_CATCHER_WORD, 2);
    CHECK_EQUAL(1, DumpMocks_GetDumpMemoryCallCount());
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(0, &words, CRASH_CATCHER_WORD, 2));
}

TEST(DumpMocks, VerifyDumpMemory_Issue3WritesOfVaryingSizes_ShouldReturnTrue)
{
    const uint8_t  bytes[4] = {0x11, 0x22, 0x33, 0x44};
    const uint16_t halfWords[2] = {0x5a5a, 0xa5a5};
    const uint32_t word = 0xaabbccdd;
    CrashCatcher_DumpMemory(&bytes, CRASH_CATCHER_BYTE, 4);
    CrashCatcher_DumpMemory(&halfWords, CRASH_CATCHER_HALFWORD, 2);
    CrashCatcher_DumpMemory(&word, CRASH_CATCHER_WORD, 1);
    CHECK_EQUAL(3, DumpMocks_GetDumpMemoryCallCount());
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(0, &bytes, CRASH_CATCHER_BYTE, 4));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(1, &halfWords, CRASH_CATCHER_HALFWORD, 2));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(2, &word, CRASH_CATCHER_WORD, 1));
}

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

TEST(DumpMocks, EnableDumpStartStackOverflowSimulation_ValidateStackModified)
{
    g_crashCatcherStack[0] = CRASH_CATCHER_STACK_SENTINEL;
    DumpMocks_EnableDumpStartStackOverflowSimulation();
    CrashCatcher_DumpStart(&m_dummyInfo);
    CHECK_EQUAL(0x00000000, g_crashCatcherStack[0]);
}

TEST(DumpMocks, EnableDumpStartStackOverflowSimulation_ValidateDefaultsToNoModification)
{
    g_crashCatcherStack[0] = CRASH_CATCHER_STACK_SENTINEL;
    CrashCatcher_DumpStart(&m_dummyInfo);
    CHECK_EQUAL(CRASH_CATCHER_STACK_SENTINEL, g_crashCatcherStack[0]);
}
