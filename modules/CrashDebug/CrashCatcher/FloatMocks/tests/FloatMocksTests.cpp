/* Copyright (C) 2015  Adam Green (https://github.com/adamgreen)

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
    #include <FloatMocks.h>
}
#include <string.h>

// Include C++ headers for test harness.
#include <CppUTest/TestHarness.h>


TEST_GROUP(FloatMocks)
{
    void setup()
    {
        FloatMocks_Init();
    }

    void teardown()
    {
        FloatMocks_Uninit();
    }
};


TEST(FloatMocks, SetAllFloatingPointRegisters_ValidateCopyUsesThoseValues)
{
    uint32_t testFloatRegs[32+1];
    for (size_t i = 0 ; i < 32 ; i++)
        testFloatRegs[i] = i;
    testFloatRegs[32] = 0x12345678;
    FloatMocks_SetAllFloatingPointRegisters(testFloatRegs);

    uint32_t floatRegs[32+1];
    memset(floatRegs, 0, sizeof(floatRegs));
    CrashCatcher_CopyAllFloatingPointRegisters(floatRegs);
    CHECK_TRUE(0 == memcmp(testFloatRegs, floatRegs, sizeof(testFloatRegs)));
}

TEST(FloatMocks, SetUpperFloatingPointRegisters_ValidateCopyUsesThoseValues)
{
    uint32_t testFloatRegs[16];
    for (size_t i = 0 ; i < 16 ; i++)
        testFloatRegs[i] = i;
    FloatMocks_SetUpperFloatingPointRegisters(testFloatRegs);

    uint32_t floatRegs[16];
    memset(floatRegs, 0, sizeof(floatRegs));
    CrashCatcher_CopyUpperFloatingPointRegisters(floatRegs);
    CHECK_TRUE(0 == memcmp(testFloatRegs, floatRegs, sizeof(testFloatRegs)));
}
