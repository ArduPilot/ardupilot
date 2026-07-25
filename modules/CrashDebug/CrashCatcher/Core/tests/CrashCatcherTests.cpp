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
#include <stdint.h>
#include <string.h>

// Include headers from C modules under test.
extern "C"
{
    #include <CrashCatcher.h>
    #include <CrashCatcherPriv.h>
    #include <DumpMocks.h>
    #include <FloatMocks.h>

    // Provides the upper 32-bits of 64-bit pointer addresses.
    // When running unit tests on 64-bit machines, the 32-bit emulated PSP an MSP stack pointer addresses don't
    // contain enough bits to make a proper 64-bit pointer.  This provides those upper bits.
    extern uint64_t g_crashCatcherTestBaseAddress;

    // The unit tests can point the core to a fake location for the SCB->CPUID register.
    extern uint32_t* g_pCrashCatcherCpuId;

    // The unit tests can point the core to a fake location for the fault status registers.
    extern uint32_t* g_pCrashCatcherFaultStatusRegisters;

    // The unit tests can point the core to a fake location for the Coprocessor Access Control Register.
    extern uint32_t* g_pCrashCatcherCoprocessorAccessControlRegister;
}


static const uint8_t g_expectedSignature[4] = {CRASH_CATCHER_SIGNATURE_BYTE0,
                                               CRASH_CATCHER_SIGNATURE_BYTE1,
                                               CRASH_CATCHER_VERSION_MAJOR,
                                               CRASH_CATCHER_VERSION_MINOR};

#define USING_PSP false
#define USING_MSP true

#define NOP_INSTRUCTION     0xBF00
#define BKPT_INSTRUCTION    0xBE00

// Include C++ headers for test harness.
#include <CppUTest/TestHarness.h>


TEST_GROUP(CrashCatcher)
{
    CrashCatcherExceptionRegisters m_exceptionRegisters;
    uint32_t                       m_expectedFlags;
    uint32_t                       m_emulatedPSP[8];
    uint32_t                       m_emulatedMSP[8 + 16 + 1];
    uint32_t                       m_emulatedCpuId;
    uint32_t                       m_emulatedFaultStatusRegisters[5];
    uint32_t                       m_emulatedCoprocessorAccessControlRegister;
    uint32_t                       m_expectedSP;
    uint32_t                       m_memoryStart;
    uint32_t                       m_faultStatusRegistersStart;
    uint32_t                       m_expectedFloatingPointRegisters[32+1];
    int                            m_expectedIsBKPT;
    uint16_t                       m_emulatedInstruction;
    uint8_t                        m_memory[16];

    void setup()
    {
        DumpMocks_Init();
        initExceptionRegisters();
        initPSP();
        initMSP();
        emulateNOP();
        initMemory();
        initCpuId();
        initFaultStatusRegisters();
        initFloatingPoint();
        if (sizeof(int*) == sizeof(uint64_t))
            g_crashCatcherTestBaseAddress = (uint64_t)&m_emulatedPSP & 0xFFFFFFFF00000000ULL;
    }

    void initExceptionRegisters()
    {
        m_exceptionRegisters.exceptionPSR = 0;
        m_exceptionRegisters.psp = (uint32_t)(unsigned long)m_emulatedPSP;
        m_exceptionRegisters.msp = (uint32_t)(unsigned long)m_emulatedMSP;
        m_exceptionRegisters.r4  = 0x44444444;
        m_exceptionRegisters.r5  = 0x55555555;
        m_exceptionRegisters.r6  = 0x66666666;
        m_exceptionRegisters.r7  = 0x77777777;
        m_exceptionRegisters.r8  = 0x88888888;
        m_exceptionRegisters.r9  = 0x99999999;
        m_exceptionRegisters.r10 = 0xAAAAAAAA;
        m_exceptionRegisters.r11 = 0xBBBBBBBB;
        emulateMSPEntry();
    }

    void initPSP()
    {
        m_emulatedPSP[0] = 0xFFFF0000;
        m_emulatedPSP[1] = 0xFFFF1111;
        m_emulatedPSP[2] = 0xFFFF2222;
        m_emulatedPSP[3] = 0xFFFF3333;
        m_emulatedPSP[4] = 0xFFFF4444;
        m_emulatedPSP[5] = 0xFFFF5555;
        // Point PC to emulated instruction.
        m_emulatedPSP[6] = (uint32_t)(unsigned long)&m_emulatedInstruction;
        m_emulatedPSP[7] = 0;
    }

    void initMSP()
    {
        m_emulatedMSP[0] = 0x0000FFFF;
        m_emulatedMSP[1] = 0x1111FFFF;
        m_emulatedMSP[2] = 0x2222FFFF;
        m_emulatedMSP[3] = 0x3333FFFF;
        m_emulatedMSP[4] = 0x4444FFFF;
        m_emulatedMSP[5] = 0x5555FFFF;
        // Point PC to emulated instruction.
        m_emulatedMSP[6] = (uint32_t)(unsigned long)&m_emulatedInstruction;
        m_emulatedMSP[7] = 0;
    }

    void emulateNOP()
    {
        m_emulatedInstruction = NOP_INSTRUCTION;
        m_expectedIsBKPT = 0;
    }

    void emulateBKPT(uint8_t bkptNumber)
    {
        m_emulatedInstruction = BKPT_INSTRUCTION | (uint16_t)bkptNumber;
        m_expectedIsBKPT = 1;
    }

    void initMemory()
    {
        for (size_t i = 0 ; i < sizeof(m_memory) ; i++)
            m_memory[i]= i;
        m_memoryStart = (uint32_t)(unsigned long)m_memory;
    }

    void initCpuId()
    {
        static const uint32_t cpuIdCortexM0 = 0x410CC200;
        m_emulatedCpuId = cpuIdCortexM0;
        g_pCrashCatcherCpuId = &m_emulatedCpuId;
    }

    void initFaultStatusRegisters()
    {
        memset(m_emulatedFaultStatusRegisters, 0, sizeof(m_emulatedFaultStatusRegisters));
        g_pCrashCatcherFaultStatusRegisters = m_emulatedFaultStatusRegisters;
        m_faultStatusRegistersStart = (uint32_t)(unsigned long)m_emulatedFaultStatusRegisters;
    }

    void initFloatingPoint()
    {
        m_expectedFlags = 0;
        m_emulatedCoprocessorAccessControlRegister = 0;
        g_pCrashCatcherCoprocessorAccessControlRegister = &m_emulatedCoprocessorAccessControlRegister;
        for (size_t i = 0 ; i < 32 ; i++)
            m_expectedFloatingPointRegisters[i] = i;
        m_expectedFloatingPointRegisters[32] = 0x12345678;
    }

    void teardown()
    {
        validateDumpStartInfo();
        DumpMocks_Uninit();
    }

    void emulateMSPEntry()
    {
        m_exceptionRegisters.exceptionLR = 0xFFFFFFF1;
        m_expectedSP = (uint32_t)(unsigned long)&m_emulatedMSP + 8 * sizeof(uint32_t);
    }

    void emulatePSPEntry()
    {
        m_exceptionRegisters.exceptionLR = 0xFFFFFFFD;
        m_expectedSP = (uint32_t)(unsigned long)&m_emulatedPSP + 8 * sizeof(uint32_t);
    }

    void emulateStackAlignmentDuringException()
    {
        m_emulatedMSP[7] = 0x200;
        m_expectedSP |= 4;
    }

    void emulateFloatingPointStacking()
    {
        m_exceptionRegisters.exceptionLR &= ~LR_FLOAT;
        m_expectedSP += 18 * sizeof(uint32_t);
        memcpy(&m_emulatedMSP[8], &m_expectedFloatingPointRegisters, 16 * sizeof(uint32_t));
        m_emulatedMSP[8 + 16] = m_expectedFloatingPointRegisters[32];
    }

    void validateHeaderAndDumpedRegisters(bool usingMSP)
    {
        uint32_t* pSP = usingMSP ? m_emulatedMSP : m_emulatedPSP;
        // Need to handle the fact that the PC on stack might have been advanced past a hardcoded breakpoint but the 
        // dump would contain the original value at the time of the crash.
        uint32_t  registersLR_PC_XPSR[3] = { pSP[5], (uint32_t)(unsigned long)&m_emulatedInstruction, pSP[7] };
        
        CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(0, g_expectedSignature, CRASH_CATCHER_BYTE, sizeof(g_expectedSignature)));
        CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(1, &m_expectedFlags, CRASH_CATCHER_BYTE, sizeof(m_expectedFlags)));
        CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(2, &pSP[0], CRASH_CATCHER_BYTE, 4 * sizeof(uint32_t)));
        CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(3, &m_exceptionRegisters.r4, CRASH_CATCHER_BYTE, (11 - 4 + 1) * sizeof(uint32_t)));
        CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(4, &pSP[4], CRASH_CATCHER_BYTE, sizeof(uint32_t)));
        CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(5, &m_expectedSP, CRASH_CATCHER_BYTE, sizeof(uint32_t)));
        CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(6, &registersLR_PC_XPSR[0], CRASH_CATCHER_BYTE, 3 * sizeof(uint32_t)));
        CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(7, &m_exceptionRegisters.msp, CRASH_CATCHER_BYTE, 3 * sizeof(uint32_t)));
        if (m_expectedFlags & CRASH_CATCHER_FLAGS_FLOATING_POINT)
            CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(8, m_expectedFloatingPointRegisters, CRASH_CATCHER_BYTE, sizeof(m_expectedFloatingPointRegisters)));
    }

    void validateDumpStartInfo()
    {
        const CrashCatcherInfo* pInfo = DumpMocks_GetDumpStartInfo();

        CHECK_EQUAL(m_expectedSP, pInfo->sp);
        CHECK_EQUAL(m_expectedIsBKPT, pInfo->isBKPT);
    }
};


TEST(CrashCatcher, DumpRegistersOnly_MSP_StackAlignmentNeeded)
{
    emulateStackAlignmentDuringException();
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(8, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpRegistersOnly_MSP_StackAlignmentNotNeeded)
{
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(8, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpRegistersOnly_PSP_StackAlignmentNotNeeded)
{
    emulatePSPEntry();
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(8, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_PSP);
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpRegistersOnly_MSP_AdvanceProgramCounterPastBKPT0)
{
    uint32_t expectedPC = m_emulatedMSP[6] + 2;
    emulateBKPT(0);
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(8, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
    CHECK_EQUAL(expectedPC, m_emulatedMSP[6]);
}

TEST(CrashCatcher, DumpRegistersOnly_MSP_AdvanceProgramCounterPastBKPT255)
{
    uint32_t expectedPC = m_emulatedMSP[6] + 2;
    emulateBKPT(255);
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(8, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
    CHECK_EQUAL(expectedPC, m_emulatedMSP[6]);
}

TEST(CrashCatcher, DumpEndReturnTryAgainOnce_ShouldDumpTwice)
{
    DumpMocks_SetDumpEndLoops(1);
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(2, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(16, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_EQUAL(2, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpOneDoubleByteRegion)
{
    static const CrashCatcherMemoryRegion regions[] = { {m_memoryStart, m_memoryStart + 2, CRASH_CATCHER_BYTE},
                                                        {   0xFFFFFFFF,        0xFFFFFFFF, CRASH_CATCHER_BYTE} };
    DumpMocks_SetMemoryRegions(regions);
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(10, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(8, &regions[0], CRASH_CATCHER_BYTE, 2 * sizeof(uint32_t)));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(9, m_memory, CRASH_CATCHER_BYTE, 2));
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpOneWordRegion)
{
    static const CrashCatcherMemoryRegion regions[] = { {m_memoryStart, m_memoryStart + 4, CRASH_CATCHER_WORD},
                                                        {   0xFFFFFFFF,        0xFFFFFFFF, CRASH_CATCHER_BYTE} };
    DumpMocks_SetMemoryRegions(regions);
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(10, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(8, &regions[0], CRASH_CATCHER_BYTE, 2 * sizeof(uint32_t)));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(9, m_memory, CRASH_CATCHER_WORD, 1));
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpOneHalfwordRegion)
{
    static const CrashCatcherMemoryRegion regions[] = { {m_memoryStart, m_memoryStart + 2, CRASH_CATCHER_HALFWORD},
                                                        {   0xFFFFFFFF,        0xFFFFFFFF, CRASH_CATCHER_BYTE} };
    DumpMocks_SetMemoryRegions(regions);
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(10, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(8, &regions[0], CRASH_CATCHER_BYTE, 2 * sizeof(uint32_t)));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(9, m_memory, CRASH_CATCHER_HALFWORD, 1));
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpMultipleRegions)
{
    static const CrashCatcherMemoryRegion regions[] = { {        m_memoryStart,         m_memoryStart + 1, CRASH_CATCHER_BYTE},
                                                        {    m_memoryStart + 1,     m_memoryStart + 1 + 2, CRASH_CATCHER_HALFWORD},
                                                        {m_memoryStart + 1 + 2, m_memoryStart + 1 + 2 + 4, CRASH_CATCHER_WORD},
                                                        {           0xFFFFFFFF,                0xFFFFFFFF, CRASH_CATCHER_BYTE} };
    DumpMocks_SetMemoryRegions(regions);
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(14, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(8, &regions[0], CRASH_CATCHER_BYTE, 2 * sizeof(uint32_t)));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(9, &m_memory, CRASH_CATCHER_BYTE, 1));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(10, &regions[1], CRASH_CATCHER_BYTE, 2 * sizeof(uint32_t)));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(11, &m_memory[1], CRASH_CATCHER_HALFWORD, 1));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(12, &regions[2], CRASH_CATCHER_BYTE, 2 * sizeof(uint32_t)));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(13, &m_memory[3], CRASH_CATCHER_WORD, 1));
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, SimulateStackOverflow_ShouldAppendExtraMagicWordToEndOfData)
{
    uint8_t magicValueIndicatingStackOverflow[4] = {0xAC, 0xCE, 0x55, 0xED};

    DumpMocks_EnableDumpStartStackOverflowSimulation();
    CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(9, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(8, magicValueIndicatingStackOverflow,
                                              CRASH_CATCHER_BYTE, sizeof(magicValueIndicatingStackOverflow)));
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpOneWordRegion_EmulateCortexM3_ShouldAppendFaultStatusRegisters)
{
    static const uint32_t cpuIdCortexM3 = 0x410FC230;
    static const CrashCatcherMemoryRegion regions[] = { {m_memoryStart, m_memoryStart + 4, CRASH_CATCHER_WORD},
                                                        {   0xFFFFFFFF,        0xFFFFFFFF, CRASH_CATCHER_BYTE} };
    DumpMocks_SetMemoryRegions(regions);
    m_emulatedCpuId = cpuIdCortexM3;
    m_emulatedFaultStatusRegisters[0] = 0x12345678;
    m_emulatedFaultStatusRegisters[1] = 0x11111111;
    m_emulatedFaultStatusRegisters[2] = 0x22222222;
    m_emulatedFaultStatusRegisters[3] = 0x33333333;
    m_emulatedFaultStatusRegisters[4] = 0x44444444;
        CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(12, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(8, &regions[0], CRASH_CATCHER_BYTE, 2 * sizeof(uint32_t)));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(9, m_memory, CRASH_CATCHER_WORD, 1));

    CrashCatcherMemoryRegion faultStatusRegisters = {m_faultStatusRegistersStart,
                                                     m_faultStatusRegistersStart + 5 * (uint32_t)sizeof(uint32_t),
                                                     CRASH_CATCHER_BYTE};
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(10, &faultStatusRegisters, CRASH_CATCHER_BYTE, 2 * sizeof(uint32_t)));
    CHECK_TRUE(DumpMocks_VerifyDumpMemoryItem(11, m_emulatedFaultStatusRegisters, CRASH_CATCHER_WORD, 5));
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpRegistersOnly_OnlyCp10Enabled_ShouldOnlyDumpIntegerRegisters)
{
    m_emulatedCoprocessorAccessControlRegister = 3 << 20;
        CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(8, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpRegistersOnly_OnlyCp11Enabled_ShouldOnlyDumpIntegerRegisters)
{
    m_emulatedCoprocessorAccessControlRegister = 3 << 22;
        CrashCatcher_Entry(&m_exceptionRegisters);
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(8, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpRegistersOnly_EnableCp10AndCp11_NoAutoStack_ShouldDumpIntegerAndFloatingPointRegisters)
{
    m_emulatedCoprocessorAccessControlRegister = (3 << 20) | (3 << 22);
    FloatMocks_SetAllFloatingPointRegisters(m_expectedFloatingPointRegisters);
        CrashCatcher_Entry(&m_exceptionRegisters);
    m_expectedFlags |= CRASH_CATCHER_FLAGS_FLOATING_POINT;
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(9, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}

TEST(CrashCatcher, DumpRegistersOnly_EnableCp10AndCp11_AutoStack_ShouldDumpIntegerAndFloatingPointRegisters)
{
    m_emulatedCoprocessorAccessControlRegister = (3 << 20) | (3 << 22);
    emulateFloatingPointStacking();
    FloatMocks_SetUpperFloatingPointRegisters(&m_expectedFloatingPointRegisters[16]);
        CrashCatcher_Entry(&m_exceptionRegisters);
    m_expectedFlags |= CRASH_CATCHER_FLAGS_FLOATING_POINT;
    CHECK_EQUAL(1, DumpMocks_GetDumpStartCallCount());
    CHECK_EQUAL(9, DumpMocks_GetDumpMemoryCallCount());
    validateHeaderAndDumpedRegisters(USING_MSP);
    CHECK_EQUAL(1, DumpMocks_GetDumpEndCallCount());
}
