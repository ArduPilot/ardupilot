/*  Copyright (C) 2019  Adam Green (https://github.com/adamgreen)

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
    #include <GdbLogParser.h>
}

#include "DumpBaseTest.h"


#define DUMMY_FILE_HANDLE (FILE*)1

TEST_GROUP_BASE(GdbLogParser, DumpBaseTest)
{
    void setup()
    {
        DumpBaseTest::setup();
        m_expectedRegisters.R[MSP] = DEFAULT_SP_VALUE;
        m_expectedRegisters.R[PSP] = DEFAULT_SP_VALUE;
        fakeLogFileAccess();
    }

    void fakeLogFileAccess()
    {
        fopenSetReturn(DUMMY_FILE_HANDLE);
        fcloseIgnore();
        fseekSetReturn(0);
    }

    void teardown()
    {
        DumpBaseTest::teardown();
        fopenRestore();
        fcloseRestore();
        fseekRestore();
        fgetsRestore();
    }
};


TEST(GdbLogParser, InvalidLogFilename_ShouldThrowMSPandPSPnotInit)
{
    fopenSetReturn(NULL);
        __try_and_catch( GdbLogParse(m_pMem, &m_actualRegisters, "invalid_file.log") );
    CHECK_EQUAL(fileException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("Failed to open \"invalid_file.log\" GDB log.", getExceptionMessage());
    m_expectedRegisters.R[MSP] = 0xDEADBEEF;
    m_expectedRegisters.R[PSP] = 0xDEADBEEF;
}

TEST(GdbLogParser, OneLineLogFile_1RamValue_FailFSeekCall_ShouldThrow)
{
    static const char* testLines[] = { "0x10000000:\t0x11111111" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
    fseekSetReturn(-1);
        __try_and_catch( GdbLogParse(m_pMem, &m_actualRegisters, "foo.log") );
    CHECK_EQUAL(fileException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("Failed to rewind GDB log for second pass.", getExceptionMessage());
}

TEST(GdbLogParser, EmptyLogfile_ShouldReturnNoRegions)
{
    static const char* xmlForEmptyRegions = "<?xml version=\"1.0\"?>"
                                            "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                            "<memory-map>"
                                            "</memory-map>";
    fgetsSetData(NULL, 0);
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForEmptyRegions, pMemoryLayout);
}

TEST(GdbLogParser, OneLineLogFile_NotRamOrRegisters_ShouldReturnNoRegions)
{
    static const char* xmlForEmptyRegions = "<?xml version=\"1.0\"?>"
                                            "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                            "<memory-map>"
                                            "</memory-map>";
    static const char* testLines[] = { "56	GPIO leds[5] = {" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForEmptyRegions, pMemoryLayout);
}

TEST(GdbLogParser, OneLineLogFile_1RamValue_ShouldReturn1WordRegion)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x4\"></memory>"
                                      "</memory-map>";
    static const char* testLines[] = { "0x10000000:\t0x11111111" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
}

TEST(GdbLogParser, OneLineLogFile_1RamValueWithMixedCaseHexDigits_ShouldReturn1WordRegion)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0xABCDEF00\" length=\"0x4\"></memory>"
                                      "</memory-map>";
    static const char* testLines[] = { "0xAbCdEf00:\t0x11111111" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0xABCDEF00));
}

TEST(GdbLogParser, OneLineLogFile_4RamValues_ShouldReturn4WordRegion)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x10\"></memory>"
                                      "</memory-map>";
    static const char* testLines[] = { "0x10000000:\t0x11111111\t0x22222222\t0x33333333\t0x44444444" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_pMem, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_pMem, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_pMem, 0x1000000c));
}

TEST(GdbLogParser, OneLineLogFile_AddressSymbolAnd4RamValues_ShouldReturn4WordRegion)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x10\"></memory>"
                                      "</memory-map>";
    static const char* testLines[] = { "0x10000000 <impure_data>:\t0x11111111\t0x22222222\t0x33333333\t0x44444444" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_pMem, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_pMem, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_pMem, 0x1000000c));
}

TEST(GdbLogParser, OneLineLogFile_5RamValues_ShouldReturn4WordRegion)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x10\"></memory>"
                                      "</memory-map>";
    static const char* testLines[] = { "0x10000000:\t0x11111111\t0x22222222\t0x33333333\t0x44444444\t55555555" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_pMem, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_pMem, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_pMem, 0x1000000c));
}

TEST(GdbLogParser, OneLineLogFile_4RamValuesWithSymbols_ShouldReturn4WordRegion)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x10\"></memory>"
                                      "</memory-map>";
    static const char* testLines[] = { "0x10000000:\t0x11111111 <symbol>\t0x22222222 <<symbol>>\t0x33333333<<<symbol>>>\t0x44444444<<<<symbol>>>>" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_pMem, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_pMem, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_pMem, 0x1000000c));
}

TEST(GdbLogParser, TwoLineLogFile_8RamValues_ShouldReturn8WordRegion)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x20\"></memory>"
                                      "</memory-map>";
    static const char* testLines[] = { "0x10000000:\t0x11111111\t0x22222222\t0x33333333\t0x44444444",
                                       "0x10000010:\t0x55555555\t0x66666666\t0x77777777\t0x88888888" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_pMem, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_pMem, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_pMem, 0x1000000c));
    CHECK_EQUAL(0x55555555, IMemory_Read32(m_pMem, 0x10000010));
    CHECK_EQUAL(0x66666666, IMemory_Read32(m_pMem, 0x10000014));
    CHECK_EQUAL(0x77777777, IMemory_Read32(m_pMem, 0x10000018));
    CHECK_EQUAL(0x88888888, IMemory_Read32(m_pMem, 0x1000001c));
}

TEST(GdbLogParser, TwoLineLogFile_8RamValues_ShouldReturnTwo4WordRegions)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x10\"></memory>"
                                      "<memory type=\"ram\" start=\"0x20000000\" length=\"0x10\"></memory>"
                                      "</memory-map>";
    static const char* testLines[] = { "0x10000000:\t0x11111111\t0x22222222\t0x33333333\t0x44444444",
                                       "0x20000000:\t0x55555555\t0x66666666\t0x77777777\t0x88888888" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_pMem, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_pMem, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_pMem, 0x1000000c));
    CHECK_EQUAL(0x55555555, IMemory_Read32(m_pMem, 0x20000000));
    CHECK_EQUAL(0x66666666, IMemory_Read32(m_pMem, 0x20000004));
    CHECK_EQUAL(0x77777777, IMemory_Read32(m_pMem, 0x20000008));
    CHECK_EQUAL(0x88888888, IMemory_Read32(m_pMem, 0x2000000c));
}

TEST(GdbLogParser, FailMemoryAllocationForRegion_ShouldThrow)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "</memory-map>";
    static const char* testLines[] = { "0x10000000:\t0x11111111" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
    MallocFailureInject_FailAllocation(1);
        __try_and_catch( GdbLogParse(m_pMem, &m_actualRegisters, "foo.log") );
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(outOfMemoryException, getExceptionCode());
    clearExceptionCode();
}

TEST(GdbLogParser, OneLineLogFile_JustR0Register_ShouldReturnNoRegionsAndSetR0)
{
    static const char* xmlForEmptyRegions = "<?xml version=\"1.0\"?>"
                                            "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                            "<memory-map>"
                                            "</memory-map>";
    static const char* testLines[] = { "r0             0x11111111\t286331153" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForEmptyRegions, pMemoryLayout);
    m_expectedRegisters.R[R0] = 0x11111111;
}

TEST(GdbLogParser, HaveAllRegisters_ShouldReturnNoRegionsAndSetAllRegisters)
{
    static const char* xmlForEmptyRegions = "<?xml version=\"1.0\"?>"
                                            "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                            "<memory-map>"
                                            "</memory-map>";
    static const char* testLines[] = { "r0             0x00000000\t0",
                                       "r1             0x11111111",
                                       "r2             0x22222222",
                                       "r3             0x33333333",
                                       "r4             0x44444444",
                                       "r5             0x55555555",
                                       "r6             0x66666666",
                                       "r7             0x77777777",
                                       "r8             0x88888888",
                                       "r9             0x99999999",
                                       "r10            0xAAAAAAAA",
                                       "r11            0xbbbbbbbb",
                                       "r12            0xcccccccc",
                                       "sp             0xdddddddd",
                                       "lr             0xeeeeeeee",
                                       "pc             0xffffffff",
                                       "xpsr           0xf00df00d",
                                       "msp            0x45454545",
                                       "psp            0x54545454"};

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForEmptyRegions, pMemoryLayout);
    m_expectedRegisters.R[R0]  = 0x0;
    m_expectedRegisters.R[R1]  = 0x11111111;
    m_expectedRegisters.R[R2]  = 0x22222222;
    m_expectedRegisters.R[R3]  = 0x33333333;
    m_expectedRegisters.R[R4]  = 0x44444444;
    m_expectedRegisters.R[R5]  = 0x55555555;
    m_expectedRegisters.R[R6]  = 0x66666666;
    m_expectedRegisters.R[R7]  = 0x77777777;
    m_expectedRegisters.R[R8]  = 0x88888888;
    m_expectedRegisters.R[R9]  = 0x99999999;
    m_expectedRegisters.R[R10] = 0xAAAAAAAA;
    m_expectedRegisters.R[R11] = 0xBBBBBBBB;
    m_expectedRegisters.R[R12] = 0xCCCCCCCC;
    m_expectedRegisters.R[SP]  = 0xDDDDDDDD;
    m_expectedRegisters.R[LR]  = 0xEEEEEEEE;
    m_expectedRegisters.R[PC]  = 0xFFFFFFFF;
    m_expectedRegisters.R[XPSR] = 0xF00DF00D;
    m_expectedRegisters.R[MSP] = 0x45454545;
    m_expectedRegisters.R[PSP] = 0x54545454;
}

TEST(GdbLogParser, HaveAllRegistersAndTwoMemoryBanks_ShouldReturnTwoRegionsAndSetAllRegisters)
{
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x10\"></memory>"
                                      "<memory type=\"ram\" start=\"0x20000000\" length=\"0x10\"></memory>"
                                      "</memory-map>";
    static const char* testLines[] = { "0x10000000:\t0x11111111\t0x22222222\t0x33333333\t0x44444444",
                                       "0x20000000:\t0x55555555\t0x66666666\t0x77777777\t0x88888888",
                                       "r0             0x00000000\t0",
                                       "r1             0x11111111",
                                       "r2             0x22222222",
                                       "r3             0x33333333",
                                       "r4             0x44444444",
                                       "r5             0x55555555",
                                       "r6             0x66666666",
                                       "r7             0x77777777",
                                       "r8             0x88888888",
                                       "r9             0x99999999",
                                       "r10            0xAAAAAAAA",
                                       "r11            0xbbbbbbbb",
                                       "r12            0xcccccccc",
                                       "sp             0xdddddddd",
                                       "lr             0xeeeeeeee",
                                       "pc             0xffffffff",
                                       "xpsr           0xf00df00d",
                                       "msp            0x45454545",
                                       "psp            0x54545454"};


    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_pMem, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_pMem, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_pMem, 0x1000000c));
    CHECK_EQUAL(0x55555555, IMemory_Read32(m_pMem, 0x20000000));
    CHECK_EQUAL(0x66666666, IMemory_Read32(m_pMem, 0x20000004));
    CHECK_EQUAL(0x77777777, IMemory_Read32(m_pMem, 0x20000008));
    CHECK_EQUAL(0x88888888, IMemory_Read32(m_pMem, 0x2000000c));
    m_expectedRegisters.R[R0]  = 0x0;
    m_expectedRegisters.R[R1]  = 0x11111111;
    m_expectedRegisters.R[R2]  = 0x22222222;
    m_expectedRegisters.R[R3]  = 0x33333333;
    m_expectedRegisters.R[R4]  = 0x44444444;
    m_expectedRegisters.R[R5]  = 0x55555555;
    m_expectedRegisters.R[R6]  = 0x66666666;
    m_expectedRegisters.R[R7]  = 0x77777777;
    m_expectedRegisters.R[R8]  = 0x88888888;
    m_expectedRegisters.R[R9]  = 0x99999999;
    m_expectedRegisters.R[R10] = 0xAAAAAAAA;
    m_expectedRegisters.R[R11] = 0xBBBBBBBB;
    m_expectedRegisters.R[R12] = 0xCCCCCCCC;
    m_expectedRegisters.R[SP]  = 0xDDDDDDDD;
    m_expectedRegisters.R[LR]  = 0xEEEEEEEE;
    m_expectedRegisters.R[PC]  = 0xFFFFFFFF;
    m_expectedRegisters.R[XPSR] = 0xF00DF00D;
    m_expectedRegisters.R[MSP] = 0x45454545;
    m_expectedRegisters.R[PSP] = 0x54545454;
}

TEST(GdbLogParser, HaveAllIntegerAndFloatingPointRegisters_ShouldReturnNoRegionsAndSetAllRegisters)
{
    static const char* xmlForEmptyRegions = "<?xml version=\"1.0\"?>"
                                            "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                            "<memory-map>"
                                            "</memory-map>";
    static const char* testLines[] = { "r0             0x00000000\t0",
                                       "r1             0x11111111",
                                       "r2             0x22222222",
                                       "r3             0x33333333",
                                       "r4             0x44444444",
                                       "r5             0x55555555",
                                       "r6             0x66666666",
                                       "r7             0x77777777",
                                       "r8             0x88888888",
                                       "r9             0x99999999",
                                       "r10            0xAAAAAAAA",
                                       "r11            0xbbbbbbbb",
                                       "r12            0xcccccccc",
                                       "sp             0xdddddddd",
                                       "lr             0xeeeeeeee",
                                       "pc             0xffffffff",
                                       "xpsr           0xf00df00d",
                                       "msp            0x45454545",
                                       "psp            0x54545454",
                                       "fpscr          0x1	1",
                                       "s0             0	(raw 0x00000000)",
                                       "s1             1	(raw 0x3f800000)",
                                       "s2             2	(raw 0x40000000)",
                                       "s3             3	(raw 0x40400000)",
                                       "s4             4	(raw 0x40800000)",
                                       "s5             5	(raw 0x40a00000)",
                                       "s6             6	(raw 0x40c00000)",
                                       "s7             7	(raw 0x40e00000)",
                                       "s8             8	(raw 0x41000000)",
                                       "s9             9	(raw 0x41100000)",
                                       "s10            10	(raw 0x41200000)",
                                       "s11            11	(raw 0x41300000)",
                                       "s12            12	(raw 0x41400000)",
                                       "s13            13	(raw 0x41500000)",
                                       "s14            14	(raw 0x41600000)",
                                       "s15            15	(raw 0x41700000)",
                                       "s16            16	(raw 0x41800000)",
                                       "s17            17	(raw 0x41880000)",
                                       "s18            18	(raw 0x41900000)",
                                       "s19            19	(raw 0x41980000)",
                                       "s20            20	(raw 0x41a00000)",
                                       "s21            21	(raw 0x41a80000)",
                                       "s22            22	(raw 0x41b00000)",
                                       "s23            23	(raw 0x41b80000)",
                                       "s24            24	(raw 0x41c00000)",
                                       "s25            25	(raw 0x41c80000)",
                                       "s26            26	(raw 0x41d00000)",
                                       "s27            27	(raw 0x41d80000)",
                                       "s28            28	(raw 0x41e00000)",
                                       "s29            29	(raw 0x41e80000)",
                                       "s30            30	(raw 0x41f00000)",
                                       "s31            31	(raw 0x41f80000)"
                                       };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForEmptyRegions, pMemoryLayout);
    m_expectedRegisters.flags = CRASH_CATCHER_FLAGS_FLOATING_POINT;
    for (uint32_t i = 0 ; i < 16 ; i++)
        m_expectedRegisters.R[i] = 0x11111111 * i;
    m_expectedRegisters.R[XPSR] = 0xF00DF00D;
    m_expectedRegisters.R[MSP] = 0x45454545;
    m_expectedRegisters.R[PSP] = 0x54545454;
    for (int i = 0 ; i < 32 ; i++)
    {
        float    floatVal = (float)i;
        uint32_t val;
        memcpy(&val, &floatVal, sizeof(val));
        m_expectedRegisters.FPR[i] = val;
    }
    m_expectedRegisters.FPR[FPSCR] = 1;
}

TEST(GdbLogParser, FloatingPointValueWithMissingRawValue_ShouldSetRegisterToNegativeOne)
{
    static const char* xmlForEmptyRegions = "<?xml version=\"1.0\"?>"
                                            "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                            "<memory-map>"
                                            "</memory-map>";
    static const char* testLines[] = { "s0             55" };

    fgetsSetData(testLines, ARRAY_SIZE(testLines));
        GdbLogParse(m_pMem, &m_actualRegisters, "foo.log");
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForEmptyRegions, pMemoryLayout);
    m_expectedRegisters.flags = CRASH_CATCHER_FLAGS_FLOATING_POINT;
    m_expectedRegisters.FPR[0] = -1;
}
