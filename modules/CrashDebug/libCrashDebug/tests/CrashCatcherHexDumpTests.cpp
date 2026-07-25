/*  Copyright (C) 2017  Adam Green (https://github.com/adamgreen)

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


TEST_GROUP_BASE(CrashCatcherHexDump, DumpBaseTest)
{
    void setup()
    {
        m_pTestFilename = "CrashCatcherHexDumpTest.dmp";
        DumpBaseTest::setup();
    }

    void teardown()
    {
        DumpBaseTest::teardown();
    }

    void createTestDumpFile(const void* pData, size_t dataSize)
    {
        static const char nibbleToHex[] = "0123456789ABCDEF";
        const uint8_t* pCurr = (uint8_t*)pData;
        FILE* pFile = fopen(m_pTestFilename, "w");
        while (dataSize-- > 0)
        {
            uint8_t currByte = *pCurr++;
            char    nibble = nibbleToHex[currByte >> 4];
            fwrite(&nibble, 1, 1, pFile);
            nibble = nibbleToHex[currByte & 0xF];
            fwrite(&nibble, 1, 1, pFile);
        }
        fclose(pFile);
    }
};


// Pull in shared binary/hex shared tests from header file.
#define DUMP_TEST(X) TEST(CrashCatcherHexDump, X)
#define CRASH_CATCHER_DUMP_READ_FUNC CrashCatcherDump_ReadHex

#include "CrashCatcherDumpTests.h"


// Tests that are specific to HexDump logs.
TEST(CrashCatcherHexDump, DumpContainingNewlinesBetweenHexDigits_VerifyRegistersReadAndEmptyMemoryRegions)
{
    static const char testHexDump[] = "63430300\r\n"
                                      "00000000\r\n"
                                      "00000000111111112222222233333333\n"
                                      "44444444555555556666666677777777\r"
                                      "8888888899999999AAAAAAAABBBBBBBB\n\r"
                                      "CCCCCCCCDDDDDDDDEEEEEEEEFFFFFFFF\r\n"
                                      "0DF00DF04545454554545454EDFEADBA";
    FILE* pFile = fopen(m_pTestFilename, "w");
    fwrite(testHexDump, 1, sizeof(testHexDump) - 1, pFile);
    fclose(pFile);
        CrashCatcherDump_ReadHex(m_pMem, &m_actualRegisters, m_pTestFilename);
    validateNoMemoryRegionsCreated();
    m_expectedRegisters.R[R0]            = 0x0;
    m_expectedRegisters.R[R1]            = 0x11111111;
    m_expectedRegisters.R[R2]            = 0x22222222;
    m_expectedRegisters.R[R3]            = 0x33333333;
    m_expectedRegisters.R[R4]            = 0x44444444;
    m_expectedRegisters.R[R5]            = 0x55555555;
    m_expectedRegisters.R[R6]            = 0x66666666;
    m_expectedRegisters.R[R7]            = 0x77777777;
    m_expectedRegisters.R[R8]            = 0x88888888;
    m_expectedRegisters.R[R9]            = 0x99999999;
    m_expectedRegisters.R[R10]           = 0xAAAAAAAA;
    m_expectedRegisters.R[R11]           = 0xBBBBBBBB;
    m_expectedRegisters.R[R12]           = 0xCCCCCCCC;
    m_expectedRegisters.R[SP]            = 0xDDDDDDDD;
    m_expectedRegisters.R[LR]            = 0xEEEEEEEE;
    m_expectedRegisters.R[PC]            = 0xFFFFFFFF;
    m_expectedRegisters.R[XPSR]          = 0xF00DF00D;
    m_expectedRegisters.R[MSP]           = 0x45454545;
    m_expectedRegisters.R[PSP]           = 0x54545454;
    m_expectedRegisters.exceptionPSR     = 0xBAADFEED;
}

TEST(CrashCatcherHexDump, SameTestAsPreviousButWithLowercaseHexDigits)
{
    static const char testHexDump[] = "63430300\r\n"
                                      "00000000\r\n"
                                      "00000000111111112222222233333333\n"
                                      "44444444555555556666666677777777\r"
                                      "8888888899999999aaaaaaaabbbbbbbb\n\r"
                                      "ccccccccddddddddeeeeeeeeffffffff\r\n"
                                      "0df00df04545454554545454edfeadba";
    FILE* pFile = fopen(m_pTestFilename, "w");
    fwrite(testHexDump, 1, sizeof(testHexDump) - 1, pFile);
    fclose(pFile);
        CrashCatcherDump_ReadHex(m_pMem, &m_actualRegisters, m_pTestFilename);
    validateNoMemoryRegionsCreated();
    m_expectedRegisters.R[R0]            = 0x0;
    m_expectedRegisters.R[R1]            = 0x11111111;
    m_expectedRegisters.R[R2]            = 0x22222222;
    m_expectedRegisters.R[R3]            = 0x33333333;
    m_expectedRegisters.R[R4]            = 0x44444444;
    m_expectedRegisters.R[R5]            = 0x55555555;
    m_expectedRegisters.R[R6]            = 0x66666666;
    m_expectedRegisters.R[R7]            = 0x77777777;
    m_expectedRegisters.R[R8]            = 0x88888888;
    m_expectedRegisters.R[R9]            = 0x99999999;
    m_expectedRegisters.R[R10]           = 0xAAAAAAAA;
    m_expectedRegisters.R[R11]           = 0xBBBBBBBB;
    m_expectedRegisters.R[R12]           = 0xCCCCCCCC;
    m_expectedRegisters.R[SP]            = 0xDDDDDDDD;
    m_expectedRegisters.R[LR]            = 0xEEEEEEEE;
    m_expectedRegisters.R[PC]            = 0xFFFFFFFF;
    m_expectedRegisters.R[XPSR]          = 0xF00DF00D;
    m_expectedRegisters.R[MSP]           = 0x45454545;
    m_expectedRegisters.R[PSP]           = 0x54545454;
    m_expectedRegisters.exceptionPSR     = 0xBAADFEED;
}

TEST(CrashCatcherHexDump, DumpContainingOneTooFewHexDigits_VerifyExceptionThrown)
{
    static const char testHexDump[] = "63430300\r\n"
                                      "00000000\r\n"
                                      "00000000111111112222222233333333\n"
                                      "44444444555555556666666677777777\r"
                                      "8888888899999999AAAAAAAABBBBBBBB\n\r"
                                      "CCCCCCCCDDDDDDDDEEEEEEEEFFFFFFFF\r\n"
                                      "0DF00DF04545454554545454EDFEADBA";
    FILE* pFile = fopen(m_pTestFilename, "w");
    fwrite(testHexDump, 1, sizeof(testHexDump) - 2, pFile);
    fclose(pFile);
        __try_and_catch( CrashCatcherDump_ReadHex(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    validateNoMemoryRegionsCreated();
    m_expectedRegisters.R[R0]            = 0x0;
    m_expectedRegisters.R[R1]            = 0x11111111;
    m_expectedRegisters.R[R2]            = 0x22222222;
    m_expectedRegisters.R[R3]            = 0x33333333;
    m_expectedRegisters.R[R4]            = 0x44444444;
    m_expectedRegisters.R[R5]            = 0x55555555;
    m_expectedRegisters.R[R6]            = 0x66666666;
    m_expectedRegisters.R[R7]            = 0x77777777;
    m_expectedRegisters.R[R8]            = 0x88888888;
    m_expectedRegisters.R[R9]            = 0x99999999;
    m_expectedRegisters.R[R10]           = 0xAAAAAAAA;
    m_expectedRegisters.R[R11]           = 0xBBBBBBBB;
    m_expectedRegisters.R[R12]           = 0xCCCCCCCC;
    m_expectedRegisters.R[SP]            = 0xDDDDDDDD;
    m_expectedRegisters.R[LR]            = 0xEEEEEEEE;
    m_expectedRegisters.R[PC]            = 0xFFFFFFFF;
    m_expectedRegisters.R[XPSR]          = 0xF00DF00D;
    m_expectedRegisters.R[MSP]           = 0x45454545;
    m_expectedRegisters.R[PSP]           = 0x54545454;
    m_expectedRegisters.exceptionPSR     = 0x00ADFEED;
}

TEST(CrashCatcherHexDump, DumpContainingNonHexDigitInLastByte_VerifyExceptionThrown)
{
    static const char testHexDump[] = "63430300\r\n"
                                      "00000000\r\n"
                                      "00000000111111112222222233333333\n"
                                      "44444444555555556666666677777777\r"
                                      "8888888899999999AAAAAAAABBBBBBBB\n\r"
                                      "CCCCCCCCDDDDDDDDEEEEEEEEFFFFFFFF\r\n"
                                      "0DF00DF04545454554545454EDFEADBG";
    FILE* pFile = fopen(m_pTestFilename, "w");
    fwrite(testHexDump, 1, sizeof(testHexDump) - 1, pFile);
    fclose(pFile);
        __try_and_catch( CrashCatcherDump_ReadHex(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    validateNoMemoryRegionsCreated();
    m_expectedRegisters.R[R0]            = 0x0;
    m_expectedRegisters.R[R1]            = 0x11111111;
    m_expectedRegisters.R[R2]            = 0x22222222;
    m_expectedRegisters.R[R3]            = 0x33333333;
    m_expectedRegisters.R[R4]            = 0x44444444;
    m_expectedRegisters.R[R5]            = 0x55555555;
    m_expectedRegisters.R[R6]            = 0x66666666;
    m_expectedRegisters.R[R7]            = 0x77777777;
    m_expectedRegisters.R[R8]            = 0x88888888;
    m_expectedRegisters.R[R9]            = 0x99999999;
    m_expectedRegisters.R[R10]           = 0xAAAAAAAA;
    m_expectedRegisters.R[R11]           = 0xBBBBBBBB;
    m_expectedRegisters.R[R12]           = 0xCCCCCCCC;
    m_expectedRegisters.R[SP]            = 0xDDDDDDDD;
    m_expectedRegisters.R[LR]            = 0xEEEEEEEE;
    m_expectedRegisters.R[PC]            = 0xFFFFFFFF;
    m_expectedRegisters.R[XPSR]          = 0xF00DF00D;
    m_expectedRegisters.R[MSP]           = 0x45454545;
    m_expectedRegisters.R[PSP]           = 0x54545454;
    m_expectedRegisters.exceptionPSR     = 0x00ADFEED;
}
