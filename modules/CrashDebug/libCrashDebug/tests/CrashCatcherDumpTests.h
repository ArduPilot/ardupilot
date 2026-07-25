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
/* CrashCatcherBinaryDumpTests.cpp and CrashCatcherHexDumpTests.cpp include this source file so that they can
   share tests by just creating differently formatted test files and using a different function to load them.
*/


DUMP_TEST(InvalidLogFilename_ShouldThrowFileException)
{
    fopenSetReturn(NULL);
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, "invalidDumpFilename.dmp") );
    CHECK_EQUAL(fileException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("Failed to open the \"invalidDumpFilename.dmp\" dump file.", getExceptionMessage());
}

DUMP_TEST(InvalidSignature_ShouldThrowFileFormatException)
{
    m_fileTop.signature[3] += 1;
    createTestDumpFile(&m_fileTop, sizeof(m_fileTop));
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("The dump file didn't start with the expected 4-byte signature.", getExceptionMessage());
}

DUMP_TEST(FailSignatureRead_ShouldThrowFileFormatException)
{
    createTestDumpFile(&m_fileTop, sizeof(m_fileTop));
    freadFail(-1);
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("The dump file was too short to contain the 4-byte signature.", getExceptionMessage());
}

DUMP_TEST(FileTooShortForFlags_ShouldThrowFileFormatException)
{
    createTestDumpFile(&m_fileTop, offsetof(DumpFileTop, R) - 1);
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("The dump file was too short to contain the flags.", getExceptionMessage());
}

DUMP_TEST(FileTooShortForExceptionPSR_ShouldThrowFileFormatException)
{
    createTestDumpFile(&m_fileTop, sizeof(m_fileTop) - 1);
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("The dump file was too short to contain the exception PSR.", getExceptionMessage());
}

DUMP_TEST(FileTooShortForIntegerRegisters_ShouldThrowFileFormatException)
{
    createTestDumpFile(&m_fileTop, sizeof(m_fileTop) - 5);
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("The dump file was too short to contain the integer registers.", getExceptionMessage());
}

DUMP_TEST(DumpContainingRegistersOnly_VerifyRegistersReadAndEmptyMemoryRegions)
{
    for (uint32_t i = 0 ; i < 16 ; i++)
        m_fileTop.R[i] = i * 0x11111111;
    m_fileTop.R[XPSR] = 0xF00DF00D;
    m_fileTop.R[MSP] = 0x45454545;
    m_fileTop.R[PSP] = 0x54545454;
    m_fileTop.exceptionPSR =  0xBAADFEED;
    createTestDumpFile(&m_fileTop, sizeof(m_fileTop));
        CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename);
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

DUMP_TEST(Version2DumpContainingRegistersOnly_VerifyRegistersReadDefaultMSPandPSPAndEmptyMemoryRegions)
{
    for (uint32_t i = 0 ; i < 16 ; i++)
        m_fileTopVersion2.R[i] = i * 0x11111111;
    m_fileTopVersion2.R[XPSR] = 0xF00DF00D;
    m_fileTopVersion2.exceptionPSR =  0xBAADFEED;
    createTestDumpFile(&m_fileTopVersion2, sizeof(m_fileTopVersion2));
        CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename);
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
    m_expectedRegisters.R[MSP]           = DEFAULT_SP_VALUE;
    m_expectedRegisters.R[PSP]           = DEFAULT_SP_VALUE;
    m_expectedRegisters.exceptionPSR     = 0xBAADFEED;
}

DUMP_TEST(DumpContainingFloatRegistersToo_VerifyRegistersReadAndEmptyMemoryRegions)
{
    DumpFPFileTop fileData;

    memcpy(&fileData, &m_fileTop, sizeof(m_fileTop));
    for (uint32_t i = 0 ; i < 16 ; i++)
        fileData.R[i] = i * 0x11111111;
    fileData.R[XPSR] = 0xF00DF00D;
    fileData.R[MSP] = 0x45454545;
    fileData.R[PSP] = 0x54545454;
    fileData.exceptionPSR =  0xBAADFEED;
    fileData.flags = CRASH_CATCHER_FLAGS_FLOATING_POINT;
    for (uint32_t i = 0 ; i < 32 ; i++)
        fileData.FPR[i] = i;
    fileData.FPR[FPSCR] = 0xBAADF00D;
    createTestDumpFile(&fileData, sizeof(fileData));
        CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename);
    validateNoMemoryRegionsCreated();
    m_expectedRegisters.flags            = CRASH_CATCHER_FLAGS_FLOATING_POINT;
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
    m_expectedRegisters.FPR[S0]          = 0;
    m_expectedRegisters.FPR[S1]          = 1;
    m_expectedRegisters.FPR[S2]          = 2;
    m_expectedRegisters.FPR[S3]          = 3;
    m_expectedRegisters.FPR[S4]          = 4;
    m_expectedRegisters.FPR[S5]          = 5;
    m_expectedRegisters.FPR[S6]          = 6;
    m_expectedRegisters.FPR[S7]          = 7;
    m_expectedRegisters.FPR[S8]          = 8;
    m_expectedRegisters.FPR[S9]          = 9;
    m_expectedRegisters.FPR[S10]         = 10;
    m_expectedRegisters.FPR[S11]         = 11;
    m_expectedRegisters.FPR[S12]         = 12;
    m_expectedRegisters.FPR[S13]         = 13;
    m_expectedRegisters.FPR[S14]         = 14;
    m_expectedRegisters.FPR[S15]         = 15;
    m_expectedRegisters.FPR[S16]         = 16;
    m_expectedRegisters.FPR[S17]         = 17;
    m_expectedRegisters.FPR[S18]         = 18;
    m_expectedRegisters.FPR[S19]         = 19;
    m_expectedRegisters.FPR[S20]         = 20;
    m_expectedRegisters.FPR[S21]         = 21;
    m_expectedRegisters.FPR[S22]         = 22;
    m_expectedRegisters.FPR[S23]         = 23;
    m_expectedRegisters.FPR[S24]         = 24;
    m_expectedRegisters.FPR[S25]         = 25;
    m_expectedRegisters.FPR[S26]         = 26;
    m_expectedRegisters.FPR[S27]         = 27;
    m_expectedRegisters.FPR[S28]         = 28;
    m_expectedRegisters.FPR[S29]         = 29;
    m_expectedRegisters.FPR[S30]         = 30;
    m_expectedRegisters.FPR[S31]         = 31;
    m_expectedRegisters.FPR[FPSCR]       = 0xBAADF00D;
}

DUMP_TEST(Version2DumpContainingFloatRegistersToo_VerifyRegistersDefaultsForMSPandPSPReadAndEmptyMemoryRegions)
{
    DumpFPFileTopVersion2 fileData;

    memcpy(&fileData, &m_fileTopVersion2, sizeof(m_fileTop));
    for (uint32_t i = 0 ; i < 16 ; i++)
        fileData.R[i] = i * 0x11111111;
    fileData.R[XPSR] = 0xF00DF00D;
    fileData.exceptionPSR =  0xBAADFEED;
    fileData.flags = CRASH_CATCHER_FLAGS_FLOATING_POINT;
    for (uint32_t i = 0 ; i < 32 ; i++)
        fileData.FPR[i] = i;
    fileData.FPR[FPSCR] = 0xBAADF00D;
    createTestDumpFile(&fileData, sizeof(fileData));
        CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename);
    validateNoMemoryRegionsCreated();
    m_expectedRegisters.flags            = CRASH_CATCHER_FLAGS_FLOATING_POINT;
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
    m_expectedRegisters.R[MSP]           = DEFAULT_SP_VALUE;
    m_expectedRegisters.R[PSP]           = DEFAULT_SP_VALUE;
    m_expectedRegisters.exceptionPSR     = 0xBAADFEED;
    m_expectedRegisters.FPR[S0]          = 0;
    m_expectedRegisters.FPR[S1]          = 1;
    m_expectedRegisters.FPR[S2]          = 2;
    m_expectedRegisters.FPR[S3]          = 3;
    m_expectedRegisters.FPR[S4]          = 4;
    m_expectedRegisters.FPR[S5]          = 5;
    m_expectedRegisters.FPR[S6]          = 6;
    m_expectedRegisters.FPR[S7]          = 7;
    m_expectedRegisters.FPR[S8]          = 8;
    m_expectedRegisters.FPR[S9]          = 9;
    m_expectedRegisters.FPR[S10]         = 10;
    m_expectedRegisters.FPR[S11]         = 11;
    m_expectedRegisters.FPR[S12]         = 12;
    m_expectedRegisters.FPR[S13]         = 13;
    m_expectedRegisters.FPR[S14]         = 14;
    m_expectedRegisters.FPR[S15]         = 15;
    m_expectedRegisters.FPR[S16]         = 16;
    m_expectedRegisters.FPR[S17]         = 17;
    m_expectedRegisters.FPR[S18]         = 18;
    m_expectedRegisters.FPR[S19]         = 19;
    m_expectedRegisters.FPR[S20]         = 20;
    m_expectedRegisters.FPR[S21]         = 21;
    m_expectedRegisters.FPR[S22]         = 22;
    m_expectedRegisters.FPR[S23]         = 23;
    m_expectedRegisters.FPR[S24]         = 24;
    m_expectedRegisters.FPR[S25]         = 25;
    m_expectedRegisters.FPR[S26]         = 26;
    m_expectedRegisters.FPR[S27]         = 27;
    m_expectedRegisters.FPR[S28]         = 28;
    m_expectedRegisters.FPR[S29]         = 29;
    m_expectedRegisters.FPR[S30]         = 30;
    m_expectedRegisters.FPR[S31]         = 31;
    m_expectedRegisters.FPR[FPSCR]       = 0xBAADF00D;
}

DUMP_TEST(FileTooShortForFloatRegisters_ShouldThrowFileFormatException)
{
    DumpFPFileTop fileData;
    memset(&fileData, 0, sizeof(fileData));
    memcpy(&fileData, &m_fileTop, sizeof(m_fileTop));
    fileData.flags = CRASH_CATCHER_FLAGS_FLOATING_POINT;

    createTestDumpFile(&fileData, sizeof(fileData) - 1);
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    m_expectedRegisters.flags = CRASH_CATCHER_FLAGS_FLOATING_POINT;
    STRCMP_EQUAL("The dump file was too short to contain the floating point registers.", getExceptionMessage());
}

DUMP_TEST(DumpContainingOneMemoryRegionOf1Word_VerifyMemoryContents)
{
    struct FileData
    {
        DumpFileTop                  fileTop;
        CrashCatcherMemoryRegionInfo region1;
        uint32_t                     region1Data[1];
    } fileData;
    memcpy(&fileData.fileTop, &m_fileTop, sizeof(m_fileTop));
    fileData.region1.startAddress = 0x10000000;
    fileData.region1.endAddress = 0x10000004;
    fileData.region1Data[0] = 0x11111111;
    createTestDumpFile(&fileData, sizeof(fileData));
        CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename);
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x4\"></memory>"
                                      "</memory-map>";
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
}

DUMP_TEST(FailAllocationForRegion_VerifyEmptyMemoryAndExceptionThrown)
{
    struct FileData
    {
        DumpFileTop                  fileTop;
        CrashCatcherMemoryRegionInfo region1;
        uint32_t                     region1Data[1];
    } fileData;
    memcpy(&fileData.fileTop, &m_fileTop, sizeof(m_fileTop));
    fileData.region1.startAddress = 0x10000000;
    fileData.region1.endAddress = 0x10000004;
    fileData.region1Data[0] = 0x11111111;
    createTestDumpFile(&fileData, sizeof(fileData));
    MallocFailureInject_FailAllocation(1);
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(outOfMemoryException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("The dump file failed to load RAM memory region at 0x10000000 - 0x10000004.", getExceptionMessage());
    validateNoMemoryRegionsCreated();
}

DUMP_TEST(DumpContainingTruncatedMemoryRegionDescription_ExceptionShouldBeThrownAndNoMemoryRegionsCreated)
{
    struct FileData
    {
        DumpFileTop                  fileTop;
        CrashCatcherMemoryRegionInfo region1;
        uint32_t                     region1Data[1];
    } fileData;
    memcpy(&fileData.fileTop, &m_fileTop, sizeof(m_fileTop));
    fileData.region1.startAddress = 0x10000000;
    fileData.region1.endAddress = 0x10000004;
    fileData.region1Data[0] = 0x11111111;
    createTestDumpFile(&fileData, sizeof(fileData) - sizeof(uint32_t) - 1);
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("The dump file contained a truncated memory region header.", getExceptionMessage());
    validateNoMemoryRegionsCreated();
}

DUMP_TEST(DumpContainingTruncatedMemoryRegionData_ExceptionShouldBeThrown_PartiallyCompletedMemoryRegion)
{
    struct FileData
    {
        DumpFileTop                  fileTop;
        CrashCatcherMemoryRegionInfo region1;
        uint32_t                     region1Data[1];
    } fileData;
    memcpy(&fileData.fileTop, &m_fileTop, sizeof(m_fileTop));
    fileData.region1.startAddress = 0x10000000;
    fileData.region1.endAddress = 0x10000004;
    fileData.region1Data[0] = 0x11111111;
    createTestDumpFile(&fileData, sizeof(fileData) - 1);
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(fileFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("The dump file failed to load RAM memory region at 0x10000000 - 0x10000004.", getExceptionMessage());
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x4\"></memory>"
                                      "</memory-map>";
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x00111111, IMemory_Read32(m_pMem, 0x10000000));
}

DUMP_TEST(DumpContainingTwoMemoryRegionsOf1Word_VerifyMemoryContents)
{
    struct FileData
    {
        DumpFileTop                  fileTop;
        CrashCatcherMemoryRegionInfo region1;
        uint32_t                     region1Data[1];
        CrashCatcherMemoryRegionInfo region2;
        uint32_t                     region2Data[1];
    } fileData;
    memcpy(&fileData.fileTop, &m_fileTop, sizeof(m_fileTop));
    fileData.region1.startAddress = 0x10000000;
    fileData.region1.endAddress = 0x10000004;
    fileData.region1Data[0] = 0x11111111;
    fileData.region2.startAddress = 0x20000000;
    fileData.region2.endAddress = 0x20000004;
    fileData.region2Data[0] = 0x22222222;
    createTestDumpFile(&fileData, sizeof(fileData));
        CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename);
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x4\"></memory>"
                                      "<memory type=\"ram\" start=\"0x20000000\" length=\"0x4\"></memory>"
                                      "</memory-map>";
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_pMem, 0x20000000));
}

DUMP_TEST(DumpContainingOneMemoryRegionsFollowedByStackOverflowError_VerifyMemoryContentsAndExceptionThrown)
{
    struct FileData
    {
        DumpFileTop                  fileTop;
        CrashCatcherMemoryRegionInfo region1;
        uint32_t                     region1Data[1];
        uint32_t                     overflowSentinel;
    } fileData;
    memcpy(&fileData.fileTop, &m_fileTop, sizeof(m_fileTop));
    fileData.region1.startAddress = 0x10000000;
    fileData.region1.endAddress = 0x10000004;
    fileData.region1Data[0] = 0x11111111;
    fileData.overflowSentinel = CRASH_CATCHER_STACK_SENTINEL;
    createTestDumpFile(&fileData, sizeof(fileData));
        __try_and_catch( CRASH_CATCHER_DUMP_READ_FUNC(m_pMem, &m_actualRegisters, m_pTestFilename) );
    CHECK_EQUAL(stackOverflowException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("The dump file ended with an indication that CrashCatcher detected a stack overflow.", getExceptionMessage());
    static const char* xmlForMemory = "<?xml version=\"1.0\"?>"
                                      "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                      "<memory-map>"
                                      "<memory type=\"ram\" start=\"0x10000000\" length=\"0x4\"></memory>"
                                      "</memory-map>";
    const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
    STRCMP_EQUAL(xmlForMemory, pMemoryLayout);
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_pMem, 0x10000000));
}
