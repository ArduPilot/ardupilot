/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#include "string.h"

// Include headers from C modules under test.
extern "C"
{
    #include <common.h>
    #include <ElfPriv.h>
    #include <FileFailureInject.h>
    #include <MallocFailureInject.h>
    #include <CrashDebugCommandLine.h>
    #include <printfSpy.h>
}

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


// This test harness wants access to the actual fgets() routine and not the mock.
#undef fgets


struct ElfFile
{
    Elf32_Ehdr elfHeader;
    Elf32_Phdr pgmHeader;
    uint32_t   data[2];
};


static const char     g_usageString[] = "Usage:";
static const char*    g_imageFilename = "image.bin";
static const char*    g_elfFilename = "image.elf";
static const char*    g_dumpFilenameV2 = "gdb_v2.txt";
static const char*    g_dumpFilenameV3 = "gdb_v3.txt";
static const char*    g_hexDumpFilenameV2 = "crash_v2.txt";
static const char*    g_hexDumpFilenameV3 = "crash_v3.txt";
static const char*    g_binDumpFilenameV2 = "crash_v2.dmp";
static const char*    g_binDumpFilenameV3 = "crash_v3.dmp";
static const uint32_t g_imageData[2] = { 0x10000004, 0x00000100 };
static const char     g_dumpDataV2[] =  "0x10000000:\t0x11111111\t0x22222222\t0x33333333\t0x44444444\n"
                                      "r0             0x5a5a5a5a\t0\n"
                                      "r1             0x11111111\n"
                                      "r2             0x22222222\n"
                                      "r3             0x33333333\n"
                                      "r4             0x44444444\n"
                                      "r5             0x55555555\n"
                                      "r6             0x66666666\n"
                                      "r7             0x77777777\n"
                                      "r8             0x88888888\n"
                                      "r9             0x99999999\n"
                                      "r10            0xAAAAAAAA\n"
                                      "r11            0xbbbbbbbb\n"
                                      "r12            0xcccccccc\n"
                                      "sp             0xdddddddd\n"
                                      "lr             0xeeeeeeee\n"
                                      "pc             0xffffffff\n"
                                      "xpsr           0xf00df00d\n";
static const char     g_dumpDataV3[] =  "0x10000000:\t0x11111111\t0x22222222\t0x33333333\t0x44444444\n"
                                      "r0             0x5a5a5a5a\t0\n"
                                      "r1             0x11111111\n"
                                      "r2             0x22222222\n"
                                      "r3             0x33333333\n"
                                      "r4             0x44444444\n"
                                      "r5             0x55555555\n"
                                      "r6             0x66666666\n"
                                      "r7             0x77777777\n"
                                      "r8             0x88888888\n"
                                      "r9             0x99999999\n"
                                      "r10            0xAAAAAAAA\n"
                                      "r11            0xbbbbbbbb\n"
                                      "r12            0xcccccccc\n"
                                      "sp             0xdddddddd\n"
                                      "lr             0xeeeeeeee\n"
                                      "pc             0xffffffff\n"
                                      "xpsr           0xf00df00d\n"
                                      "msp            0xa5a5a5a5\n"
                                      "psp            0xbcbcbcbc\n";
static const char     g_hexCrashDumpV2[] = "63430200\r\n"
                                         "00000000\r\n"
                                         "5a5a5a5a111111112222222233333333\r\n"
                                         "44444444555555556666666677777777\r\n"
                                         "8888888899999999AAAAAAAABBBBBBBB\r\n"
                                         "CCCCCCCC\r\n"
                                         "DDDDDDDD\r\n"
                                         "EEEEEEEEFFFFFFFF0DF00DF0\r\n"
                                         "03000000\r\n"
                                         "0000001010000010\r\n"
                                         "11111111222222223333333344444444\r\n";
static const char     g_hexCrashDumpV3[] = "63430300\r\n"
                                         "00000000\r\n"
                                         "5a5a5a5a111111112222222233333333\r\n"
                                         "44444444555555556666666677777777\r\n"
                                         "8888888899999999AAAAAAAABBBBBBBB\r\n"
                                         "CCCCCCCC\r\n"
                                         "DDDDDDDD\r\n"
                                         "EEEEEEEEFFFFFFFF0DF00DF0\r\n"
                                         "A5A5A5A5BCBCBCBC03000000\r\n"
                                         "0000001010000010\r\n"
                                         "11111111222222223333333344444444\r\n";
static const char     g_binCrashDumpV2[] = "\x63\x43\x02\x00"
                                         "\x00\x00\x00\x00"
                                         "\x5a\x5a\x5a\x5a\x11\x11\x11\x11\x22\x22\x22\x22\x33\x33\x33\x33"
                                         "\x44\x44\x44\x44\x55\x55\x55\x55\x66\x66\x66\x66\x77\x77\x77\x77"
                                         "\x88\x88\x88\x88\x99\x99\x99\x99\xAA\xAA\xAA\xAA\xBB\xBB\xBB\xBB"
                                         "\xCC\xCC\xCC\xCC"
                                         "\xDD\xDD\xDD\xDD"
                                         "\xEE\xEE\xEE\xEE\xFF\xFF\xFF\xFF\x0D\xF0\x0D\xF0"
                                         "\x03\x00\x00\x00"
                                         "\x00\x00\x00\x10\x10\x00\x00\x10"
                                         "\x11\x11\x11\x11\x22\x22\x22\x22\x33\x33\x33\x33\x44\x44\x44\x44";
static const char     g_binCrashDumpV3[] = "\x63\x43\x03\x00"
                                         "\x00\x00\x00\x00"
                                         "\x5a\x5a\x5a\x5a\x11\x11\x11\x11\x22\x22\x22\x22\x33\x33\x33\x33"
                                         "\x44\x44\x44\x44\x55\x55\x55\x55\x66\x66\x66\x66\x77\x77\x77\x77"
                                         "\x88\x88\x88\x88\x99\x99\x99\x99\xAA\xAA\xAA\xAA\xBB\xBB\xBB\xBB"
                                         "\xCC\xCC\xCC\xCC"
                                         "\xDD\xDD\xDD\xDD"
                                         "\xEE\xEE\xEE\xEE\xFF\xFF\xFF\xFF\x0D\xF0\x0D\xF0"
                                         "\xA5\xA5\xA5\xA5\xBC\xBC\xBC\xBC\x03\x00\x00\x00"
                                         "\x00\x00\x00\x10\x10\x00\x00\x10"
                                         "\x11\x11\x11\x11\x22\x22\x22\x22\x33\x33\x33\x33\x44\x44\x44\x44";




TEST_GROUP(CrashDebugCommandLine)
{
    const char*           m_argv[16];
    CrashDebugCommandLine m_commandLine;
    int                   m_argc;
    RegisterContext       m_expectedRegisters;
    ElfFile               m_elfFile;
    char*                 (*m_fgets)(char * str, int size, FILE * stream);

    void setup()
    {
        memset(m_argv, 0, sizeof(m_argv));
        memset(&m_commandLine, 0xff, sizeof(m_commandLine));
        memset(&m_expectedRegisters, 0x00, sizeof(m_expectedRegisters));
        m_argc = 0;

        unhookFgets();
        printfSpy_Hook(strlen(g_usageString));
    }

    void unhookFgets()
    {
        m_fgets = hook_fgets;
        hook_fgets = fgets;
    }

    void teardown()
    {
        CHECK_EQUAL(noException, getExceptionCode());
        checkRegisters();
        rehookFgets();
        fopenRestore();
        fseekRestore();
        ftellRestore();
        freadRestore();
        fwriteRestore();
        printfSpy_Unhook();
        MallocFailureInject_Restore();
        CrashDebugCommandLine_Uninit(&m_commandLine);
        remove(g_imageFilename);
        remove(g_dumpFilenameV2);
        remove(g_hexDumpFilenameV2);
        remove(g_binDumpFilenameV2);
        remove(g_dumpFilenameV3);
        remove(g_hexDumpFilenameV3);
        remove(g_binDumpFilenameV3);
        remove(g_elfFilename);
    }

    void checkRegisters()
    {
        for (size_t i = 0 ; i < ARRAY_SIZE(m_expectedRegisters.R) ; i++)
        {
            CHECK_EQUAL(m_expectedRegisters.R[i], m_commandLine.context.R[i]);
        }
    }

    void rehookFgets()
    {
        hook_fgets = m_fgets;
    }

    void addArg(const char* pArg)
    {
        CHECK(m_argc < (int)ARRAY_SIZE(m_argv));
        m_argv[m_argc++] = pArg;
    }

    void validateExceptionThrownAndUsageStringDisplayed(int expectedException, const char* pExpectedExceptionMessage)
    {
        CHECK_EQUAL(expectedException, getExceptionCode());
        clearExceptionCode();
        STRCMP_EQUAL(pExpectedExceptionMessage, getExceptionMessage());
        STRCMP_EQUAL("ERROR:", printfSpy_GetNthErrorOutput(3));
        STRCMP_EQUAL("\nCrash", printfSpy_GetNthErrorOutput(2));
        STRCMP_EQUAL(g_usageString, printfSpy_GetNthErrorOutput(1));
    }

    void createTestFiles()
    {
        FILE* pFile = fopen(g_imageFilename, "w");
        fwrite(g_imageData, 1, sizeof(g_imageData), pFile);
        fclose(pFile);

        pFile = fopen(g_elfFilename, "w");
        fwrite(&m_elfFile, 1, sizeof(m_elfFile), pFile);
        fclose(pFile);

        pFile = fopen(g_dumpFilenameV2, "w");
        fwrite(g_dumpDataV2, 1, sizeof(g_dumpDataV2), pFile);
        fclose(pFile);

        pFile = fopen(g_hexDumpFilenameV2, "w");
        fwrite(g_hexCrashDumpV2, 1, sizeof(g_hexCrashDumpV2) - 1, pFile);
        fclose(pFile);

        pFile = fopen(g_binDumpFilenameV2, "w");
        fwrite(g_binCrashDumpV2, 1, sizeof(g_binCrashDumpV2) - 1, pFile);
        fclose(pFile);

        pFile = fopen(g_dumpFilenameV3, "w");
        fwrite(g_dumpDataV3, 1, sizeof(g_dumpDataV3), pFile);
        fclose(pFile);

        pFile = fopen(g_hexDumpFilenameV3, "w");
        fwrite(g_hexCrashDumpV3, 1, sizeof(g_hexCrashDumpV3) - 1, pFile);
        fclose(pFile);

        pFile = fopen(g_binDumpFilenameV3, "w");
        fwrite(g_binCrashDumpV3, 1, sizeof(g_binCrashDumpV3) - 1, pFile);
        fclose(pFile);
    }

    void initElfFile()
    {
        ElfFile* pElfFile = &m_elfFile;
        initElfHeader(&pElfFile->elfHeader);
        initPgmHeader(&pElfFile->pgmHeader);
        pElfFile->data[0] = g_imageData[0];
        pElfFile->data[1] = g_imageData[1];
    }

    void initElfHeader(Elf32_Ehdr* pHeader)
    {
        memset(pHeader, 0x00, sizeof(*pHeader));
        pHeader->e_ident[EI_MAG0] = ELFMAG0;
        pHeader->e_ident[EI_MAG1] = ELFMAG1;
        pHeader->e_ident[EI_MAG2] = ELFMAG2;
        pHeader->e_ident[EI_MAG3] = ELFMAG3;
        pHeader->e_ident[EI_CLASS] = ELFCLASS32;
        pHeader->e_ident[EI_DATA] = ELFDATA2LSB;
        pHeader->e_type = ET_EXEC;
        pHeader->e_phoff = sizeof(Elf32_Ehdr);
        pHeader->e_phnum = 1;
        pHeader->e_phentsize = sizeof(Elf32_Phdr);
    }

    void initPgmHeader(Elf32_Phdr* pHeader)
    {
        memset(pHeader, 0x00, sizeof(*pHeader));
        pHeader->p_type = PT_LOAD;
        pHeader->p_flags = PF_R | PF_X;
        pHeader->p_offset = sizeof(Elf32_Ehdr) + sizeof(Elf32_Phdr);
        pHeader->p_filesz = 2 * sizeof(uint32_t);
        pHeader->p_memsz = 2 * sizeof(uint32_t);
    }
};


TEST(CrashDebugCommandLine, NoParameters_ShouldThrow)
{
    __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "Must provide --bin or --elf command line option.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, NotDoubleSlashArgument_ShouldThrowAsNotAllowedForThisApp)
{
    addArg(g_imageFilename);
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "Command line options must start with \"--\".");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, InvalidCommandLineFlag_ShouldThrow)
{
    addArg("--foo");
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "\"--foo\" isn't a valid command line option.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, JustImageFilename_ShouldThrowAsDumpFilenameIsRequiredToo)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "Must provide --dump command line option.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, LeaveOffImageFilenameAndBaseAddress_ShouldThrow)
{
    addArg("--dump");
    addArg(g_dumpFilenameV2);
    addArg("--bin");
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "The --bin command line option requires filename and base address.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, LeaveOffBaseAddress_ShouldThrow)
{
    addArg("--dump");
    addArg(g_dumpFilenameV2);
    addArg("--bin");
    addArg(g_imageFilename);
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "The --bin command line option requires filename and base address.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, LeaveOffDumpFilename_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "The --dump command line option requires filename.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, ValidImageAndVersion2DumpFilenames_ValidateMemoryAndRegisters)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV2);
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00000004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = DEFAULT_SP_VALUE;
    m_expectedRegisters.R[PSP] = DEFAULT_SP_VALUE;
}

TEST(CrashDebugCommandLine, ValidImageAndVersion3DumpFilenames_ValidateMemoryAndRegisters)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00000004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = 0xa5a5a5a5;
    m_expectedRegisters.R[PSP] = 0xbcbcbcbc;
}

TEST(CrashDebugCommandLine, ValidImageAndVersion2DumpFilenames_DifferentBaseAddress_ValidateMemoryAndRegisters)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x4000");
    addArg("--dump");
    addArg(g_dumpFilenameV2);
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00004000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00004004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = DEFAULT_SP_VALUE;
    m_expectedRegisters.R[PSP] = DEFAULT_SP_VALUE;
}

TEST(CrashDebugCommandLine, ValidImageAndVersion3DumpFilenames_DifferentBaseAddress_ValidateMemoryAndRegisters)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x4000");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00004000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00004004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = 0xa5a5a5a5;
    m_expectedRegisters.R[PSP] = 0xbcbcbcbc;
}

TEST(CrashDebugCommandLine, ValidImageAndCrashCatcherVersion2HexDumpFilenames_ValidateMemoryAndRegisters)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_hexDumpFilenameV2);
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00000004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = DEFAULT_SP_VALUE;
    m_expectedRegisters.R[PSP] = DEFAULT_SP_VALUE;
}

TEST(CrashDebugCommandLine, ValidImageAndCrashCatcherVersion3HexDumpFilenames_ValidateMemoryAndRegisters)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_hexDumpFilenameV3);
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00000004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = 0xa5a5a5a5;
    m_expectedRegisters.R[PSP] = 0xbcbcbcbc;
}

TEST(CrashDebugCommandLine, ValidImageAndVersion2CrashCatcherBinaryDumpFilenames_ValidateMemoryAndRegisters)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_binDumpFilenameV2);
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00000004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = DEFAULT_SP_VALUE;
    m_expectedRegisters.R[PSP] = DEFAULT_SP_VALUE;
}

TEST(CrashDebugCommandLine, ValidImageAndVersion3CrashCatcherBinaryDumpFilenames_ValidateMemoryAndRegisters)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_binDumpFilenameV3);
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00000004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = 0xa5a5a5a5;
    m_expectedRegisters.R[PSP] = 0xbcbcbcbc;
}

TEST(CrashDebugCommandLine, FailImageFileOpen_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    createTestFiles();
    fopenSetReturn(NULL);
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(fileException, "Failed to open \"image.bin\".");
}

TEST(CrashDebugCommandLine, FailImageFileSeekInGetFileSize_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    createTestFiles();
    fseekSetReturn(-1);
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(fileException, "Failed to get file size of \"image.bin\".");
}

TEST(CrashDebugCommandLine, FailImageFileBufferAllocation_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    createTestFiles();
    MallocFailureInject_FailAllocation(1);
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(outOfMemoryException, "Failed to allocate 8 bytes for reading \"image.bin\".");
}

TEST(CrashDebugCommandLine, FailImageFileRead_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    createTestFiles();
    freadFail(-1);
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(fileException, "Failed to read \"image.bin\".");
}

TEST(CrashDebugCommandLine, FailMemoryRegionAllocation_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    createTestFiles();
    MallocFailureInject_FailAllocation(2);
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(outOfMemoryException, "Failed to load read-only code into memory region at address 0x00000000.");
}

TEST(CrashDebugCommandLine, InvalidDumpFilename_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg("invalidFilename.dmp");
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(fileException, "Failed to open \"invalidFilename.dmp\".");
}

TEST(CrashDebugCommandLine, ValidElfAndVersion2DumpFilenames_ValidateMemoryAndRegisters)
{
    addArg("--elf");
    addArg(g_elfFilename);
    addArg("--dump");
    addArg(g_dumpFilenameV2);
    initElfFile();
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00000004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = DEFAULT_SP_VALUE;
    m_expectedRegisters.R[PSP] = DEFAULT_SP_VALUE;
}

TEST(CrashDebugCommandLine, ValidElfAndVersion3DumpFilenames_ValidateMemoryAndRegisters)
{
    addArg("--elf");
    addArg(g_elfFilename);
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    initElfFile();
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00000004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = 0xa5a5a5a5;
    m_expectedRegisters.R[PSP] = 0xbcbcbcbc;
}

TEST(CrashDebugCommandLine, LeaveOffElfFilename_ShouldThrow)
{
    addArg("--dump");
    addArg(g_dumpFilenameV2);
    addArg("--elf");
    initElfFile();
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "The --elf command line option requires filename.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, InvalidElfSignature_ShouldThrow)
{
    addArg("--elf");
    addArg(g_elfFilename);
    addArg("--dump");
    addArg(g_dumpFilenameV2);
    initElfFile();
    m_elfFile.elfHeader.e_ident[EI_MAG0] += 1;
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(elfFormatException, "ELF header doesn't start with expected magic ELF identifier.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, ValidElfAndVersion2DumpFilenames_DifferentBaseAddress_ValidateMemoryAndRegisters)
{
    addArg("--elf");
    addArg(g_elfFilename);
    addArg("--dump");
    addArg(g_dumpFilenameV2);
    initElfFile();
    m_elfFile.pgmHeader.p_paddr = 0x4000;
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00004000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00004004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = DEFAULT_SP_VALUE;
    m_expectedRegisters.R[PSP] = DEFAULT_SP_VALUE;
}

TEST(CrashDebugCommandLine, ValidElfAndVersion3DumpFilenames_DifferentBaseAddress_ValidateMemoryAndRegisters)
{
    addArg("--elf");
    addArg(g_elfFilename);
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    initElfFile();
    m_elfFile.pgmHeader.p_paddr = 0x4000;
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00004000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00004004));
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = 0xA5A5A5A5;
    m_expectedRegisters.R[PSP] = 0xBCBCBCBC;
}

TEST(CrashDebugCommandLine, LeaveOffBaseAddressSizeRedirectAddress_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    addArg("--alias");
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "The --alias command line option requires baseAddress, size, and redirectAddress.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, LeaveOffSizeRedirectAddress_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    addArg("--alias");
    addArg("0xA0000000");
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "The --alias command line option requires baseAddress, size, and redirectAddress.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, LeaveOffRedirectAddress_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    addArg("--alias");
    addArg("0xA0000000");
    addArg("8");
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(invalidArgumentException, "The --alias command line option requires baseAddress, size, and redirectAddress.");
    CHECK(m_commandLine.pMemory == NULL);
}

TEST(CrashDebugCommandLine, AttemptToAliasToUnknownRegion_ShouldThrow)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    addArg("--alias");
    addArg("0xA0000000");
    addArg("128");
    addArg("0xBAADBEEF");
    createTestFiles();
        __try_and_catch( CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv) );
    validateExceptionThrownAndUsageStringDisplayed(busErrorException, "Failed to create alias of 0xA0000000 to 0xBAADBEEF of size 128.");
    CHECK(m_commandLine.pMemory == NULL);
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = 0xa5a5a5a5;
    m_expectedRegisters.R[PSP] = 0xbcbcbcbc;
}

TEST(CrashDebugCommandLine, UsingAliasesWithLargeSizesToBeTruncated_ValidateMemoryAndRegisters)
{
    addArg("--bin");
    addArg(g_imageFilename);
    addArg("0x0");
    addArg("--dump");
    addArg(g_dumpFilenameV3);
    addArg("--alias");
    addArg("0xA0000000");
    addArg("128");
    addArg("0x0");
    addArg("--alias");
    addArg("0xB0000000");
    addArg("128");
    addArg("0x10000000");
    createTestFiles();
        CrashDebugCommandLine_Init(&m_commandLine, m_argc, m_argv);
    // Use aliases for FLASH.
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0xA0000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0xA0000004));
    // Use original addresses for FLASH.
    CHECK_EQUAL(g_imageData[0], IMemory_Read32(m_commandLine.pMemory, 0x00000000));
    CHECK_EQUAL(g_imageData[1], IMemory_Read32(m_commandLine.pMemory, 0x00000004));
    // Use alias addresses for RAM.
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0xB0000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0xB0000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0xB0000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0xB000000c));
    // Use original addresses for RAM.
    CHECK_EQUAL(0x11111111, IMemory_Read32(m_commandLine.pMemory, 0x10000000));
    CHECK_EQUAL(0x22222222, IMemory_Read32(m_commandLine.pMemory, 0x10000004));
    CHECK_EQUAL(0x33333333, IMemory_Read32(m_commandLine.pMemory, 0x10000008));
    CHECK_EQUAL(0x44444444, IMemory_Read32(m_commandLine.pMemory, 0x1000000c));
    m_expectedRegisters.R[R0]  = 0x5a5a5a5a;
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
    m_expectedRegisters.R[MSP] = 0xa5a5a5a5;
    m_expectedRegisters.R[PSP] = 0xbcbcbcbc;
}
