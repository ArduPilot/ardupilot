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
    #include <ElfLoad.h>
    #include <ElfPriv.h>
    #include <MemorySim.h>
}

#include <string.h>

// Include C++ headers for test harness.
#include <CppUTest/TestHarness.h>

struct ElfFile1
{
    Elf32_Ehdr elfHeader;
    Elf32_Phdr pgmHeader;
    uint32_t   data[2];
};

TEST_GROUP(ElfLoad)
{
    IMemory* m_pMemory;

    void setup()
    {
        m_pMemory = MemorySim_Init();
    }

    void teardown()
    {
        CHECK_EQUAL(noException, getExceptionCode());
        clearExceptionCode();
        MemorySim_Uninit(m_pMemory);
    }

    void initElfFile(ElfFile1* pElfFile)
    {
        initElfHeader(&pElfFile->elfHeader);
        initPgmHeader(&pElfFile->pgmHeader);
        pElfFile->data[0] = 0x10008000;
        pElfFile->data[1] = 0x00000100;

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


TEST(ElfLoad, ElfSmallerThanHeaderByOneByte_ShouldThrow)
{
    Elf32_Ehdr testHeader;

    memset(&testHeader, 0xff, sizeof(testHeader));
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testHeader, sizeof(testHeader)-1) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF was too short to contain valid header.", getExceptionMessage());
}

TEST(ElfLoad, FirstByteOfElfIdentWrong_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.elfHeader.e_ident[EI_MAG0] += 1;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF header doesn't start with expected magic ELF identifier.", getExceptionMessage());
}

TEST(ElfLoad, LastByteOfElfIdentWrong_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.elfHeader.e_ident[EI_MAG3] -= 1;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF header doesn't start with expected magic ELF identifier.", getExceptionMessage());
}

TEST(ElfLoad, ElfHeaderClassTypeNot32Bit_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.elfHeader.e_ident[EI_CLASS] = ELFCLASS64;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF header doesn't contain 32-bit flag.", getExceptionMessage());
}

TEST(ElfLoad, ElfHeaderDataTypeNotLSB_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.elfHeader.e_ident[EI_DATA] = ELFDATA2MSB;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF header doesn't contain little endian flag.", getExceptionMessage());
}

TEST(ElfLoad, ElfHeaderNotExecutableType_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.elfHeader.e_type = ET_DYN;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF header doesn't contain executable flag.", getExceptionMessage());
}

TEST(ElfLoad, ElfHeaderWithPageHeaderOffsetOfZero_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.elfHeader.e_phoff = 0;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF header contains an invalid offset of 0 for the page headers.", getExceptionMessage());
}

TEST(ElfLoad, ElfHeaderWithPageHeaderEntryCountOfZero_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.elfHeader.e_phnum = 0;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF header contains an invalid page header entry count of 0.", getExceptionMessage());
}

TEST(ElfLoad, ElfHeaderWithPageHeaderSizeSmallerThanExpected_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.elfHeader.e_phentsize = sizeof(Elf32_Phdr) - 1;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF header contains a page header entry size of 31, which is smaller than the expected size of 32.",
                 getExceptionMessage());
}

TEST(ElfLoad, FirstPageHeaderEntryStartsOutOfBounds_ShouldThrow)
{
    Elf32_Ehdr testHeader;
    initElfHeader(&testHeader);
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testHeader, sizeof(testHeader)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF page header entry 0 is at an invalid file offset of 52.", getExceptionMessage());
}

TEST(ElfLoad, FirstPageHeaderEntryEndsOutOfBounds_ShouldThrow)
{
    ElfFile1 testElf;
    initElfHeader(&testElf.elfHeader);
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(Elf32_Ehdr) + sizeof(Elf32_Phdr) - 1) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF page header entry 0 is at an invalid file offset of 52.", getExceptionMessage());
}

TEST(ElfLoad, NoPageEntryIsLoadable_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.pgmHeader.p_type = PT_DYNAMIC;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF contained no entries which were loadable and had a valid non-zero filesz <= to memsz.",
                 getExceptionMessage());
}

TEST(ElfLoad, NoPageEntryHasNonZeroFileSize_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.pgmHeader.p_filesz = 0;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF contained no entries which were loadable and had a valid non-zero filesz <= to memsz.",
                 getExceptionMessage());
}

TEST(ElfLoad, NoPageEntryHasMemSizeGreaterThanOrEqualToFileSize_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.pgmHeader.p_memsz = testElf.pgmHeader.p_filesz - 1;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF contained no entries which were loadable and had a valid non-zero filesz <= to memsz.",
                 getExceptionMessage());
}

TEST(ElfLoad, PageEntryOffsetIsOverflows_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.pgmHeader.p_offset = sizeof(testElf);
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF failed to load entry from file at offsets 92 to 99.", getExceptionMessage());
}

TEST(ElfLoad, PageEntryFileSizeOverflows_ShouldThrow)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.pgmHeader.p_filesz += 1;
    testElf.pgmHeader.p_memsz += 1;
        __try_and_catch( ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf)) );
    CHECK_EQUAL(elfFormatException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("ELF failed to load entry from file at offsets 84 to 92.", getExceptionMessage());
}

TEST(ElfLoad, LoadFromFirstAndOnlyProgramHeaderEntry_ValidateMemoryContents)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
        ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf));
    CHECK_EQUAL(0x10008000, IMemory_Read32(m_pMemory, 0));
    CHECK_EQUAL(0x00000100, IMemory_Read32(m_pMemory, 4));
}

TEST(ElfLoad, NonZeroPhysicalAddressDifferentThanZeroVirtualAddress_ValidateMemoryContents)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.pgmHeader.p_paddr = 0x10000000;
        ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf));
    CHECK_EQUAL(0x10008000, IMemory_Read32(m_pMemory, 0x10000000));
    CHECK_EQUAL(0x00000100, IMemory_Read32(m_pMemory, 0x10000004));
}

TEST(ElfLoad, WriteablePageEntry_ValidateMemoryContents)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.pgmHeader.p_paddr = 0x10000000;
    testElf.pgmHeader.p_flags |= PF_W;
        ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf));
    CHECK_EQUAL(0x10008000, IMemory_Read32(m_pMemory, 0x10000000));
    CHECK_EQUAL(0x00000100, IMemory_Read32(m_pMemory, 0x10000004));
}

TEST(ElfLoad, ReadOnlyNonExecutablePageEntry_ValidateMemoryContents)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.pgmHeader.p_flags &= ~PF_X;
        ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf));
    CHECK_EQUAL(0x10008000, IMemory_Read32(m_pMemory, 0));
    CHECK_EQUAL(0x00000100, IMemory_Read32(m_pMemory, 4));
}

TEST(ElfLoad, FileSizeSmallerThanMemSize_ValidateMemoryContents_ShouldTruncateToFileSize)
{
    ElfFile1 testElf;
    initElfFile(&testElf);
    testElf.pgmHeader.p_filesz = sizeof(uint32_t);
        ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf));
    CHECK_EQUAL(0x10008000, IMemory_Read32(m_pMemory, 0));

    // Verify that no zero extending has occurred and that reading second word will fail.
    __try_and_catch( IMemory_Read32(m_pMemory, 4) );
    CHECK_EQUAL(busErrorException, getExceptionCode());
    clearExceptionCode();
    STRCMP_EQUAL("", getExceptionMessage());
}

TEST(ElfLoad, LoadFromSecondProgramHeaderEntry_ValidateMemoryContents)
{
    struct ElfFile2
    {
        Elf32_Ehdr elfHeader;
        Elf32_Phdr pgmHeader[2];
        uint32_t   data[2];
    } testElf;
    memset(&testElf, 0, sizeof(testElf));
    initElfHeader(&testElf.elfHeader);
    initPgmHeader(&testElf.pgmHeader[1]);
    testElf.elfHeader.e_phnum = 2;
    testElf.pgmHeader[1].p_offset = offsetof(ElfFile2, data);
    testElf.data[0] = 0x10008000;
    testElf.data[1] = 0x00000100;
        ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf));
    CHECK_EQUAL(0x10008000, IMemory_Read32(m_pMemory, 0));
    CHECK_EQUAL(0x00000100, IMemory_Read32(m_pMemory, 4));
}

TEST(ElfLoad, LoadFromSecondProgramHeaderEntryWithNonStandardEntrySize_ValidateMemoryContents)
{
    struct ElfFile2
    {
        Elf32_Ehdr elfHeader;
        Elf32_Phdr pgmHeader0;
        uint32_t   pad0;
        Elf32_Phdr pgmHeader1;
        uint32_t   pad1;
        uint32_t   data[2];
    } testElf;
    memset(&testElf, 0, sizeof(testElf));
    initElfHeader(&testElf.elfHeader);
    initPgmHeader(&testElf.pgmHeader1);
    testElf.elfHeader.e_phnum = 2;
    testElf.elfHeader.e_phentsize += sizeof(uint32_t);
    testElf.pgmHeader1.p_offset = offsetof(ElfFile2, data);
    testElf.data[0] = 0xFFFFFFFF;
    testElf.data[1] = 0x00000000;
        ElfLoad_FromMemory(m_pMemory, &testElf, sizeof(testElf));
    CHECK_EQUAL(0xFFFFFFFF, IMemory_Read32(m_pMemory, 0));
    CHECK_EQUAL(0x00000000, IMemory_Read32(m_pMemory, 4));
}
