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

// Include headers from C modules under test.
extern "C"
{
    #include <common.h>
    #include <CrashCatcherDump.h>
    #include <CrashCatcher.h>
    #include <FileFailureInject.h>
    #include <MallocFailureInject.h>
}

#include <stddef.h>
#include <string.h>

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


struct DumpFileTop
{
    uint8_t  signature[4];
    uint32_t flags;
    uint32_t R[TOTAL_REG_COUNT];
    uint32_t exceptionPSR;
};

struct DumpFPFileTop : public DumpFileTop
{
    uint32_t     FPR[TOTAL_FPREG_COUNT];
};

struct DumpFileTopVersion2
{
    uint8_t  signature[4];
    uint32_t flags;
    uint32_t R[16+1];
    uint32_t exceptionPSR;
};

struct DumpFPFileTopVersion2 : public DumpFileTopVersion2
{
    uint32_t     FPR[TOTAL_FPREG_COUNT];
};


class DumpBaseTest : public Utest
{
protected:
    IMemory*            m_pMem;
    RegisterContext     m_actualRegisters;
    RegisterContext     m_expectedRegisters;
    DumpFileTop         m_fileTop;
    DumpFileTopVersion2 m_fileTopVersion2;
    const char*         m_pTestFilename;

    void setup()
    {
        m_pMem = MemorySim_Init();
        initRegisterContext();
        initFileTop();
        initFileTopVersion2();
    }

    void initRegisterContext()
    {
        memset(&m_actualRegisters, 0, sizeof(m_actualRegisters));
        memset(&m_expectedRegisters, 0, sizeof(m_expectedRegisters));
        for (size_t i = 0 ; i < ARRAY_SIZE(m_actualRegisters.R) ; i++)
        {
            m_actualRegisters.R[i] = 0xDEADBEEF;
            m_expectedRegisters.R[i] = 0xDEADBEEF;
        }
    }

    void initFileTop()
    {
        m_fileTop.signature[0] = CRASH_CATCHER_SIGNATURE_BYTE0;
        m_fileTop.signature[1] = CRASH_CATCHER_SIGNATURE_BYTE1;
        m_fileTop.signature[2] = CRASH_CATCHER_VERSION_MAJOR;
        m_fileTop.signature[3] = CRASH_CATCHER_VERSION_MINOR;
        m_fileTop.flags = 0;
        for (size_t i = 0 ; i < ARRAY_SIZE(m_fileTop.R) ; i++)
            m_fileTop.R[i] = 0xDEADBEEF;
    }

    void initFileTopVersion2()
    {
        m_fileTopVersion2.signature[0] = CRASH_CATCHER_SIGNATURE_BYTE0;
        m_fileTopVersion2.signature[1] = CRASH_CATCHER_SIGNATURE_BYTE1;
        m_fileTopVersion2.signature[2] = 2;
        m_fileTopVersion2.signature[3] = 0;
        m_fileTopVersion2.flags = 0;
        for (size_t i = 0 ; i < ARRAY_SIZE(m_fileTopVersion2.R) ; i++)
            m_fileTopVersion2.R[i] = 0xDEADBEEF;
    }

    void teardown()
    {
        CHECK_EQUAL(noException, getExceptionCode());
        checkRegisters();
        fopenRestore();
        freadRestore();
        remove(m_pTestFilename);
        MemorySim_Uninit(m_pMem);
        MallocFailureInject_Restore();
        clearExceptionCode();
    }

    void checkRegisters()
    {
        CHECK_EQUAL(m_expectedRegisters.flags, m_actualRegisters.flags);
        for (size_t i = 0 ; i < ARRAY_SIZE(m_actualRegisters.R) ; i++)
        {
            CHECK_EQUAL(m_expectedRegisters.R[i], m_actualRegisters.R[i]);
        }
        CHECK_EQUAL(m_expectedRegisters.exceptionPSR, m_actualRegisters.exceptionPSR);
        for (size_t i = 0 ; i < ARRAY_SIZE(m_actualRegisters.FPR) ; i++)
        {
            CHECK_EQUAL(m_expectedRegisters.FPR[i], m_actualRegisters.FPR[i]);
        }
    }

    void validateNoMemoryRegionsCreated()
    {
        static const char* xmlForEmptyRegions = "<?xml version=\"1.0\"?>"
                                                "<!DOCTYPE memory-map PUBLIC \"+//IDN gnu.org//DTD GDB Memory Map V1.0//EN\" \"http://sourceware.org/gdb/gdb-memory-map.dtd\">"
                                                "<memory-map>"
                                                "</memory-map>";
        const char* pMemoryLayout = MemorySim_GetMemoryMapXML(m_pMem);
        STRCMP_EQUAL(xmlForEmptyRegions, pMemoryLayout);
    }
};
