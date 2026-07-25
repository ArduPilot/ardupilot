/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
extern "C"
{
#include "common.h"
#include "FileFailureInject.h"
}

const char* testFilename = "GetFileSizeTest.tst";

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"

TEST_GROUP(GetFileSize)
{
    FILE* m_testFile;

    void setup()
    {
        m_testFile = fopen(testFilename, "wb");
    }

    void teardown()
    {
        fclose(m_testFile);
        remove(testFilename);
        ftellRestore();
        fseekRestore();
        CHECK_EQUAL(0, getExceptionCode());
    }

    void validateExceptionThrown(int expectedException)
    {
        CHECK_EQUAL(expectedException, getExceptionCode());
        clearExceptionCode();
    }
};

TEST(GetFileSize, FailFSeek_ShouldThrow)
{
    fseekSetReturn(-1);
    __try_and_catch( GetFileSize(m_testFile) );
    validateExceptionThrown(fileException);
}

TEST(GetFileSize, FailFTell_ShouldThrow)
{
    ftellFail(-1);
    __try_and_catch( GetFileSize(m_testFile) );
    validateExceptionThrown(fileException);
}

TEST(GetFileSize, FailSecondFSeek_ShouldThrow)
{
    fseekSetReturn(-1);
    fseekSetCallsBeforeFailure(1);
    __try_and_catch( GetFileSize(m_testFile) );
    validateExceptionThrown(fileException);
}

TEST(GetFileSize, RunSuccessfullyOnZeroLengthFile)
{
    long fileSize = GetFileSize(m_testFile);
    CHECK_EQUAL(0, fileSize);
}

TEST(GetFileSize, RunSuccessfullyOn5ByteFile)
{
    fprintf(m_testFile, "12345");
    long fileSize = GetFileSize(m_testFile);
    CHECK_EQUAL(5, fileSize);
}
