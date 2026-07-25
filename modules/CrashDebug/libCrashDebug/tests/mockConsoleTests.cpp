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
#include <errno.h>
// Include headers from C modules under test.
extern "C"
{
    #include <Console.h>
    #include <mockConsole.h>
}


// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


TEST_GROUP(Console)
{
    void setup()
    {
    }

    void teardown()
    {
        CHECK_EQUAL(noException, getExceptionCode());
        ConsoleMock_Uninit();
    }
};


TEST(Console, HasStdinDataToRead_SetToReturn0_Verify)
{
    ConsoleMock_HasStdInDataToRead_SetReturn(0);
        int result = Console_HasStdInDataToRead();
    CHECK_EQUAL(0, result);
}

TEST(Console, HasStdinDataToRead_SetToReturn1_Verify)
{
    ConsoleMock_HasStdInDataToRead_SetReturn(1);
        int result = Console_HasStdInDataToRead();
    CHECK_EQUAL(1, result);
}

TEST(Console, HasStdinDataToRead_SetToThrow_VerifyException)
{
    ConsoleMock_HasStdInDataToRead_SetException(fileException);
        __try_and_catch( Console_HasStdInDataToRead() );
    CHECK_EQUAL(fileException, getExceptionCode());
    clearExceptionCode();
}



TEST(Console, ReadStdIn_ShouldDefaultToEmptyBufferAndReturnZero)
{
    int result = Console_ReadStdIn();
    CHECK_EQUAL(0, result);
}

TEST(Console, ReadStdIn_SetToReturnOneChar_VerifyOneChar)
{
    const char testBuffer[] = { 'a' };
    ConsoleMock_ReadStdIn_SetBuffer(testBuffer, sizeof(testBuffer));
        int result = Console_ReadStdIn();
    CHECK_EQUAL('a', result);
}

TEST(Console, ReadStdIn_SetToReturnTwoChars_VerifyTwoChars)
{
    const char testBuffer[] = { 'a', 'z' };
    ConsoleMock_ReadStdIn_SetBuffer(testBuffer, sizeof(testBuffer));
        int result1 = Console_ReadStdIn();
        int result2 = Console_ReadStdIn();
    CHECK_EQUAL('a', result1);
    CHECK_EQUAL('z', result2);
}

TEST(Console, ReadStdIn_SetToReturnOneChar_VerifyBufferOverrunReturnZero)
{
    const char testBuffer[] = { 'a', 'z' };
    ConsoleMock_ReadStdIn_SetBuffer(testBuffer, 1);
        int result1 = Console_ReadStdIn();
        int result2 = Console_ReadStdIn();
    CHECK_EQUAL('a', result1);
    CHECK_EQUAL(0, result2);
}

TEST(Console, ReadStdIn_SetToThrow_VerifyException)
{
    ConsoleMock_ReadStdIn_SetException(fileException);
        __try_and_catch( Console_ReadStdIn() );
    CHECK_EQUAL(fileException, getExceptionCode());
    clearExceptionCode();
}



TEST(Console, WriteStdOut_SetToThrow_VerifyException)
{
    ConsoleMock_WriteStdOut_SetException(fileException);
        __try_and_catch( Console_WriteStdOut('a') );
    CHECK_EQUAL(fileException, getExceptionCode());
    clearExceptionCode();
}

TEST(Console, WriteStdOut_CaptureOneCharToOneCharBuffer)
{
    ConsoleMock_WriteStdOut_SetCaptureBufferSize(1);
        Console_WriteStdOut('a');
    STRCMP_EQUAL("a", ConsoleMock_WriteStdOut_GetCapturedText());
}

TEST(Console, WriteStdOut_CaptureTwoCharsToTwoCharBuffer)
{
    ConsoleMock_WriteStdOut_SetCaptureBufferSize(2);
        Console_WriteStdOut('a');
        Console_WriteStdOut('z');
    STRCMP_EQUAL("az", ConsoleMock_WriteStdOut_GetCapturedText());
}

TEST(Console, WriteStdOut_AttemptToCaptureTwoCharsToOneCharBuffer_ShouldJustCaptureFirstChar)
{
    ConsoleMock_WriteStdOut_SetCaptureBufferSize(1);
        Console_WriteStdOut('z');
        Console_WriteStdOut('a');
    STRCMP_EQUAL("z", ConsoleMock_WriteStdOut_GetCapturedText());
}
