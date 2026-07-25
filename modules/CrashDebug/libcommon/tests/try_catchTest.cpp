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
extern "C"
{
#include "try_catch.h"
}
#include <string.h>

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"

TEST_GROUP(TryCatch)
{
    int m_exceptionThrown;
    char m_longMessage[sizeof(g_exceptionMessage)+1];

    void setup()
    {
        m_exceptionThrown = 0;
    }

    void teardown()
    {
    }

    void flagExceptionHit()
    {
        m_exceptionThrown = 1;
    }

    void throwNoException()
    {
    }

    void throwUndefinedAndUnpredictableExceptions()
    {
        __try
            throwUndefinedException();
        __catch
            throwUnpredictableException();
    }

    void throwUnpredictableAndUndefinedExceptions()
    {
        __try
            throwUnpredictableException();
        __catch
            throwUndefinedException();
    }

    int throwUndefinedException()
    {
        __throw(undefinedException);
    }

    void throwUnpredictableException()
    {
        __throw(unpredictableException);
    }

    int rethrowUndefinedException()
    {
        __try
            throwUndefinedException();
        __catch
            __rethrow;
        __throw(unpredictableException);
    }

    void validateException(int expectedExceptionCode)
    {
        if (expectedExceptionCode == noException)
        {
            CHECK_FALSE(m_exceptionThrown);
        }
        else
        {
            CHECK_TRUE(m_exceptionThrown);
        }
        LONGS_EQUAL(expectedExceptionCode, getExceptionCode());
    }
};

TEST(TryCatch, NoException)
{
    __try
        throwNoException();
    __catch
        flagExceptionHit();
    validateException(noException);
}

TEST(TryCatch, undefinedException)
{
    __try
        throwUndefinedException();
    __catch
        flagExceptionHit();
    validateException(undefinedException);
}

TEST(TryCatch, EscalatingExceptions)
{
    __try
        throwUndefinedAndUnpredictableExceptions();
    __catch
        flagExceptionHit();

    validateException(unpredictableException);
}

TEST(TryCatch, NonEscalatingExceptions)
{
    __try
        throwUnpredictableAndUndefinedExceptions();
    __catch
        flagExceptionHit();

    validateException(unpredictableException);
}

TEST(TryCatch, CatchFirstThrow)
{
    __try
    {
        throwUndefinedException();
        throwUnpredictableException();
    }
    __catch
    {
        flagExceptionHit();
    }

    validateException(undefinedException);
}

TEST(TryCatch, CatchSecondThrow)
{
    __try
    {
        throwNoException();
        throwUnpredictableException();
    }
    __catch
    {
        flagExceptionHit();
    }

    validateException(unpredictableException);
}

TEST(TryCatch, ThrowAndSkipAssignment)
{
    int value = -2;

    __try
        value = throwUndefinedException();
    __catch
        flagExceptionHit();

    LONGS_EQUAL( -2, value );
    validateException(undefinedException);
}

TEST(TryCatch, RethrowUndefinedException)
{
    __try
        rethrowUndefinedException();
    __catch
        flagExceptionHit();
    validateException(undefinedException);
}

TEST(TryCatch, RethrowUndefinedExceptionAndSkipAssignment)
{
    int value = -2;

    __try
        value = rethrowUndefinedException();
    __catch
        flagExceptionHit();

    LONGS_EQUAL( -2, value );
    validateException(undefinedException);
}

TEST(TryCatch, throwWithSimpleMessage)
{
    __try
        __throw_msg(undefinedException, "throw message");
    __catch
        flagExceptionHit();

    validateException(undefinedException);
    STRCMP_EQUAL("throw message", getExceptionMessage());
}

TEST(TryCatch, throwWithFormattedMessage)
{
    __try
        __throw_msg(undefinedException, "throw message #%d", 1);
    __catch
        flagExceptionHit();

    validateException(undefinedException);
    STRCMP_EQUAL("throw message #1", getExceptionMessage());
}

TEST(TryCatch, throwWithLongestMessage)
{
    memset(m_longMessage, '_', sizeof(m_longMessage));
    m_longMessage[sizeof(g_exceptionMessage)-1] = '\0';
    __try
        __throw_msg(undefinedException, "%s", m_longMessage);
    __catch
        flagExceptionHit();

    validateException(undefinedException);
    STRCMP_EQUAL(m_longMessage, getExceptionMessage());
}

TEST(TryCatch, throwWithTooLongMessageShoudTruncate)
{
    memset(m_longMessage, '_', sizeof(m_longMessage));
    m_longMessage[sizeof(g_exceptionMessage)] = '\0';
    __try
        __throw_msg(undefinedException, "%s", m_longMessage);
    __catch
        flagExceptionHit();

    validateException(undefinedException);
    m_longMessage[sizeof(g_exceptionMessage)-1] = '\0';
    STRCMP_EQUAL(m_longMessage, getExceptionMessage());
}


TEST(TryCatch, subsequentTryClearsExceptionMessage)
{
    __try
        __throw_msg(undefinedException, "throw message");
    __catch
        flagExceptionHit();
    validateException(undefinedException);
    STRCMP_EQUAL("throw message", getExceptionMessage());

    m_exceptionThrown = 0;
    __try
        throwNoException();
    __catch
        flagExceptionHit();
    validateException(noException);
    STRCMP_EQUAL("", getExceptionMessage());
}
