/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

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

extern "C"
{
#include "try_catch.h"
}

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"

TEST_GROUP(TryCatch)
{
    int m_exceptionThrown;
    
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
    
    void throwBufferAndArgumentExceptions()
    {
        throwBufferOverrunException();
        throwInvalidArgumentException();
    }
    
    void throwArgumentAndBufferExceptions()
    {
        throwInvalidArgumentException();
        throwBufferOverrunException();
    }
    
    void throwBufferOverrunException()
    {
        __throw(bufferOverrunException);
    }
    
    void throwInvalidArgumentException()
    {
        __throw(invalidArgumentException);
    }
    
    int throwBufferOverrunExceptionAndReturnNegative1()
    {
        __throw_and_return(bufferOverrunException, -1);
    }
    
    void rethrowBufferOverrunException()
    {
        __try
            throwBufferOverrunException();
        __catch
            __rethrow;
        __throw(invalidArgumentException);
    }
    
    int rethrowBufferOverrunExceptionAndReturnNegative1()
    {
        __try
            throwBufferOverrunException();
        __catch
            __rethrow_and_return(-1);
        __throw_and_return(invalidArgumentException, 0);
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

TEST(TryCatch, bufferOverrunException)
{
    __try
        throwBufferOverrunException();
    __catch
        flagExceptionHit();
    validateException(bufferOverrunException);
}

TEST(TryCatch, EscalatingExceptions)
{
    __try
        throwBufferAndArgumentExceptions();
    __catch
        flagExceptionHit();

    validateException(invalidArgumentException);
}

TEST(TryCatch, NonEscalatingExceptions)
{
    __try
        throwArgumentAndBufferExceptions();
    __catch
        flagExceptionHit();

    validateException(invalidArgumentException);
}

TEST(TryCatch, CatchFirstThrow)
{
    __try
    {
        __throwing_func( throwBufferOverrunException() );
        __throwing_func( throwInvalidArgumentException() );
    }
    __catch
    {
        flagExceptionHit();
    }
    
    validateException(bufferOverrunException);
}

TEST(TryCatch, CatchSecondThrow)
{
    __try
    {
        __throwing_func( throwNoException() );
        __throwing_func( throwInvalidArgumentException() );
    }
    __catch
    {
        flagExceptionHit();
    }
    
    validateException(invalidArgumentException);
}

TEST(TryCatch, ThrowAndReturn)
{
    int value;
    
    __try
        value = throwBufferOverrunExceptionAndReturnNegative1();
    __catch
        flagExceptionHit();
        
    LONGS_EQUAL( -1, value );
    validateException(bufferOverrunException);
}

TEST(TryCatch, RethrowBufferOverrunException)
{
    __try
        rethrowBufferOverrunException();
    __catch
        flagExceptionHit();
    validateException(bufferOverrunException);
}

TEST(TryCatch, RethrowBufferOverrunExceptionAndReturnNegative1)
{
    int value;
    
    __try
        value = rethrowBufferOverrunExceptionAndReturnNegative1();
    __catch
        flagExceptionHit();

    LONGS_EQUAL( -1, value );
    validateException(bufferOverrunException);
}
