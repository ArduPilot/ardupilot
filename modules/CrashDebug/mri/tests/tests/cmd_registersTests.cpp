/* Copyright 2014 Adam Green (http://mbed.org/users/AdamGreen/)

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
#include <try_catch.h>
#include <mri.h>

void __mriDebugException(void);
}
#include <platformMock.h>

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


TEST_GROUP(cmdRegisters)
{
    int     m_expectedException;            
    
    void setup()
    {
        m_expectedException = noException;
        platformMock_Init();
        __mriInit("MRI_UART_MBED_USB");
    }

    void teardown()
    {
        LONGS_EQUAL ( m_expectedException, getExceptionCode() );
        clearExceptionCode();
        platformMock_Uninit();
    }
    
    void validateExceptionCode(int expectedExceptionCode)
    {
        m_expectedException = expectedExceptionCode;
        LONGS_EQUAL ( expectedExceptionCode, getExceptionCode() );
    }
};

TEST(cmdRegisters, GetRegisters)
{
    uint32_t* pContext = platformMock_GetContext();
    pContext[0] = 0x11111111;
    pContext[1] = 0x22222222;
    pContext[2] = 0x33333333;
    pContext[3] = 0x44444444;
    
    platformMock_CommInitReceiveChecksummedData("+$g#", "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$11111111222222223333333344444444#50+") );
}

TEST(cmdRegisters, SetRegisters)
{
    platformMock_CommInitReceiveChecksummedData("+$G1234567822222222333333339abcdef0#", "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    uint32_t* pContext = platformMock_GetContext();
    CHECK_EQUAL ( 0x78563412, pContext[0] );    
    CHECK_EQUAL ( 0x22222222, pContext[1] );    
    CHECK_EQUAL ( 0x33333333, pContext[2] );    
    CHECK_EQUAL ( 0xf0debc9a, pContext[3] );    
}

TEST(cmdRegisters, SetRegisters_BufferTooShort)
{
    platformMock_CommInitReceiveChecksummedData("+$G1234567822222222333333339abcdef#", "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_BUFFER_OVERRUN "#a9+") );
    uint32_t* pContext = platformMock_GetContext();
    CHECK_EQUAL ( 0x78563412, pContext[0] );    
    CHECK_EQUAL ( 0x22222222, pContext[1] );    
    CHECK_EQUAL ( 0x33333333, pContext[2] );    
    CHECK_EQUAL ( 0xffdebc9a, pContext[3] );    
}
