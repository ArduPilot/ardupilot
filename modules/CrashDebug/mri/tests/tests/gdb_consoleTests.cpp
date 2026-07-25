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
#include <gdb_console.h>
}
#include <platformMock.h>

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


TEST_GROUP(gdbConsole)
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

TEST(gdbConsole, WriteStringToGdbConsole_SendAsGdbPacketWhenNotShared)
{
    platformMock_SetCommSharingWithApplication(0);
    WriteStringToGdbConsole("Test\n");
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$O546573740a#89") );
}

TEST(gdbConsole, WriteStringToGdbConsole_SendRawStringWhenShared)
{
    platformMock_SetCommSharingWithApplication(1);
    WriteStringToGdbConsole("Test string\n");
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("Test string\n") );
}

TEST(gdbConsole, WriteStringToGdbConsole_FailToSendAsGdbPacketBecauseOfSmallBuffer)
{
    platformMock_SetPacketBufferSize(10);
    WriteStringToGdbConsole("Test\n");
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("") );
    validateExceptionCode(bufferOverrunException);
}

TEST(gdbConsole, WriteHexValueToGdbConsole_SendMinimumValue)
{
    platformMock_SetCommSharingWithApplication(1);
    WriteHexValueToGdbConsole(0);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("0x00") );
}

TEST(gdbConsole, WriteHexValueToGdbConsole_SendMaximumValue)
{
    platformMock_SetCommSharingWithApplication(1);
    WriteHexValueToGdbConsole(~0U);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("0xffffffff") );
}