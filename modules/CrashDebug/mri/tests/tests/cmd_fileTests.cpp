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
#include <cmd_file.h>
#include <core.h>
}
#include <platformMock.h>

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


TEST_GROUP(cmdFile)
{
    int  m_expectedException;            
    
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

TEST(cmdFile, IssueGdbFileOpenRequest_ReturnSuccess)
{
    OpenParameters params = { 0x11111111, 0x22222222, 0x33333333, 0x44444444 };
    platformMock_CommInitReceiveChecksummedData("+$F0#");
        IssueGdbFileOpenRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Fopen,11111111/22222222,33333333,44444444#fb+") );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( 0, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileOpenRequest_ReturnError)
{
    OpenParameters params = { 0x11111111, 0x22222222, 0x33333333, 0x44444444 };
    platformMock_CommInitReceiveChecksummedData("+$F-1,12345678#");
        IssueGdbFileOpenRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Fopen,11111111/22222222,33333333,44444444#fb+") );
    CHECK_EQUAL ( -1, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( -1, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0x12345678, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileOpenRequest_ReturnErrorAndControlC)
{
    OpenParameters params = { 0x11111111, 0x22222222, 0x33333333, 0x44444444 };
    platformMock_CommInitReceiveChecksummedData("+$F-1,12345678,C#");
        IssueGdbFileOpenRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Fopen,11111111/22222222,33333333,44444444#fb+") );
    CHECK_EQUAL ( -1, platformMock_GetSemihostCallReturnValue() );
    CHECK_TRUE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( -1, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0x12345678, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileOpenRequest_ReturnInterruptErrorAndControlC)
{
    OpenParameters params = { 0x11111111, 0x22222222, 0x33333333, 0x44444444 };
    platformMock_CommInitReceiveChecksummedData("+$F-1,4,C#"); // 4 is EINTR
        IssueGdbFileOpenRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Fopen,11111111/22222222,33333333,44444444#fb+") );
    CHECK_TRUE ( WasControlCFlagSentFromGdb() );
    CHECK_TRUE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( -1, GetSemihostReturnCode() );
    CHECK_EQUAL ( 4, GetSemihostErrno() );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() ); // Doesn't call in this scenario.
    CHECK_EQUAL ( 0, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileWriteRequest_ReturnSuccess)
{
    TransferParameters params = { 0x11111111, 0x22222222, 0x33333333 };
    platformMock_CommInitReceiveChecksummedData("+$F0#");
        IssueGdbFileWriteRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Fwrite,11111111,22222222,33333333#a5+") );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( 0, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileReadRequest_ReturnSuccess)
{
    TransferParameters params = { 0x11111111, 0x22222222, 0x33333333 };
    platformMock_CommInitReceiveChecksummedData("+$F0#");
        IssueGdbFileReadRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Fread,11111111,22222222,33333333#16+") );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( 0, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileCloseRequest_ReturnSuccess)
{
    platformMock_CommInitReceiveChecksummedData("+$F0#");
        IssueGdbFileCloseRequest(0x12345678);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Fclose,12345678#2c+") );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( 0, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileSeekRequest_ReturnSuccess)
{
    SeekParameters params = { 0x11111111, 0x22222222, 0x33333333 };
    platformMock_CommInitReceiveChecksummedData("+$F0#");
        IssueGdbFileSeekRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Flseek,11111111,22222222,33333333#8e+") );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( 0, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileFStatRequest_ReturnSuccess)
{
    platformMock_CommInitReceiveChecksummedData("+$F0#");
        IssueGdbFileFStatRequest(0x11111111, 0x22222222);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Ffstat,11111111,22222222#d8+") );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( 0, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileUnlinkRequest_ReturnSuccess)
{
    RemoveParameters params = { 0x11111111, 0x22222222 };
    platformMock_CommInitReceiveChecksummedData("+$F0#");
        IssueGdbFileUnlinkRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Funlink,11111111/22222222#4a+") );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( 0, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileStatRequest_ReturnSuccess)
{
    StatParameters params = { 0x11111111, 0x22222222, 0x12345678 };
    platformMock_CommInitReceiveChecksummedData("+$F0#");
        IssueGdbFileStatRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Fstat,11111111/22222222,12345678#45+") );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( 0, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}

TEST(cmdFile, IssueGdbFileRenameRequest_ReturnSuccess)
{
    RenameParameters params = { 0x11111111, 0x22222222, 0x33333333, 0x44444444 };
    platformMock_CommInitReceiveChecksummedData("+$F0#");
        IssueGdbFileRenameRequest(&params);
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$Frename,11111111/22222222,33333333/44444444#c4+") );
    CHECK_EQUAL ( 0, platformMock_GetSemihostCallReturnValue() );
    CHECK_FALSE ( WasControlCFlagSentFromGdb() );
    CHECK_FALSE ( WasSemihostCallCancelledByGdb() );
    CHECK_EQUAL ( 0, GetSemihostReturnCode() );
    CHECK_EQUAL ( 0, GetSemihostErrno() );
    CHECK_EQUAL ( 1, platformMock_AdvanceProgramCounterToNextInstructionCalls() );
}
