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
#include <string.h>

extern "C"
{
    #include <mockConsole.h>
    #include <StandardIComm.h>
}

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"

TEST_GROUP(StandardIComm)
{
    IComm* m_pComm;
    void setup()
    {
        clearExceptionCode();
        m_pComm = StandardIComm_Init();
        CHECK(m_pComm != NULL);
        ConsoleMock_WriteStdOut_SetCaptureBufferSize(5);
    }

    void teardown()
    {
        CHECK_EQUAL(noException, getExceptionCode());
        StandardIComm_Uninit(m_pComm);
        ConsoleMock_Uninit();
    }
};


TEST(StandardIComm, ShouldStopRun_AlwaysReturnFALSE)
{
    CHECK_FALSE(IComm_ShouldStopRun(m_pComm));
}


TEST(StandardIComm, HasReceiveData_ThrowException_ShouldReturnFalse)
{
    ConsoleMock_HasStdInDataToRead_SetException(fileException);
    CHECK_FALSE(IComm_HasReceiveData(m_pComm));
}

TEST(StandardIComm, HasReceiveData_Return1_ShouldReturnTrue)
{
    ConsoleMock_HasStdInDataToRead_SetReturn(1);
    CHECK_TRUE(IComm_HasReceiveData(m_pComm));
}

TEST(StandardIComm, HasReceiveData_Return0_ShouldReturnFalse)
{
    ConsoleMock_HasStdInDataToRead_SetReturn(0);
    CHECK_FALSE(IComm_HasReceiveData(m_pComm));
}


TEST(StandardIComm, ReceiveChar_ThrowException_VerifyExceptionThrown)
{
    ConsoleMock_ReadStdIn_SetException(fileException);
    ConsoleMock_ReadStdIn_SetBuffer("a", 1);
        __try_and_catch( IComm_ReceiveChar(m_pComm) );
    CHECK_EQUAL(fileException, getExceptionCode());
    clearExceptionCode();
}

TEST(StandardIComm, ReceiveChar_ReadSingleChar)
{
    ConsoleMock_ReadStdIn_SetBuffer("a", 1);
    char c = IComm_ReceiveChar(m_pComm);
    CHECK_EQUAL('a', c);
}


TEST(StandardIComm, SendChar_ThrowException_VerifyExceptionThrown)
{
    ConsoleMock_WriteStdOut_SetException(fileException);
        __try_and_catch( IComm_SendChar(m_pComm, 'z') );
    CHECK_EQUAL(fileException, getExceptionCode());
    clearExceptionCode();
}

TEST(StandardIComm, SendChar_VerifySuccessfullySentBytes)
{
    IComm_SendChar(m_pComm, 'x');
    IComm_SendChar(m_pComm, 'y');
    IComm_SendChar(m_pComm, 'z');
    STRCMP_EQUAL("xyz", ConsoleMock_WriteStdOut_GetCapturedText());
}

TEST(StandardIComm, IsGdbConnected_ShouldReturnFalse)
{
    CHECK_FALSE(IComm_IsGdbConnected(m_pComm));
}

TEST(StandardIComm, IsGdbConnected_ShouldReturnTrueOnceHasRecieveData)
{
    CHECK_FALSE(IComm_IsGdbConnected(m_pComm));
        ConsoleMock_HasStdInDataToRead_SetReturn(1);
    CHECK_TRUE(IComm_IsGdbConnected(m_pComm));
}

TEST(StandardIComm, IsGdbConnected_ShouldReturnTrueOnceOneByteHasBeenRead)
{
    CHECK_FALSE(IComm_IsGdbConnected(m_pComm));
        ConsoleMock_ReadStdIn_SetBuffer("+", 1);
    CHECK_FALSE(IComm_IsGdbConnected(m_pComm));
        char c = IComm_ReceiveChar(m_pComm);
        CHECK_EQUAL('+', c);
    CHECK_TRUE(IComm_IsGdbConnected(m_pComm));
}