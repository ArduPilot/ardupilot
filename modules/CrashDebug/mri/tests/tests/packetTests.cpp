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
#include <string.h>

extern "C"
{
#include "packet.h"
#include "try_catch.h"
#include "token.h"
}
#include "platformMock.h"

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"

TEST_GROUP(Packet)
{
    Packet            m_packet;
    Buffer            m_buffer;
    char*             m_pCharacterArray;
    int               m_exceptionThrown;
    static const char m_fillChar = 0xFF;
    
    void setup()
    {
        m_pCharacterArray = NULL;
        allocateBuffer(32);
        m_exceptionThrown = 0;
        platformMock_CommInitTransmitDataBuffer(16);
    }

    void teardown()
    {
        validateNoExceptionThrown();
        platformMock_Uninit();
        free(m_pCharacterArray);
    }
    
    void validateNoExceptionThrown()
    {
        CHECK_FALSE ( m_exceptionThrown );
        LONGS_EQUAL ( 0, getExceptionCode() );
    }
    
    void allocateBuffer(size_t sizeOfBuffer)
    {
        free(m_pCharacterArray);
        m_pCharacterArray = (char*)malloc(sizeOfBuffer);
        memset(m_pCharacterArray, m_fillChar, sizeOfBuffer);
        Buffer_Init(&m_buffer, m_pCharacterArray, sizeOfBuffer);
    }
    
    void allocateBuffer(const char* pBufferString)
    {
        free(m_pCharacterArray);
        m_pCharacterArray = NULL;
        Buffer_Init(&m_buffer, (char*)pBufferString, strlen(pBufferString));
    }

    void tryPacketGet()
    {
        __try
            Packet_GetFromGDB(&m_packet, &m_buffer);
        __catch
            m_exceptionThrown = 1;
    }
    
    void tryPacketSend()
    {
        __try
            Packet_SendToGDB(&m_packet, &m_buffer);
        __catch
            m_exceptionThrown = 1;
    }
    
    void validateThatEmptyGdbPacketWasRead()
    {
        LONGS_EQUAL( 0, Buffer_GetLength(&m_buffer) );
    }
    
    void validateBufferMatches(const char* pExpectedOutput)
    {
        CHECK_TRUE( Buffer_MatchesString(&m_buffer, pExpectedOutput, strlen(pExpectedOutput)) );
    }
};

TEST(Packet, PacketGetFromGDB_Empty)
{
    platformMock_CommInitReceiveData("$#00");
    tryPacketGet();
    validateThatEmptyGdbPacketWasRead();
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("+") );
}

TEST(Packet, PacketGetFromGDB_Short)
{
    platformMock_CommInitReceiveData("$?#3f");
    tryPacketGet();
    validateBufferMatches("?");
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("+") );
}

TEST(Packet, PacketGetFromGDB_ShortWithUpperCaseHexDigits)
{
    platformMock_CommInitReceiveData("$?#3F");
    tryPacketGet();
    validateBufferMatches("?");
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("+") );
}

TEST(Packet, PacketGetFromGDB_BadChecksum)
{
    platformMock_CommInitReceiveData("$?#f3");
    tryPacketGet();
    validateThatEmptyGdbPacketWasRead();
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("-+") ); 
}

TEST(Packet, PacketGetFromGDB_BadHexDigitInChecksum)
{
    platformMock_CommInitReceiveData("$?#g3");
    tryPacketGet();
    validateThatEmptyGdbPacketWasRead();
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("-+") ); 
}

TEST(Packet, PacketGetFromGDB_TwoPackets)
{
    platformMock_CommInitReceiveData("$#00$?#3f");
    tryPacketGet();
    validateBufferMatches("?");
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("+") );
}

TEST(Packet, PacketGetFromGDB_SearchForStartOfPacket)
{
    platformMock_CommInitReceiveData("#00$?#3f");
    tryPacketGet();
    validateBufferMatches("?");
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("+") );
}

TEST(Packet, PacketGetFromGDB_StartOfPacketWithinPacket)
{
    platformMock_CommInitReceiveData("$?$?#3f");
    tryPacketGet();
    validateBufferMatches("?");
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("+") );
}

TEST(Packet, PacketGetFromGDB_BufferTooSmall)
{
    platformMock_CommInitReceiveData("$qSupported:qRelocInsn+#9af");
    allocateBuffer(8);
    tryPacketGet();
    validateThatEmptyGdbPacketWasRead();
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("+") );
}

TEST(Packet, PacketSendToGDB_EmptyWithAck)
{
    allocateBuffer("");
    platformMock_CommInitReceiveData("+");
    tryPacketSend();
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("$#00") );
}

TEST(Packet, PacketSendToGDB_OkWithAck)
{
    allocateBuffer("OK");
    platformMock_CommInitReceiveData("+");
    tryPacketSend();
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("$OK#9a") );
}

TEST(Packet, PacketSendToGDB_OkWithNackAndAck)
{
    allocateBuffer("OK");
    platformMock_CommInitReceiveData("-+");
    tryPacketSend();
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("$OK#9a$OK#9a") );
}

TEST(Packet, PacketSendToGDB_OkWithCancelForNewPacket)
{
    allocateBuffer("OK");
    platformMock_CommInitReceiveData("$");
    tryPacketSend();
    CHECK_TRUE( platformMock_CommDoesTransmittedDataEqual("$OK#9a") );
}
