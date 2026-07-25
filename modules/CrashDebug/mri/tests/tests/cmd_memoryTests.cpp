/* Copyright 2017 Adam Green (http://mbed.org/users/AdamGreen/)

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
#include <stdio.h>

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"


TEST_GROUP(cmdMemory)
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

TEST(cmdMemory, MemoryRead32Aligned)
{
    uint32_t value = 0x12345678;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,4#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$78563412#a4+") );
}

TEST(cmdMemory, MemoryRead16Aligned)
{
    uint16_t value = 0x1234;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,2#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$3412#ca+") );
}

TEST(cmdMemory, MemoryRead8)
{
    uint8_t  value = 0x12;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,1#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$12#63+") );
}

TEST(cmdMemory, MemoryRead_InvalidParameterSeparator_ErrorResponse)
{
    uint8_t  value = 0x12;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x:1#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_INVALID_ARGUMENT "#a6+") );
}

TEST(cmdMemory, MemoryRead_PacketBufferTooSmall_ShouldReturnOverflowResponse)
{
    uint8_t  value[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,8#", (uint32_t)(size_t)value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_SetPacketBufferSize(15);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_BUFFER_OVERRUN "#a9+") );
}

TEST(cmdMemory, MemoryRead32Unaligned)
{
    uint32_t value[2] = { 0x12345678, 0x9abcdef0 };
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,4#", ((uint32_t)(size_t)value) + 2);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$3412f0de#29+") );
}

TEST(cmdMemory, MemoryRead16Unaligned)
{
    uint16_t value[2] = { 0x1234, 0x5678 };
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,2#", ((uint32_t)(size_t)value) + 1);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$1278#d2+") );
}

TEST(cmdMemory, MemoryRead32_FaultAndReturnNoBytes)
{
    uint32_t value = 0x12345678;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,4#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(1);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$E03#a8+") );
}

TEST(cmdMemory, MemoryRead16_FaultAndReturnNoBytes)
{
    uint16_t value = 0x1234;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,2#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(1);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$E03#a8+") );
}

TEST(cmdMemory, MemoryRead8_FaultAndReturnNoBytes)
{
    uint8_t  value = 0x12;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,1#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(1);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$E03#a8+") );
}

TEST(cmdMemory, MemoryRead8_FaultOnLastOfThreeBytes)
{
    uint8_t  values[3] = {0x12, 0x34, 0x56};
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$m%08x,3#", (uint32_t)(size_t)values);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(3);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$1234#ca+") );
}



TEST(cmdMemory, MemoryWrite32Aligned)
{
    uint32_t value = 0xFFFFFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,4:12345678#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x78563412, value );
}

TEST(cmdMemory, MemoryWrite16Aligned)
{
    uint16_t value = 0xFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,2:1234#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x3412, value );
}

TEST(cmdMemory, MemoryWrite32_FaultOnFinalWrite)
{
    uint32_t value = 0xFFFFFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,4:12345678#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(1);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#a8+") );
}

TEST(cmdMemory, MemoryWrite16_FaultOnFinalWrite)
{
    uint16_t value = 0xFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,2:1234#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(1);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#a8+") );
 }
 
 
TEST(cmdMemory, MemoryWrite8)
{
    uint8_t  value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,1:12#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x12, value );
}

TEST(cmdMemory, MemoryWrite8_InvalidParameterSeparator_ShouldReturnError)
{
    uint8_t  value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,1,12#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_INVALID_ARGUMENT "#a6+") );
    CHECK_EQUAL ( 0xFF, value );
}

TEST(cmdMemory, MemoryWrite32Unaligned)
{
    uint32_t value[2] = { 0xFFFFFFFF, 0xFFFFFFFF };
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,4:12345678#", ((uint32_t)(size_t)value) + 2);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x3412FFFF, value[0] );
    CHECK_EQUAL ( 0xFFFF7856, value[1] );
}

TEST(cmdMemory, MemoryWrite16Unaligned)
{
    uint16_t value[2] = { 0xFFFF, 0xFFFF };
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,2:1234#", ((uint32_t)(size_t)value) + 1);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x12FF, value[0] );
    CHECK_EQUAL ( 0xFF34, value[1] );
}

TEST(cmdMemory, MemoryWrite32_TooFewBytesInPacket_ShouldReturnErrorAndNotModifyMemory)
{
    uint32_t value = 0xFFFFFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,4:123456#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_BUFFER_OVERRUN "#a9+") );
    CHECK_EQUAL ( 0xFFFFFFFF, value );
}

TEST(cmdMemory, MemoryWrite16_TooFewBytesInPacket_ShouldReturnErrorAndNotModifyMemory)
{
    uint16_t value = 0xFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,2:12#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_BUFFER_OVERRUN "#a9+") );
    CHECK_EQUAL ( 0xFFFF, value );
}

TEST(cmdMemory, MemoryWrite8_TooFewBytesInPacket_ShouldReturnErrorAndNotModifyMemory)
{
    uint8_t  value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,1:#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_BUFFER_OVERRUN "#a9+") );
    CHECK_EQUAL ( 0xFF, value );
}

TEST(cmdMemory, MemoryWrite8_FaultOnFirstWrite)
{
    uint8_t value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,1:12#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(1);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#a8+") );
}

TEST(cmdMemory, MemoryWrite8_FaultOnSecondWrite)
{
    uint8_t value[3] = {0xFF, 0xFF, 0xFF};
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$M%08x,3:010203#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(2);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#a8+") );
    CHECK_EQUAL( 1, value[0]);
    // This next write made it through since the mocked fault is only checked after actual native write.
    CHECK_EQUAL( 2, value[1]);
    // It shouldn't make it to this next memory write though.
    CHECK_EQUAL( 0xFF, value[2]);
}



TEST(cmdMemory, BinaryMemoryWrite32Aligned)
{
    uint32_t value = 0xFFFFFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,4:\x12\x34\x56\x78#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x78563412, value );
}

TEST(cmdMemory, BinaryMemoryWrite16Aligned)
{
    uint16_t value = 0xFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,2:\x12\x34#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x3412, value );
}

TEST(cmdMemory, BinaryMemoryWrite32_FaultOnFinalWraite)
{
    uint32_t value = 0xFFFFFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,4:\x12\x34\x56\x78#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(1);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#a8+") );
}

TEST(cmdMemory, BinaryMemoryWrite16_FaultOnFinalWrite)
{
    uint16_t value = 0xFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,2:\x12\x34#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(1);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#a8+") );
}

TEST(cmdMemory, BinaryMemoryWrite8)
{
    uint8_t  value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,1:\x12#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x12, value );
}

TEST(cmdMemory, BinaryMemoryWrite8_InvalidParameterSeparator_ShouldReturnError)
{
    uint8_t  value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,1,\x12#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_INVALID_ARGUMENT "#a6+") );
    CHECK_EQUAL ( 0xFF, value );
}

TEST(cmdMemory, BinaryMemoryWrite32Unaligned)
{
    uint32_t value[2] = { 0xFFFFFFFF, 0xFFFFFFFF };
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,4:\x12\x34\x56\x78#", ((uint32_t)(size_t)value) + 2);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x3412FFFF, value[0] );
    CHECK_EQUAL ( 0xFFFF7856, value[1] );
}

TEST(cmdMemory, BinaryMemoryWrite16Unaligned)
{
    uint16_t value[2] = { 0xFFFF, 0xFFFF };
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,2:\x12\x34#", ((uint32_t)(size_t)value) + 1);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( 0x12FF, value[0] );
    CHECK_EQUAL ( 0xFF34, value[1] );
}

TEST(cmdMemory, BinaryMemoryWrite32_TooFewBytesInPacket_ShouldReturnErrorAndNotModifyMemory)
{
    uint32_t value = 0xFFFFFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,4:\x12\x34\x56#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_BUFFER_OVERRUN "#a9+") );
    CHECK_EQUAL ( 0xFFFFFFFF, value );
}

TEST(cmdMemory, BinaryMemoryWrite16_TooFewBytesInPacket_ShouldReturnErrorAndNotModifyMemory)
{
    uint16_t value = 0xFFFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,2:\x12#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_BUFFER_OVERRUN "#a9+") );
    CHECK_EQUAL ( 0xFFFF, value );
}

TEST(cmdMemory, BinaryMemoryWrite8_TooFewBytesInPacket_ShouldReturnErrorAndNotModifyMemory)
{
    uint8_t  value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,1:#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_BUFFER_OVERRUN "#a9+") );
    CHECK_EQUAL ( 0xFF, value );
}

TEST(cmdMemory, BinaryMemoryWrite8_FaultOnFirstWrite)
{
    uint8_t value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,1:\x12#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(1);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#a8+") );
}

TEST(cmdMemory, BinaryMemoryWrite8_FaultOnSecondWrite)
{
    uint8_t value[3] = {0xFF, 0xFF, 0xFF};
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,3:\x01\x02\x03#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
    platformMock_FaultOnSpecificMemoryCall(2);
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_MEMORY_ACCESS_FAILURE "#a8+") );
    CHECK_EQUAL( 1, value[0]);
    // This next write made it through since the mocked fault is only checked after actual native write.
    CHECK_EQUAL( 2, value[1]);
    // It shouldn't make it to this next memory write though.
    CHECK_EQUAL( 0xFF, value[2]);
}

TEST(cmdMemory, BinaryMemoryWrite8_UseEscapedByte)
{
    uint8_t  value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,1:}%c#", (uint32_t)(size_t)&value, '}' ^ 0x20);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$OK#9a+") );
    CHECK_EQUAL ( '}', value );
}

TEST(cmdMemory, BinaryMemoryWrite8_BufferTooSmallForEscapedByte_ReturnErrorResponse)
{
    uint8_t  value = 0xFF;
    char     packet[64];
    snprintf(packet, sizeof(packet), "+$X%08x,1:}#", (uint32_t)(size_t)&value);
    platformMock_CommInitReceiveChecksummedData(packet, "+$c#");
        __mriDebugException();
    CHECK_TRUE ( platformMock_CommDoesTransmittedDataEqual("$T05responseT#7c+$" MRI_ERROR_BUFFER_OVERRUN "#a9+") );
    CHECK_EQUAL ( 0xFF, value );
}
