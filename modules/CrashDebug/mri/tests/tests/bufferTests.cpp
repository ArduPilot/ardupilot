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
#include <limits.h>

extern "C"
{
#include "buffer.h"
#include "try_catch.h"
}

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"

TEST_GROUP(Buffer)
{
    Buffer            m_buffer;
    char*             m_pCharacterArray;
    size_t            m_bufferSize;
    int               m_exceptionThrown;
    int               m_validateBufferLimits;
    static const char m_fillChar = 0xFF;
    
    void setup()
    {
        m_exceptionThrown = 0;
        m_validateBufferLimits = 1;
        memset(&m_buffer, 0, sizeof(m_buffer));
    }

    void teardown()
    {
        validateBufferLimits();
        free(m_pCharacterArray);
    }
    
    void allocateBuffer(size_t sizeOfBuffer)
    {
        m_pCharacterArray = (char*)malloc(sizeOfBuffer);
        if (sizeOfBuffer > 0)
            memset(m_pCharacterArray, m_fillChar, sizeOfBuffer);
        m_bufferSize = sizeOfBuffer;
        Buffer_Init(&m_buffer, m_pCharacterArray, m_bufferSize);
    }
    
    void allocateBuffer(const char* pString)
    {
        allocateBuffer(strlen(pString));
        memcpy(m_pCharacterArray, pString, strlen(pString));
    }
    
    void validateBufferLimits()
    {
        if (!m_validateBufferLimits)
            return;
            
        POINTERS_EQUAL(m_pCharacterArray, m_buffer.pStart);
        POINTERS_EQUAL(m_pCharacterArray + m_bufferSize, m_buffer.pEnd);
    }
    
    void validateResetBuffer()
    {
        POINTERS_EQUAL(m_pCharacterArray, m_buffer.pCurrent);
    }
    
    void validateDepletedBufferNoOverrun()
    {
        validateNoException();
        CHECK_FALSE( Buffer_OverrunDetected(&m_buffer) );
        LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    }
    
    void validateDepletedBufferWithOverrun()
    {
        validateBufferOverrunException();
        CHECK_TRUE( Buffer_OverrunDetected(&m_buffer) );
        LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    }
    
    void validateInvalidHexDigitException()
    {
        CHECK_TRUE( m_exceptionThrown );
        LONGS_EQUAL( invalidHexDigitException, getExceptionCode() );
    }
    
    void validateInvalidValueException()
    {
        CHECK_TRUE( m_exceptionThrown );
        LONGS_EQUAL( invalidValueException, getExceptionCode() );
    }
    
    void validateBufferOverrunException()
    {
        CHECK_TRUE( m_exceptionThrown );
        LONGS_EQUAL( bufferOverrunException, getExceptionCode() );
    }
    
    void validateNoException()
    {
        CHECK_FALSE( m_exceptionThrown );
        LONGS_EQUAL( noException, getExceptionCode() );
    }
    
    void writeSpaces(size_t numberToWrite)
    {
        for (size_t i = 0 ; i < numberToWrite ; i++)
        {
            Buffer_WriteChar(&m_buffer, ' ');
        }
    }
};

TEST(Buffer, Buffer_Init)
{
    allocateBuffer(1);
    validateResetBuffer();
    BYTES_EQUAL(m_fillChar, m_pCharacterArray[0]);
}

TEST(Buffer, Buffer_Reset)
{
    allocateBuffer(1);
    m_buffer.pCurrent = NULL;
    Buffer_Reset(&m_buffer);
    validateResetBuffer();
    BYTES_EQUAL(m_fillChar, m_pCharacterArray[0]);
}

TEST(Buffer, Buffer_GetLength_0)
{
    allocateBuffer((size_t)0);
    
    LONGS_EQUAL( 0, Buffer_GetLength(&m_buffer) );
}

TEST(Buffer, Buffer_GetLength_WithInvalidPointers)
{
    m_buffer.pStart = (char*)~0;
    m_buffer.pEnd = NULL;
    LONGS_EQUAL( 0, Buffer_GetLength(&m_buffer) );
    allocateBuffer(1);
}

TEST(Buffer, Buffer_GetLength_512)
{
    allocateBuffer(512);
    
    LONGS_EQUAL( 512, Buffer_GetLength(&m_buffer) );
}

TEST(Buffer, Buffer_SetEndOfBuffer)
{
    allocateBuffer(512);
    
    writeSpaces(128);
    Buffer_SetEndOfBuffer(&m_buffer);
    LONGS_EQUAL( 128, Buffer_GetLength(&m_buffer) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    m_validateBufferLimits = 0;
}

TEST(Buffer, Buffer_SetEndOfBuffer_OnFullBuffer)
{
    allocateBuffer(128);
    
    writeSpaces(128);
    
    Buffer_SetEndOfBuffer(&m_buffer);
    LONGS_EQUAL( 128, Buffer_GetLength(&m_buffer) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
}

TEST(Buffer, Buffer_SetEndOfBuffer_OnOverflownBuffer)
{
    allocateBuffer(1);

    __try
        writeSpaces(2);
    __catch
        m_exceptionThrown = 1;

    validateDepletedBufferWithOverrun();
    Buffer_SetEndOfBuffer(&m_buffer);
    LONGS_EQUAL( 1, Buffer_GetLength(&m_buffer) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
}

TEST(Buffer, Buffer_GetArray)
{
    allocateBuffer(1);
    
    POINTERS_EQUAL( m_pCharacterArray, Buffer_GetArray(&m_buffer) );
}

TEST(Buffer, Buffer_BytesLeft_Empty)
{
    allocateBuffer(1);
    size_t bytesLeft = Buffer_BytesLeft(&m_buffer);
    LONGS_EQUAL(1, bytesLeft);
}

TEST(Buffer, Buffer_ByteLeft_Overrun_NoThrow)
{
    size_t bytesLeft = ~0U;
    
    allocateBuffer(1);
    writeSpaces(2);
    
    __try
        bytesLeft = Buffer_BytesLeft(&m_buffer);
    __catch
        FAIL( "Buffer_BytesLeft() threw exception." );
        
    LONGS_EQUAL(bytesLeft, 0);
}

TEST(Buffer, Buffer_WriteChar_Full_No_Overrun)
{
    allocateBuffer(1);

    __try
        writeSpaces(1);
    __catch
        m_exceptionThrown = 1;
        
    BYTES_EQUAL( ' ', m_pCharacterArray[0]);
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_WriteChar_Full_Overrun)
{
    allocateBuffer(1);

    __try
        writeSpaces(2);
    __catch
        m_exceptionThrown = 1;

    BYTES_EQUAL( ' ', m_pCharacterArray[0] );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_ReadChar_No_Overrun)
{
    char         characterRead = 0x00;
    
    allocateBuffer(1);

    __try
        characterRead = Buffer_ReadChar(&m_buffer);
    __catch
        m_exceptionThrown = 1;

    BYTES_EQUAL( m_fillChar, characterRead );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_ReadChar_Overrun)
{
    char         characterRead = 0x00;
    
    allocateBuffer(1);
    Buffer_ReadChar(&m_buffer);

    __try
        characterRead = Buffer_ReadChar(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    BYTES_EQUAL( '\0', characterRead );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_WriteByteAsHex_Full_No_Overrun)
{
    static const unsigned char testByte = 0x5A;
    static const char          expectedString[] = "5a";
    
    allocateBuffer(2);

    __try
        Buffer_WriteByteAsHex(&m_buffer, testByte);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, 2) );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_WriteByteAsHex_Full_OverrunBy2Bytes)
{
    static const unsigned char testByte = 0x5A;
    static const char          expectedString[] = "5a";
    
    allocateBuffer(2);

    Buffer_WriteByteAsHex(&m_buffer, testByte);
    __try
        Buffer_WriteByteAsHex(&m_buffer, testByte);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, 2) );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_WriteByteAsHex_Full_OverrunBy1Bytes)
{
    static const unsigned char testByte = 0x5A;
    static const char          expectedString[] = "5a";
    
    allocateBuffer(3);

    Buffer_WriteByteAsHex(&m_buffer, testByte);
    __try
        Buffer_WriteByteAsHex(&m_buffer, testByte);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, 2) );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_ReadByteAsHex_Lowercase)
{
    static const unsigned char testByte = 0x5A;
    static const char          testString[] = "5a";
    unsigned char              byteRead = 0;
    
    allocateBuffer(testString);

    __try
        byteRead = Buffer_ReadByteAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;

    BYTES_EQUAL( testByte, byteRead );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_ReadByteAsHex_Uppercase)
{
    static const unsigned char testByte = 0x5A;
    static const char          testString[] = "5A";
    unsigned char              byteRead = 0;
    
    allocateBuffer(testString);

    __try
        byteRead = Buffer_ReadByteAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    BYTES_EQUAL( testByte, byteRead );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_ReadByteAsHex_InvalidFirstHexDigit)
{
    unsigned char       byteRead = 0;
    static const char   testString[] = "\xFF\xFF";
    
    allocateBuffer(testString);
    __try
        byteRead = Buffer_ReadByteAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    BYTES_EQUAL( 0x00, byteRead );
    LONGS_EQUAL( 2, Buffer_BytesLeft(&m_buffer) );
    validateInvalidHexDigitException();
}

TEST(Buffer, Buffer_ReadByteAsHex_InvalidSecondHexDigit)
{
    unsigned char       byteRead = 0;
    static const char   testString[] = "f\xFF";
    
    allocateBuffer(testString);
    
    __try
        byteRead = Buffer_ReadByteAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    BYTES_EQUAL( 0x00, byteRead );
    LONGS_EQUAL( 2, Buffer_BytesLeft(&m_buffer) );
    validateInvalidHexDigitException();
}

TEST(Buffer, Buffer_ReadByteAsHex_Full_Overrun)
{
    static const char          testString[] = "5a";
    unsigned char              byteRead = 0;
    
    allocateBuffer(testString);

    Buffer_ReadByteAsHex(&m_buffer);
    __try
        byteRead = Buffer_ReadByteAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    BYTES_EQUAL( 0x00, byteRead );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_ReadByteAsHex_Full_OverrunBy1)
{
    static const char          testString[] = "5af";
    unsigned char              byteRead = 0;
    
    allocateBuffer(testString);

    Buffer_ReadByteAsHex(&m_buffer);
    __try
        byteRead = Buffer_ReadByteAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    BYTES_EQUAL( 0x00, byteRead );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_WriteString_Full_No_Overrun)
{
    static const char   testString[] = "Hi";
    
    allocateBuffer(2);
    __try
        Buffer_WriteString(&m_buffer, testString);
    __catch
        m_exceptionThrown = 1;

    CHECK( 0 == memcmp(m_pCharacterArray, testString, 2) );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_WriteString_Full_Overrun)
{
    static const char   testString[] = "Hi";

    allocateBuffer(1);
    __try
        Buffer_WriteString(&m_buffer, testString);
    __catch
        m_exceptionThrown = 1;
    BYTES_EQUAL( m_fillChar, m_pCharacterArray[0] );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_WriteSizedString_Full_No_Overrun)
{
    static const char   testString[] = "Hi";
    
    allocateBuffer(2);
    __try
        Buffer_WriteSizedString(&m_buffer, testString, 2);
    __catch
        m_exceptionThrown = 1;

    CHECK( 0 == memcmp(m_pCharacterArray, testString, 2) );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_WriteSizedString_Full_Overrun)
{
    static const char   testString[] = "Hi";

    allocateBuffer(1);
    __try
        Buffer_WriteSizedString(&m_buffer, testString, 2);
    __catch
        m_exceptionThrown = 1;
    BYTES_EQUAL( m_fillChar, m_pCharacterArray[0] );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_ReadUIntegerAsHex_JustComma)
{
    static const char testString[] = ",";
    uint32_t          value = ~0U;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadUIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0, value );
    LONGS_EQUAL( 1, Buffer_BytesLeft(&m_buffer) );
    validateInvalidValueException();
}

TEST(Buffer, Buffer_ReadUIntegerAsHex_Empty)
{
    static const char testString[] = "";
    uint32_t          value = ~0U;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadUIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0, value );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateInvalidValueException();
}

TEST(Buffer, Buffer_ReadUIntegerAsHex_0)
{
    static const char testString[] = "0";
    uint32_t          value = ~0U;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadUIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0, value );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_ReadUIntegerAsHex_f)
{
    static const char testString[] = "f";
    uint32_t          value = ~0U;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadUIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0xf, value );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_ReadUIntegerAsHex_F)
{
    static const char testString[] = "F";
    uint32_t          value = ~0U;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadUIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0xf, value );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_ReadUIntegerAsHex_00000000)
{
    static const char testString[] = "00000000";
    uint32_t          value = ~0U;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadUIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0x0, value );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_ReadUIntegerAsHex_ffffffff)
{
    static const char testString[] = "ffffffff";
    uint32_t          value = 0;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadUIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0xffffffff, value );
    validateDepletedBufferNoOverrun();
}

TEST(Buffer, Buffer_ReadUIntegerAsHex_12345678comma)
{
    static const char testString[] = "12345678,";
    uint32_t          value = ~0U;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadUIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0x12345678, value );
    LONGS_EQUAL( 1, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteUIntegerAsHex_0)
{
    static const char expectedString[] = "00";
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteUIntegerAsHex(&m_buffer, 0);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteUIntegerAsHex_Full_Overrun)
{
    allocateBuffer(1);
    __try
        Buffer_WriteUIntegerAsHex(&m_buffer, 0);
    __catch
        m_exceptionThrown = 1;
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_WriteUIntegerAsHex_0f)
{
    static const char expectedString[] = "0f";
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteUIntegerAsHex(&m_buffer, 0x0F);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteUIntegerAsHex_0fed_Overrun)
{
    static const char expectedString[] = "0fed";
    
    allocateBuffer(2);
    __try
        Buffer_WriteUIntegerAsHex(&m_buffer, 0x0FED);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, 2) );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_WriteUIntegerAsHex_12)
{
    static const char expectedString[] = "12";
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteUIntegerAsHex(&m_buffer, 0x12);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteUIntegerAsHex_1234)
{
    static const char expectedString[] = "1234";
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteUIntegerAsHex(&m_buffer, 0x1234);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteUIntegerAsHex_123456)
{
    static const char expectedString[] = "123456";
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteUIntegerAsHex(&m_buffer, 0x123456);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteUIntegerAsHex_01234567)
{
    static const char expectedString[] = "01234567";
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteUIntegerAsHex(&m_buffer, 0x01234567);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteUIntegerAsHex_12345678)
{
    static const char expectedString[] = "12345678";
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteUIntegerAsHex(&m_buffer, 0x12345678);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_0)
{
    static const char testString[] = "0";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0, value );
    validateNoException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_Neg0)
{
    static const char testString[] = "-0";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0, value );
    validateNoException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_00)
{
    static const char testString[] = "00";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0, value );
    validateNoException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_1)
{
    static const char testString[] = "1";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 1, value );
    validateNoException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_Neg1)
{
    static const char testString[] = "-1";
    int32_t           value = 0;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( -1, value );
    validateNoException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_Empty)
{
    int32_t           value = -1;
    
    allocateBuffer((size_t)0);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0, value );
    validateBufferOverrunException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_NegComma)
{
    static const char testString[] = "-,";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0, value );
    validateInvalidValueException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_Neg)
{
    static const char testString[] = "-";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0, value );
    validateInvalidValueException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_7fffffff)
{
    static const char testString[] = "7fffffff";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( 0x7FFFFFFF, value );
    validateNoException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_Neg80000000)
{
    static const char testString[] = "-80000000";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( INT_MIN, value );
    validateNoException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_Neg80000001)
{
    static const char testString[] = "-80000001";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( INT_MIN, value );
    validateInvalidValueException();
}

TEST(Buffer, Buffer_ReadIntegerAsHex_80000000)
{
    static const char testString[] = "80000000";
    int32_t           value = -1;
    
    allocateBuffer(testString);
    __try
        value = Buffer_ReadIntegerAsHex(&m_buffer);
    __catch
        m_exceptionThrown = 1;
    LONGS_EQUAL( INT_MAX, value );
    validateInvalidValueException();
}

TEST(Buffer, Buffer_WriteIntegerAsHex_00)
{
    static const char       expectedString[] = "00";
    static const int32_t    testValue = 0;
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteIntegerAsHex(&m_buffer, testValue);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteIntegerAsHex_1)
{
    static const char       expectedString[] = "01";
    static const int32_t    testValue = 1;
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteIntegerAsHex(&m_buffer, testValue);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteIntegerAsHex_Neg1)
{
    static const char       expectedString[] = "-01";
    static const int32_t    testValue = -1;
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteIntegerAsHex(&m_buffer, testValue);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteIntegerAsHex_MAXINT)
{
    static const char       expectedString[] = "7fffffff";
    static const int32_t    testValue = INT_MAX;
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteIntegerAsHex(&m_buffer, testValue);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteIntegerAsHex_MININT)
{
    static const char       expectedString[] = "-80000000";
    static const int32_t    testValue = INT_MIN;
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteIntegerAsHex(&m_buffer, testValue);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    validateNoException();
}

TEST(Buffer, Buffer_WriteIntegerAsHex_Empty)
{
    static const int32_t    testValue = -1;
    
    allocateBuffer((size_t)0);
    __try
        Buffer_WriteIntegerAsHex(&m_buffer, testValue);
    __catch
        m_exceptionThrown = 1;
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_WriteIntegerAsHex_RoomForMinusOnly)
{
    static const char       expectedString[] = "-";
    static const int32_t    testValue = -1;
    
    allocateBuffer(strlen(expectedString));
    __try
        Buffer_WriteIntegerAsHex(&m_buffer, testValue);
    __catch
        m_exceptionThrown = 1;
    CHECK( 0 == memcmp(m_pCharacterArray, expectedString, strlen(expectedString)) );
    validateDepletedBufferWithOverrun();
}

TEST(Buffer, Buffer_IsNextCharEqualTo_Empty)
{
    static const char   testString[] = "";
    static const char   testChar = ':';
    int                 isEqual = -1;
    
    allocateBuffer(testString);
    __try
        isEqual = Buffer_IsNextCharEqualTo(&m_buffer, testChar);
    __catch
        m_exceptionThrown = 1;
    CHECK_FALSE( isEqual );
    validateBufferOverrunException();
}

TEST(Buffer, Buffer_IsNextCharEqualTo_Match)
{
    static const char   testString[] = ":";
    static const char   testChar = ':';
    int                 isEqual = -1;
    
    allocateBuffer(testString);
    __try
        isEqual = Buffer_IsNextCharEqualTo(&m_buffer, testChar);
    __catch
        m_exceptionThrown = 1;
    CHECK_TRUE( isEqual );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_IsNextCharEqualTo_NoMatch)
{
    static const char   testString[] = ",";
    static const char   testChar = ':';
    int                 isEqual = -1;
    
    allocateBuffer(testString);
    __try
        isEqual = Buffer_IsNextCharEqualTo(&m_buffer, testChar);
    __catch
        m_exceptionThrown = 1;
    CHECK_FALSE( isEqual );
    LONGS_EQUAL( 1, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_MatchesString_TooSmall)
{
    static const char   testString[] = "String";
    static const char   compareString[] = "StringMatch";
    int                 isEqual = -1;
    
    allocateBuffer(testString);
    __try
        isEqual = Buffer_MatchesString(&m_buffer, compareString, sizeof(compareString)-1);
    __catch
        m_exceptionThrown = 1;
    CHECK_FALSE( isEqual );
    validateBufferOverrunException();
}

TEST(Buffer, Buffer_MatchesString_Match)
{
    static const char   testString[] = "StringMatch";
    static const char   compareString[] = "StringMatch";
    int                 isEqual = -1;
    
    allocateBuffer(testString);
    __try
        isEqual = Buffer_MatchesString(&m_buffer, compareString, sizeof(compareString)-1);
    __catch
        m_exceptionThrown = 1;
    CHECK_TRUE( isEqual );
    LONGS_EQUAL( 0, Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}

TEST(Buffer, Buffer_MatchesString_NoMatch)
{
    static const char   testString[] = "StringMatch";
    static const char   compareString[] = "StringMatc?";
    int                 isEqual = -1;
    
    allocateBuffer(testString);
    __try
        isEqual = Buffer_MatchesString(&m_buffer, compareString, sizeof(compareString)-1);
    __catch
        m_exceptionThrown = 1;
    CHECK_FALSE( isEqual );
    LONGS_EQUAL( strlen(testString), Buffer_BytesLeft(&m_buffer) );
    validateNoException();
}
