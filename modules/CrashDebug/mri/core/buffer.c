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
/*  'Class' which represents a text buffer.  Has routines to both extract and inject strings of various types into the
    buffer while verifying that no overflow takes place. */
#include <limits.h>
#include <string.h>
#include "buffer.h"
#include "hex_convert.h"
#include "try_catch.h"

void Buffer_Init(Buffer* pBuffer, char* pBufferStart, size_t bufferSize)
{
    pBuffer->pStart = pBufferStart;
    pBuffer->pEnd = pBufferStart + bufferSize;
    Buffer_Reset(pBuffer);
}


void Buffer_Reset(Buffer* pBuffer)
{
    pBuffer->pCurrent = pBuffer->pStart;
}


void Buffer_SetEndOfBuffer(Buffer* pBuffer)
{
    if (pBuffer->pCurrent < pBuffer->pEnd)
        pBuffer->pEnd = pBuffer->pCurrent;
}


size_t Buffer_BytesLeft(Buffer* pBuffer)
{
    if (Buffer_OverrunDetected(pBuffer))
        return 0;

    return (size_t)(pBuffer->pEnd - pBuffer->pCurrent);
}


size_t Buffer_GetLength(Buffer* pBuffer)
{
    if (pBuffer->pStart > pBuffer->pEnd)
        return 0;

    return (size_t)(pBuffer->pEnd - pBuffer->pStart);
}


char* Buffer_GetArray(Buffer* pBuffer)
{
    return pBuffer->pStart;
}


int Buffer_OverrunDetected(Buffer* pBuffer)
{
    return pBuffer->pCurrent > pBuffer->pEnd;
}


static void throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(Buffer* pBuffer, size_t size);
static void recordThatBufferOverrunHasOccurred(Buffer* pBuffer);
void Buffer_WriteChar(Buffer* pBuffer, char character)
{
    __try
        throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, 1);
    __catch
        __rethrow;

    *(pBuffer->pCurrent++) = character;
}

static void throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(Buffer* pBuffer, size_t size)
{
    if (Buffer_BytesLeft(pBuffer) < size)
    {
        recordThatBufferOverrunHasOccurred(pBuffer);
        __throw(bufferOverrunException);
    }
}

static void recordThatBufferOverrunHasOccurred(Buffer* pBuffer)
{
    pBuffer->pCurrent = pBuffer->pEnd + 1;
}


char Buffer_ReadChar(Buffer* pBuffer)
{
    __try
        throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, 1);
    __catch
        __rethrow_and_return('\0');

    return *(pBuffer->pCurrent++);
}


void Buffer_WriteByteAsHex(Buffer* pBuffer, uint8_t byte)
{
    __try
        throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, 2);
    __catch
        __rethrow;

    *(pBuffer->pCurrent++) = NibbleToHexChar[EXTRACT_HI_NIBBLE(byte)];
    *(pBuffer->pCurrent++) = NibbleToHexChar[EXTRACT_LO_NIBBLE(byte)];
}


uint8_t Buffer_ReadByteAsHex(Buffer* pBuffer)
{
    unsigned char byte;

    __try
        throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, 2);
    __catch
        __rethrow_and_return(0);

    __try
    {
        __throwing_func( byte = HexCharToNibble(pBuffer->pCurrent[0]) << 4 );
        __throwing_func( byte |= HexCharToNibble(pBuffer->pCurrent[1]) );
    }
    __catch
    {
        __rethrow_and_return(0x00);
    }
    pBuffer->pCurrent += 2;
    
    return byte;
}


void Buffer_WriteString(Buffer* pBuffer, const char* pString)
{
    Buffer_WriteSizedString(pBuffer, pString, strlen(pString));
}


void Buffer_WriteSizedString(Buffer* pBuffer, const char* pString, size_t length)
{
    __try
        throwExceptionAndFlagBufferOverrunIfBufferLeftIsSmallerThan(pBuffer, length);
    __catch
        __rethrow;
    
    while (length--)
        *(pBuffer->pCurrent++) = *pString++;
}

void Buffer_WriteStringAsHex(Buffer* pBuffer, const char* pString)
{
    while (*pString)
        Buffer_WriteByteAsHex(pBuffer, *pString++);
}

static uint32_t parseNextHexDigitAndAddNibbleToValue(Buffer* pBuffer, uint32_t currentValue);
static void     pushBackLastChar(Buffer* pBuffer);
static void     clearOverrun(Buffer* pBuffer);
uint32_t Buffer_ReadUIntegerAsHex(Buffer* pBuffer)
{
    int      hexDigitsParsed;
    uint32_t value = 0;

    for (hexDigitsParsed = 0 ; ; hexDigitsParsed++)
    {
        __try
            value = parseNextHexDigitAndAddNibbleToValue(pBuffer, value);
        __catch
            break;
    }
    /* Read buffer until non-hex digit or end of buffer was detected but don't want to return the invalid hex digit
       or buffer overrun exception to the caller so clear out the exception code that might have gotten us out of
       the above loop. */
    clearExceptionCode();
    clearOverrun(pBuffer);

    if (hexDigitsParsed == 0)
        __throw_and_return(invalidValueException, 0U);
    
    return value;
}

static uint32_t parseNextHexDigitAndAddNibbleToValue(Buffer* pBuffer, uint32_t currentValue)
{
    char     nextChar;
    uint32_t nibbleValue;
    
    __try
        nextChar = Buffer_ReadChar(pBuffer);
    __catch
        __rethrow_and_return(currentValue);

    __try
    {
        nibbleValue = HexCharToNibble(nextChar);
    }
    __catch
    {
        pushBackLastChar(pBuffer);
        __rethrow_and_return(currentValue);
    }

    return (currentValue << 4) + nibbleValue;
}

static void pushBackLastChar(Buffer* pBuffer)
{
    if (pBuffer->pCurrent > pBuffer->pStart)
        pBuffer->pCurrent--;
}

static void clearOverrun(Buffer* pBuffer)
{
    if (Buffer_OverrunDetected(pBuffer))
        pBuffer->pCurrent = pBuffer->pEnd;
}


static int     countLeadingZeroBytes(uint32_t value);
static uint8_t extractByteAtIndex(uint32_t value, int index);
void Buffer_WriteUIntegerAsHex(Buffer* pBuffer, uint32_t value)
{
    int              leadingZeroBytes;
    int              currentByteIndex;
    
    if (value == 0)
    {
        Buffer_WriteByteAsHex(pBuffer, 0);
        return;
    }
    
    leadingZeroBytes = countLeadingZeroBytes(value);
    currentByteIndex = ((int)sizeof(value) - leadingZeroBytes) - 1;
    while (currentByteIndex >= 0)
    {
        __try
            Buffer_WriteByteAsHex(pBuffer, extractByteAtIndex(value, currentByteIndex--));
        __catch
            __rethrow;
    }
}

static int countLeadingZeroBytes(uint32_t value)
{
    uint32_t mask = 0xFF000000;
    int      count = 0;
    
    while (mask && 0 == (value & mask))
    {
        count++;
        mask >>= 8;
    }
    
    return count;
}

static uint8_t extractByteAtIndex(uint32_t value, int index)
{
    static const int bitsPerByte = 8;
    uint32_t         shiftAmount = index * bitsPerByte;
    
    return (uint8_t)((value >> shiftAmount) & 0xff);
}


static int32_t convertToIntegerAndThrowIfOutOfRange(int isNegative, uint32_t value);
int32_t Buffer_ReadIntegerAsHex(Buffer* pBuffer)
{
    uint32_t value = 0;
    int      isNegative = 0;
    
    __try
    {
        __throwing_func( isNegative = Buffer_IsNextCharEqualTo(pBuffer, '-') );
        __throwing_func( value = Buffer_ReadUIntegerAsHex(pBuffer) );
    }
    __catch
    {
        __rethrow_and_return(0);
    }

    return convertToIntegerAndThrowIfOutOfRange(isNegative, value);
}

static int32_t convertToIntegerAndThrowIfOutOfRange(int isNegative, uint32_t value)
{
    if (!isNegative && value > INT_MAX)
        __throw_and_return(invalidValueException, INT_MAX);
        
    if (isNegative && value > ((uint32_t)INT_MAX + 1))
    {
        __throw_and_return(invalidValueException, INT_MIN);
    }
    
    return isNegative ? -(int)value : (int)value;
}


static int32_t calculateAbsoluteValueAndWriteMinusSignForNegativeValue(Buffer* pBuffer, int32_t value);
void Buffer_WriteIntegerAsHex(Buffer* pBuffer, int32_t value)
{
    __try
    {
        __throwing_func(value = calculateAbsoluteValueAndWriteMinusSignForNegativeValue(pBuffer, value));
        __throwing_func(Buffer_WriteUIntegerAsHex(pBuffer, (uint32_t)value));
    }
    __catch
    {
        __rethrow;
    }
}

static int32_t calculateAbsoluteValueAndWriteMinusSignForNegativeValue(Buffer* pBuffer, int32_t value)
{
    if (value < 0)
    {
        value = -value;
        Buffer_WriteChar(pBuffer, '-');
    }
    
    return value;
}


static void throwExceptionIfBufferLeftIsSmallerThan(Buffer* pBuffer, size_t size);
static char peekAtNextChar(Buffer* pBuffer);
static void advanceToNextChar(Buffer* pBuffer);
int Buffer_IsNextCharEqualTo(Buffer* pBuffer, char thisChar)
{
    __try
        throwExceptionIfBufferLeftIsSmallerThan(pBuffer, 1);
    __catch
        __rethrow_and_return(0);
        
    if (peekAtNextChar(pBuffer) == thisChar)
    {
        advanceToNextChar(pBuffer);
        return 1;
    }

    return 0;
}

static void throwExceptionIfBufferLeftIsSmallerThan(Buffer* pBuffer, size_t size)
{
    if (Buffer_BytesLeft(pBuffer) < size)
        __throw(bufferOverrunException);
}

static char peekAtNextChar(Buffer* pBuffer)
{
    return *pBuffer->pCurrent;
}

static void advanceToNextChar(Buffer* pBuffer)
{
    pBuffer->pCurrent++;
}


static int doesBufferContainThisString(Buffer* pBuffer, const char* pDesiredString, size_t stringLength);
int Buffer_MatchesString(Buffer* pBuffer, const char* pString, size_t stringLength)
{
    __try
        throwExceptionIfBufferLeftIsSmallerThan(pBuffer, stringLength);
    __catch
        __rethrow_and_return(0);
    
    if(doesBufferContainThisString(pBuffer, pString, stringLength))
    {
        pBuffer->pCurrent += stringLength;
        return 1;
    }

    return 0;
}

static int doesBufferContainThisString(Buffer* pBuffer, const char* pDesiredString, size_t stringLength)
{
    const char* pBufferString = pBuffer->pCurrent;
    
    return (strncmp(pBufferString, pDesiredString, stringLength) == 0) &&
           (Buffer_BytesLeft(pBuffer) == stringLength || 
            pBufferString[stringLength] == ':'||
            pBufferString[stringLength] == ';' ||
            pBufferString[stringLength] == ',');
}
