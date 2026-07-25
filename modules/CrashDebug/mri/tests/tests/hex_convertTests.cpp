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
#include "hex_convert.h"
}

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"

TEST_GROUP(HexConvert)
{
    void setup()
    {
    }

    void teardown()
    {
    }
};

TEST(HexConvert, ExtractLowNibble)
{
    uint8_t value = 0xaf;
    
    LONGS_EQUAL( 0xf, EXTRACT_LO_NIBBLE(value) );
}

TEST(HexConvert, ExtractHighNibble)
{
    uint8_t value = 0xfa;
    
    LONGS_EQUAL( 0xf, EXTRACT_HI_NIBBLE(value) );
}

TEST(HexConvert, NibbleToHexChar0)
{
    uint8_t nibble = 0x0;
    
    BYTES_EQUAL( '0', NibbleToHexChar[nibble] );
}

TEST(HexConvert, NibbleToHexCharF)
{
    uint8_t nibble = 0xF;
    
    BYTES_EQUAL( 'f', NibbleToHexChar[nibble] );
}

TEST(HexConvert, HexCharToNibble_0)
{
    __try
    {
        BYTES_EQUAL( 0, HexCharToNibble('0') );
    }
    __catch
    {
        FAIL( "HexCharToNibble threw unexpected exception." );
    }
}

TEST(HexConvert, HexCharToNibble_9)
{
    __try
    {
        BYTES_EQUAL( 9, HexCharToNibble('9') );
    }
    __catch
    {
        FAIL( "HexCharToNibble threw unexpected exception." );
    }
}

TEST(HexConvert, HexCharToNibble_a)
{
    __try
    {
        BYTES_EQUAL( 0xa, HexCharToNibble('a') );
    }
    __catch
    {
        FAIL( "HexCharToNibble threw unexpected exception." );
    }
}

TEST(HexConvert, HexCharToNibble_f)
{
    __try
    {
        BYTES_EQUAL( 0xf, HexCharToNibble('f') );
    }
    __catch
    {
        FAIL( "HexCharToNibble threw unexpected exception." );
    }
}

TEST(HexConvert, HexCharToNibble_A)
{
    __try
    {
        BYTES_EQUAL( 0xa, HexCharToNibble('A') );
    }
    __catch
    {
        FAIL( "HexCharToNibble threw unexpected exception." );
    }
}

TEST(HexConvert, HexCharToNibble_F)
{
    __try
    {
        BYTES_EQUAL( 0xf, HexCharToNibble('F') );
    }
    __catch
    {
        FAIL( "HexCharToNibble threw unexpected exception." );
    }
}

TEST(HexConvert, HexCharToNibble_InvalidG)
{
    int value;
    int exceptionThrown = 0;
    
    __try
        value = HexCharToNibble('G');
    __catch
        exceptionThrown = 1;
        
    LONGS_EQUAL( -1, value );
    CHECK_TRUE( exceptionThrown );
    LONGS_EQUAL( invalidHexDigitException, getExceptionCode() );
}

TEST(HexConvert, HexCharToNibble_Invalidg)
{
    int value;
    int exceptionThrown = 0;
    
    __try
        value = HexCharToNibble('g');
    __catch
        exceptionThrown = 1;
        
    LONGS_EQUAL( -1, value );
    CHECK_TRUE( exceptionThrown );
    LONGS_EQUAL( invalidHexDigitException, getExceptionCode() );
}
