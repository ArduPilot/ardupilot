/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

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
}
#include "mockIComm.h"

// Include C++ headers for test harness.
#include "CppUTest/TestHarness.h"

TEST_GROUP(mockIComm)
{
    void setup()
    {
    }

    void teardown()
    {
        mockIComm_Uninit();
    }
};

TEST(mockIComm, IComm_HasRecieveData_Empty)
{
    static const char emptyData[] = "";

    mockIComm_InitReceiveData(emptyData);
    CHECK_FALSE( IComm_HasReceiveData(mockIComm_Get()) );
}

TEST(mockIComm, IComm_HasRecieveData_NotEmpty)
{
    static const char testData[] = "$";

    mockIComm_InitReceiveData(testData);
    CHECK_TRUE( IComm_HasReceiveData(mockIComm_Get()) );
}

TEST(mockIComm, IComm_HasRecieveData_NotEmpty_ButDelayResponseByTwoCalls)
{
    static const char testData[] = "$";

    mockIComm_InitReceiveData(testData);
    mockIComm_DelayReceiveData(2);
    CHECK_FALSE( IComm_HasReceiveData(mockIComm_Get()) );
    CHECK_FALSE( IComm_HasReceiveData(mockIComm_Get()) );
    CHECK_TRUE( IComm_HasReceiveData(mockIComm_Get()) );
}

TEST(mockIComm, IComm_RecieveChar_NotEmpty)
{
    static const char testData[] = "$";

    mockIComm_InitReceiveData(testData);
    LONGS_EQUAL( '$', IComm_ReceiveChar(mockIComm_Get()) );
}

TEST(mockIComm, IComm_HasReceiveData_SwitchToEmptyGdbPacket)
{
    static const char emptyData[] = "";
    static const char emptyGdbPacket[] = "$#00";
    char              buffer[16];
    char*             p = buffer;

    mockIComm_InitReceiveData(emptyData);
    CHECK_FALSE( IComm_HasReceiveData(mockIComm_Get()) );
    CHECK_TRUE( IComm_HasReceiveData(mockIComm_Get()) );

    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(emptyGdbPacket), (p - buffer) );
    CHECK( 0 == memcmp(emptyGdbPacket, buffer, strlen(emptyGdbPacket)) );
}

TEST(mockIComm, IComm_ReceiveChar_SwitchToEmptyGdbPacket)
{
    static const char emptyData[] = "";
    static const char emptyGdbPacket[] = "$#00";
    char              buffer[16];
    char*             p = buffer;

    mockIComm_InitReceiveData(emptyData);

    do
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    while (IComm_HasReceiveData(mockIComm_Get()));

    LONGS_EQUAL ( strlen(emptyGdbPacket), (p - buffer) );
    CHECK( 0 == memcmp(emptyGdbPacket, buffer, strlen(emptyGdbPacket)) );
}

TEST(mockIComm, mockIComm_ReceiveEmptyGdbPacket)
{
    static const char emptyGdbPacket[] = "$#00";
    char              buffer[16];
    char*             p = buffer;

    mockIComm_InitReceiveData(emptyGdbPacket);

    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(emptyGdbPacket), (p - buffer) );
    CHECK( 0 == memcmp(emptyGdbPacket, buffer, strlen(emptyGdbPacket)) );
}

TEST(mockIComm, mockIComm_Receive_OneGdbPacketAndSwitchToEmptyPacket)
{
    static const char packet1[] = "$packet1#00";
    static const char emptyGdbPacket[] = "$#00";
    char              buffer[16];
    char*             p = buffer;

    mockIComm_InitReceiveData(packet1);

    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(packet1), (p - buffer) );
    CHECK( 0 == memcmp(packet1, buffer, strlen(packet1)) );

    p = buffer;
    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(emptyGdbPacket), (p - buffer) );
    CHECK( 0 == memcmp(emptyGdbPacket, buffer, strlen(emptyGdbPacket)) );

    CHECK_FALSE( IComm_HasReceiveData(mockIComm_Get()) );
}

TEST(mockIComm, mockIComm_Receive_TwoGdbPackets)
{
    static const char packet1[] = "$packet1#00";
    static const char packet2[] = "$packet2#ff";
    char              buffer[16];
    char*             p = buffer;

    mockIComm_InitReceiveData(packet1, packet2);

    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(packet1), (p - buffer) );
    CHECK( 0 == memcmp(packet1, buffer, strlen(packet1)) );

    p = buffer;
    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(packet2), (p - buffer) );
    CHECK( 0 == memcmp(packet2, buffer, strlen(packet2)) );

    CHECK_FALSE( IComm_HasReceiveData(mockIComm_Get()) );
}

TEST(mockIComm, mockIComm_Receive_OneGdbPacketsWithCalculatedCRCAndSwitchToEmptyPacket)
{
    static const char packet1[] = "$packet1#";
    static const char emptyGdbPacket[] = "$#00";
    static const char checksummedPacket1[] = "$packet1#a9";
    char              buffer[16];
    char*             p = buffer;

    mockIComm_InitReceiveChecksummedData(packet1);

    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(checksummedPacket1), (p - buffer) );
    CHECK( 0 == memcmp(checksummedPacket1, buffer, strlen(checksummedPacket1)) );

    p = buffer;
    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(emptyGdbPacket), (p - buffer) );
    CHECK( 0 == memcmp(emptyGdbPacket, buffer, strlen(emptyGdbPacket)) );

    CHECK_FALSE( IComm_HasReceiveData(mockIComm_Get()) );
}

TEST(mockIComm, mockIComm_Receive_TwoGdbPacketsWithCalculatedCRC)
{
    static const char packet1[] = "$packet1#";
    static const char packet2[] = "$packet2#";
    static const char checksummedPacket1[] = "$packet1#a9";
    static const char checksummedPacket2[] = "$packet2#aa";
    char              buffer[16];
    char*             p = buffer;

    mockIComm_InitReceiveChecksummedData(packet1, packet2);

    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(checksummedPacket1), (p - buffer) );
    CHECK( 0 == memcmp(checksummedPacket1, buffer, strlen(checksummedPacket1)) );

    p = buffer;
    while (IComm_HasReceiveData(mockIComm_Get()))
    {
        *p++ = (char)IComm_ReceiveChar(mockIComm_Get());
    }
    LONGS_EQUAL ( strlen(checksummedPacket2), (p - buffer) );
    CHECK( 0 == memcmp(checksummedPacket2, buffer, strlen(checksummedPacket2)) );

    CHECK_FALSE( IComm_HasReceiveData(mockIComm_Get()) );
}

TEST(mockIComm, ChecksumEmptyPacketData)
{
    STRCMP_EQUAL("$#00", mockIComm_ChecksumData("$#"));
}

TEST(mockIComm, TransmitAndCapture1Byte)
{
    mockIComm_InitTransmitDataBuffer(2);

    IComm_SendChar(mockIComm_Get(), '-');

    STRCMP_EQUAL("-", mockIComm_GetTransmittedData());
}

TEST(mockIComm, TransmitAndCapturePacketData)
{
    mockIComm_InitTransmitDataBuffer(4);

    IComm_SendChar(mockIComm_Get(), '$');
    IComm_SendChar(mockIComm_Get(), '#');
    IComm_SendChar(mockIComm_Get(), '0');
    IComm_SendChar(mockIComm_Get(), '0');

    STRCMP_EQUAL(mockIComm_ChecksumData("$#"), mockIComm_GetTransmittedData());
}

TEST(mockIComm, TransmitAndCapture2BytesWithOverflow)
{
    mockIComm_InitTransmitDataBuffer(2);

    IComm_SendChar(mockIComm_Get(), '-');
    IComm_SendChar(mockIComm_Get(), '+');
    IComm_SendChar(mockIComm_Get(), '*');

    STRCMP_EQUAL("-+", mockIComm_GetTransmittedData());
}

TEST(mockIComm, TransmitAndFailToCompareByData)
{
    mockIComm_InitTransmitDataBuffer(2);

    IComm_SendChar(mockIComm_Get(), '-');

    CHECK_FALSE( 0 == strcmp("+", mockIComm_GetTransmittedData()) );
}

TEST(mockIComm, ShouldStopRun_SetToReturnFalse)
{
    mockIComm_SetShouldStopRunFlag(0);
    CHECK_FALSE(IComm_ShouldStopRun(mockIComm_Get()));
}

TEST(mockIComm, ShouldStopRun_SetToReturnTrue)
{
    mockIComm_SetShouldStopRunFlag(1);
    CHECK_TRUE(IComm_ShouldStopRun(mockIComm_Get()));
}

TEST(mockIComm, IsGdbConnected_ShouldDefaultToReturnTrue)
{
    CHECK_TRUE(IComm_IsGdbConnected(mockIComm_Get()));
}

TEST(mockIComm, IsGdbConnected_SetToReturnFalse)
{
    mockIComm_SetIsGdbConnectedFlag(0);
    CHECK_FALSE(IComm_IsGdbConnected(mockIComm_Get()));
}

TEST(mockIComm, IsGdbConnected_SetToReturnTrue)
{
    mockIComm_SetIsGdbConnectedFlag(1);
    CHECK_TRUE(IComm_IsGdbConnected(mockIComm_Get()));
}
