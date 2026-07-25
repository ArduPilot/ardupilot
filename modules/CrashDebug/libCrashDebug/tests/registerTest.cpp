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
#include <CrashCatcher.h>
#include <signal.h>
#include "mriPlatformBaseTest.h"

TEST_GROUP_BASE(registerTests, mriPlatformBase)
{
    void setup()
    {
        mriPlatformBase::setup();
        // Set IPSR to DebugMonitor exception to generate SIGTRAP exception code.
        setIPSR(12);
    }

    void teardown()
    {
        mriPlatformBase::teardown();
    }
};


TEST(registerTests, ReadRegisters)
{
    for (uint32_t i = 0 ; i < 15 ; i++)
        m_context.R[i] = 0x11111111 * i;
    m_context.R[PC] = 0xFFFFFFFE;

    mockIComm_InitReceiveChecksummedData("+$g#", "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, 0xDDDDDDDD, 0xEEEEEEEE, 0xFFFFFFFE);
    appendExpectedString("+$00000000111111112222222233333333"
                           "44444444555555556666666677777777"
                           "8888888899999999aaaaaaaabbbbbbbb"
                           "ccccccccddddddddeeeeeeeefeffffff"
                           "00000001a5a5a5a55a5a5a5a#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(registerTests, ReadIntegerAndFloatRegisters)
{
    m_context.flags = CRASH_CATCHER_FLAGS_FLOATING_POINT;
    for (uint32_t i = 0 ; i < 15 ; i++)
        m_context.R[i] = 0x11111111 * i;
    m_context.R[PC] = 0xFFFFFFFE;
    for (int i = 0 ; i < 31 ; i++)
        m_context.FPR[i] = i;
    m_context.FPR[FPSCR] = 0xBAADF00D;

    mockIComm_InitReceiveChecksummedData("+$g#", "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xCCCCCCCC, 0xDDDDDDDD, 0xEEEEEEEE, 0xFFFFFFFE);
    appendExpectedString("+$00000000111111112222222233333333"
                           "44444444555555556666666677777777"
                           "8888888899999999aaaaaaaabbbbbbbb"
                           "ccccccccddddddddeeeeeeeefeffffff"
                           "00000001a5a5a5a55a5a5a5a"
                           "0000000001000000020000000300000004000000050000000600000007000000"
                           "08000000090000000a0000000b0000000c0000000d0000000e0000000f000000"
                           "1000000011000000120000001300000014000000150000001600000017000000"
                           "18000000190000001a0000001b0000001c0000001d0000001e00000000000000"
                           "0df0adba#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());
}

TEST(registerTests, WriteRegisters)
{
    memset(m_context.R, 0xa5, 16 * sizeof(uint32_t));
    mockIComm_InitReceiveChecksummedData("+$G00000000111111112222222233333333"
                                            "44444444555555556666666677777777"
                                            "8888888899999999aaaaaaaabbbbbbbb"
                                            "ccccccccddddddddeeeeeeeefeffffff"
                                            "ffffffff4545454554545454#", "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xA5A5A5A5, 0xA5A5A5A5, 0xA5A5A5A5, 0xA5A5A5A5);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());

    for (int i = 0 ; i < 15 ; i++)
        CHECK_EQUAL(0x11111111U * i, m_context.R[i]);
    CHECK_EQUAL(0xFFFFFFFEU, m_context.R[PC]);
    CHECK_EQUAL(0xFFFFFFFFU, m_context.R[XPSR]);
    CHECK_EQUAL(0x45454545U, m_context.R[MSP]);
    CHECK_EQUAL(0x54545454U, m_context.R[PSP]);
}

TEST(registerTests, WriteIntegerAndFloatRegisters)
{
    memset(m_context.R, 0xa5, 16 * sizeof(uint32_t));
    m_context.flags = CRASH_CATCHER_FLAGS_FLOATING_POINT;
    mockIComm_InitReceiveChecksummedData("+$G00000000111111112222222233333333"
                                            "44444444555555556666666677777777"
                                            "8888888899999999aaaaaaaabbbbbbbb"
                                            "ccccccccddddddddeeeeeeeefeffffff"
                                            "ffffffff4545454554545454"
                                            "0000000001000000020000000300000004000000050000000600000007000000"
                                            "08000000090000000a0000000b0000000c0000000d0000000e0000000f000000"
                                            "1000000011000000120000001300000014000000150000001600000017000000"
                                            "18000000190000001a0000001b0000001c0000001d0000001e00000000000000"
                                            "0df0adba#", "+$c#");
        mriPlatform_Run(mockIComm_Get());
    appendExpectedTPacket(SIGTRAP, 0xA5A5A5A5, 0xA5A5A5A5, 0xA5A5A5A5, 0xA5A5A5A5);
    appendExpectedString("+$OK#+");
    STRCMP_EQUAL(checksumExpected(), mockIComm_GetTransmittedData());

    for (int i = 0 ; i < 15 ; i++)
        CHECK_EQUAL(0x11111111U * i, m_context.R[i]);
    CHECK_EQUAL(0xFFFFFFFEU, m_context.R[PC]);
    CHECK_EQUAL(0xFFFFFFFFU, m_context.R[XPSR]);
    CHECK_EQUAL(0x45454545U, m_context.R[MSP]);
    CHECK_EQUAL(0x54545454U, m_context.R[PSP]);
    for (uint32_t i = 0 ; i < 15 ; i++)
        CHECK_EQUAL(i, m_context.FPR[i]);
    CHECK_EQUAL(0xBAADF00D, m_context.FPR[FPSCR]);
}
