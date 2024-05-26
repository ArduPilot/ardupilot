/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <limits>
#include <uavcan/marshal/scalar_codec.hpp>
#include <uavcan/transport/transfer_buffer.hpp>


TEST(ScalarCodec, Basic)
{
    uavcan::StaticTransferBuffer<38> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    {
        uint64_t u64 = 0;
        ASSERT_EQ(0, sc_wr.decode<64>(u64));    // Out of buffer space
    }

    /*
     * Encoding some variables
     */
    ASSERT_EQ(1, sc_wr.encode<12>(uint16_t(0xbeda)));    // --> 0xeda
    ASSERT_EQ(1, sc_wr.encode<1>(uint8_t(1)));
    ASSERT_EQ(1, sc_wr.encode<1>(uint16_t(0)));
    ASSERT_EQ(1, sc_wr.encode<4>(uint8_t(8)));
    ASSERT_EQ(1, sc_wr.encode<32>(uint32_t(0xdeadbeef)));
    ASSERT_EQ(1, sc_wr.encode<3>(int8_t(-1)));
    ASSERT_EQ(1, sc_wr.encode<4>(int8_t(-6)));
    ASSERT_EQ(1, sc_wr.encode<20>(int32_t(-123456)));
    ASSERT_EQ(1, sc_wr.encode<64>(std::numeric_limits<int64_t>::min()));
    ASSERT_EQ(1, sc_wr.encode<64>(std::numeric_limits<int64_t>::max()));
    ASSERT_EQ(1, sc_wr.encode<15>(int16_t(-1)));
    ASSERT_EQ(1, sc_wr.encode<2>(int16_t(-1)));
    ASSERT_EQ(1, sc_wr.encode<16>(std::numeric_limits<int16_t>::min()));
    ASSERT_EQ(1, sc_wr.encode<64>(std::numeric_limits<uint64_t>::max())); // Total 302 bit (38 bytes)

    ASSERT_EQ(0, sc_wr.encode<64>(std::numeric_limits<uint64_t>::min())); // Out of buffer space

    /*
     * Decoding back
     */
    uavcan::BitStream bs_rd(buf);
    uavcan::ScalarCodec sc_rd(bs_rd);

    uint8_t u8   = 0;
    int8_t i8    = 0;
    uint16_t u16 = 0;
    int16_t i16  = 0;
    uint32_t u32 = 0;
    int32_t i32  = 0;
    uint64_t u64 = 0;
    int64_t i64  = 0;

#define CHECK(bitlen, variable, expected_value) \
    do { \
        ASSERT_EQ(1, sc_rd.decode<bitlen>(variable)); \
        ASSERT_EQ(expected_value, variable); \
    } while (0)

    CHECK(12, u16, 0xeda);
    CHECK(1,  u8,  1);
    CHECK(1,  u16, 0);
    CHECK(4,  u8,  8);
    CHECK(32, u32, 0xdeadbeef);
    CHECK(3,  i8,  -1);
    CHECK(4,  i8,  -6);
    CHECK(20, i32, -123456);
    CHECK(64, i64, std::numeric_limits<int64_t>::min());
    CHECK(64, i64, std::numeric_limits<int64_t>::max());
    CHECK(15, i16, -1);
    CHECK(2,  i8,  -1);
    CHECK(16, i16, std::numeric_limits<int16_t>::min());
    CHECK(64, u64, std::numeric_limits<uint64_t>::max());

#undef CHECK

    ASSERT_EQ(0, sc_rd.decode<64>(u64)); // Out of buffer space
}

TEST(ScalarCodec, RepresentationCorrectness)
{
    uavcan::StaticTransferBuffer<4> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    ASSERT_EQ(1, sc_wr.encode<12>(uint16_t(0xbeda)));    // --> 0xeda
    ASSERT_EQ(1, sc_wr.encode<3>(int8_t(-1)));
    ASSERT_EQ(1, sc_wr.encode<4>(int8_t(-5)));
    ASSERT_EQ(1, sc_wr.encode<2>(int16_t(-1)));
    ASSERT_EQ(1, sc_wr.encode<4>(uint8_t(0x88)));        // --> 8

    // This representation was carefully crafted and triple checked:
    static const std::string REFERENCE = "11011010 11101111 01111100 00000000";
    ASSERT_EQ(REFERENCE, bs_wr.toString());
}
