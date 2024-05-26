/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/marshal/types.hpp>
#include <uavcan/transport/transfer_buffer.hpp>


TEST(IntegerSpec, Limits)
{
    using uavcan::IntegerSpec;
    using uavcan::SignednessSigned;
    using uavcan::SignednessUnsigned;
    using uavcan::CastModeSaturate;
    using uavcan::CastModeTruncate;

    typedef IntegerSpec<8,  SignednessUnsigned, CastModeSaturate> UInt8;
    typedef IntegerSpec<4,    SignednessSigned, CastModeTruncate> SInt4;
    typedef IntegerSpec<32, SignednessUnsigned, CastModeTruncate> UInt32;
    typedef IntegerSpec<40, SignednessUnsigned, CastModeSaturate> UInt40;
    typedef IntegerSpec<64, SignednessUnsigned, CastModeTruncate> UInt64;
    typedef IntegerSpec<64,   SignednessSigned, CastModeSaturate> SInt64;
    typedef IntegerSpec<63, SignednessUnsigned, CastModeSaturate> UInt63;

    ASSERT_EQ(255, UInt8::max());
    ASSERT_EQ(0, UInt8::min());

    ASSERT_EQ(7, SInt4::max());
    ASSERT_EQ(-8, SInt4::min());

    ASSERT_EQ(0xFFFFFFFF, UInt32::max());
    ASSERT_EQ(0, UInt32::min());

    ASSERT_EQ(0xFFFFFFFFFF, UInt40::max());
    ASSERT_EQ(0, UInt40::min());

    ASSERT_EQ(0xFFFFFFFFFFFFFFFF, UInt64::max());
    ASSERT_EQ(0, UInt64::min());

    ASSERT_EQ(0x7FFFFFFFFFFFFFFF, SInt64::max());
    ASSERT_EQ(-0x8000000000000000, SInt64::min());

    ASSERT_EQ(0x7FFFFFFFFFFFFFFF, UInt63::max());
    ASSERT_EQ(0, UInt63::min());

    ASSERT_EQ(SInt64::max(), UInt63::max());
}


TEST(IntegerSpec, Basic)
{
    using uavcan::IntegerSpec;
    using uavcan::SignednessSigned;
    using uavcan::SignednessUnsigned;
    using uavcan::CastModeSaturate;
    using uavcan::CastModeTruncate;
    using uavcan::StorageType;

    uavcan::StaticTransferBuffer<100> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    typedef IntegerSpec<8,  SignednessUnsigned, CastModeSaturate> UInt8S;
    typedef IntegerSpec<4,    SignednessSigned, CastModeTruncate> SInt4T;
    typedef IntegerSpec<32, SignednessUnsigned, CastModeTruncate> UInt32T;
    typedef IntegerSpec<40, SignednessUnsigned, CastModeSaturate> UInt40S;
    typedef IntegerSpec<64, SignednessUnsigned, CastModeTruncate> UInt64T;
    typedef IntegerSpec<58,   SignednessSigned, CastModeSaturate> SInt58S;
    typedef IntegerSpec<63, SignednessUnsigned, CastModeSaturate> UInt63S;
    typedef IntegerSpec<10,   SignednessSigned, CastModeSaturate> SInt10S;
    typedef IntegerSpec<1,  SignednessUnsigned, CastModeSaturate> UInt1S;

    ASSERT_EQ(1, UInt8S::encode(UInt8S::StorageType(123), sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, SInt4T::encode(SInt4T::StorageType(-0x44), sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, UInt32T::encode(UInt32T::StorageType(0xFFFFFFFF), sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, UInt40S::encode(UInt40S::StorageType(0xFFFFFFFFFFFFFFFF), sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, UInt64T::encode(UInt64T::StorageType(0xFFFFFFFFFFFFFFFF), sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, SInt58S::encode(SInt58S::StorageType(0xFFFFFFFFFFFFFFF), sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, UInt63S::encode(UInt63S::StorageType(0xFFFFFFFFFFFFFFFF), sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, SInt10S::encode(SInt10S::StorageType(-30000), sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, UInt1S::encode(UInt1S::StorageType(42), sc_wr, uavcan::TailArrayOptDisabled));

    std::cout << bs_wr.toString() << std::endl;

    uavcan::BitStream bs_rd(buf);
    uavcan::ScalarCodec sc_rd(bs_rd);

#define CHECK(IntType, expected_value) \
    do { \
        StorageType<IntType>::Type var = StorageType<IntType>::Type(); \
        ASSERT_EQ(1, IntType::decode(var, sc_rd, uavcan::TailArrayOptDisabled)); \
        ASSERT_EQ(expected_value, var); \
    } while (0)

    CHECK(UInt8S,  123);
    CHECK(SInt4T,  -4);
    CHECK(UInt32T, 0xFFFFFFFF);
    CHECK(UInt40S, 0xFFFFFFFFFF);
    CHECK(UInt64T, 0xFFFFFFFFFFFFFFFF);
    CHECK(SInt58S, 0x1FFFFFFFFFFFFFF);
    CHECK(UInt63S, 0x7FFFFFFFFFFFFFFF);
    CHECK(SInt10S, -512);
    CHECK(UInt1S,  1);

#undef CHECK

    StorageType<UInt1S>::Type var;
    ASSERT_EQ(0, UInt1S::decode(var, sc_rd, uavcan::TailArrayOptDisabled));
}
