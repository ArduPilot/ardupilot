/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/marshal/types.hpp>


TEST(MarshalTypeUtil, IntegerBitLen)
{
    using uavcan::IntegerBitLen;

    ASSERT_EQ(0, IntegerBitLen<0>::Result);
    ASSERT_EQ(1, IntegerBitLen<1>::Result);
    ASSERT_EQ(6, IntegerBitLen<42>::Result);
    ASSERT_EQ(8, IntegerBitLen<232>::Result);
    ASSERT_EQ(32, IntegerBitLen<0x81234567>::Result);
}


TEST(MarshalTypeUtil, BitLenToByteLen)
{
    using uavcan::BitLenToByteLen;

    ASSERT_EQ(2, BitLenToByteLen<16>::Result);
    ASSERT_EQ(1, BitLenToByteLen<8>::Result);
    ASSERT_EQ(1, BitLenToByteLen<7>::Result);
    ASSERT_EQ(1, BitLenToByteLen<1>::Result);
    ASSERT_EQ(2, BitLenToByteLen<9>::Result);
}
