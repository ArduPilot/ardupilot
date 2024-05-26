/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#if __GNUC__
# pragma GCC diagnostic ignored "-Wfloat-equal"
# pragma GCC diagnostic ignored "-Wdouble-promotion"
#endif

#include <gtest/gtest.h>
#include <uavcan/marshal/types.hpp>
#include <uavcan/transport/transfer_buffer.hpp>

using uavcan::Array;
using uavcan::ArrayModeDynamic;
using uavcan::ArrayModeStatic;
using uavcan::IntegerSpec;
using uavcan::FloatSpec;
using uavcan::SignednessSigned;
using uavcan::SignednessUnsigned;
using uavcan::CastModeSaturate;
using uavcan::CastModeTruncate;

struct CustomType
{
    typedef uavcan::IntegerSpec<8, uavcan::SignednessSigned, uavcan::CastModeTruncate> A;
    typedef uavcan::FloatSpec<16, uavcan::CastModeSaturate> B;
    // Dynamic array of max len 5 --> 3 bits for len, 5 bits for data --> 1 byte max len
    typedef uavcan::Array<uavcan::IntegerSpec<1, uavcan::SignednessUnsigned, uavcan::CastModeSaturate>,
                          uavcan::ArrayModeDynamic, 5> C;

    enum { MinBitLen = A::MinBitLen + B::MinBitLen + C::MinBitLen };
    enum { MaxBitLen = A::MaxBitLen + B::MaxBitLen + C::MaxBitLen };

    typename uavcan::StorageType<A>::Type a;
    typename uavcan::StorageType<B>::Type b;
    typename uavcan::StorageType<C>::Type c;

    CustomType()
        : a()
        , b()
        , c()
    { }

    bool operator==(const CustomType& rhs) const
    {
        return a == rhs.a &&
               uavcan::areFloatsExactlyEqual(b, rhs.b) &&
               c == rhs.c;
    }

    static int encode(const CustomType& obj, uavcan::ScalarCodec& codec,
                      uavcan::TailArrayOptimizationMode tao_mode = uavcan::TailArrayOptEnabled)
    {
        int res = 0;
        res = A::encode(obj.a, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
        {
            return res;
        }
        res = B::encode(obj.b, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
        {
            return res;
        }
        res = C::encode(obj.c, codec, tao_mode);
        if (res <= 0)
        {
            return res;
        }
        return 1;
    }

    static int decode(CustomType& obj, uavcan::ScalarCodec& codec,
                      uavcan::TailArrayOptimizationMode tao_mode = uavcan::TailArrayOptEnabled)
    {
        int res = 0;
        res = A::decode(obj.a, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
        {
            return res;
        }
        res = B::decode(obj.b, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
        {
            return res;
        }
        res = C::decode(obj.c, codec, tao_mode);
        if (res <= 0)
        {
            return res;
        }
        return 1;
    }
};


TEST(Array, Basic)
{
    typedef Array<IntegerSpec<8, SignednessSigned, CastModeTruncate>, ArrayModeStatic, 4> A1;
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeStatic, 2> A2;
    typedef Array<CustomType, ArrayModeStatic, 2> A3;

    A1 a1;
    A2 a2;
    A3 a3;

    ASSERT_EQ(1, A3::ValueType::C::RawValueType::BitLen);

    ASSERT_EQ(8 * 4, A1::MaxBitLen);
    ASSERT_EQ(16 * 2, A2::MaxBitLen);
    ASSERT_EQ((8 + 16 + 5 + 3) * 2, A3::MaxBitLen);

    /*
     * Zero initialization check
     */
    ASSERT_FALSE(a1.empty());
    for (A1::const_iterator it = a1.begin(); it != a1.end(); ++it)
    {
        ASSERT_EQ(0, *it);
    }

    ASSERT_FALSE(a2.empty());
    for (A2::const_iterator it = a2.begin(); it != a2.end(); ++it)
    {
        ASSERT_EQ(0, *it);
    }

    for (A3::const_iterator it = a3.begin(); it != a3.end(); ++it)
    {
        ASSERT_EQ(0, it->a);
        ASSERT_EQ(0, it->b);
        ASSERT_EQ(0, it->c.size());
        ASSERT_TRUE(it->c.empty());
    }

    /*
     * Modification with known values; array lengths are hard coded.
     */
    for (uint8_t i = 0; i < 4; i++)
    {
        a1.at(i) = int8_t(i);
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        a2.at(i) = i;
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        a3[i].a = int8_t(i);
        a3[i].b = i;
        for (uint8_t i2 = 0; i2 < 5; i2++)
        {
            a3[i].c.push_back(i2 & 1);
        }
        ASSERT_EQ(5, a3[i].c.size());
        ASSERT_FALSE(a3[i].c.empty());
    }

    /*
     * Representation check
     * Note that TAO in A3 is not possible because A3::C has less than one byte per item
     */
    uavcan::StaticTransferBuffer<16> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    ASSERT_EQ(1, A1::encode(a1, sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, A2::encode(a2, sc_wr, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, A3::encode(a3, sc_wr, uavcan::TailArrayOptEnabled));

    ASSERT_EQ(0, A3::encode(a3, sc_wr, uavcan::TailArrayOptEnabled));  // Out of buffer space

    static const std::string Reference =
        "00000000 00000001 00000010 00000011 " // A1 (0, 1, 2, 3)
        "00000000 00000000 00000000 00111100 " // A2 (0, 1)
        "00000000 00000000 00000000 10101010 " // A3[0] (0, 0, bool[5])
        "00000001 00000000 00111100 10101010"; // A3[1] (1, 1, bool[5])

    ASSERT_EQ(Reference, bs_wr.toString());

    /*
     * Read back
     */
    uavcan::BitStream bs_rd(buf);
    uavcan::ScalarCodec sc_rd(bs_rd);

    A1 a1_;
    A2 a2_;
    A3 a3_;

    ASSERT_EQ(1, A1::decode(a1_, sc_rd, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, A2::decode(a2_, sc_rd, uavcan::TailArrayOptDisabled));
    ASSERT_EQ(1, A3::decode(a3_, sc_rd, uavcan::TailArrayOptEnabled));

    ASSERT_EQ(a1_, a1);
    ASSERT_EQ(a2_, a2);
    ASSERT_EQ(a3_, a3);

    for (uint8_t i = 0; i < 4; i++)
    {
        ASSERT_EQ(a1[i], a1_[i]);
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        ASSERT_EQ(a2[i], a2_[i]);
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        ASSERT_EQ(a3[i].a, a3_[i].a);
        ASSERT_EQ(a3[i].b, a3_[i].b);
        ASSERT_EQ(a3[i].c, a3_[i].c);
    }

    ASSERT_EQ(0, A3::decode(a3_, sc_rd, uavcan::TailArrayOptEnabled));  // Out of buffer space

    /*
     * STL compatibility
     */
    std::vector<int> v1;
    v1.push_back(0);
    v1.push_back(1);
    v1.push_back(2);
    v1.push_back(3);

    ASSERT_TRUE(a1 == v1);
    ASSERT_FALSE(a1 != v1);
    ASSERT_TRUE(v1 == a1);
    ASSERT_FALSE(v1 != a1);
    ASSERT_FALSE(a1 < v1);

    v1[0] = 9000;
    ASSERT_FALSE(a1 == v1);
    ASSERT_TRUE(a1 != v1);
    ASSERT_TRUE(a1 < v1);

    ASSERT_EQ(0, a1.front());
    ASSERT_EQ(3, a1.back());

    // Boolean vector
    std::vector<bool> v2;
    v2.push_back(false);
    v2.push_back(true);
    v2.push_back(false);
    v2.push_back(true);
    v2.push_back(false);

    ASSERT_TRUE(a3[0].c == v2);
    ASSERT_FALSE(a3[0].c == v1);
    ASSERT_FALSE(a3[0].c != v2);
    ASSERT_TRUE(a3[0].c != v1);

    v2[0] = true;
    ASSERT_TRUE(a3[0].c != v2);
    ASSERT_FALSE(a3[0].c == v2);
}


TEST(Array, Dynamic)
{
    typedef Array<IntegerSpec<1, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 5> A;
    typedef Array<IntegerSpec<8, SignednessSigned, CastModeSaturate>, ArrayModeDynamic, 255> B;

    A a;
    B b;
    B b2;

    ASSERT_EQ(3 + 5, A::MaxBitLen);
    ASSERT_EQ(8 + 255 * 8, B::MaxBitLen);

    ASSERT_TRUE(a.empty());
    ASSERT_TRUE(b.empty());
    ASSERT_TRUE(b2.empty());

    {
        uavcan::StaticTransferBuffer<16> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);

        ASSERT_EQ(1, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::encode(b, sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::encode(b2, sc_wr, uavcan::TailArrayOptEnabled));

        ASSERT_EQ("000" "00000 000" "00000", bs_wr.toString()); // Last array was optimized away completely

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);

        ASSERT_EQ(1, A::decode(a, sc_rd, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::decode(b, sc_rd, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::decode(b2, sc_rd, uavcan::TailArrayOptEnabled));

        ASSERT_TRUE(a.empty());
        ASSERT_TRUE(b.empty());
        ASSERT_TRUE(b2.empty());
    }

    a.push_back(true);
    a.push_back(false);
    a.push_back(true);
    a.push_back(false);
    a.push_back(true);

    b.push_back(42);
    b.push_back(-42);

    b2.push_back(123);
    b2.push_back(72);

    {
        uavcan::StaticTransferBuffer<16> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);

        ASSERT_EQ(1, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::encode(b, sc_wr, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::encode(b2, sc_wr, uavcan::TailArrayOptEnabled));  // No length field

        //         A        B len    B[0]     B[1]     B2[0]    B2[1]
        ASSERT_EQ("10110101 00000010 00101010 11010110 01111011 01001000", bs_wr.toString());

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);

        a.clear();
        b.clear();
        b2.clear();
        ASSERT_TRUE(a.empty());
        ASSERT_TRUE(b.empty());
        ASSERT_TRUE(b2.empty());

        ASSERT_EQ(1, A::decode(a, sc_rd, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::decode(b, sc_rd, uavcan::TailArrayOptDisabled));
        ASSERT_EQ(1, B::decode(b2, sc_rd, uavcan::TailArrayOptEnabled));

        ASSERT_EQ(5, a.size());
        ASSERT_EQ(2, b.size());
        ASSERT_EQ(2, b2.size());

        ASSERT_TRUE(a[0]);
        ASSERT_FALSE(a[1]);
        ASSERT_TRUE(a[2]);
        ASSERT_FALSE(a[3]);
        ASSERT_TRUE(a[4]);

        ASSERT_EQ(42, b[0]);
        ASSERT_EQ(-42, b[1]);

        ASSERT_EQ(123, b2[0]);
        ASSERT_EQ(72, b2[1]);
    }

    ASSERT_FALSE(a == b);
    ASSERT_FALSE(b == a);
    ASSERT_TRUE(a != b);
    ASSERT_TRUE(b != a);

    a.resize(0);
    b.resize(0);
    ASSERT_TRUE(a.empty());
    ASSERT_TRUE(b.empty());

    a.resize(5, true);
    b.resize(255, 72);
    ASSERT_EQ(5, a.size());
    ASSERT_EQ(255, b.size());

    for (uint8_t i = 0; i < 5; i++)
    {
        ASSERT_TRUE(a[i]);
    }
    for (uint8_t i = 0; i < 255; i++)
    {
        ASSERT_EQ(72, b[i]);
    }
}


template <typename B>
struct CustomType2
{
    typedef uavcan::FloatSpec<16, uavcan::CastModeSaturate> A;

    enum { MinBitLen = A::MinBitLen + B::MinBitLen };
    enum { MaxBitLen = A::MaxBitLen + B::MaxBitLen };

    typename uavcan::StorageType<A>::Type a;
    typename uavcan::StorageType<B>::Type b;

    CustomType2()
        : a()
        , b()
    { }

    bool operator==(const CustomType2& rhs) const
    {
        return uavcan::areFloatsExactlyEqual(a, rhs.a) &&
               b == rhs.b;
    }

    static int encode(const CustomType2& obj, uavcan::ScalarCodec& codec,
                      uavcan::TailArrayOptimizationMode tao_mode = uavcan::TailArrayOptEnabled)
    {
        int res = 0;
        res = A::encode(obj.a, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
        {
            return res;
        }
        res = B::encode(obj.b, codec, tao_mode);
        if (res <= 0)
        {
            return res;
        }
        return 1;
    }

    static int decode(CustomType2& obj, uavcan::ScalarCodec& codec,
                      uavcan::TailArrayOptimizationMode tao_mode = uavcan::TailArrayOptEnabled)
    {
        int res = 0;
        res = A::decode(obj.a, codec, uavcan::TailArrayOptDisabled);
        if (res <= 0)
        {
            return res;
        }
        res = B::decode(obj.b, codec, tao_mode);
        if (res <= 0)
        {
            return res;
        }
        return 1;
    }
};


template <typename T>
static std::string runEncodeDecode(const typename uavcan::StorageType<T>::Type& value,
                                   const uavcan::TailArrayOptimizationMode tao_mode)
{
    uavcan::StaticTransferBuffer<(T::MaxBitLen + 7) / 8> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);
    EXPECT_EQ(1, T::encode(value, sc_wr, tao_mode));

    typename uavcan::StorageType<T>::Type value2 = typename uavcan::StorageType<T>::Type();
    // Decode multiple times to make sure that the decoded type is being correctly de-initialized
    for (int i = 0; i < 3; i++)
    {
        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        EXPECT_EQ(1, T::decode(value2, sc_rd, tao_mode));
        EXPECT_EQ(value, value2);
    }
    return bs_wr.toString();
}


TEST(Array, TailArrayOptimization)
{
    typedef Array<IntegerSpec<1, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 5>   OneBitArray;
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 255> EightBitArray;
    typedef CustomType2<Array<OneBitArray,   ArrayModeDynamic, 255> > A;
    typedef CustomType2<Array<EightBitArray, ArrayModeDynamic, 255> > B;
    typedef CustomType2<EightBitArray> C;

    A a;
    B b;
    C c;

    /*
     * Empty
     */
    //         a LSB    a MSB    b len
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<A>(a, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<A>(a, uavcan::TailArrayOptDisabled));

    //         a LSB    a MSB    b len
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<B>(b, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<B>(b, uavcan::TailArrayOptDisabled));

    //         a LSB    a MSB
    ASSERT_EQ("00000000 00000000",          runEncodeDecode<C>(c, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000 00000000 00000000", runEncodeDecode<C>(c, uavcan::TailArrayOptDisabled));

    /*
     * A
     */
    a.b.resize(2);
    a.b[0].push_back(true);
    a.b[0].push_back(false);
    // a.b[1] remains empty
    //         a LSB    a MSB    b len    b: len(2), 1, 0, len(0)
    ASSERT_EQ("00000000 00000000 00000010 01010000", runEncodeDecode<A>(a, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000 00000000 00000010 01010000", runEncodeDecode<A>(a, uavcan::TailArrayOptDisabled));

    /*
     * B
     */
    b.b.resize(3);
    b.b[0].push_back(42);
    b.b[0].push_back(72);
    // b.b[1] remains empty
    b.b[2].push_back(123);
    b.b[2].push_back(99);
    //         a LSB    a MSB    b len    b[0]len  42       72       b[1]len  123      99      (b[2] len optimized out)
    ASSERT_EQ("00000000 00000000 00000011 00000010 00101010 01001000 00000000 01111011 01100011",
              runEncodeDecode<B>(b, uavcan::TailArrayOptEnabled));
    // Same as above, but b[2] len is present                                 v here v
    ASSERT_EQ("00000000 00000000 00000011 00000010 00101010 01001000 00000000 00000010 01111011 01100011",
              runEncodeDecode<B>(b, uavcan::TailArrayOptDisabled));

    /*
     * C
     */
    c.a = 1;
    c.b.push_back(1);
    c.b.push_back(2);
    c.b.push_back(3);
    //         a LSB    a MSB    1        2        3
    ASSERT_EQ("00000000 00111100 00000001 00000010 00000011",
              runEncodeDecode<C>(c, uavcan::TailArrayOptEnabled));
    //         a LSB    a MSB    b len    1        2        3
    ASSERT_EQ("00000000 00111100 00000011 00000001 00000010 00000011",
              runEncodeDecode<C>(c, uavcan::TailArrayOptDisabled));
}


TEST(Array, TailArrayOptimizationErrors)
{
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 5> A;

    A a;
    ASSERT_TRUE(a.empty());
    ASSERT_EQ("",         runEncodeDecode<A>(a, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("00000000", runEncodeDecode<A>(a, uavcan::TailArrayOptDisabled));

    // Correct decode/encode
    a.push_back(1);
    a.push_back(126);
    a.push_back(5);
    ASSERT_FALSE(a.empty());
    ASSERT_EQ("00000001 01111110 00000101",          runEncodeDecode<A>(a, uavcan::TailArrayOptEnabled));
    ASSERT_EQ("01100000 00101111 11000000 10100000", runEncodeDecode<A>(a, uavcan::TailArrayOptDisabled));

    // Invalid decode - length field is out of range
    uavcan::StaticTransferBuffer<7> buf;
    uavcan::BitStream bs_wr(buf);
    uavcan::ScalarCodec sc_wr(bs_wr);

    ASSERT_EQ(1, sc_wr.encode<3>(uint8_t(6)));  // Length - more than 5 items, error
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(42)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(72)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(126)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(1)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(2)));
    ASSERT_EQ(1, sc_wr.encode<8>(uint8_t(3)));  // Out of range - only 5 items allowed

    //         197      73       15       192      32       ...
    ASSERT_EQ("11000101 01001001 00001111 11000000 00100000 01000000 01100000", bs_wr.toString());

    {
        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        A a2;
        a2.push_back(56);   // Garbage
        ASSERT_EQ(1, a2.size());
        // Will fail - declared length is more than 5 items
        ASSERT_GT(0, A::decode(a2, sc_rd, uavcan::TailArrayOptDisabled));
        // Must be cleared
        ASSERT_TRUE(a2.empty());
    }
    {
        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        A a2;
        a2.push_back(56);   // Garbage
        ASSERT_EQ(1, a2.size());
        // Will fail - no length field, but the stream is too long
        ASSERT_GT(0, A::decode(a2, sc_rd, uavcan::TailArrayOptEnabled));
        // Will contain some garbage
        ASSERT_EQ(5, a2.size());
        // Interpreted stream - see the values above
        ASSERT_EQ(197, a2[0]);
        ASSERT_EQ(73,  a2[1]);
        ASSERT_EQ(15,  a2[2]);
        ASSERT_EQ(192, a2[3]);
        ASSERT_EQ(32,  a2[4]);
    }
}


TEST(Array, DynamicEncodeDecodeErrors)
{
    typedef CustomType2<Array<Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>,
                                    ArrayModeDynamic, 255>,
                              ArrayModeDynamic, 255> > A;
    A a;
    a.b.resize(2);
    a.b[0].push_back(55);
    a.b[0].push_back(66);
    {
        uavcan::StaticTransferBuffer<4> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);
        ASSERT_EQ(0, A::encode(a, sc_wr, uavcan::TailArrayOptEnabled));  // Not enough buffer space

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        ASSERT_EQ(0, A::decode(a, sc_rd, uavcan::TailArrayOptEnabled));
    }
    {
        uavcan::StaticTransferBuffer<4> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);
        ASSERT_EQ(0, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));  // Not enough buffer space

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        ASSERT_EQ(0, A::decode(a, sc_rd, uavcan::TailArrayOptDisabled));
    }
}


TEST(Array, StaticEncodeDecodeErrors)
{
    typedef CustomType2<Array<Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>,
                                    ArrayModeStatic, 2>,
                              ArrayModeStatic, 2> > A;
    A a;
    a.a = 1.0;
    a.b[0][0] = 0x11;
    a.b[0][1] = 0x22;
    a.b[1][0] = 0x33;
    a.b[1][1] = 0x44;
    {   // Just enough buffer space - 6 bytes
        uavcan::StaticTransferBuffer<6> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);
        ASSERT_EQ(1, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));

        ASSERT_EQ("00000000 00111100 00010001 00100010 00110011 01000100", bs_wr.toString());

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        ASSERT_EQ(1, A::decode(a, sc_rd, uavcan::TailArrayOptEnabled));
    }
    {   // Not enough space
        uavcan::StaticTransferBuffer<5> buf;
        uavcan::BitStream bs_wr(buf);
        uavcan::ScalarCodec sc_wr(bs_wr);
        ASSERT_EQ(0, A::encode(a, sc_wr, uavcan::TailArrayOptDisabled));

        ASSERT_EQ("00000000 00111100 00010001 00100010 00110011", bs_wr.toString());

        uavcan::BitStream bs_rd(buf);
        uavcan::ScalarCodec sc_rd(bs_rd);
        ASSERT_EQ(0, A::decode(a, sc_rd, uavcan::TailArrayOptEnabled));
    }
}


TEST(Array, Copyability)
{
    typedef Array<IntegerSpec<1, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 5>   OneBitArray;
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 255> EightBitArray;
    typedef Array<OneBitArray,   ArrayModeDynamic, 255> A;
    typedef Array<EightBitArray, ArrayModeDynamic, 255> B;
    typedef EightBitArray C;

    A a;
    B b;
    C c;

    A a2 = a;
    B b2 = b;
    C c2 = c;

    ASSERT_TRUE(a == a2);
    ASSERT_TRUE(b == b2);
    ASSERT_TRUE(c == c2);

    a.push_back(OneBitArray());
    b.push_back(EightBitArray());
    c.push_back(42);

    ASSERT_TRUE(a != a2);
    ASSERT_TRUE(b != b2);
    ASSERT_TRUE(c != c2);

    a2 = a;
    b2 = b;
    c2 = c;

    ASSERT_TRUE(a2 == a);
    ASSERT_TRUE(b2 == b);
    ASSERT_TRUE(c2 == c);
}


TEST(Array, Appending)
{
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeDynamic, 2> A;
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeDynamic, 257> B;
    A a;
    B b;

    a.push_back(1);
    a.push_back(2);
    a += b;
    ASSERT_EQ(2, a.size());
    ASSERT_EQ(1, a[0]);
    ASSERT_EQ(2, a[1]);

    b += a;
    ASSERT_TRUE(b == a);
    b += a;
    ASSERT_EQ(4, b.size());
    ASSERT_EQ(1, b[0]);
    ASSERT_EQ(2, b[1]);
    ASSERT_EQ(1, b[2]);
    ASSERT_EQ(2, b[3]);
}


TEST(Array, Strings)
{
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 32> A8;
    typedef Array<IntegerSpec<7, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 32> A7;

    A8 a8;
    A8 a8_2;
    A7 a7;

    ASSERT_TRUE(a8 == a7);
    // cppcheck-suppress duplicateExpression
    ASSERT_TRUE(a8 == a8);
    // cppcheck-suppress duplicateExpression
    ASSERT_TRUE(a7 == a7);
    ASSERT_TRUE(a8 == "");
    ASSERT_TRUE(a7 == "");

    a8 = "Hello world!";
    a7 = "123";
    ASSERT_TRUE(a8 == "Hello world!");
    ASSERT_TRUE(a7 == "123");

    a8 = "Our sun is dying.";
    a7 = "456";
    ASSERT_TRUE("Our sun is dying." == a8);
    ASSERT_TRUE("456" == a7);

    a8 += " 123456";
    a8 += "-789";
    ASSERT_TRUE("Our sun is dying. 123456-789" == a8);

    ASSERT_TRUE(a8_2 == "");
    ASSERT_TRUE(a8_2.empty());
    ASSERT_TRUE(a8_2 != a8);
    a8_2 = a8;
    ASSERT_TRUE(a8_2 == "Our sun is dying. 123456-789");
    ASSERT_TRUE(a8_2 == a8);

    /*
     * c_str()
     */
    ASSERT_STREQ("", A8().c_str());
    ASSERT_STREQ("", A7().c_str());
    ASSERT_STREQ("Our sun is dying. 123456-789", a8_2.c_str());
    ASSERT_STREQ("Our sun is dying. 123456-789", a8.c_str());
    ASSERT_STREQ("456", a7.c_str());

    /*
     * String constructor
     */
    A8 a8_3("123");
    A7 a7_3 = "456";
    ASSERT_EQ(3, a8_3.size());
    ASSERT_EQ(3, a7_3.size());
    ASSERT_STREQ("123", a8_3.c_str());
    ASSERT_STREQ("456", a7_3.c_str());
}


TEST(Array, AppendFormatted)
{
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 45> A8;

    A8 a;

    ASSERT_TRUE("" == a);

    a.appendFormatted("%4.1f", 12.3);                      // 4
    a += " ";                                             // 1
    a.appendFormatted("%li", -123456789L);                 // 10
    a.appendFormatted("%s", " TOTAL PERSPECTIVE VORTEX "); // 26
    a.appendFormatted("0x%X", 0xDEADBEEF);                 // 10 --> 4

    ASSERT_STREQ("12.3 -123456789 TOTAL PERSPECTIVE VORTEX 0xDE", a.c_str());
}


TEST(Array, FlatStreaming)
{
    typedef Array<IntegerSpec<8, SignednessUnsigned, CastModeSaturate>, ArrayModeDynamic, 32> A8D;
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeDynamic, 16> AF16D;
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeStatic, 3> AF16S;

    A8D a1;
    a1 = "12\n3\x44\xa5\xde\xad\x79";
    uavcan::YamlStreamer<A8D>::stream(std::cout, a1, 0);
    std::cout << std::endl;

    A8D a2;
    a2 = "Hello";
    uavcan::YamlStreamer<A8D>::stream(std::cout, a2, 0);
    std::cout << std::endl;

    AF16D af16d1;
    af16d1.push_back(1.23F);
    af16d1.push_back(4.56F);
    uavcan::YamlStreamer<AF16D>::stream(std::cout, af16d1, 0);
    std::cout << std::endl;

    AF16D af16d2;
    uavcan::YamlStreamer<AF16D>::stream(std::cout, af16d2, 0);
    std::cout << std::endl;

    AF16S af16s;
    uavcan::YamlStreamer<AF16S>::stream(std::cout, af16s, 0);
    std::cout << std::endl;
}


TEST(Array, MultidimensionalStreaming)
{
    typedef Array<FloatSpec<16, CastModeSaturate>, ArrayModeDynamic, 16> Float16Array;
    typedef Array<Float16Array, ArrayModeDynamic, 8> TwoDimensional;
    typedef Array<TwoDimensional, ArrayModeDynamic, 4> ThreeDimensional;

    ThreeDimensional threedee;
    threedee.resize(3);
    for (uint8_t x = 0; x < threedee.size(); x++)
    {
        threedee[x].resize(3);
        for (uint8_t y = 0; y < threedee[x].size(); y++)
        {
            threedee[x][y].resize(3);
            for (uint8_t z = 0; z < threedee[x][y].size(); z++)
            {
                threedee[x][y][z] = 1.0F / (float(x + y + z) + 1.0F);
            }
        }
    }

    uavcan::YamlStreamer<ThreeDimensional>::stream(std::cout, threedee, 0);
    std::cout << std::endl;
}


TEST(Array, SquareMatrixPacking)
{
    Array<FloatSpec<16, CastModeSaturate>, ArrayModeDynamic, 9> m3x3s;
    Array<FloatSpec<32, CastModeSaturate>, ArrayModeDynamic, 4> m2x2f;
    Array<FloatSpec<64, CastModeSaturate>, ArrayModeDynamic, 36> m6x6d;

    // NAN will be reduced to empty array
    {
        const double nans3x3[] =
        {
            NAN, NAN, NAN,
            NAN, NAN, NAN,
            NAN, NAN, NAN
        };
        m3x3s.packSquareMatrix(nans3x3);
        ASSERT_EQ(0, m3x3s.size());

        // Empty array will be decoded as zero matrix
        double nans3x3_out[9];
        m3x3s.unpackSquareMatrix(nans3x3_out);
        for (uint8_t i = 0; i < 9; i++)
        {
            ASSERT_DOUBLE_EQ(0, nans3x3_out[i]);
        }
    }
    {
        std::vector<double> empty;
        m3x3s.packSquareMatrix(empty);
        ASSERT_EQ(0, m3x3s.size());

        empty.resize(9);
        m3x3s.unpackSquareMatrix(empty);
        for (uint8_t i = 0; i < 9; i++)
        {
            ASSERT_DOUBLE_EQ(0, empty.at(i));
        }
    }

    // Scalar matrix will be reduced to a single value
    {
        std::vector<float> scalar2x2(4);
        scalar2x2[0] = scalar2x2[3] = 3.14F;
        m2x2f.packSquareMatrix(scalar2x2);
        ASSERT_EQ(1, m2x2f.size());
        ASSERT_FLOAT_EQ(3.14F, m2x2f[0]);

        m2x2f.unpackSquareMatrix(scalar2x2);
        const float reference[] =
        {
            3.14F, 0.0F,
            0.0F, 3.14F
        };
        ASSERT_TRUE(std::equal(scalar2x2.begin(), scalar2x2.end(), reference));
    }
    {
        const float scalar6x6[] =
        {
            -18, 0, 0, 0, 0, 0,
            0, -18, 0, 0, 0, 0,
            0, 0, -18, 0, 0, 0,
            0, 0, 0, -18, 0, 0,
            0, 0, 0, 0, -18, 0,
            0, 0, 0, 0, 0, -18
        };
        m6x6d.packSquareMatrix(scalar6x6);
        ASSERT_EQ(1, m6x6d.size());
        ASSERT_DOUBLE_EQ(-18, m6x6d[0]);

        std::vector<long double> output(36);
        m6x6d.unpackSquareMatrix(output);
        ASSERT_TRUE(std::equal(output.begin(), output.end(), scalar6x6));
    }

    // Diagonal matrix will be reduced to an array of length Width
    {
        const float diagonal6x6[] =
        {
            1,  0,  0,  0,  0,  0,
            0, -2,  0,  0,  0,  0,
            0,  0,  3,  0,  0,  0,
            0,  0,  0, -4,  0,  0,
            0,  0,  0,  0,  5,  0,
            0,  0,  0,  0,  0, -6
        };
        m6x6d.packSquareMatrix(diagonal6x6);
        ASSERT_EQ(6, m6x6d.size());
        ASSERT_DOUBLE_EQ(1,  m6x6d[0]);
        ASSERT_DOUBLE_EQ(-2, m6x6d[1]);
        ASSERT_DOUBLE_EQ(3,  m6x6d[2]);
        ASSERT_DOUBLE_EQ(-4, m6x6d[3]);
        ASSERT_DOUBLE_EQ(5,  m6x6d[4]);
        ASSERT_DOUBLE_EQ(-6, m6x6d[5]);

        std::vector<long double> output(36);
        m6x6d.unpackSquareMatrix(output);
        ASSERT_TRUE(std::equal(output.begin(), output.end(), diagonal6x6));
    }

    // A matrix filled with random values will not be compressed
    {
        std::vector<float> full3x3(9);
        for (uint8_t i = 0; i < 9; i++)
        {
            full3x3[i] = float(i);
        }
        m3x3s.packSquareMatrix(full3x3);
        ASSERT_EQ(9, m3x3s.size());
        for (uint8_t i = 0; i < 9; i++)
        {
            ASSERT_FLOAT_EQ(float(i), m3x3s[i]);
        }

        long output[9];
        m3x3s.unpackSquareMatrix(output);
        ASSERT_TRUE(std::equal(full3x3.begin(), full3x3.end(), output));
    }

    // This will be represented as diagonal - NANs are exceptional
    {
        const double scalarnan3x3[] =
        {
            NAN, 0, 0,
            0, NAN, 0,
            0, 0, NAN
        };
        m3x3s.packSquareMatrix(scalarnan3x3);
        ASSERT_EQ(3, m3x3s.size());
        ASSERT_FALSE(m3x3s[0] <= m3x3s[0]);   // NAN
        ASSERT_FALSE(m3x3s[1] <= m3x3s[1]);   // NAN
        ASSERT_FALSE(m3x3s[2] <= m3x3s[2]);   // NAN

        float output[9];
        m3x3s.unpackSquareMatrix(output);
        ASSERT_FALSE(output[0] <= output[0]);   // NAN
        ASSERT_EQ(0, output[1]);
        ASSERT_EQ(0, output[2]);
        ASSERT_EQ(0, output[3]);
        ASSERT_FALSE(output[4] <= output[4]);   // NAN
        ASSERT_EQ(0, output[5]);
        ASSERT_EQ(0, output[6]);
        ASSERT_EQ(0, output[7]);
        ASSERT_FALSE(output[8] <= output[8]);   // NAN
    }

    // This is a full matrix too (notice the NAN)
    {
        const float full2x2[] =
        {
            1,  NAN,
            0, -2
        };
        m2x2f.packSquareMatrix(full2x2);
        ASSERT_EQ(4, m2x2f.size());
        ASSERT_FLOAT_EQ(1, m2x2f[0]);
        ASSERT_FALSE(m2x2f[1] <= m2x2f[1]);   // NAN
        ASSERT_FLOAT_EQ(0, m2x2f[2]);
        ASSERT_FLOAT_EQ(-2, m2x2f[3]);

        float output[4];
        m2x2f.unpackSquareMatrix(output);
        ASSERT_EQ(1, output[0]);
        ASSERT_FALSE(output[1] <= output[1]);   // NAN
        ASSERT_EQ(0, output[2]);
        ASSERT_EQ(-2, output[3]);
    }

    // Zero matrix will be represented as scalar matrix
    {
        const float zero2x2[] =
        {
            0, 0,
            0, 0
        };
        m2x2f.packSquareMatrix(zero2x2);
        ASSERT_EQ(1, m2x2f.size());
        ASSERT_FLOAT_EQ(0, m2x2f[0]);
    }

    // Symmetric matrix will contain only upper-right triangle
    {
        const float sym2x2[] =
        {
            1, 2,
            2, 1
        };
        m2x2f.packSquareMatrix(sym2x2);
        ASSERT_EQ(3, m2x2f.size());

        float sym2x2_out[4];
        m2x2f.unpackSquareMatrix(sym2x2_out);
        ASSERT_FLOAT_EQ(1, sym2x2_out[0]);
        ASSERT_FLOAT_EQ(2, sym2x2_out[1]);
        ASSERT_FLOAT_EQ(2, sym2x2_out[2]);
        ASSERT_FLOAT_EQ(1, sym2x2_out[3]);
    }
    {
        const float sym3x3[] =
        {
            1, 2, 3,
            2, 4, 5,
            3, 5, 6
        };
        m3x3s.packSquareMatrix(sym3x3);
        ASSERT_EQ(6, m3x3s.size());
        ASSERT_EQ(1, m3x3s[0]);
        ASSERT_EQ(2, m3x3s[1]);
        ASSERT_EQ(3, m3x3s[2]);
        ASSERT_EQ(4, m3x3s[3]);
        ASSERT_EQ(5, m3x3s[4]);
        ASSERT_EQ(6, m3x3s[5]);

        float sym3x3_out[9];
        m3x3s.unpackSquareMatrix(sym3x3_out);

        for (int i = 0; i < 9; i++)
        {
            ASSERT_FLOAT_EQ(sym3x3[i], sym3x3_out[i]);
        }
    }
    {
        const double sym6x6[] =
        {
            1,  2,  3,  4,  5,  6,
            2,  7,  8,  9, 10, 11,
            3,  8, 12, 13, 14, 15,
            4,  9, 13, 16, 17, 18,
            5, 10, 14, 17, 19, 20,
            6, 11, 15, 18, 20, 21
        };
        m6x6d.packSquareMatrix(sym6x6);
        ASSERT_EQ(21, m6x6d.size());
        for (uavcan::uint8_t i = 0; i < 21; i++)
        {
            ASSERT_DOUBLE_EQ(double(i + 1), m6x6d[i]);
        }

        double sym6x6_out[36];
        m6x6d.unpackSquareMatrix(sym6x6_out);

        for (int i = 0; i < 36; i++)
        {
            ASSERT_DOUBLE_EQ(sym6x6[i], sym6x6_out[i]);
        }
    }
}


TEST(Array, FuzzySquareMatrixPacking)
{
    Array<FloatSpec<64, CastModeSaturate>, ArrayModeDynamic, 36> m6x6d;

    // Diagonal matrix will be reduced to an array of length Width
    {
        float diagonal6x6[] =
        {
            1,  0,  0,  0,  0,  0,
            0, -2,  0,  0,  0,  0,
            0,  0,  3,  0,  0,  0,
            0,  0,  0, -4,  0,  0,
            0,  0,  0,  0,  5,  0,
            0,  0,  0,  0,  0, -6
        };

        // Some almost-zeroes
        diagonal6x6[1]  =  std::numeric_limits<float>::epsilon();
        diagonal6x6[4]  = -std::numeric_limits<float>::epsilon();
        diagonal6x6[34] = -std::numeric_limits<float>::epsilon();

        m6x6d.packSquareMatrix(diagonal6x6);
        ASSERT_EQ(6, m6x6d.size());
        ASSERT_DOUBLE_EQ(1,  m6x6d[0]);
        ASSERT_DOUBLE_EQ(-2, m6x6d[1]);
        ASSERT_DOUBLE_EQ(3,  m6x6d[2]);
        ASSERT_DOUBLE_EQ(-4, m6x6d[3]);
        ASSERT_DOUBLE_EQ(5,  m6x6d[4]);
        ASSERT_DOUBLE_EQ(-6, m6x6d[5]);

        std::vector<long double> output(36);
        m6x6d.unpackSquareMatrix(output);

        // This comparison will fail because epsilons
        ASSERT_FALSE(std::equal(output.begin(), output.end(), diagonal6x6));

        // This comparison will be ok
        ASSERT_TRUE(std::equal(output.begin(), output.end(), diagonal6x6, &uavcan::areClose<float, float>));
    }
}


TEST(Array, SquareMatrixPackingIntegers)
{
    Array<IntegerSpec<30, SignednessSigned, CastModeSaturate>, ArrayModeDynamic, 9> m3x3int;
    {
        const long scalar[] =
        {
            42, 0, 0,
            0, 42, 0,
            0, 0, 42
        };
        m3x3int.packSquareMatrix(scalar);
        ASSERT_EQ(1, m3x3int.size());
        ASSERT_EQ(42, m3x3int[0]);

        std::vector<int> output(9);
        m3x3int.unpackSquareMatrix(output);
        ASSERT_TRUE(std::equal(output.begin(), output.end(), scalar));
    }
    {
        std::vector<short> diagonal(9);
        diagonal[0] = 6;
        diagonal[4] = -57;
        diagonal[8] = 1139;
        m3x3int.packSquareMatrix(diagonal);
        ASSERT_EQ(3, m3x3int.size());
        ASSERT_EQ(6, m3x3int[0]);
        ASSERT_EQ(-57, m3x3int[1]);
        ASSERT_EQ(1139, m3x3int[2]);
    }
    {
        std::vector<long double> full(9);
        for (uint8_t i = 0; i < 9; i++)
        {
            full[i] = i;
        }
        m3x3int.packSquareMatrix(full);
        ASSERT_EQ(9, m3x3int.size());
        for (uint8_t i = 0; i < 9; i++)
        {
            ASSERT_EQ(i, m3x3int[i]);
        }
    }
}

#if UAVCAN_EXCEPTIONS
TEST(Array, SquareMatrixPackingErrors)
{
    Array<FloatSpec<16, CastModeSaturate>, ArrayModeDynamic, 9> m3x3s;

    std::vector<float> ill_formed_row_major(8);
    ASSERT_THROW(m3x3s.packSquareMatrix(ill_formed_row_major), std::out_of_range);

    ASSERT_THROW(m3x3s.unpackSquareMatrix(ill_formed_row_major), std::out_of_range);
}
#endif

TEST(Array, SquareMatrixPackingInPlace)
{
    Array<FloatSpec<16, CastModeSaturate>, ArrayModeDynamic, 9> m3x3s;

    // Will do nothing - matrix is empty
    m3x3s.packSquareMatrix();
    ASSERT_TRUE(m3x3s.empty());

    // Will fill with zeros - matrix is empty
    m3x3s.unpackSquareMatrix();
    ASSERT_EQ(9, m3x3s.size());
    for (uint8_t i = 0; i < 9; i++)
    {
        ASSERT_EQ(0, m3x3s[i]);
    }

    // Fill an unpackaple matrix
    m3x3s.clear();
    m3x3s.push_back(11);
    m3x3s.push_back(12);
    m3x3s.push_back(13);

#if UAVCAN_EXCEPTIONS
    // Shall throw - matrix is ill-formed
    ASSERT_THROW(m3x3s.packSquareMatrix(), std::out_of_range);
#endif

    m3x3s.push_back(21);
    m3x3s.push_back(22);
    m3x3s.push_back(23);
    m3x3s.push_back(31);
    m3x3s.push_back(32);
    m3x3s.push_back(33);

    // Will pack/unpack successfully
    ASSERT_EQ(9, m3x3s.size());
    m3x3s.packSquareMatrix();
    ASSERT_EQ(9, m3x3s.size());
    m3x3s.unpackSquareMatrix();

    // Make sure it was unpacked properly
    ASSERT_EQ(11, m3x3s[0]);
    ASSERT_EQ(12, m3x3s[1]);
    ASSERT_EQ(13, m3x3s[2]);
    ASSERT_EQ(21, m3x3s[3]);
    ASSERT_EQ(22, m3x3s[4]);
    ASSERT_EQ(23, m3x3s[5]);
    ASSERT_EQ(31, m3x3s[6]);
    ASSERT_EQ(32, m3x3s[7]);
    ASSERT_EQ(33, m3x3s[8]);

    // Try again with a scalar matrix
    m3x3s.clear();
    for (unsigned i = 0; i < 9; i++)
    {
        const bool diagonal = (i == 0) || (i == 4) || (i == 8);
        m3x3s.push_back(diagonal ? 123 : 0);
    }

    ASSERT_EQ(9, m3x3s.size());
    m3x3s.packSquareMatrix();
    ASSERT_EQ(1, m3x3s.size());
    m3x3s.unpackSquareMatrix();
    ASSERT_EQ(9, m3x3s.size());

    for (uint8_t i = 0; i < 9; i++)
    {
        const bool diagonal = (i == 0) || (i == 4) || (i == 8);
        ASSERT_EQ((diagonal ? 123 : 0), m3x3s[i]);
    }

    // Try again with symmetric matrix
    /*
     * Full matrix:
     * 1 2 3
     * 2 4 5
     * 3 5 6
     * Compressed triangle:
     * 1 2 3
     *   4 5
     *     6
     */
    m3x3s.clear();
    m3x3s.push_back(1);
    m3x3s.push_back(2);
    m3x3s.push_back(3);
    m3x3s.push_back(4);
    m3x3s.push_back(5);
    m3x3s.push_back(6);
    // Unpacking
    ASSERT_EQ(6, m3x3s.size());
    m3x3s.unpackSquareMatrix();
    ASSERT_EQ(9, m3x3s.size());
    // Validating
    ASSERT_EQ(1, m3x3s[0]);
    ASSERT_EQ(2, m3x3s[1]);
    ASSERT_EQ(3, m3x3s[2]);
    ASSERT_EQ(2, m3x3s[3]);
    ASSERT_EQ(4, m3x3s[4]);
    ASSERT_EQ(5, m3x3s[5]);
    ASSERT_EQ(3, m3x3s[6]);
    ASSERT_EQ(5, m3x3s[7]);
    ASSERT_EQ(6, m3x3s[8]);
    // Packing back
    m3x3s.packSquareMatrix();
    ASSERT_EQ(6, m3x3s.size());
    // Validating
    ASSERT_EQ(1, m3x3s[0]);
    ASSERT_EQ(2, m3x3s[1]);
    ASSERT_EQ(3, m3x3s[2]);
    ASSERT_EQ(4, m3x3s[3]);
    ASSERT_EQ(5, m3x3s[4]);
    ASSERT_EQ(6, m3x3s[5]);
}

TEST(Array, FuzzyComparison)
{
    typedef Array<Array<Array<FloatSpec<32, CastModeSaturate>, ArrayModeStatic, 2>,
                ArrayModeStatic, 2>,
          ArrayModeStatic, 2> ArrayStatic32;

    typedef Array<Array<Array<FloatSpec<64, CastModeSaturate>, ArrayModeDynamic, 2>,
                ArrayModeDynamic, 2>,
          ArrayModeDynamic, 2> ArrayDynamic64;

    ArrayStatic32 array_s32;

    ArrayDynamic64 array_d64;

    array_d64.resize(2);
    array_d64[0].resize(2);
    array_d64[1].resize(2);
    array_d64[0][0].resize(2);
    array_d64[0][1].resize(2);
    array_d64[1][0].resize(2);
    array_d64[1][1].resize(2);

    std::cout << "One:";
    uavcan::YamlStreamer<ArrayStatic32>::stream(std::cout, array_s32, 0);
    std::cout << std::endl << "------";
    uavcan::YamlStreamer<ArrayDynamic64>::stream(std::cout, array_d64, 0);
    std::cout << std::endl;

    // Both are equal right now
    ASSERT_TRUE(array_d64 == array_s32);
    ASSERT_TRUE(array_d64.isClose(array_s32));
    ASSERT_TRUE(array_s32.isClose(array_d64));

    // Slightly modifying - still close enough
    array_s32[0][0][0] = 123.456F + uavcan::NumericTraits<float>::epsilon() * 123.0F;
    array_s32[0][0][1] = uavcan::NumericTraits<float>::infinity();
    array_s32[0][1][0] = uavcan::NumericTraits<float>::epsilon();
    array_s32[0][1][1] = -uavcan::NumericTraits<float>::epsilon();

    array_d64[0][0][0] = 123.456;
    array_d64[0][0][1] = uavcan::NumericTraits<double>::infinity();
    array_d64[0][1][0] = -uavcan::NumericTraits<double>::epsilon();  // Note that the sign is inverted
    array_d64[0][1][1] = uavcan::NumericTraits<double>::epsilon();

    std::cout << "Two:";
    uavcan::YamlStreamer<ArrayStatic32>::stream(std::cout, array_s32, 0);
    std::cout << std::endl << "------";
    uavcan::YamlStreamer<ArrayDynamic64>::stream(std::cout, array_d64, 0);
    std::cout << std::endl;

    // They are close bot not exactly equal
    ASSERT_FALSE(array_d64 == array_s32);
    ASSERT_TRUE(array_d64.isClose(array_s32));
    ASSERT_TRUE(array_s32.isClose(array_d64));

    // Not close
    array_d64[0][0][0] = 123.457;

    ASSERT_FALSE(array_d64 == array_s32);
    ASSERT_FALSE(array_d64.isClose(array_s32));
    ASSERT_FALSE(array_s32.isClose(array_d64));

    // Values are close, but lengths differ
    array_d64[0][0][0] = 123.456;

    ASSERT_FALSE(array_d64 == array_s32);
    ASSERT_TRUE(array_d64.isClose(array_s32));
    ASSERT_TRUE(array_s32.isClose(array_d64));

    array_d64[0][0].resize(1);

    ASSERT_FALSE(array_d64 == array_s32);
    ASSERT_FALSE(array_d64.isClose(array_s32));
    ASSERT_FALSE(array_s32.isClose(array_d64));

    std::cout << "Three:";
    uavcan::YamlStreamer<ArrayStatic32>::stream(std::cout, array_s32, 0);
    std::cout << std::endl << "------";
    uavcan::YamlStreamer<ArrayDynamic64>::stream(std::cout, array_d64, 0);
    std::cout << std::endl;
}

TEST(Array, CaseConversion)
{
    Array<IntegerSpec<8, SignednessUnsigned, CastModeTruncate>, ArrayModeDynamic, 30> str;

    str.convertToLowerCaseASCII();
    str.convertToUpperCaseASCII();

    ASSERT_STREQ("", str.c_str());

    str = "Hello World!";

    ASSERT_STREQ("Hello World!", str.c_str());
    str.convertToLowerCaseASCII();
    ASSERT_STREQ("hello world!", str.c_str());
    str.convertToUpperCaseASCII();
    ASSERT_STREQ("HELLO WORLD!", str.c_str());
}
