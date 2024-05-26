/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_MARSHAL_SCALAR_CODEC_HPP_INCLUDED
#define UAVCAN_MARSHAL_SCALAR_CODEC_HPP_INCLUDED

#include <cassert>
#include <uavcan/std.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/marshal/bit_stream.hpp>

namespace uavcan
{
/**
 * This class implements fast encoding/decoding of primitive type scalars into/from bit arrays.
 * It uses the compile-time type information to eliminate run-time operations where possible.
 */
class UAVCAN_EXPORT ScalarCodec
{
    BitStream& stream_;

    static void swapByteOrder(uint8_t* bytes, unsigned len);

    template <unsigned BitLen, unsigned Size>
    static typename EnableIf<(BitLen > 8)>::Type
    convertByteOrder(uint8_t (&bytes)[Size])
    {
#if defined(BYTE_ORDER) && defined(BIG_ENDIAN)
        static const bool big_endian = BYTE_ORDER == BIG_ENDIAN;
#else
        union { long int l; char c[sizeof(long int)]; } u;
        u.l = 1;
        const bool big_endian = u.c[sizeof(long int) - 1] == 1;
#endif
        /*
         * I didn't have any big endian machine nearby, so big endian support wasn't tested yet.
         * It is likely to be OK anyway, so feel free to remove this UAVCAN_ASSERT() as needed.
         */
        UAVCAN_ASSERT(big_endian == false);
        if (big_endian)
        {
            swapByteOrder(bytes, Size);
        }
    }

    template <unsigned BitLen, unsigned Size>
    static typename EnableIf<(BitLen <= 8)>::Type
    convertByteOrder(uint8_t (&)[Size]) { }

    template <unsigned BitLen, typename T>
    static typename EnableIf<static_cast<bool>(NumericTraits<T>::IsSigned) && ((sizeof(T) * 8) > BitLen)>::Type
    fixTwosComplement(T& value)
    {
        StaticAssert<NumericTraits<T>::IsInteger>::check(); // Not applicable to floating point types
        if (value & (T(1) << (BitLen - 1)))                        // The most significant bit is set --> negative
        {
            value |= T(T(0xFFFFFFFFFFFFFFFFULL) & ~((T(1) << BitLen) - 1));
        }
    }

    template <unsigned BitLen, typename T>
    static typename EnableIf<!static_cast<bool>(NumericTraits<T>::IsSigned) || ((sizeof(T) * 8) == BitLen)>::Type
    fixTwosComplement(T&) { }

    template <unsigned BitLen, typename T>
    static typename EnableIf<((sizeof(T) * 8) > BitLen)>::Type
    clearExtraBits(T& value)
    {
        value &= (T(1) << BitLen) - 1;  // Signedness doesn't matter
    }

    template <unsigned BitLen, typename T>
    static typename EnableIf<((sizeof(T) * 8) == BitLen)>::Type
    clearExtraBits(T&) { }

    template <unsigned BitLen, typename T>
    void validate()
    {
        StaticAssert<((sizeof(T) * 8) >= BitLen)>::check();
        StaticAssert<(BitLen <= BitStream::MaxBitsPerRW)>::check();
        StaticAssert<static_cast<bool>(NumericTraits<T>::IsSigned) ? (BitLen > 1) : true>::check();
    }

    int encodeBytesImpl(uint8_t* bytes, unsigned bitlen);
    int decodeBytesImpl(uint8_t* bytes, unsigned bitlen);

public:
    explicit ScalarCodec(BitStream& stream)
        : stream_(stream)
    { }

    template <unsigned BitLen, typename T>
    int encode(const T value);

    template <unsigned BitLen, typename T>
    int decode(T& value);
};

// ----------------------------------------------------------------------------

template <unsigned BitLen, typename T>
int ScalarCodec::encode(const T value)
{
    validate<BitLen, T>();
    union ByteUnion
    {
        T value;
        uint8_t bytes[sizeof(T)];
    } byte_union;
    byte_union.value = value;
    clearExtraBits<BitLen, T>(byte_union.value);
    convertByteOrder<BitLen>(byte_union.bytes);
    return encodeBytesImpl(byte_union.bytes, BitLen);
}

template <unsigned BitLen, typename T>
int ScalarCodec::decode(T& value)
{
    validate<BitLen, T>();
    union ByteUnion
    {
        T value;
        uint8_t bytes[sizeof(T)];
    } byte_union;
    byte_union.value = T();
    const int read_res = decodeBytesImpl(byte_union.bytes, BitLen);
    if (read_res > 0)
    {
        convertByteOrder<BitLen>(byte_union.bytes);
        fixTwosComplement<BitLen, T>(byte_union.value);
        value = byte_union.value;
    }
    return read_res;
}

}

#endif // UAVCAN_MARSHAL_SCALAR_CODEC_HPP_INCLUDED
