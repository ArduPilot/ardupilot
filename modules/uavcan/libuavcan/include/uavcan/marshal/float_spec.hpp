/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_MARSHAL_FLOAT_SPEC_HPP_INCLUDED
#define UAVCAN_MARSHAL_FLOAT_SPEC_HPP_INCLUDED

#include <cmath>
#include <uavcan/std.hpp>
#include <uavcan/data_type.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/marshal/type_util.hpp>
#include <uavcan/marshal/integer_spec.hpp>

#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <limits>     // Assuming that in C++11 mode all standard headers are available
#endif

namespace uavcan
{

template <unsigned BitLen>
struct NativeFloatSelector
{
    struct ErrorNoSuchFloat;
    typedef typename Select<(sizeof(float) * 8 >= BitLen), float,
            typename Select<(sizeof(double) * 8 >= BitLen), double,
            typename Select<(sizeof(long double) * 8 >= BitLen), long double,
                              ErrorNoSuchFloat>::Result>::Result>::Result Type;
};


class UAVCAN_EXPORT IEEE754Converter
{
    // TODO: Non-IEEE float support

    static uint16_t nativeIeeeToHalf(float value);
    static float halfToNativeIeee(uint16_t value);

    IEEE754Converter();

    template <unsigned BitLen>
    static void enforceIeee()
    {
        /*
         * Some compilers may have is_iec559 to be defined false despite the fact that IEEE754 is supported.
         * An acceptable workaround would be to put an #ifdef here.
         */
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
        StaticAssert<std::numeric_limits<typename NativeFloatSelector<BitLen>::Type>::is_iec559>::check();
#endif
    }

public:
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
    /// UAVCAN requires rounding to nearest for all float conversions
    static std::float_round_style roundstyle() { return std::round_to_nearest; }
#endif

    template <unsigned BitLen>
    static typename IntegerSpec<BitLen, SignednessUnsigned, CastModeTruncate>::StorageType
    toIeee(typename NativeFloatSelector<BitLen>::Type value)
    {
        enforceIeee<BitLen>();
        union
        {
            typename IntegerSpec<BitLen, SignednessUnsigned, CastModeTruncate>::StorageType i;
            typename NativeFloatSelector<BitLen>::Type f;
        } u;
        StaticAssert<sizeof(u.f) * 8 == BitLen>::check();
        u.f = value;
        return u.i;
    }

    template <unsigned BitLen>
    static typename NativeFloatSelector<BitLen>::Type
    toNative(typename IntegerSpec<BitLen, SignednessUnsigned, CastModeTruncate>::StorageType value)
    {
        enforceIeee<BitLen>();
        union
        {
            typename IntegerSpec<BitLen, SignednessUnsigned, CastModeTruncate>::StorageType i;
            typename NativeFloatSelector<BitLen>::Type f;
        } u;
        StaticAssert<sizeof(u.f) * 8 == BitLen>::check();
        u.i = value;
        return u.f;
    }
};
template <>
inline typename IntegerSpec<16, SignednessUnsigned, CastModeTruncate>::StorageType
IEEE754Converter::toIeee<16>(typename NativeFloatSelector<16>::Type value)
{
    return nativeIeeeToHalf(value);
}
template <>
inline typename NativeFloatSelector<16>::Type
IEEE754Converter::toNative<16>(typename IntegerSpec<16, SignednessUnsigned, CastModeTruncate>::StorageType value)
{
    return halfToNativeIeee(value);
}


template <unsigned BitLen> struct IEEE754Limits;
template <> struct IEEE754Limits<16>
{
    typedef typename NativeFloatSelector<16>::Type NativeType;
    static NativeType max() { return static_cast<NativeType>(65504.0); }
    static NativeType epsilon() { return static_cast<NativeType>(9.77e-04); }
};
template <> struct IEEE754Limits<32>
{
    typedef typename NativeFloatSelector<32>::Type NativeType;
    static NativeType max() { return static_cast<NativeType>(3.40282346638528859812e+38); }
    static NativeType epsilon() { return static_cast<NativeType>(1.19209289550781250000e-7); }
};
template <> struct IEEE754Limits<64>
{
    typedef typename NativeFloatSelector<64>::Type NativeType;
    static NativeType max() { return static_cast<NativeType>(1.79769313486231570815e+308L); }
    static NativeType epsilon() { return static_cast<NativeType>(2.22044604925031308085e-16L); }
};


template <unsigned BitLen_, CastMode CastMode>
class UAVCAN_EXPORT FloatSpec : public IEEE754Limits<BitLen_>
{
    FloatSpec();

public:
    enum { BitLen = BitLen_ };
    enum { MinBitLen = BitLen };
    enum { MaxBitLen = BitLen };
    enum { IsPrimitive = 1 };

    typedef typename NativeFloatSelector<BitLen>::Type StorageType;

#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
    enum { IsExactRepresentation = (sizeof(StorageType) * 8 == BitLen) };
#else
    enum { IsExactRepresentation = (sizeof(StorageType) * 8 == BitLen) && std::numeric_limits<StorageType>::is_iec559 };
#endif

    using IEEE754Limits<BitLen>::max;
    using IEEE754Limits<BitLen>::epsilon;
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
    static std::float_round_style roundstyle() { return IEEE754Converter::roundstyle(); }
#endif

    static int encode(StorageType value, ScalarCodec& codec, TailArrayOptimizationMode)
    {
        // cppcheck-suppress duplicateExpression
        if (CastMode == CastModeSaturate)
        {
            saturate(value);
        }
        else
        {
            truncate(value);
        }
        return codec.encode<BitLen>(IEEE754Converter::toIeee<BitLen>(value));
    }

    static int decode(StorageType& out_value, ScalarCodec& codec, TailArrayOptimizationMode)
    {
        typename IntegerSpec<BitLen, SignednessUnsigned, CastModeTruncate>::StorageType ieee = 0;
        const int res = codec.decode<BitLen>(ieee);
        if (res <= 0)
        {
            return res;
        }
        out_value = IEEE754Converter::toNative<BitLen>(ieee);
        return res;
    }

    static void extendDataTypeSignature(DataTypeSignature&) { }

private:
    static inline void saturate(StorageType& value)
    {
        if ((IsExactRepresentation == 0) && isFinite(value))
        {
            if (value > max())
            {
                value = max();
            }
            else if (value < -max())
            {
                value = -max();
            }
            else
            {
                ; // Valid range
            }
        }
    }

    static inline void truncate(StorageType& value)
    {
        if ((IsExactRepresentation == 0) && isFinite(value))
        {
            if (value > max())
            {
                value = NumericTraits<StorageType>::infinity();
            }
            else if (value < -max())
            {
                value = -NumericTraits<StorageType>::infinity();
            }
            else
            {
                ; // Valid range
            }
        }
    }
};


template <unsigned BitLen, CastMode CastMode>
class UAVCAN_EXPORT YamlStreamer<FloatSpec<BitLen, CastMode> >
{
    typedef typename FloatSpec<BitLen, CastMode>::StorageType StorageType;

public:
    template <typename Stream>  // cppcheck-suppress passedByValue
    static void stream(Stream& s, const StorageType value, int)
    {
        s << value;
    }
};

}

#endif // UAVCAN_MARSHAL_FLOAT_SPEC_HPP_INCLUDED
