/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_MARSHAL_TYPE_UTIL_HPP_INCLUDED
#define UAVCAN_MARSHAL_TYPE_UTIL_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/util/comparison.hpp>

namespace uavcan
{
/**
 * Read the specs to learn more about cast modes.
 */
enum CastMode { CastModeSaturate, CastModeTruncate };

/**
 * Read the specs to learn more about tail array optimizations.
 */
enum TailArrayOptimizationMode { TailArrayOptDisabled, TailArrayOptEnabled };

/**
 * Compile-time: Returns the number of bits needed to represent an integer value.
 */
template <unsigned long Num>
struct IntegerBitLen
{
    enum { Result = 1 + IntegerBitLen<(Num >> 1)>::Result };
};
template <>
struct IntegerBitLen<0>
{
    enum { Result = 0 };
};

/**
 * Compile-time: Returns the number of bytes needed to contain the given number of bits. Assumes 1 byte == 8 bit.
 */
template <unsigned long BitLen>
struct BitLenToByteLen
{
    enum { Result = (BitLen + 7) / 8 };
};

/**
 * Compile-time: Helper for integer and float specialization classes. Returns the platform-specific storage type.
 */
template <typename T, typename Enable = void>
struct StorageType
{
    typedef T Type;
};
template <typename T>
struct StorageType<T, typename EnableIfType<typename T::StorageType>::Type>
{
    typedef typename T::StorageType Type;
};

/**
 * Compile-time: Whether T is a primitive type on this platform.
 */
template <typename T>
class IsPrimitiveType
{
    typedef char Yes;
    struct No { Yes dummy[8]; };

    template <typename U>
    static typename EnableIf<U::IsPrimitive, Yes>::Type test(int);

    template <typename>
    static No test(...);

public:
    enum { Result = sizeof(test<T>(0)) == sizeof(Yes) };
};

/**
 * Streams a given value into YAML string. Please see the specializations.
 */
template <typename T>
class UAVCAN_EXPORT YamlStreamer;

}

#endif // UAVCAN_MARSHAL_TYPE_UTIL_HPP_INCLUDED
