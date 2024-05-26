/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/util/templates.hpp>
#include <limits>

struct NonConvertible { };

struct ConvertibleToBool
{
    const bool value;
    ConvertibleToBool(bool value) : value(value) { }
    operator bool() const { return value; }
    bool operator!() const { return !value; }
};

struct NonDefaultConstructible
{
    NonDefaultConstructible(int) { }
};

TEST(Util, CoerceOrFallback)
{
    using uavcan::coerceOrFallback;

    ASSERT_FALSE(coerceOrFallback<bool>(NonConvertible()));
    ASSERT_TRUE(coerceOrFallback<bool>(NonConvertible(), true));

    ASSERT_EQ(0, coerceOrFallback<long>(NonConvertible()));
    ASSERT_EQ(9000, coerceOrFallback<long>(NonConvertible(), 9000));

    ASSERT_TRUE(coerceOrFallback<bool>(ConvertibleToBool(true)));
    ASSERT_TRUE(coerceOrFallback<bool>(ConvertibleToBool(true), false));
    ASSERT_FALSE(coerceOrFallback<bool>(ConvertibleToBool(false)));
    ASSERT_FALSE(coerceOrFallback<bool>(ConvertibleToBool(false), true));
    ASSERT_EQ(1, coerceOrFallback<long>(ConvertibleToBool(true)));
    ASSERT_EQ(0, coerceOrFallback<long>(ConvertibleToBool(false), -100));

    //coerceOrFallback<NonDefaultConstructible>(ConvertibleToBool(true));   // Will fail to compile
    coerceOrFallback<NonDefaultConstructible>(NonConvertible(), NonDefaultConstructible(64));
}

TEST(Util, FloatClassification)
{
    // NAN
    ASSERT_TRUE(uavcan::isNaN(std::numeric_limits<float>::quiet_NaN()));
    ASSERT_FALSE(uavcan::isNaN(std::numeric_limits<double>::infinity()));
    ASSERT_FALSE(uavcan::isNaN(std::numeric_limits<long double>::infinity()));
    ASSERT_FALSE(uavcan::isNaN(123.456));

    // INF
    ASSERT_TRUE(uavcan::isInfinity(std::numeric_limits<float>::infinity()));
    ASSERT_TRUE(uavcan::isInfinity(-std::numeric_limits<long double>::infinity()));
    ASSERT_FALSE(uavcan::isInfinity(std::numeric_limits<float>::quiet_NaN()));
    ASSERT_FALSE(uavcan::isInfinity(-0.1L));

    // Signbit
    ASSERT_FALSE(uavcan::getSignBit(12));
    ASSERT_TRUE(uavcan::getSignBit(-std::numeric_limits<long double>::infinity()));
    ASSERT_FALSE(uavcan::getSignBit(std::numeric_limits<float>::infinity()));
    ASSERT_TRUE(uavcan::getSignBit(-0.0));                                           // Negative zero
}
