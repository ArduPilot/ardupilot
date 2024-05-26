/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/util/comparison.hpp>

TEST(Comparison, Basic)
{
    // Basic same type floats
    ASSERT_TRUE(uavcan::areClose(0.1, 0.1));
    ASSERT_TRUE(uavcan::areClose(0.1F, 0.1F));
    ASSERT_TRUE(uavcan::areClose(0.1L, 0.1L));

    // Basic mixed type floats
    ASSERT_TRUE(uavcan::areClose(0.1F, 0.1));
    ASSERT_TRUE(uavcan::areClose(0.1,  0.1F));
    ASSERT_TRUE(uavcan::areClose(0.1F, 0.1L));
    ASSERT_TRUE(uavcan::areClose(0.1L, 0.1F));
    ASSERT_TRUE(uavcan::areClose(0.1,  0.1L));
    ASSERT_TRUE(uavcan::areClose(0.1L, 0.1));

    // Basic floats
    ASSERT_TRUE(uavcan::areClose(0x07, '\x07'));
    ASSERT_TRUE(uavcan::areClose(123456789LL, 123456789));
    ASSERT_TRUE(uavcan::areClose("123", std::string("123")));

    // Non-equality
    ASSERT_FALSE(uavcan::areClose(-0.1, 0.1));
    ASSERT_FALSE(uavcan::areClose(-0.1F, 0.0L));
    ASSERT_FALSE(uavcan::areClose("123", std::string("12")));
    ASSERT_FALSE(uavcan::areClose(0x07L, '\0'));
}

TEST(Comparison, FloatSpecialCase)
{
    ASSERT_FALSE(uavcan::areClose(0.1, std::numeric_limits<double>::infinity()));

    ASSERT_TRUE(uavcan::areClose(std::numeric_limits<float>::infinity(),
                                 std::numeric_limits<long double>::infinity()));

    ASSERT_FALSE(uavcan::areClose(std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()));

    ASSERT_FALSE(uavcan::areClose(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()));
}

TEST(Comparison, FloatULP)
{
    ASSERT_FALSE(uavcan::areClose(0.100000000000000001L, 0.1L));
    ASSERT_TRUE( uavcan::areClose(0.100000000000000001,  0.1L));
    ASSERT_TRUE( uavcan::areClose(0.100000000000000001F, 0.1L));

    // Near zero
    ASSERT_TRUE( uavcan::areClose(0.0F,  std::numeric_limits<float>::epsilon()));
    ASSERT_TRUE( uavcan::areClose(0.0F, -std::numeric_limits<float>::epsilon()));
    ASSERT_FALSE(uavcan::areClose(0.0F,  std::numeric_limits<float>::epsilon() * 2));
}

TEST(Comparison, BruteforceValidation)
{
    const std::streamsize default_precision = std::cout.precision();
    std::cout.precision(20);

    float x = -uavcan::NumericTraits<float>::max();

    while (x < uavcan::NumericTraits<float>::max())
    {
        const float y1 = x + std::abs(x) * (uavcan::NumericTraits<float>::epsilon() * 2);                // Still equal
        const float y2 = x + uavcan::max(std::abs(x), 1.F) * (uavcan::NumericTraits<float>::epsilon() * 20); // Nope

        if (!uavcan::areClose(y1, x))
        {
            std::cout << "y1=" << y1 << " y2=" << y2 << " x=" << x << std::endl;
            ASSERT_TRUE(false);
        }
        if (uavcan::areClose(y2, x))
        {
            std::cout << "y1=" << y1 << " y2=" << y2 << " x=" << x << std::endl;
            ASSERT_TRUE(false);
        }

        x = y2;
    }

    std::cout.precision(default_precision);
}


struct B
{
    long double b;
    B(long double val = 0.0L) : b(val) { }
};

struct A
{
    float a;
    explicit A(float val = 0.0F) : a(val) { }

    bool isClose(A rhs) const
    {
        std::cout << "bool A::isClose(A) --> " << rhs.a << std::endl;
        return uavcan::areClose(a, rhs.a);
    }

    bool isClose(const B& rhs) const
    {
        std::cout << "bool A::isClose(const B&) --> " << rhs.b << std::endl;
        return uavcan::areClose(a, rhs.b);
    }
};

struct C
{
    long long c;
    explicit C(long long val = 0.0L) : c(val) { }

    bool operator==(B rhs) const
    {
        std::cout << "bool C::operator==(B) --> " << rhs.b << std::endl;
        return c == static_cast<long long>(rhs.b);
    }
};

TEST(Comparison, IsCloseMethod)
{
    B b;
    A a;
    C c;

    std::cout << 1 << std::endl;
    ASSERT_TRUE(uavcan::areClose(a, b));   // Fuzzy
    ASSERT_TRUE(uavcan::areClose(a, A())); // Fuzzy
    ASSERT_TRUE(uavcan::areClose(b, a));   // Fuzzy, reverse application
    ASSERT_TRUE(uavcan::areClose(c, b));   // Exact

    std::cout << 2 << std::endl;

    a.a = uavcan::NumericTraits<float>::epsilon();

    ASSERT_TRUE(uavcan::areClose(a, b));
    ASSERT_TRUE(uavcan::areClose(b, a));
    ASSERT_TRUE(a.isClose(b));
    ASSERT_TRUE(a.isClose(A()));
    ASSERT_TRUE(uavcan::areClose(A(), a));

    std::cout << 3 << std::endl;

    a.a = 1e-5F;

    ASSERT_FALSE(uavcan::areClose(a, b));
    ASSERT_FALSE(uavcan::areClose(b, a));
    ASSERT_FALSE(uavcan::areClose(A(), a));

    std::cout << 4 << std::endl;

    b.b = 1.1L;
    c.c = 1;

    ASSERT_TRUE(uavcan::areClose(c, b));      // Round to integer
    ASSERT_TRUE(uavcan::areClose(c, 1.0L));   // Implicit cast to B
    ASSERT_FALSE(uavcan::areClose(c, 0.0L));
}
