#include <AP_gtest.h>
#include <AP_Common/float16.h>
#include <cmath>

// Float16 has a 10-bit mantissa, giving ~0.1% relative precision.
// Values that are exact in float16 use EXPECT_FLOAT_EQ; others use EXPECT_NEAR.

TEST(Float16, Zero)
{
    Float16_t f;
    f.set(0.0f);
    EXPECT_FLOAT_EQ(0.0f, f.get());
}

TEST(Float16, NegativeZero)
{
    Float16_t f;
    f.set(-0.0f);
    EXPECT_FLOAT_EQ(-0.0f, f.get());
}

TEST(Float16, ExactValues)
{
    // These values are exactly representable in float16
    const float exact[] = {
        1.0f, -1.0f,
        0.5f, -0.5f,
        2.0f, -2.0f,
        0.25f, -0.25f,
        100.0f, -100.0f,
        1000.0f, -1000.0f,
    };
    for (const float v : exact) {
        Float16_t f;
        f.set(v);
        EXPECT_FLOAT_EQ(v, f.get()) << "failed for value " << v;
    }
}

TEST(Float16, RoundTripPrecision)
{
    // Values not exactly representable; round-trip should be within float16 precision (~0.1%)
    const float approx[] = {
        3.14159f, -3.14159f,
        1.1f, -1.1f,
        99.9f,
        0.1f, -0.1f,
        1234.5f,
    };
    const float rel_tol = 0.002f; // 0.2% - comfortable margin over the ~0.1% float16 limit
    for (const float v : approx) {
        Float16_t f;
        f.set(v);
        const float got = f.get();
        EXPECT_NEAR(got, v, fabsf(v) * rel_tol) << "failed for value " << v;
    }
}

TEST(Float16, Infinity)
{
    Float16_t f;

    f.set(1.0f / 0.0f);
    EXPECT_TRUE(std::isinf(f.get()));
    EXPECT_GT(f.get(), 0.0f);

    f.set(-1.0f / 0.0f);
    EXPECT_TRUE(std::isinf(f.get()));
    EXPECT_LT(f.get(), 0.0f);
}

TEST(Float16, NaN)
{
    Float16_t f;
    f.set(std::numeric_limits<float>::quiet_NaN());
    EXPECT_TRUE(std::isnan(f.get()));
}

TEST(Float16, MaxValue)
{
    // float16 max is 65504
    Float16_t f;
    f.set(65504.0f);
    EXPECT_FLOAT_EQ(65504.0f, f.get());
}

TEST(Float16, Overflow)
{
    // Values beyond float16 max (65504) should saturate to infinity
    Float16_t f;
    f.set(1e10f);
    EXPECT_TRUE(std::isinf(f.get()));
}

AP_GTEST_MAIN()
