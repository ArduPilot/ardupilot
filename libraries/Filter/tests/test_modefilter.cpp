
// given we are in the Math library, you're epected to know what
// you're doing when directly comparing floats:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <AP_gtest.h>

#include <Filter/Filter.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(ModeFilterTest, Int16_Size5)
{
    {
        // 5-entry filter taking the middle element by magnitude of
        // the last 5 samples:
        ModeFilterInt16_Size5 filt{2};
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));

        EXPECT_FLOAT_EQ(0.0, filt.apply(5));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(10));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(10));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(10));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(10));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(10));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(10));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(10));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(10));
        EXPECT_FLOAT_EQ(0.0, filt.apply(0));
        EXPECT_FLOAT_EQ(0.0, filt.apply(10));
    }
}

AP_GTEST_MAIN()

#pragma GCC diagnostic pop
