#include <AP_gtest.h>

#include <Filter/Filter.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(ModeFilterTest, Int16_Size5)
{
    {
        // 5-entry filter taking the middle element by magnitude of
        // the last 5 samples:
        ModeFilterInt16_Size5 filt{2};
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(0));

        EXPECT_EQ(0, filt.apply(5));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(10));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(10));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(10));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(10));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(10));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(10));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(10));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(10));
        EXPECT_EQ(0, filt.apply(0));
        EXPECT_EQ(0, filt.apply(10));
    }
    ModeFilterInt16_Size5 filtf_fail{8};
    EXPECT_EQ(1, filtf_fail.apply(1));
    EXPECT_EQ(3, filtf_fail.apply(3));
    EXPECT_EQ(2, filtf_fail.apply(2));
    EXPECT_EQ(3, filtf_fail.apply(4));
    EXPECT_EQ(3, filtf_fail.apply(5));
    EXPECT_EQ(4, filtf_fail.apply(6));
    EXPECT_EQ(4, filtf_fail.apply(7));
    EXPECT_EQ(5, filtf_fail.apply(8));

    EXPECT_EQ(5, filtf_fail.get());
}

TEST(ModeFilterTest, Float_Size5)
{
    {
        // 5-entry filter taking the middle element by magnitude of
        // the last 5 samples:
        ModeFilterFloat_Size5 filt{2};
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(0));

        EXPECT_FLOAT_EQ(0, filt.apply(5));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(10));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(10));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(10));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(10));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(10));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(10));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(10));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(10));
        EXPECT_FLOAT_EQ(0, filt.apply(0));
        EXPECT_FLOAT_EQ(0, filt.apply(10));
    }
    ModeFilterFloat_Size5 filtf_fail{8};
    EXPECT_FLOAT_EQ(1, filtf_fail.apply(1));
    EXPECT_FLOAT_EQ(3, filtf_fail.apply(3));
    EXPECT_FLOAT_EQ(2, filtf_fail.apply(2));
    EXPECT_FLOAT_EQ(3, filtf_fail.apply(4));
    EXPECT_FLOAT_EQ(3, filtf_fail.apply(5));
    EXPECT_FLOAT_EQ(4, filtf_fail.apply(6));
    EXPECT_FLOAT_EQ(4, filtf_fail.apply(7));
    EXPECT_FLOAT_EQ(5, filtf_fail.apply(8));

    EXPECT_FLOAT_EQ(5, filtf_fail.get());
}

AP_GTEST_MAIN()
