#include <AP_gtest.h>

#include <Filter/Filter.h>




TEST(AverageFilterTest, UInt16_Size5)
{
    AverageFilterUInt16_Size4 test_filter;
    const uint8_t size = 4;
    const uint8_t test_value = 5;

    EXPECT_EQ(size, test_filter.get_filter_size());
    uint8_t i = 0;
    for (; i < size + 2; i++) {
        EXPECT_EQ(test_value, test_filter.apply(test_value));
    }
    EXPECT_EQ(test_value * 2, test_filter.apply(test_value * (test_filter.get_filter_size() + 1)));
    EXPECT_EQ(test_value, test_filter.get_sample((i % size) - 1));
    EXPECT_EQ(test_value * (test_filter.get_filter_size() + 1), test_filter.get_sample(i % size));
    test_filter.reset();
    EXPECT_EQ(test_value, test_filter.apply(test_value));
}

TEST(AverageFilterTest, Float_Size5)
{
    AverageFilterFloat_Size5 test_filter;
    const uint8_t size = 5;
    const float test_value = 5;

    EXPECT_FLOAT_EQ(5, test_filter.get_filter_size());
    uint8_t i = 0;
    for (; i < size + 2; i++) {
        EXPECT_FLOAT_EQ(test_value, test_filter.apply(test_value));
    }
    EXPECT_FLOAT_EQ(test_value * 2, test_filter.apply(test_value * (test_filter.get_filter_size() + 1)));
    EXPECT_FLOAT_EQ(test_value, test_filter.get_sample((i % size) - 1));
    EXPECT_FLOAT_EQ(test_value * (test_filter.get_filter_size() + 1), test_filter.get_sample(2));
    test_filter.reset();
    EXPECT_FLOAT_EQ(test_value, test_filter.apply(test_value));

}

AP_GTEST_MAIN()
