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

TEST(AverageIntegralFilterTest, UInt16_Size5)
{
    AverageIntegralFilter<uint16_t, uint32_t, 5> test_filter;
    // filter should be zero even on stack (this may not pick up bugs, but may):
    EXPECT_FLOAT_EQ(0, test_filter.getf());
    EXPECT_FLOAT_EQ(0, test_filter.getd());

    EXPECT_EQ(0, test_filter.apply(1));  // apply always returns 0
    EXPECT_FLOAT_EQ(1, test_filter.getf());

    EXPECT_EQ(0, test_filter.apply(3));  // apply always returns 0
    EXPECT_FLOAT_EQ(2, test_filter.getf());

    EXPECT_EQ(0, test_filter.apply(2));  // apply always returns 0
    EXPECT_FLOAT_EQ(2, test_filter.getf());

    EXPECT_EQ(0, test_filter.apply(4));  // apply always returns 0
    EXPECT_FLOAT_EQ(2.5, test_filter.getf());

    EXPECT_EQ(0, test_filter.apply(100));  // apply always returns 0
    EXPECT_FLOAT_EQ(22, test_filter.getf());

    EXPECT_EQ(0, test_filter.apply(110));  // apply always returns 0
    EXPECT_FLOAT_EQ(43.799999, test_filter.getf());
    EXPECT_FLOAT_EQ(43.799999, test_filter.getd());
}

AP_GTEST_MAIN()
