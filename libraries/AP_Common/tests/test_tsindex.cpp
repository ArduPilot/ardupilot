#include <AP_gtest.h>

#include <AP_Common/TSIndex.h>

TEST(TSIndex, operators)
{
    DECLARE_TYPESAFE_INDEX(testTSIndex, uint8_t);

    testTSIndex test_value = testTSIndex();
    EXPECT_EQ(0, test_value.get_int());
    uint8_t test_eq = 1;
    test_value = 1;
    EXPECT_EQ(test_eq, test_value.get_int());
    EXPECT_EQ(test_eq++, (test_value++).get_int());
    EXPECT_EQ(++test_eq, (++test_value).get_int());
    test_eq = 1;
    EXPECT_EQ(test_eq, (test_value % 2).get_int());
    test_eq = 10;
    EXPECT_TRUE(test_value < test_eq);
    test_eq = 3;
    EXPECT_TRUE(test_value <= test_eq);
    test_eq = 2;
    EXPECT_TRUE(test_value >= test_eq);
    test_eq = 1;
    EXPECT_TRUE(test_value > test_eq);
    test_eq = 2;
    EXPECT_TRUE(test_value != test_eq);
    test_eq = 3;
    EXPECT_TRUE(test_value == test_eq);
    test_eq = 4;
    EXPECT_EQ(test_eq, (test_value + 1).get_int());
    test_eq = 3;
    EXPECT_EQ(test_eq, uint8_t(test_value));
}

AP_GTEST_MAIN()
