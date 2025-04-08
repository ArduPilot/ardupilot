#include <AP_gtest.h>

#include <AP_Common/TSIndex.h>
DECLARE_TYPESAFE_INDEX(testTSIndex, uint8_t);

TEST(TSIndex, operators)
{
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

TEST(TSIndex, RestrictIDArray)
{

    testTSIndex i_0(0);
    testTSIndex i_1(1);
    RestrictIDTypeArray<int32_t , 2, testTSIndex> state{};

    EXPECT_EQ(state[i_0], 0);
    EXPECT_EQ(state[i_1], 0);
    state[i_1] = 42;
    EXPECT_EQ(state[i_1], 42);
    const int32_t state_1 = state[i_1];
    EXPECT_EQ(state_1, state[i_1]);
    EXPECT_NE(state_1, state[i_0]);
    const RestrictIDTypeArray<int32_t , 2, testTSIndex> state_const{42, 43};
    EXPECT_TRUE(state_const[i_0] == 42);
    EXPECT_TRUE(state_const[i_1] == 43);
}
AP_GTEST_MAIN()
