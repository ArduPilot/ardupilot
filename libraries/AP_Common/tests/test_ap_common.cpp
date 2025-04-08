#include <AP_gtest.h>

#include <AP_Common/AP_Common.h>

TEST(AP_Common, HexToUint8)
{
    uint8_t res;
    for (uint8_t test_value = '0', expected_res = 0; test_value < ':'; test_value++, expected_res++) {
        EXPECT_TRUE(hex_to_uint8(test_value, res));
        EXPECT_EQ(expected_res, res);
    }
    for (uint8_t test_value = 'A', expected_res = 10; test_value < 'G'; test_value++, expected_res++) {
        EXPECT_TRUE(hex_to_uint8(test_value, res));
        EXPECT_EQ(expected_res, res);
    }
    for (uint8_t test_value = 'a', expected_res = 10; test_value < 'g'; test_value++, expected_res++) {
        EXPECT_TRUE(hex_to_uint8(test_value, res));
        EXPECT_EQ(expected_res, res);
    }
    EXPECT_FALSE(hex_to_uint8('G', res));
    EXPECT_FALSE(hex_to_uint8('g', res));
    EXPECT_FALSE(hex_to_uint8(';', res));
    EXPECT_FALSE(hex_to_uint8('/', res));
    EXPECT_FALSE(hex_to_uint8('@', res));
    EXPECT_FALSE(hex_to_uint8('`', res));
}

TEST(AP_Common, BoundedInt32)
{
    EXPECT_TRUE(is_bounded_int32(1, 0, 2));
    EXPECT_FALSE(is_bounded_int32(3, 0, 2));
    EXPECT_FALSE(is_bounded_int32(-1, 0, 2));
    EXPECT_TRUE(is_bounded_int32(0, -1, 2));
    EXPECT_TRUE(is_bounded_int32(-1, -2, 0));
}

TEST(AP_Common, BitSet)
{
    uint16_t test_value1 = 128;
    BIT_SET(test_value1, 3);
    EXPECT_EQ(test_value1, 136u);
    BIT_CLEAR(test_value1, 7);
    EXPECT_EQ(test_value1, 8u);

    uint32_t test_value2 = 128;
    BIT_SET(test_value2, 3);
    EXPECT_EQ(test_value2, 136u);
    BIT_CLEAR(test_value2, 7);
    EXPECT_EQ(test_value2, 8u);

    unsigned long test_value3 = 128;
    BIT_SET(test_value3, 3);
    EXPECT_EQ(test_value3, 136u);
    BIT_CLEAR(test_value3, 7);
    EXPECT_EQ(test_value3, 8u);
}


TEST(AP_Common, StrcpyNoTerm)
{
    const char src[] = "This is ArduPilot";
    char dest[ARRAY_SIZE(src)-5]{};

    strncpy_noterm(dest, src, 12);
    EXPECT_STRNE(dest, src);
    EXPECT_STREQ("This is Ardu", dest);

    const char src2[] = "ArduPilot";
    char dest2[ARRAY_SIZE(src)-5]{};
    strncpy_noterm(dest2, src2, 12);
    EXPECT_STREQ(dest2, src2);
}
AP_GTEST_MAIN()
