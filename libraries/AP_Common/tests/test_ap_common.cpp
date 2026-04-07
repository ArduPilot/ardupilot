#include <AP_gtest.h>

#include <AP_Common/AP_Common.h>

TEST(AP_Common, HexCharToNibble)
{
    uint8_t res;
    for (uint8_t test_value = '0', expected_res = 0; test_value < ':'; test_value++, expected_res++) {
        EXPECT_TRUE(hex_char_to_nibble(test_value, res));
        EXPECT_EQ(expected_res, res);
    }
    for (uint8_t test_value = 'A', expected_res = 10; test_value < 'G'; test_value++, expected_res++) {
        EXPECT_TRUE(hex_char_to_nibble(test_value, res));
        EXPECT_EQ(expected_res, res);
    }
    for (uint8_t test_value = 'a', expected_res = 10; test_value < 'g'; test_value++, expected_res++) {
        EXPECT_TRUE(hex_char_to_nibble(test_value, res));
        EXPECT_EQ(expected_res, res);
    }
    EXPECT_FALSE(hex_char_to_nibble('G', res));
    EXPECT_FALSE(hex_char_to_nibble('g', res));
    EXPECT_FALSE(hex_char_to_nibble(';', res));
    EXPECT_FALSE(hex_char_to_nibble('/', res));
    EXPECT_FALSE(hex_char_to_nibble('@', res));
    EXPECT_FALSE(hex_char_to_nibble('`', res));
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
TEST(AP_Common, HexTwocharsToUint8)
{
    uint8_t res;

    // basic known values
    EXPECT_TRUE(hex_twochars_to_uint8("00", res));  EXPECT_EQ(res, 0x00u);
    EXPECT_TRUE(hex_twochars_to_uint8("FF", res));  EXPECT_EQ(res, 0xFFu);
    EXPECT_TRUE(hex_twochars_to_uint8("3f", res));  EXPECT_EQ(res, 0x3Fu);
    EXPECT_TRUE(hex_twochars_to_uint8("A5", res));  EXPECT_EQ(res, 0xA5u);
    EXPECT_TRUE(hex_twochars_to_uint8("a5", res));  EXPECT_EQ(res, 0xA5u);

    // invalid first character
    EXPECT_FALSE(hex_twochars_to_uint8("G0", res));
    // invalid second character
    EXPECT_FALSE(hex_twochars_to_uint8("0G", res));
}

TEST(AP_Common, HexCharpairsToUint8s)
{
    uint8_t out[4];

    // basic three-byte decode
    EXPECT_TRUE(hex_charpairs_to_uint8s("0102FF", 3, out));
    EXPECT_EQ(out[0], 0x01u);
    EXPECT_EQ(out[1], 0x02u);
    EXPECT_EQ(out[2], 0xFFu);

    // mixed case
    EXPECT_TRUE(hex_charpairs_to_uint8s("fFaA", 2, out));
    EXPECT_EQ(out[0], 0xFFu);
    EXPECT_EQ(out[1], 0xAAu);

    // zero pairs — trivially succeeds
    EXPECT_TRUE(hex_charpairs_to_uint8s("", 0, out));

    // invalid character in second pair
    EXPECT_FALSE(hex_charpairs_to_uint8s("01GG", 2, out));
}

TEST(AP_Common, HexCharsToUint32)
{
    uint32_t out;

    // four-char example from the docstring
    EXPECT_TRUE(hex_chars_to_uint32("1A2B", 4, out));
    EXPECT_EQ(out, 0x1A2Bu);

    // three chars (SLCAN standard-frame ID width)
    EXPECT_TRUE(hex_chars_to_uint32("FFF", 3, out));
    EXPECT_EQ(out, 0xFFFu);

    // eight chars (SLCAN extended-frame ID width)
    EXPECT_TRUE(hex_chars_to_uint32("12345678", 8, out));
    EXPECT_EQ(out, 0x12345678u);

    // single char
    EXPECT_TRUE(hex_chars_to_uint32("A", 1, out));
    EXPECT_EQ(out, 0xAu);

    // lowercase
    EXPECT_TRUE(hex_chars_to_uint32("deadbeef", 8, out));
    EXPECT_EQ(out, 0xDEADBEEFu);

    // invalid character
    EXPECT_FALSE(hex_chars_to_uint32("1G2B", 4, out));
}

AP_GTEST_MAIN()
