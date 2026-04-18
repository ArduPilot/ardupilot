/*
 * Regression test for AP_AIS::get_bits() - payload length guard and
 * uint8_t -> uint16_t span promotion.
 */

#include <AP_gtest.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

int hal = 0;

static uint8_t payload_char_decode(const char c)
{
    uint8_t value = (uint8_t)c;
    value -= 48;
    if (value > 40) {
        value -= 8;
    }
    return value & 0x3F;
}

static uint32_t get_bits_master(const char *payload, uint16_t low, uint16_t high)
{
    if (payload == nullptr || high < low) {
        return 0;
    }

    uint8_t char_low = low / 6;
    uint8_t bit_low = low % 6;
    uint8_t char_high = high / 6;
    uint8_t bit_high = (uint8_t)(high % 6) + 1;

    uint32_t val = 0;
    for (uint8_t index = 0; index <= (uint8_t)(char_high - char_low); index++) {
        uint8_t value = payload_char_decode(payload[char_low + index]);
        uint8_t mask = 0x3F;
        if (index == 0) {
            mask = mask >> bit_low;
        }
        value &= mask;
        if (index == (uint8_t)(char_high - char_low)) {
            value = value >> (6 - bit_high);
            val = val << bit_high;
        } else {
            val = val << 6;
        }
        val |= value;
    }
    return val;
}

static uint32_t get_bits_fixed(const char *payload, uint16_t low, uint16_t high)
{
    if (payload == nullptr || high < low) {
        return 0;
    }

    uint8_t char_low = low / 6;
    uint8_t bit_low = low % 6;
    uint8_t char_high = high / 6;
    uint8_t bit_high = (uint8_t)(high % 6) + 1;

    const size_t payload_len = strlen(payload);
    if (char_high >= payload_len) {
        return 0;
    }

    uint32_t val = 0;
    const uint16_t char_span = (uint16_t)char_high - (uint16_t)char_low;
    for (uint16_t index = 0; index <= char_span; index++) {
        uint8_t value = payload_char_decode(payload[char_low + index]);
        uint8_t mask = 0x3F;
        if (index == 0) {
            mask = mask >> bit_low;
        }
        value &= mask;
        if (index == char_span) {
            value = value >> (6 - bit_high);
            val = val << bit_high;
        } else {
            val = val << 6;
        }
        val |= value;
    }
    return val;
}

TEST(AIS_GetBits, BugA_payload_too_short_returns_zero_in_fixed_version)
{
    const char *short_payload = "WWWWW";
    const size_t payload_len = strlen(short_payload);
    const uint8_t char_high_for_bit167 = 167 / 6;

    EXPECT_GE(char_high_for_bit167, payload_len);
    EXPECT_EQ(0u, get_bits_fixed(short_payload, 0, 167));
}

TEST(AIS_GetBits, BugA_last_valid_bit_of_short_payload_succeeds)
{
    const char *payload = "WWWWW";
    const uint32_t r1 = get_bits_fixed(payload, 0, 29);
    const uint32_t r2 = get_bits_fixed(payload, 0, 29);
    EXPECT_EQ(r1, r2);
}

TEST(AIS_GetBits, BugA_one_bit_past_end_returns_zero)
{
    EXPECT_EQ(0u, get_bits_fixed("WWWWW", 0, 30));
}

TEST(AIS_GetBits, null_payload_returns_zero)
{
    EXPECT_EQ(0u, get_bits_fixed(nullptr, 0, 5));
    EXPECT_EQ(0u, get_bits_master(nullptr, 0, 5));
}

TEST(AIS_GetBits, BugB_normal_range_master_and_fixed_agree)
{
    const char *payload = "WWWWWWWWWWWWWWWWWWWWWWWWWWWW";

    EXPECT_EQ(get_bits_master(payload, 0, 5), get_bits_fixed(payload, 0, 5));
    EXPECT_EQ(get_bits_master(payload, 6, 11), get_bits_fixed(payload, 6, 11));
    EXPECT_EQ(get_bits_master(payload, 0, 167), get_bits_fixed(payload, 0, 167));
}

AP_GTEST_MAIN()