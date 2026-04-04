#include <AP_gtest.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/Util.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define streq(x, y) (!strcmp(x, y))

// from the GNU snprintf manpage:

       // The functions snprintf() and vsnprintf() do not write  more  than  size
       // bytes  (including the terminating null byte ('\0')).  If the output was
       // truncated due to this limit, then the return value  is  the  number  of
       // characters  (excluding the terminating null byte) which would have been
       // written to the final string if enough space had been available.   Thus,
       // a  return  value  of  size or more means that the output was truncated.

TEST(vsnprintf_Test, Basic)
{
    char output[300];
    {
        int bytes_required = hal.util->snprintf(output, ARRAY_SIZE(output), "Fred: %u", 37);
        EXPECT_TRUE(streq(output, "Fred: 37"));
        EXPECT_EQ(bytes_required, 8);
    }
    {
        int bytes_required = hal.util->snprintf(output, 3, "Fred: %u", 37);
        EXPECT_TRUE(streq(output, "Fr"));
        EXPECT_EQ(bytes_required, 8);
    }
    {
        snprintf(output, ARRAY_SIZE(output), "0123");
        int bytes_required = hal.util->snprintf(output, 0, "Fred: %u", 37);
        EXPECT_TRUE(streq(output, "0123"));
        EXPECT_EQ(bytes_required, 8);
    }
    { // ensure rest of buffer survives
        memset(output, 'A', 10);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
        const int bytes_required = snprintf(output, 5, "012345678");
#pragma GCC diagnostic pop
        EXPECT_TRUE(streq(output, "0123"));
        EXPECT_EQ(bytes_required, 9);
        EXPECT_EQ(output[6], 'A');
    }
    { // simple float
        const int bytes_required = hal.util->snprintf(output, ARRAY_SIZE(output), "%f", 1/3.0);
        EXPECT_EQ(bytes_required, 8);
        EXPECT_TRUE(streq(output, "0.333333"));
    }
    { // less simple float
        const int bytes_required = hal.util->snprintf(output, ARRAY_SIZE(output), "%30.9f", 1/3.0);
        EXPECT_EQ(bytes_required, 28);
        EXPECT_TRUE(streq(output, "                   0.3333333"));
    }

    { // simple string
        const int bytes_required = hal.util->snprintf(output, ARRAY_SIZE(output), "%s %s %c", "ABC", "DEF", 'x');
        EXPECT_EQ(bytes_required, 9);
        EXPECT_TRUE(streq(output, "ABC DEF x"));
    }
}

static const char* do_subnormal_format(uint32_t val_hex) {
    // format float represented as a hex number long enough to see all digits

    // note that something else is wrong here and all the strings should be the
    // same width of 99 chars (or whatever the format string means)! when that
    // is fixed, update the test and add an assert here.

    static char buf[256];

    float val;
    static_assert(sizeof(uint32_t) == sizeof(float));
    memcpy(&val, &val_hex, sizeof(float));

    hal.util->snprintf(buf, ARRAY_SIZE(buf), "%.99f", val);

    return buf;
}

TEST(vsnprintf_Test, SubnormalFormat)
{
    // arbitrary small number (1e-31)
    // EXPECT_STREQ("0.000000000000000000000000000000099999998000000000000000000000000000000000000000000000000000000000000",
    EXPECT_STREQ("0.0000000000000000000000000000001000000",
        do_subnormal_format(0x0C01CEB3));

    // smallest normal (1.1754944e-38)
    // EXPECT_STREQ("0.000000000000000000000000000000000000011754944000000000000000000000000000000000000000000000000000000",
    EXPECT_STREQ("0.00000000000000000000000000000000000001175494",
        do_subnormal_format(0x00800000));

    // largest subnormal (1.1754942e-38)
    // EXPECT_STREQ("0.000000000000000000000000000000000000011754942000000000000000000000000000000000000000000000000000000",
    EXPECT_STREQ("0.00000000000000000000000000000000000001175494", // !! same as above
        do_subnormal_format(0x007FFFFF));

    // moderate subnormal (1.1478e-41)
    // EXPECT_STREQ("0.000000000000000000000000000000000000000011478036000000000000000000000000000000000000000000000000000",
    EXPECT_STREQ("0.00000000000000000000000000000000000000001147804",
        do_subnormal_format(0x00001FFF));

    // smallest subnormal (1e-45)
    // EXPECT_STREQ("0.000000000000000000000000000000000000000000001401290000000000000000000000000000000000000000000000000",
    EXPECT_STREQ("0.00000000000000000000000000000000000000000000140130",
        do_subnormal_format(0x00000001));

    // zero
    EXPECT_STREQ("0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000",
        do_subnormal_format(0x00000000));
}


AP_GTEST_MAIN()
