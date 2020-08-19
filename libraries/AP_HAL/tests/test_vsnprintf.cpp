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

AP_GTEST_MAIN()
