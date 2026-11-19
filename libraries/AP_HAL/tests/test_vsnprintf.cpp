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
//Main loop where the action takes place
#pragma GCC diagnostic push
#if defined(__clang_major__)
// clang doesn't understand -Wformat-truncation
#else
#pragma GCC diagnostic ignored "-Wformat-truncation"
        memset(output, 'A', 10);
        const int bytes_required = snprintf(output, 5, "012345678");
        EXPECT_TRUE(streq(output, "0123"));
        EXPECT_EQ(bytes_required, 9);
        EXPECT_EQ(output[6], 'A');
#endif
#pragma GCC diagnostic pop
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

static const char* do_special_format(uint32_t val_hex) {
    // format float represented as a hex number

    static char buf[256];

    float val;
    static_assert(sizeof(uint32_t) == sizeof(float));
    memcpy(&val, &val_hex, sizeof(float));

    hal.util->snprintf(buf, ARRAY_SIZE(buf), "%f", val);

    return buf;
}

TEST(vsnprintf_Test, SpecialFormat)
{
    // positive infinity
    EXPECT_STREQ("inf", do_special_format(0x7F800000));

    // negative infinity
    EXPECT_STREQ("-inf", do_special_format(0xFF800000));

    // signaling NaN with arbitrary payload A
    EXPECT_STREQ("nan", do_special_format(0x7FF00800));

    // signaling NaN with arbitrary payload B
    EXPECT_STREQ("nan", do_special_format(0x7FF01230));

    // quiet NaN with arbitrary payload A
    EXPECT_STREQ("nan", do_special_format(0x7FB00800));

    // quiet NaN with arbitrary payload B
    EXPECT_STREQ("nan", do_special_format(0x7FB01230));

    // negative signaling NaN with arbitrary payload A
    EXPECT_STREQ("nan", do_special_format(0xFFF00800));

    // negative signaling NaN with arbitrary payload B
    EXPECT_STREQ("nan", do_special_format(0xFFF01230));

    // negative quiet NaN with arbitrary payload A
    EXPECT_STREQ("nan", do_special_format(0xFFB00800));

    // negative quiet NaN with arbitrary payload B
    EXPECT_STREQ("nan", do_special_format(0xFFB01230));

    // largest NaN payload
    EXPECT_STREQ("nan", do_special_format(0x7FFFFFFF));

    // smallest NaN payload
    EXPECT_STREQ("nan", do_special_format(0x7F800001));
}


#if AP_MODULE_ENABLED
TEST(vsnprintf_Test, TypedArguments)
{
    using Arg = AP_HAL::Util::FormatArg;
    Arg args[8]{};
    args[0].type = Arg::Type::INT32;
    args[0].i32 = -7;
    args[1].type = Arg::Type::FLOAT64;
    args[1].real = 12.5;
    args[2].type = Arg::Type::STRING;
    args[2].text = "abc";
    args[3].type = Arg::Type::INT64;
    args[3].i64 = -1234567890123LL;
    args[4].type = Arg::Type::UINT64;
    args[4].u64 = UINT64_MAX;
    args[5].type = Arg::Type::UINT32;
    args[5].u32 = 0xdeadbeef;
    args[6].type = Arg::Type::FLOAT64;
    args[6].real = 0.25;
    args[7].type = Arg::Type::INT32;
    args[7].i32 = 'A';
    const char *format = "%d %.1f %s %lld %llu %08X %.2f %c %%";
    char actual[256], expected[256];
    const int length = hal.util->snprintf(expected, sizeof(expected), format,
                                        -7, 12.5, "abc", -1234567890123LL,
                                        static_cast<unsigned long long>(UINT64_MAX), 0xdeadbeefU, 0.25, 'A');
    EXPECT_EQ(length, hal.util->snprintf_args(actual, sizeof(actual), format, args, 8));
    EXPECT_STREQ(actual, expected);
    EXPECT_EQ(length, hal.util->snprintf_args(nullptr, 0, format, args, 8));
    EXPECT_EQ(length, hal.util->snprintf_args(actual, 4, format, args, 8));
    EXPECT_STREQ(actual, "-7 ");
}

TEST(vsnprintf_Test, TypedArgumentsScalarSlots)
{
    using Arg = AP_HAL::Util::FormatArg;
    Arg args[8]{};
    args[0].type = Arg::Type::INT32;
    args[0].i32 = -1;
    args[1].type = Arg::Type::UINT32;
    args[1].u32 = UINT32_MAX;
    args[2].type = Arg::Type::INT32;
    args[2].i32 = 'Z';
    args[3].type = Arg::Type::STRING;
    args[3].text = "slots";
    args[4].type = Arg::Type::INT32;
    args[4].i32 = INT32_MIN;
    args[5].type = Arg::Type::INT64;
    args[5].i64 = INT64_MIN;
    args[6].type = Arg::Type::UINT64;
    args[6].u64 = UINT64_MAX;
    args[7].type = Arg::Type::FLOAT64;
    args[7].real = -0.5;
    const char *format = "%i %u %c %s %i %lld %llu %.1f";
    char actual[256], expected[256];
    const int length = hal.util->snprintf(expected, sizeof(expected), format,
                                        -1, UINT32_MAX, 'Z', "slots", INT32_MIN,
                                        static_cast<long long>(INT64_MIN),
                                        static_cast<unsigned long long>(UINT64_MAX), -0.5);
    EXPECT_EQ(length, hal.util->snprintf_args(actual, sizeof(actual), format, args, 8));
    EXPECT_STREQ(actual, expected);
}

TEST(vsnprintf_Test, TypedArgumentsFloatSlots)
{
    using Arg = AP_HAL::Util::FormatArg;
    Arg args[8]{};
    for (uint8_t i = 0; i < ARRAY_SIZE(args); i++) {
        args[i].type = Arg::Type::FLOAT64;
        args[i].real = (i - 4) * 0.25;
    }
    const char *format = "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f";
    char actual[128], expected[128];
    const int length = hal.util->snprintf(expected, sizeof(expected), format,
                                        -1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75);
    EXPECT_EQ(length, hal.util->snprintf_args(actual, sizeof(actual), format, args, ARRAY_SIZE(args)));
    EXPECT_STREQ(actual, expected);
}

TEST(vsnprintf_Test, TypedArgumentsRejectInvalidFormats)
{
    using Arg = AP_HAL::Util::FormatArg;
    Arg arg{};
    arg.type = Arg::Type::FLOAT64;
    arg.real = 12.5;
    const char *invalid[] = {"%d", "%s", "%n", "%p", "%*f", "%.*f", "%1$f", "%ld",
                             "%256f", "%.61f", "%llf", "%", "%l", "%ll", "%f %f", "unused"};
    char output[32] = "unchanged";
    for (const char *format : invalid) {
        EXPECT_EQ(-1, hal.util->snprintf_args(output, sizeof(output), format, &arg, 1)) << format;
        EXPECT_STREQ(output, "unchanged");
    }
    EXPECT_EQ(-1, hal.util->snprintf_args(output, sizeof(output), "%f", &arg, 0));
    EXPECT_EQ(-1, hal.util->snprintf_args(output, sizeof(output), "%f", &arg, 9));
    EXPECT_EQ(-1, hal.util->snprintf_args(output, sizeof(output), "%f", nullptr, 1));
    EXPECT_EQ(-1, hal.util->snprintf_args(output, sizeof(output), nullptr, &arg, 1));
    EXPECT_EQ(-1, hal.util->snprintf_args(nullptr, 1, "%f", &arg, 1));
    EXPECT_EQ(4, hal.util->snprintf_args(output, sizeof(output), "100%%", nullptr, 0));
    EXPECT_STREQ(output, "100%");
}
#endif // AP_MODULE_ENABLED

AP_GTEST_MAIN()
