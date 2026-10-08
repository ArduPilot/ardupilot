#include <AP_gtest.h>

#include <AP_Common/strtod.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <ctype.h>
#include <dlfcn.h>
#include <errno.h>
#include <fenv.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

// correct rounding needs exact floating point comparisons
#pragma GCC diagnostic ignored "-Wfloat-equal"

/*
  tests for ap_strtod() and ap_strtof(), against fixed values and against
  the C library
 */

typedef double (*strtod_fn)(const char *, char **);
typedef float (*strtof_fn)(const char *, char **);

// the C library versions, bypassing the SITL --wrap
static strtod_fn libc_strtod()
{
    return (strtod_fn)dlsym(RTLD_NEXT, "strtod");
}

static strtof_fn libc_strtof()
{
    return (strtof_fn)dlsym(RTLD_NEXT, "strtof");
}

static uint64_t bits(double v)
{
    uint64_t b;
    memcpy(&b, &v, sizeof(b));
    return b;
}

static uint32_t bits(float v)
{
    uint32_t b;
    memcpy(&b, &v, sizeof(b));
    return b;
}

// distance in units in the last place, for finite values of the same sign
static uint64_t ulps(double a, double b)
{
    const uint64_t x = bits(a), y = bits(b);
    return x > y ? x - y : y - x;
}

static uint32_t ulps(float a, float b)
{
    const uint32_t x = bits(a), y = bits(b);
    return x > y ? x - y : y - x;
}

struct Expected {
    const char *str;
    double value;
    int end;
};

TEST(AP_Common_strtod, KnownValues)
{
    const Expected cases[] = {
        { "0", 0, 1 },
        { "1", 1, 1 },
        { "1.5", 1.5, 3 },
        { ".5", 0.5, 2 },
        { "5.", 5, 2 },
        { "  \t\n42", 42, 6 },
        { "+3", 3, 2 },
        { "-3.25e2", -325, 7 },
        { "1e10", 1e10, 4 },
        { "1E-5", 1e-5, 4 },
        { "123.456e-7", 123.456e-7, 10 },
        { "3723.2475", 3723.2475, 9 },
        { "12233.1234", 12233.1234, 10 },
        { "0.000001", 0.000001, 8 },
        { "000123", 123, 6 },
        { "1.2500000", 1.25, 9 },
        { "1e22", 1e22, 4 },
        { "1e23", 1e23, 4 },
        { "9007199254740993", 9007199254740992.0, 16 },
        { "9007199254740995", 9007199254740996.0, 16 },
        { "1.7976931348623157e308", DBL_MAX, 22 },
        { "2.2250738585072014e-308", DBL_MIN, 23 },
        { "0.1", 0.1, 3 },
        { "0.3", 0.3, 3 },
        { "1e0005", 1e5, 6 },
        { "1e+5", 1e5, 4 },
        { "1.00000000000000000000000000001", 1, 31 },
    };
    for (const auto &c : cases) {
        char *end;
        errno = 0;
        const double d = ap_strtod(c.str, &end);
        EXPECT_EQ(bits(d), bits(c.value)) << c.str;
        EXPECT_EQ(end - c.str, c.end) << c.str;
        EXPECT_EQ(errno, 0) << c.str;

        errno = 0;
        const float f = ap_strtof(c.str, &end);
        if (fabs(c.value) <= DBL_MAX && fabs(c.value) <= FLT_MAX && (c.value == 0 || fabs(c.value) >= FLT_MIN)) {
            EXPECT_EQ(bits(f), bits(float(c.value))) << c.str;
            EXPECT_EQ(errno, 0) << c.str;
        }
        EXPECT_EQ(end - c.str, c.end) << c.str;
    }
}

TEST(AP_Common_strtod, FloatValues)
{
    EXPECT_EQ(bits(ap_strtof("3.4028235e38", nullptr)), bits(FLT_MAX));
    EXPECT_EQ(bits(ap_strtof("1.17549435e-38", nullptr)), bits(FLT_MIN));
    EXPECT_EQ(bits(ap_strtof("0.1", nullptr)), bits(0.1f));
    EXPECT_EQ(bits(ap_strtof("16777217", nullptr)), bits(16777216.0f));
    EXPECT_EQ(bits(ap_strtof("16777219", nullptr)), bits(16777220.0f));
    // either side of 1 + 2^-24, halfway between two floats. Both round to
    // exactly that as a double, so rounding via a double would tie
    EXPECT_EQ(bits(ap_strtof("1.000000059604644776", nullptr)), bits(1.00000012f));
    EXPECT_EQ(bits(ap_strtof("1.000000059604644775", nullptr)), bits(1.0f));
}

TEST(AP_Common_strtod, LimitedDecimalPrecision)
{
    // Even short decimal inputs can be close enough to a rounding midpoint
    // that limited double-double precision chooses an adjacent value.
    const struct {
        const char *str;
        double value;
    } cases[] = {
        { "93405643475160e217", 0x1.340a1c932c1eep+767 },
        { "9964716443911029e-261", 0x1.16b0a3a2cedd1p-814 },
        { "59178966397722867e-24", 0x1.fc57ec608ae6fp-25 },
        { "1473088984278055305e-25", 0x1.3c57ec608ae6fp-23 },
    };
    for (const auto &c : cases) {
        EXPECT_LE(ulps(ap_strtod(c.str, nullptr), c.value), 1U) << c.str;
        const std::string negative = std::string("-") + c.str;
        EXPECT_LE(ulps(ap_strtod(negative.c_str(), nullptr), -c.value), 1U) << negative;
    }
}

// with more than 19 significant digits a value within the dropped digits of
// halfway can round either way, so it is only checked to be a neighbour
TEST(AP_Common_strtod, LongHalfwayIsANeighbour)
{
    // exactly halfway between 1 and the next float up
    const float f = ap_strtof("1.000000059604644775390625", nullptr);
    EXPECT_TRUE(bits(f) == bits(1.0f) || bits(f) == bits(1.00000012f));
    // exactly halfway between 1 and the next double up
    const double d = ap_strtod("1.00000000000000011102230246251565404236316680908203125", nullptr);
    EXPECT_TRUE(bits(d) == bits(1.0) || bits(d) == bits(1.0000000000000002));
}

TEST(AP_Common_strtod, SignedZero)
{
    EXPECT_EQ(bits(ap_strtod("-0", nullptr)), bits(-0.0));
    EXPECT_EQ(bits(ap_strtod("-0.0e10", nullptr)), bits(-0.0));
    EXPECT_EQ(bits(ap_strtod("+0", nullptr)), bits(0.0));
    EXPECT_EQ(bits(ap_strtof("-0", nullptr)), bits(-0.0f));
    EXPECT_EQ(bits(ap_strtod("-1e-400", nullptr)), bits(-0.0));
}

TEST(AP_Common_strtod, InfAndNan)
{
    struct {
        const char *str;
        int end;
        bool nan;
        bool negative;
    } cases[] = {
        { "inf", 3, false, false },
        { "INF", 3, false, false },
        { "-Infinity", 9, false, true },
        { "infinityx", 8, false, false },
        { "infx", 3, false, false },
        { "infinit", 3, false, false },
        { "+inf", 4, false, false },
        { "nan", 3, true, false },
        { "NaN", 3, true, false },
        { "-nan", 4, true, true },
        { "nan(123)", 8, true, false },
        { "nan(a_Z9)", 9, true, false },
        { "nan()", 5, true, false },
        { "nan(12", 3, true, false },
        { "nan(1-2)", 3, true, false },
    };
    for (const auto &c : cases) {
        char *end;
        errno = 0;
        const double d = ap_strtod(c.str, &end);
        EXPECT_EQ(end - c.str, c.end) << c.str;
        EXPECT_EQ(isnan(d), c.nan) << c.str;
        EXPECT_EQ(isinf(d), !c.nan) << c.str;
        EXPECT_EQ(signbit(d) != 0, c.negative) << c.str;
        EXPECT_EQ(errno, 0) << c.str;
        const float f = ap_strtof(c.str, &end);
        EXPECT_EQ(end - c.str, c.end) << c.str;
        EXPECT_EQ(isnan(f), c.nan) << c.str;
        EXPECT_EQ(signbit(f) != 0, c.negative) << c.str;
    }
}

TEST(AP_Common_strtod, FloatingPointExceptions)
{
    fenv_t saved;
    ASSERT_EQ(feholdexcept(&saved), 0);
    const int trapped = FE_OVERFLOW | FE_DIVBYZERO | FE_INVALID;
    for (int exponent = 256; exponent <= 360; exponent++) {
        char s[32];
        snprintf(s, sizeof(s), "1e-%d", exponent);
        feclearexcept(FE_ALL_EXCEPT);
        ap_strtod(s, nullptr);
        EXPECT_EQ(fetestexcept(trapped), 0) << s;
        if (exponent <= 308) {
            snprintf(s, sizeof(s), "1e%d", exponent);
            feclearexcept(FE_ALL_EXCEPT);
            ap_strtod(s, nullptr);
            EXPECT_EQ(fetestexcept(trapped), 0) << s;
        }
    }
    const char *nans[] = { "nan", "-nan", "nan(payload)" };
    for (const char *s : nans) {
        feclearexcept(FE_ALL_EXCEPT);
        EXPECT_TRUE(isnan(ap_strtod(s, nullptr)));
        EXPECT_EQ(fetestexcept(trapped), 0) << s;
        feclearexcept(FE_ALL_EXCEPT);
        EXPECT_TRUE(isnan(ap_strtof(s, nullptr)));
        EXPECT_EQ(fetestexcept(trapped), 0) << s;
    }
    const char *hex_underflows[] = {
        "0x1p-2097", "0x1p-2098", "-0x1p-3000", "0x0p-3000",
        "0x1p-999999", "-0x0p-999999",
    };
    for (const char *s : hex_underflows) {
        feclearexcept(FE_ALL_EXCEPT);
        const double d = ap_strtod(s, nullptr);
        EXPECT_EQ(bits(d), bits(s[0] == '-' ? -0.0 : 0.0)) << s;
        EXPECT_EQ(fetestexcept(trapped), 0) << s;
        feclearexcept(FE_ALL_EXCEPT);
        const float f = ap_strtof(s, nullptr);
        EXPECT_EQ(bits(f), bits(s[0] == '-' ? -0.0f : 0.0f)) << s;
        EXPECT_EQ(fetestexcept(trapped), 0) << s;
    }
    EXPECT_EQ(fesetenv(&saved), 0);
}

TEST(AP_Common_strtod, NoConversion)
{
    const char *cases[] = { "", "   ", "abc", ".", "+", "-", "+.", "-.", "e5", ".e5", "-x", "in", "na", "\x01" "1" };
    const int errors[] = { 0, EDOM, ERANGE };
    for (const char *s : cases) {
        for (const int initial_errno : errors) {
            char *end = nullptr;
            errno = initial_errno;
            EXPECT_EQ(bits(ap_strtod(s, &end)), bits(0.0)) << s;
            EXPECT_EQ(end, s) << s;
            EXPECT_EQ(errno, initial_errno) << s;
            errno = initial_errno;
            EXPECT_EQ(bits(ap_strtof(s, &end)), bits(0.0f)) << s;
            EXPECT_EQ(end, s) << s;
            EXPECT_EQ(errno, initial_errno) << s;
        }
    }
}

TEST(AP_Common_strtod, PartialParse)
{
    const Expected cases[] = {
        { "1e", 1, 1 },
        { "1e+", 1, 1 },
        { "1e-x", 1, 1 },
        { "1.2.3", 1.2, 3 },
        { "12abc", 12, 2 },
        { "1,5", 1, 1 },
        { "0x", 0, 1 },
        { "0xg", 0, 1 },
        { "0x.p1", 0, 1 },
        { "-0x", -0.0, 2 },
        { "1p5", 1, 1 },
        { "0x1e5", 0x1e5, 5 },
        { "0x1p", 1, 3 },
        { "0x1p+", 1, 3 },
        { "4.5 ", 4.5, 3 },
        { "4.5\n7", 4.5, 3 },
        { "10*", 10, 2 },
    };
    const int errors[] = { 0, EDOM, ERANGE };
    for (const auto &c : cases) {
        for (const int initial_errno : errors) {
            char *end;
            errno = initial_errno;
            EXPECT_EQ(bits(ap_strtod(c.str, &end)), bits(c.value)) << c.str;
            EXPECT_EQ(end - c.str, c.end) << c.str;
            EXPECT_EQ(errno, initial_errno) << c.str;
            errno = initial_errno;
            EXPECT_EQ(bits(ap_strtof(c.str, &end)), bits(float(c.value))) << c.str;
            EXPECT_EQ(end - c.str, c.end) << c.str;
            EXPECT_EQ(errno, initial_errno) << c.str;
        }
    }
}

TEST(AP_Common_strtod, Hex)
{
    const Expected cases[] = {
        { "0x10", 16, 4 },
        { "0X1p4", 16, 5 },
        { "0x1.8p1", 3, 7 },
        { "0x.8", 0.5, 4 },
        { "-0x1p-2", -0.25, 7 },
        { "0xABCDEF", 0xABCDEF, 8 },
        { "0x1.fffffffffffffp1023", DBL_MAX, 22 },
        { "0x1p-1022", DBL_MIN, 9 },
        { "0x1p-1074", 4.9406564584124654e-324, 9 },
        { "0x0.0000000000001p-1022", 4.9406564584124654e-324, 23 },
        { "0x123456789abcdef0123p0", 0x123456789abcdef0123p0, 23 },
        { "0x1.000000000000080000000001p0", 1.0000000000000002, 30 },
        { "0x1.00000000000008p0", 1, 20 },
        { "0x1.00000000000018p0", 1.0000000000000004, 20 },
    };
    for (const auto &c : cases) {
        char *end;
        errno = 0;
        EXPECT_EQ(bits(ap_strtod(c.str, &end)), bits(c.value)) << c.str;
        EXPECT_EQ(end - c.str, c.end) << c.str;
        EXPECT_EQ(errno, 0) << c.str;
    }
}

TEST(AP_Common_strtod, Range)
{
    struct {
        const char *str;
        double value;
        bool range_error;
    } cases[] = {
        { "1e309", INFINITY, true },
        { "-1e309", -INFINITY, true },
        { "1e-400", 0, true },
        { "1e-310", 1e-310, true },
        { "-0X7B6CEc.aFe62152p-1047", -0x0.3db67657f310bp-1022, true },
        { "2.2250738585072011e-308", nextafter(DBL_MIN, 0), true },
        { "-2.2250738585072011e-308", -nextafter(DBL_MIN, 0), true },
        { "2.2250738585072012e-308", DBL_MIN, false },
        { "-2.2250738585072012e-308", -DBL_MIN, false },
        { "0x1p-2098", 0, true },
        { "-0x1p-3000", -0.0, true },
        { "0x0p-3000", 0, false },
        { "-0x0p-999999", -0.0, false },
        { "0x1p-1075", 0, true },
        { "0x1.8p-1074", 2 * 4.9406564584124654e-324, true },
        { "0x1p-1074", 4.9406564584124654e-324, false },
        { "0x1p1024", INFINITY, true },
        { "0x1p100000", INFINITY, true },
        { "-0x1p100000", -INFINITY, true },
        { "0x0p100000", 0, false },
        { "1.7976931348623159e308", INFINITY, true },
        { "0e-1000", 0, false },
        { "0e1000", 0, false },
        { "1e99999999999999999999", INFINITY, true },
        { "1e-99999999999999999999", 0, true },
    };
    for (const auto &c : cases) {
        errno = 0;
        EXPECT_EQ(bits(ap_strtod(c.str, nullptr)), bits(c.value)) << c.str;
        EXPECT_EQ(errno == ERANGE, c.range_error) << c.str;
    }

    struct {
        const char *str;
        float value;
        bool range_error;
    } float_cases[] = {
        { "3.5e38", INFINITY, true },
        { "3.4028235677973366e38", FLT_MAX, false },
        { "-3.4028235677973366e38", -FLT_MAX, false },
        { "3.4028235677973367e38", INFINITY, true },
        { "1.1754942807573642e-38", nextafterf(FLT_MIN, 0), true },
        { "1.1754942807573643e-38", FLT_MIN, false },
        { "-1.1754942807573643e-38", -FLT_MIN, false },
        { "-1e39", -INFINITY, true },
        { "1e-46", 0, true },
        { "1e-40", 1e-40f, true },
        { "0x1p-149", 1.4e-45f, false },
        { "0x1p-150", 0, true },
        { "0x1.8p-149", 2.8e-45f, true },
        { "1e300", INFINITY, true },
        { "1e-300", 0, true },
        { "0x1p100000", INFINITY, true },
        { "-0x1p100000", -INFINITY, true },
        { "0x0p100000", 0, false },
        { "0xaaB2388p-155", 0x1.55647p-128f, true },
    };
    for (const auto &c : float_cases) {
        errno = 0;
        EXPECT_EQ(bits(ap_strtof(c.str, nullptr)), bits(c.value)) << c.str;
        EXPECT_EQ(errno == ERANGE, c.range_error) << c.str;
    }
}

TEST(AP_Common_strtod, LongInputs)
{
    // a thousand digits
    std::string digits(1000, '7');
    errno = 0;
    EXPECT_TRUE(isinf(ap_strtod(digits.c_str(), nullptr)));
    EXPECT_EQ(errno, ERANGE);

    std::string fraction = "0." + digits;
    char *end;
    EXPECT_EQ(bits(ap_strtod(fraction.c_str(), &end)), bits(0.7777777777777778));
    EXPECT_EQ(end - fraction.c_str(), long(fraction.size()));

    // leading zeros don't use up significant digits
    std::string small = "0." + std::string(499, '0') + "1e500";
    EXPECT_EQ(bits(ap_strtod(small.c_str(), &end)), bits(1.0));
    EXPECT_EQ(end - small.c_str(), long(small.size()));

    std::string zeros = std::string(1000, '0') + "1";
    EXPECT_EQ(bits(ap_strtod(zeros.c_str(), nullptr)), bits(1.0));

    // trailing digits beyond the significant ones still scale the value
    std::string big = "1" + std::string(400, '0') + "e-400";
    EXPECT_EQ(bits(ap_strtod(big.c_str(), nullptr)), bits(1.0));

    EXPECT_EQ(bits(ap_strtod("1e000000000000000000001", nullptr)), bits(10.0));
    EXPECT_EQ(bits(ap_strtod("123456789012345678901234567890", nullptr)), bits(1.2345678901234568e29));
}

TEST(AP_Common_strtod, MantissaExponentLimits)
{
    struct {
        const char *prefix;
        size_t zeros;
        const char *suffix;
        double value;
        bool range_error;
    } cases[] = {
        // Cancellation must still work near the largest parsed exponent.
        { "1", 999999, "e-999999!", 1, false },
        { "0.", 999998, "1e999999!", 1, false },
        { "0x1", 249999, "p-999996!", 1, false },
        { "0x0.", 249998, "1p999996!", 1, false },
        // Beyond the mantissa limit even the largest exponent cannot cancel it.
        { "1", 1100000, "e-999999!", INFINITY, true },
        { "0.", 1100000, "1e999999!", 0, true },
        { "0x1", 1100000, ".fp-999999!", INFINITY, true },
        { "0x0.", 1100000, "123456789abcdef1p999999!", 0, true },
        { "0.", 1100000, "e999999!", 0, false },
        { "0x0.", 1100000, "p999999!", 0, false },
    };
    for (const auto &c : cases) {
        const std::string s = c.prefix + std::string(c.zeros, '0') + c.suffix;
        char *end = nullptr;
        errno = 0;
        EXPECT_EQ(bits(ap_strtod(s.c_str(), &end)), bits(c.value)) << c.prefix << c.suffix;
        EXPECT_EQ(end - s.c_str(), long(s.size() - 1));
        EXPECT_EQ(errno, c.range_error ? ERANGE : 0);
        errno = 0;
        EXPECT_EQ(bits(ap_strtof(s.c_str(), &end)), bits(float(c.value))) << c.prefix << c.suffix;
        EXPECT_EQ(end - s.c_str(), long(s.size() - 1));
        EXPECT_EQ(errno, c.range_error ? ERANGE : 0);
    }
}

TEST(AP_Common_strtod, LongScanPaths)
{
    const std::string padding(1100000, 'a');
    const std::string cases[] = {
        std::string(1100000, ' ') + "1!",
        "1e+" + std::string(1100000, '9') + "!",
        "1e-" + std::string(1100000, '9') + "!",
        "0x1p+" + std::string(1100000, '9') + "!",
        "0x1p-" + std::string(1100000, '9') + "!",
        "nan(" + padding + ")!",
        "nan(" + padding,
        "nan(" + padding + "-)!",
    };
    const double values[] = { 1, INFINITY, 0, INFINITY, 0, NAN, NAN, NAN };
    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        const std::string &s = cases[i];
        const bool nan = isnan(values[i]);
        const long expected_end = i >= 6 ? 3 : long(s.size() - 1);
        const int expected_errno = i >= 1 && i <= 4 ? ERANGE : 0;
        char *end = nullptr;
        errno = 0;
        const double d = ap_strtod(s.c_str(), &end);
        EXPECT_TRUE(nan ? isnan(d) : bits(d) == bits(values[i]));
        EXPECT_EQ(end - s.c_str(), expected_end);
        EXPECT_EQ(errno, expected_errno);
        errno = 0;
        const float f = ap_strtof(s.c_str(), &end);
        EXPECT_TRUE(nan ? isnan(f) : bits(f) == bits(float(values[i])));
        EXPECT_EQ(end - s.c_str(), expected_end);
        EXPECT_EQ(errno, expected_errno);
    }
}

TEST(AP_Common_strtod, NullEndptrAndAtof)
{
    EXPECT_EQ(bits(ap_strtod("2.5x", nullptr)), bits(2.5));
    EXPECT_EQ(bits(ap_strtof("2.5x", nullptr)), bits(2.5f));
    EXPECT_EQ(bits(atof("-12.75")), bits(-12.75));
    EXPECT_EQ(bits(atof("junk")), bits(0.0));
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(CYGWIN_BUILD) && !defined(__EMSCRIPTEN__)
// the C library names go to our versions on SITL, as on ChibiOS
TEST(AP_Common_strtod, LibraryNamesReplaced)
{
    EXPECT_NE((void *)&strtod, (void *)libc_strtod());
    EXPECT_NE((void *)&strtof, (void *)libc_strtof());
}
#endif

/*
  compare with the C library over many generated strings
 */
class Generator {
public:
    explicit Generator(uint64_t seed) : state(seed) {}

    uint32_t below(uint32_t n)
    {
        // xorshift64*
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        return uint32_t((state * 0x2545F4914F6CDD1DULL) >> 32) % n;
    }

    std::string decimal(uint32_t max_digits, int32_t min_exp, int32_t max_exp)
    {
        static const char *signs[] = { "", "", "-", "+" };
        std::string s = signs[below(4)];
        const uint32_t n = 1 + below(max_digits);
        std::string digits;
        for (uint32_t i = 0; i < n; i++) {
            digits += char('0' + below(10));
        }
        if (below(5) == 0) {
            digits = std::string(below(5), '0') + digits;
        }
        const uint32_t point = below(uint32_t(digits.size()) + 2);
        if (point <= digits.size() && below(3) != 0) {
            digits.insert(point, ".");
        }
        s += digits;
        if (below(3) != 0) {
            s += below(2) ? "e" : "E";
            s += std::to_string(min_exp + int32_t(below(uint32_t(max_exp - min_exp + 1))));
        }
        return s;
    }

    std::string hex()
    {
        static const char *hex_digits = "0123456789abcdefABCDEF";
        std::string s = below(2) ? "0x" : "-0X";
        const uint32_t n = 1 + below(20);
        std::string digits;
        for (uint32_t i = 0; i < n; i++) {
            digits += hex_digits[below(22)];
        }
        if (below(2)) {
            digits.insert(below(n), ".");
        }
        s += digits;
        if (below(2)) {
            s += below(2) ? "p" : "P";
            s += std::to_string(int32_t(below(2200)) - 1100);
        }
        return s;
    }

    // short strings of characters that can appear in numbers
    std::string junk()
    {
        static const char *chars = "0123456789..eE+-xXpPinfaINFAtyY() \t";
        std::string s;
        const uint32_t n = below(12);
        for (uint32_t i = 0; i < n; i++) {
            s += chars[below(uint32_t(strlen(chars)))];
        }
        return s;
    }

    std::string long_decimal(int32_t min_exp, int32_t max_exp)
    {
        std::string s = below(2) ? "" : "-";
        const uint32_t n = 20 + below(60);
        std::string digits;
        for (uint32_t i = 0; i < n; i++) {
            digits += char('0' + below(10));
        }
        if (digits[0] == '0') {
            digits[0] = '1';
        }
        digits.insert(1 + below(n - 1), ".");
        return s + digits + "e" + std::to_string(min_exp + int32_t(below(uint32_t(max_exp - min_exp + 1))));
    }

private:
    uint64_t state;
};

// Integer-only rounding in units of 2^-1074. Older glibc versions can
// double-round hexadecimal subnormals, so libc is not an exact oracle there.
static bool hex_subnormal_bits(const std::string &s, uint64_t &result)
{
    const char *p = s.c_str();
    while (isspace(uint8_t(*p))) {
        p++;
    }
    const bool negative = *p == '-';
    if (*p == '-' || *p == '+') {
        p++;
    }
    if (p[0] != '0' || (p[1] != 'x' && p[1] != 'X')) {
        return false;
    }
    p += 2;
    uint32_t mantissa[4] {};
    int32_t exponent = 1074;
    bool point = false;
    bool any_digits = false;
    static const char hex_digits[] = "0123456789abcdef";
    while (*p != '\0') {
        if (*p == '.' && !point) {
            point = true;
            p++;
            continue;
        }
        const char *digit = strchr(hex_digits, tolower(uint8_t(*p)));
        if (digit == nullptr) {
            break;
        }
        if (mantissa[3] >> 28) {
            return false;
        }
        for (uint8_t i = 3; i > 0; i--) {
            mantissa[i] = (mantissa[i] << 4) | (mantissa[i - 1] >> 28);
        }
        mantissa[0] = (mantissa[0] << 4) | uint8_t(digit - hex_digits);
        if (point) {
            exponent -= 4;
        }
        any_digits = true;
        p++;
    }
    if (!any_digits) {
        return false;
    }
    if (*p == 'p' || *p == 'P') {
        const long e = strtol(p + 1, nullptr, 10);
        if (e < -100000 || e > 100000) {
            return false;
        }
        exponent += e;
    }
    const uint64_t normal_min = uint64_t(1) << 52;
    uint64_t rounded = 0;
    if (exponent >= 0) {
        const uint64_t value = (uint64_t(mantissa[1]) << 32) | mantissa[0];
        if (mantissa[2] != 0 || mantissa[3] != 0 || exponent > 52 || value > (normal_min >> exponent)) {
            return false;
        }
        rounded = value << exponent;
    } else {
        const uint32_t shift = -exponent;
        bool half = false;
        bool sticky = false;
        for (uint32_t bit = 0; bit < 128; bit++) {
            if ((mantissa[bit / 32] & (uint32_t(1) << (bit % 32))) == 0) {
                continue;
            }
            if (bit >= shift) {
                if (bit - shift > 52) {
                    return false;
                }
                rounded |= uint64_t(1) << (bit - shift);
            } else if (bit == shift - 1) {
                half = true;
            } else {
                sticky = true;
            }
        }
        if (half && (sticky || (rounded & 1))) {
            rounded++;
        }
        if (rounded > normal_min) {
            return false;
        }
    }
    result = uint64_t(rounded) | (negative ? uint64_t(1) << 63 : 0);
    return true;
}

TEST(AP_Common_strtod, HexSubnormalReference)
{
    const struct {
        const char *str;
        uint64_t value;
    } cases[] = {
        { "0x1p-1074", 1 },
        { "0x1p-1075", 0 },
        { "0x3p-1075", 2 },
        { "0x5p-1075", 2 },
        { "0x7p-1075", 4 },
        { "0xfffffffffffffff1p-1139", 0 },
        { "0xfffffffffffffff1p-1138", 1 },
        { "0x1p-2098", 0 },
        { "0x0p-3000", 0 },
        { "-0x0p-3000", 0x8000000000000000 },
        { "0x80000000000000000000000000000000p-1202", 0 },
        { "0x80000000000000000000000000000001p-1202", 1 },
        { "0xffffffffffffffffffffffffffffffffp-1203", 0 },
        { "0x0.fffffffffffff7p-1022", 0xfffffffffffff },
        { "0x0.fffffffffffff8p-1022", 0x10000000000000 },
        { "0x0.fffffffffffff9p-1022", 0x10000000000000 },
        { "-0X7B6CEc.aFe62152p-1047", 0x8003db67657f310b },
    };
    for (const auto &c : cases) {
        uint64_t reference;
        ASSERT_TRUE(hex_subnormal_bits(c.str, reference)) << c.str;
        EXPECT_EQ(reference, c.value) << c.str;
        EXPECT_EQ(bits(ap_strtod(c.str, nullptr)), c.value) << c.str;
    }
}

// Require exact values for these comparison cases; digit count alone does
// not guarantee correct rounding near a halfway value.
static void compare_exact(const std::string &s)
{
    const strtod_fn ref_d = libc_strtod();
    const strtof_fn ref_f = libc_strtof();
    char *end1, *end2;

    errno = 0;
    const double a = ap_strtod(s.c_str(), &end1);
    const int errno1 = errno;
    errno = 0;
    const double b = ref_d(s.c_str(), &end2);
    const int errno2 = errno;
    ASSERT_EQ(end1, end2) << s;
    // libc can report underflow when rounding up to DBL_MIN. Our errno
    // contract is checked independently in Range.
    if (errno2 == ERANGE && fabs(b) == DBL_MIN) {
        ASSERT_TRUE(errno1 == 0 || errno1 == ERANGE) << s;
    } else {
        ASSERT_EQ(errno1, errno2) << s;
    }
    if (isnan(b)) {
        ASSERT_TRUE(isnan(a)) << s;
        ASSERT_EQ(signbit(a), signbit(b)) << s;
    } else {
        uint64_t reference;
        if (hex_subnormal_bits(s, reference)) {
            ASSERT_EQ(bits(a), reference) << s;
        } else {
            ASSERT_EQ(bits(a), bits(b)) << s;
        }
    }

    errno = 0;
    const float fa = ap_strtof(s.c_str(), &end1);
    const int ferrno1 = errno;
    errno = 0;
    const float fb = ref_f(s.c_str(), &end2);
    const int ferrno2 = errno;
    ASSERT_EQ(end1, end2) << s;
    // Older glibc versions can omit ERANGE for inexact hexadecimal
    // subnormals, or report it when rounding up to FLT_MIN. Our errno
    // contract is checked independently in Range.
    if (ferrno2 == 0 && fabsf(fb) > 0 && fabsf(fb) < FLT_MIN) {
        ASSERT_TRUE(ferrno1 == 0 || ferrno1 == ERANGE) << s;
    } else if (ferrno2 == ERANGE && fabsf(fb) == FLT_MIN) {
        ASSERT_TRUE(ferrno1 == 0 || ferrno1 == ERANGE) << s;
    } else {
        ASSERT_EQ(ferrno1, ferrno2) << s;
    }
    if (isnan(fb)) {
        ASSERT_TRUE(isnan(fa)) << s;
    } else {
        ASSERT_EQ(bits(fa), bits(fb)) << s;
    }
}

TEST(AP_Common_strtod, MatchesLibraryFloatBoundaries)
{
    compare_exact("3.4028235677973366e38");
    compare_exact("-3.4028235677973366e38");
    compare_exact("1.1754942807573643e-38");
    compare_exact("-1.1754942807573643e-38");
}

TEST(AP_Common_strtod, MatchesLibraryDoubleBoundaries)
{
    compare_exact("2.2250738585072012e-308");
    compare_exact("-2.2250738585072012e-308");
}

// Exercise character classification at each parsing position, including
// bytes that are negative when char is signed.
TEST(AP_Common_strtod, MatchesLibraryCharacterClasses)
{
    const char *prefixes[] = { "", "1", "1e", "0x", "0x.", "0x1", "0x1p", "nan(", "in", "infin", "na" };
    for (const char *prefix : prefixes) {
        for (uint16_t c = 1; c <= UINT8_MAX; c++) {
            const std::string s = std::string(prefix) + char(c) + "1)";
            compare_exact(s);
        }
    }
}

TEST(AP_Common_strtod, MatchesLibraryDecimal)
{
    Generator g(1);
    for (uint32_t i = 0; i < 200000; i++) {
        compare_exact(g.decimal(19, -350, 320));
        compare_exact(g.decimal(9, -50, 40));
        if (HasFatalFailure()) {
            return;
        }
    }
}

TEST(AP_Common_strtod, MatchesLibraryHex)
{
    Generator g(2);
    for (uint32_t i = 0; i < 200000; i++) {
        compare_exact(g.hex());
        if (HasFatalFailure()) {
            return;
        }
    }
}

TEST(AP_Common_strtod, MatchesLibraryJunk)
{
    Generator g(3);
    for (uint32_t i = 0; i < 200000; i++) {
        compare_exact(g.junk());
        if (HasFatalFailure()) {
            return;
        }
    }
}

// halfway between two floats or doubles, in up to 19 significant digits
TEST(AP_Common_strtod, MatchesLibraryNearHalfway)
{
    Generator g(4);
    char buf[64];
    for (uint32_t i = 0; i < 100000; i++) {
        uint64_t b64 = (uint64_t(g.below(0x7fefffff)) << 32) | g.below(0xffffffff);
        double d;
        memcpy(&d, &b64, sizeof(d));
        const long double mid = (static_cast<long double>(d) + nextafter(d, INFINITY)) / 2;
        snprintf(buf, sizeof(buf), "%.*Le", int(g.below(19)), mid);
        compare_exact(buf);

        uint32_t b32 = g.below(0x7f7fffff);
        float f;
        memcpy(&f, &b32, sizeof(f));
        const double fmid = double(f) + (double(nextafterf(f, INFINITY)) - double(f)) / 2;
        snprintf(buf, sizeof(buf), "%.*e", int(g.below(19)), fmid);
        compare_exact(buf);
        if (HasFatalFailure()) {
            return;
        }
    }
}

// more than 19 significant digits can be a bit out when very close to halfway
TEST(AP_Common_strtod, MatchesLibraryLongWithinOneUlp)
{
    Generator g(5);
    const strtod_fn ref_d = libc_strtod();
    const strtof_fn ref_f = libc_strtof();
    for (uint32_t i = 0; i < 100000; i++) {
        const std::string s = g.long_decimal(-340, 300);
        char *end1, *end2;
        const double a = ap_strtod(s.c_str(), &end1);
        const double b = ref_d(s.c_str(), &end2);
        ASSERT_EQ(end1, end2) << s;
        if (isinf(b) || b == 0) {
            ASSERT_EQ(bits(a), bits(b)) << s;
        } else {
            ASSERT_LE(ulps(a, b), 1U) << s;
        }
        const float fa = ap_strtof(s.c_str(), &end1);
        const float fb = ref_f(s.c_str(), &end2);
        ASSERT_EQ(end1, end2) << s;
        if (isinf(fb) || fb == 0) {
            ASSERT_EQ(bits(fa), bits(fb)) << s;
        } else {
            ASSERT_LE(ulps(fa, fb), 1U) << s;
        }
    }
}

AP_GTEST_MAIN()
