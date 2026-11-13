/*
  conversion of JSON numbers to double and float

  A number with few enough significant digits, and a small enough power
  of ten, is converted with a single multiplication or division of two
  values that are exact in the target type. IEEE arithmetic rounds that
  one operation correctly, so the result is the same as strtod's or
  strtof's (Clinger's fast path). Other numbers fall back to strtod /
  strtof. The fast path is only compiled where floating point arithmetic
  has no extended precision.
 */

#include "AP_JSON_Number.h"

#include <float.h>
#include <stdint.h>
#include <stdlib.h>

#if defined(FLT_EVAL_METHOD) && FLT_EVAL_METHOD == 0
#define AP_JSON_NUMBER_FAST_PATH 1
#else
#define AP_JSON_NUMBER_FAST_PATH 0
#endif

#if AP_JSON_NUMBER_FAST_PATH
namespace {

/*
  split a number into sign, decimal mantissa and power of ten. Returns
  false if it has more than max_digits significant digits. The mantissa
  is built in 32 bits while it fits (9 digits), as 64 bit arithmetic is
  slow on 32 bit cores
 */
bool decimal_parts(const char *p, uint8_t max_digits, bool &negative, uint64_t &mantissa, int &exp10)
{
    negative = (*p == '-');
    if (negative) {
        p++;
    }
    uint32_t m32 = 0;
    uint64_t m64 = 0;
    uint8_t digits = 0;
    int e10 = 0;
    bool fraction = false;
    for (;; p++) {
        uint8_t d;
        if (*p >= '0' && *p <= '9') {
            d = uint8_t(*p - '0');
        } else if (*p == '.' && !fraction) {
            fraction = true;
            continue;
        } else {
            break;
        }
        if (fraction) {
            e10--;
        }
        if (digits == 0 && d == 0) {
            continue;   // leading zeros are not significant
        }
        if (++digits > max_digits) {
            return false;
        }
        if (digits <= 9) {
            m32 = m32 * 10 + d;
        } else {
            if (digits == 10) {
                m64 = m32;
            }
            m64 = m64 * 10 + d;
        }
    }
    if (*p == 'e' || *p == 'E') {
        p++;
        const bool exp_negative = (*p == '-');
        if (*p == '+' || *p == '-') {
            p++;
        }
        int e = 0;
        for (; *p >= '0' && *p <= '9'; p++) {
            if (e >= 10000) {
                // leading zeros can offset any exponent, so leave it to
                // strtod rather than clamp it
                return false;
            }
            e = e * 10 + (*p - '0');
        }
        e10 += exp_negative ? -e : e;
    }
    mantissa = digits <= 9 ? m32 : m64;
    exp10 = e10;
    return true;
}

// powers of ten that are exact in the type: 10^22 for double, 10^10 for
// float. Built by integer multiplication so -fsingle-precision-constant
// can not round them
constexpr double pow10_double(unsigned n)
{
    return n == 0 ? 1 : 10 * pow10_double(n - 1);
}
constexpr float pow10_float(unsigned n)
{
    return n == 0 ? 1 : 10 * pow10_float(n - 1);
}
const double powers_double[] = {
    pow10_double(0), pow10_double(1), pow10_double(2), pow10_double(3), pow10_double(4),
    pow10_double(5), pow10_double(6), pow10_double(7), pow10_double(8), pow10_double(9),
    pow10_double(10), pow10_double(11), pow10_double(12), pow10_double(13), pow10_double(14),
    pow10_double(15), pow10_double(16), pow10_double(17), pow10_double(18), pow10_double(19),
    pow10_double(20), pow10_double(21), pow10_double(22),
};
const float powers_float[] = {
    pow10_float(0), pow10_float(1), pow10_float(2), pow10_float(3), pow10_float(4),
    pow10_float(5), pow10_float(6), pow10_float(7), pow10_float(8), pow10_float(9),
    pow10_float(10),
};

} // namespace
#endif // AP_JSON_NUMBER_FAST_PATH

double AP_JSON_Number::to_double(const char *s)
{
#if AP_JSON_NUMBER_FAST_PATH
    bool negative;
    uint64_t mantissa;
    int exp10;
    // 15 digits: mantissa < 10^15 < 2^53 is exact as a double
    if (decimal_parts(s, 15, negative, mantissa, exp10)) {
        double v = double(mantissa);
        if (mantissa == 0) {
            return negative ? -v : v;
        }
        if (exp10 >= 0 && exp10 <= 22) {
            v *= powers_double[exp10];
            return negative ? -v : v;
        }
        if (exp10 < 0 && exp10 >= -22) {
            v /= powers_double[-exp10];
            return negative ? -v : v;
        }
    }
#endif
    return strtod(s, nullptr);
}

float AP_JSON_Number::to_float(const char *s)
{
#if AP_JSON_NUMBER_FAST_PATH
    bool negative;
    uint64_t mantissa;
    int exp10;
    // 7 digits: mantissa < 10^7 < 2^24 is exact as a float
    if (decimal_parts(s, 7, negative, mantissa, exp10)) {
        // at most 7 digits, so the mantissa fits in 32 bits
        float v = float(uint32_t(mantissa));
        if (mantissa == 0) {
            return negative ? -v : v;
        }
        if (exp10 >= 0 && exp10 <= 10) {
            v *= powers_float[exp10];
            return negative ? -v : v;
        }
        if (exp10 < 0 && exp10 >= -10) {
            v /= powers_float[-exp10];
            return negative ? -v : v;
        }
    }
#endif
    return strtof(s, nullptr);
}
