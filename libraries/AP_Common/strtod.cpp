/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  strtod/strtof without the memory allocation of newlib's versions.

  Up to 19 significant decimal digits are kept and scaled in double-double
  arithmetic, then rounded once. Correct rounding is not guaranteed near a
  halfway value: limited arithmetic precision or digits beyond the 19th can
  decide the rounding incorrectly.
 */

#include "strtod.h"
#include "AP_Common.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

// correct rounding needs exact floating point comparisons
#pragma GCC diagnostic ignored "-Wfloat-equal"

static const uint8_t MAX_DECIMAL_DIGITS = 19;
static const uint8_t MAX_HEX_DIGITS = 15;
// beyond any exponent that can give a finite non-zero result
static const int32_t EXPONENT_LIMIT = 100000;
// Allow cancellation by the largest parsed exponent (10 * EXPONENT_LIMIT - 1),
// with room for the binary64 range and the retained mantissa digits.
static const int32_t MANTISSA_EXPONENT_LIMIT = 10 * EXPONENT_LIMIT + 2048;

// 10^n for n <= 22, which is exact in a double
static double exact_pow10(uint8_t n)
{
    double v = 1;
    while (n--) {
        v *= 10;
    }
    return v;
}

// 2^e for -1074 <= e <= 0, which is exact
static double pow2_negative(int32_t e)
{
    const double p60 = double(1ULL << 60);
    double p = 1;
    while (e < -60) {
        p /= p60;
        e += 60;
    }
    return p / double(1ULL << -e);
}

// v * 2^e for positive v, rounding at most once
static double scale2(double v, int32_t e)
{
    const double p60 = double(1ULL << 60);
    while (e > 60) {
        v *= p60;
        e -= 60;
        if (v > DBL_MAX) {
            return v;
        }
    }
    // only scale down in steps while the result stays normal
    const double normal_limit = DBL_MIN * p60;
    while (e < -60 && v >= normal_limit) {
        v /= p60;
        e += 60;
    }
    if (e > 0) {
        return v * double(1ULL << e);
    }
    if (e < -1074) {
        // v is below normal_limit, so this is too small for a subnormal
        return 0;
    }
    return e < 0 ? v * pow2_negative(e) : v;
}

// a double-double value, hi + lo with |lo| at most half an ulp of hi
struct DoubleDouble {
    double hi;
    double lo;
};

static DoubleDouble two_sum(double a, double b)
{
    const double s = a + b;
    const double bb = s - a;
    return { s, (a - (s - bb)) + (b - bb) };
}

// a * b exactly as hi + lo (Dekker), for values well below DBL_MAX / 2^27
static DoubleDouble two_product(double a, double b)
{
    const double split = double((1ULL << 27) + 1);
    const double p = a * b;
    double t = split * a;
    const double a_hi = t - (t - a);
    const double a_lo = a - a_hi;
    t = split * b;
    const double b_hi = t - (t - b);
    const double b_lo = b - b_hi;
    return { p, ((a_hi * b_hi - p) + a_hi * b_lo + a_lo * b_hi) + a_lo * b_lo };
}

static DoubleDouble dd_multiply(const DoubleDouble &a, const DoubleDouble &b)
{
    const DoubleDouble p = two_product(a.hi, b.hi);
    return two_sum(p.hi, p.lo + (a.hi * b.lo + a.lo * b.hi));
}

// 5^n to about 100 bits
static DoubleDouble pow5(uint16_t n)
{
    DoubleDouble result { 1, 0 };
    DoubleDouble base { 5, 0 };
    while (n != 0) {
        if (n & 1) {
            result = dd_multiply(result, base);
        }
        n >>= 1;
        if (n != 0) {
            base = dd_multiply(base, base);
        }
    }
    return result;
}

// an unsigned 64 bit integer as a double-double
static DoubleDouble from_uint64(uint64_t v)
{
    const double hi = double(v);
    return { hi, double(int64_t(v - uint64_t(hi))) };
}

/*
  a positive number as (hi + lo) * 2^exponent, before the final rounding
 */
struct Number {
    DoubleDouble value;
    int32_t exponent;
};

/*
  mantissa * 10^exponent. 10^e is split into 5^e * 2^e so the double-double
  arithmetic stays well inside the double range
 */
static Number scale10(const DoubleDouble &mantissa, int32_t exponent)
{
    const double m = mantissa.hi;
    // with at most 19 digits these round to zero and infinity
    if (exponent < -360) {
        return { { 0, 0 }, 0 };
    }
    if (exponent > 310) {
        return { { INFINITY, 0 }, 0 };
    }
    const DoubleDouble p = exponent > -23 && exponent < 23 ?
        DoubleDouble { exact_pow10(exponent >= 0 ? exponent : -exponent), 0 } :
        pow5(uint16_t(exponent >= 0 ? exponent : -exponent));
    const int32_t exponent2 = exponent > -23 && exponent < 23 ? 0 : exponent;
    if (exponent >= 0) {
        const DoubleDouble r = two_product(m, p.hi);
        return { two_sum(r.hi, r.lo + (m * p.lo + mantissa.lo * p.hi)), exponent2 };
    }
    const double q = m / p.hi;
    const DoubleDouble qp = two_product(q, p.hi);
    const double remainder = ((m - qp.hi) - qp.lo + mantissa.lo) - q * p.lo;
    return { two_sum(q, remainder / p.hi), exponent2 };
}

static bool is_odd(double v)
{
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));
    return bits & 1;
}

/*
  round a positive number to a double. Only a subnormal result is rounded
  in the scaling, and then a tie in hi is decided by lo
 */
static double to_double(const Number &n)
{
    // Hex mantissas occupy at most 61 bits including sticky, so this is
    // below half the smallest subnormal. Decimal scaling never gets here.
    // Avoid overflowing when scaling the rounding threshold back up.
    if (n.exponent < -1136) {
        return 0;
    }
    const double hi = n.value.hi;
    const double lo = n.value.lo;
    double v = scale2(hi, n.exponent);
    if (n.exponent == 0 || v > DBL_MIN) {
        // normal or infinite, so the scaling was exact
        return v;
    }
    // the smallest subnormal and half of it in unscaled units
    const double smallest = pow2_negative(-1074);
    const double half = scale2(smallest, -n.exponent) / 2;
    // exact as both are multiples of the ulp of hi
    const double remainder = hi - scale2(v, -n.exponent);
    if ((remainder == half && (lo > 0 || (lo == 0 && is_odd(v))))) {
        v += smallest;
    } else if (remainder == -half && (lo < 0 || (lo == 0 && is_odd(v)))) {
        v -= smallest;
    }
    return v;
}

static float next_float(float f, int32_t direction)
{
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    bits += direction;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

/*
  round a positive number to a float once. The float range is inside the
  normal double range, so hi and lo scale exactly, then a tie in hi is
  decided by lo
 */
static float to_float(const Number &n)
{
    const double hi = scale2(n.value.hi, n.exponent);
    const double lo = n.value.lo == 0 ? 0 : scale2(n.value.lo < 0 ? -n.value.lo : n.value.lo, n.exponent);
    const bool lo_negative = n.value.lo < 0;
    // The overflow midpoint rounds hi to infinity, but a negative lo
    // puts the complete value below it, where it rounds to FLT_MAX.
    if (lo_negative && hi == double(FLT_MAX) + 0x1p103) {
        return FLT_MAX;
    }
    float f = float(hi);
    if (lo == 0 || f > FLT_MAX) {
        return f;
    }
    const double diff = hi - double(f);
    if (diff > 0) {
        // f is below hi, so a tie is with the next float up
        const float up = next_float(f, 1);
        if (diff == (double(up) - double(f)) / 2 && !lo_negative) {
            f = up;
        }
    } else if (diff < 0) {
        const float down = next_float(f, -1);
        if (-diff == (double(f) - double(down)) / 2 && lo_negative) {
            f = down;
        }
    }
    return f;
}

// optional exponent after the mantissa, leaving s unchanged if there are no digits
static int32_t parse_exponent(const char *&s, char marker, char marker_upper)
{
    if (*s != marker && *s != marker_upper) {
        return 0;
    }
    const char *p = s + 1;
    bool negative = false;
    if (*p == '+' || *p == '-') {
        negative = *p == '-';
        p++;
    }
    if (!isdigit(uint8_t(*p))) {
        return 0;
    }
    int32_t e = 0;
    for (; isdigit(uint8_t(*p)); p++) {
        if (e < EXPONENT_LIMIT) {
            e = e * 10 + (*p - '0');
        }
    }
    s = p;
    return negative ? -e : e;
}

/*
  a parsed number. exact is set if the mantissa had no dropped digits and
  is held exactly, so a subnormal result might be exact
 */
struct Parsed {
    Number number;
    bool nonzero;
    bool exact;
};

// hex mantissa and exponent after the 0x prefix
static Parsed parse_hex(const char *&s)
{
    uint64_t mantissa = 0;
    uint8_t digits = 0;
    int32_t exponent = 0;
    bool sticky = false;
    bool point = false;
    for (;; s++) {
        if (*s == '.' && !point) {
            point = true;
            continue;
        }
        uint8_t v;
        if (!hex_char_to_nibble(*s, v)) {
            break;
        }
        if (mantissa == 0 && v == 0) {
            if (point && exponent > -MANTISSA_EXPONENT_LIMIT) {
                exponent -= 4;
            }
        } else if (digits < MAX_HEX_DIGITS) {
            mantissa = (mantissa << 4) | uint64_t(v);
            digits++;
            if (point) {
                exponent -= 4;
            }
        } else {
            sticky |= v != 0;
            if (!point && exponent < MANTISSA_EXPONENT_LIMIT) {
                exponent += 4;
            }
        }
    }
    if (sticky) {
        // below the last bit we keep, so it only decides ties
        mantissa = (mantissa << 1) | 1;
        exponent--;
    }
    exponent += parse_exponent(s, 'p', 'P');
    const DoubleDouble m = from_uint64(mantissa);
    return { { m, exponent }, mantissa != 0, !sticky && m.lo == 0 };
}

// decimal mantissa and exponent, returning false if there are no digits
static bool parse_decimal(const char *&s, Parsed &parsed)
{
    uint64_t mantissa = 0;
    uint8_t digits = 0;
    int32_t exponent = 0;
    bool any_digits = false;
    bool point = false;
    bool dropped = false;
    for (;; s++) {
        if (*s == '.' && !point) {
            point = true;
            continue;
        }
        if (!isdigit(uint8_t(*s))) {
            break;
        }
        any_digits = true;
        const uint8_t d = *s - '0';
        if (mantissa == 0 && d == 0) {
            if (point && exponent > -MANTISSA_EXPONENT_LIMIT) {
                exponent--;
            }
        } else if (digits < MAX_DECIMAL_DIGITS) {
            mantissa = mantissa * 10 + d;
            digits++;
            if (point) {
                exponent--;
            }
        } else {
            dropped |= d != 0;
            if (!point && exponent < MANTISSA_EXPONENT_LIMIT) {
                exponent++;
            }
        }
    }
    if (!any_digits) {
        return false;
    }
    exponent += parse_exponent(s, 'e', 'E');
    parsed.nonzero = mantissa != 0;
    // a decimal fraction is rarely exact in binary
    parsed.exact = false;
    if (!parsed.nonzero) {
        parsed.number = { { 0, 0 }, 0 };
        return true;
    }
    DoubleDouble m = from_uint64(mantissa);
    if (dropped) {
        // the digits we dropped are worth between 0 and 1 in the last place
        m = two_sum(m.hi, m.lo + 0.5f);
    }
    parsed.number = scale10(m, exponent);
    return true;
}

/*
  parse a number, setting endptr to str if there is none. nonzero is set
  for a finite number with a non-zero mantissa, which can over or underflow
 */
static bool parse(const char *str, char **endptr, Parsed &parsed, bool &negative)
{
    const char *s = str;
    while (isspace(uint8_t(*s))) {
        s++;
    }
    negative = false;
    if (*s == '+' || *s == '-') {
        negative = *s == '-';
        s++;
    }

    parsed.nonzero = false;
    parsed.exact = false;
    if (strncasecmp(s, "inf", 3) == 0) {
        s += 3;
        if (strncasecmp(s, "inity", 5) == 0) {
            s += 5;
        }
        parsed.number = { { INFINITY, 0 }, 0 };
    } else if (strncasecmp(s, "nan", 3) == 0) {
        s += 3;
        if (*s == '(') {
            // optional nan(n-char-sequence)
            const char *p = s + 1;
            while (isalnum(uint8_t(*p)) || *p == '_') {
                p++;
            }
            if (*p == ')') {
                s = p + 1;
            }
        }
        parsed.number = { { NAN, 0 }, 0 };
    } else if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X') &&
               (isxdigit(uint8_t(s[2])) || (s[2] == '.' && isxdigit(uint8_t(s[3]))))) {
        s += 2;
        parsed = parse_hex(s);
    } else if (!parse_decimal(s, parsed)) {
        s = str;
        negative = false;
        parsed.number = { { 0, 0 }, 0 };
    }

    if (endptr != nullptr) {
        *endptr = const_cast<char *>(s);
    }
    return s != str;
}

// ERANGE on overflow, or on an inexact result in the subnormal range or zero
double ap_strtod(const char *str, char **endptr)
{
    Parsed parsed;
    bool negative;
    parse(str, endptr, parsed, negative);
    const double value = to_double(parsed.number);
    if (parsed.nonzero) {
        const bool exact = parsed.exact && scale2(value, -parsed.number.exponent) == parsed.number.value.hi;
        if (value > DBL_MAX || (!exact && value < DBL_MIN)) {
            errno = ERANGE;
        }
    }
    return negative ? -value : value;
}

float ap_strtof(const char *str, char **endptr)
{
    Parsed parsed;
    bool negative;
    parse(str, endptr, parsed, negative);
    const float value = to_float(parsed.number);
    if (parsed.nonzero) {
        const bool exact = parsed.exact && scale2(value, -parsed.number.exponent) == parsed.number.value.hi;
        if (value > FLT_MAX || (!exact && value < FLT_MIN)) {
            errno = ERANGE;
        }
    }
    return negative ? -value : value;
}

/*
  replace the C library functions. On ChibiOS this keeps newlib's versions,
  which can abort, out of the firmware. SITL uses them too, so they get the
  same testing
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS || (CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(__APPLE__))
// Darwin has no --wrap, and with two-level namespaces this only replaces our own calls
#define AP_STRTOD_NAME(name) name
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(CYGWIN_BUILD) && !defined(__EMSCRIPTEN__)
// linked with --wrap
#define AP_STRTOD_NAME(name) __wrap_##name
#endif

#ifdef AP_STRTOD_NAME
extern "C" {
double AP_STRTOD_NAME(strtod)(const char *str, char **endptr);
float AP_STRTOD_NAME(strtof)(const char *str, char **endptr);
double AP_STRTOD_NAME(atof)(const char *str);
}

double AP_STRTOD_NAME(strtod)(const char *str, char **endptr)
{
    return ap_strtod(str, endptr);
}

float AP_STRTOD_NAME(strtof)(const char *str, char **endptr)
{
    return ap_strtof(str, endptr);
}

double AP_STRTOD_NAME(atof)(const char *str)
{
    return ap_strtod(str, nullptr);
}
#endif
