#include <AP_HAL/AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(AP_PICOLIBC_SITL_ENABLED) && AP_PICOLIBC_SITL_ENABLED

#include <stdint.h>
#include <string.h>

extern "C" double ap_picolibc_strtod(const char *str, char **endptr);
extern "C" float ap_picolibc_strtof(const char *str, char **endptr);
extern "C" double ap_picolibc_atof(const char *str);
extern "C" int ap_picolibc___fpclassifyd(double value);
extern "C" int ap_picolibc___fpclassifyf(float value);
extern "C" int ap_picolibc___isnand(double value);
extern "C" int ap_picolibc___isnanf(float value);
#if defined(__EMSCRIPTEN__)
extern "C" double ap_picolibc_public_strtod(const char *str, char **endptr) __asm__("__wrap_strtod");
extern "C" float ap_picolibc_public_strtof(const char *str, char **endptr) __asm__("__wrap_strtof");
extern "C" double ap_picolibc_public_atof(const char *str) __asm__("__wrap_atof");
#elif defined(__APPLE__)
extern "C" double ap_picolibc_public_strtod(const char *str, char **endptr) __asm__("_strtod");
extern "C" float ap_picolibc_public_strtof(const char *str, char **endptr) __asm__("_strtof");
extern "C" double ap_picolibc_public_atof(const char *str) __asm__("_atof");
#else
extern "C" double ap_picolibc_public_strtod(const char *str, char **endptr) __asm__("strtod");
extern "C" float ap_picolibc_public_strtof(const char *str, char **endptr) __asm__("strtof");
extern "C" double ap_picolibc_public_atof(const char *str) __asm__("atof");
#endif

extern "C" double ap_picolibc_public_strtod(const char *str, char **endptr)
{
    return ap_picolibc_strtod(str, endptr);
}

extern "C" float ap_picolibc_public_strtof(const char *str, char **endptr)
{
    return ap_picolibc_strtof(str, endptr);
}

extern "C" double ap_picolibc_public_atof(const char *str)
{
    return ap_picolibc_atof(str);
}

static uint64_t double_bits(double value)
{
    uint64_t bits;
    static_assert(sizeof(bits) == sizeof(value), "unexpected double size");
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static uint32_t float_bits(float value)
{
    uint32_t bits;
    static_assert(sizeof(bits) == sizeof(value), "unexpected float size");
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

extern "C" int ap_picolibc___fpclassifyd(double value)
{
    const uint64_t bits = double_bits(value);
    const uint64_t exponent = bits & UINT64_C(0x7ff0000000000000);
    const uint64_t fraction = bits & UINT64_C(0x000fffffffffffff);
    if (exponent == UINT64_C(0x7ff0000000000000)) {
        return fraction == 0 ? 1 : 0; // FP_INFINITE : FP_NAN
    }
    if (exponent == 0) {
        return fraction == 0 ? 2 : 3; // FP_ZERO : FP_SUBNORMAL
    }
    return 4; // FP_NORMAL
}

extern "C" int ap_picolibc___fpclassifyf(float value)
{
    const uint32_t bits = float_bits(value);
    const uint32_t exponent = bits & UINT32_C(0x7f800000);
    const uint32_t fraction = bits & UINT32_C(0x007fffff);
    if (exponent == UINT32_C(0x7f800000)) {
        return fraction == 0 ? 1 : 0; // FP_INFINITE : FP_NAN
    }
    if (exponent == 0) {
        return fraction == 0 ? 2 : 3; // FP_ZERO : FP_SUBNORMAL
    }
    return 4; // FP_NORMAL
}

extern "C" int ap_picolibc___isnand(double value)
{
    return ap_picolibc___fpclassifyd(value) == 0;
}

extern "C" int ap_picolibc___isnanf(float value)
{
    return ap_picolibc___fpclassifyf(value) == 0;
}

#endif
