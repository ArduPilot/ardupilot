#include "AP_Math.h"
#include <float.h>

template <class FloatOne, class FloatTwo>
bool is_equal(const FloatOne v_1, const FloatTwo v_2)
{
    static_assert(std::is_arithmetic<FloatOne>::value, "template parameter not of type float or int");
    static_assert(std::is_arithmetic<FloatTwo>::value, "template parameter not of type float or int");
    return fabsf(v_1 - v_2) < std::numeric_limits<decltype(v_1 - v_2)>::epsilon();
}

template bool is_equal<int>(const int v_1, const int v_2);
template bool is_equal<short>(const short v_1, const short v_2);
template bool is_equal<float>(const float v_1, const float v_2);
template bool is_equal<double>(const double v_1, const double v_2);

template <class T>
float safe_asin(const T v)
{
    if (isnan(static_cast<float>(v))) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return static_cast<float>(M_PI_2);
    }
    if (v <= -1.0f) {
        return static_cast<float>(-M_PI_2);
    }
    return asinf(static_cast<float>(v));
}

template float safe_asin<int>(const int v);
template float safe_asin<short>(const short v);
template float safe_asin<float>(const float v);
template float safe_asin<double>(const double v);

template <class T>
float safe_sqrt(const T v)
{
    float ret = sqrtf(static_cast<float>(v));
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

template float safe_sqrt<int>(const int v);
template float safe_sqrt<short>(const short v);
template float safe_sqrt<float>(const float v);
template float safe_sqrt<double>(const double v);

/*
  linear interpolation based on a variable in a range
 */
float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high)
{
    if (var_value <= var_low) {
        return low_output;
    }
    if (var_value >= var_high) {
        return high_output;
    }
    float p = (var_value - var_low) / (var_high - var_low);
    return low_output + p * (high_output - low_output);
}

template <class T>
float wrap_180(const T angle, float unit_mod)
{
    auto res = wrap_360(angle, unit_mod);
    if (res > 180.f * unit_mod) {
        res -= 360.f * unit_mod;
    }
    return res;
}

template float wrap_180<int>(const int angle, float unit_mod);
template float wrap_180<short>(const short angle, float unit_mod);
template float wrap_180<float>(const float angle, float unit_mod);
template float wrap_180<double>(const double angle, float unit_mod);

template <class T>
auto wrap_180_cd(const T angle) -> decltype(wrap_180(angle, 100.f))
{
    return wrap_180(angle, 100.f);
}

template auto wrap_180_cd<float>(const float angle) -> decltype(wrap_180(angle, 100.f));
template auto wrap_180_cd<int>(const int angle) -> decltype(wrap_180(angle, 100.f));
template auto wrap_180_cd<short>(const short angle) -> decltype(wrap_180(angle, 100.f));
template auto wrap_180_cd<double>(const double angle) -> decltype(wrap_360(angle, 100.f));

template <class T>
float wrap_360(const T angle, float unit_mod)
{
    const float ang_360 = 360.f * unit_mod;
    float res = fmodf(static_cast<float>(angle), ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}

template float wrap_360<int>(const int angle, float unit_mod);
template float wrap_360<short>(const short angle, float unit_mod);
template float wrap_360<float>(const float angle, float unit_mod);
template float wrap_360<double>(const double angle, float unit_mod);

template <class T>
auto wrap_360_cd(const T angle) -> decltype(wrap_360(angle, 100.f))
{
    return wrap_360(angle, 100.f);
}

template auto wrap_360_cd<float>(const float angle) -> decltype(wrap_360(angle, 100.f));
template auto wrap_360_cd<int>(const int angle) -> decltype(wrap_360(angle, 100.f));
template auto wrap_360_cd<short>(const short angle) -> decltype(wrap_360(angle, 100.f));
template auto wrap_360_cd<double>(const double angle) -> decltype(wrap_360(angle, 100.f));

template <class T>
float wrap_PI(const T radian)
{
    auto res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

template float wrap_PI<int>(const int radian);
template float wrap_PI<short>(const short radian);
template float wrap_PI<float>(const float radian);
template float wrap_PI<double>(const double radian);

template <class T>
float wrap_2PI(const T radian)
{
    float res = fmodf(static_cast<float>(radian), M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

template float wrap_2PI<int>(const int radian);
template float wrap_2PI<short>(const short radian);
template float wrap_2PI<float>(const float radian);
template float wrap_2PI<double>(const double radian);

template <class T>
T constrain_value(const T amt, const T low, const T high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_float(). The normal
    // float semantics already handle -Inf and +Inf
    if (isnan(amt)) {
        return (low + high) * 0.5f;
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

template int constrain_value<int>(const int amt, const int low, const int high);
template short constrain_value<short>(const short amt, const short low, const short high);
template float constrain_value<float>(const float amt, const float low, const float high);
template double constrain_value<double>(const double amt, const double low, const double high);
