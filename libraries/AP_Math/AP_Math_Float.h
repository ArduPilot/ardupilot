/*
 * This file is the current standard math implementation until the "DBL_MATH" build flag is set.
 * With this template implementation all double promotions are avoided, 
 * which can happen because of lazy integer usage in ArduPilot.
 */

#pragma once

#include <math.h>

#include <cmath>
#include <limits>
#include <stdint.h>
#include <type_traits>

#include "definitions.h"

/*
 * Should be in the std library, but isn't for many platforms
 */
#undef isfinite
namespace std {
    template<class T>
    bool isfinite(const T& val) {
        static_assert(std::is_floating_point<T>::value, "ERROR - isfinite(): template parameter not of type float\n");
        return (isnan(val) || isinf(val)) ? false : true;
    }
};

/* 
 * @brief: Checks whether two floats are equal
 */
template <class FloatOne, class FloatTwo>
bool is_equal(const FloatOne fVal1, const FloatTwo fVal2) {
    static_assert(std::is_arithmetic<FloatOne>::value, "ERROR - is_equal(): template parameter not of type float or int\n");
    static_assert(std::is_arithmetic<FloatTwo>::value, "ERROR - is_equal(): template parameter not of type float or int\n");
    return fabsf(fVal1 - fVal2) < std::numeric_limits<decltype(fVal1 - fVal2)>::epsilon() ? true : false; 
}

/* 
 * @brief: Checks whether a float is zero
 */
template <class T>
bool is_zero(const T fVal) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - is_zero(): template parameter not of type float or int\n");
    return fabsf(fVal) < std::numeric_limits<T>::epsilon() ? true : false; 
} 

/*
 * A varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is returned.
 */
template <class T>
float safe_asin(const T &v) {
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

/* 
 * A varient of sqrt() that checks the input ranges and ensures a 
 * valid value as output. If a negative number is given then 0 is returned. 
 * The reasoning is that a negative number for sqrt() in our 
 * code is usually caused by small numerical rounding errors, so the 
 * real input should have been zero
 */
template <class T>
float safe_sqrt(const T &v) {
    float ret = sqrt(static_cast<float>(v));
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

/* 
 * @brief: Constrains an euler angle to be within the range: -180 to 180 degrees
 */
template <class T>
float wrap_180(const T &angle, float unit_mod = 1) {   
    const float ang_180 = 180.f*unit_mod;
    const float ang_360 = 360.f*unit_mod;
    float res = fmod(static_cast<float>(angle) + ang_180, ang_360);
    if (res < 0 || is_zero(res)) {
        res += ang_180;
    }
    else {
        res -= ang_180;
    }
    return res;
}

/* 
 * @brief: Constrains an euler angle to be within the range: 0 to 360 degrees
 * The second parameter changes the units. Standard: 1 == degrees, 10 == dezi, 100 == centi ..
 */
template <class T>
float wrap_360(const T &angle, float unit_mod = 1) {
    const float ang_360 = 360.f*unit_mod;
    float res = fmod(static_cast<float>(angle), ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <class T>
float wrap_PI(const T &radian) {
    float res = fmod(static_cast<float>(radian) + M_PI, M_2PI);
    if (res < 0 || is_zero(res)) {
        res += M_PI;
    }
    else {
        res -= M_PI;
    }
    return res;
}

/*
 * wrap an angle in radians to 0..2PI
 */
template <class T>
float wrap_2PI(const T &radian) {
    float res = fmod(static_cast<float>(radian), M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}

template<class T>
float sq(const T &val) {
    return powf(static_cast<float>(val), 2);
}
template<class T, class... Params>
float sq(const T &first, const Params&... parameters) {
    return sq(first) + sq(parameters...);
}
template<class T, class... Params>
float norm(const T &first, const Params&... parameters) {
    return sqrt(static_cast<float>(sq(first, parameters...)));
}

/*
 * @brief: Constrains a value to be within the range: low and high
 */
template <class T>
T constrain_value(const T &amt, const T &low, const T &high) {
    if (isnan(low) || isnan(high)) {
        return amt;
    }
    return amt < low ? low : (amt > high ? high : amt);
}

/* 
 * @brief: Converts an euler angle with units 'degree' to an angle with the unit 'radian'
 */
template <class T>
float radians(const T &deg) {
    return static_cast<float>(deg) * DEG_TO_RAD;
}

/* 
 * @brief: Converts an euler angle with units 'radian' to an angle with the unit 'degree'
 */
template <class T>
float degrees(const T &rad) {
    return static_cast<float>(rad) * RAD_TO_DEG;
}

