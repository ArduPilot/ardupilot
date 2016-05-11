/*
 * This file is the advanced math implementation inclding support for DP math,
 * only active in case the "DBL_MATH" build flag is set AND the architecture supports DP math (Linux).
 * This implementation will work so far only for SITL and Linux, because PX4 and similar lack DP math.
 */

#pragma once

#include <cmath>
#include <limits>
#include <cstdint>
#include <type_traits>

#include "definitions.h"

/* 
 * @brief: Checks whether two floats are equal
 */
template <class FloatOne, class FloatTwo>
bool is_equal(const FloatOne v_1, const FloatTwo v_2) 
{
    static_assert(std::is_arithmetic<FloatOne>::value, "ERROR - is_equal(): template parameter not of type float or int\n");
    static_assert(std::is_arithmetic<FloatTwo>::value, "ERROR - is_equal(): template parameter not of type float or int\n");
    return std::abs(v_1 - v_2) < std::numeric_limits<decltype(v_1 - v_2)>::epsilon() ? true : false; 
}

/* 
 * @brief: Checks whether a float is zero
 */
template <class T>
bool is_zero(const T v) 
{
    //static_assert(std::is_arithmetic<T>::value, "ERROR - is_zero(): template parameter not of type float or int\n");
    return std::abs(v) < std::numeric_limits<T>::epsilon() ? true : false; 
} 

/*
 * A varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is returned.
 */
template <class T>
auto safe_asin(const T v) -> decltype(std::asin(v)) 
{
    static_assert(std::is_floating_point<T>::value, "ERROR - safe_asin(): template parameter not of type float\n");

    if (std::isnan(v)) {
        return 0.0f;
    }
    if (v >= 1.0f) {
        return M_PI_2;
    }
    if (v <= -1.0f) {
        return -M_PI_2;
    }
    return std::asin(v);
}

/* 
 * A varient of sqrt() that checks the input ranges and ensures a 
 * valid value as output. If a negative number is given then 0 is returned. 
 * The reasoning is that a negative number for sqrt() in our 
 * code is usually caused by small numerical rounding errors, so the 
 * real input should have been zero
 */
template <class T>
auto safe_sqrt(const T v) -> decltype(std::sqrt(v)) 
{
    auto ret = std::sqrt(v);
    if (std::isnan(ret)) {
        return 0;
    }
    return ret;
}

/* 
 * @brief: Constrains an euler angle to be within the range: -180 to 180 degrees
 */
template <class T>
auto wrap_180(const T angle, float unit_mod = 1) -> decltype(std::fmod(angle + 180.f*unit_mod, 360.f*unit_mod)) 
{
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_180(): template parameter not of type float or int\n");
    
    const auto ang_180 = 180.f*unit_mod;
    const auto ang_360 = 360.f*unit_mod;
    auto res = std::fmod(angle + ang_180, ang_360);
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
auto wrap_360(const T angle, float unit_mod = 1) -> decltype(std::fmod(angle, 360.f*unit_mod)) 
{
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_360(): template parameter not of type float or int\n");
    
    const auto ang_360 = 360.f*unit_mod;
    auto res = std::fmod(angle, ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_360_cd(const T angle) -> decltype(wrap_360(angle, 100.f)) 
{
    return wrap_360(angle, 100.f);
}

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_180_cd(const T angle) -> decltype(wrap_180(angle, 100.f)) 
{
    return wrap_180(angle, 100.f);
}

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <class T>
auto wrap_PI(const T radian) -> decltype(std::fmod(radian + M_PI, M_2PI)) 
{
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_PI(): template parameter not of type float or int\n");
    auto res = std::fmod(radian + M_PI, M_2PI);
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
auto wrap_2PI(const T radian) -> decltype(std::fmod(radian, M_2PI)) 
{
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_2PI(): template parameter not of type float or int\n");
    auto res = std::fmod(radian, M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
} 

/*
 * Variadic template for calculating the square number
 */
template<class T>
auto sq(const T val) -> decltype(std::pow(val, 2)) 
{
    return std::pow(val, 2);
}
template<class T, class... Params>
auto sq(const T first, const Params... parameters) -> decltype(std::pow(first, 2)) 
{
    return sq(first) + sq(parameters...);
}

/*
 * Variadic template for calculating the norm (pythagoras)
 */
template<class T, class... Params>
auto norm(const T first, const Params... parameters) -> decltype(std::sqrt(sq(first, parameters...))) 
{
    return std::sqrt(sq(first, parameters...));
}

/*
 * @brief: Constrains a value to be within the range: low and high
 */
template <class T>
T constrain_value(const T amt, const T low, const T high) 
{  
    if (std::isnan(low) || std::isnan(high)) {
        return amt;
    }
    return amt < low ? low : (amt > high ? high : amt);
}

/* 
 * @brief: Converts an euler angle with units 'degree' to an angle with the unit 'radian'
 */
template <class T>
auto radians(const T deg) -> decltype(deg * DEG_TO_RAD) 
{
    return deg * DEG_TO_RAD;
}

/* 
 * @brief: Converts an euler angle with units 'radian' to an angle with the unit 'degree'
 */
template <class T>
auto degrees(const T rad) -> decltype(rad * RAD_TO_DEG) 
{
    return rad * RAD_TO_DEG;
}