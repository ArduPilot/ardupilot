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
/*
#undef isfinite
namespace std {
    template<class T>
    bool isfinite(const T val) {
        static_assert(std::is_floating_point<T>::value, "ERROR - isfinite(): template parameter not of type float\n");
        return (isnan(val) || isinf(val)) ? false : true;
    }
};
*/
/* 
 * @brief: Checks whether two floats are equal
 */
template <class FloatOne, class FloatTwo>
bool is_equal(const FloatOne, const FloatTwo);

/* 
 * @brief: Checks whether a float is zero
 */
template <class T>
bool is_zero(const T);

/*
 * A varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is returned.
 */
template <class T>
float safe_asin(const T v);

/* 
 * A varient of sqrt() that checks the input ranges and ensures a 
 * valid value as output. If a negative number is given then 0 is returned. 
 * The reasoning is that a negative number for sqrt() in our 
 * code is usually caused by small numerical rounding errors, so the 
 * real input should have been zero
 */
template <class T>
float safe_sqrt(const T v);

/* 
 * @brief: Constrains an euler angle to be within the range: -180 to 180 degrees
 */
template <class T>
float wrap_180(const T angle, float unit_mod = 1);

/* 
 * @brief: Constrains an euler angle to be within the range: 0 to 360 degrees
 * The second parameter changes the units. Standard: 1 == degrees, 10 == dezi, 100 == centi ..
 */
template <class T>
float wrap_360(const T angle, float unit_mod = 1);

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_360_cd(const T angle) -> decltype(wrap_360(angle, 100.f));

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_180_cd(const T angle) -> decltype(wrap_180(angle, 100.f));

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <class T>
float wrap_PI(const T radian);

/*
 * wrap an angle in radians to 0..2PI
 */
template <class T>
float wrap_2PI(const T radian);

/*
 * Variadic template for calculating the square number
 */
template<class T>
float sq(const T val) {
    return powf(static_cast<float>(val), 2);
}
template<class T, class... Params>
float sq(const T first, const Params... parameters) {
    return sq(first) + sq(parameters...);
}

/*
 * Variadic template for calculating the norm (pythagoras)
 */
template<class T, class... Params>
float norm(const T first, const Params... parameters) {
    return sqrt(static_cast<float>(sq(first, parameters...)));
}

/*
 * @brief: Constrains a value to be within the range: low and high
 */
template <class T>
T constrain_value(const T amt, const T low, const T high);

/* 
 * @brief: Converts an euler angle with units 'degree' to an angle with the unit 'radian'
 */
template <class T>
float radians(const T deg) {
    return static_cast<float>(deg) * DEG_TO_RAD;
}

/* 
 * @brief: Converts an euler angle with units 'radian' to an angle with the unit 'degree'
 */
template <class T>
float degrees(const T rad) {
    return static_cast<float>(rad) * RAD_TO_DEG;
}

