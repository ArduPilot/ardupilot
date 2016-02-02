// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_MATH_H
#define AP_MATH_H

// Assorted useful math operations for ArduPilot(Mega)

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include <limits>
#include <type_traits>
#include <cmath>

#include <math.h>
#include <stdint.h>

#include "definitions.h"

#include "rotations.h"
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"
#include "quaternion.h"
#include "polygon.h"
#include "edc.h"
#include <AP_Param/AP_Param.h>
#include "location.h"


/*
 * There is a macro mismatch and the std::function doesn't exist @PX4
 * We create our own std::standard function for PX4
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#undef isfinite
namespace std {
    template<class T>
    bool isfinite(const T& val) {
        static_assert(std::is_floating_point<T>::value, "ERROR - isfinite(): template parameter not of type float\n");
        return (isnan(val) || isinf(val)) ? false : true;
    }
};
#endif

// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);

/* 
 * @brief: Checks whether two floats are equal
 */
template <class FloatOne, class FloatTwo>
inline bool is_equal(const FloatOne fVal1, const FloatTwo fVal2) {
    static_assert((std::is_floating_point<FloatOne>::value || std::is_base_of<FloatOne,AP_Float>::value), "ERROR - is_equal(): template parameters not of type float\n");
    static_assert((std::is_floating_point<FloatTwo>::value || std::is_base_of<FloatTwo,AP_Float>::value), "ERROR - is_equal(): template parameters not of type float\n");
    
    return fabsf(fVal1 - fVal2) < std::numeric_limits<decltype(fVal1 - fVal2)>::epsilon() ? true : false; 
}

/* 
 * @brief: Checks whether a float is zero
 */
template <class T>
inline bool is_zero(const T fVal1) { 
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value, "ERROR - is_zero(): template parameter not of type float\n");

    return fabsf(fVal1) < std::numeric_limits<T>::epsilon() ? true : false; 
}

/*
 * A varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is returned.
 */
template <class FloatType>
FloatType safe_asin(const FloatType &v) {
    static_assert(std::is_floating_point<FloatType>::value, "ERROR - safe_asin(): template parameter not of type float\n");

#if CONFIG_HAL_BOARD != HAL_BOARD_PX4
    if (std::isnan(v)) {
        return 0.0f;
    }
#else
    if (isnan(v)) {
        return 0.0f;
    }
#endif
    if (v >= 1.0f) {
        return M_PI_2;
    }
    if (v <= -1.0f) {
        return -M_PI_2;
    }
    return asinf(v);
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
    float ret = std::sqrt(static_cast<float>(v));
#if CONFIG_HAL_BOARD != HAL_BOARD_PX4
    if (std::isnan(ret)) {
        return 0;
    }
#else
    if (isnan(ret)) {
        return 0;
    }
#endif
    return ret;
}

// return distance in meters between two locations
float                   get_distance(const struct Location &loc1, const struct Location &loc2);

// return distance in centimeters between two locations
uint32_t                get_distance_cm(const struct Location &loc1, const struct Location &loc2);

// return bearing in centi-degrees between two locations
int32_t                 get_bearing_cd(const struct Location &loc1, const struct Location &loc2);

// return determinant of square matrix
float                   detnxn(const float C[], const uint8_t n);

// Output inverted nxn matrix when returns true, otherwise matrix is Singular
bool                    inversenxn(const float x[], float y[], const uint8_t n);

// invOut is an inverted 4x4 matrix when returns true, otherwise matrix is Singular
bool                    inverse3x3(float m[], float invOut[]);

// invOut is an inverted 3x3 matrix when returns true, otherwise matrix is Singular
bool                    inverse4x4(float m[],float invOut[]);

// matrix multiplication of two NxN matrices
float* mat_mul(float *A, float *B, uint8_t n);

/* 
 * @brief: Constrains an euler angle to be within the range: -180 to 180 degrees
 */
template <class T>
auto wrap_180(const T &angle, float unit_mod = 1) -> decltype(std::fmod(angle + 180.f*unit_mod, 360.f*unit_mod)) {
    static_assert(std::is_floating_point<T>::value || std::is_integral<T>::value, "ERROR - wrap_180(): template parameter not of type float or int\n");
    
    const auto ang_180 = 180.f*unit_mod;
    const auto ang_360 = 360.f*unit_mod;
    auto res = std::fmod(angle + ang_180, ang_360);
    if (res < 0.f) {
        res += (decltype(res))ang_360;
    }
    res -= (decltype(res))ang_180;
    return res;
}

/* 
 * @brief: Constrains an euler angle to be within the range: 0 to 360 degrees
 * The second parameter changes the units. Standard: 1 == degrees, 10 == dezi, 100 == centi ..
 */
template <class T>
auto wrap_360(const T &angle, float unit_mod = 1) -> decltype(std::fmod(angle, 360.f*unit_mod)) {
    static_assert(std::is_floating_point<T>::value || std::is_integral<T>::value, "ERROR - wrap_360(): template parameter not of type float or int\n");
    
    const auto ang_360 = 360.f*unit_mod;
    auto res = std::fmod(angle, ang_360);
    if (res < 0.f) {
        res += (decltype(res))ang_360;
    }
    return res;
}

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_360_cd(const T &angle) -> decltype(wrap_360(angle, 100.f)) {
    return wrap_360(angle, 100.f);
}

/*
 * Wrap an angle in centi-degrees
 */
template <class T>
auto wrap_180_cd(const T &angle) -> decltype(wrap_180(angle, 100.f)) {
    return wrap_180(angle, 100.f);
}

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <class T>
auto wrap_PI(const T &radian) -> decltype(std::fmod(radian + static_cast<float>(M_PI), static_cast<float>(M_2_PI))) {
    static_assert(std::is_floating_point<T>::value || std::is_integral<T>::value, "ERROR - wrap_PI(): template parameter not of type float or int\n");
    auto res = std::fmod(radian + static_cast<float>(M_PI), static_cast<float>(M_2_PI));
    if (res < 0.f) {
        res += (decltype(res))M_2_PI;
    }
    res -= (decltype(res))M_PI;
    return res;
}

/*
 * wrap an angle in radians to 0..2PI
 */
template <class T>
auto wrap_2PI(const T &radian) -> decltype(std::fmod(radian, static_cast<float>(M_2_PI))) {
    static_assert(std::is_floating_point<T>::value || std::is_integral<T>::value, "ERROR - wrap_2PI(): template parameter not of type float or int\n");
    auto res = std::fmod(radian, static_cast<float>(M_2_PI));
    if (res < 0.f) {
        res += (decltype(res))M_2_PI;
    }
    return res;
}

/*
 * @brief: Constrains a value to be within the range: low and high
 */
template <class T>
T constrain_value(const T &amt, const T &low, const T &high) {  
#if CONFIG_HAL_BOARD != HAL_BOARD_PX4
    if (std::isnan(low) || std::isnan(high)) {
        return amt;
    }
#else
    if (isnan(low) || isnan(high)) {
        return amt;
    }
#endif
    return amt < low ? low : (amt > high ? high : amt);
}

auto const constrain_float = &constrain_value<float>;
auto const constrain_int16 = &constrain_value<int16_t>;
auto const constrain_int32 = &constrain_value<int32_t>;

//matrix algebra
bool inverse(float x[], float y[], uint16_t dim);

/*
 * WE COULD MAKE AP_MATH BETTER! PLZ take care about the types in ArduPilot,
 * and we can make these functions auto
 * 
 * Calculate the squared distance as sum of each squared parameter
 * @return: Return type currently forced to be float, 
 * because AP_Int/AP_Float types yield a compile time error.
 */
template<class T>
float sq(const T &val) {
    return std::pow(static_cast<float>(val), 2);
}

template<class T, class... Params>
float sq(const T &first, const Params&... parameters) {
    return sq(first) + sq(parameters...);
}

/*
 * Calculates the norm: sqrt(val1²+val2²+val3²+..)
 */
template<class T, class... Params>
auto norm(const T &first, const Params&... parameters) -> decltype(std::sqrt(sq(first, parameters...))) {
    return std::sqrt(sq(first, parameters...));
}

template<typename A, typename B>
static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two) {
    return one < two ? one : two;
}

template<typename A, typename B>
static inline auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two) {
    return one > two ? one : two;
}

/* 
 * @brief: Converts an euler angle with units 'degree' to an angle with the unit 'radian'
 */
template <class T>
float radians(const T &deg) {
    return static_cast<float>(deg) * DEG_TO_RAD;
}

/* 
 * WE COULD MAKE AP_MATH BETTER! PLZ take care about the types in ArduPilot,
 * and we can make these functions auto. 
 * These functions are currently often used with int types which causes a double promotion and no one cares ..
 * 
 * @brief: Converts an euler angle with units 'radian' to an angle with the unit 'degree'
 */
template <class T>
float degrees(const T &rad) {
    return static_cast<float>(rad) * RAD_TO_DEG;
}

/*
 * Converter functions
 *  - Avoid zero divisions
 *  - Inheritss a float cast (because of PX4)
 */
template<class T>
T hz_to_nsec(const T &freq) {
    T val = !is_zero(static_cast<float>(freq)) ? NSEC_PER_SEC / freq : 0;
    return val;
}

template<class T>
T nsec_to_hz(const T &nsec) {
    T val = !is_zero(static_cast<float>(nsec)) ? NSEC_PER_SEC / nsec : 0;
    return val;
}

template<class T>
T usec_to_nsec(const T &usec) {
    T val = usec * NSEC_PER_USEC;
    return val;
}

template<class T>
T nsec_to_usec(const T &nsec) {
    T val = !is_zero(static_cast<float>(nsec)) ? nsec / NSEC_PER_USEC : 0;
    return val;
}

template<class T>
T hz_to_usec(const T &freq) {
    T val = !is_zero(static_cast<float>(freq)) ? USEC_PER_SEC / freq : 0;
    return val;
}

template<class T>
T usec_to_hz(const T &usec) {
    T val = !is_zero(static_cast<float>(usec)) ? USEC_PER_SEC / usec : 0;
    return val;
}

#endif // AP_MATH_H

