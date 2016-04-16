#pragma once

#include "definitions.h"

#include <limits>
#include <type_traits>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include <limits>
#include <type_traits>
#include <cmath>
#include <math.h>
#include <stdint.h>

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
/*#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#undef isfinite
namespace std {
    template<class T>
    bool isfinite(const T& val) {
        static_assert(std::is_floating_point<T>::value, "ERROR - isfinite(): template parameter not of type float\n");
        return (isnan(val) || isinf(val)) ? false : true;
    }
};
#endif*/

// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);

// are two floats equal
static inline bool is_equal(const float fVal1, const float fVal2) { return fabsf(fVal1 - fVal2) < FLT_EPSILON ? true : false; }

// is a float is zero
static inline bool is_zero(const float fVal1) { return fabsf(fVal1) < FLT_EPSILON ? true : false; }

/*
 * A varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is returned.
 */
#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
template <class T>
auto safe_asin(const T &v) -> decltype(std::asin(v)) {
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
#else
template <class T>
float safe_asin(const T &v) {
    static_assert(std::is_floating_point<T>::value, "ERROR - safe_asin(): template parameter not of type float\n");
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
#endif

/* 
 * A varient of sqrt() that checks the input ranges and ensures a 
 * valid value as output. If a negative number is given then 0 is returned. 
 * The reasoning is that a negative number for sqrt() in our 
 * code is usually caused by small numerical rounding errors, so the 
 * real input should have been zero
 */
#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
template <class T>
auto safe_sqrt(const T &v) -> decltype(std::sqrt(v)) {
    auto ret = std::sqrt(v);
    if (std::isnan(ret)) {
        return 0;
    }
    return ret;
}
#else
template <class T>
float safe_sqrt(const T &v) {
    float ret = sqrt(static_cast<float>(v));
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}
#endif

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
#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
template <class T>
auto wrap_180(const T &angle, float unit_mod = 1) -> decltype(std::fmod(angle + 180.f*unit_mod, 360.f*unit_mod)) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_180(): template parameter not of type float or int\n");
    
    const auto ang_180 = 180.f*unit_mod;
    const auto ang_360 = 360.f*unit_mod;
    auto res = std::fmod(angle + ang_180, ang_360);
    if (res < 0 || is_zero(res)) {
        res += ang_180;
    }
    res -= ang_180;
    return res;
}
#else
template <class T>
float wrap_180(const T &angle, float unit_mod = 1) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_180(): template parameter not of type float or int\n");
    
    const float ang_180 = 180.f*unit_mod;
    const float ang_360 = 360.f*unit_mod;
    float res = fmod(static_cast<float>(angle) + ang_180, ang_360);
    if (res < 0 || is_zero(res)) {
        res += ang_180;
    }
    res -= ang_180;
    return res;
}
#endif

/* 
 * @brief: Constrains an euler angle to be within the range: 0 to 360 degrees
 * The second parameter changes the units. Standard: 1 == degrees, 10 == dezi, 100 == centi ..
 */
#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
template <class T>
auto wrap_360(const T &angle, float unit_mod = 1) -> decltype(std::fmod(angle, 360.f*unit_mod)) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_360(): template parameter not of type float or int\n");
    
    const auto ang_360 = 360.f*unit_mod;
    auto res = std::fmod(angle, ang_360);
    if (res < 0) {
        res += ang_360;
    }
    return res;
}
#else
template <class T>
float wrap_360(const T &angle, float unit_mod = 1) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_360(): template parameter not of type float or int\n");
    
    const float ang_360 = 360.f*unit_mod;
    float res = fmod(static_cast<float>(angle), ang_360);
    if (res < 0 || is_zero(res)) {
        res += ang_360;
    }
    return res;
}
#endif


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
 * wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
template <class T>
auto wrap_PI(const T &radian) -> decltype(std::fmod(radian + M_PI, M_2PI)) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_PI(): template parameter not of type float or int\n");
    auto res = std::fmod(radian + M_PI, M_2PI);
    if (res < 0 || is_zero(res)) {
        res += M_PI;
    }
    res -= M_PI;
    return res;
}
#else
template <class T>
float wrap_PI(const T &radian) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_PI(): template parameter not of type float or int\n");
    float res = fmod(radian + static_cast<float>(M_PI), static_cast<float>(M_2PI));
    if (res < 0 || is_zero(res)) {
        res += (float)M_PI;
    }
    res -= (float)M_PI;
    return res;
}
#endif

/*
 * wrap an angle in radians to 0..2PI
 */
#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
template <class T>
auto wrap_2PI(const T &radian) -> decltype(std::fmod(radian, M_2PI)) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_2PI(): template parameter not of type float or int\n");
    auto res = std::fmod(radian, M_2PI);
    if (res < 0.f) {
        res += M_2PI;
    }
    return res;
} 
#else
template <class T>
float wrap_2PI(const T &radian) {
    static_assert(std::is_arithmetic<T>::value, "ERROR - wrap_2PI(): template parameter not of type float or int\n");
    float res = fmod(radian, static_cast<float>(M_2PI));
    if (res < 0.f) {
        res += (float)M_2PI;
    }
    return res;
} 
#endif

/*
 * @brief: Constrains a value to be within the range: low and high
 */
#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
template <class T>
T constrain_value(const T &amt, const T &low, const T &high) {  
    if (std::isnan(low) || std::isnan(high)) {
        return amt;
    }
    return amt < low ? low : (amt > high ? high : amt);
}
#else
template <class T>
T constrain_value(const T &amt, const T &low, const T &high) {  
    if (isnan(low) || isnan(high)) {
        return amt;
    }
    return amt < low ? low : (amt > high ? high : amt);
}
#endif

auto const constrain_float = &constrain_value<float>;
auto const constrain_int16 = &constrain_value<int16_t>;
auto const constrain_int32 = &constrain_value<int32_t>;

//matrix algebra
bool inverse(float x[], float y[], uint16_t dim);

// degrees -> radians
static inline float radians(float deg) {
	return deg * DEG_TO_RAD;
}

// radians -> degrees
static inline float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
template<class T>
auto sq(const T &val) -> decltype(std::pow(val, 2)) {
    return std::pow(val, 2);
}
template<class T, class... Params>
auto sq(const T &first, const Params&... parameters) -> decltype(std::pow(first, 2)) {
    return sq(first) + sq(parameters...);
}
template<class T, class... Params>
auto norm(const T &first, const Params&... parameters) -> decltype(std::sqrt(sq(first, parameters...))) {
    return std::sqrt(sq(first, parameters...));
}
#else
template<class T>
float sq(const T &val) {
    return std::pow(static_cast<float>(val), 2);
}
template<class T, class... Params>
float sq(const T &first, const Params&... parameters) {
    return sq(first) + sq(parameters...);
}
template<class T, class... Params>
float norm(const T &first, const Params&... parameters) {
    return std::sqrt(static_cast<float>(sq(first, parameters...)));
}
#endif

template<typename A, typename B>
static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two) {
    return one < two ? one : two;
}

template<typename A, typename B>
static inline auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two) {
    return one > two ? one : two;
}

inline uint32_t hz_to_nsec(uint32_t freq)
{
    return NSEC_PER_SEC / freq;
}

inline uint32_t nsec_to_hz(uint32_t nsec)
{
    return NSEC_PER_SEC / nsec;
}

inline uint32_t usec_to_nsec(uint32_t usec)
{
    return usec * NSEC_PER_USEC;
}

inline uint32_t nsec_to_usec(uint32_t nsec)
{
    return nsec / NSEC_PER_USEC;
}

inline uint32_t hz_to_usec(uint32_t freq)
{
    return USEC_PER_SEC / freq;
}

inline uint32_t usec_to_hz(uint32_t usec)
{
    return USEC_PER_SEC / usec;
}

/*
  linear interpolation based on a variable in a range
 */
float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high);

