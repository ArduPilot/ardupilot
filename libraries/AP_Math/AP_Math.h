#pragma once

#include "definitions.h"

#include <limits>
#include <type_traits>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <cmath>
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


// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);

// are two floats equal
static inline bool is_equal(const float fVal1, const float fVal2) { return fabsf(fVal1 - fVal2) < FLT_EPSILON ? true : false; }

// is a float is zero
static inline bool is_zero(const float fVal1) { return fabsf(fVal1) < FLT_EPSILON ? true : false; }

// a varient of asin() that always gives a valid answer.
float           safe_asin(float v);

// a varient of sqrt() that always gives a valid answer.
float           safe_sqrt(float v);

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
 * Constrain an angle to be within the range: -180 to 180 degrees. The second
 * parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
template <class T>
float wrap_180(const T angle, float unit_mod = 1);

/*
 * Wrap an angle in centi-degrees. See wrap_180().
 */
template <class T>
auto wrap_180_cd(const T angle) -> decltype(wrap_180(angle, 100.f));

/*
 * Constrain an euler angle to be within the range: 0 to 360 degrees. The
 * second parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
template <class T>
float wrap_360(const T angle, float unit_mod = 1);

/*
 * Wrap an angle in centi-degrees. See wrap_360().
 */
template <class T>
auto wrap_360_cd(const T angle) -> decltype(wrap_360(angle, 100.f));

/*
  wrap an angle in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <class T>
float wrap_PI(const T radian);

/*
 * wrap an angle in radians to 0..2PI
 */
template <class T>
float wrap_2PI(const T radian);

/*
 * Constrain a value to be within the range: low and high
 */
template <class T>
T constrain_value(const T amt, const T low, const T high);

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

template<class T>
float sq(const T val)
{
    return powf(static_cast<float>(val), 2);
}

/*
 * Variadic template for calculating the square norm of a vector of any
 * dimension.
 */
template<class T, class... Params>
float sq(const T first, const Params... parameters)
{
    return sq(first) + sq(parameters...);
}

/*
 * Variadic template for calculating the norm (pythagoras) of a vector of any
 * dimension.
 */
template<class T, class... Params>
float norm(const T first, const Params... parameters)
{
    return sqrt(static_cast<float>(sq(first, parameters...)));
}

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

