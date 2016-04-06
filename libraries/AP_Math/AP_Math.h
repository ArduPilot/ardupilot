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
  wrap an angle in centi-degrees
 */
int32_t wrap_360_cd(int32_t error);
int32_t wrap_180_cd(int32_t error);
float wrap_360_cd_float(float angle);
float wrap_180_cd_float(float angle);

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
float wrap_PI(float angle_in_radians);

/*
  wrap an angle defined in radians to the interval [0,2*PI)
 */
float wrap_2PI(float angle);

// constrain a value
// constrain a value
static inline float constrain_float(float amt, float low, float high)
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
// constrain a int16_t value
static inline int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int32_t value
static inline int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

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

// square
static inline float sq(float v) {
	return v*v;
}

// 2D vector length
static inline float pythagorous2(float a, float b) {
	return sqrtf(sq(a)+sq(b));
}

// 3D vector length
static inline float pythagorous3(float a, float b, float c) {
	return sqrtf(sq(a)+sq(b)+sq(c));
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

