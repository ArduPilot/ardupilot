#pragma once

#include <cmath>
#include <limits>
#include <stdint.h>
#include <type_traits>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include "definitions.h"
#include "crc.h"
#include "matrix3.h"
#include "polygon.h"
#include "quaternion.h"
#include "rotations.h"
#include "vector2.h"
#include "vector3.h"
#include "spline5.h"
#include "location.h"

// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);

/*
 * Check whether two floats are equal
 */
template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_integral<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value ,bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);

template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_floating_point<typename std::common_type<Arithmetic1, Arithmetic2>::type>::value, bool>::type
is_equal(const Arithmetic1 v_1, const Arithmetic2 v_2);

/* 
 * @brief: Check whether a float is zero
 */
template <typename T>
inline bool is_zero(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return (fabsf(static_cast<float>(fVal1)) < FLT_EPSILON);
}

/* 
 * @brief: Check whether a float is greater than zero
 */
template <typename T>
inline bool is_positive(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) >= FLT_EPSILON);
}


/* 
 * @brief: Check whether a float is less than zero
 */
template <typename T>
inline bool is_negative(const T fVal1) {
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
                  "Template parameter not of type float");
    return (static_cast<float>(fVal1) <= (-1.0 * FLT_EPSILON));
}


/*
 * A variant of asin() that checks the input ranges and ensures a valid angle
 * as output. If nan is given as input then zero is returned.
 */
template <typename T>
float safe_asin(const T v);

/*
 * A variant of sqrt() that checks the input ranges and ensures a valid value
 * as output. If a negative number is given then 0 is returned.  The reasoning
 * is that a negative number for sqrt() in our code is usually caused by small
 * numerical rounding errors, so the real input should have been zero
 */
template <typename T>
float safe_sqrt(const T v);

// invOut is an inverted 4x4 matrix when returns true, otherwise matrix is Singular
bool inverse3x3(float m[], float invOut[]) WARN_IF_UNUSED;

// invOut is an inverted 3x3 matrix when returns true, otherwise matrix is Singular
bool inverse4x4(float m[],float invOut[]) WARN_IF_UNUSED;

// matrix multiplication of two NxN matrices
float *mat_mul(float *A, float *B, uint8_t n);

// matrix algebra
bool inverse(float x[], float y[], uint16_t dim) WARN_IF_UNUSED;

/*
 * Constrain an angle to be within the range: -180 to 180 degrees. The second
 * parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
template <typename T>
T wrap_180(const T angle);

/*
 * Wrap an angle in centi-degrees. See wrap_180().
 */
template <typename T>
T wrap_180_cd(const T angle);

/*
 * Constrain an euler angle to be within the range: 0 to 360 degrees. The
 * second parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
float wrap_360(const float angle);
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
double wrap_360(const double angle);
#endif
int wrap_360(const int angle);

int wrap_360_cd(const int angle);
long wrap_360_cd(const long angle);
float wrap_360_cd(const float angle);
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
double wrap_360_cd(const double angle);
#endif


/*
  wrap an angle in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <typename T>
float wrap_PI(const T radian);

/*
 * wrap an angle in radians to 0..2PI
 */
template <typename T>
float wrap_2PI(const T radian);

/*
 * Constrain a value to be within the range: low and high
 */
template <typename T>
T constrain_value(const T amt, const T low, const T high);

inline float constrain_float(const float amt, const float low, const float high)
{
    return constrain_value(amt, low, high);
}

inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high)
{
    return constrain_value(amt, low, high);
}

inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high)
{
    return constrain_value(amt, low, high);
}

inline int64_t constrain_int64(const int64_t amt, const int64_t low, const int64_t high)
{
    return constrain_value(amt, low, high);
}

// degrees -> radians
static inline constexpr float radians(float deg)
{
    return deg * DEG_TO_RAD;
}

// radians -> degrees
static inline constexpr float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

template<typename T>
float sq(const T val)
{
    float v = static_cast<float>(val);
    return v*v;
}

/*
 * Variadic template for calculating the square norm of a vector of any
 * dimension.
 */
template<typename T, typename... Params>
float sq(const T first, const Params... parameters)
{
    return sq(first) + sq(parameters...);
}

/*
 * Variadic template for calculating the norm (pythagoras) of a vector of any
 * dimension.
 */
template<typename T, typename U, typename... Params>
float norm(const T first, const U second, const Params... parameters)
{
    return sqrtf(sq(first, second, parameters...));
}

template<typename A, typename B>
static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two)
{
    return one < two ? one : two;
}

template<typename A, typename B>
static inline auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two)
{
    return one > two ? one : two;
}

inline constexpr uint32_t hz_to_nsec(uint32_t freq)
{
    return AP_NSEC_PER_SEC / freq;
}

inline constexpr uint32_t nsec_to_hz(uint32_t nsec)
{
    return AP_NSEC_PER_SEC / nsec;
}

inline constexpr uint32_t usec_to_nsec(uint32_t usec)
{
    return usec * AP_NSEC_PER_USEC;
}

inline constexpr uint32_t nsec_to_usec(uint32_t nsec)
{
    return nsec / AP_NSEC_PER_USEC;
}

inline constexpr uint32_t hz_to_usec(uint32_t freq)
{
    return AP_USEC_PER_SEC / freq;
}

inline constexpr uint32_t usec_to_hz(uint32_t usec)
{
    return AP_USEC_PER_SEC / usec;
}

/*
  linear interpolation based on a variable in a range
 */
float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high);

/* cubic "expo" curve generator 
 * alpha range: [0,1] min to max expo
 * input range: [-1,1]
 */
constexpr float expo_curve(float alpha, float input);

/* throttle curve generator
 * thr_mid: output at mid stick
 * alpha: expo coefficient
 * thr_in: [0-1]
 */
float throttle_curve(float thr_mid, float alpha, float thr_in);

/* simple 16 bit random number generator */
uint16_t get_random16(void);

// generate a random float between -1 and 1, for use in SITL
float rand_float(void);

// generate a random Vector3f of size 1
Vector3f rand_vec3f(void);

// return true if two rotations are equal
bool rotation_equal(enum Rotation r1, enum Rotation r2) WARN_IF_UNUSED;

/*
 * return a velocity correction (in m/s in NED) for a sensor's position given it's position offsets
 * this correction should be added to the sensor NED measurement
 * sensor_offset_bf is in meters in body frame (Foward, Right, Down)
 * rot_ef_to_bf is a rotation matrix to rotate from earth-frame (NED) to body frame
 * angular_rate is rad/sec
 */
Vector3f get_vel_correction_for_sensor_offset(const Vector3f &sensor_offset_bf, const Matrix3f &rot_ef_to_bf, const Vector3f &angular_rate);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// fill an array of float with NaN, used to invalidate memory in SITL
void fill_nanf(float *f, uint16_t count);
#endif

