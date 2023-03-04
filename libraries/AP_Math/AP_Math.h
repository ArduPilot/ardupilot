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
#include "control.h"

#if HAL_WITH_EKF_DOUBLE
typedef Vector2<double> Vector2F;
typedef Vector3<double> Vector3F;
typedef Matrix3<double> Matrix3F;
typedef QuaternionD QuaternionF;
#else
typedef Vector2<float> Vector2F;
typedef Vector3<float> Vector3F;
typedef Matrix3<float> Matrix3F;
typedef Quaternion QuaternionF;
#endif

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
    return is_zero(static_cast<float>(fVal1));
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
 * @brief: Check whether a double is greater than zero
 */
inline bool is_positive(const double fVal1) {
    return (fVal1 >= static_cast<double>(FLT_EPSILON));
}

/*
 * @brief: Check whether a double is less than zero
 */
inline bool is_negative(const double fVal1) {
    return (fVal1 <= static_cast<double>((-1.0 * FLT_EPSILON)));
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

// matrix multiplication of two NxN matrices
template <typename T>
void mat_mul(const T *A, const T *B, T *C, uint16_t n);

// matrix inverse
template <typename T>
bool mat_inverse(const T *x, T *y, uint16_t dim) WARN_IF_UNUSED;

// matrix identity
template <typename T>
void mat_identity(T *x, uint16_t dim);

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
ftype wrap_PI(const ftype radian);

/*
 * wrap an angle in radians to 0..2PI
 */
ftype wrap_2PI(const ftype radian);

/*
 * Constrain a value to be within the range: low and high
 */
template <typename T>
T constrain_value(const T amt, const T low, const T high);

template <typename T>
T constrain_value_line(const T amt, const T low, const T high, uint32_t line);

#define constrain_float(amt, low, high) constrain_value_line(float(amt), float(low), float(high), uint32_t(__AP_LINE__))
#define constrain_ftype(amt, low, high) constrain_value_line(ftype(amt), ftype(low), ftype(high), uint32_t(__AP_LINE__))

inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high)
{
    return constrain_value(amt, low, high);
}

inline uint16_t constrain_uint16(const uint16_t amt, const uint16_t low, const uint16_t high)
{
    return constrain_value(amt, low, high);
}

inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high)
{
    return constrain_value(amt, low, high);
}

inline uint32_t constrain_uint32(const uint32_t amt, const uint32_t low, const uint32_t high)
{
    return constrain_value(amt, low, high);
}

inline int64_t constrain_int64(const int64_t amt, const int64_t low, const int64_t high)
{
    return constrain_value(amt, low, high);
}

inline uint64_t constrain_uint64(const uint64_t amt, const uint64_t low, const uint64_t high)
{
    return constrain_value(amt, low, high);
}

inline double constrain_double(const double amt, const double low, const double high)
{
    return constrain_value(amt, low, high);
}

// degrees -> radians
static inline constexpr ftype radians(ftype deg)
{
    return deg * DEG_TO_RAD;
}

// radians -> degrees
static inline constexpr float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

template<typename T>
ftype sq(const T val)
{
    ftype v = static_cast<ftype>(val);
    return v*v;
}
static inline constexpr float sq(const float val)
{
    return val*val;
}

/*
 * Variadic template for calculating the square norm of a vector of any
 * dimension.
 */
template<typename T, typename... Params>
ftype sq(const T first, const Params... parameters)
{
    return sq(first) + sq(parameters...);
}

/*
 * Variadic template for calculating the norm (pythagoras) of a vector of any
 * dimension.
 */
template<typename T, typename U, typename... Params>
ftype norm(const T first, const U second, const Params... parameters)
{
    return sqrtF(sq(first, second, parameters...));
}

#undef MIN
template<typename A, typename B>
static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two)
{
    return one < two ? one : two;
}

#undef MAX
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
  return value will be in the range [var_low,var_high]

  Either polarity is supported, so var_low can be higher than var_high
 */
float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high);

/* cubic "expo" curve generator 
 * alpha range: [0,1] min to max expo
 * input range: [-1,1]
 */
float expo_curve(float alpha, float input);

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

// generate a random Vector3f with each value between -1.0 and 1.0
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
Vector3F get_vel_correction_for_sensor_offset(const Vector3F &sensor_offset_bf, const Matrix3F &rot_ef_to_bf, const Vector3F &angular_rate);

/*
  calculate a low pass filter alpha value
 */
float calc_lowpass_alpha_dt(float dt, float cutoff_freq);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// fill an array of float with NaN, used to invalidate memory in SITL
void fill_nanf(float *f, uint16_t count);
void fill_nanf(double *f, uint16_t count);
#endif

// from https://embeddedartistry.com/blog/2018/07/12/simple-fixed-point-conversion-in-c/
// Convert to/from 16-bit fixed-point and float
float fixed2float(const uint16_t input, const uint8_t fractional_bits = 8);
uint16_t float2fixed(const float input, const uint8_t fractional_bits = 8);

/*
  calculate turn rate in deg/sec given a bank angle and airspeed for a
  fixed wing aircraft
 */
float fixedwing_turn_rate(float bank_angle_deg, float airspeed);

// convert degrees farenheight to Kelvin
float degF_to_Kelvin(float temp_f);

/*
  conversion functions to prevent undefined behaviour
 */
int16_t float_to_int16(const float v);
uint16_t float_to_uint16(const float v);
int32_t float_to_int32(const float v);
uint32_t float_to_uint32(const float v);
uint32_t double_to_uint32(const double v);
int32_t double_to_int32(const double v);

