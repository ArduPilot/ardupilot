#pragma once

/*
  allow for builds with either single or double precision EKF
 */

#include <AP_HAL/AP_HAL.h>
#include <float.h>

/*
  capital F is used to denote the chosen float type (float or double)
 */

#if HAL_WITH_EKF_DOUBLE
typedef double ftype;
#define sinF(x) sin(x)
#define acosF(x) acos(x)
#define asinF(x) asin(x)
#define cosF(x) cos(x)
#define tanF(x) tan(x)
#define atanF(x) atan(x)
#define atan2F(x,y) atan2(x,y)
#define sqrtF(x) sqrt(x)
#define fmaxF(x,y) fmax(x,y)
#define powF(x,y) pow(x,y)
#define logF(x) log(x)
#define fabsF(x) fabs(x)
#define ceilF(x) ceil(x)
#define fminF(x,y) fmin(x,y)
#define fmodF(x,y) fmod(x,y)
#define fabsF(x) fabs(x)
#define toftype todouble
#else
typedef float ftype;
#define acosF(x) acosf(x)
#define asinF(x) asinf(x)
#define sinF(x) sinf(x)
#define cosF(x) cosf(x)
#define tanF(x) tanf(x)
#define atanF(x) atanf(x)
#define atan2F(x,y) atan2f(x,y)
#define sqrtF(x) sqrtf(x)
#define fmaxF(x,y) fmaxf(x,y)
#define powF(x,y) powf(x,y)
#define logF(x) logf(x)
#define fabsF(x) fabsf(x)
#define ceilF(x) ceilf(x)
#define fminF(x,y) fminf(x,y)
#define fmodF(x,y) fmodf(x,y)
#define fabsF(x) fabsf(x)
#define toftype tofloat
#endif

#if MATH_CHECK_INDEXES
#define ZERO_FARRAY(a) a.zero()
#else
#define ZERO_FARRAY(a) memset(a, 0, sizeof(a))
#endif

/*
 * @brief: Check whether a float is zero
 */
inline bool is_zero(const float x) {
    return fabsf(x) < FLT_EPSILON;
}

/*
 * @brief: Check whether a double is zero
 */
inline bool is_zero(const double x) {
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
  return fabs(x) < FLT_EPSILON;
#else
  return fabsf(static_cast<float>(x)) < FLT_EPSILON;
#endif
}
