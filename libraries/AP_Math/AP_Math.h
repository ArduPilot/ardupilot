#pragma once

#include <limits>
#include <type_traits>
#include <cmath>

#include <stdint.h>

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>

#include "definitions.h"
#include "rotations.h"
#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"
#include "quaternion.h"
#include "polygon.h"
#include "edc.h"
#include "location.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #include "AP_Math_Linux.h"
#else
    #include "AP_Math_PX4.h"
#endif

// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);


/* 
 * @brief: This magic functions swaps the value of the two input parameters
 */
template <class T> 
void swap(T& a, T& b) {
    T c(a); 
    a=b; 
    b=c;
}

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

auto const constrain_float = &constrain_value<float>;
auto const constrain_int16 = &constrain_value<int16_t>;
auto const constrain_int32 = &constrain_value<int32_t>;

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
 * @brief: Gets two values and returns the smaller one.
 */
template<typename A, typename B>
auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two) { 
    return one < two ? one : two;
}

/* 
 * @brief: Gets two values and returns the bigger one.
 */
template<typename A, typename B>
auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two) {  
    return one > two ? one : two;
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

/*
 * MATRIX
 */
//matrix algebra
bool    inverse(float x[], float y[], uint16_t dim);
// invOut is an inverted 4x4 matrix when returns true, otherwise matrix is Singular
bool    inverse3x3(float m[], float invOut[]);
// invOut is an inverted 3x3 matrix when returns true, otherwise matrix is Singular
bool    inverse4x4(float m[],float invOut[]);
// matrix multiplication of two NxN matrices
float   *mat_mul(float *A, float *B, uint8_t n);
//matrix algebra
bool    inverse(float x[], float y[], uint16_t dim);
//
void    mat_LU_decompose(float*, float*, float*, float*, uint8_t);
//
bool    mat_inverse(float*, float*, uint8_t);
//
void    mat_back_sub(float*, float*, uint8_t);
//
void    mat_forward_sub(float *L, float *out, uint8_t n);
//
void    mat_pivot(float* A, float* pivot, uint8_t n);
