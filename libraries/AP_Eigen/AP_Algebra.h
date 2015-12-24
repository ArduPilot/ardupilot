/*
 * @brief: Simple ArduPilot related algebra template library
 * @authors: Daniel Frenzel <dgdanielf@gmail.com>
 */

#ifndef AP_ALGEBRA_H
#define AP_ALGEBRA_H

#include <limits>
#include <type_traits>
#include <cmath>
#include <initializer_list>


/*
 * A varient of asin() that checks the input ranges and ensures a
 * valid angle as output. If nan is given as input then zero is returned.
 */
template <class FloatType>
FloatType safe_asin(const FloatType &v) {
    static_assert(std::is_floating_point<FloatType>::value, 
        "ERROR - safe_asin(): template parameter not of type float\n");

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
template <class FloatType>
FloatType safe_sqrt(const FloatType &v) {
    static_assert(std::is_floating_point<FloatType>::value, 
        "ERROR - safe_sqrt(): template parameter not of type float\n");

    FloatType ret = std::sqrt(v);
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

/*
 * Calculated the euclidian distance of the obtained parameter list
 */
template<class T>
T squared_distance(const T &val) {
    static_assert(std::is_floating_point<T>::value || std::is_integral<T>::value, 
        "ERROR - squared_distance(): template parameter not of type integer or float\n");
    
    return std::pow(val, 2);
}

template<class T, class... Params> 
T squared_distance(const T &first, const Params&... parameters) {
    return std::pow(first, 2) + squared_distance(parameters...);
}

/*
 * @brief: Safe way to convert floating point numbers to integers
 */
template <class IntType, class FloatType>
static inline IntType safeFloatToInt(const FloatType &num) {
    //check if float fits into integer
    if (std::numeric_limits<IntType>::digits > std::numeric_limits<FloatType>::digits) {
        return static_cast<IntType>(num);
    }

    // check if float is smaller than max int
    if (num < static_cast<FloatType>(std::numeric_limits<IntType>::max()) &&
       num > static_cast<FloatType>(std::numeric_limits<IntType>::min()))
    {
        return static_cast<IntType>(num); //safe to cast
    }

    //NaN is not defined for int return the largest int value
    //printf("CRITICAL - safeFloatToInt(): Unsafe conversion of value: %f\n", num);
    return num < 0 ? std::numeric_limits<IntType>::min() : std::numeric_limits<IntType>::max();
}

/*
 * @brief: Constrains a value to be within the range: low and high
 */
template <class T>
static inline T constrain_value(T amt, T low, T high) {  
#if CONFIG_HAL_BOARD != HAL_BOARD_PX4
    if (std::isnan(low) || std::isnan(high)) {
        //printf("CRITICAL - constrain_value(): One or more parameters are NaN\n");
        return (low+high) * 0.5f;
    }
#else
    if (isnan(low) || isnan(high)) {
        //printf("CRITICAL - constrain_value(): One or more parameters are NaN\n");
        return (low+high) * 0.5f;
    }
#endif
    return amt < low ? low : (amt > high ? high : amt);
}

/* 
 * @brief: Checks whether two floats are equal
 */
template <class FloatOne, class FloatTwo>
static inline bool is_equal(const FloatOne fVal1, const FloatTwo fVal2) {
    static_assert(std::is_floating_point<FloatOne>::value && std::is_floating_point<FloatTwo>::value, 
        "ERROR - is_equal(): template parameters not of type float\n");
    
    return fabsf(fVal1 - fVal2) < std::numeric_limits<decltype(fVal1 - fVal2)>::epsilon() ? true : false; 
}

/* 
 * @brief: Checks whether a float is zero
 */
template <class T>
static inline bool is_zero(const T fVal1) { 
    static_assert(std::is_floating_point<T>::value,
        "ERROR - is_zero(): template parameter not of type float\n");

    return fabsf(fVal1) < std::numeric_limits<T>::epsilon() ? true : false; 
}

/* 
 * @brief: Constrains an euler angle to be within the range: -180 to 180 degrees
 */
template <class T>
static inline T constrain_euler_180(const T &angle) {
    static_assert(std::is_floating_point<T>::value || std::is_integral<T>::value, 
        "ERROR - constrain_euler_180(): template parameter not of type float or int\n");
    
    const T _ang_180 = T(180.f);
    const T _ang_360 = T(360.f);
    
    T res = std::fmod(angle + _ang_180, _ang_360);
    if (res < 0.f) {
        res += _ang_360;
    }
    res -= _ang_180;
    return res;
}

/* 
 * @brief: Constrains an euler angle to be within the range: 0 to 360 degrees
 */
template <class T>
static inline T constrain_euler_360(const T &angle) {
    static_assert(std::is_floating_point<T>::value || std::is_integral<T>::value, 
        "ERROR - constrain_euler_360(): template parameter not of type float or int\n");
  
    const T _ang_360 = T(360.f);
    
    T res = std::fmod(angle, _ang_360);
    if (res < 0.f) {
        res += _ang_360;
    }
    return res;
}

/* 
 * @brief: Converts an euler angle with units 'degree' to an angle with the unit 'radian'
 */
template <class T>
static inline T to_radian(const T &deg, const int &flag) {
    static_assert(std::is_floating_point<T>::value,
        "ERROR - to_radian(): template parameter not a valid floating point number)\n");
  
    const T deg2rad = 180.f / M_PI;
    return deg * deg2rad;
}

/* 
 * @brief: Converts an euler angle with units 'radian' to an angle with the unit 'degree'
 */
template <class T>
static inline T to_degree(const T &rad, const int &flag) {
    static_assert(std::is_floating_point<T>::value,
        "ERROR - to_radian(): template parameter not a valid floating point number)\n");
  
    const T deg2rad = 180.f / M_PI;
    return rad / deg2rad;
}

/* 
 * @brief: Gets two values and returns the smaller one.
 */
template<typename A, typename B>
static inline auto smaller(const A &one, const B &two) -> decltype(one < two ? one : two) {
    static_assert(std::is_floating_point<A>::value || std::is_integral<A>::value, 
        "ERROR - smaller(): template parameters not of type float or int\n");
    static_assert(std::is_floating_point<B>::value || std::is_integral<B>::value, 
        "ERROR - smaller(): template parameters not of type float or int\n"); 
 
    return one < two ? one : two;
}

/* 
 * @brief: Gets two values and returns the bigger one.
 */
template<typename A, typename B>
static inline auto bigger(const A &one, const B &two) -> decltype(one > two ? one : two) {
    static_assert(std::is_floating_point<A>::value || std::is_integral<A>::value, 
        "ERROR - bigger(): template parameters not of type float or int\n");
    static_assert(std::is_floating_point<B>::value || std::is_integral<B>::value, 
        "ERROR - bigger(): template parameters not of type float or int\n"); 
  
    return one > two ? one : two;
}


#endif // AP_ALGEBRA_H