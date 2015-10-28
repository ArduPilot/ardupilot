#ifndef AP_EXTH_H
#define AP_EXTH_H

#include <limits>
#include <type_traits>
#include <float.h>
#include <stdio.h>

/*
 * @brief: Safe way to convert floating point numbers to integers
 * @authors: Daniel Frenzel <dgdanielf@gmail.com>
 */
template <class IntType, class FloatType>
static inline IntType safeFloatToInt(const FloatType &num) {
    //check if float fits into integer
    if(std::numeric_limits<IntType>::digits > std::numeric_limits<FloatType>::digits) {
        return static_cast<IntType>(num);
    }

    // check if float is smaller than max int
    if(num < static_cast<FloatType>(std::numeric_limits<IntType>::max()) &&
       num > static_cast<FloatType>(std::numeric_limits<IntType>::min()))
    {
        return static_cast<IntType>(num); //safe to cast
    }

    //NaN is not defined for int return the largest int value
    printf("CRITICAL - safeFloatToInt(): Unsafe conversion of value: %d\n", num);
    return num < 0 ? std::numeric_limits<IntType>::min() : std::numeric_limits<IntType>::max();
}

/*
 * @brief: Constrains a value to be within the range: low and high
 */
template <class T>
static inline T constrain_value(T amt, T low, T high) {  
    if(isnanf(low) || isnanf(high)) {
        printf("CRITICAL - constrain_value(): One or more parameters are NaN\n");
        return (low+high) * 0.5f;
    }
    
    return amt < low ? low : (amt > high ? high : amt);
}

// are two floats equal
template <class FloatOne, class FloatTwo>
static inline bool is_equal(const FloatOne fVal1, const FloatTwo fVal2) {
    static_assert(std::is_floating_point<FloatOne>::value && std::is_floating_point<FloatTwo>::value, 
		  "CRITICAL - is_equal(): template parameters not of type float\n");
    
    return fabsf(fVal1 - fVal2) < FLT_EPSILON ? true : false; 
}

// is a float is zero
template <class T>
static inline bool is_zero(const T fVal1) { 
    static_assert(std::is_floating_point<T>::value,
		  "CRITICAL - is_zero(): template parameter not of type float\n");

    return fabsf(fVal1) < FLT_EPSILON ? true : false; 
}

/*
 * @brief:  wrap an angle to fit into a range: -180 to 180 or 0 to 360 degrees. This is defined by the RNG parameter
 * @parameters: The ANG template parameter defines the treatment of the angles: whether euler or radian. 
 * @return: The return value can be either an euler or a radian angle, dependent on the template parameter: ANG.
 */

enum constr_settings {
    RADIAN  = 1 << 0,
    EULER   = 1 << 1,
    DEG_180 = 1 << 2,
    DEG_360 = 1 << 3
};

template <class T>
T constrain_angle(const T &angle, const int &flag) {
    static_assert(std::is_floating_point<T>::value || std::is_integral<T>::value,
		  "CRITICAL - constrain_euler(): template parameter not a valid type (integral or floating point number)\n");
  
    // Always perform calculations in radian, may reduce precision losses (other way round)
    T res = angle;
    if(flag & EULER) {
        res *= 2.f*M_PI;
    }

    const float _ang_180 =  180.f * 2.f*M_PI;
    const float _ang_360 =  360.f * 2.f*M_PI;

    if(flag & DEG_180 & DEG_360 || flag & RADIAN & EULER) {
        printf("CRITICAL - constrain_euler(): The settings flag makes no sense\n");
	return angle;
    }
    
    if(flag & DEG_180) {
        res = fmod(res + _ang_180, _ang_360);
        if (res < 0.f) {
            res += _ang_360;
        }
        res -= _ang_180;
    }
    else if(flag & DEG_360) {
        res = fmod(res, _ang_360);
        if (res < 0.f) {
            res += _ang_360;
        }
    }
    else {
        printf("CRITICAL - constrain_euler(): The angle was not constrained, \
                because one or more template parameter(s) were invalid.\n");
        return angle;
    }

    // may convert back to euler
    if(flag & EULER) {
        res /= 2.f*M_PI;
    }

    return res;
}

#endif // AP_EXTH_H