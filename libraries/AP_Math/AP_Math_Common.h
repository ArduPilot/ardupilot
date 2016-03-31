#pragma once

#include <limits>
#include <math.h>


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
bool is_equal(const FloatOne fVal1, const FloatTwo fVal2) {
    static_assert((std::is_floating_point<FloatOne>::value || std::is_base_of<FloatOne,AP_Float>::value), "ERROR - is_equal(): template parameters not of type float\n");
    static_assert((std::is_floating_point<FloatTwo>::value || std::is_base_of<FloatTwo,AP_Float>::value), "ERROR - is_equal(): template parameters not of type float\n");
    
    return fabsf(fVal1 - fVal2) < std::numeric_limits<decltype(fVal1 - fVal2)>::epsilon() ? true : false; 
}

/* 
 * @brief: Checks whether a float is zero
 */
template <class T>
bool is_zero(const T fVal1) { 
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value, "ERROR - is_zero(): template parameter not of type float\n");

    return fabsf(fVal1) < std::numeric_limits<T>::epsilon() ? true : false; 
} 
