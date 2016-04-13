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