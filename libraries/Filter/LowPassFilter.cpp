//
/// @file LowPassFilter.cpp
/// @brief  A class to implement a low pass filter

#ifndef HAL_DEBUG_BUILD
#define AP_INLINE_VECTOR_OPS
#pragma GCC optimize("O2")
#endif
#include "LowPassFilter.h"
#include <AP_InternalError/AP_InternalError.h>

////////////////////////////////////////////////////////////////////////////////////////////
// DigitalLPF, base class
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
DigitalLPF<T>::DigitalLPF() {
  // built in initialization
  output = T();
}

// add a new raw value to the filter, retrieve the filtered result
template <class T>
T DigitalLPF<T>::_apply(const T &sample, const float &alpha) {
    output += (sample - output) * alpha;
    if (!initialised) {
        initialised = true;
        output = sample;
    }
    return output;
}

// get latest filtered value from filter (equal to the value returned by latest call to apply method)
template <class T>
const T &DigitalLPF<T>::get() const {
    return output;
}

// Reset filter to given value
template <class T>
void DigitalLPF<T>::reset(const T &value) { 
    output = value;
    initialised = true;
}

// Set reset flag such that the filter will be reset to the next value applied
template <class T>
void DigitalLPF<T>::reset() {
    initialised = false;
}

template class DigitalLPF<float>;
template class DigitalLPF<Vector2f>;
template class DigitalLPF<Vector3f>;

////////////////////////////////////////////////////////////////////////////////////////////
// Low pass filter with constant time step
////////////////////////////////////////////////////////////////////////////////////////////

// constructor
template <class T>
LowPassFilterConstDt<T>::LowPassFilterConstDt(const float &sample_freq, const float &new_cutoff_freq)
{
    set_cutoff_frequency(sample_freq, new_cutoff_freq);
}

// change parameters
template <class T>
void LowPassFilterConstDt<T>::set_cutoff_frequency(const float &sample_freq, const float &new_cutoff_freq) {
    cutoff_freq = new_cutoff_freq;

    if (sample_freq <= 0) {
        alpha = 1;
    } else {
        alpha = calc_lowpass_alpha_dt(1.0/sample_freq, cutoff_freq);
    }
}

// return the cutoff frequency
template <class T>
float LowPassFilterConstDt<T>::get_cutoff_freq(void) const {
    return cutoff_freq;
}

// add a new raw value to the filter, retrieve the filtered result
template <class T>
T LowPassFilterConstDt<T>::apply(const T &sample) {
    return this->_apply(sample, alpha);
}

template class LowPassFilterConstDt<float>;
template class LowPassFilterConstDt<Vector2f>;
template class LowPassFilterConstDt<Vector3f>;

////////////////////////////////////////////////////////////////////////////////////////////
// Low pass filter with variable time step
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
LowPassFilter<T>::LowPassFilter(const float &new_cutoff_freq)
{
    set_cutoff_frequency(new_cutoff_freq);
}

// change parameters
template <class T>
void LowPassFilter<T>::set_cutoff_frequency(const float &new_cutoff_freq) {
    cutoff_freq = new_cutoff_freq;
}

// return the cutoff frequency
template <class T>
float LowPassFilter<T>::get_cutoff_freq() const {
    return cutoff_freq;
}

// add a new raw value to the filter, retrieve the filtered result
template <class T>
T LowPassFilter<T>::apply(const T &sample, const float &dt) {
    const float alpha = calc_lowpass_alpha_dt(dt, cutoff_freq);
    return this->_apply(sample, alpha);
}

/* 
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class LowPassFilter<float>;
template class LowPassFilter<Vector2f>;
template class LowPassFilter<Vector3f>;

