#ifndef HAL_DEBUG_BUILD
#define AP_INLINE_VECTOR_OPS
#pragma GCC optimize("O2")
#endif
#include "LowPassFilter2p.h"


////////////////////////////////////////////////////////////////////////////////////////////
// DigitalBiquadFilter
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
DigitalBiquadFilter<T>::DigitalBiquadFilter() {
  _delay_element_1 = T();
  _delay_element_2 = T();
}

template <class T>
T DigitalBiquadFilter<T>::apply(const T &sample, const struct biquad_params &params) {
    if(is_zero(params.cutoff_freq) || is_zero(params.sample_freq)) {
        return sample;
    }

    if (!initialised) {
        reset(sample, params);
        initialised = true;
    }

    T delay_element_0 = sample - _delay_element_1 * params.a1 - _delay_element_2 * params.a2;
    T output = delay_element_0 * params.b0 + _delay_element_1 * params.b1 + _delay_element_2 * params.b2;

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
}

template <class T>
void DigitalBiquadFilter<T>::reset() { 
    _delay_element_1 = _delay_element_2 = T();
    initialised = false;
}

template <class T>
void DigitalBiquadFilter<T>::reset(const T &value, const struct biquad_params &params) {
    _delay_element_1 = _delay_element_2 = value * (1.0 / (1 + params.a1 + params.a2));
    initialised = true;
}

template <class T>
void DigitalBiquadFilter<T>::compute_params(float sample_freq, float cutoff_freq, biquad_params &ret) {
    ret.cutoff_freq = cutoff_freq;
    ret.sample_freq = sample_freq;
    if (!is_positive(ret.cutoff_freq)) {
        // zero cutoff means pass-thru
        return;
    }

    float fr = sample_freq/cutoff_freq;
    float ohm = tanf(M_PI/fr);
    float c = 1.0f+2.0f*cosf(M_PI/4.0f)*ohm + ohm*ohm;

    ret.b0 = ohm*ohm/c;
    ret.b1 = 2.0f*ret.b0;
    ret.b2 = ret.b0;
    ret.a1 = 2.0f*(ohm*ohm-1.0f)/c;
    ret.a2 = (1.0f-2.0f*cosf(M_PI/4.0f)*ohm+ohm*ohm)/c;
}


////////////////////////////////////////////////////////////////////////////////////////////
// LowPassFilter2p
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
LowPassFilter2p<T>::LowPassFilter2p() { 
    memset(&_params, 0, sizeof(_params) ); 
}

// constructor
template <class T>
LowPassFilter2p<T>::LowPassFilter2p(float sample_freq, float cutoff_freq) {
    // set initial parameters
    set_cutoff_frequency(sample_freq, cutoff_freq);
}

// change parameters
template <class T>
void LowPassFilter2p<T>::set_cutoff_frequency(float sample_freq, float cutoff_freq) {
    DigitalBiquadFilter<T>::compute_params(sample_freq, cutoff_freq, _params);
}

// return the cutoff frequency
template <class T>
float LowPassFilter2p<T>::get_cutoff_freq(void) const {
    return _params.cutoff_freq;
}

template <class T>
float LowPassFilter2p<T>::get_sample_freq(void) const {
    return _params.sample_freq;
}

template <class T>
T LowPassFilter2p<T>::apply(const T &sample) {
    if (!is_positive(_params.cutoff_freq)) {
        // zero cutoff means pass-thru
        return sample;
    }
    return _filter.apply(sample, _params);
}

template <class T>
void LowPassFilter2p<T>::reset(void) {
    return _filter.reset();
}

template <class T>
void LowPassFilter2p<T>::reset(const T &value) {
    return _filter.reset(value, _params);
}

/* 
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class LowPassFilter2p<int>;
template class LowPassFilter2p<long>;
template class LowPassFilter2p<float>;
template class LowPassFilter2p<Vector2f>;
template class LowPassFilter2p<Vector3f>;
