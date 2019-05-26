#include "LowPassFilter2p.h"


////////////////////////////////////////////////////////////////////////////////////////////
// DigitalBiquadFilter
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
DigitalBiquadFilter<T>::DigitalBiquadFilter() {
  _delay_sample_1 = T();
  _delay_sample_2 = T();
  _delay_output_1 = T();
  _delay_output_2 = T();
}

template <class T>
T DigitalBiquadFilter<T>::apply(const T &sample, const struct biquad_params &params) {
    if(is_zero(params.cutoff_freq) || is_zero(params.sample_freq)) {
        return sample;
    }

    T output = sample * params.b0 + _delay_sample_1 * params.b1 + _delay_sample_2 * params.b2 - _delay_output_1 * params.a1 - _delay_output_2 * params.a2;

    _delay_sample_2 = _delay_sample_1;
    _delay_sample_1 = sample;

    _delay_output_2 = _delay_output_1;
    _delay_output_1 = output;

    return output;
}

template <class T>
void DigitalBiquadFilter<T>::reset() { 
    _delay_sample_1 = _delay_sample_2 = T();
}

// compute biquad params for a LPF (ref: http://shepazu.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html)
template <class T>
void DigitalBiquadFilter<T>::compute_params(float sample_freq, float cutoff_freq, biquad_params &ret) {
    ret.cutoff_freq = cutoff_freq;
    ret.sample_freq = sample_freq;
    if (!is_positive(ret.cutoff_freq)) {
        // zero cutoff means pass-thru
        return;
    }

    float Q = 1 / sqrtf(2);
    float omega = 2 * M_PI * cutoff_freq / sample_freq;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2 * Q);

    float a0 = 1 + alpha;

    ret.b0 = ((1 - cs) / 2) /a0;
    ret.b1 = (1 - cs) / a0;
    ret.b2 = ret.b0;
	ret.a1 = (-2 * cs) / a0;
	ret.a2 = (1 - alpha) / a0;
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

/* 
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class LowPassFilter2p<int>;
template class LowPassFilter2p<long>;
template class LowPassFilter2p<float>;
template class LowPassFilter2p<Vector2f>;
template class LowPassFilter2p<Vector3f>;
