#include "LeadFilter.h"
#include <math.h>
LeadFilter::LeadFilter() :
_sample_freq(0),
_omega(0),
_ratio(0),
_u_prev(0),
_y_prev(0)
{}

void LeadFilter::set_params(float omega, float ratio, float sample_freq) {
    if (omega != _omega || ratio != _ratio || sample_freq != _sample_freq) {
        _omega = omega;
        _ratio = ratio;
        _sample_freq = sample_freq;
        compute_params();
    }
}

void LeadFilter::compute_params() {
    if(_sample_freq == 0 || _ratio == 0 || _omega == 0) {
        return;
    }
    float dt = 1.0f/_sample_freq;
    float sqrt_ratio = sqrtf(_ratio);
    float wp = _omega*sqrt_ratio;
    float wz = _omega/sqrt_ratio;
    float K = 2*wz + dt*wp*wz;
    _B0 = (2*wp + dt*wp*wz)/K;
    _B1 = (dt*wp*wz - 2*wp)/K;
    _A1 = (dt*wp*wz - 2*wz)/K;
}

float LeadFilter::apply(float u) {
    if(_sample_freq == 0 || _ratio == 0 || _omega == 0) {
        return u;
    }

    _y_prev = _B0*u + _B1*_u_prev - _A1*_y_prev;
    _u_prev = u;
    return _y_prev;
}
