/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NotchFilter.h"

/*
  initialise filter
 */
template <class T>
void NotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    float omega = 2.0 * M_PI * center_freq_hz / sample_freq_hz;
    float octaves = log2f(center_freq_hz  / (center_freq_hz - bandwidth_hz/2)) * 2;
    float A = powf(10, -attenuation_dB/40);
    float Q = sqrtf(powf(2, octaves)) / (powf(2,octaves) - 1);
    float alpha = sinf(omega) / (2 * Q/A);
    b0 =  1.0 + alpha*A;
    b1 = -2.0 * cosf(omega);
    b2 =  1.0 - alpha*A;
    a0_inv =  1.0/(1.0 + alpha/A);
    a1 = -2.0 * cosf(omega);
    a2 =  1.0 - alpha/A;
    initialised = true;
}

/*
  apply a new input sample, returning new output
 */
template <class T>
T NotchFilter<T>::apply(const T &sample)
{
    if (!initialised) {
        // if we have not been initialised when return the input
        // sample as output
        return sample;
    }
    ntchsig2 = ntchsig1;
    ntchsig1 = ntchsig;
    ntchsig = sample;
    T output = (ntchsig*b0 + ntchsig1*b1 + ntchsig2*b2 - signal1*a1 - signal2*a2) * a0_inv;
    signal2 = signal1;
    signal1 = output;
    return output;
}

// table of user settable parameters
const AP_Param::GroupInfo NotchFilterVector3fParam::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable notch filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, NotchFilterVector3fParam, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FREQ
    // @DisplayName: Frequency
    // @Description: Notch center frequency in Hz
    // @Range: 10 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FREQ", 2, NotchFilterVector3fParam, center_freq_hz, 80),

    // @Param: BW
    // @DisplayName: Bandwidth
    // @Description: Notch bandwidth in Hz
    // @Range: 5 50
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("BW", 3, NotchFilterVector3fParam, bandwidth_hz, 20),

    // @Param: ATT
    // @DisplayName: Attenuation
    // @Description: Notch attenuation in dB
    // @Range: 5 30
    // @Units: dB
    // @User: Advanced
    AP_GROUPINFO("ATT", 4, NotchFilterVector3fParam, attenuation_dB, 15),
    
    AP_GROUPEND
};

/*
  a notch filter with enable and filter parameters - constructor
 */
NotchFilterVector3fParam::NotchFilterVector3fParam(void)
{
    AP_Param::setup_object_defaults(this, var_info);    
}

/*
  initialise filter
 */
void NotchFilterVector3fParam::init(float _sample_freq_hz)
{
    filter.init(_sample_freq_hz, center_freq_hz, bandwidth_hz, attenuation_dB);

    sample_freq_hz = _sample_freq_hz;
    last_center_freq = center_freq_hz;
    last_bandwidth = bandwidth_hz;
    last_attenuation = attenuation_dB;
}

/*
  apply a filter sample
 */
Vector3f NotchFilterVector3fParam::apply(const Vector3f &sample)
{
    if (!enable) {
        // when not enabled it is a simple pass-through
        return sample;
    }

    // check for changed parameters
    if (!is_equal(center_freq_hz.get(), last_center_freq) ||
        !is_equal(bandwidth_hz.get(), last_bandwidth) ||
        !is_equal(attenuation_dB.get(), last_attenuation)) {
        if (!is_zero(sample_freq_hz)) {
            init(sample_freq_hz);
        }
    }
    
    return filter.apply(sample);
}

/* 
   instantiate template classes
 */
template class NotchFilter<float>;
template class NotchFilter<Vector3f>;
