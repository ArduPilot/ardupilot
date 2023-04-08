
#include "NotchFilter.h"
#include "NotchFilterIncParams.h"

// table of user settable parameters
template <class T>
const AP_Param::GroupInfo NotchFilterIncParams<T>::var_info[] = {

    // @Param: ENAB
    // @DisplayName: Enable filter
    // @Description: Enable static notch filter
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENAB", 1, NotchFilterIncParams, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ATT
    // @DisplayName: Notch filter attenuation
    // @Description: Notch filter attenuation
    // @Range: 35 60
    // @User: Advanced
    AP_GROUPINFO("ATT", 2, NotchFilterIncParams, _attenuation_dB, 40),

    // @Param: BW
    // @DisplayName: Notch filter bandwidth
    // @Description: Notch filter bandwidth
    // @Range: 2 50
    // @User: Advanced
    AP_GROUPINFO("BW", 3, NotchFilterIncParams, _bandwidth_hz, 2),

    // @Param: FREQ
    // @DisplayName: Notch filter centre frequency
    // @Description: Notch filter centre frequency
    // @Range: 35 60
    // @User: Advanced
    AP_GROUPINFO("FREQ", 4, NotchFilterIncParams, _center_freq_hz, 8),

    AP_GROUPEND
};


template <class T>
NotchFilterIncParams<T>::NotchFilterIncParams(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}


template <class T>
void NotchFilterIncParams<T>::init(float sample_freq_hz)
{
    // sanity check the input
    if (is_zero(sample_freq_hz) || isnan(sample_freq_hz)) {
        return;
    }

    NotchFilter<T>::init(sample_freq_hz, _center_freq_hz.get(), _bandwidth_hz.get(), _attenuation_dB.get());
}


/*
   instantiate template classes
 */
template class NotchFilterIncParams<float>;
template class NotchFilterIncParams<Vector2f>;
template class NotchFilterIncParams<Vector3f>;