#include "AP_Filter_config.h"

#if AP_FILTER_ENABLED

#include "AP_Filter.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

const AP_Param::GroupInfo AP_NotchFilter_params::var_info[] = {
  
    // @Param: NOTCH_FREQ
    // @DisplayName: Notch Filter center frequency
    // @Description: Notch Filter center frequency in Hz.
    // @Range: 10 495
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NOTCH_FREQ", 1, AP_NotchFilter_params, _center_freq_hz, 0),

    // @Param: NOTCH_Q
    // @DisplayName: Notch Filter quality factor
    // @Description: Notch Filter quality factor given by the notch centre frequency divided by its bandwidth.
    // @Range: 1 10
    // @User: Advanced
    AP_GROUPINFO("NOTCH_Q", 2, AP_NotchFilter_params, _quality, 2),

    // @Param: NOTCH_ATT
    // @DisplayName: Notch Filter attenuation
    // @Description: Notch Filter attenuation in dB.
    // @Range: 5 50
    // @Units: dB
    // @User: Advanced
    AP_GROUPINFO("NOTCH_ATT", 3, AP_NotchFilter_params, _attenuation_dB, 40),

    // @Param: NOTCH_HMNC
    // @DisplayName: Harmonic Notch Filter harmonics
    // @Description: Bitmask of harmonic frequencies to apply Harmonic Notch Filter to. A value of 0 disables this filter. The first harmonic refers to the base frequency.
    // @Bitmask: 0:  1st harmonic
    // @Bitmask: 1:  2nd harmonic
    // @Bitmask: 2:  3rd harmonic
    // @Bitmask: 3:  4th harmonic
    // @Bitmask: 4:  5th harmonic
    // @Bitmask: 5:  6th harmonic
    // @Bitmask: 6:  7th harmonic
    // @Bitmask: 7:  8th harmonic
    // @Bitmask: 8:  9th harmonic
    // @Bitmask: 9:  10th harmonic
    // @Bitmask: 10: 11th harmonic
    // @Bitmask: 11: 12th harmonic
    // @Bitmask: 12: 13th harmonic
    // @Bitmask: 13: 14th harmonic
    // @Bitmask: 14: 15th harmonic
    // @Bitmask: 15: 16th harmonic
    // @User: Advanced
    AP_GROUPINFO("NOTCH_HMNC", 4, AP_NotchFilter_params, _harmonics, 1),

    AP_GROUPEND
};

AP_NotchFilter_params::AP_NotchFilter_params() : AP_Filter(AP_Filter::FilterType::FILTER_NOTCH)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_NotchFilter_params::setup_notch_filter(NotchFilterFloat& filter, float sample_rate)
{
    if (is_zero(_quality) || is_zero(_center_freq_hz) || is_zero(_attenuation_dB) || (_harmonics == 0)) {
        return false;
    }

    if (!is_equal(sample_rate, filter.sample_freq_hz())
        || !is_equal(_center_freq_hz.get(), filter.center_freq_hz())
        || (_harmonics.get() != filter.harmonics())) {
        filter.init(sample_rate, _center_freq_hz, _center_freq_hz / _quality, _attenuation_dB, (uint16_t)_harmonics.get());
        return _harmonics.get() == filter.harmonics();
    }
    return true;
}

#endif // AP_FILTER_ENABLED
