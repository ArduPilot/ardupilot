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

#ifndef HAL_DEBUG_BUILD
#define AP_INLINE_VECTOR_OPS
#pragma GCC optimize("O2")
#endif

#include "HarmonicNotchFilter.h"
#include <GCS_MAVLink/GCS.h>

#define HNF_MAX_FILTERS HAL_HNF_MAX_FILTERS // must be even for double-notch filters
#define HNF_MAX_HARMONICS 8

// table of user settable parameters
const AP_Param::GroupInfo HarmonicNotchFilterParams::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Harmonic Notch Filter enable
    // @Description: Harmonic Notch Filter enable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, HarmonicNotchFilterParams, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FREQ
    // @DisplayName: Harmonic Notch Filter base frequency
    // @Description: Harmonic Notch Filter base center frequency in Hz. This should be set at most half the backend gyro rate (which is typically 1Khz). For helicopters using RPM sensor to dynamically set the notch frequency, use this parameter to provide a lower limit to the dynamic notch filter.  Recommend setting it to half the operating rotor speed in Hz.
    // @Range: 10 495
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FREQ", 2, HarmonicNotchFilterParams, _center_freq_hz, 80),

    // @Param: BW
    // @DisplayName: Harmonic Notch Filter bandwidth
    // @Description: Harmonic Notch Filter bandwidth in Hz. This is typically set to half the base frequency. The ratio of base frequency to bandwidth determines the notch quality factor and is fixed across harmonics.
    // @Range: 5 250
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("BW", 3, HarmonicNotchFilterParams, _bandwidth_hz, 40),

    // @Param: ATT
    // @DisplayName: Harmonic Notch Filter attenuation
    // @Description: Harmonic Notch Filter attenuation in dB. Values greater than 40dB will typically produce a hard notch rather than a modest attenuation of motor noise.
    // @Range: 5 50
    // @Units: dB
    // @User: Advanced
    AP_GROUPINFO("ATT", 4, HarmonicNotchFilterParams, _attenuation_dB, 40),

    // @Param: HMNCS
    // @DisplayName: Harmonic Notch Filter harmonics
    // @Description: Bitmask of harmonic frequencies to apply Harmonic Notch Filter to. This option takes effect on the next reboot. A value of 0 disables this filter. The first harmonic refers to the base frequency.
    // @Bitmask: 0:1st harmonic,1:2nd harmonic,2:3rd harmonic,3:4th hamronic,4:5th harmonic,5:6th harmonic,6:7th harmonic,7:8th harmonic
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("HMNCS", 5, HarmonicNotchFilterParams, _harmonics, 3),

    // @Param: REF
    // @DisplayName: Harmonic Notch Filter reference value
    // @Description: A reference value of zero disables dynamic updates on the Harmonic Notch Filter and a positive value enables dynamic updates on the Harmonic Notch Filter.  For throttle-based scaling, this parameter is the reference value associated with the specified frequency to facilitate frequency scaling of the Harmonic Notch Filter. For RPM and ESC telemetry based tracking, this parameter is set to 1 to enable the Harmonic Notch Filter using the RPM sensor or ESC telemetry set to measure rotor speed.  The sensor data is converted to Hz automatically for use in the Harmonic Notch Filter.  This reference value may also be used to scale the sensor data, if required.  For example, rpm sensor data is required to measure heli motor RPM. Therefore the reference value can be used to scale the RPM sensor to the rotor RPM.
    // @User: Advanced
    // @Range: 0.0 1.0
    // @RebootRequired: True
    AP_GROUPINFO("REF", 6, HarmonicNotchFilterParams, _reference, 0),

    // @Param: MODE
    // @DisplayName: Harmonic Notch Filter dynamic frequency tracking mode
    // @Description: Harmonic Notch Filter dynamic frequency tracking mode. Dynamic updates can be throttle, RPM sensor, ESC telemetry or dynamic FFT based. Throttle-based updates should only be used with multicopters.
    // @Range: 0 4
    // @Values: 0:Disabled,1:Throttle,2:RPM Sensor,3:ESC Telemetry,4:Dynamic FFT,5:Second RPM Sensor
    // @User: Advanced
    AP_GROUPINFO("MODE", 7, HarmonicNotchFilterParams, _tracking_mode, int8_t(HarmonicNotchDynamicMode::UpdateThrottle)),

    // @Param: OPTS
    // @DisplayName: Harmonic Notch Filter options
    // @Description: Harmonic Notch Filter options. Triple and double-notches can provide deeper attenuation across a wider bandwidth with reduced latency than single notches and are suitable for larger aircraft. Dynamic harmonics attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks, in the case of ESC it will attach notches to each of four motor RPM values. Loop rate update changes the notch center frequency at the scheduler loop rate rather than at the default of 200Hz. If both double and triple notches are specified only double notches will take effect.
    // @Bitmask: 0:Double notch,1:Dynamic harmonic,2:Update at loop rate,3:EnableOnAllIMUs,4:Triple notch
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OPTS", 8, HarmonicNotchFilterParams, _options, 0),

    // @Param: FM_RAT
    // @DisplayName: Throttle notch min freqency ratio
    // @Description: The minimum ratio below the configured frequency to take throttle based notch filters when flying at a throttle level below the reference throttle. Note that lower frequency notch filters will have more phase lag. If you want throttle based notch filtering to be effective at a throttle up to 30% below the configured notch frequency then set this parameter to 0.7. The default of 1.0 means the notch will not go below the frequency in the FREQ parameter.
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("FM_RAT", 9, HarmonicNotchFilterParams, _freq_min_ratio, 1.0),
    
    AP_GROUPEND
};

/*
  destroy all of the associated notch filters
 */
template <class T>
HarmonicNotchFilter<T>::~HarmonicNotchFilter() {
    delete[] _filters;
    _num_filters = 0;
    _num_enabled_filters = 0;
}

/*
  initialise the associated filters with the provided shaping constraints
  the constraints are used to determine attenuation (A) and quality (Q) factors for the filter
 */
template <class T>
void HarmonicNotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    // sanity check the input
    if (_filters == nullptr || is_zero(sample_freq_hz) || isnan(sample_freq_hz)) {
        return;
    }

    _sample_freq_hz = sample_freq_hz;

    const float nyquist_limit = sample_freq_hz * 0.48f;
    const float bandwidth_limit = bandwidth_hz * 0.52f;
    // adjust the fundamental center frequency to be in the allowable range
    center_freq_hz = constrain_float(center_freq_hz, bandwidth_limit, nyquist_limit);
    // Calculate spread required to achieve an equivalent single notch using two notches with Bandwidth/2
    _notch_spread = bandwidth_hz / (32 * center_freq_hz);

    // position the individual notches so that the attenuation is no worse than a single notch
    // calculate attenuation and quality from the shaping constraints
    NotchFilter<T>::calculate_A_and_Q(center_freq_hz, bandwidth_hz / _composite_notches, attenuation_dB, _A, _Q);

    _initialised = true;
    update(center_freq_hz);
}

/*
  allocate a collection of, at most HNF_MAX_FILTERS, notch filters to be managed by this harmonic notch filter
 */
template <class T>
void HarmonicNotchFilter<T>::allocate_filters(uint8_t num_notches, uint8_t harmonics, uint8_t composite_notches)
{
    _composite_notches = MIN(composite_notches, 3);
    _num_harmonics = __builtin_popcount(harmonics);
    _num_filters = _num_harmonics * num_notches * _composite_notches;
    _harmonics = harmonics;

    if (_num_filters > 0) {
        _filters = new NotchFilter<T>[_num_filters];
        if (_filters == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to allocate %u bytes for notch filter", (unsigned int)(_num_filters * sizeof(NotchFilter<T>)));
            _num_filters = 0;
        }
    }
}

/*
  expand the number of filters at runtime, allowing for RPM sources such as lua scripts
 */
template <class T>
void HarmonicNotchFilter<T>::expand_filter_count(uint8_t num_notches)
{
    uint8_t num_filters = _num_harmonics * num_notches * _composite_notches;
    if (num_filters <= _num_filters) {
        // enough already
        return;
    }
    if (_alloc_has_failed) {
        // we've failed to allocate before, don't try again
        return;
    }
    /*
      note that we rely on the semaphore in
      AP_InertialSensor_Backend.cpp to make this thread safe
     */
    auto filters = new NotchFilter<T>[num_filters];
    if (filters == nullptr) {
        _alloc_has_failed = true;
        return;
    }
    memcpy(filters, _filters, sizeof(filters[0])*_num_filters);
    auto _old_filters = _filters;
    _filters = filters;
    _num_filters = num_filters;
    delete[] _old_filters;
}

/*
  update the underlying filters' center frequency using the current attenuation and quality
  this function is cheaper than init() because A & Q do not need to be recalculated
 */
template <class T>
void HarmonicNotchFilter<T>::update(float center_freq_hz)
{
    if (!_initialised) {
        return;
    }

    // adjust the fundamental center frequency to be in the allowable range
    const float nyquist_limit = _sample_freq_hz * 0.48f;
    center_freq_hz = constrain_float(center_freq_hz, 1.0f, nyquist_limit);

    _num_enabled_filters = 0;
    // update all of the filters using the new center frequency and existing A & Q
    for (uint8_t i = 0; i < HNF_MAX_HARMONICS && _num_enabled_filters < _num_filters; i++) {
        if ((1U<<i) & _harmonics) {
            const float notch_center = center_freq_hz * (i+1);
            if (_composite_notches != 2) {
                // only enable the filter if its center frequency is below the nyquist frequency
                if (notch_center < nyquist_limit) {
                    _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center, _A, _Q);
                }
            }
            if (_composite_notches > 1) {
                float notch_center_double;
                // only enable the filter if its center frequency is below the nyquist frequency
                notch_center_double = notch_center * (1.0 - _notch_spread);
                if (notch_center_double < nyquist_limit) {
                    _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center_double, _A, _Q);
                }
                // only enable the filter if its center frequency is below the nyquist frequency
                notch_center_double = notch_center * (1.0 + _notch_spread);
                if (notch_center_double < nyquist_limit) {
                    _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center_double, _A, _Q);
                }
            }
        }
    }
}

/*
  update the underlying filters' center frequency using the current attenuation and quality
  this function is cheaper than init() because A & Q do not need to be recalculated
 */
template <class T>
void HarmonicNotchFilter<T>::update(uint8_t num_centers, const float center_freq_hz[])
{
    if (!_initialised) {
        return;
    }

    // adjust the frequencies to be in the allowable range
    const float nyquist_limit = _sample_freq_hz * 0.48f;

    if (num_centers > _num_filters) {
        // alloc realloc of filters
        expand_filter_count(num_centers);
    }

    _num_enabled_filters = 0;

    // update all of the filters using the new center frequencies and existing A & Q
    for (uint8_t i = 0; i < num_centers * HNF_MAX_HARMONICS && _num_enabled_filters < _num_filters; i++) {
        const uint8_t harmonic_n = i / num_centers;
        const uint8_t center_n = i % num_centers;
        // the filters are ordered by center and then harmonic so
        // f1h1, f2h1, f3h1, f4h1, f1h2, f2h2, etc
        if (!((1U<<harmonic_n) & _harmonics)) {
            continue;
        }

        const float notch_center = constrain_float(center_freq_hz[center_n] * (harmonic_n+1), 1.0f, nyquist_limit);
        if (_composite_notches != 2) {
            // only enable the filter if its center frequency is below the nyquist frequency
            if (notch_center < nyquist_limit) {
                _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center, _A, _Q);
            }
        }
        if (_composite_notches > 1) {
            float notch_center_double;
            // only enable the filter if its center frequency is below the nyquist frequency
            notch_center_double = notch_center * (1.0 - _notch_spread);
            if (notch_center_double < nyquist_limit) {
                _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center_double, _A, _Q);
            }
            // only enable the filter if its center frequency is below the nyquist frequency
            notch_center_double = notch_center * (1.0 + _notch_spread);
            if (notch_center_double < nyquist_limit) {
                _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center_double, _A, _Q);
            }
        }
    }
}

/*
  apply a sample to each of the underlying filters in turn and return the output
 */
template <class T>
T HarmonicNotchFilter<T>::apply(const T &sample)
{
    if (!_initialised) {
        return sample;
    }

    T output = sample;
    for (uint8_t i = 0; i < _num_enabled_filters; i++) {
        output = _filters[i].apply(output);
    }
    return output;
}

/*
  reset all of the underlying filters
 */
template <class T>
void HarmonicNotchFilter<T>::reset()
{
    if (!_initialised) {
        return;
    }

    for (uint8_t i = 0; i < _num_filters; i++) {
        _filters[i].reset();
    }
}

/*
  create parameters for the harmonic notch filter and initialise defaults
 */
HarmonicNotchFilterParams::HarmonicNotchFilterParams(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  save changed parameters
 */
void HarmonicNotchFilterParams::save_params()
{
    _enable.save();
    _center_freq_hz.save();
    _bandwidth_hz.save();
    _attenuation_dB.save();
    _harmonics.save();
    _reference.save();
}

/* 
  instantiate template classes
 */
template class HarmonicNotchFilter<Vector3f>;
template class HarmonicNotchFilter<float>;

