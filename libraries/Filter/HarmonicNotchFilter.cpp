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
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

/*
  optional logging for SITL only of all notch frequencies
 */
#ifndef NOTCH_DEBUG_LOGGING
#define NOTCH_DEBUG_LOGGING 0
#endif

#if NOTCH_DEBUG_LOGGING
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#endif

/*
  cutoff proportion of sample rate above which we do not use the notch
 */
#define HARMONIC_NYQUIST_CUTOFF 0.48f

/*
  point at which the harmonic notch goes to zero attenuation
 */
#define NOTCHFILTER_ATTENUATION_CUTOFF 0.25

#if APM_BUILD_TYPE(APM_BUILD_Heli)
    // We cannot use throttle based notch on helis
    #define NOTCHFILTER_DEFAULT_MODE float(HarmonicNotchDynamicMode::Fixed) // fixed
#else
    #define NOTCHFILTER_DEFAULT_MODE float(HarmonicNotchDynamicMode::UpdateThrottle) // throttle based
#endif


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
    // @Description: Harmonic Notch Filter base center frequency in Hz. This is the center frequency for static notches, the center frequency for Throttle based notches at the reference thrust value, and the minimum limit of center frequency variation for all other notch types. This should always be set lower than half the backend gyro rate (which is typically 1Khz). 
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
    // @Description: Harmonic Notch Filter dynamic frequency tracking mode. Dynamic updates can be throttle, RPM sensor, ESC telemetry or dynamic FFT based. Throttle-based harmonic notch cannot be used on fixed wing only planes. It can for Copters, QuaadPlane(while in VTOL modes), and Rovers.
    // @Range: 0 5
    // @Values: 0:Fixed,1:Throttle,2:RPM Sensor,3:ESC Telemetry,4:Dynamic FFT,5:Second RPM Sensor
    // @User: Advanced
    AP_GROUPINFO("MODE", 7, HarmonicNotchFilterParams, _tracking_mode, NOTCHFILTER_DEFAULT_MODE),

    // @Param: OPTS
    // @DisplayName: Harmonic Notch Filter options
    // @Description: Harmonic Notch Filter options. Triple and double-notches can provide deeper attenuation across a wider bandwidth with reduced latency than single notches and are suitable for larger aircraft. Multi-Source attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks, in the case of ESC it will attach notches to each of four motor RPM values. Loop rate update changes the notch center frequency at the scheduler loop rate rather than at the default of 200Hz. If both double and triple notches are specified only double notches will take effect.
    // @Bitmask: 0:Double notch,1:Multi-Source,2:Update at loop rate,3:EnableOnAllIMUs,4:Triple notch, 5:Use min freq on RPM source failure, 6:Quintuple notch
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OPTS", 8, HarmonicNotchFilterParams, _options, 0),

    // @Param: FM_RAT
    // @DisplayName: Throttle notch min frequency ratio
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
void HarmonicNotchFilter<T>::init(float sample_freq_hz, HarmonicNotchFilterParams &_params)
{
    // keep a copy of the params object
    params = &_params;

    // sanity check the input
    if (_filters == nullptr || is_zero(sample_freq_hz) || isnan(sample_freq_hz)) {
        return;
    }

    _sample_freq_hz = sample_freq_hz;

    const float bandwidth_hz = params->bandwidth_hz();
    const float attenuation_dB = params->attenuation_dB();
    float center_freq_hz = params->center_freq_hz();

    const float nyquist_limit = sample_freq_hz * HARMONIC_NYQUIST_CUTOFF;
    const float bandwidth_limit = bandwidth_hz * 0.52f;

    // remember the lowest frequency we will have a notch enabled at
    _minimum_freq = center_freq_hz * params->freq_min_ratio();

    /*
      adjust the fundamental center frequency we use for the initial
      calculation of A and Q to be in the allowable range
    */
    center_freq_hz = constrain_float(center_freq_hz, bandwidth_limit, nyquist_limit);

    // Calculate spread required to achieve an equivalent single notch using two notches with Bandwidth/2
    _notch_spread = bandwidth_hz / (32 * center_freq_hz);

    // position the individual notches so that the attenuation is no worse than a single notch
    // calculate attenuation and quality from the shaping constraints
    NotchFilter<T>::calculate_A_and_Q(center_freq_hz, bandwidth_hz / _composite_notches, attenuation_dB, _A, _Q);

    _initialised = true;

    // ensure static notches are allocated and working
    if (params->tracking_mode() == HarmonicNotchDynamicMode::Fixed) {
        update(center_freq_hz);
    }
}

/*
  allocate a collection of, at most HAL_HNF_MAX_FILTERS, notch filters to be managed by this harmonic notch filter
 */
template <class T>
void HarmonicNotchFilter<T>::allocate_filters(uint8_t num_notches, uint32_t harmonics, uint8_t composite_notches)
{
    _composite_notches = MIN(composite_notches, 3);
    _num_harmonics = __builtin_popcount(harmonics);
    _num_filters = MIN(_num_harmonics * num_notches * _composite_notches, HAL_HNF_MAX_FILTERS);
    _harmonics = harmonics;

    if (_num_filters > 0) {
        _filters = NEW_NOTHROW NotchFilter<T>[_num_filters];
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
void HarmonicNotchFilter<T>::expand_filter_count(uint16_t total_notches)
{
    if (total_notches <= _num_filters) {
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
    auto filters = NEW_NOTHROW NotchFilter<T>[total_notches];
    if (filters == nullptr) {
        _alloc_has_failed = true;
        return;
    }
    memcpy(filters, _filters, sizeof(filters[0])*_num_filters);
    auto _old_filters = _filters;
    _filters = filters;
    _num_filters = total_notches;
    delete[] _old_filters;
}

/*
  set the center frequency of a single notch harmonic

  The spread_mul is the frequency multiplier from the spread of the
  double or triple notch. The harmonic_mul is the multiplier for the
  frequency for this harmonic
 */
template <class T>
void HarmonicNotchFilter<T>::set_center_frequency(uint16_t idx, float notch_center, float spread_mul, uint8_t harmonic_mul)
{
    const float nyquist_limit = _sample_freq_hz * HARMONIC_NYQUIST_CUTOFF;
    auto &notch = _filters[idx];

    // scale the notch with the harmonic multiplier
    notch_center *= harmonic_mul;

    /* disable the filter if its center frequency is above the nyquist
       frequency

       NOTE: should this be notch_center*spread_mul ? As it is here we
       do sometimes run the upper notch in a double or triple notch at
       higher than the nyquist.
    */
    if (notch_center >= nyquist_limit) {
        notch.disable();
        return;
    }

    // the minimum frequency for a harmonic is the base minimum
    float harmonic_min_freq = _minimum_freq;

    // we can adjust the attenuation at low frequencies
    float A = _A;

    // on some vehicles we want to treat zero or very low frequency as
    // the minimum frequency, on others we want to disable the
    // notch for low frequencies
    const bool treat_low_freq_as_min = params->hasOption(HarmonicNotchFilterParams::Options::TreatLowAsMin);

    if (treat_low_freq_as_min) {
        /*
          when we are treating low as min we don't want to collapse
          the harmonics down to low frequencies
         */
        harmonic_min_freq *= harmonic_mul;
    } else {
        /*
          disable if the notch is less than 25% of the min frequency
        */
        const float disable_freq = harmonic_min_freq * NOTCHFILTER_ATTENUATION_CUTOFF;
        if (notch_center < disable_freq) {
            notch.disable();
            return;
        }

        if (notch_center < harmonic_min_freq) {
            /*
              scale the attenuation so that we fade out the notch as
              we get further below the min frequency. The attenuation
              factor A goes to 1.0 (which means no attenuation)
              Scaling the attenuation in this way means we don't get a
              glitch at the disable point
             */
            A = linear_interpolate(A, 1.0, notch_center, harmonic_min_freq, disable_freq);
        }
    }

    // don't let the notch go below the min frequency
    notch_center = MAX(notch_center, harmonic_min_freq);

    /* adjust notch center for spread for double and triple notch.
       This adjustment is applied last to maintain the symmetry of the
       double and triple notches
    */
    notch_center *= spread_mul;

    notch.init_with_A_and_Q(_sample_freq_hz, notch_center, A, _Q);
}

/*
  update the underlying filters' center frequency using the current attenuation and quality
  this function is cheaper than init() because A & Q do not need to be recalculated
 */
template <class T>
void HarmonicNotchFilter<T>::update(float center_freq_hz)
{
    update(1, &center_freq_hz);
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
    const float nyquist_limit = _sample_freq_hz * HARMONIC_NYQUIST_CUTOFF;

    const uint16_t total_notches = MIN(num_centers * _num_harmonics * _composite_notches, HAL_HNF_MAX_FILTERS);
    if (total_notches > _num_filters) {
        // alloc realloc of filters
        expand_filter_count(total_notches);
    }

    _num_enabled_filters = 0;

    // update all of the filters using the new center frequencies and existing A & Q
    for (uint16_t i = 0; i < num_centers * HNF_MAX_HARMONICS && _num_enabled_filters < _num_filters; i++) {
        const uint8_t harmonic_n = i / num_centers;
        const uint8_t center_n = i % num_centers;
        // the filters are ordered by center and then harmonic so
        // f1h1, f2h1, f3h1, f4h1, f1h2, f2h2, etc
        if (!((1U<<harmonic_n) & _harmonics)) {
            continue;
        }

        const float notch_center = constrain_float(center_freq_hz[center_n], 0.0f, nyquist_limit);
        const uint8_t harmonic_mul = (harmonic_n+1);
        if (_composite_notches != 2) {
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0, harmonic_mul);
        }
        if (_composite_notches > 1) {
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0 - _notch_spread, harmonic_mul);
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0 + _notch_spread, harmonic_mul);
        }
        if (_composite_notches > 3) {
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0 - 2 * _notch_spread, harmonic_mul);
            set_center_frequency(_num_enabled_filters++, notch_center, 1.0 + 2 * _notch_spread, harmonic_mul);
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

#if NOTCH_DEBUG_LOGGING
    static int dfd = -1;
    if (dfd == -1) {
        dfd = ::open("notch.txt", O_WRONLY|O_CREAT|O_TRUNC, 0644);
    }
#endif

    T output = sample;
    for (uint16_t i = 0; i < _num_enabled_filters; i++) {
#if NOTCH_DEBUG_LOGGING
        if (!_filters[i].initialised) {
            ::dprintf(dfd, "------- ");
        } else {
            ::dprintf(dfd, "%.4f ", _filters[i]._center_freq_hz);
        }
#endif
        output = _filters[i].apply(output);
    }
#if NOTCH_DEBUG_LOGGING
    if (_num_enabled_filters > 0) {
        ::dprintf(dfd, "\n");
    }
#endif
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

    for (uint16_t i = 0; i < _num_filters; i++) {
        _filters[i].reset();
    }
}

#if HAL_LOGGING_ENABLED
// @LoggerMessage: FCN
// @Description: Filter Center Message - per motor
// @Field: TimeUS: microseconds since system startup
// @Field: I: instance
// @Field: NF: total number of active harmonic notches
// @Field: CF1: First harmonic centre frequency for motor 1
// @Field: CF2: First harmonic centre frequency for motor 2
// @Field: CF3: First harmonic centre frequency for motor 3
// @Field: CF4: First harmonic centre frequency for motor 4
// @Field: CF5: First harmonic centre frequency for motor 5
// @Field: CF6: First harmonic centre frequency for motor 6
// @Field: HF1: Second harmonic centre frequency for motor 1
// @Field: HF2: Second harmonic centre frequency for motor 2
// @Field: HF3: Second harmonic centre frequency for motor 3
// @Field: HF4: Second harmonic centre frequency for motor 4
// @Field: HF5: Second harmonic centre frequency for motor 5
// @Field: HF6: Second harmonic centre frequency for motor 6

// @LoggerMessage: FCNS
// @Description: Filter Center Message
// @Field: TimeUS: microseconds since system startup
// @Field: I: instance
// @Field: CF: notch centre frequency
// @Field: HF: 2nd harmonic frequency

/*
  log center frequencies of 1st and 2nd harmonic of a harmonic notch
  instance for up to 6 frequency sources

  the instance number passed in corresponds to the harmonic notch
  instance in AP_InertialSensor
 */
template <class T>
void HarmonicNotchFilter<T>::log_notch_centers(uint8_t instance, uint64_t now_us) const
{
     /*
      for composite notches we only log the first entry. For triple
      and single notch this is the center. For double notch it is the
      lower frequency
    */
    const uint16_t filters_per_source = _composite_notches * _num_harmonics;
    if (_num_filters == 0 || filters_per_source == 0) {
        return;
    }
    const uint8_t num_sources = MIN(6, _num_filters / filters_per_source);
    float centers[6] {};
    float first_harmonic[6] {};

    for (uint8_t i=0; i<num_sources; i++) {
        /*
          note the ordering of the filters from update() above:
            f1h1, f2h1, f3h1, f4h1, f1h2, f2h2, f3h2, f4h2 etc
         */
        centers[i] = _filters[i*_composite_notches].logging_frequency();
        first_harmonic[i] = _filters[num_sources*_composite_notches + i*_composite_notches].logging_frequency();
    }

    if (num_sources > 1) {
        AP::logger().WriteStreaming(
            "FCN", "TimeUS,I,NF,CF1,CF2,CF3,CF4,CF5,CF6,HF1,HF2,HF3,HF4,HF5,HF6", "s#-zzzzzzzzzzzz", "F--------------", "QBHffffffffffff",
            now_us,
            instance,
            _num_filters,
            centers[0], centers[1], centers[2], centers[3], centers[4], centers[5],
            first_harmonic[0], first_harmonic[1], first_harmonic[2], first_harmonic[3], first_harmonic[4], first_harmonic[5]);
    } else {
        // log single center frequency
        AP::logger().WriteStreaming(
            "FCNS", "TimeUS,I,CF,HF", "s#zz", "F---", "QBff",
            now_us,
            instance,
            centers[0],
            first_harmonic[0]);
    }
}
#endif // HAL_LOGGING_ENABLED

/*
  create parameters for the harmonic notch filter and initialise defaults
 */
HarmonicNotchFilterParams::HarmonicNotchFilterParams(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void HarmonicNotchFilterParams::init()
{
    _harmonics.convert_bitmask_parameter_width(AP_PARAM_INT8);
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
    _freq_min_ratio.save();
}


// return the number of composite notches given the options
uint8_t HarmonicNotchFilterParams::num_composite_notches(void) const
{
    if (hasOption(Options::DoubleNotch)) {
        return 2;
    }
    if (hasOption(Options::TripleNotch)) {
        return 3;
    }
    if (hasOption(Options::QuintupleNotch)) {
        return 5;
    }
    return 1;
}


/* 
  instantiate template classes
 */
template class HarmonicNotchFilter<Vector3f>;
template class HarmonicNotchFilter<float>;

