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
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <AP_Param/AP_Param.h>
#include "NotchFilter.h"

#define HNF_MAX_HARMONICS 16

class HarmonicNotchFilterParams;

/*
  a filter that manages a set of notch filters targetted at a fundamental center frequency
  and multiples of that fundamental frequency
 */
template <class T>
class HarmonicNotchFilter {
public:
    ~HarmonicNotchFilter();
    // allocate a bank of notch filters for this harmonic notch filter
    void allocate_filters(uint8_t num_notches, uint32_t harmonics, uint8_t composite_notches);
    // expand filter bank with new filters
    void expand_filter_count(uint16_t total_notches);
    // initialize the underlying filters using the provided filter parameters
    void init(float sample_freq_hz, HarmonicNotchFilterParams &params);
    // update the underlying filters' center frequencies using center_freq_hz as the fundamental
    void update(float center_freq_hz);
    // update all of the underlying center frequencies individually
    void update(uint8_t num_centers, const float center_freq_hz[]);

    /*
      set center frequency of one notch.
      spread_mul is a scale factor for spreading of double or triple notch
      harmonic_mul is the multiplier for harmonics, 1 is for the fundamental
    */
    void set_center_frequency(uint16_t idx, float center_freq_hz, float spread_mul, uint8_t harmonic_mul);

    // apply a sample to each of the underlying filters in turn
    T apply(const T &sample);
    // reset each of the underlying filters
    void reset();

    /*
      log notch center frequencies and first harmonic
     */
    void log_notch_centers(uint8_t instance, uint64_t now_us) const;

private:
    // underlying bank of notch filters
    NotchFilter<T>*  _filters;
    // sample frequency for each filter
    float _sample_freq_hz;
    // base double notch bandwidth for each filter
    float _notch_spread;
    // attenuation for each filter
    float _A;
    // quality factor of each filter
    float _Q;
    // a bitmask of the harmonics to use
    uint32_t _harmonics;
    // number of notches that make up a composite notch
    uint8_t _composite_notches;
    // number of allocated filters
    uint16_t _num_filters;
    // pre-calculated number of harmonics
    uint8_t _num_harmonics;
    // number of enabled filters
    uint16_t _num_enabled_filters;
    bool _initialised;

    // have we failed to expand filters?
    bool _alloc_has_failed;

    // minimum frequency (from INS_HNTCH_FREQ * INS_HNTCH_FM_RAT)
    float _minimum_freq;

    // pointer to params object for this filter
    HarmonicNotchFilterParams *params;
};

// Harmonic notch update mode
enum class HarmonicNotchDynamicMode {
    Fixed           = 0,
    UpdateThrottle  = 1,
    UpdateRPM       = 2,
    UpdateBLHeli    = 3,
    UpdateGyroFFT   = 4,
    UpdateRPM2      = 5,
};

/*
  harmonic notch filter configuration parameters
 */
class HarmonicNotchFilterParams : public NotchFilterParams {
public:
    enum class Options {
        DoubleNotch = 1<<0,
        DynamicHarmonic = 1<<1,
        LoopRateUpdate = 1<<2,
        EnableOnAllIMUs = 1<<3,
        TripleNotch = 1<<4,
        TreatLowAsMin = 1<<5,
    };

    HarmonicNotchFilterParams(void);

    void init();

    // set the fundamental center frequency of the harmonic notch
    void set_center_freq_hz(float center_freq) { _center_freq_hz.set(center_freq); }

    // set the bandwidth of the harmonic notch
    void set_bandwidth_hz(float bandwidth_hz) { _bandwidth_hz.set(bandwidth_hz); }

    // set the attenuation of the harmonic notch
    void set_attenuation(float attenuation_dB) { _attenuation_dB.set(attenuation_dB); }
    
    // harmonics enabled on the harmonic notch
    uint32_t harmonics(void) const { return _harmonics; }

    // set the harmonics value
    void set_harmonics(uint32_t hmncs) { _harmonics.set(hmncs); }

    // has the user set the harmonics value
    void set_default_harmonics(uint32_t hmncs) { _harmonics.set_default(hmncs); }

    // reference value of the harmonic notch
    float reference(void) const { return _reference; }
    void set_reference(float ref) { _reference.set(ref); }

    // notch options
    bool hasOption(Options option) const { return _options & uint16_t(option); }
    // notch dynamic tracking mode
    HarmonicNotchDynamicMode tracking_mode(void) const { return HarmonicNotchDynamicMode(_tracking_mode.get()); }
    static const struct AP_Param::GroupInfo var_info[];

    // return minimum frequency ratio for throttle notch
    float freq_min_ratio(void) const {
        return _freq_min_ratio;
    }
    void set_freq_min_ratio(float ratio) { _freq_min_ratio.set(ratio); }

    // set options flags
    void set_options(uint16_t options) { _options.set(options); }

    // save parameters
    void save_params();

    // return the number of composite notches given the options
    uint8_t num_composite_notches(void) const {
        return hasOption(Options::DoubleNotch) ? 2 : hasOption(Options::TripleNotch) ? 3: 1;
    }

private:
    // configured notch harmonics
    AP_Int32 _harmonics;
    // notch reference value
    AP_Float _reference;
    // notch dynamic tracking mode
    AP_Int8 _tracking_mode;
    // notch options
    AP_Int16 _options;

    // minimum frequency ratio for throttle based notches
    AP_Float _freq_min_ratio;
};

typedef HarmonicNotchFilter<Vector3f> HarmonicNotchFilterVector3f;

