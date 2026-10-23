/*
 * chirp.cpp
 *
 * Copyright (C) Leonard Hall 2020
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
* This object generates a chirp signal based on the input variables.  A chirp is
* a sine wave starting from the minimum frequency and ending at the maximum frequency.
* The progression in frequency is not linear but designed to increase exponentially.
* The chirp can be designed to dwell at the minimum frequency for a specified time
* before sweeping through the frequencies and also can fade in the magnitude at the
* beginning and fade out the magnitude at the end.  This object can also generate
* constant frequency sine waves by setting the minimum and maximum frequency to the
* same value.
*/
#include <AP_Math/AP_Math.h>
#include "chirp.h"

// constructor
Chirp::Chirp() {}

// initializes the chirp object
void Chirp::init(float time_record, float frequency_start_hz, float frequency_stop_hz, float time_fade_in, float time_fade_out, float time_const_freq)
{
    // pass in variables to class
    record = time_record;
    wMin = M_2PI * frequency_start_hz;
    wMax = M_2PI * frequency_stop_hz;
    fade_in = time_fade_in;
    fade_out = time_fade_out;
    const_freq = time_const_freq;

    B = logf(wMax / wMin);

    // Mark as incomplete
    complete = false;
}

// determine chirp signal output at the specified time and amplitude
float Chirp::update(float time, float waveform_magnitude)
{
    magnitude = waveform_magnitude;
    if (time <= 0.0f) {
        window = 0.0f;
    } else if (time <= fade_in) {
        window = 0.5 - 0.5 * cosf(M_PI * time / fade_in);
    } else if (time <= record - fade_out) {
        window = 1.0;
    } else if (time <= record) {
        window = 0.5 - 0.5 * cosf(M_PI * (time - (record - fade_out)) / fade_out + M_PI);
    } else {
        window = 0.0;
    }

    if (time <= 0.0f) {
        waveform_freq_rads = wMin;
        output = 0.0f;
    } else if (time <= const_freq) {
        waveform_freq_rads = wMin;
        output = window * magnitude * sinf(wMin * time - wMin * const_freq);
    } else if (time <= record) {
        // handles constant frequency dwells and chirps
        if (is_equal(wMin, wMax)) {
            waveform_freq_rads = wMin;
            output = window * magnitude * sinf(wMin * time);
        } else {
            waveform_freq_rads = wMin * expf(B * (time - const_freq) / (record - const_freq));
            output = window * magnitude * sinf((wMin * (record - const_freq) / B) * (expf(B * (time - const_freq) / (record - const_freq)) - 1));
        }
    } else {
        waveform_freq_rads = wMax;
        output = 0.0f;
    }

    complete = time > record;

    return output;
}
