#pragma once

class Chirp {

public:

    // constructor
    Chirp();

    // initializes the chirp object
    void init(float time_record, float frequency_start_hz, float frequency_stop_hz, float time_fade_in, float time_fade_out, float time_const_freq);

    // determine chirp signal output at the specified time and amplitude
    float update(float time, float waveform_magnitude);

    // accessor for the current waveform frequency
    float get_frequency_rads() {return waveform_freq_rads; }

    // Return true if chirp is completed
    bool completed() const { return complete; }

private:
    // Total chirp length in seconds
    float record;

    // Chirp oscillation amplitude
    float magnitude;

    // Chirp start frequency in rad/s
    float wMin;

    // Chirp end frequency in rad/s
    float wMax;

    // Amplitude fade in time in seconds
    float fade_in;

    // Amplitude fade out time in seconds
    float fade_out;

    // Time that chirp will remain at the min frequency before increasing to max frequency
    float const_freq;

    // frequency ratio
    float B;

    // current waveform frequency in rad/s
    float waveform_freq_rads;

    // current amplitude of chirp
    float window;

    // output of chirp signal at the requested time
    float output;

    // True if chirp is complete, reset to false on init
    bool complete;

};
