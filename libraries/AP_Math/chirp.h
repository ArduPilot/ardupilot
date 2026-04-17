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
    float record{0.0f};

    // Chirp oscillation amplitude
    float magnitude{0.0f};

    // Chirp start frequency in rad/s
    float wMin{0.0f};

    // Chirp end frequency in rad/s
    float wMax{0.0f};

    // Amplitude fade in time in seconds
    float fade_in{0.0f};

    // Amplitude fade out time in seconds
    float fade_out{0.0f};

    // Time that chirp will remain at the min frequency before increasing to max frequency
    float const_freq{0.0f};

    // frequency ratio
    float B{0.0f};

    // current waveform frequency in rad/s
    float waveform_freq_rads{0.0f};

    // current amplitude of chirp
    float window{0.0f};

    // output of chirp signal at the requested time
    float output{0.0f};

    // True if chirp is complete, reset to false on init
    bool complete{false};

};
