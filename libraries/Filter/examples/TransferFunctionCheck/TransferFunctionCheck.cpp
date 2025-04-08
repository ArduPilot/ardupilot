// Example and MATLAB script for verifying transfer functions of filters as implemented

/* on Linux run with
    ./waf configure --board linux
    ./waf --targets examples/TransferFunctionCheck
    ./build/linux/examples/TransferFunctionCheck
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>
#include <Filter/NotchFilter.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Some helper classes to allow accessing protected variables, also useful for adding type specific prints

class LowPassConstDtHelper : public LowPassFilterConstDtFloat {
public:
    using LowPassFilterConstDtFloat::LowPassFilterConstDtFloat;

    void set_cutoff_frequency_override(float sample_freq, float new_cutoff_freq) {
        // Stash the sample rate so we can use it later
        _sample_freq = sample_freq;
        set_cutoff_frequency(sample_freq, new_cutoff_freq);
    }

    // Were really cheating here and using the same method as the filter to get the coefficient
    // rather than pulling the coefficient directly
    void print_transfer_function() {
        hal.console->printf("LowPassFilterConstDtFloat\n");
        hal.console->printf("Sample rate: %.9f Hz, Cutoff: %.9f Hz\n", _sample_freq, get_cutoff_freq());
        hal.console->printf("Low pass filter in the form: H(z) = a/(1-(1-a)*z^-1)\n");
        hal.console->printf("a: %.9f\n", calc_lowpass_alpha_dt(1.0/_sample_freq, get_cutoff_freq()));
    }

private:
    float _sample_freq;
};

class LowPassHelper : public LowPassFilterFloat {
public:
    using LowPassFilterFloat::LowPassFilterFloat;

    void set_cutoff_frequency_override(float sample_freq, float new_cutoff_freq) {
        // Stash the DT so we can use it later
        _DT = 1.0 / sample_freq;
        set_cutoff_frequency(new_cutoff_freq);
    }

    // Were really cheating here and using the same method as the filter to get the coefficient
    // rather than pulling the coefficient directly
    void print_transfer_function() {
        hal.console->printf("LowPassFilterFloat\n");
        hal.console->printf("Sample rate: %.9f Hz, Cutoff: %.9f Hz\n", 1.0 / _DT, get_cutoff_freq());
        hal.console->printf("Low pass filter in the form: H(z) = a/(1-(1-a)*z^-1)\n");
        hal.console->printf("a: %.9f\n", calc_lowpass_alpha_dt(_DT, get_cutoff_freq()));
    }

    float apply_override(const float sample) {
        return apply(sample, _DT);
    }

private:
    float _DT;
};

class LowPass2pHelper : public LowPassFilter2pFloat {
public:
    using LowPassFilter2pFloat::LowPassFilter2pFloat;

    // Print transfer function and variables to console
    void print_transfer_function() {
        hal.console->printf("LowPassFilter2pFloat\n");
        hal.console->printf("Sample rate: %.9f Hz, Cutoff: %.9f Hz\n", get_sample_freq(), get_cutoff_freq());
        hal.console->printf("Biquad filter in the form: H(z) = (b0 + b1*z^-1 + b2*z^-2)/(1 + a1*z^-1 + a2*z^-2)\n");
        hal.console->printf("a1: %.9f, a2: %.9f, b0: %.9f, b1: %.9f, b2: %.9f\n", _params.a1, _params.a2, _params.b0, _params.b1, _params.b2);
    }
};

class NotchHelper : public NotchFilterFloat {
public:
    using NotchFilterFloat::NotchFilterFloat;

    // Print transfer function and variables to console
    void print_transfer_function() {
        hal.console->printf("NotchFilterFloat\n");
        hal.console->printf("Sample rate: %.9f Hz, Center: %.9f Hz\n", _sample_freq_hz, _center_freq_hz);
        hal.console->printf("Notch filter in the form: H(z) = (b0 + b1*z^-1 + b2*z^-2)/(a0 + a1*z^-1 + a2*z^-2)\n");
        hal.console->printf("a0: %.9f, a1: %.9f, a2: %.9f, b0: %.9f, b1: %.9f, b2: %.9f\n", 1.0, a1, a2, b0, b1, b2);
    }
};


// create an instance each filter to test
LowPassConstDtHelper lowpassConstDt;
LowPassHelper lowpass;
LowPass2pHelper biquad;
NotchHelper notch;
NotchHelper notch2;

enum class filter_type {
    LowPassConstDT,
    LowPass,
    Biquad,
    Notch,
    Combination,
} type;

void setup();
void loop();
void reset_all();
float apply_to_filter_under_test(float input);
void sweep(uint16_t num_samples,  uint16_t max_freq, float sample_rate);

void loop() {};

// setup routine
void setup()
{
    hal.console->printf("Frequency sweep transfer function test\n");
    hal.console->printf("Sweeping a range of frequencies in the form sin(2*pi*t*f)\n");

    // Set sample rate and target frequency
    const float sample_rate = 1000;
    const float target_freq = 50;

    type = filter_type::LowPassConstDT;

    // Run 1000 time steps at each frequency
    const uint16_t num_samples = 1000;

    // Run upto 150 hz
    const uint16_t max_freq = 150;

    // Print transfer function of filter under test
    hal.console->printf("\n");
    switch (type) {
    case filter_type::LowPassConstDT:
        lowpassConstDt.set_cutoff_frequency_override(sample_rate, target_freq);
        lowpassConstDt.print_transfer_function();
        break;

    case filter_type::LowPass:
        lowpass.set_cutoff_frequency_override(sample_rate, target_freq);
        lowpass.print_transfer_function();
        break;

    case filter_type::Biquad:
        biquad.set_cutoff_frequency(sample_rate, target_freq);
        biquad.print_transfer_function();
        break;

    case filter_type::Notch:
        notch.init(sample_rate, target_freq, target_freq*0.25, 40);
        notch.print_transfer_function();
        break;

    case filter_type::Combination:
        // A combination of two notches and a biquad lowpass, representative of a typical setup
        notch.init(sample_rate, target_freq, target_freq*0.25, 40);
        notch2.init(sample_rate, target_freq*2.00, target_freq*0.25, 40);
        biquad.set_cutoff_frequency(sample_rate, target_freq*1.5);
        notch.print_transfer_function();
        hal.console->printf("\n");
        notch2.print_transfer_function();
        hal.console->printf("\n");
        biquad.print_transfer_function();
        break;
    }
    hal.console->printf("\n");

    // Run sweep over given range
    sweep(num_samples, max_freq, sample_rate);

    // Wait a while for print buffer to empty and exit
    hal.scheduler->delay(1000);
    exit(0);
}

void reset_all()
{
    lowpassConstDt.reset(0.0);
    lowpass.reset(0.0);
    biquad.reset(0.0);
    notch.reset();
    notch2.reset();
}

float apply_to_filter_under_test(float input)
{
    switch (type) {
        case filter_type::LowPassConstDT:
            return lowpassConstDt.apply(input);

        case filter_type::LowPass:
            return lowpass.apply_override(input);

        case filter_type::Biquad:
            return biquad.apply(input);

        case filter_type::Notch:
            return notch.apply(input);

        case filter_type::Combination: // Ordering does not matter
            return biquad.apply(notch.apply(notch2.apply(input)));

    }
    return input;
}

void sweep(uint16_t num_samples,  uint16_t max_freq, float sample_rate)
{
    // print header
    hal.console->printf("f(hz)");
    for (uint16_t i = 0; i < num_samples; i++) {
        hal.console->printf(", t = %.9f", i / sample_rate);
    }
    hal.console->printf("\n");


    for (uint16_t f = 1; f <= max_freq; f++) {
        // Print freq and reset filter
        hal.console->printf("%i", f);
        reset_all();

        // Run over the given number of samples
        for (uint16_t i = 0; i < num_samples; i++) {
            const float t = i / sample_rate;
            const float input = sinf(M_2PI*t*f);
            const float output = apply_to_filter_under_test(input);
            hal.console->printf(", %+.9f", output);
        }
        hal.console->printf("\n");

        // Try not to overflow the print buffer
        hal.scheduler->delay(100);
    }
}

AP_HAL_MAIN();
