#include <AP_HAL/AP_HAL.h>

#include <AP_Math/chirp.h>
#include <APM_Control/AP_RollController.h>
#include <APM_Control/AP_PitchController.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <AP_AHRS/AP_AHRS.h>
#include <stdio.h>
#include <errno.h>
#include <AP_Scheduler/AP_Scheduler.h>

/* run with:
    ./waf configure --board linux
    ./waf build --targets examples/AP_FW_Controller_test
    ./build/linux/examples/AP_FW_Controller_test
*/

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Need a scheduler so the controllers can get DT
AP_Scheduler scheduler;

// When using stop clock the normal hal scheduler sleeps don't work.
// We need to sleep to clear print buffer, so microsleep implementation is copied here
void microsleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
}


// Chirp object to generate test signal
Chirp chirp;

// Chirp setup params
const float magnitude = 60.0; // deg - large magnitude means slew limit should kick in (very high values needed because of low default gains)
const float duration = 60.0; // seconds
const float frequency_start = 1.0; // hz
const float frequency_stop = 20; // hz, stay under the 25Hz nyquist limit on a 50Hz loop rate
const float time_fade_in = 15; // seconds
const float time_fade_out = 10; // seconds
const float time_const_freq = 2.0 / frequency_start; // seconds

// Param object used by controllers
AP_FixedWing params;

// Controller objects to test
AP_RollController roll_control { params };
AP_PitchController pitch_control { params };

// 50 hz loop, start at time 0
// Loop rate must match SCHEDULER_DEFAULT_LOOP_RATE
const uint16_t dt_us = 20000;
uint64_t waveform_time_us = 0;

// Scaling speed parameter value
float scaling_speed = 15;

// Default values for user provided configuration
enum class Axis {
    Roll = 0,
    Pitch = 1,
} test_axis = Axis::Roll;
bool override_have_airspeed = true;
float override_airspeed = 10;
float override_roll = 0;
float override_pitch = 0;
bool ground_mode = false;
bool disable_integrator = false;

// Define the AHRS functions we need
AP_AHRS::AP_AHRS(uint8_t flags) :
    _ekf_flags(flags)
{
    _singleton = this;
}

// This is a dirty hack so we can set private values inside the ahrs
void AP_AHRS::update(bool skip_ins_update)
{
    roll = override_roll;
    pitch = override_pitch;
    update_cd_values();

    state.airspeed = override_airspeed;
    state.airspeed_ok = override_have_airspeed;
}

// Copys of the functions we need out of AP_AHRS.cpp
bool AP_AHRS::airspeed_estimate(float &airspeed_ret) const
{
    airspeed_ret = state.airspeed;
    return state.airspeed_ok;
}
float AP_AHRS::get_EAS2TAS(void) const
{
    if (is_positive(state.EAS2TAS)) {
        return state.EAS2TAS;
    }
    return 1.0;
}

// singleton instance
AP_AHRS *AP_AHRS::_singleton;

namespace AP {
AP_AHRS &ahrs() {
    return *AP_AHRS::get_singleton();
}
}

AP_AHRS ahrs;

// Calculate the airspeed scaler based on airspeed and the configured param
// Method from `Plane::calc_speed_scaler`
float calc_speed_scaler()
{
    float aspeed;
    if (!ahrs.airspeed_estimate(aspeed)) {
        return 1.0;
    }

    // ensure we have scaling over the full configured airspeed
    const float airspeed_min = MAX(params.airspeed_min, 5);
    const float scale_min = MIN(0.5, scaling_speed / (2.0 * params.airspeed_max));
    const float scale_max = MAX(2.0, scaling_speed / (0.7 * airspeed_min));
    float speed_scaler = scale_max;
    if (aspeed > 0.0001) {
        speed_scaler = scaling_speed / aspeed;
    }
    return constrain_float(speed_scaler, scale_min, scale_max);
}


// setup function
void setup()
{
    // Read in user provided configuration
    uint8_t argc;
    char * const *argv;
    hal.util->commandline_arguments(argc, argv);
    if (argc > 1) {
        for (uint8_t i = 1; i < argc; i++) {
            const char *arg = argv[i];
            const char *eq = strchr(arg, '=');

            if (eq == NULL) {
                ::printf("Expected argument with \"=\"\n");
                exit(1);
            }

            char cmd[20] {};
            strncpy(cmd, arg, eq-arg);
            const float value = atof(eq+1);
            if (strcmp(cmd, "axis") == 0) {
                if (strcmp(eq+1, "roll") == 0) {
                    test_axis = Axis::Roll;
                } else if (strcmp(eq+1, "pitch") == 0) {
                    test_axis = Axis::Pitch;

                } else {
                    ::printf("Unknown axis: %s\n", eq+1);
                    exit(1);
                }

            } else if (strcmp(cmd, "roll") == 0) {
                override_roll = radians(value);

            } else if (strcmp(cmd, "pitch") == 0) {
                override_pitch = radians(value);

            } else if (strcmp(cmd, "airspeed") == 0) {
                override_airspeed = value;

            } else if (strcmp(cmd,"airspeed_fail") == 0) {
                override_have_airspeed = value <= 0.0;

            } else if (strcmp(cmd,"ground_mode") == 0) {
                ground_mode = value > 0.0;

            } else if (strcmp(cmd,"disable_integrator") == 0) {
                disable_integrator = value > 0.0;

            } else {
                ::printf("Expected \"axis\", \"roll\", \"pitch\", \"airspeed\", \"airspeed_fail\",  \"ground_mode\",  \"disable_integrator\"\n");
                exit(1);
            }
        }
    }

    ::printf("FixedWing controller test - ");
    switch (test_axis) {
        case Axis::Roll:
            ::printf("Roll");
            break;

        case Axis::Pitch:
            ::printf("Pitch");
            break;
    }
    ::printf(" - ground_mode: %i, disable_integrator: %i\n", ground_mode, disable_integrator);

    // Set the configured values into the AHRS
    ahrs.update();

    // Set default values for the params used by the controllers
    params.airspeed_min.set(9);
    params.airspeed_max.set(22);
    params.roll_limit.set(45);
    params.pitch_limit_max.set(20);
    params.pitch_limit_min.set(-25);
    params.autotune_level.set(6);
    params.autotune_options.set(0);

    // Setup chirp object
    chirp.init(duration, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    // Force clock to start at 0
    hal.scheduler->stop_clock(waveform_time_us);

    // Print constant inputs
    ::printf("Chirp - duration: %fs, magnitude: %f, start: %fHz, stop: %fHz, fade in: %fs, fade out:%fs, hold: %fs\n", duration, magnitude, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    // Print AHRS state
    float airspeed = -1;
    bool airspeed_ok = ahrs.airspeed_estimate(airspeed);
    ::printf("AHRS - roll: %f, pitch: %f, airspeed: %f, airspeed OK: %i, EAS2TAS %f, gyro: { %f, %f, %f }\n", ahrs.roll_sensor * 0.01, ahrs.pitch_sensor * 0.01, airspeed, airspeed_ok, ahrs.get_EAS2TAS(), ahrs.get_gyro().x, ahrs.get_gyro().y, ahrs.get_gyro().z);

    // Header for results
    ::printf("Time (s), angle error, output, PID target, PID actual, error, P, I, D, FF, DFF, Dmod, Slew rate, limit, PD limit, reset, I term set\n");

}

void loop(void)
{
    // Update the chirp to generate input for controller
    const float waveform_time_s = waveform_time_us * 1.0e-6;
    const float nav_angle_cd = chirp.update(waveform_time_s, magnitude) * 100.0;

    // Calculate speed scaler
    const float speed_scaler = calc_speed_scaler();

    // Run the controller
    float angle_error_cd;
    float output;
    const AP_PIDInfo* info = nullptr;

    switch (test_axis) {
        case Axis::Roll:
            angle_error_cd = nav_angle_cd - ahrs.roll_sensor;
            output = roll_control.get_servo_out(angle_error_cd, speed_scaler, disable_integrator, ground_mode);
            info = &roll_control.get_pid_info();
            break;

        case Axis::Pitch:
            angle_error_cd = nav_angle_cd - ahrs.pitch_sensor;
            output = pitch_control.get_servo_out(angle_error_cd, speed_scaler, disable_integrator, ground_mode);
            info = &pitch_control.get_pid_info();
            break;
    }

    // Print results
    ::printf("%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%u,%u,%u,%u\n",
        waveform_time_s,
        angle_error_cd * 0.01,
        output,
        info->target,
        info->actual,
        info->error,
        info->P,
        info->I,
        info->D,
        info->FF,
        info->DFF,
        info->Dmod,
        info->slew_rate,
        info->limit,
        info->PD_limit,
        info->reset,
        info->I_term_set);

    if (chirp.completed()) {
        // Sleep gives the print buffer a chance to keep up
        microsleep(10000);
        exit(1);
    }

    // Force clock times so we can run faster than real time
    waveform_time_us += dt_us;
    hal.scheduler->stop_clock(waveform_time_us);

}

AP_HAL_MAIN();
