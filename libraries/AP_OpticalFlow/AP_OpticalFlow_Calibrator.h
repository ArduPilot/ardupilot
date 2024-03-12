#pragma once

#include <AP_Math/AP_Math.h>

#define AP_OPTICALFLOW_CAL_MAX_SAMPLES 50  // number of samples required before calibration begins

class AP_OpticalFlow_Calibrator {
public:
    AP_OpticalFlow_Calibrator() {};

    // start or stop the calibration
    void start();
    void stop();

    // update the state machine and calculate scaling
    // returns true if new scaling values have been found
    bool update();

    // get final scaling values
    // scaling values used during sample collection should be multiplied by these scalars
    Vector2f get_scalars();

private:

    // single sample for a single axis
    struct sample_t {
        float flow_rate;
        float body_rate;
        float los_pred;
    };

    // attempt to add a new sample to the buffer
    void add_sample(uint32_t timestamp_ms, const Vector2f& flow_rate, const Vector2f& body_rate, const Vector2f& los_pred);

    // returns true once the sample buffer is full
    bool sample_buffers_full() const;

    // run calibration algorithm for both axis
    // returns true on success and updates _cal_data[0,1].best_scale and best_scale_fitness
    bool run_calibration();

    // Run fitting algorithm for all samples of the given axis
    // returns a scalar and fitness (lower numbers mean a better result) in the arguments provided
    bool calc_scalars(uint8_t axis, float& scalar, float& fitness);

    // calculate a single sample's residual given a scalar parameter
    float calc_sample_residual(const sample_t& sample, float scalar) const;

    // calculate the scalar that minimises the residual for a single sample
    // returns true on success and populates the best_scalar argument
    bool calc_sample_best_scalar(const sample_t& sample, float& best_scalar) const;

    // calculate mean squared residual for all samples of a single axis (0 or 1) given a scalar parameter
    float calc_mean_squared_residuals(uint8_t axis, float scalar) const;

    // log a sample
    void log_sample(uint8_t axis, uint8_t sample_num, float flow_rate, float body_rate, float los_pred);

    // calibration states
    enum class CalState {
        NOT_STARTED = 0,
        RUNNING,            // collecting samples
        READY_TO_CALIBRATE, // ready to calibrate (may wait until vehicle is disarmed)
        SUCCESS,
        FAILED
    } _cal_state;

    // local variables
    uint32_t _start_time_ms;                                // time the calibration was started
    struct {
        sample_t samples[AP_OPTICALFLOW_CAL_MAX_SAMPLES];   // buffer of sensor samples
        uint8_t num_samples;                                // number of samples in samples buffer
        float best_scalar;                                  // best scaling value found so far
        float best_scalar_fitness;                          // fitness (rms of error) of best scaling value
    } _cal_data[2];                                         // x and y axis
    uint32_t _last_sample_timestamp_ms;                     // system time of last sample's timestamp, used to ignore duplicates
    uint32_t _last_report_ms;                               // system time of last status report
};
