#ifndef __ACCELCALIBRATOR_H__
#define __ACCELCALIBRATOR_H__
#include <AP_Math.h>

#define ACCEL_CAL_MAX_NUM_PARAMS 9

#define MAX_ITERATIONS
enum accel_cal_status_t {
    ACCEL_CAL_NOT_STARTED=0,
    ACCEL_CAL_WAITING_FOR_ORIENTATION=1,
    ACCEL_CAL_COLLECTING_SAMPLE=2,
    ACCEL_CAL_SUCCESS=3,
    ACCEL_CAL_FAILED=4
};

enum accel_cal_fit_type_t {
    ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID=0,
    ACCEL_CAL_ELLIPSOID=1
};

class AccelCalibrator {
public:
    AccelCalibrator();
    void start(enum accel_cal_fit_type_t fit_type = ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, uint8_t num_samples = 6, float sample_time = 0.5f);
    void start(enum accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time, Vector3f offset, Vector3f diag, Vector3f offdiag);
    void clear();
    bool running();
    void collect_sample();
    void check_for_timeout();
    void new_sample(Vector3f delta_velocity, float dt);
    void get_calibration(Vector3f& offset);
    void get_calibration(Vector3f& offset, Vector3f& diag);
    void get_calibration(Vector3f& offset, Vector3f& diag, Vector3f& offdiag);
    bool get_sample(uint8_t i, Vector3f& s) const;
    bool get_sample_corrected(uint8_t i, Vector3f& s) const;
    void set_tolerance(float tolerance) { _conf_tolerance = tolerance; }
    enum accel_cal_status_t get_status() const { return _status; }
    uint8_t get_num_samples_collected() const { return _samples_collected; }

    float get_fitness() { return _fitness; }

    struct param_t {
        Vector3f offset;
        Vector3f diag;
        Vector3f offdiag;
    };

private:
    struct AccelSample {
        Vector3f delta_velocity;
        float delta_time;
    };

    //configuration
    uint8_t _conf_num_samples;
    float _conf_sample_time;
    enum accel_cal_fit_type_t _conf_fit_type;
    float _conf_tolerance;

    // state
    accel_cal_status_t _status;
    struct AccelSample* _sample_buffer;
    uint8_t _samples_collected;
    struct param_t _params;
    float _fitness;
    uint32_t _last_samp_frag_collected_ms;

    // private methods
    bool accept_sample(const Vector3f& sample);
    void reset_state();
    void set_status(enum accel_cal_status_t);
    uint8_t get_num_params();
    bool accept_result() const;

    float calc_residual(const Vector3f& sample, const struct param_t& params) const;
    float calc_mean_squared_residuals() const;
    float calc_mean_squared_residuals(const struct param_t& params) const;
    void calc_jacob(const Vector3f& sample, const struct param_t& params, float* ret) const;
    void run_fit(uint8_t max_iterations, float& fitness);
};
#endif //__ACCELCALIBRATOR_H__
