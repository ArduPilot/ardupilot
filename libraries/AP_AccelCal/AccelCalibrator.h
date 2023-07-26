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
#include <AP_Math/vectorN.h>

#define ACCEL_CAL_MAX_NUM_PARAMS 9
#define ACCEL_CAL_TOLERANCE 0.1
#define MAX_ITERATIONS  50
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

    //Select options, initialise variables and initiate accel calibration
    void start(enum accel_cal_fit_type_t fit_type = ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, uint8_t num_samples = 6, float sample_time = 0.5f);
    void start(enum accel_cal_fit_type_t fit_type, uint8_t num_samples, float sample_time, Vector3f offset, Vector3f diag, Vector3f offdiag);
    
    // set Accel calibrator status to make itself ready for future accel cals
    void clear();

    // returns true if accel calibrator is running
    bool running();

    // set Accel calibrator to start collecting samples in the next cycle
    void collect_sample();

    // check if client's calibrator is active
    void check_for_timeout();

    // get diag,offset or offdiag parameters as per the selected fitting surface or request
    void get_calibration(Vector3f& offset) const;
    void get_calibration(Vector3f& offset, Vector3f& diag) const;
    void get_calibration(Vector3f& offset, Vector3f& diag, Vector3f& offdiag) const;


    // collect and avg sample to be passed onto LSQ estimator after all requisite orientations are done
    void new_sample(const Vector3f& delta_velocity, float dt);

    // interface for LSq estimator to read sample buffer sent after conversion from delta velocity
    // to averaged acc over time
    bool get_sample(uint8_t i, Vector3f& s) const;

    // returns truen and sample corrected with diag offdiag parameters as calculated by LSq estimation procedure
    // returns false if no correct parameter exists to be applied along with existing sample without corrections
    bool get_sample_corrected(uint8_t i, Vector3f& s) const;

    // set tolerance bar for parameter fitness value to cross so as to be deemed as correct values
    void set_tolerance(float tolerance) { _conf_tolerance = tolerance; }

    // returns current state of accel calibrators
    enum accel_cal_status_t get_status() const { return _status; }

    // returns number of samples collected
    uint8_t get_num_samples_collected() const { return _samples_collected; }

    // returns mean squared fitness of sample points to the selected surface
    float get_fitness() const { return _fitness; }

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
    typedef    VectorN<float, ACCEL_CAL_MAX_NUM_PARAMS> VectorP;

    union param_u {
        struct param_t s;
        VectorN<float, ACCEL_CAL_MAX_NUM_PARAMS> a;

        param_u() : a{}
        {
            static_assert(sizeof(*this) == sizeof(struct param_t),
                          "Invalid union members: sizes do not match");
        }
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
    union param_u _param;
    float _fitness;
    uint32_t _last_samp_frag_collected_ms;
    float _min_sample_dist;

    // private methods
    // check sanity of including the sample and add it to buffer if test is passed
    bool accept_sample(const Vector3f& sample);

    // reset to calibrator state before the start of calibration
    void reset_state();

    // sets status of calibrator and takes appropriate actions
    void set_status(enum accel_cal_status_t);

    // determines if the result is acceptable
    bool accept_result() const;

    // returns number of parameters are required for selected Fit type
    uint8_t get_num_params() const;

    // Function related to Gauss Newton Least square regression process
    float calc_residual(const Vector3f& sample, const struct param_t& params) const;
    float calc_mean_squared_residuals(const struct param_t& params) const;
    void calc_jacob(const Vector3f& sample, const struct param_t& params, VectorP& ret) const;
    void run_fit(uint8_t max_iterations, float& fitness);
};
