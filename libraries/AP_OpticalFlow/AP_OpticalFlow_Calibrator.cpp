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

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_CALIBRATOR_ENABLED

#include "AP_OpticalFlow_Calibrator.h"
#include <AP_InternalError/AP_InternalError.h>

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

const uint32_t AP_OPTICALFLOW_CAL_TIMEOUT_SEC = 120;        // calibration timesout after 120 seconds
const uint32_t AP_OPTICALFLOW_CAL_STATUSINTERVAL_SEC = 3;   // status updates printed at 3 second intervals
const float AP_OPTICALFLOW_CAL_YAW_MAX_RADS = radians(20);  // maximum yaw rotation (must be low to ensure good scaling)
const float AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS = radians(20);    // minimum acceptable roll or pitch rotation
const float AP_OPTICALFLOW_CAL_SCALE_MIN = 0.20f;           // min acceptable scaling value.  If resulting scaling is below this then the calibration fails
const float AP_OPTICALFLOW_CAL_SCALE_MAX = 4.0f;            // max acceptable scaling value.  If resulting scaling is above this then the calibration fails
const float AP_OPTICALFLOW_CAL_FITNESS_THRESH = 0.5f;       // min acceptable fitness
const float AP_OPTICALFLOW_CAL_RMS_FAILED = 1.0e30f;        // calc_mean_squared_residuals returns this value when it fails to calculate a good value

extern const AP_HAL::HAL& hal;

// start the calibration
void AP_OpticalFlow_Calibrator::start()
{
    // exit immediately if already running
    if (_cal_state == CalState::RUNNING) {
        return;
    }

    _cal_state = CalState::RUNNING;
    _start_time_ms = AP_HAL::millis();

    // clear samples buffers
    _cal_data[0].num_samples = 0;
    _cal_data[1].num_samples = 0;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlowCal: Started");
}

void AP_OpticalFlow_Calibrator::stop()
{
    // exit immediately if already stopped
    if (_cal_state == CalState::NOT_STARTED) {
        return;
    }

    _cal_state = CalState::NOT_STARTED;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FlowCal: Stopped");
}

// update the state machine and calculate scaling
bool AP_OpticalFlow_Calibrator::update()
{
    // prefix for reporting
    const char* prefix_str = "FlowCal:";

    // while running add samples
    if (_cal_state == CalState::RUNNING) {
        uint32_t now_ms = AP_HAL::millis();
        uint32_t timestamp_ms;
        Vector2f flow_rate, body_rate, los_pred;
        if (AP::ahrs().getOptFlowSample(timestamp_ms, flow_rate, body_rate, los_pred)) {
            add_sample(timestamp_ms, flow_rate, body_rate, los_pred);

            // while collecting samples display percentage complete
            if (now_ms - _last_report_ms > AP_OPTICALFLOW_CAL_STATUSINTERVAL_SEC * 1000UL) {
                _last_report_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s x:%d%% y:%d%%",
                                prefix_str,
                                (int)((_cal_data[0].num_samples * 100.0 / AP_OPTICALFLOW_CAL_MAX_SAMPLES)),
                                (int)((_cal_data[1].num_samples * 100.0 / AP_OPTICALFLOW_CAL_MAX_SAMPLES)));
            }

            // advance state once sample buffers are full
            if (sample_buffers_full()) {
                _cal_state = CalState::READY_TO_CALIBRATE;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s samples collected", prefix_str);
            }
        }

        // check for timeout
        if (now_ms - _start_time_ms > AP_OPTICALFLOW_CAL_TIMEOUT_SEC * 1000UL) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s timeout", prefix_str);
            _cal_state = CalState::FAILED;
        }
    }

    // start calibration
    if (_cal_state == CalState::READY_TO_CALIBRATE) {
        // run calibration and mark failure or success
        if (run_calibration()) {
            _cal_state = CalState::SUCCESS;
            return true;
        } else {
            _cal_state = CalState::FAILED;
        }
    }

    // return indicating calibration is not complete
    return false;
}

// get final scaling values
// scaling values used during sample collection should be multiplied by these scalars
Vector2f AP_OpticalFlow_Calibrator::get_scalars()
{
    // return best scaling values
    return Vector2f{_cal_data[0].best_scalar, _cal_data[1].best_scalar};
}

// add new sample to the calibrator
void AP_OpticalFlow_Calibrator::add_sample(uint32_t timestamp_ms, const Vector2f& flow_rate, const Vector2f& body_rate, const Vector2f& los_pred)
{
    // return immediately if not running
    if (_cal_state != CalState::RUNNING) {
        return;
    }

    // check for duplicates
    if (timestamp_ms == _last_sample_timestamp_ms) {
        return;
    }
    _last_sample_timestamp_ms = timestamp_ms;

    // check yaw rotation is low
    const Vector3f gyro = AP::ahrs().get_gyro();
    if (fabsf(gyro.z) > AP_OPTICALFLOW_CAL_YAW_MAX_RADS) {
        return;
    }

    // check enough roll or pitch movement and record sample
    const bool rates_x_sufficient = (fabsf(body_rate.x) >= AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS) && (fabsf(flow_rate.x) >= AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS);
    if (rates_x_sufficient && (_cal_data[0].num_samples < ARRAY_SIZE(_cal_data[0].samples))) {
# if HAL_LOGGING_ENABLED
        log_sample(0, _cal_data[0].num_samples, flow_rate.x, body_rate.x, los_pred.x);
#endif
        _cal_data[0].samples[_cal_data[0].num_samples].flow_rate = flow_rate.x;
        _cal_data[0].samples[_cal_data[0].num_samples].body_rate = body_rate.x;
        _cal_data[0].samples[_cal_data[0].num_samples].los_pred = los_pred.x;
        _cal_data[0].num_samples++;
    }
    const bool rates_y_sufficient = (fabsf(body_rate.y) >= AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS) && (fabsf(flow_rate.y) >= AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS);
    if (rates_y_sufficient && (_cal_data[1].num_samples < ARRAY_SIZE(_cal_data[1].samples))) {
# if HAL_LOGGING_ENABLED
        log_sample(1, _cal_data[1].num_samples, flow_rate.y, body_rate.y, los_pred.y);
#endif
        _cal_data[1].samples[_cal_data[1].num_samples].flow_rate = flow_rate.y;
        _cal_data[1].samples[_cal_data[1].num_samples].body_rate = body_rate.y;
        _cal_data[1].samples[_cal_data[1].num_samples].los_pred = los_pred.y;
        _cal_data[1].num_samples++;
    }
}

// returns true once the sample buffer is full
bool AP_OpticalFlow_Calibrator::sample_buffers_full() const
{
    return ((_cal_data[0].num_samples >= ARRAY_SIZE(_cal_data[0].samples)) && (_cal_data[1].num_samples >= ARRAY_SIZE(_cal_data[1].samples)));
}

// run calibration algorithm for both axis
// returns true on success and updates _cal_data[0,1].best_scale and best_scale_fitness
bool AP_OpticalFlow_Calibrator::run_calibration()
{
    // run calibration for x and y axis
    const bool calx_res = calc_scalars(0, _cal_data[0].best_scalar, _cal_data[0].best_scalar_fitness);
    const bool caly_res = calc_scalars(1, _cal_data[1].best_scalar, _cal_data[1].best_scalar_fitness);

    return calx_res && caly_res;
}

// Run Gauss Newton fitting algorithm for all samples of the given axis
// returns a scalar and fitness (lower numbers mean a better result) in the arguments provided
bool AP_OpticalFlow_Calibrator::calc_scalars(uint8_t axis, float& scalar, float& fitness)
{
    // prefix for reporting
    const char* prefix_str = "FlowCal:";
    const char* axis_str = axis == 0 ? "x" : "y";

    // check we have samples
    // this should never fail because this method should only be called once the sample buffer is full
    const uint8_t num_samples = _cal_data[axis].num_samples;
    if (num_samples == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s failed because no samples", prefix_str);
        return false;
    }

    // calculate total absolute residual from all samples
    float total_abs_residual = 0;
    for (uint8_t i = 0; i < num_samples; i++) {
        const sample_t& samplei = _cal_data[axis].samples[i];
        total_abs_residual += fabsf(calc_sample_residual(samplei, 1.0));
    }

    // if there are no residuals then scaling is perfect
    if (is_zero(total_abs_residual)) {
        scalar = 1.0;
        fitness = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s perfect scalar%s of 1.0", prefix_str, axis_str);
        return true;
    }

    // for each sample calculate the residual and scalar that best reduces the residual
    float best_scalar_total = 0;
    for (uint8_t i = 0; i < num_samples; i++) {
        float sample_best_scalar;
        const sample_t& samplei = _cal_data[axis].samples[i];
        if (!calc_sample_best_scalar(samplei, sample_best_scalar)) {
            // failed to find the best scalar for a single sample
            // this should never happen because of checks when capturing samples
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s failed because of zero flow rate", prefix_str);
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
        }
        const float sample_residual = calc_sample_residual(samplei, 1.0);
        best_scalar_total += sample_best_scalar * fabsf(sample_residual) / total_abs_residual;
    }

    // check for out of range results
    if (best_scalar_total < AP_OPTICALFLOW_CAL_SCALE_MIN) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s scalar%s:%4.3f too low (<%3.1f)", prefix_str, axis_str, (double)best_scalar_total, (double)AP_OPTICALFLOW_CAL_SCALE_MIN);
        return false;
    }
    if (best_scalar_total > AP_OPTICALFLOW_CAL_SCALE_MAX) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s scalar%s:%4.3f too high (>%3.1f)", prefix_str, axis_str, (double)best_scalar_total, (double)AP_OPTICALFLOW_CAL_SCALE_MAX);
        return false;
    }

    // check for poor fitness
    float fitness_new = calc_mean_squared_residuals(axis, best_scalar_total);
    if (fitness_new > AP_OPTICALFLOW_CAL_FITNESS_THRESH) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s scalar%s:%4.3f fit:%4.3f too high (>%3.1f)", prefix_str, axis_str, (double)scalar, (double)fitness_new, (double)AP_OPTICALFLOW_CAL_FITNESS_THRESH);
        return false;
    }

    // success if fitness has improved
    float fitness_orig = calc_mean_squared_residuals(axis, 1.0);
    if (fitness_new <= fitness_orig) {
        scalar = best_scalar_total;
        fitness = fitness_new;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s scalar%s:%4.3f fit:%4.2f", prefix_str, axis_str, (double)scalar, (double)fitness);
        return true;
    }

    // failed to find a better scalar than 1.0
    scalar = 1.0;
    fitness = fitness_orig;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s no better scalar%s:%4.3f (fit:%4.3f > orig:%4.3f)", prefix_str, axis_str, (double)best_scalar_total, (double)fitness_new, (double)fitness_orig);
    return false;
}

// calculate a single sample's residual
float AP_OpticalFlow_Calibrator::calc_sample_residual(const sample_t& sample, float scalar) const
{
    return (sample.body_rate + ((sample.flow_rate * scalar) - sample.los_pred));
}

// calculate the scalar that minimises the residual for a single sample
// returns true on success and populates the best_scalar argument
bool AP_OpticalFlow_Calibrator::calc_sample_best_scalar(const sample_t& sample, float& best_scalar) const
{
    // if sample's flow_rate is zero scalar has no effect
    // this should never happen because samples should have been checked before being added
    if (is_zero(sample.flow_rate)) {
        return false;
    }
    best_scalar = (sample.los_pred - sample.body_rate) / sample.flow_rate;
    return true;
}

// calculate mean squared residual for all samples of a single axis (0 or 1) given a scalar parameter
float AP_OpticalFlow_Calibrator::calc_mean_squared_residuals(uint8_t axis, float scalar) const
{
    // sanity check axis
    if (axis >= 2) {
        return AP_OPTICALFLOW_CAL_RMS_FAILED;
    }

    // calculate and sum residuals of each sample
    float sum = 0.0f;
    uint16_t num_samples = 0;
    for (uint8_t i = 0; i < _cal_data[axis].num_samples; i++) {
        sum += sq(calc_sample_residual(_cal_data[axis].samples[i], scalar));
        num_samples++;
    }

    // return a huge residual if no samples
    if (num_samples == 0) {
        return AP_OPTICALFLOW_CAL_RMS_FAILED;
    }

    sum /= num_samples;
    return sum;
}

#if HAL_LOGGING_ENABLED
// log all samples
void AP_OpticalFlow_Calibrator::log_sample(uint8_t axis, uint8_t sample_num, float flow_rate, float body_rate, float los_pred)
{
    // @LoggerMessage: OFCA
    // @Description: Optical Flow Calibration sample
    // @Field: TimeUS: Time since system startup
    // @Field: Axis: Axis (X=0 Y=1)
    // @Field: Num: Sample number
    // @Field: FRate: Flow rate
    // @Field: BRate: Body rate
    // @Field: LPred: Los pred

    AP::logger().Write(
        "OFCA",
        "TimeUS,Axis,Num,FRate,BRate,LPred",
        "s#-EEE",
        "F00000",
        "QBBfff",
        AP_HAL::micros64(),
        (unsigned)axis,
        (unsigned)sample_num,
        (double)flow_rate,
        (double)body_rate,
        (double)los_pred);
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AP_OPTICALFLOW_CALIBRATOR_ENABLED
