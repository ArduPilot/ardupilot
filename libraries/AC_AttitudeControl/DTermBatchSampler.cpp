#include "AC_AttitudeControl.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// Class level parameters
const AP_Param::GroupInfo AC_AttitudeControl::DTermBatchSampler::var_info[] = {
    // @Param: BAT_CNT
    // @DisplayName: sample count per batch
    // @Description: Number of samples to take when logging streams of IMU sensor readings.  Will be rounded down to a multiple of 32.
    // @User: Advanced
    // @Increment: 32
    AP_GROUPINFO("BAT_CNT",  1, AC_AttitudeControl::DTermBatchSampler, _required_count,   1024),

    // @Param: BAT_MASK
    // @DisplayName: Control Term Bitmask
    // @Description: Bitmap of which control terms to log batch data for
    // @User: Advanced
    // @Values: 0:None,1:Raw D-Term input,2:Post-filter D-Term output,255:All
    // @Bitmask: 0:Raw-DTerm,1:Filter-DTerm
    AP_GROUPINFO("BAT_MASK",  2, AC_AttitudeControl::DTermBatchSampler, _control_mask,   DEFAULT_AC_ATTITUDE_LOG_BAT_MASK),

    // @Param: BAT_OPT
    // @DisplayName: Batch Logging Options Mask
    // @Description: Options for the BatchSampler
    // @User: Advanced
    AP_GROUPINFO("BAT_OPT",  3, AC_AttitudeControl::DTermBatchSampler, _batch_options_mask, 0),

    // @Param: BAT_LGIN
    // @DisplayName: logging interval
    // @Description: Interval between pushing samples to the DataFlash log
    // @Units: ms
    // @Increment: 10
    AP_GROUPINFO("BAT_LGIN", 4, AC_AttitudeControl::DTermBatchSampler, push_interval_ms,   20),

    // @Param: BAT_LGCT
    // @DisplayName: logging count
    // @Description: Number of samples to push to count every @PREFIX@BAT_LGIN
    // @Increment: 1
    AP_GROUPINFO("BAT_LGCT", 5, AC_AttitudeControl::DTermBatchSampler, samples_per_msg,   32),

    AP_GROUPEND
};


extern const AP_HAL::HAL& hal;
void AC_AttitudeControl::DTermBatchSampler::init(float dt)
{
    if (_control_mask == 0) {
        return;
    }
    if (_required_count <= 0) {
        return;
    }

    // MPU-6000 can measure +/- 2000 deg/s, so multiplier should be maximum change in dt
    // This works well in SITL, but saturates in actual flight
    // multiplier = INT16_MAX/(radians(2000)*dt);
    multiplier = INT16_MAX/(radians(2000)/dt);
    _required_count -= _required_count % 32; // round down to nearest multiple of 32

    const uint32_t total_allocation = 3*_required_count*sizeof(uint16_t);
    gcs().send_text(MAV_SEVERITY_DEBUG, "ATC: alloc %u bytes for ISB (free=%u)", total_allocation, hal.util->available_memory());

    data_x = (int16_t*)calloc(_required_count, sizeof(int16_t));
    data_y = (int16_t*)calloc(_required_count, sizeof(int16_t));
    data_z = (int16_t*)calloc(_required_count, sizeof(int16_t));
    if (data_x == nullptr || data_y == nullptr || data_z == nullptr) {
        free(data_x);
        free(data_y);
        free(data_z);
        data_x = nullptr;
        data_y = nullptr;
        data_z = nullptr;
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to allocate %u bytes for ATC batch sampling", total_allocation);
        return;
    }

    rotate_to_next_control_point();

    initialised = true;
}

void AC_AttitudeControl::DTermBatchSampler::periodic()
{
    if (_control_mask == 0) {
        return;
    }
    push_data_to_log();
}

void AC_AttitudeControl::DTermBatchSampler::rotate_to_next_control_point()
{
    if (_control_mask == 0) {
        // should not have been called
        return;
    }
    if ((1U<<control_point) > (uint8_t)_control_mask) {
        // should only ever happen if user resets _control_mask
        control_point = 0;
    }

    // find next control point to log:
    bool haveinstance = false;
    for (uint8_t i=control_point+1; i<AC_ATTITUDE_LOG_MAX_CONTROL_POINTS; i++) {
        if (_control_mask & (1U<<i)) {
            control_point = i;
            haveinstance = true;
            break;
        }
    }
    if (!haveinstance) {
        for (uint8_t i=0; i<=control_point; i++) {
            if (_control_mask & (1U<<i)) {
                control_point = i;
                haveinstance = true;
                break;
            }
        }
    }
    if (!haveinstance) {
        // should not happen!
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        abort();
#endif
        control_point = 0;
        return;
    }
}

void AC_AttitudeControl::DTermBatchSampler::push_data_to_log()
{
    if (!initialised) {
        return;
    }
    if (_control_mask == 0) {
        return;
    }
    if (data_write_offset - data_read_offset < samples_per_msg) {
        // insuffucient data to pack a packet
        return;
    }
    if (AP_HAL::millis() - last_sent_ms < (uint16_t)push_interval_ms) {
        // avoid flooding DataFlash's buffer
        return;
    }
    AP_Logger *dataflash = AP_Logger::get_singleton();
    if (dataflash == nullptr) {
        // should not have been called
        return;
    }

    // possibly send isb header:
    if (!isbh_sent && data_read_offset == 0) {
        float sample_rate = 1.0 / _controller._dt;
        if (!dataflash->Write_ISBH(isb_seqnum,
                                   AP_InertialSensor::DTERM_CONTROL_POINT,
                                   control_point,
                                   multiplier,
                                   _required_count,
                                   measurement_started_us,
                                   sample_rate)) {
            // buffer full?
            return;
        }
        isbh_sent = true;
    }
    // pack and send a data packet:
    if (!dataflash->Write_ISBD(isb_seqnum,
                               data_read_offset/samples_per_msg,
                               &data_x[data_read_offset],
                               &data_y[data_read_offset],
                               &data_z[data_read_offset])) {
        // maybe later?!
        return;
    }
    data_read_offset += samples_per_msg;
    last_sent_ms = AP_HAL::millis();
    if (data_read_offset >= _required_count) {
        // that was the last one.  Clean up:
        data_read_offset = 0;
        isb_seqnum++;
        isbh_sent = false;
        // rotate to next instance:
        rotate_to_next_control_point();
        data_write_offset = 0; // unlocks writing process
    }
}

bool AC_AttitudeControl::DTermBatchSampler::should_log(uint8_t _control_point)
{
    if (_control_mask == 0) {
        return false;
    }
    if (!initialised) {
        return false;
    }
    if (_control_point != control_point) {
        return false;
    }
    if (data_write_offset >= _required_count) {
        return false;
    }
    AP_Logger *dataflash = AP_Logger::get_singleton();
    if (dataflash == nullptr) {
        return false;
    }
#define MASK_LOG_ANY                    0xFFFF
    if (!dataflash->should_log(MASK_LOG_ANY)) {
        return false;
    }
    return true;
}

void AC_AttitudeControl::DTermBatchSampler::sample(AC_AttitudeControl::DTERM_CONTROL_POINT_TYPE _control_point, uint64_t sample_us, const Vector3f &_sample)
{
    if (!should_log(_control_point)) {
        return;
    }
    if (data_write_offset == 0) {
        measurement_started_us = sample_us;
    }

    data_x[data_write_offset] = multiplier*_sample.x;
    data_y[data_write_offset] = multiplier*_sample.y;
    data_z[data_write_offset] = multiplier*_sample.z;

    data_write_offset++; // may unblock the reading process
}
