#include "AP_InertialSensor.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

// Class level parameters
const AP_Param::GroupInfo AP_InertialSensor::BatchSampler::var_info[] = {
    // @Param: BAT_CNT
    // @DisplayName: sample count per batch
    // @Description: Number of samples to take when logging streams of IMU sensor readings.  Will be rounded down to a multiple of 32. This option takes effect on the next reboot.
    // @User: Advanced
    // @Increment: 32
    // @RebootRequired: True
    AP_GROUPINFO("BAT_CNT",  1, AP_InertialSensor::BatchSampler, _required_count,   1024),

    // @Param: BAT_MASK
    // @DisplayName: Sensor Bitmask
    // @Description: Bitmap of which IMUs to log batch data for. This option takes effect on the next reboot.
    // @User: Advanced
    // @Values: 0:None,1:First IMU,255:All
    // @Bitmask: 0:IMU1,1:IMU2,2:IMU3
    // @RebootRequired: True
    AP_GROUPINFO("BAT_MASK",  2, AP_InertialSensor::BatchSampler, _sensor_mask,   DEFAULT_IMU_LOG_BAT_MASK),

    // @Param: BAT_OPT
    // @DisplayName: Batch Logging Options Mask
    // @Description: Options for the BatchSampler
    // @Bitmask: 0:Sensor-Rate Logging (sample at full sensor rate seen by AP), 1: Sample post-filtering
    // @User: Advanced
    AP_GROUPINFO("BAT_OPT",  3, AP_InertialSensor::BatchSampler, _batch_options_mask, 0),

    // @Param: BAT_LGIN
    // @DisplayName: logging interval
    // @Description: Interval between pushing samples to the AP_Logger log
    // @Units: ms
    // @Increment: 10
    AP_GROUPINFO("BAT_LGIN", 4, AP_InertialSensor::BatchSampler, _push_interval_ms,   20),

    AP_GROUPEND
};

// This size is hardcoded in AP_Logger::Write_ISBD
#define BATCH_SAMPLES_PER_MESSAGE 32

extern const AP_HAL::HAL& hal;
void AP_InertialSensor::BatchSampler::init()
{
    if (_sensor_mask == 0) {
        return;
    }
    if (_required_count <= 0) {
        return;
    }

    // we assume the number of gyros and accels is the same, taking
    // this minimum stops us doing bad things if that isn't true:
    _instance_count = MIN(_imu._accel_count, _imu._gyro_count);
    const uint8_t sensor_count = _instance_count * 2;
    _required_count -= _required_count % BATCH_SAMPLES_PER_MESSAGE; // round down to nearest multiple of 32

    const uint32_t total_allocation = 3 * _required_count * sizeof(uint16_t) * sensor_count;
    gcs().send_text(MAV_SEVERITY_DEBUG, "INS: alloc %u bytes for ISB (free=%u)", (unsigned int)total_allocation, (unsigned int)hal.util->available_memory());

    for (uint8_t i = 0; i < sensor_count; i++) {
        _data_x[i] = (int16_t*)calloc(_required_count, sizeof(int16_t));
        _data_y[i] = (int16_t*)calloc(_required_count, sizeof(int16_t));
        _data_z[i] = (int16_t*)calloc(_required_count, sizeof(int16_t));
        _data_read_offset[i] = 0;
        _data_write_offset[i] = 0;
        if (_data_x[i] == nullptr || _data_y[i] == nullptr || _data_z[i] == nullptr) {
            for (uint8_t j = 0; j <= i; j++) {
                free(_data_x[j]);
                free(_data_y[j]);
                free(_data_z[j]);
                _data_x[j] = nullptr;
                _data_y[j] = nullptr;
                _data_z[j] = nullptr;
            }
            gcs().send_text(MAV_SEVERITY_WARNING, "Failed to allocate %u bytes for IMU batch sampling", (unsigned int)total_allocation);
            return;
        }
    }

    rotate_to_next_sensor();

    _initialised = true;
}

void AP_InertialSensor::BatchSampler::periodic()
{
    if (_sensor_mask == 0) {
        return;
    }
    push_data_to_log();
}

void AP_InertialSensor::BatchSampler::update_doing_sensor_rate_logging()
{
    // We can't do post-filter sensor rate logging
    if ((batch_opt_t)(_batch_options_mask.get()) & BATCH_OPT_POST_FILTER) {
        _doing_post_filter_logging = true;
        _doing_sensor_rate_logging = false;
        return;
    }
    _doing_post_filter_logging = false;
    if (!((batch_opt_t)(_batch_options_mask.get()) & BATCH_OPT_SENSOR_RATE)) {
        _doing_sensor_rate_logging = false;
        return;
    }
    const uint8_t bit = (1<<_instance);
    switch (_type) {
    case IMU_SENSOR_TYPE_GYRO:
        _doing_sensor_rate_logging = _imu._gyro_sensor_rate_sampling_enabled & bit;
        break;
    case IMU_SENSOR_TYPE_ACCEL:
        _doing_sensor_rate_logging = _imu._accel_sensor_rate_sampling_enabled & bit;
        break;
    }
}

void AP_InertialSensor::BatchSampler::rotate_to_next_sensor()
{
    if (_sensor_mask == 0) {
        // should not have been called
        return;
    }
    if ((1U<<_instance) > (uint8_t)_sensor_mask) {
        // should only ever happen if user resets _sensor_mask
        _instance = 0;
    }

    if (_type == IMU_SENSOR_TYPE_ACCEL) {
        // we have logged accelerometers, now log gyros:
        _type = IMU_SENSOR_TYPE_GYRO;
        _multiplier = _imu._gyro_raw_sampling_multiplier[_instance];
        update_doing_sensor_rate_logging();
        _sensor_index = _instance + _instance_count;
        return;
    }

    // log for accel sensor:
    _type = IMU_SENSOR_TYPE_ACCEL;

    // move to next IMU backend:
    bool haveinstance = false;
    for (uint8_t i = _instance + 1; i < _instance_count; i++) {
        if (_sensor_mask & (1U<<i)) {
            _instance = i;
            haveinstance = true;
            break;
        }
    }
    if (!haveinstance) {
        for (uint8_t i = 0; i <= _instance; i++) {
            if (_sensor_mask & (1U<<i)) {
                _instance = i;
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
        _instance = 0;
        return;
    }

    _sensor_index = _instance;
    _multiplier = _imu._accel_raw_sampling_multiplier[_instance];
    update_doing_sensor_rate_logging();
}

void AP_InertialSensor::BatchSampler::push_data_to_log()
{
    if (!_initialised) {
        return;
    }
    if (_sensor_mask == 0) {
        return;
    }
    // calculate data to write taking into account wrap around
    const uint16_t data_read_offset = _data_read_offset[_sensor_index];
    const uint16_t num_samples = (_data_write_offset[_sensor_index] - data_read_offset + _required_count) % _required_count;
    if (num_samples < BATCH_SAMPLES_PER_MESSAGE) {
        // insuffucient data to pack a packet
        return;
    }

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        // should not have been called
        return;
    }

    // possibly send isb header:
    if (!_isbh_sent && data_read_offset == 0) {
        if (AP_HAL::millis() - _last_sent_ms < (uint16_t)_push_interval_ms) {
            // avoid flooding DataFlash's buffer
            return;
        }
        else {
            // beginning of a new cycle
            _last_sent_ms = AP_HAL::millis();
        }
        float sample_rate = 0; // avoid warning about uninitialised values
        switch(_type) {
        case IMU_SENSOR_TYPE_GYRO:
            sample_rate = _imu._gyro_raw_sample_rates[_instance];
            if (_doing_sensor_rate_logging) {
                sample_rate *= _imu._gyro_over_sampling[_instance];
            }
            break;
        case IMU_SENSOR_TYPE_ACCEL:
            sample_rate = _imu._accel_raw_sample_rates[_instance];
            if (_doing_sensor_rate_logging) {
                sample_rate *= _imu._accel_over_sampling[_instance];
            }
            break;
        }
        if (!logger->Write_ISBH(_isb_seqnum,
                                       _type,
                                       _instance,
                                       _multiplier,
                                       _required_count,
                                       _measurement_started_us,
                                       sample_rate)) {
            // buffer full?
            return;
        }
        _isbh_sent = true;
    }
    // pack and send a data packet:
    if (!logger->Write_ISBD(_isb_seqnum,
                                   data_read_offset / BATCH_SAMPLES_PER_MESSAGE,
                                   &_data_x[_sensor_index][data_read_offset],
                                   &_data_y[_sensor_index][data_read_offset],
                                   &_data_z[_sensor_index][data_read_offset])) {
        // maybe later?!
        return;
    }

    _data_read_offset[_sensor_index] += BATCH_SAMPLES_PER_MESSAGE;

    if (_data_read_offset[_sensor_index] >= _required_count) {
        // that was the last one.  Clean up:
        _data_read_offset[_sensor_index] = 0;
        _isb_seqnum++;
        _isbh_sent = false;
        // rotate to next instance:
        rotate_to_next_sensor();
    }
}

bool AP_InertialSensor::BatchSampler::should_log(uint8_t sensor_index)
{
    if (_sensor_mask == 0) {
        return false;
    }

    if (!_initialised) {
        return false;
    }
    // if there nearly a full buffer of samples still to write then back off sampling
    if (((_data_write_offset[sensor_index] - _data_read_offset[sensor_index] + _required_count) % _required_count) >= _required_count - 1) {
        return false;
    }

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return false;
    }
#define MASK_LOG_ANY                    0xFFFF
    if (!logger->should_log(MASK_LOG_ANY)) {
        return false;
    }
    return true;
}

// sample a sensor, called from the gyro thread so concurrency issues must be observed
void AP_InertialSensor::BatchSampler::sample(uint8_t instance, AP_InertialSensor::IMU_SENSOR_TYPE type, uint64_t sample_us, const Vector3f &sensor_sample)
{
    uint8_t sensor_index = 0;
    uint16_t multiplier = 0;

    switch (type) {
    case IMU_SENSOR_TYPE_ACCEL:
        sensor_index = instance;
        multiplier = _imu._accel_raw_sampling_multiplier[instance];
        break;
    case IMU_SENSOR_TYPE_GYRO:
        sensor_index = instance + _instance_count;
        multiplier = _imu._gyro_raw_sampling_multiplier[instance];
        break;
    }

    if (!should_log(sensor_index)) {
        return;
    }

    if (_data_write_offset[sensor_index] == 0) {
        _measurement_started_us = sample_us;
    }

    const uint16_t data_write_offset = _data_write_offset[sensor_index];
    _data_x[sensor_index][data_write_offset] = multiplier * sensor_sample.x;
    _data_y[sensor_index][data_write_offset] = multiplier * sensor_sample.y;
    _data_z[sensor_index][data_write_offset] = multiplier * sensor_sample.z;

    // this assumes that we can write out sample packets as fast as we accumulate samples, we write 32 samples at a time
    // which means the gyro loop rate can be 32 / sensors * 2 faster than the output rate, so 5x for 3 sensors
    // periodic is run at 400Hz which gives a maximum backend sample rate of 2Khz
    _data_write_offset[sensor_index] = (_data_write_offset[sensor_index] + 1) % _required_count;
}
