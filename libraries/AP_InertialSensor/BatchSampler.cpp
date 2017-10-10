#include "AP_InertialSensor.h"
#include <GCS_MAVLink/GCS.h>

// Class level parameters
const AP_Param::GroupInfo AP_InertialSensor::BatchSampler::var_info[] = {
    // @Param: BAT_CNT
    // @DisplayName: sample count per batch
    // @Description: Number of samples to take when logging streams of IMU sensor readings.  Will be rounded down to a multiple of 32.
    // @User: Advanced
    // @Increment: 32
    AP_GROUPINFO("BAT_CNT",  1, AP_InertialSensor::BatchSampler, _required_count,   1024),

    // @Param: BAT_MASK
    // @DisplayName: Sensor Bitmask
    // @Description: Bitmap of which IMUs to log batch data for
    // @User: Advanced
    // @Values: 0:None,1:First IMU,255:All
    // @Bitmask: 0:IMU1,1:IMU2,2:IMU3
    AP_GROUPINFO("BAT_MASK",  2, AP_InertialSensor::BatchSampler, _sensor_mask,   DEFAULT_IMU_LOG_BAT_MASK),

    AP_GROUPEND
};


extern const AP_HAL::HAL& hal;
void AP_InertialSensor::BatchSampler::init()
{
    if (_sensor_mask == 0) {
        return;
    }
    if (_required_count <= 0) {
        return;
    }

    _required_count -= _required_count % 32; // round down to nearest multiple of 32

    const uint32_t total_allocation = 3*_required_count*sizeof(uint16_t);
    gcs().send_text(MAV_SEVERITY_DEBUG, "INS: alloc %u bytes for ISB (free=%u)", total_allocation, hal.util->available_memory());

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
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to allocate %u bytes for IMU batch sampling", total_allocation);
        return;
    }

    rotate_to_next_sensor();

    initialised = true;
}

void AP_InertialSensor::BatchSampler::periodic()
{
    if (_sensor_mask == 0) {
        return;
    }
    push_data_to_log();
}

void AP_InertialSensor::BatchSampler::rotate_to_next_sensor()
{
    if (_sensor_mask == 0) {
        // should not have been called
        return;
    }
    if ((1U<<instance) > (uint8_t)_sensor_mask) {
        // should only ever happen if user resets _sensor_mask
        instance = 0;
    }

    if (type == IMU_SENSOR_TYPE_ACCEL) {
        // we have logged accelerometers, now log gyros:
        type = IMU_SENSOR_TYPE_GYRO;
        multiplier = multiplier_gyro;
        return;
    }

    // log for accel sensor:
    type = IMU_SENSOR_TYPE_ACCEL;
    multiplier = multiplier_accel;

    // move to next IMU backend:

    // we assume the number of gyros and accels is the same, taking
    // this minimum stops us doing bad things if that isn't true:
    const uint8_t _count = MIN(_imu._accel_count, _imu._gyro_count);

    // find next backend instance to log:
    for (uint8_t i=instance+1; i<_count; i++) {
        if (_sensor_mask & (1U<<i)) {
            instance = i;
            return;
        }
    }
    for (uint8_t i=0; i<instance; i++) {
        if (_sensor_mask & (1U<<i)) {
            instance = i;
            return;
        }
    }
}

void AP_InertialSensor::BatchSampler::push_data_to_log()
{
    if (!initialised) {
        return;
    }
    if (_sensor_mask == 0) {
        return;
    }
    if (data_write_offset - data_read_offset < samples_per_msg) {
        // insuffucient data to pack a packet
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < push_interval_ms) {
        // avoid flooding DataFlash's buffer
        return;
    }
    DataFlash_Class *dataflash = DataFlash_Class::instance();
    if (dataflash == nullptr) {
        // should not have been called
        return;
    }

    // possibly send isb header:
    if (!isbh_sent && data_read_offset == 0) {
        float sample_rate = 0; // avoid warning about uninitialised values
        switch(type) {
        case IMU_SENSOR_TYPE_GYRO:
            sample_rate = _imu._gyro_raw_sample_rates[instance];
            break;
        case IMU_SENSOR_TYPE_ACCEL:
            sample_rate = _imu._accel_raw_sample_rates[instance];
            break;
        }
        if (!dataflash->Log_Write_ISBH(isb_seqnum,
                                       type,
                                       instance,
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
    if (!dataflash->Log_Write_ISBD(isb_seqnum,
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
        rotate_to_next_sensor();
        data_write_offset = 0; // unlocks writing process
    }
}

bool AP_InertialSensor::BatchSampler::should_log(uint8_t _instance, IMU_SENSOR_TYPE _type)
{
    if (_sensor_mask == 0) {
        return false;
    }
    if (!initialised) {
        return false;
    }
    if (_instance != instance) {
        return false;
    }
    if (_type != type) {
        return false;
    }
    if (data_write_offset >= _required_count) {
        return false;
    }
    DataFlash_Class *dataflash = DataFlash_Class::instance();
#define MASK_LOG_ANY                    0xFFFF
    if (!dataflash->should_log(MASK_LOG_ANY)) {
        return false;
    }
    return true;
}

void AP_InertialSensor::BatchSampler::sample(uint8_t _instance, AP_InertialSensor::IMU_SENSOR_TYPE _type, uint64_t sample_us, const Vector3f &_sample)
{
    if (!should_log(_instance, _type)) {
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
