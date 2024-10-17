#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

// Write ACC data packet: raw accel data
void AP_InertialSensor_Backend::Write_ACC(const uint8_t instance, const uint64_t sample_us, const Vector3f &accel) const
{
        const uint64_t now = AP_HAL::micros64();
        const struct log_ACC pkt {
            LOG_PACKET_HEADER_INIT(LOG_ACC_MSG),
            time_us   : now,
            instance  : instance,
            sample_us : sample_us?sample_us:now,
            AccX      : accel.x,
            AccY      : accel.y,
            AccZ      : accel.z
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write GYR data packet: raw gyro data
void AP_InertialSensor_Backend::Write_GYR(const uint8_t instance, const uint64_t sample_us, const Vector3f &gyro, bool use_sample_timestamp) const
{
        const uint64_t now = use_sample_timestamp?sample_us:AP_HAL::micros64();
        const struct log_GYR pkt{
            LOG_PACKET_HEADER_INIT(LOG_GYR_MSG),
            time_us   : now,
            instance  : instance,
            sample_us : sample_us?sample_us:now,
            GyrX      : gyro.x,
            GyrY      : gyro.y,
            GyrZ      : gyro.z
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write IMU data packet: raw accel/gyro data
void AP_InertialSensor::Write_IMU_instance(const uint64_t time_us, const uint8_t imu_instance) const
{
    const Vector3f &gyro = get_gyro(imu_instance);
    const Vector3f &accel = get_accel(imu_instance);
    const struct log_IMU pkt{
        LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
        time_us : time_us,
        instance: imu_instance,
        gyro_x  : gyro.x,
        gyro_y  : gyro.y,
        gyro_z  : gyro.z,
        accel_x : accel.x,
        accel_y : accel.y,
        accel_z : accel.z,
        gyro_error  : _gyro_error_count[imu_instance],
        accel_error : _accel_error_count[imu_instance],
        temperature : get_temperature(imu_instance),
        gyro_health : (uint8_t)get_gyro_health(imu_instance),
        accel_health : (uint8_t)get_accel_health(imu_instance),
        gyro_rate : get_gyro_rate_hz(imu_instance),
        accel_rate : get_accel_rate_hz(imu_instance),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write IMU data packet for all instances
void AP_InertialSensor::Write_IMU() const
{
    const uint64_t time_us = AP_HAL::micros64();

    uint8_t n = MAX(get_accel_count(), get_gyro_count());
    for (uint8_t i=0; i<n; i++) {
        Write_IMU_instance(time_us, i);
    }
}

// Write VIBE data packet for all instances
void AP_InertialSensor::Write_Vibration() const
{
    const uint64_t time_us = AP_HAL::micros64();
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        if (!use_accel(i)) {
            continue;
        }

        const Vector3f vibration = get_vibration_levels(i);
        const struct log_Vibe pkt{
            LOG_PACKET_HEADER_INIT(LOG_VIBE_MSG),
            time_us     : time_us,
            imu         : i,
            vibe_x      : vibration.x,
            vibe_y      : vibration.y,
            vibe_z      : vibration.z,
            clipping    : get_accel_clip_count(i)
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
// Write information about a series of IMU readings to log:
bool AP_InertialSensor::BatchSampler::Write_ISBH(const float sample_rate_hz) const
{
    uint8_t instance_to_write = instance;
    if (post_filter && (_doing_pre_post_filter_logging
            || (_doing_post_filter_logging && _doing_sensor_rate_logging))) {
        instance_to_write += (type == IMU_SENSOR_TYPE_ACCEL ? _imu._accel_count : _imu._gyro_count);
    }
    const struct log_ISBH pkt{
        LOG_PACKET_HEADER_INIT(LOG_ISBH_MSG),
        time_us        : AP_HAL::micros64(),
        seqno          : isb_seqnum,
        sensor_type    : (uint8_t)type,
        instance       : instance_to_write,
        multiplier     : multiplier,
        sample_count   : (uint16_t)_real_required_count,
        sample_us      : measurement_started_us,
        sample_rate_hz : sample_rate_hz,
    };

    return AP::logger().WriteBlock_first_succeed(&pkt, sizeof(pkt));
}

// Write a series of IMU readings to log:
bool AP_InertialSensor::BatchSampler::Write_ISBD() const
{
    struct log_ISBD pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ISBD_MSG),
        time_us    : AP_HAL::micros64(),
        isb_seqno  : isb_seqnum,
        seqno      : (uint16_t) (data_read_offset/samples_per_msg)
    };
    memcpy(pkt.x, &data_x[data_read_offset], sizeof(pkt.x));
    memcpy(pkt.y, &data_y[data_read_offset], sizeof(pkt.y));
    memcpy(pkt.z, &data_z[data_read_offset], sizeof(pkt.z));

    return AP::logger().WriteBlock_first_succeed(&pkt, sizeof(pkt));
}
#endif

#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
// @LoggerMessage: FTN
// @Description: Filter Tuning Message - per motor
// @Field: TimeUS: microseconds since system startup
// @Field: I: instance
// @Field: NDn: number of active harmonic notches
// @Field: NF1: desired harmonic notch centre frequency for motor 1
// @Field: NF2: desired harmonic notch centre frequency for motor 2
// @Field: NF3: desired harmonic notch centre frequency for motor 3
// @Field: NF4: desired harmonic notch centre frequency for motor 4
// @Field: NF5: desired harmonic notch centre frequency for motor 5
// @Field: NF6: desired harmonic notch centre frequency for motor 6
// @Field: NF7: desired harmonic notch centre frequency for motor 7
// @Field: NF8: desired harmonic notch centre frequency for motor 8
// @Field: NF9: desired harmonic notch centre frequency for motor 9
// @Field: NF10: desired harmonic notch centre frequency for motor 10
// @Field: NF11: desired harmonic notch centre frequency for motor 11
// @Field: NF12: desired harmonic notch centre frequency for motor 12

// @LoggerMessage: FTNS
// @Description: Filter Tuning Message
// @Field: TimeUS: microseconds since system startup
// @Field: I: instance
// @Field: NF: desired harmonic notch centre frequency

void AP_InertialSensor::write_notch_log_messages() const
{
    const uint64_t now_us = AP_HAL::micros64();
    for (auto &notch : harmonic_notches) {
        const uint8_t i = &notch - &harmonic_notches[0];
        if (!notch.params.enabled()) {
            continue;
        }
        const float* notches = notch.calculated_notch_freq_hz;
        if (notch.num_calculated_notch_frequencies > 1) {
            // log per motor center frequencies
            AP::logger().WriteStreaming(
                "FTN", "TimeUS,I,NDn,NF1,NF2,NF3,NF4,NF5,NF6,NF7,NF8,NF9,NF10,NF11,NF12", "s#-zzzzzzzzzzzz", "F--------------", "QBBffffffffffff",
                now_us,
                i,
                notch.num_calculated_notch_frequencies,
                notches[0], notches[1], notches[2], notches[3],
                notches[4], notches[5], notches[6], notches[7],
                notches[8], notches[9], notches[10], notches[11]);
        } else {
            // log single center frequency
            AP::logger().WriteStreaming(
                "FTNS", "TimeUS,I,NF", "s#z", "F--", "QBf",
                now_us,
                i,
                notches[0]);
        }

        // ask the HarmonicNotchFilter object for primary gyro to
        // log the actual notch centers
        notch.filter[_primary_gyro].log_notch_centers(i, now_us);
    }
}
#endif  // AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED

#endif  // HAL_LOGGING_ENABLED
