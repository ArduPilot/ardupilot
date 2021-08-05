#include <cmath>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_IOMCU/AP_IOMCU.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

static AP_BoardConfig BoardConfig;

#include "include/mavlink/v2.0/ardupilotmega/version.h"
#include "include/mavlink/v2.0/mavlink_types.h"
#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"

#include "Config.h"
#include "SimpleGCS.h"
#include "Sub.h"

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

void setup();
void loop();
void handle_mavlink_message(mavlink_message_t latest_message);
void send_heartbeat_if_needed();
void report_performance_stats_if_needed();
void send_sensor_messages_if_needed();
void add_performance_sample(uint32_t *sample_array, uint8_t &sample_number, uint32_t &last_sample_time);
float get_sample_average(uint32_t *samples_ptr, uint8_t number_samples);
void update_complementary_filter(Vector3f gyro_reading, Vector3f accel_reading, uint32_t last_update_millis, float accel_weight, float gyro_weight, float &fused_roll, float &fused_pitch);

uint32_t main_loop_rate_samples[PERFORMANCE_HISTORY_LENGTH];
uint32_t motor_control_packet_rate_samples[PERFORMANCE_HISTORY_LENGTH];

uint8_t main_loop_number(0);
uint8_t motor_packet_number(0);

uint32_t last_main_loop_time(0);
uint32_t last_motor_packet_time(0);
uint32_t last_performance_report(0);

// used by the complementary filter. Useful for subtracting out gravity
float roll_filtered(0.0);
float pitch_filtered(0.0);

struct TimeInfo
{
    TimeInfo() : last_heartbeat_message_time(0), last_sensor_message_time(0) {}

    uint32_t last_heartbeat_message_time;
    uint32_t last_sensor_message_time;
};

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
static AP_InertialSensor inertial_sensor;

uint8_t accel_count;
uint8_t gyro_count;
uint8_t number_inertial_sensors;

SimpleSub *sub_ptr;
SimpleGCS *gcs_ptr;
TimeInfo *time_info_ptr;

uint32_t last_imu_message_send_time(0);

void handle_mavlink_message(mavlink_message_t latest_message)
{
    switch (latest_message.msgid)
    {
    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        mavlink_command_long_t command_long_packet;
        mavlink_msg_command_long_decode(&latest_message, &command_long_packet);
        bool success = sub_ptr->handle_command_long_packet(command_long_packet);

        MAV_RESULT command_result;
        if (success)
        {
            command_result = MAV_RESULT_ACCEPTED;
        }
        else
        {
            command_result = MAV_RESULT_UNSUPPORTED;
        }

        gcs_ptr->send_command_ack_packet(command_long_packet.command, command_result);

#ifdef SIMPLE_SUB_DEBUG
        gcs_ptr->send_text(MAV_SEVERITY_INFO, "Got command long packet with command %u", command_long_packet.command);
#endif
        break;
    }
    // case MAVLINK_MSG_ID_DIRECT_MOTOR_CONTROL: {

    //     mavlink_direct_motor_control_t direct_motor_control_packet;
    //     mavlink_msg_direct_motor_control_decode(&latest_message, &direct_motor_control_packet);
    //     sub_ptr->handle_direct_motor_control_packet(direct_motor_control_packet);

    //     #ifdef SIMPLE_SUB_DEBUG
    //         gcs_ptr->send_text(MAV_SEVERITY_INFO, "Got direct motor control packet");
    //     #endif

    //     break;
    // }
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // reformats as a direct motor control packet runs
        mavlink_rc_channels_override_t rc_override_message;
        mavlink_msg_rc_channels_override_decode(&latest_message, &rc_override_message);

        sub_ptr->handle_rc_override_packet(rc_override_message);
        break;
    }
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
#ifdef SIMPLE_SUB_DEBUG
        //gcs_ptr->send_text(MAV_SEVERITY_INFO, "Request data stream ignored.");
#endif
        break;
    }
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
#ifdef SIMPLE_SUB_DEBUG
        //gcs_ptr->send_text(MAV_SEVERITY_INFO, "Heartbeat ignored.");
#endif
        break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
#ifdef SIMPLE_SUB_DEBUG
        //gcs_ptr->send_text(MAV_SEVERITY_INFO, "Param request list ignored.");
#endif
        break;
    }
    default:
    {
#ifdef SIMPLE_SUB_DEBUG
        gcs_ptr->send_text(MAV_SEVERITY_INFO, "Unrecognized message id! %u", latest_message.msgid);
#endif
        break;
    }
    }
}

void setup(void)
{
    BoardConfig.init();

    hal.uartA->begin(SERIAL_BAUD_RATE);

    gcs_ptr = new SimpleGCS(hal.uartA);
    sub_ptr = new SimpleSub(gcs_ptr, hal.rcout);
    time_info_ptr = new TimeInfo();

    sub_ptr->enable_motor_rc_channels();
    sub_ptr->disarm();

    inertial_sensor.init(AP_INERTIAL_SENSOR_IMU_SAMPLE_RATE_MILLIS);
    inertial_sensor.update();

    accel_count = inertial_sensor.get_accel_count();
    gyro_count = inertial_sensor.get_gyro_count();
    number_inertial_sensors = MAX(accel_count, gyro_count);
}

void send_heartbeat_if_needed()
{
    uint32_t current_time = AP_HAL::millis();
    uint32_t last_update = time_info_ptr->last_heartbeat_message_time;

    if (current_time < last_update)
    {
        gcs_ptr->send_heartbeat(sub_ptr->get_is_armed());
        return;
    }

    uint32_t time_gap = current_time - last_update;
    if (HEART_BEAT_RATE_MILLIS < time_gap)
    {
#ifdef SIMPLE_SUB_DEBUG
        //gcs_ptr->send_text(MAV_SEVERITY_INFO, "Sending heart beat message");
#endif
        gcs_ptr->send_heartbeat(sub_ptr->get_is_armed());
        time_info_ptr->last_heartbeat_message_time = current_time;
        return;
    }
}

void update_complementary_filter(Vector3f gyro_reading, Vector3f accel_reading, uint32_t last_update_millis, float accel_weight, float gyro_weight, float &fused_roll, float &fused_pitch)
{
    float acceleration_norm = accel_reading.length();
    float time_delta_seconds = (static_cast<float>(last_update_millis) - static_cast<float>(AP_HAL::millis())) / 1000.0f;

    float ac_w = 0.0025;
    float accum_w = 1.0 - ac_w;

    fused_roll -= static_cast<float>(gyro_reading[0]) * time_delta_seconds;
    fused_pitch += static_cast<float>(gyro_reading[1]) * time_delta_seconds;

    // ensure we don't pack in a giant acceleration vector
    if (acceleration_norm < 12.0 && acceleration_norm > 6.5)
    {
        float pitch_accel = atan2f(accel_reading.x, accel_reading.z);
        float roll_accel = atan2f(accel_reading.y, accel_reading.z);

        fused_roll = atan2f(
            sinf(fused_roll) * accum_w + sinf(roll_accel) * ac_w,
            cosf(fused_roll) * accum_w + cosf(roll_accel) * ac_w);

        fused_pitch = atan2f(
            sinf(fused_pitch) * accum_w + sinf(pitch_accel) * ac_w,
            cosf(fused_pitch) * accum_w + cosf(pitch_accel) * ac_w);
    }
}

void send_sensor_messages_if_needed()
{
    // accel units from sensor are m/s/s
    // gyro units are rads/sec

    // scaled units are:
    //  gyro: mrads/sec
    // accel: mGs (weird unit)

    uint32_t current_time = AP_HAL::millis();

    if (SEND_SENSOR_MESSAGES)
    {
        if (current_time - last_imu_message_send_time > SENSOR_MESSAGE_RATE_MILLIS || current_time < last_imu_message_send_time)
        {

            Vector3f acceleration_vector;
            Vector3f gyro_vector;

            acceleration_vector = inertial_sensor.get_accel(REPORT_IMU_INDEX);
            gyro_vector = inertial_sensor.get_gyro(REPORT_IMU_INDEX);

            update_complementary_filter(
                gyro_vector,
                acceleration_vector,
                last_imu_message_send_time,
                0.02,
                0.98,
                roll_filtered,
                pitch_filtered);

            float yaw = 0.0;
            float temperature = 0.0;

            gcs_ptr->send_imu_data(
                acceleration_vector.x * 1000.0 * 0.101972,
                acceleration_vector.y * 1000.0 * 0.101972,
                acceleration_vector.z * 1000.0 * 0.101972,
                gyro_vector.x * 1000.0,
                gyro_vector.y * 1000.0,
                gyro_vector.z * 1000.0,
                temperature);

            gcs_ptr->send_attitude(
                roll_filtered,
                pitch_filtered,
                yaw,
                gyro_vector.x,
                gyro_vector.y,
                gyro_vector.z);

            last_imu_message_send_time = current_time;
        }
    }
}

float get_sample_average(uint32_t *samples_ptr, uint8_t number_samples)
{
    float total_samples_seconds = 0.0;
    for (uint8_t i = 0; i < number_samples; ++i)
    {
        total_samples_seconds += (float)(samples_ptr[i]) / 1000.0;
    }

    return total_samples_seconds / (float)number_samples;
}

void report_performance_stats_if_needed()
{
    // rate of main loop
    // rate of motor packets being received
    // hoo boy another few rates to deal with....

    if (AP_HAL::millis() - last_performance_report > PERFORMANCE_STATS_REPORT_RATE)
    {
        float main_rate = get_sample_average(main_loop_rate_samples, PERFORMANCE_HISTORY_LENGTH);
        float motor_rate = get_sample_average(motor_control_packet_rate_samples, PERFORMANCE_HISTORY_LENGTH);

        gcs_ptr->send_text(MAV_SEVERITY_INFO, "Main rate: %f Motor rate: %f", main_rate, motor_rate);

        //for (int i = 0; i < 14; ++i) {
        //    gcs_ptr->send_text(MAV_SEVERITY_INFO, "PWM chan %u freq %u", i, sub_ptr->rcout->get_freq(i));
        //}
        last_performance_report = AP_HAL::millis();
    }
}

void add_performance_sample(uint32_t *sample_array, uint8_t &sample_number, uint32_t &last_sample_time)
{
    uint32_t current_time = AP_HAL::millis();
    sample_array[sample_number++] = current_time - last_sample_time;
    sample_number %= PERFORMANCE_HISTORY_LENGTH;

    last_sample_time = current_time;
}

void loop(void)
{
    if (gcs_ptr->update_receive())
    {
        mavlink_message_t latest_message = gcs_ptr->get_last_parsed_message();
        handle_mavlink_message(latest_message);

        gcs_ptr->clear_mavlink_buffer();
    }

    inertial_sensor.wait_for_sample();
    inertial_sensor.update();

    // highest priority is sending the motors
    sub_ptr->stop_if_delay_between_messages_too_long();
    sub_ptr->output_to_motors();

    send_heartbeat_if_needed();

    if (REPORT_PERFORMANCE_STATS)
    {
        report_performance_stats_if_needed();
    }

    send_sensor_messages_if_needed();

    add_performance_sample(main_loop_rate_samples, main_loop_number, last_main_loop_time);
}

AP_HAL_MAIN();
