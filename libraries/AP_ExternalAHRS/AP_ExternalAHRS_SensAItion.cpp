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
/*
  Support for SensAItion serial connected AHRS and IMU
  Implements SensAItion protocol with ArduPilot-specific adaptations
*/

#include "AP_ExternalAHRS_SensAItion.h"

#if AP_EXTERNAL_AHRS_SENSAITION_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_SensAItion::AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    parser(option_is_set(AP_ExternalAHRS::OPTIONS::SENSAITION_AHRS) ?
           AP_ExternalAHRS_SensAItion_Parser::ConfigMode::CONFIG_MODE_AHRS :
           AP_ExternalAHRS_SensAItion_Parser::ConfigMode::CONFIG_MODE_IMU)
{

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart || baudrate == 0 || port_num == -1) {
        AP_HAL::panic("SensAItion: No UART configured for AHRS protocol");
    }

    // Create thread for non-blocking UART processing
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SensAItion::update_thread, void),
            "AHRS_SensAItion", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SensAItion thread creation failed");
        AP_HAL::panic("SensAItion Failed to start ExternalAHRS update thread");
    }
}

// Interface methods
int8_t AP_ExternalAHRS_SensAItion::get_port() const
{
    if (!uart) {
        return -1;
    }
    return port_num;
}

const char* AP_ExternalAHRS_SensAItion::get_name() const
{
    return "Kebni SensAItion";

}

bool AP_ExternalAHRS_SensAItion::healthy() const
{
    uint32_t now_ms = AP_HAL::millis();
    return (now_ms - last_valid_packet_ms) < 1000 && valid_packets > 0;
}

bool AP_ExternalAHRS_SensAItion::initialised() const
{
    return setup_complete;
}

bool AP_ExternalAHRS_SensAItion::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Kebni SensAItion unhealthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_SensAItion::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    memset(&status, 0, sizeof(status));


    if (healthy()) {
        status.flags.initalized = true;
        state.have_origin = true; // We don't provide origin, but we can't get around the pre-arm check without this. Update pre-arm condition?
        if (option_is_set(AP_ExternalAHRS::OPTIONS::SENSAITION_AHRS)) {
            status.flags.attitude = true;
        }
    }
}

void AP_ExternalAHRS_SensAItion::update_thread()
{
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(100);
        }
    }
}

bool AP_ExternalAHRS_SensAItion::check_uart()
{
    if (!uart) {
        return false;
    }

    if (!setup_complete) {
        uart->begin(baudrate);
        setup_complete = true;
    }

    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    n = MIN(n, sizeof(buffer));
    ssize_t nread = uart->read(buffer, n);

    if (nread > 0) {
        parser.parse_bytes(buffer, nread, sensor_measurement);
        if (sensor_measurement.type ==
            AP_ExternalAHRS_SensAItion_Parser::MeasurementType::UNINITIALIZED) {
            return false;
        }

        valid_packets++;
        auto const current_time_ms = AP_HAL::millis();
        last_valid_packet_ms = current_time_ms;

        AP_ExternalAHRS::ins_data_message_t ins;
        ins.accel = sensor_measurement.acceleration_mss;
        ins.gyro = sensor_measurement.angular_velocity_rads;
        ins.temperature = sensor_measurement.temperature_degc;
        AP::ins().handle_external(ins);

#if AP_COMPASS_EXTERNALAHRS_ENABLED
        AP_ExternalAHRS::mag_data_message_t mag;
        mag.field = sensor_measurement.magnetic_field_mgauss;
        AP::compass().handle_external(mag);
#endif

#if AP_BARO_EXTERNALAHRS_ENABLED
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = sensor_measurement.air_pressure_p;
        baro.temperature = sensor_measurement.temperature_degc;
        AP::baro().handle_external(baro);
#endif

        // Log Euler angles from the AHRS sensor at slow rate, just to allow comparison
        // with those from EKF3.
        const uint32_t EULER_ANGLE_LOG_INTERVAL_MS = 100;
        if (sensor_measurement.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::AHRS
            && current_time_ms % EULER_ANGLE_LOG_INTERVAL_MS == 0) {
            float roll_rad;
            float pitch_rad;
            float yaw_rad;

            // Translate the quaternion to Euler angles, to be applied in
            // "321" order, i.e., yaw, then pitch, then roll.
            sensor_measurement.orientation.to_euler(roll_rad, pitch_rad, yaw_rad);

            // @LoggerMessage: KEBN
            // @Description: Attitude estimate from SensAItion AHRS. Apply in yaw, pitch, roll order.
            // @Field: TimeUS: Current system time (us)
            // @Field: Roll: Roll (deg)
            // @Field: Pitch: Pitch (deg)
            // @Field: Yaw: Yaw/heading (deg)
            AP::logger().Write("KEBN", "TimeUS,Roll,Pitch,Yaw", "Qfff",
                               AP_HAL::micros64(),
                               degrees(roll_rad),
                               degrees(pitch_rad),
                               degrees(yaw_rad));
        }

        // Modify state last to minimize scope of the semaphore
        WITH_SEMAPHORE(state.sem);
        state.accel = sensor_measurement.acceleration_mss;
        state.gyro = sensor_measurement.angular_velocity_rads;
        if (sensor_measurement.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::AHRS) {
            state.quat = sensor_measurement.orientation;
            state.have_quaternion = true;
        } else {
            state.have_quaternion = false;
        }
        return true;
    }

    return false;
}

#endif  // AP_EXTERNAL_AHRS_SENSAITION_ENABLED