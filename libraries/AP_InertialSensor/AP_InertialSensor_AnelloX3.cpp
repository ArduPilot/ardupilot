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
    Serial InertialSensor backend for the Anello X3 FOG IMU.
 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_InertialSensor_AnelloX3.h"

#if AP_INERTIALSENSOR_ANELLOX3_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

AP_InertialSensor_AnelloX3::AP_InertialSensor_AnelloX3(AP_InertialSensor &imu,
                                                       AP_HAL::UARTDriver *_uart,
                                                       uint8_t _serial_port) :
    AP_InertialSensor_Backend(imu),
    uart(_uart),
    serial_port(_serial_port)
{
}

// probe a SERIALx_PROTOCOL = AnelloX3 port
AP_InertialSensor_Backend *AP_InertialSensor_AnelloX3::probe(AP_InertialSensor &imu)
{
    auto &sm = AP::serialmanager();
    AP_HAL::UARTDriver *uart = sm.find_serial(AP_SerialManager::SerialProtocol_AnelloX3, 0);
    if (uart == nullptr) {
        return nullptr;
    }
    const int8_t port = sm.find_portnum(AP_SerialManager::SerialProtocol_AnelloX3, 0);
    return NEW_NOTHROW AP_InertialSensor_AnelloX3(imu, uart, uint8_t(port));
}

void AP_InertialSensor_AnelloX3::start()
{
    if (uart == nullptr) {
        return;
    }

    // register at the X3's nominal output rate; the dt the EKF integrates on
    // comes from the sensor's mcu_time, not from this rate.
    if (!_imu.register_gyro(gyro_instance, RATE_HZ,
            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, serial_port, 1, DEVTYPE_INS_ANELLOX3)) ||
        !_imu.register_accel(accel_instance, RATE_HZ,
            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, serial_port, 2, DEVTYPE_INS_ANELLOX3))) {
        return;
    }
    started = true;

    // a dedicated thread owns the UART read and parser
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_AnelloX3::update_thread, void),
                                      "AnelloX3", 2048, AP_HAL::Scheduler::PRIORITY_UART, 0)) {
        AP_BoardConfig::allocation_error("AnelloX3 thread");
    }
}

bool AP_InertialSensor_AnelloX3::update()
{
    if (!started) {
        return false;
    }
    update_gyro(gyro_instance);
    update_accel(accel_instance);
    return true;
}

bool AP_InertialSensor_AnelloX3::get_output_banner(char* banner, uint8_t banner_len)
{
    hal.util->snprintf(banner, banner_len, "IMU%u: AnelloX3 FOG %uHz", gyro_instance, RATE_HZ);
    return true;
}

void AP_InertialSensor_AnelloX3::update_thread()
{
    if (uart == nullptr) {
        return;
    }

    // we need to call begin() from here so that the thread context owns the uart
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_AnelloX3, 0), 8192, 128);

    while (true) {
        build_packet();
        hal.scheduler->delay_microseconds(500);
    }
}

// read available bytes from the UART, parsing each into packets
void AP_InertialSensor_AnelloX3::build_packet()
{
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes-- > 0) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        if (handle_byte(b)) {
            // Validate the length matches the wire struct before the memcpy so a
            // short/variant packet can't be misinterpreted.
            if (message_in.packet.descriptor == uint8_t(DescriptorSet::IMUData) &&
                message_in.packet.length == IMU_PAYLOAD_LEN) {
                AnelloX3_BinaryPayload bin_payload;
                // copy exactly the wire length (55), not sizeof (56, padded).
                // void* cast: the struct is trivially-copyable but the compiler
                // sees Vector3<>'s ctor and flags memcpy (-Wclass-memaccess).
                memcpy((void *)&bin_payload, message_in.packet.payload, IMU_PAYLOAD_LEN);

                convert_imu_data(bin_payload);
                publish_imu();

                if (!has_announced_detected && (AP_HAL::millis() > 5000)) {
                    has_announced_detected = true;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IMU%u: AnelloX3 HW detected", gyro_instance);
                }
            }
        }
    }
}

// parsing state machine for incoming bytes
bool AP_InertialSensor_AnelloX3::handle_byte(const uint8_t b)
{
    switch (message_in.state) {
    case ParseState::WaitingFor_SyncOne:
        if (b == AnelloX3_Packet::SYNC_ONE) {
            message_in.state = ParseState::WaitingFor_SyncTwo;
        }
        break;
    case ParseState::WaitingFor_SyncTwo:
        if (b == AnelloX3_Packet::SYNC_TWO) {
            message_in.state = ParseState::WaitingFor_Descriptor;
        } else {
            message_in.state = ParseState::WaitingFor_SyncOne;
        }
        break;
    case ParseState::WaitingFor_Descriptor:
        message_in.packet.descriptor = b;
        message_in.state = ParseState::WaitingFor_PayloadLength;
        break;
    case ParseState::WaitingFor_PayloadLength:
        message_in.packet.length = b;
        if (message_in.packet.length == 0) {
            message_in.state = ParseState::WaitingFor_Checksum;
        } else {
            message_in.state = ParseState::WaitingFor_Data;
        }
        message_in.index = 0;
        break;
    case ParseState::WaitingFor_Data:
        message_in.packet.payload[message_in.index++] = b;
        if (message_in.index >= message_in.packet.length) {
            message_in.state = ParseState::WaitingFor_Checksum;
            message_in.index = 0;
        }
        break;
    case ParseState::WaitingFor_Checksum:
        message_in.packet.checksum[message_in.index++] = b;
        if (message_in.index >= 2) {
            message_in.state = ParseState::WaitingFor_SyncOne;
            if (valid_packet(message_in.packet)) {
                return true;
            }
            // checksum mismatch: corrupt or dropped bytes on the wire
            crc_error_count++;
        }
        break;
    }
    return false;
}

// checksum verification (Fletcher-8 over descriptor, length and payload)
bool AP_InertialSensor_AnelloX3::valid_packet(const AnelloX3_Packet &packet)
{
    uint8_t checksum_one = packet.descriptor;
    uint8_t checksum_two = checksum_one;
    checksum_one += packet.length;
    checksum_two += checksum_one;

    for (int i = 0; i < packet.length; i++) {
        checksum_one += packet.payload[i];
        checksum_two += checksum_one;
    }

    return packet.checksum[0] == checksum_one && packet.checksum[1] == checksum_two;
}

// convert the binary data to engineering values; timing + data-loss detection
void AP_InertialSensor_AnelloX3::convert_imu_data(const AnelloX3_BinaryPayload &bin_payload)
{
    // mems_ranges bit layout (MSB..LSB): aaaa aggg gggg gggg
    //   top 5 bits  = accel range, low 11 bits = gyro range
    // (verify against the Anello datasheet: a wrong split silently corrupts both
    //  MEMS scale factors. Log gyro RMS matched the internal IMUs to ~1%, which
    //  is consistent with this split being correct.)
    imu_data.mems_acc_range = bin_payload.mems_ranges >> 11;     // top 5 bits
    imu_data.mems_gyro_range = bin_payload.mems_ranges & 0x07FF; // low 11 bits
    imu_data.fog_gyro_range = bin_payload.fog_range;

    // mems accel (g -> m/s^2)
    imu_data.mems_accel = bin_payload.a.tofloat() * imu_data.mems_acc_range * 3.05e-5 * 9.81f;

    // mems gyro (dps -> rad/s)
    imu_data.mems_gyro = bin_payload.w.tofloat() * imu_data.mems_gyro_range * 3.5e-5 * DEG_TO_RAD;

    // fog gyro (dps -> rad/s)
    imu_data.fog_gyro = bin_payload.og_w.tofloat() * 2.32830644e-7 * DEG_TO_RAD;

    // mag (gauss -> milligauss); parsed/logged only, not fused as a compass here
    imu_data.mag = bin_payload.mag.tofloat() * 0.2441;

    // temperature (degC)
    imu_data.temp = bin_payload.temp * 1e-2;

    imu_data.fusion_status = bin_payload.fusion_status;

    // --- timing + data-loss detection from the sensor's own monotonic clock ---
    imu_data.mcu_time_us = bin_payload.mcu_time * 0.001; // ns -> us
    if (last_mcu_time_us != 0) {
        const uint32_t nominal_us = 1000000UL / RATE_HZ;
        const int64_t gap_us = int64_t(imu_data.mcu_time_us - last_mcu_time_us);
        // a healthy stream steps by ~nominal_us; a larger jump means the gap
        // spans one or more samples that never reached us.
        if (gap_us > int64_t(nominal_us + nominal_us/2)) {
            dropped_samples += uint32_t((gap_us + nominal_us/2) / nominal_us) - 1;
            const uint32_t now_ms = AP_HAL::millis();
            if (now_ms - last_drop_report_ms > 5000) {
                last_drop_report_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "IMU%u: AnelloX3: %u samples dropped", gyro_instance, (unsigned)dropped_samples);
            }
        }
    }
    last_mcu_time_us = imu_data.mcu_time_us;

    // Map the sensor's monotonic mcu_time onto a jitter-free local timestamp; this
    // is the value the EKF integrates dt on (vs UART arrival-time jitter).
    imu_data.sample_us = jitter_correction.correct_offboard_timestamp_usec(imu_data.mcu_time_us, AP_HAL::micros64());

#if HAL_LOGGING_ENABLED
    const auto now = AP_HAL::micros64();
    // @LoggerMessage: AX31
    // @Description: Anello Photonics X3 IMU data
    // @Field: TimeUS: Time since system startup
    // @Field: BootNS: Time since IMU startup
    // @Field: SyncNS: Time since last sync signal
    // @Field: AX1: Accel x value
    // @Field: AY1: Accel y value
    // @Field: AZ1: Accel z value
    // @Field: WX1: Mems gyro x value
    // @Field: WY1: Mems gyro y value
    // @Field: WZ1: Mems gyro z value
    // @Field: OG_WX: FOG gyro x value
    // @Field: OG_WY: FOG gyro y value
    // @Field: OG_WZ: FOG gyro z value
    // @Field: SampUS: jitter-corrected sample timestamp fed to the EKF (compare its stride to TimeUS to see the dt jitter the EKF actually integrates on)
    AP::logger().WriteStreaming("AX31", "TimeUS,BootNS,SyncNS,AX1,AY1,AZ1,WX1,WY1,WZ1,OG_WX,OG_WY,OG_WZ,SampUS",
                                        "sssoooEEEEEEs",
                                        "FII000000000F",
                                        "QQQfffffffffQ",
                       now,
                       bin_payload.mcu_time, bin_payload.sync_time,
                       imu_data.mems_accel.x, imu_data.mems_accel.y, imu_data.mems_accel.z,
                       imu_data.mems_gyro.x, imu_data.mems_gyro.y, imu_data.mems_gyro.z,
                       imu_data.fog_gyro.x, imu_data.fog_gyro.y, imu_data.fog_gyro.z,
                       imu_data.sample_us
                       );

    // @LoggerMessage: AX32
    // @Description: Anello Photonics X3 Mag + other data
    // @Field: TimeUS: Time since system startup
    // @Field: MAG_X: Mag x value
    // @Field: MAG_Y: Mag y value
    // @Field: MAG_Z: Mag z value
    // @Field: Temp: system temperature, in Celsius
    // @Field: FusStatX: fusion status for the x-axis IMU
    // @Field: FusStatY: fusion status for the y-axis IMU
    // @Field: FusStatZ: fusion status for the z-axis IMU
    // @Field: Drop: cumulative IMU samples missed (mcu_time gaps)
    // @Field: CRCErr: cumulative checksum-rejected packets
    AP::logger().WriteStreaming("AX32", "TimeUS,MAG_X,MAG_Y,MAG_Z,Temp,FusStatX,FusStatY,FusStatZ,Drop,CRCErr",
                                        "sGGGO-----",
                                        "FCCC0-----",
                                        "QffffBBBII",
                       now, imu_data.mag.x, imu_data.mag.y, imu_data.mag.z,
                       imu_data.temp, imu_data.fusion_status.x, imu_data.fusion_status.y, imu_data.fusion_status.z,
                       dropped_samples, crc_error_count);
#endif  // HAL_LOGGING_ENABLED
}

// push the latest sample into the frontend; the FOG is the gyro of record
void AP_InertialSensor_AnelloX3::publish_imu()
{
    const uint64_t sample_us = imu_data.sample_us;

    Vector3f accel = imu_data.mems_accel;
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel, sample_us);

    _publish_temperature(accel_instance, imu_data.temp);

    Vector3f gyro = imu_data.fog_gyro;
    _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);
    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, sample_us);
}

#endif  // AP_INERTIALSENSOR_ANELLOX3_ENABLED
