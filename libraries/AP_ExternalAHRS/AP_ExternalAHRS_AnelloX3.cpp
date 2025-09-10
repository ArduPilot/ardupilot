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
    driver for Anello X3 IMU system
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_ANELLOX3_ENABLED

#include "AP_ExternalAHRS_AnelloX3.h"
#include "AP_Compass/AP_Compass_config.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/AP_Math.h>
#include <inttypes.h>
#include <AP_HAL/utility/sparse-endian.h>


extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_AnelloX3::AP_ExternalAHRS_AnelloX3(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    
    // will attach to the first SERIALx port with type AHRS
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    // couldnt find a UART
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Anello X3 ExternalAHRS no UART");
        return;
    }

    // configure to match the X3's default sensor avail
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) ||
            uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS)); 

    // request a thread to run the X3 be created 
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_AnelloX3::update_thread, void),
                "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_UART, 0)) {
        AP_BoardConfig::allocation_error("Anello X3 failed to allocate ExternalAHRS update thread");
    }

    hal.scheduler->delay(5000);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Anello X3 ExternalAHRS initialised");
}

// main loop for X3 thread
void AP_ExternalAHRS_AnelloX3::update_thread(void)
{
    if (!port_open) {
        port_open = true;
        uart->begin(baudrate);
    }

    while (true) {
        build_packet();
        hal.scheduler->delay_microseconds(500);
    }
}

// recieves incoming UART stream and recognizes once a valid packet has been registered
void AP_ExternalAHRS_AnelloX3::build_packet()
{
    if (uart == nullptr) {
        return;
    }
    
    // read bytes from the UART, passing each to the parser
    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes--> 0) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        DescriptorSet descriptor;
        if (handle_byte(b, descriptor)) {
            switch (descriptor) {
                case DescriptorSet::IMUData:
                    post_imu();
                    break;
        }
    }
    }
}

// parsing state machine for incoming bytes
bool AP_ExternalAHRS_AnelloX3::handle_byte(const uint8_t b, DescriptorSet& descriptor)
{
    switch (message_in.state) {
        case ParseState::WaitingFor_SyncOne:
            if (b == SYNC_ONE) {
                message_in.packet.header[0] = b;
                message_in.state = ParseState::WaitingFor_SyncTwo;
            }
            break;
        case ParseState::WaitingFor_SyncTwo:
            if (b == SYNC_TWO) {
                message_in.packet.header[1] = b;
                message_in.state = ParseState::WaitingFor_Descriptor;
            } else {
                message_in.state = ParseState::WaitingFor_SyncOne;
            }
            break;
        case ParseState::WaitingFor_Descriptor:
            message_in.packet.descriptor_set(b);
            message_in.state = ParseState::WaitingFor_PayloadLength;
            break;
        case ParseState::WaitingFor_PayloadLength:
            message_in.packet.payload_length(b);
            message_in.state = ParseState::WaitingFor_Data;
            message_in.index = 0;
            break;
        case ParseState::WaitingFor_Data:
            message_in.packet.payload[message_in.index++] = b;
            if (message_in.index >= message_in.packet.payload_length()) {
                message_in.state = ParseState::WaitingFor_Checksum;
                message_in.index = 0;
            }
            break;
        case ParseState::WaitingFor_Checksum:
            message_in.packet.checksum[message_in.index++] = b;
            if (message_in.index >= 2) {
                message_in.state = ParseState::WaitingFor_SyncOne;
                message_in.index = 0;

                if (valid_packet(message_in.packet)) {
                    descriptor = handle_packet(message_in.packet);
                    return true;
                }
            }
            break;
        }
    return false;
}


// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_AnelloX3::post_imu() const
{
    // save values to the eahr's state variable
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = imu_data.mems_accel;
        if (option_is_set(AP_ExternalAHRS::OPTIONS::X3_USE_MEMS_GYRO)) {
            state.gyro = imu_data.mems_gyro;
        } else {
            state.gyro = imu_data.fog_gyro;
        }
        state.have_quaternion = false;
    }

    // notify the ins to publish our data
    {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel: state.accel,
            gyro: state.gyro,
            temperature: imu_data.temp 
        };
        AP::ins().handle_external(ins);
    }

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    {
        AP_ExternalAHRS::mag_data_message_t mag {
            field: imu_data.mag
        };
        AP::compass().handle_external(mag);
    }
#endif

}

int8_t AP_ExternalAHRS_AnelloX3::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

const char* AP_ExternalAHRS_AnelloX3::get_name() const
{
    return "Anello X3";
}

bool AP_ExternalAHRS_AnelloX3::healthy(void) const
{
    uint32_t now = AP_HAL::millis();

    // unhealthy if packet delayed or fusion status flags are raised
    return (now - last_imu_pkt < 40 && imu_data.fusion_status_x == 0 &&
            imu_data.fusion_status_y == 0 && imu_data.fusion_status_z == 0);
}

bool AP_ExternalAHRS_AnelloX3::initialised(void) const
{
    return last_imu_pkt != 0;
}

bool AP_ExternalAHRS_AnelloX3::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Anello X3 unhealthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_AnelloX3::get_filter_status(nav_filter_status &status) const
{
    // no filter status info from sensor 
}

bool AP_ExternalAHRS_AnelloX3::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    // no variance data from sensor
    return false;
}


uint8_t AP_ExternalAHRS_AnelloX3::num_gps_sensors(void) const
{
    // system does not host GPS sensors
    return 0;
}

// checksum verification
bool AP_ExternalAHRS_AnelloX3::valid_packet(const AnelloX3_Packet & packet)
{
    uint8_t checksum_one = 0;
    uint8_t checksum_two = 0;

    for (int i = 2; i < 4; i++) {
        checksum_one += packet.header[i];
        checksum_two += checksum_one;
    }

    for (int i = 0; i < packet.payload_length(); i++) {
        checksum_one += packet.payload[i];
        checksum_two += checksum_one;
    }

    return packet.checksum[0] == checksum_one && packet.checksum[1] == checksum_two;
}

// directs differnt message types to resp parsers (currently, only IMU)
AP_ExternalAHRS_AnelloX3::DescriptorSet AP_ExternalAHRS_AnelloX3::handle_packet(const AnelloX3_Packet& packet)
{
    const DescriptorSet descriptor = packet.descriptor_set();
    switch (descriptor) {
    case DescriptorSet::IMUData:
        handle_imu(packet);
        break;
    }
    return descriptor;
}

// unpacking function for the X3 IMU messages
void AP_ExternalAHRS_AnelloX3::handle_imu(const AnelloX3_Packet& packet)
{
    // keep track of last recv packet
    last_imu_pkt = AP_HAL::millis(); 

    // struct for holding raw binary data with endian conversion
    struct AnelloX3_BinaryPayload bin_payload;

    // state variables for unpacking
    int i = 0;

    // go through payload chunk-by-chunk
    bin_payload.mcu_time = le64toh_ptr(packet.payload + i);
    i+= 8;

    bin_payload.sync_time = le64toh_ptr(packet.payload + i);
    i += 8;

    bin_payload.ax1 = le16toh_ptr(packet.payload + i);
    i+= 2;

    bin_payload.ay1 = le16toh_ptr(packet.payload + i);
    i += 2;

    bin_payload.az1 = le16toh_ptr(packet.payload + i);
    i += 2;
    
    bin_payload.wx1 = le16toh_ptr(packet.payload + i);
    i+= 2;

    bin_payload.wy1 = le16toh_ptr(packet.payload + i);
    i += 2;

    bin_payload.wz1 = le16toh_ptr(packet.payload + i);
    i += 2;

    bin_payload.og_wx = le32toh_ptr(packet.payload + i);
    i+= 4;

    bin_payload.og_wy = le32toh_ptr(packet.payload + i);
    i += 4;

    bin_payload.og_wz = le32toh_ptr(packet.payload + i);
    i += 4;

    bin_payload.mag_x = le16toh_ptr(packet.payload + i);
    i+= 2;

    bin_payload.mag_y = le16toh_ptr(packet.payload + i);
    i += 2;

    bin_payload.mag_z = le16toh_ptr(packet.payload + i);
    i += 2;

    bin_payload.temp = le16toh_ptr(packet.payload + i);
    i+= 2;

    bin_payload.mems_ranges = le16toh_ptr(packet.payload + i);
    i+= 2;

    bin_payload.fog_range = le16toh_ptr(packet.payload + i);
    i+= 2;

    bin_payload.fusion_status_x = packet.payload[i++];
    bin_payload.fusion_status_y = packet.payload[i++];
    bin_payload.fusion_status_z = packet.payload[i++];

    convert_imu_data(bin_payload);
}

// convert the binary data to actual values
void AP_ExternalAHRS_AnelloX3::convert_imu_data(const AnelloX3_BinaryPayload& bin_payload)
{
    // mems ranges: gggg gggg ggga aaaa where 'g' is a gyro bit and 'a' is an acc bit

    // parse out ranges
    imu_data.mems_acc_range = bin_payload.mems_ranges >> 11; // acc range mask
    imu_data.mems_gyro_range = bin_payload.mems_ranges & 0x07FF; // gyro range shift
    imu_data.fog_gyro_range = bin_payload.fog_range;

    // calculate mems acc data
    imu_data.mems_accel.x = bin_payload.ax1 * imu_data.mems_acc_range * 3.05e-5 * 9.81; // conv from g to m/s^2
    imu_data.mems_accel.y = bin_payload.ay1 * imu_data.mems_acc_range * 3.05e-5 * 9.81;
    imu_data.mems_accel.z = bin_payload.az1 * imu_data.mems_acc_range * 3.05e-5 * 9.81;

    // calculate mems gyro data
    imu_data.mems_gyro.x = bin_payload.wx1 * imu_data.mems_gyro_range * 3.5e-5 * DEG_TO_RAD;
    imu_data.mems_gyro.y = bin_payload.wy1 * imu_data.mems_gyro_range * 3.5e-5 * DEG_TO_RAD;
    imu_data.mems_gyro.z = bin_payload.wz1 * imu_data.mems_gyro_range * 3.5e-5 * DEG_TO_RAD;

    // calculate fog gyro data
    imu_data.fog_gyro.x = bin_payload.og_wx * 1e-7 * DEG_TO_RAD;
    imu_data.fog_gyro.y = bin_payload.og_wy * 1e-7 * DEG_TO_RAD;
    imu_data.fog_gyro.z = bin_payload.og_wz * 1e-7 * DEG_TO_RAD;

    // calculate mag data
    imu_data.mag.x = bin_payload.mag_x * 0.2441; // combined conv gauss to milligauss
    imu_data.mag.y = bin_payload.mag_y * 0.2441;
    imu_data.mag.z = bin_payload.mag_z * 0.2441;

    // calculate temperature
    imu_data.temp = bin_payload.temp * 1e-2;

    // transfer statuses
    imu_data.fusion_status_x = bin_payload.fusion_status_x;
    imu_data.fusion_status_y = bin_payload.fusion_status_y;
    imu_data.fusion_status_z = bin_payload.fusion_status_z;

#if HAL_LOGGING_ENABLED
    auto now =  AP_HAL::micros64();
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

    
    AP::logger().WriteStreaming("AX31", "TimeUS,BootNS,SyncNS,AX1,AY1,AZ1,WX1,WY1,WZ1,OG_WX,OG_WY,OG_WZ",
                                        "sssoooEEEEEE",
                                        "FII000000000",
                                        "QQQfffffffff",
                       now,
                       bin_payload.mcu_time, bin_payload.sync_time,
                       imu_data.mems_accel.x, imu_data.mems_accel.y, imu_data.mems_accel.z,
                       imu_data.mems_gyro.x, imu_data.mems_gyro.y, imu_data.mems_gyro.z,
                       imu_data.fog_gyro.x, imu_data.fog_gyro.y, imu_data.fog_gyro.z
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

    
    AP::logger().WriteStreaming("AX32", "TimeUS,MAG_X,MAG_Y,MAG_Z,Temp,FusStatX,FusStatY,FusStatZ",
                                        "sGGGO---",
                                        "FCCC0---",
                                        "QffffBBB",
                       now, imu_data.mag.x, imu_data.mag.y, imu_data.mag.z,
                       imu_data.temp, imu_data.fusion_status_x, imu_data.fusion_status_y, imu_data.fusion_status_z);
#endif  // HAL_LOGGING_ENABLED

}


#endif // AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED 
