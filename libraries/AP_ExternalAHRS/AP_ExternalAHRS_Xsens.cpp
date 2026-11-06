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
  support for Xsens INS devices (MTi-680G, MTi-670G, MTi-G-710, MTi-7, MTi-8)
 */


#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_XSENS_ENABLED

#include <AP_HAL/SPIDevice.h>
#include "AP_ExternalAHRS_Xsens.h"
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <cstring>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_Xsens::AP_ExternalAHRS_Xsens(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state),
        drdy_gpio_pin(-1)
{
    // Don't offer the IMU by default: at the EAHRS_RATE output rate (50Hz default) it is
    // far too slow for the vehicle rate controllers, and ArduPilot marks it unhealthy
    // in every loop without a new sample. The autopilot's own IMUs are used for rate
    // control; attitude, GPS, baro and compass still come from the Xsens.
    // Set EAHRS_SENSORS bit 1 to use the Xsens IMU anyway.
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    // Check if SPI mode is requested
    if (option_is_set(AP_ExternalAHRS::OPTIONS::XSENS_USE_SPI)) {
        interface_type = InterfaceType::SPI;
        
        if (!init_spi()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Xsens SPI init failed");
            return;
        }
        
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS: Xsens using SPI6");
    } else {
        interface_type = InterfaceType::UART;
        
        auto &sm = AP::serialmanager();
        uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
        
        baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
        port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

        if (!uart) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
            return;
        }
        
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS: Xsens using UART");
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_Xsens::update_thread, void), 
                                      "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS update thread");
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xsens ExternalAHRS initialised");
}

// update_thread to handle UART port opening
void AP_ExternalAHRS_Xsens::update_thread(void)
{
    if (interface_type == InterfaceType::UART && !port_open) {
        port_open = true;
        uart->begin(baudrate);
    }

    while (true) {
        process_received_data();
        update_state_machine();
        hal.scheduler->delay_microseconds(1000);
    }
}

// write_message to support both interfaces
bool AP_ExternalAHRS_Xsens::write_message(const uint8_t *message, size_t length)
{
    if (interface_type == InterfaceType::SPI) {
        send_spi_message(message, length);
        return true;
    } else {
        // Original UART code
        if (uart == nullptr) {
            return false;
        }
        ssize_t bytes_written = uart->write(message, length);
        return bytes_written == (ssize_t)length;
    }
}

// read_data to support both interfaces
size_t AP_ExternalAHRS_Xsens::read_data(uint8_t *buffer, size_t max_length)
{
    if (interface_type == InterfaceType::SPI) {
        return 0; // SPI uses different read mechanism
    } else {
        // Original UART code
        if (uart == nullptr) {
            return 0;
        }
        ssize_t bytes_read = uart->read(buffer, max_length);
        if (bytes_read < 0) {
            return 0;
        }
        return static_cast<size_t>(bytes_read);
    }
}


void AP_ExternalAHRS_Xsens::process_received_data()
{
    WITH_SEMAPHORE(sem);
    
    if (interface_type == InterfaceType::SPI) {
        // Check if data is ready
        if (!check_drdy()) {
            return;
        }
        
        uint16_t notif_size = 0;
        uint16_t meas_size = 0;
        read_pipe_status(notif_size, meas_size);
        
        // Validate sizes to prevent buffer overflows
        if (notif_size > BUFFER_SIZE - 2) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Xsens: Notification size too large: %u", notif_size);
            notif_size = 0;
        }
        
        if (meas_size > BUFFER_SIZE - 2) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Xsens: Measurement size too large: %u", meas_size);
            meas_size = 0;
        }
        
        // Read notification pipe
        if (notif_size > 0) {
            uint8_t temp_buffer[BUFFER_SIZE];
            temp_buffer[0] = XBUS_PREAMBLE;
            temp_buffer[1] = XBUS_MASTERDEVICE;
            
            read_pipe(&temp_buffer[2], notif_size, XBUS_NOTIFICATION_PIPE);
            
            if (verify_checksum(temp_buffer)) {
                handle_message(temp_buffer);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Xsens: Notification checksum failed");
            }
        }
        
        // Read measurement pipe
        if (meas_size > 0) {
            uint8_t temp_buffer[BUFFER_SIZE];
            temp_buffer[0] = XBUS_PREAMBLE;
            temp_buffer[1] = XBUS_MASTERDEVICE;
            
            read_pipe(&temp_buffer[2], meas_size, XBUS_MEASUREMENT_PIPE);
            
             if (verify_checksum(temp_buffer)) {
                handle_message(temp_buffer);
             } else {
                 GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Xsens: Measurement checksum failed");
             }
        }
    } else {
        // Original UART code
        size_t bytes_available = read_data(rx_buffer + rx_buffer_pos, BUFFER_SIZE - rx_buffer_pos);
        
        if (bytes_available == 0) {
            return;
        }

        rx_buffer_pos += bytes_available;

        // Look for complete messages in the buffer
        size_t search_start = 0;

        // Bytes before search_start are consumed; they are removed once, after this loop
        while (search_start < rx_buffer_pos) {
            // Look for preamble (0xFA)
            size_t preamble_pos = search_start;
            while (preamble_pos < rx_buffer_pos && rx_buffer[preamble_pos] != XBUS_PREAMBLE) {
                preamble_pos++;
            }

            if (preamble_pos >= rx_buffer_pos) {
                // No preamble found, discard everything
                search_start = rx_buffer_pos;
                break;
            }

            // Check if we have enough data for a complete message header
            if (preamble_pos + 4 > rx_buffer_pos) {
                // Partial header: keep it and wait for more bytes
                search_start = preamble_pos;
                break;
            }

            // Validate message structure
            if (rx_buffer[preamble_pos + 1] != XBUS_MASTERDEVICE) {
                // Invalid BID, skip this byte
                search_start = preamble_pos + 1;
                continue;
            }

            uint8_t length = rx_buffer[preamble_pos + 3];
            size_t total_msg_length = 4 + length + 1; // Header + payload + checksum

            if (preamble_pos + total_msg_length > rx_buffer_pos) {
                // Partial message: keep it and wait for more bytes
                search_start = preamble_pos;
                break;
            }

            // We have a complete message, verify checksum
            if (verify_checksum(rx_buffer + preamble_pos)) {
                handle_message(rx_buffer + preamble_pos);
                search_start = preamble_pos + total_msg_length;
            } else {
                // Bad message: resync from the next byte instead of skipping a whole length
                search_start = preamble_pos + 1;
            }
        }

        // Remove processed data from buffer
        if (search_start > 0) {
            if (search_start < rx_buffer_pos) {
                memmove(rx_buffer, rx_buffer + search_start, rx_buffer_pos - search_start);
                rx_buffer_pos -= search_start;
            } else {
                rx_buffer_pos = 0;
            }
        }
        
    }
}


void AP_ExternalAHRS_Xsens::handle_message(const uint8_t *message)
{
    uint8_t message_id = get_message_id(message);

    switch (message_id) {
        case XsMessageId::GotoConfigAck:
            if (device_state == DeviceState::WAITING_FOR_CONFIG_MODE) {
                set_device_state(DeviceState::CONFIGURING_OUTPUT);
            }
            break;

        case XsMessageId::OutputConfig:
            if (device_state == DeviceState::WAITING_FOR_OUTPUT_CONFIG) {
                set_device_state(DeviceState::CONFIGURING_ROTLOCAL);
            }
            break;

        case XsMessageId::AlignmentRotationAck:
            if (device_state == DeviceState::WAITING_FOR_ROTLOCAL_CONFIG) {
                set_device_state(DeviceState::CONFIGURING_ROTSENSOR);
            } else if (device_state == DeviceState::WAITING_FOR_ROTSENSOR_CONFIG) {
                set_device_state(DeviceState::ENTERING_MEASUREMENT_MODE);
            }
            break;

        case XsMessageId::GotoMeasurementAck:
            if (device_state == DeviceState::WAITING_FOR_MEASUREMENT_MODE) {
                set_device_state(DeviceState::RUNNING);
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xsens device configured, now running");
            }
            break;

        case XsMessageId::MtData2:
	    set_device_state(DeviceState::RUNNING);
            handle_mtdata2_message(message);
            break;

        case XsMessageId::Error:
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Xsens device error");
            handle_error_message(message);
            break;

        default:
            break;
    }
}

void AP_ExternalAHRS_Xsens::handle_mtdata2_message(const uint8_t *message)
{
    SensorData sensor_data;
    if (parse_mtdata2(message, sensor_data)) {
        last_ins_pkt = AP_HAL::millis();
        
        // Update buffered GNSS PVT data and GPS status if available
        if (sensor_data.hasGnssPvtData) {
            buffered_gnss_pvt = sensor_data.gnssPvtData;
            has_buffered_gnss_pvt = true;
            last_gnss_pvt_update = AP_HAL::millis();
            
            // Update persistent GPS status from GNSS data - this is the TRUTH
            last_valid_fix_type = convert_fix_type(buffered_gnss_pvt.fixType, buffered_gnss_pvt.flags);
            last_satellites_in_view = buffered_gnss_pvt.numSv;
            last_horizontal_pos_accuracy = buffered_gnss_pvt.hAcc * 1.0e-3f; // Convert mm to m
            last_vertical_pos_accuracy = buffered_gnss_pvt.vAcc * 1.0e-3f;   // Convert mm to m
            last_horizontal_vel_accuracy = buffered_gnss_pvt.sAcc * 1.0e-3f; // Convert mm/s to m/s
            // PVT DOPs are scaled by 100, which is also what AP_GPS expects
            last_hdop = buffered_gnss_pvt.hDop;
            last_vdop = buffered_gnss_pvt.vDop;

            // Geoid separation (ellipsoid height - MSL height), only meaningful with a fix.
            // The MTi outputs altitude above the ellipsoid, ArduPilot wants AMSL.
            if (buffered_gnss_pvt.fixType >= 2 && buffered_gnss_pvt.fixType <= 4) {
                geoid_separation_m = (buffered_gnss_pvt.height - buffered_gnss_pvt.hMSL) * 1.0e-3f;
                have_geoid_separation = true;
            }

            gps_status_initialized = true;
        }

        // Remember when the MTi last output a position
        if (sensor_data.hasLatLon && sensor_data.hasVelocityXYZ) {
            last_pos_ms = AP_HAL::millis();
        }

        if (sensor_data.hasStatusWord) {
            last_status_word = sensor_data.statusWord;
            have_status_word = true;
        }

        publish_sensor_data(sensor_data);
    }
}

void AP_ExternalAHRS_Xsens::handle_error_message(const uint8_t *message)
{
    set_device_state(DeviceState::ERROR);
}

void AP_ExternalAHRS_Xsens::update_state_machine()
{
    uint32_t now = AP_HAL::millis();

    // Check for timeout
    if (now > state_timeout) {
        switch (device_state) {
            case DeviceState::WAITING_FOR_CONFIG_MODE:
                set_device_state(DeviceState::ENTERING_CONFIG_MODE);
                break;
            case DeviceState::WAITING_FOR_OUTPUT_CONFIG:
                set_device_state(DeviceState::CONFIGURING_OUTPUT);
                break;
            case DeviceState::WAITING_FOR_ROTLOCAL_CONFIG:
                set_device_state(DeviceState::CONFIGURING_ROTLOCAL);
                break;
            case DeviceState::WAITING_FOR_ROTSENSOR_CONFIG:
                set_device_state(DeviceState::CONFIGURING_ROTSENSOR);
                break;
            case DeviceState::WAITING_FOR_MEASUREMENT_MODE:
                set_device_state(DeviceState::ENTERING_MEASUREMENT_MODE);
                break;
            case DeviceState::RUNNING:
                if (now - last_ins_pkt > 5000) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Xsens data timeout, reinitializing");
                    set_device_state(DeviceState::ENTERING_CONFIG_MODE);
                }
                return;
            default:
                set_device_state(DeviceState::ERROR);
                break;
        }
        return;
    }

    switch (device_state) {
        case DeviceState::ENTERING_CONFIG_MODE:
            if (goto_config_mode()) {
                set_device_state(DeviceState::WAITING_FOR_CONFIG_MODE);
            } else {
                set_device_state(DeviceState::ERROR);
            }
            break;

        case DeviceState::CONFIGURING_OUTPUT:
            if (configure_output_settings()) {
                set_device_state(DeviceState::WAITING_FOR_OUTPUT_CONFIG);
            } else {
                set_device_state(DeviceState::ERROR);
            }
            break;

        case DeviceState::CONFIGURING_ROTLOCAL:
            if (set_ned_frame()) {
                set_device_state(DeviceState::WAITING_FOR_ROTLOCAL_CONFIG);
            } else {
                set_device_state(DeviceState::ERROR);
            }
            break;

        case DeviceState::CONFIGURING_ROTSENSOR:
            {   
                // Use EAHRS_OPTIONS bit 1 for sensor orientation
                // Bit 1: 0 = sensor upward  (default), 1 = sensor downward facing
                bool is_sensor_down_facing = option_is_set(AP_ExternalAHRS::OPTIONS::XSENS_SENSOR_DOWNWARD);
                if (set_sensor_frame(is_sensor_down_facing)) { 
                    set_device_state(DeviceState::WAITING_FOR_ROTSENSOR_CONFIG);
                } else {
                    set_device_state(DeviceState::ERROR);
                }
            }
            break;

        case DeviceState::ENTERING_MEASUREMENT_MODE:
            if (goto_measurement_mode()) {
                set_device_state(DeviceState::WAITING_FOR_MEASUREMENT_MODE);
            } else {
                set_device_state(DeviceState::ERROR);
            }
            break;

        case DeviceState::ERROR:
            if (now - state_timeout > 5000) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xsens attempting recovery from error");
                set_device_state(DeviceState::ENTERING_CONFIG_MODE);
            }
            break;

        default:
            break;
    }
}

void AP_ExternalAHRS_Xsens::set_device_state(DeviceState new_state)
{
    if (device_state != new_state) {
        device_state = new_state;
        
        switch (new_state) {
            case DeviceState::RUNNING:
                state_timeout = AP_HAL::millis() + 3600000; // 1 hour timeout
                running_since_ms = AP_HAL::millis();
                break;
            case DeviceState::WAITING_FOR_CONFIG_MODE:
            case DeviceState::WAITING_FOR_OUTPUT_CONFIG:
            case DeviceState::WAITING_FOR_ROTLOCAL_CONFIG:
            case DeviceState::WAITING_FOR_ROTSENSOR_CONFIG:
            case DeviceState::WAITING_FOR_MEASUREMENT_MODE:
                state_timeout = AP_HAL::millis() + 2000; // 2 second timeout
                break;
            default:
                state_timeout = AP_HAL::millis() + 5000; // 5 second timeout
                break;
        }
    }
}

bool AP_ExternalAHRS_Xsens::goto_config_mode()
{
    uint8_t config_msg[5];
    create_message(config_msg, XBUS_MASTERDEVICE, XsMessageId::GotoConfig, 0);
    insert_checksum(config_msg);
    return write_message(config_msg, 5);
}

bool AP_ExternalAHRS_Xsens::configure_output_settings()
{
    uint8_t config_msg[64];
    create_message(config_msg, XBUS_MASTERDEVICE, XsMessageId::SetOutputConfig, 0);

    uint8_t *payload = get_pointer_to_payload(config_msg);
    int payload_idx = 0;

    uint16_t output_rate = get_rate(); // This gets the user-configured EAHRS_RATE parameter

    // Configure desired data items
    uint16_t data_items[] = {
        XDI::PACKET_COUNTER,
        XDI::SAMPLE_TIME_FINE,
        XDI::UTC_TIME,
        XDI::TEMPERATURE,
        XDI::QUATERNION,
        XDI::ACCELERATION,
        XDI::RATE_OF_TURN,
        XDI::MAGNETIC_FIELD,
        XDI::STATUS_WORD,
        XDI::BAROMETRIC_PRESSURE,
        XDI::LAT_LON,
        XDI::ALTITUDE_ELLIPSOID,
        XDI::VELOCITY_XYZ,
        XDI::GNSSPVTDATA
    };

    for (uint16_t item : data_items) {
        payload[payload_idx++] = (item >> 8) & 0xFF;
        payload[payload_idx++] = item & 0xFF;
        payload[payload_idx++] = (output_rate >> 8) & 0xFF;
        payload[payload_idx++] = output_rate & 0xFF;
    }

    set_payload_length(config_msg, payload_idx);
    insert_checksum(config_msg);

    return write_message(config_msg, get_raw_length(config_msg));
}

bool AP_ExternalAHRS_Xsens::set_ned_frame()
{
    uint8_t config_msg[22];
    create_message(config_msg, XBUS_MASTERDEVICE, XsMessageId::SetAlignmentRotation, 0);

    uint8_t *payload = get_pointer_to_payload(config_msg);
    int payload_idx = 0;

    // RotLocal frame (0x01)
    payload[payload_idx++] = 0x01;

    // NED frame quaternion: [0, 0.7071068, 0.7071068, 0]
    uint32_t quat_bytes[] = {0x00000000, 0x3F3504F4, 0x3F3504F4, 0x00000000};
    
    for (int i = 0; i < 4; i++) {
        payload[payload_idx++] = (quat_bytes[i] >> 24) & 0xFF;
        payload[payload_idx++] = (quat_bytes[i] >> 16) & 0xFF;
        payload[payload_idx++] = (quat_bytes[i] >> 8) & 0xFF;
        payload[payload_idx++] = quat_bytes[i] & 0xFF;
    }

    set_payload_length(config_msg, payload_idx);
    insert_checksum(config_msg);

    return write_message(config_msg, get_raw_length(config_msg));
}

bool AP_ExternalAHRS_Xsens::set_sensor_frame(bool is_sensor_down_facing)
{
    uint8_t config_msg[22];
    create_message(config_msg, XBUS_MASTERDEVICE, XsMessageId::SetAlignmentRotation, 0);

    uint8_t *payload = get_pointer_to_payload(config_msg);
    int payload_idx = 0;

    // RotSensor frame (0x00)
    payload[payload_idx++] = 0x00;

    uint32_t quat_bytes[4];
    if (is_sensor_down_facing) 
    {
        // Sensor face downwards: quaternion [1, 0, 0, 0] (no rotation)
        quat_bytes[0] = 0x3F800000; // w = 1.0
        quat_bytes[1] = 0x00000000; // x = 0.0
        quat_bytes[2] = 0x00000000; // y = 0.0
        quat_bytes[3] = 0x00000000; // z = 0.0
    }
    else {
        // Sensor face upwards: quaternion [0, 1, 0, 0] (180° rotation around X)
        quat_bytes[0] = 0x00000000; // w = 0.0
        quat_bytes[1] = 0x3F800000; // x = 1.0
        quat_bytes[2] = 0x00000000; // y = 0.0
        quat_bytes[3] = 0x00000000; // z = 0.0
    }

    for (int i = 0; i < 4; i++) {
        payload[payload_idx++] = (quat_bytes[i] >> 24) & 0xFF;
        payload[payload_idx++] = (quat_bytes[i] >> 16) & 0xFF;
        payload[payload_idx++] = (quat_bytes[i] >> 8) & 0xFF;
        payload[payload_idx++] = quat_bytes[i] & 0xFF;
    }

    set_payload_length(config_msg, payload_idx);
    insert_checksum(config_msg);

    return write_message(config_msg, get_raw_length(config_msg));
}

bool AP_ExternalAHRS_Xsens::goto_measurement_mode()
{
    uint8_t measurement_msg[5];
    create_message(measurement_msg, XBUS_MASTERDEVICE, XsMessageId::GotoMeasurement, 0);
    insert_checksum(measurement_msg);
    return write_message(measurement_msg, 5);
}

// Xbus protocol helper functions
bool AP_ExternalAHRS_Xsens::check_preamble(const uint8_t* xbus_message) const
{
    return xbus_message[0] == XBUS_PREAMBLE;
}

uint8_t AP_ExternalAHRS_Xsens::get_message_id(const uint8_t* xbus_message) const
{
    return xbus_message[2];
}

uint8_t AP_ExternalAHRS_Xsens::get_payload_length(const uint8_t* xbus_message) const
{
    uint8_t length = xbus_message[3];
    if (length != LENGTH_EXTENDER_BYTE) {
        return length;
    } else {
        // Extended length format
        return (xbus_message[4] << 8) | xbus_message[5];
    }
}

void AP_ExternalAHRS_Xsens::create_message(uint8_t* xbus_message, uint8_t bid, uint8_t mid, uint16_t len)
{
    xbus_message[0] = XBUS_PREAMBLE;
    xbus_message[1] = bid;
    xbus_message[2] = mid;
    set_payload_length(xbus_message, len);
}

void AP_ExternalAHRS_Xsens::set_payload_length(uint8_t* xbus_message, uint16_t payload_length)
{
    if (payload_length < 255) {
        xbus_message[3] = payload_length & 0xFF;
    } else {
        xbus_message[3] = LENGTH_EXTENDER_BYTE;
        xbus_message[4] = (payload_length >> 8) & 0xFF;
        xbus_message[5] = payload_length & 0xFF;
    }
}

uint8_t* AP_ExternalAHRS_Xsens::get_pointer_to_payload(uint8_t* xbus_message)
{
    if (xbus_message[3] == LENGTH_EXTENDER_BYTE) {
        return xbus_message + 6;
    } else {
        return xbus_message + 4;
    }
}

const uint8_t* AP_ExternalAHRS_Xsens::get_const_pointer_to_payload(const uint8_t* xbus_message) const
{
    if (xbus_message[3] == LENGTH_EXTENDER_BYTE) {
        return xbus_message + 6;
    } else {
        return xbus_message + 4;
    }
}

int AP_ExternalAHRS_Xsens::get_raw_length(const uint8_t* xbus_message) const
{
    int payload_len = get_payload_length(xbus_message);
    
    if (xbus_message[3] == LENGTH_EXTENDER_BYTE) {
        return payload_len + 7; // Header (6) + checksum (1)
    } else {
        return payload_len + 5; // Header (4) + checksum (1)
    }
}

void AP_ExternalAHRS_Xsens::insert_checksum(uint8_t* xbus_message)
{
    int msg_length = get_raw_length(xbus_message);
    
    uint8_t checksum = 0;
    for (int i = 1; i < msg_length - 1; i++) {
        checksum += xbus_message[i];
    }
    checksum = 0x100 - checksum;
    
    xbus_message[msg_length - 1] = checksum;
}

bool AP_ExternalAHRS_Xsens::verify_checksum(const uint8_t* xbus_message) const
{
    int msg_length = get_raw_length(xbus_message);
    
    uint8_t checksum = 0;
    for (int i = 1; i < msg_length; i++) {
        checksum += xbus_message[i];
    }
    
    return (checksum & 0xFF) == 0;
}

// Data parsing helper functions
uint8_t AP_ExternalAHRS_Xsens::read_uint8(const uint8_t* data, int& index) const
{
    return data[index++];
}

uint16_t AP_ExternalAHRS_Xsens::read_uint16(const uint8_t* data, int& index) const
{
    uint16_t result = 0;
    result |= data[index++] << 8;
    result |= data[index++] << 0;
    return result;
}

uint32_t AP_ExternalAHRS_Xsens::read_uint32(const uint8_t* data, int& index) const
{
    uint32_t result = 0;
    result |= data[index++] << 24;
    result |= data[index++] << 16;
    result |= data[index++] << 8;
    result |= data[index++] << 0;
    return result;
}

int32_t AP_ExternalAHRS_Xsens::read_int32(const uint8_t* data, int& index) const
{
    uint32_t temp = read_uint32(data, index);
    return static_cast<int32_t>(temp);
}

float AP_ExternalAHRS_Xsens::read_float(const uint8_t* data, int& index) const
{
    uint32_t temp = read_uint32(data, index);
    float result;
    memcpy(&result, &temp, 4);
    return result;
}

double AP_ExternalAHRS_Xsens::read_fp1632(const uint8_t* data, int& index) const
{
    // FP16.32 format: 32-bit fractional part followed by 16-bit integer part
    uint32_t fractional_part = read_uint32(data, index);
    uint16_t integer_part_unsigned = read_uint16(data, index);
    int16_t integer_part = static_cast<int16_t>(integer_part_unsigned);

    // Combine into 48-bit signed value and convert to double
    int64_t fixed_point_value = (static_cast<int64_t>(integer_part) << 32) | 
                                (static_cast<int64_t>(fractional_part) & 0xffffffff);
    // Convert from fixed point to double: divide by 2^32
    double result = static_cast<double>(fixed_point_value) / 4294967296.0;
    
    return result;
}

bool AP_ExternalAHRS_Xsens::parse_gnss_pvt_data(const uint8_t* payload, int& index, uint8_t size, GnssPvtData& gnss_pvt)
{
    if (size != 94) {
        // Invalid size for GnssPvtData
        index += size;
        return false;
    }

    gnss_pvt.iTOW = read_uint32(payload, index);
    gnss_pvt.year = read_uint16(payload, index);
    gnss_pvt.month = read_uint8(payload, index);
    gnss_pvt.day = read_uint8(payload, index);
    gnss_pvt.hour = read_uint8(payload, index);
    gnss_pvt.min = read_uint8(payload, index);
    gnss_pvt.sec = read_uint8(payload, index);
    gnss_pvt.valid = read_uint8(payload, index);
    gnss_pvt.tAcc = read_uint32(payload, index);
    gnss_pvt.nano = read_int32(payload, index);
    gnss_pvt.fixType = read_uint8(payload, index);
    gnss_pvt.flags = read_uint8(payload, index);
    gnss_pvt.numSv = read_uint8(payload, index);
    gnss_pvt.reserved = read_uint8(payload, index);
    gnss_pvt.lon = read_int32(payload, index);
    gnss_pvt.lat = read_int32(payload, index);
    gnss_pvt.height = read_int32(payload, index);
    gnss_pvt.hMSL = read_int32(payload, index);
    gnss_pvt.hAcc = read_uint32(payload, index);
    gnss_pvt.vAcc = read_uint32(payload, index);
    gnss_pvt.velN = read_int32(payload, index);
    gnss_pvt.velE = read_int32(payload, index);
    gnss_pvt.velD = read_int32(payload, index);
    gnss_pvt.gSpeed = read_int32(payload, index);
    gnss_pvt.headMot = read_int32(payload, index);
    gnss_pvt.sAcc = read_uint32(payload, index);
    gnss_pvt.headAcc = read_uint32(payload, index);
    gnss_pvt.headVeh = read_int32(payload, index);
    gnss_pvt.gDop = read_uint16(payload, index);
    gnss_pvt.pDop = read_uint16(payload, index);
    gnss_pvt.tDop = read_uint16(payload, index);
    gnss_pvt.vDop = read_uint16(payload, index);
    gnss_pvt.hDop = read_uint16(payload, index);
    gnss_pvt.nDop = read_uint16(payload, index);
    gnss_pvt.eDop = read_uint16(payload, index);

    return true;
}

bool AP_ExternalAHRS_Xsens::parse_mtdata2(const uint8_t* xbus_data, SensorData& sensor_data)
{
    if (!check_preamble(xbus_data)) {
        return false;
    }

    uint8_t message_id = get_message_id(xbus_data);
    if (message_id != XsMessageId::MtData2) {
        return false;
    }

    // Reset the sensor data structure
    sensor_data = SensorData();

    int payload_length = get_payload_length(xbus_data);
    const uint8_t* payload = get_const_pointer_to_payload(xbus_data);

    int index = 0;

    // Parse all data items in the payload
    while (index < payload_length) {
        if (index + 3 > payload_length) {
            break; // Not enough bytes for XDI and size
        }

        uint16_t xdi = read_uint16(payload, index);
        uint8_t size = read_uint8(payload, index);

        if (index + size > payload_length) {
            break; // Not enough bytes for the data
        }

        switch (xdi) {
            case XDI::PACKET_COUNTER:
                if (size == 2) {
                    sensor_data.packetCounter = read_uint16(payload, index);
                    sensor_data.hasPacketCounter = true;
                } else {
                    index += size;
                }
                break;

            case XDI::SAMPLE_TIME_FINE:
                if (size == 4) {
                    sensor_data.sampleTimeFine = read_uint32(payload, index);
                    sensor_data.hasSampleTimeFine = true;
                } else {
                    index += size;
                }
                break;

            case XDI::STATUS_WORD:
                if (size == 4) {
                    sensor_data.statusWord = read_uint32(payload, index);
                    sensor_data.hasStatusWord = true;
                } else {
                    index += size;
                }
                break;

            case XDI::LAT_LON:
                if (size == 12) {
                    sensor_data.latLon.latitude = read_fp1632(payload, index);
                    sensor_data.latLon.longitude = read_fp1632(payload, index);
                    sensor_data.hasLatLon = true;
                } else {
                    index += size;
                }
                break;

            case XDI::ALTITUDE_ELLIPSOID:
                if (size == 6) {
                    sensor_data.altitudeEllipsoid = read_fp1632(payload, index);
                    sensor_data.hasAltitudeEllipsoid = true;
                } else {
                    index += size;
                }
                break;

            case XDI::VELOCITY_XYZ:
                if (size == 18) {
                    sensor_data.velocityXYZ.velX = read_fp1632(payload, index);
                    sensor_data.velocityXYZ.velY = read_fp1632(payload, index);
                    sensor_data.velocityXYZ.velZ = read_fp1632(payload, index);
                    sensor_data.hasVelocityXYZ = true;
                } else {
                    index += size;
                }
                break;

            case XDI::UTC_TIME:
                if (size == 12) {
                    sensor_data.utcTime.nanoseconds = read_uint32(payload, index);
                    sensor_data.utcTime.year = read_uint16(payload, index);
                    sensor_data.utcTime.month = read_uint8(payload, index);
                    sensor_data.utcTime.day = read_uint8(payload, index);
                    sensor_data.utcTime.hour = read_uint8(payload, index);
                    sensor_data.utcTime.minute = read_uint8(payload, index);
                    sensor_data.utcTime.second = read_uint8(payload, index);
                    sensor_data.utcTime.flags = read_uint8(payload, index);
                    sensor_data.hasUtcTime = true;
                } else {
                    index += size;
                }
                break;

            case XDI::QUATERNION:
                if (size == 16) {
                    sensor_data.quaternion.q0 = read_float(payload, index);
                    sensor_data.quaternion.q1 = read_float(payload, index);
                    sensor_data.quaternion.q2 = read_float(payload, index);
                    sensor_data.quaternion.q3 = read_float(payload, index);
                    sensor_data.hasQuaternion = true;
                } else {
                    index += size;
                }
                break;

            case XDI::BAROMETRIC_PRESSURE:
                if (size == 4) {
                    sensor_data.barometricPressure = read_uint32(payload, index);
                    sensor_data.hasBarometricPressure = true;
                } else {
                    index += size;
                }
                break;

            case XDI::ACCELERATION:
                if (size == 12) {
                    sensor_data.acceleration.accX = read_float(payload, index);
                    sensor_data.acceleration.accY = read_float(payload, index);
                    sensor_data.acceleration.accZ = read_float(payload, index);
                    sensor_data.hasAcceleration = true;
                } else {
                    index += size;
                }
                break;

            case XDI::RATE_OF_TURN:
                if (size == 12) {
                    sensor_data.rateOfTurn.gyrX = read_float(payload, index);
                    sensor_data.rateOfTurn.gyrY = read_float(payload, index);
                    sensor_data.rateOfTurn.gyrZ = read_float(payload, index);
                    sensor_data.hasRateOfTurn = true;
                } else {
                    index += size;
                }
                break;

            case XDI::MAGNETIC_FIELD:
                if (size == 12) {
                    sensor_data.magneticField.magX = read_float(payload, index);
                    sensor_data.magneticField.magY = read_float(payload, index);
                    sensor_data.magneticField.magZ = read_float(payload, index);
                    sensor_data.hasMagneticField = true;
                } else {
                    index += size;
                }
                break;

            case XDI::TEMPERATURE:
                if (size == 4) {
                    sensor_data.temperature = read_float(payload, index);
                    sensor_data.hasTemperature = true;
                } else {
                    index += size;
                }
                break;

            case XDI::GNSSPVTDATA:
                if (parse_gnss_pvt_data(payload, index, size, sensor_data.gnssPvtData)) {
                    sensor_data.hasGnssPvtData = true;
                }
                break;

            default:
                // Unknown XDI, skip it
                index += size;
                break;
        }
    }

    return true;
}

AP_GPS_FixType AP_ExternalAHRS_Xsens::convert_fix_type(uint8_t fix_type, uint8_t flags) const
{
    switch (fix_type) {
        case 0:
        case 1:
            return AP_GPS_FixType::NONE;
        case 2:
            return AP_GPS_FixType::FIX_2D;
        case 3:
            // carrSoln (bits 7..6) must be checked before diffSoln (bit 1):
            // RTK solutions also have diffSoln set
            switch ((flags >> 6) & 0x03) {
            case 1:
                return AP_GPS_FixType::RTK_FLOAT;
            case 2:
                return AP_GPS_FixType::RTK_FIXED;
            default:
                break;
            }
            if (flags & 0b00000010) {  // diffSoln
                return AP_GPS_FixType::DGPS;
            }
            return AP_GPS_FixType::FIX_3D;
        case 4:
            return AP_GPS_FixType::FIX_3D;
        case 5:
        default:
            return AP_GPS_FixType::NONE;
    }
}

void AP_ExternalAHRS_Xsens::publish_sensor_data(const SensorData &data)
{
    {
        WITH_SEMAPHORE(state.sem);
        
        // Update IMU data
        if (data.hasAcceleration) {
            state.accel = Vector3f(data.acceleration.accX, data.acceleration.accY, data.acceleration.accZ);
        }
        
        if (data.hasRateOfTurn) {
            state.gyro = Vector3f(data.rateOfTurn.gyrX, data.rateOfTurn.gyrY, data.rateOfTurn.gyrZ);
        }

        // Update quaternion
        if (data.hasQuaternion) {
            state.quat = Quaternion(data.quaternion.q0, data.quaternion.q1, 
                                   data.quaternion.q2, data.quaternion.q3);
            state.have_quaternion = true;
        }

        // Update position data. The MTi altitude is above the ellipsoid; Location ABSOLUTE
        // is AMSL, so wait until the geoid separation from GnssPvtData is known.
        if (data.hasLatLon && data.hasAltitudeEllipsoid && have_geoid_separation) {
            const Location loc {
                int32_t(data.latLon.latitude * 1e7),
                int32_t(data.latLon.longitude * 1e7),
                int32_t((data.altitudeEllipsoid - geoid_separation_m) * 100), // m to cm, AMSL
                Location::AltFrame::ABSOLUTE
            };
            state.location = loc;
            state.have_location = true;
            state.last_location_update_us = AP_HAL::micros();

            // Set origin if not set, once the MTi filter uses GNSS
            if (!state.have_origin && filter_gnss_aided()) {
                state.origin = loc;
                state.have_origin = true;
            }
        }

        // Update velocity data
        if (data.hasVelocityXYZ) {
            state.velocity = Vector3f(data.velocityXYZ.velX, data.velocityXYZ.velY, data.velocityXYZ.velZ);
            state.have_velocity = true;
        }
    }

    // Publish to external systems
    if (data.hasAcceleration && data.hasRateOfTurn) {
        AP_ExternalAHRS::ins_data_message_t ins{};
        ins.accel = Vector3f(data.acceleration.accX, data.acceleration.accY, data.acceleration.accZ);
        ins.gyro = Vector3f(data.rateOfTurn.gyrX, data.rateOfTurn.gyrY, data.rateOfTurn.gyrZ);
        ins.temperature = data.hasTemperature ? data.temperature : -300;
        AP::ins().handle_external(ins);
    }

    if (data.hasMagneticField) {
        // Convert from arbitrary units to milliGauss for ArduPilot compass (approximate conversion), 1 a.u = 0.49Gauss = 490mG
        constexpr float A_U_TO_MILLIGAUSS = 490.0f; 
        AP_ExternalAHRS::mag_data_message_t mag{};
        mag.field = Vector3f(data.magneticField.magX * A_U_TO_MILLIGAUSS,
                        data.magneticField.magY * A_U_TO_MILLIGAUSS,
                        data.magneticField.magZ * A_U_TO_MILLIGAUSS);
        AP::compass().handle_external(mag);
    }

    if (data.hasBarometricPressure) {
        AP_ExternalAHRS::baro_data_message_t baro{};
        baro.instance = 0;
        baro.pressure_pa = data.barometricPressure;
        baro.temperature = data.hasTemperature ? data.temperature : 25;
        AP::baro().handle_external(baro);
    }

    // Publish GPS data using sensor fusion data with buffered GPS status.
    // Rate-limited to 10Hz: AP_GPS assumes a GPS rate of at most 20Hz when it derives
    // message timing from ms_tow, and at the 50Hz MTi output rate its lag check
    // intermittently reported the GPS as unhealthy. It also needs at least ~5Hz.
    // The schedule keeps a fixed phase so the average interval is exactly the period;
    // AP_GPS assumes a constant rate and flags a drifting GPS as lagged/unhealthy.
    const uint32_t gps_now_ms = AP_HAL::millis();
    if (data.hasLatLon && data.hasAltitudeEllipsoid && data.hasVelocityXYZ &&
        gps_status_initialized && have_geoid_separation &&
        gps_now_ms - last_gps_publish_ms >= GPS_PUBLISH_PERIOD_MS) {
        last_gps_publish_ms += GPS_PUBLISH_PERIOD_MS;
        if (gps_now_ms - last_gps_publish_ms >= GPS_PUBLISH_PERIOD_MS) {
            // first message, or after a gap: restart the schedule from now
            last_gps_publish_ms = gps_now_ms;
        }
        AP_ExternalAHRS::gps_data_message_t gps{};
        
        // Calculate GPS timing from high-rate UTC time data
        if (data.hasUtcTime) {
            calculate_gps_time_from_utc(data.utcTime.year, data.utcTime.month, data.utcTime.day,
                                      data.utcTime.hour, data.utcTime.minute, data.utcTime.second,
                                      data.utcTime.nanoseconds, gps.gps_week, gps.ms_tow);
        }
        
        // Always use the last known GPS status from GNSSPVTDATA - this is the TRUTH
        gps.fix_type = last_valid_fix_type;
        gps.satellites_in_view = last_satellites_in_view;
        gps.horizontal_pos_accuracy = last_horizontal_pos_accuracy;
        gps.vertical_pos_accuracy = last_vertical_pos_accuracy;
        gps.horizontal_vel_accuracy = last_horizontal_vel_accuracy;
        gps.hdop = last_hdop;
        gps.vdop = last_vdop;
        
        // Check if GPS status is stale (no GNSS updates for too long)
        uint32_t now = AP_HAL::millis();
        if (has_buffered_gnss_pvt && (now - last_gnss_pvt_update) > 15000) { // 15 second timeout
            // GNSS data is very stale, indicate degraded status but don't fake it
            gps.fix_type = AP_GPS_FixType::NONE;
            gps.satellites_in_view = 0;
            gps.hdop = GPS_UNKNOWN_DOP;
            gps.vdop = GPS_UNKNOWN_DOP;
        }

        // Use current high-rate position and velocity data from sensor fusion
        gps.longitude = data.latLon.longitude * 1e7;
        gps.latitude = data.latLon.latitude * 1e7;
        gps.msl_altitude = (data.altitudeEllipsoid - geoid_separation_m) * 100; // m to cm, AMSL
        
        gps.ned_vel_north = data.velocityXYZ.velX;
        gps.ned_vel_east = data.velocityXYZ.velY;
        gps.ned_vel_down = data.velocityXYZ.velZ;
        
        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance)) {
            AP::gps().handle_external(gps, instance);
        }
    }
}

void AP_ExternalAHRS_Xsens::calculate_gps_time_from_utc(uint16_t year, uint8_t month, uint8_t day, 
                                                       uint8_t hour, uint8_t minute, uint8_t second, 
                                                       int32_t nano, uint16_t &gps_week, uint32_t &ms_tow) const
{
    // GPS epoch: January 6, 1980, 00:00:00 UTC
    // Calculate days since GPS epoch
    
    // Days in each month (non-leap year)
    static const uint16_t days_in_month[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    
    // Calculate total days since GPS epoch (January 6, 1980)
    uint32_t total_days = 0;
    
    // Add days for complete years since 1980
    for (uint16_t y = 1980; y < year; y++) {
        bool is_leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
        total_days += is_leap ? 366 : 365;
    }
    
    // Add days for complete months in the current year
    for (uint8_t m = 1; m < month; m++) {
        total_days += days_in_month[m];
        // Add extra day for February in leap years
        if (m == 2 && year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) {
            total_days++;
        }
    }
    
    // Add days in the current month
    total_days += day - 1; // -1 because day 1 = 0 days elapsed
    
    // GPS epoch was January 6, 1980, so subtract 5 days (Jan 1-5, 1980)
    if (total_days >= 5) {
        total_days -= 5;
    } else {
        gps_week = 0;
        ms_tow = 0;
        return; // Date is before GPS epoch
    }
    
    // GPS time runs ahead of UTC by the leap seconds. AP_GPS expects the full
    // (not 1024-rolled-over) week number.
    const uint64_t gps_ms = (uint64_t(total_days) * 86400ULL +
                             hour * 3600U + minute * 60U + second) * 1000ULL +
                            nano / 1000000 +
                            GPS_LEAPSECONDS_MILLIS;
    gps_week = gps_ms / AP_MSEC_PER_WEEK;
    ms_tow = gps_ms % AP_MSEC_PER_WEEK;
}

// get_port to indicate SPI usage
int8_t AP_ExternalAHRS_Xsens::get_port(void) const
{
    if (interface_type == InterfaceType::SPI) {
        return -2; // Special value to indicate SPI
    }
    
    if (!uart) {
        return -1;
    }
    return port_num;
}

bool AP_ExternalAHRS_Xsens::healthy(void) const
{
    // Read the timestamps BEFORE millis(): this runs on the main thread while the
    // Xsens thread updates them, and a timestamp newer than 'now' would make
    // 'now - timestamp' wrap around to ~49 days
    const uint32_t ins_pkt_ms = last_ins_pkt;
    const uint32_t now = AP_HAL::millis();

    // Healthy means the MTi is delivering data. GNSS quality is reported separately
    // through AP_GPS and get_filter_status(), so losing GNSS does not make the
    // attitude solution unhealthy (same approach as the other ExternalAHRS backends).
    return device_state == DeviceState::RUNNING && now - ins_pkt_ms < 500;
}

bool AP_ExternalAHRS_Xsens::initialised(void) const
{
    return last_ins_pkt != 0;
}

bool AP_ExternalAHRS_Xsens::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Xsens unhealthy");
        return false;
    }
    
    if (device_state != DeviceState::RUNNING) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Xsens not running");
        return false;
    }

    if (!filter_valid()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Xsens filter not valid");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_Xsens::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    
    if (last_ins_pkt != 0 && healthy()) {
        status.flags.initalized = 1;
        // attitude is only valid once the MTi reports its filter as valid
        status.flags.attitude = filter_valid();
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;

        // Horizontal position and velocity are only valid while the MTi outputs a
        // position, its filter reports GNSS-aided mode AND there is a fresh 3D GNSS fix.
        // Without GNSS the MTi dead-reckons, which ArduPilot must not use for position
        // control.
        // (read the timestamps before millis(), see healthy())
        const uint32_t pvt_update_ms = last_gnss_pvt_update;
        const uint32_t pos_ms = last_pos_ms;
        const uint32_t now = AP_HAL::millis();
        const bool gnss_valid = has_buffered_gnss_pvt &&
                                now - pvt_update_ms < GNSS_PVT_TIMEOUT_MS &&
                                last_valid_fix_type >= AP_GPS_FixType::FIX_3D;
        const bool position_recent = pos_ms != 0 && now - pos_ms < 500;
        if (gnss_valid && position_recent && filter_gnss_aided()) {
            status.flags.horiz_vel = 1;
            status.flags.horiz_pos_rel = 1;
            status.flags.horiz_pos_abs = 1;
            status.flags.pred_horiz_pos_rel = 1;
            status.flags.pred_horiz_pos_abs = 1;
            status.flags.using_gps = 1;
        }
    }
}

// True when the MTi reports a valid orientation filter. Without StatusWord output
// (e.g. a custom output configuration) the filter is assumed valid.
bool AP_ExternalAHRS_Xsens::filter_valid() const
{
    return !have_status_word || (last_status_word & STATUS_FILTER_VALID) != 0;
}

// True when the MTi filter is in "with GNSS" mode, i.e. its position is GNSS aided
bool AP_ExternalAHRS_Xsens::filter_gnss_aided() const
{
    return !have_status_word ||
           (last_status_word & STATUS_FILTER_MODE_MASK) == STATUS_FILTER_MODE_WITH_GNSS;
}

// get_variances allows the EKF status report to be sent for External AHRS
bool AP_ExternalAHRS_Xsens::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = last_horizontal_vel_accuracy * vel_gate_scale;
    posVar = last_horizontal_pos_accuracy * pos_gate_scale;
    hgtVar = last_vertical_pos_accuracy * hgt_gate_scale;
    tasVar = 0;
    return true;
}

void AP_ExternalAHRS_Xsens::update()
{
    // Read the timestamps written by the Xsens thread BEFORE millis(), see healthy()
    const uint32_t ins_pkt_ms = last_ins_pkt;
    const uint32_t running_ms = running_since_ms;
    uint32_t now = AP_HAL::millis();

    // Watchdog check - if no data received for too long, trigger restart.
    // Also require 10s in RUNNING, so a restart after a data gap is not immediately
    // restarted again before the first new packet arrives.
    if (device_state == DeviceState::RUNNING &&
        now - ins_pkt_ms > 10000 &&
        now - running_ms > 10000) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Xsens: No data for 10s, restarting");
        set_device_state(DeviceState::ENTERING_CONFIG_MODE);
    }
    
    // Check for communication errors and reset if needed
    if (device_state == DeviceState::ERROR && now - state_timeout > 10000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xsens: Attempting recovery from prolonged error");
        set_device_state(DeviceState::ENTERING_CONFIG_MODE);
    }
}


// SPI initialization
bool AP_ExternalAHRS_Xsens::init_spi()
{
    spi_dev = hal.spi->get_device("xsens");
    
    if (!spi_dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Xsens: SPI device not found");
        return false;
    }
    
    // Configure SPI: 1MHz, Mode 3 (CPOL=1, CPHA=1)
    spi_dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    
    // Setup DRDY pin (GPIO 94 = PF3 on CUAV V6X)
#ifdef HAL_GPIO_XSENS_DRDY
    drdy_gpio_pin = HAL_GPIO_XSENS_DRDY;
    hal.gpio->pinMode(drdy_gpio_pin, HAL_GPIO_INPUT);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xsens: DRDY pin %d configured", drdy_gpio_pin);
#else
    drdy_gpio_pin = -1;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Xsens: DRDY pin not defined");
#endif
    
    // Perform software reset
    //software_reset();
    
    return true;
}

// SPI transfer implementation
void AP_ExternalAHRS_Xsens::spi_transfer(uint8_t opcode,
                                        uint8_t *data,
                                        uint16_t len,
                                        bool is_read)
{
    if (!spi_dev || len == 0) {
        return;
    }

    const uint16_t total_len = 4 + len;

    uint8_t tx[total_len];
    uint8_t rx[total_len];

    tx[0] = opcode;
    tx[1] = 0;
    tx[2] = 0;
    tx[3] = 0;

    if (!is_read) {
        memcpy(&tx[4], data, len);
    } else {
        memset(&tx[4], 0, len); 
    }

    spi_dev->get_semaphore()->take_blocking();
    hal.scheduler->delay_microseconds(4);

    spi_dev->transfer_fullduplex(tx, rx, total_len);

    hal.scheduler->delay_microseconds(4);
    spi_dev->get_semaphore()->give();

    if (is_read) {
        memcpy(data, &rx[4], len);
    }
}

// Read pipe status
void AP_ExternalAHRS_Xsens::read_pipe_status(uint16_t &notif_size, uint16_t &meas_size)
{
    uint8_t status[4] {};
    spi_transfer(XBUS_PIPE_STATUS, status, 4, true);
    
    notif_size = status[0] | (status[1] << 8);
    meas_size = status[2] | (status[3] << 8);
}

// Read pipe
void AP_ExternalAHRS_Xsens::read_pipe(uint8_t *buffer, uint16_t size, uint8_t pipe)
{
    spi_transfer(pipe, buffer, size, true);
}

// Create raw SPI message
uint16_t AP_ExternalAHRS_Xsens::create_raw_spi_message(uint8_t *dest, const uint8_t *msg)
{
    uint16_t len = get_payload_length(msg);
    // uint16_t total_len = (msg[3] == LENGTH_EXTENDER_BYTE) ? len + 7 : len + 5;
    
    dest[0] = XBUS_CONTROL_PIPE;
    dest[1] = 0;
    dest[2] = 0;
    dest[3] = 0;
    
    uint8_t checksum = 0;
    checksum -= XBUS_MASTERDEVICE;
    
    dest[4] = msg[2]; // MID
    checksum -= dest[4];
    
    uint16_t idx = 5;
    if (len < LENGTH_EXTENDER_BYTE) {
        dest[idx++] = len;
        checksum -= len;
    } else {
        dest[idx++] = LENGTH_EXTENDER_BYTE;
        checksum -= LENGTH_EXTENDER_BYTE;
        dest[idx++] = len >> 8;
        checksum -= (len >> 8);
        dest[idx++] = len & 0xFF;
        checksum -= (len & 0xFF);
    }
    
    const uint8_t* payload = get_const_pointer_to_payload(msg);
    for (uint16_t i = 0; i < len; i++) {
        dest[idx++] = payload[i];
        checksum -= payload[i];
    }
    
    dest[idx++] = checksum;
    return idx;
}

// Send SPI message
void AP_ExternalAHRS_Xsens::send_spi_message(const uint8_t *message, size_t length)
{
    uint8_t spi_buffer[256];
    uint16_t spi_len = create_raw_spi_message(spi_buffer, message);
    
    spi_transfer(XBUS_CONTROL_PIPE, spi_buffer, spi_len, false);
}

// Check DRDY
bool AP_ExternalAHRS_Xsens::check_drdy()
{
    if (drdy_gpio_pin < 0) {
        // If DRDY pin not configured, always return true to poll
        return true;
    }
    
    // DRDY is active HIGH
    return hal.gpio->read(drdy_gpio_pin) == 1;
}

void AP_ExternalAHRS_Xsens::software_reset()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xsens: Performing software reset");
    
    // Create reset message (XMID_Reset = 0x40)
    uint8_t reset_msg[5];
    create_message(reset_msg, XBUS_MASTERDEVICE, XsMessageId::Reset, 0);
    insert_checksum(reset_msg);
    
    // Send reset command
    write_message(reset_msg, 5);
    
    // Wait for device to reset and boot (typically ~100ms)
    hal.scheduler->delay(150);
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xsens: Software reset complete");
}


#endif // AP_EXTERNAL_AHRS_XSENS_ENABLED
