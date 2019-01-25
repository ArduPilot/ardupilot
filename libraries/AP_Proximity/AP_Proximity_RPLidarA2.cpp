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
 * ArduPilot device driver for SLAMTEC RPLIDAR A2 (16m range version)
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM RPLIDAR DATASHEET:
 *
 * https://www.slamtec.com/en/Lidar
 * http://bucket.download.slamtec.com/63ac3f0d8c859d3a10e51c6b3285fcce25a47357/LR001_SLAMTEC_rplidar_protocol_v1.0_en.pdf
 *
 * Author: Steven Josefs, IAV GmbH
 * Based on the LightWare SF40C ArduPilot device driver from Randy Mackay
 *
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_RPLidarA2.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>



// Debug settings
#define RP_DEBUG_LEVEL          0       /* Choose level of verbosity (0 to disable everything)*/
#define RP_DEBUG_ADD_TIMESTAMP  0       /* Choose to append timestamp to the debug message (useful but adds more time)*/
#define RP_DEBUG_GCS_ENABLED    1       /* Enable debugging to GCS */
#define RP_DEBUG_UART_ENABLED   1       /* Enable debugging to UART console */

// Debug macros
#if RP_DEBUG_LEVEL
    #define DST_GCS       0x01
    #define DST_UART      0x02
    #define LVL_DISABLED  0
    #define LVL_1         1
    #define LVL_2         2
    #define LVL_3         3

    #define Debug(level, dest, fmt, args ...) do { if ((level <= RP_DEBUG_LEVEL)&&(level > 0)) {dbg_gcs(fmt, ## args); dbg_uart(fmt, ## args);} } while (0);
    #if RP_DEBUG_GCS_ENABLED
        #if RP_DEBUG_ADD_TIMESTAMP
            #define dbg_gcs(fmt, args ...) gcs().send_text(MAV_SEVERITY_INFO, "%dms A2: " fmt, AP_HAL::millis(), ## args);
        #else
            #define dbg_gcs(fmt, args ...) gcs().send_text(MAV_SEVERITY_INFO, "A2: " fmt, ## args);
        #endif
    #else
        #define dbg_gcs(fmt, args ...)
    #endif
    #if RP_DEBUG_UART_ENABLED
        #if RP_DEBUG_ADD_TIMESTAMP
            #define dbg_uart(fmt, args ...)  printf( "%dms A2: " fmt "\n", AP_HAL::millis(), ## args);
        #else
            #define dbg_uart(fmt, args ...)     printf( "A2: " fmt "\n", ## args);
        #endif
    #else
        #define dbg_uart(fmt, args ...)
    #endif
#else
    #define Debug(level, dest, fmt, args ...)
#endif


// Operational settings
//-----------------------------------------
#define COMM_ACTIVITY_TIMEOUT_MS        200
#define RESET_RPA2_WAIT_MS              20
#define RESYNC_TIMEOUT                  5000
#define RESET_TIMEOUT                   1000
#define MIN_SCAN_QUALITY                10
#define RX_BUFFER_SIZE                  1024

// Commands
//-----------------------------------------
#define RPLIDAR_PREAMBLE               0xA5

// Commands without payload and response
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_RESET              0x40     // Be aware of an undocumented device information message that comes afterwards

// Commands without payload but have response
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21

#define RPLIDAR_CMD_GET_DEVICE_INFO    0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH  0x52
#define RPLIDAR_CMD_GET_DEVICE_RATE    0x59

// Commands with payload and have response
#define RPLIDAR_CMD_EXPRESS_SCAN       0x82

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
 */
AP_Proximity_RPLidarA2::AP_Proximity_RPLidarA2(AP_Proximity &_frontend,
                                               AP_Proximity::Proximity_State &_state,
                                               AP_SerialManager &serial_manager) :
                                               AP_Proximity_Backend(_frontend, _state)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);
    if (_uart != nullptr) {
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0), RX_BUFFER_SIZE, 256);
    }
    _cnt = 0 ;
    _sync_error = 0 ;
    _byte_count = 0;
}

// detect if a RPLidarA2 proximity sensor is connected by looking for a configured serial port
bool AP_Proximity_RPLidarA2::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0) != nullptr;
}

// update the _rp_state of the sensor
void AP_Proximity_RPLidarA2::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // read parameters
    if (_forward_direction != frontend.get_yaw_correction(state.instance)) {
        Debug(LVL_1,DST_GCS|DST_UART, "new forward direction received (%d io %d)", frontend.get_yaw_correction(state.instance), _forward_direction);
        _forward_direction = frontend.get_yaw_correction(state.instance);
    }
    // initialise sensor if necessary
    if (!_initialised) {
        _initialised = initialise();    //returns true if everything initialized properly
    }

    // if LIDAR in known state
    if (_initialised) {
        get_readings();
    }

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > COMM_ACTIVITY_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
        Debug(LVL_3,DST_GCS, "no data");
    } else {
        set_status(AP_Proximity::Proximity_Good);
        Debug(LVL_3,DST_GCS, "data ok");
    }
}

// get maximum distance (in meters) of sensor
float AP_Proximity_RPLidarA2::distance_max() const
{
    return 16.0f;  //16m max range RPLIDAR2, if you want to support the 8m version this is the only line to change
}

// get minimum distance (in meters) of sensor
float AP_Proximity_RPLidarA2::distance_min() const
{
    return 0.20f;  //20cm min range RPLIDAR2
}

// initialise sensor (returns true if sensor is succesfully initialised)
bool AP_Proximity_RPLidarA2::initialise()
{
    // initialise sectors
    if (!_sector_initialised) {
        init_sectors();
        return false;
    }

    // reset the device
    if (!_initialised) {
        send_request_for_reset();            // set to a known state
    }

    Debug(LVL_1,DST_GCS|DST_UART, "RPLidar initialized. Max range: %f", distance_max());

    return true;
}

// initialise sector angles using user defined ignore areas, left same as SF40C
void AP_Proximity_RPLidarA2::init_sectors()
{
    // use defaults if no ignore areas defined
    const uint8_t ignore_area_count = get_ignore_area_count();
    if (ignore_area_count == 0) {
        _sector_initialised = true;
        return;
    }

    uint8_t sector = 0;
    for (uint8_t i=0; i<ignore_area_count; i++) {

        // get ignore area info
        uint16_t ign_area_angle;
        uint8_t ign_area_width;
        if (get_ignore_area(i, ign_area_angle, ign_area_width)) {

            // calculate how many degrees of space we have between this end of this ignore area and the start of the end
            int16_t start_angle, end_angle;
            get_next_ignore_start_or_end(1, ign_area_angle, start_angle);
            get_next_ignore_start_or_end(0, start_angle, end_angle);
            int16_t degrees_to_fill = wrap_360(end_angle - start_angle);

            // divide up the area into sectors
            while ((degrees_to_fill > 0) && (sector < PROXIMITY_SECTORS_MAX)) {
                uint16_t sector_size;
                if (degrees_to_fill >= 90) {
                    // set sector to maximum of 45 degrees
                    sector_size = 45;
                } else if (degrees_to_fill > 45) {
                    // use half the remaining area to optimise size of this sector and the next
                    sector_size = degrees_to_fill / 2.0f;
                } else  {
                    // 45 degrees or less are left so put it all into the next sector
                    sector_size = degrees_to_fill;
                }
                // record the sector middle and width
                _sector_middle_deg[sector] = wrap_360(start_angle + sector_size / 2.0f);
                _sector_width_deg[sector] = sector_size;

                // move onto next sector
                start_angle += sector_size;
                sector++;
                degrees_to_fill -= sector_size;
            }
        }
    }

    // set num sectors
    _num_sectors = sector;

    // re-initialise boundary because sector locations have changed
    init_boundary();

    // record success
    _sector_initialised = true;
}

void AP_Proximity_RPLidarA2::send_request_for_reset()
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    _uart->write(tx_buffer, 2);
    Debug(LVL_1,DST_GCS, "sent reset request");
    _reset = true;           // Lock the lidar for the time of reseting
    _last_request_ms =  AP_HAL::millis();
    _rp_state = rp_reset;
}

// set Lidar into SCAN mode
void AP_Proximity_RPLidarA2::send_request_for_scan()
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_SCAN};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
    Debug(LVL_1,DST_GCS|DST_UART, "sent scan request");
    _rp_state = rp_responding;
}

// set Lidar into FORCE SCAN mode (scanning independently of the real speed)
void AP_Proximity_RPLidarA2::send_request_for_force_scan()
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_FORCE_SCAN};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
    Debug(LVL_1,DST_GCS|DST_UART, "sent force scan request");
    _rp_state = rp_responding;
}

// send request for sensor health
void AP_Proximity_RPLidarA2::send_request_for_health()
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_HEALTH};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
    Debug(LVL_1,DST_GCS|DST_UART, "sent health request");
    _rp_state = rp_responding;
}

// send request for sensor sampling rate
void AP_Proximity_RPLidarA2::send_request_for_rate()
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_RATE};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
    Debug(LVL_1,DST_GCS|DST_UART, "sent rate request");
    _rp_state = rp_responding;
}

void AP_Proximity_RPLidarA2::get_readings()
{
    if (_uart == nullptr) {
        return;
    }

    static uint8_t sync_buff[2] = {0};

    uint32_t nbytes = _uart->available();

    Debug(LVL_3,DST_GCS, "current state: %d ", _rp_state);
    Debug(LVL_3,DST_GCS, "nbytes: %d", nbytes);
    if(nbytes == RX_BUFFER_SIZE) {
        Debug(LVL_1,DST_GCS|DST_UART, "UART BUFF FULL (%d)!", nbytes);
    }

    /* After reset: wait for the welcome message to finish and take the next action.
     * The message is not documented anywhere and its length varies in different RP models,
     * thus we don't want to base on its content and use a predefined time instead.
     * The time is counted since receiving the first character of a welcome message. */
    if (_rp_state == rp_reset)
    {
        // Do if the welcome message is being transmitted
        if (_information_data == true) {
            // Wait at least 10ms for the welcome message to finish
            if (AP_HAL::millis() - _last_distance_received_ms > 10) {
                Debug(LVL_1,DST_GCS|DST_UART, "reset finished");
                _information_data = false;
                // Check health of the lidar after reset before proceeding
                send_request_for_health();
            }
        } else {
            // Or reset again after timeout
            if (AP_HAL::millis() - _last_request_ms > RESET_TIMEOUT) {
                Debug(LVL_1,DST_UART, "rp_reset timeout! (%d - %d ms)", AP_HAL::millis(),_last_request_ms);
                send_request_for_reset();
            }
        }
    }

    while (nbytes-- > 0) {
        uint8_t c = _uart->read();

        STATE:
        switch(_rp_state){

            case rp_reset:
                // Register the time of a welcome message (wait at least one call to this function to make sure there's no outdated data in the buffer)
                if (AP_HAL::millis() - _last_request_ms > RESET_RPA2_WAIT_MS && _information_data == false) {
                    Debug(LVL_1,DST_GCS|DST_UART, "welcome msg start");
                    _information_data = true;
                    _last_distance_received_ms = AP_HAL::millis();
                }
                break;

            case rp_responding:
                Debug(LVL_3,DST_GCS, "state: RESPONDING (%x)", c);
                if (c == RPLIDAR_PREAMBLE || _descriptor_data) {
                    Debug(LVL_3,DST_GCS, "_descriptor[%d] = %x", _byte_count, c);
                    _descriptor_data = true;
                    _descriptor[_byte_count] = c;
                    _byte_count++;
                    // descriptor packet has 7 bytes in total
                    if (_byte_count == sizeof(_descriptor)) {
                        Debug(LVL_3,DST_GCS,"descriptor catched");
                        _response_type = ResponseType_Descriptor;
                        // identify the payload data after the descriptor
                        parse_response_descriptor();
                        _byte_count = 0;
                        _descriptor_data = false;
                    }
                } else {
                    Debug(LVL_1,DST_UART, "invalid preamble (%x)", c);
                    _rp_state = rp_unknown;
                }
                break;

            case rp_measurements:
                if (_sync_error) {
                    // out of 5-byte sync mask -> catch a new revolution:
                    // - bit 0 = 1 and bit 1 = 0 of the correct data response that starts a new scan
                    // - bit 0 = 1 of the second byte (checkbit) to confirm it's not a coincidence
                    sync_buff[0] = sync_buff[1];
                    sync_buff[1] = c;
                    if (((sync_buff[0] & 0x03) == 0x01) && ((sync_buff[1] & 0x01) == 0x01)) {
                        Debug(LVL_1,DST_UART|DST_GCS, "resynced! (%02x %02x)", sync_buff[1], sync_buff[0]);
                        // Accept the correct bytes and continue to normal operation
                        _sync_error = 0;
                        payload[0] = sync_buff[0];
                        _byte_count = 1;
                        sync_buff[0] = 0;
                        sync_buff[1] = 0;
                    } else {
                        // Search on until valid bytes are received or timeout elapses
                        if (AP_HAL::millis() - _last_distance_received_ms > RESYNC_TIMEOUT) {
                            send_request_for_reset();
                        }
                        break;
                    }
                }
                Debug(LVL_3,DST_GCS, "%02x", c);
                payload[_byte_count] = c;
                _byte_count++;

                if (_byte_count == _payload_length) {
                    Debug(LVL_3,DST_GCS, "measurement catched");
                    parse_response_data();
                    _byte_count = 0;
                }
                break;

            case rp_unknown:
                if (c == RPLIDAR_PREAMBLE) {
                    _rp_state = rp_responding;
                    goto STATE;
                    break;
                }
                _cnt++;
                if (_cnt>10) {
                    Debug(LVL_1,DST_UART, "state: UNKNOWN");
                    send_request_for_reset();
                    _cnt=0;
                }
                break;

            default:
                Debug(LVL_1,DST_GCS|DST_UART, "invalid state");
                break;
        }
    }
}

void AP_Proximity_RPLidarA2::parse_response_descriptor()
{
    // check if descriptor packet is valid
    if (_descriptor[0] == RPLIDAR_PREAMBLE && _descriptor[1] == 0x5A) {

        if (_descriptor[2] == 0x05 && _descriptor[3] == 0x00 && _descriptor[4] == 0x00 && _descriptor[5] == 0x40 && _descriptor[6] == 0x81) {
            // payload is SCAN measurement data
            _payload_length = sizeof(payload.sensor_scan);
            static_assert(sizeof(payload.sensor_scan) == 5, "Unexpected payload.sensor_scan data structure size");
            _response_type = ResponseType_SCAN;
            Debug(LVL_3,DST_GCS, "Measurement response detected");
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state = rp_measurements;
        }
        if (_descriptor[2] == 0x03 && _descriptor[3] == 0x00 && _descriptor[4] == 0x00 && _descriptor[5] == 0x00 && _descriptor[6] == 0x06) {
            // payload is HEALTH data
            _payload_length = sizeof(payload.sensor_health);
            static_assert(sizeof(payload.sensor_health) == 3, "Unexpected payload.sensor_health data structure size");
            _response_type = ResponseType_HEALTH;
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state= rp_measurements;
        }
        if (_descriptor[2] == 0x04 && _descriptor[3] == 0x00 && _descriptor[4] == 0x00 && _descriptor[5] == 0x00 && _descriptor[6] == 0x15) {
            // payload is SAMPLE RATE data
            _payload_length = sizeof(payload.sensor_health);
            static_assert(sizeof(payload.sensor_rate) == 4, "Unexpected payload.sensor_rate data structure size");
            _response_type = ResponseType_RATE;
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state= rp_measurements;
        }
        return;
    }
    Debug(LVL_1,DST_UART, "Invalid response descriptor (%x %x)", _descriptor[0], _descriptor[1]);
    _rp_state = rp_unknown;
}

void AP_Proximity_RPLidarA2::parse_response_data()
{
    switch (_response_type){
        case ResponseType_SCAN:
            Debug(LVL_3,DST_GCS, "%02x %02x %02x %02x %02x", payload[0], payload[1], payload[2], payload[3], payload[4]); //show HEX values

            // check if valid SCAN packet: a valid packet starts with startbits which are complementary plus a checkbit in byte+1
            if ((payload.sensor_scan.startbit == !payload.sensor_scan.not_startbit) && payload.sensor_scan.checkbit) {
                if (payload.sensor_scan.quality > MIN_SCAN_QUALITY) {
                    const float angle_deg = wrap_360(payload.sensor_scan.angle_q6/64.0f - (float)_forward_direction);
                    const float distance_m = (payload.sensor_scan.distance_q2/4000.0f);
                    _last_distance_received_ms = AP_HAL::millis();
                    uint8_t sector;
                    if (convert_angle_to_sector(angle_deg, sector)) {
                        if (distance_m > distance_min()) {
                            if (_last_sector == sector) {
                                if (_distance_m_last > distance_m) {
                                    _distance_m_last = distance_m;
                                    _angle_deg_last  = angle_deg;
                                }
                            } else {
                                // a new sector started, the previous one can be updated now
                                Debug(LVL_3,DST_GCS, "D%2.1f A%3.1f", _distance_m_last, _angle_deg_last);
                                _angle[_last_sector] = _angle_deg_last;
                                _distance[_last_sector] = _distance_m_last;
                                _distance_valid[_last_sector] = true;
                                // update boundary used for avoidance
                                update_boundary_for_sector(_last_sector);
                                // initialize the new sector
                                _last_sector     = sector;
                                _distance_m_last = distance_m;
                                _angle_deg_last  = angle_deg;
                            }
                        } else {
                            _distance_valid[sector] = false;
                        }
                    }
                } else {
                    Debug(LVL_3,DST_GCS, "low Q (%d)!", payload.sensor_scan.quality);
                }
            } else {
                // not valid payload packet
                Debug(LVL_1,DST_UART, "invalid payload! (%02x)", payload[0]);
                Debug(LVL_3,DST_GCS, "invalid payload! (%02x %02x %02x %02x %02x)", payload[0], payload[1], payload[2], payload[3], payload[4]);
                _sync_error++;
            }
            break;

        case ResponseType_HEALTH:
            Debug(LVL_1,DST_GCS|DST_UART, "health status: %d", payload.sensor_health.status);
            if (payload.sensor_health.error_code > 0) {
                // Reset lidar on error
                send_request_for_reset();
                Debug(LVL_1,DST_GCS|DST_UART, " error code: %d", payload.sensor_health.error_code);
            } else {
                // Start scanning if no error occurred
                send_request_for_scan();
            }
            break;

        case ResponseType_RATE:
            // calculate and print the sampling rate in [kHz]
            if (payload.sensor_rate.t_standard > 0) {
                Debug(LVL_1,DST_GCS|DST_UART, "STANDARD SCAN sampling rate: %d", 1000/payload.sensor_rate.t_standard);
            }
            if (payload.sensor_rate.t_express > 0) {
                Debug(LVL_1,DST_GCS|DST_UART, "EXPRESS SCAN sampling rate: %d", 1000/payload.sensor_rate.t_express);
            }
            break;

        default:
            // no valid payload packets recognized: return payload data=0
            Debug(LVL_1,DST_GCS, "Unknown LIDAR packet");
            break;
    }
}
