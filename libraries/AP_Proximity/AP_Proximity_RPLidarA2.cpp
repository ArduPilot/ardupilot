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

#define RP_DEBUG_LEVEL 0

#if RP_DEBUG_LEVEL
  #include <GCS_MAVLink/GCS.h>
  #define Debug(level, fmt, args ...)  do { if (level <= RP_DEBUG_LEVEL) { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif

#define COMM_ACTIVITY_TIMEOUT_MS        200
#define RESET_RPA2_WAIT_MS              8
#define RESYNC_TIMEOUT                  5000

// Commands
//-----------------------------------------

// Commands without payload and response
#define RPLIDAR_PREAMBLE               0xA5
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40

// Commands without payload but have response
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
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0));
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
        Debug(3, "LIDAR NO DATA");
    } else {
        set_status(AP_Proximity::Proximity_Good);
        Debug(3, "LIDAR DATA OK");
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

bool AP_Proximity_RPLidarA2::initialise()
{
    // initialise sectors
    if (!_sector_initialised) {
        init_sectors();
        return false;
    }
    if (!_initialised) {
        reset_rplidar();            // set to a known state
        Debug(1, "LIDAR initialised");
        return true;
    }

    return true;
}

void AP_Proximity_RPLidarA2::reset_rplidar()
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    _uart->write(tx_buffer, 2);
    _resetted = true;   ///< be aware of extra 63 bytes coming after reset containing FW information
    Debug(1, "LIDAR reset");
    // To-Do: ensure delay of 8m after sending reset request
    _last_reset_ms =  AP_HAL::millis();
    _rp_state = rp_resetted;

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

// set Lidar into SCAN mode
void AP_Proximity_RPLidarA2::set_scan_mode()
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_SCAN};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
    Debug(1, "LIDAR SCAN MODE ACTIVATED");
    _rp_state = rp_responding;
}

// send request for sensor health
void AP_Proximity_RPLidarA2::send_request_for_health()                                    //not called yet
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_HEALTH};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
    Debug(1, "LIDAR HEALTH REQUEST ACTIVATED");
    _rp_state = rp_health;
}

// send request for sensor sampling rate
void AP_Proximity_RPLidarA2::send_request_for_rate()                                    //not called yet
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_GET_DEVICE_RATE};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
    Debug(1, "LIDAR RATE REQUEST ACTIVATED");
    _rp_state = rp_rate;
}


void AP_Proximity_RPLidarA2::get_readings()
{
    if (_uart == nullptr) {
        return;
    }

    Debug(3, "             CURRENT STATE: %d ", _rp_state);
    uint32_t nbytes = _uart->available();

    while (nbytes-- > 0) {

        uint8_t c = _uart->read();
        Debug(3, "UART READ %x <%c>", c, c); //show HEX values

        STATE:
        switch(_rp_state){

            case rp_resetted:
                Debug(3, "                  BYTE_COUNT %d", _byte_count);
                if ((c == 0x52 || _information_data) && _byte_count < 55/*62*/) {
                    if (c == 0x52) {
                        _information_data = true;
                    }
                    _rp_systeminfo[_byte_count] = c;
                    Debug(3, "_rp_systeminfo[%d]=%x",_byte_count,_rp_systeminfo[_byte_count]);
                    _byte_count++;
                    break;
                } else {

                    if (_information_data) {
                        Debug(1, "GOT RPLIDAR INFORMATION");
                        _information_data = false;
                        _byte_count = 0;
                        set_scan_mode();
                        break;
                    }

                    if (_cnt>5) {
                        _rp_state = rp_unknown;
                        _cnt=0;
                        break;
                    }
                    _cnt++;
                    break;
                }
                break;

            case rp_responding:
                Debug(2, "RESPONDING (%x)", c);
                if (c == RPLIDAR_PREAMBLE || _descriptor_data) {
//                    Debug(2, "_descriptor[%d] = %x", _byte_count, c);
                    _descriptor_data = true;
                    _descriptor[_byte_count] = c;
                    _byte_count++;
                    // descriptor packet has 7 byte in total
                    if (_byte_count == sizeof(_descriptor)) {
                        Debug(1,"LIDAR DESCRIPTOR CATCHED");
                        _response_type = ResponseType_Descriptor;
                        // identify the payload data after the descriptor
                        parse_response_descriptor();
                        _byte_count = 0;
                        _descriptor_data = false;
                    }
                } else {
                    _rp_state = rp_unknown;
                }
                break;

            case rp_measurements:
                if (_sync_error) {
                    // out of 5-byte sync mask -> catch new revolution
                    Debug(2, "       OUT OF SYNC");
                    // on first revolution bit 1 = 1, bit 2 = 0 of the first byte
                    if ((c & 0x03) == 0x01) {
                        _sync_error = 0;
                        Debug(2, "                  RESYNC");
                    } else {
                        if (AP_HAL::millis() - _last_distance_received_ms > RESYNC_TIMEOUT) {
                            reset_rplidar();
                        }
                        break;
                    }
                }
                Debug(3, "READ PAYLOAD");
                payload[_byte_count] = c;
                _byte_count++;

                if (_byte_count == _payload_length) {
                    Debug(2, "LIDAR MEASUREMENT CATCHED");
                    parse_response_data();
                    _byte_count = 0;

                    /*
                    // Send the request for health every once in a while
                    static uint16_t msgCounter = 0;
                    msgCounter++;
                    if( msgCounter == 1000 ){
                        send_request_for_health();
                    }
                    */
                }
                break;

            case rp_health:
                // Take some time for RPLidar to stop sending SCAN messages
                if( AP_HAL::millis() - _last_distance_received_ms > 10 ) {
                    Debug(2, "HEALTH - waits for response");
                    _rp_state = rp_responding;
                }
                break;

            case rp_rate:
                // Take some time for RPLidar to stop sending SCAN messages
                if( AP_HAL::millis() - _last_distance_received_ms > 10 ) {
                    Debug(2, "RATE - waits for response");
                    _rp_state = rp_responding;
                }
                break;

            case rp_unknown:
                Debug(1, "state: UNKNOWN");
                if (c == RPLIDAR_PREAMBLE) {
                    _rp_state = rp_responding;
                    goto STATE;
                    break;
                }
                _cnt++;
                if (_cnt>100) {
                    reset_rplidar();
                    _rp_state = rp_resetted;
                    _cnt=0;
                }
                break;

            default:
                Debug(1, "UNKNOWN LIDAR STATE");
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
            Debug(2, "Measurement response detected");
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state = rp_measurements;
        }
        if (_descriptor[2] == 0x03 && _descriptor[3] == 0x00 && _descriptor[4] == 0x00 && _descriptor[5] == 0x00 && _descriptor[6] == 0x06) {
            // payload is health data
            _payload_length = sizeof(payload.sensor_health);
            static_assert(sizeof(payload.sensor_health) == 3, "Unexpected payload.sensor_health data structure size");
            _response_type = ResponseType_HEALTH;
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state= rp_measurements;
        }
        if (_descriptor[2] == 0x04 && _descriptor[3] == 0x00 && _descriptor[4] == 0x00 && _descriptor[5] == 0x00 && _descriptor[6] == 0x15) {
            // payload is sample rate data
            _payload_length = sizeof(payload.sensor_health);
            static_assert(sizeof(payload.sensor_rate) == 4, "Unexpected payload.sensor_rate data structure size");
            _response_type = ResponseType_RATE;
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state= rp_measurements;
        }
        return;
    }
    Debug(1, "Invalid response descriptor (%x %x)", _descriptor[0], _descriptor[1]);
    _rp_state = rp_unknown;
}

void AP_Proximity_RPLidarA2::parse_response_data()
{
    switch (_response_type){
        case ResponseType_SCAN:
            Debug(3, "UART %02x %02x%02x %02x%02x", payload[0], payload[2], payload[1], payload[4], payload[3]); //show HEX values
            // check if valid SCAN packet: a valid packet starts with startbits which are complementary plus a checkbit in byte+1
            if ((payload.sensor_scan.startbit == !payload.sensor_scan.not_startbit) && payload.sensor_scan.checkbit) {
                const float angle_deg = payload.sensor_scan.angle_q6/64.0f;
                const float distance_m = (payload.sensor_scan.distance_q2/4000.0f);
#if RP_DEBUG_LEVEL >= 2
                const float quality = payload.sensor_scan.quality;
                Debug(3, "                                       D%02.2f A%03.1f Q%02d", distance_m, angle_deg, quality);
#endif
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
                // not valid payload packet
                Debug(1, "Invalid Payload");
                _sync_error++;
            }
            break;

        case ResponseType_HEALTH:
            Debug(1, "LIDAR health status: %d", payload.sensor_health.status);
            if (payload.sensor_health.error_code > 0) {
                Debug(1, "      error code: %d", payload.sensor_health.error_code);
            }
            break;

        case ResponseType_RATE:
            // calculate and print the sampling rate in [kHz]
            if (payload.sensor_rate.t_standard > 0) {
                Debug(1, "STANDARD SCAN sampling rate: %d", 1000/payload.sensor_rate.t_standard);
            }
            if (payload.sensor_rate.t_express > 0) {
                Debug(1, "EXPRESS SCAN sampling rate: %d", 1000/payload.sensor_rate.t_express);
            }
            break;

        default:
            // no valid payload packets recognized: return payload data=0
            Debug(1, "Unknown LIDAR packet");
            break;
    }
}
