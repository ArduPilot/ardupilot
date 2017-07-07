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
 *
 * ALL INFORMATION REGARDING PROTOCOL WAS DERIVED FROM RPLIDAR DATASHEET:
 *
 * https://www.slamtec.com/en/Lidar
 * http://bucket.download.slamtec.com/63ac3f0d8c859d3a10e51c6b3285fcce25a47357/LR001_SLAMTEC_rplidar_protocol_v1.0_en.pdf
 *
 * Author: Steven Josefs, IAV GmbH
 *
 *
 */



#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_RPLidarA2.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

#define FTDI_DEBUG 0
#define RP_DEBUG_LEVEL 0

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
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);                // selected in MP at SerialConfig
    if (_uart != nullptr) {
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0));
    }
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
    if(!_initialised)
    {
        _initialised = initialise();    //returns true if everything initialized properly
    }


    // if LIDAR in known state
    if (_initialised) {
        get_readings();
    }

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > COMM_ACTIVITY_TIMEOUT_MS)) {
        set_status(AP_Proximity::Proximity_NoData);
#if RP_DEBUG_LEVEL > 0
        GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "LIDAR NO DATA");
#endif
    } else {
        set_status(AP_Proximity::Proximity_Good);
    }
}

// get maximum distance (in meters) of sensor
float AP_Proximity_RPLidarA2::distance_max() const
{
    return 16.0f;  //16m max range RPLIDAR2
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
#if RP_DEBUG_LEVEL > 0
        GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "LIDAR initialised");
#endif
        return true;
    }

    return true;
}

void AP_Proximity_RPLidarA2::reset_rplidar()
{
    if (_uart == nullptr) {
        return;
    }
#if FTDI_DEBUG
    _uart->write(0x32); //SP as padding for ftdi cable
    _uart->write(0x32);
    _uart->write(0x32); //SP as padding for ftdi cable
    _uart->write(0x32);
    _uart->write(0x32); //SP as padding for ftdi cable
    _uart->write(0x32);
    _uart->write(0x32); //SP as padding for ftdi cable
    _uart->write(0x32);
#endif
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_RESET};
    _uart->write(tx_buffer, 2);
    _resetted = true;                   ///< be aware of extra 63 bytes coming after reset containing FW information
#if RP_DEBUG_LEVEL > 0
    GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "LIDAR reset");
#endif
    hal.scheduler->delay(RESET_RPA2_WAIT_MS);                      ///< wait at least 8ms after sending new request
    //hal.scheduler->delay_microseconds(PROXIMITY_RPA2_TIMEOUT_MS * 1000);
    _last_reset_ms =  AP_HAL::millis();
    _rp_state = rp_resetted;

}

// initialise sector angles using user defined ignore areas, left same as SF40C
// TODO: adapt to RPLIDAR A2 measurements
void AP_Proximity_RPLidarA2::init_sectors()
{
    // use defaults if no ignore areas defined
    uint8_t ignore_area_count = get_ignore_area_count();
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
#if FTDI_DEBUG
    _uart->write(0x32); //SP as padding for ftdi cable
    _uart->write(0x32);
    _uart->write(0x32); //SP as padding for ftdi cable
    _uart->write(0x32);
    _uart->write(0x32); //SP as padding for ftdi cable
    _uart->write(0x32);
    _uart->write(0x32); //SP as padding for ftdi cable
    _uart->write(0x32);
#endif
    uint8_t tx_buffer[2] = {RPLIDAR_PREAMBLE, RPLIDAR_CMD_SCAN};
    _uart->write(tx_buffer, 2);
    _last_request_ms = AP_HAL::millis();
#if RP_DEBUG_LEVEL > 0
    GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "LIDAR SCAN MODE ACTIVATED");
#endif
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
    _rp_state = rp_health;
}

void AP_Proximity_RPLidarA2::get_readings()
{
    if (_uart == nullptr) {
        return;
    }
#if RP_DEBUG_LEVEL > 1
    GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "             CURRENT STATE: %d ", _rp_state);
#endif
    uint32_t nbytes = _uart->available();
    static uint16_t byte_count = 0;

    while (nbytes-- > 0) {

        uint8_t c = _uart->read();
#if RP_DEBUG_LEVEL > 1
        GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "UART READ %x <%c>", c, c); //show HEX values
#endif

        STATE:
        switch(_rp_state){

            case rp_resetted:
#if RP_DEBUG_LEVEL > 2
                GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "                  BYTE_COUNT %d", byte_count);
#endif
                if ((c == 0x52 || _information_data) && byte_count < 62)
                {
                    if(c == 0x52) {
                        _information_data = true;
                    }
                    _rp_systeminfo[byte_count] = c;
#if RP_DEBUG_LEVEL > 2
                    GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "_rp_systeminfo[%d]=%x",byte_count,_rp_systeminfo[byte_count]);
#endif
                    byte_count++;
                    break;
                }
                else
                {

                    if (_information_data) {
#if RP_DEBUG_LEVEL > 0
                        GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "GOT RPLIDAR INFORMATION");
#endif
                        _information_data = false;
                        byte_count = 0;

                        set_scan_mode();
                        break;
                    }

                    if (cnt>5) {
                        _rp_state = rp_unknown;
                        cnt=0;
                        break;
                    }
                    cnt++;
                    break;
                }
                break;

            case rp_responding:
#if RP_DEBUG_LEVEL > 1
                GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "RESPONDING");
#endif
                if(c == RPLIDAR_PREAMBLE || _descriptor_data) {
                    _descriptor_data = true;
                    _descriptor[byte_count] = c;
                    byte_count++;
                    //descriptor packet has 7 byte in total
                    if(byte_count == sizeof(_descriptor)) {
#if RP_DEBUG_LEVEL > 1
                        GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "LIDAR DESCRIPTOR CATCHED");
#endif
                        _response_type = ResponseType_Descriptor;
                        //identify the payload data after the descriptor
                        parse_response_descriptor();
                        byte_count = 0;

                    }
                } else {
                    _rp_state = rp_unknown;
                }
                break;

            case rp_measurements:
                if (_sync_error) {    //out of 5-byte sync mask -> catch new revolution
#if RP_DEBUG_LEVEL > 0
                    GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "       OUT OF SYNC");
#endif
                    //on first revolution bit 1 = 1, bit 2 = 0 of the first byte
                    if ((c & 0x03) == 0x01) {
                        _sync_error = 0;
#if RP_DEBUG_LEVEL > 0
                        GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "                  RESYNC");
#endif
                    } else {
                        if (AP_HAL::millis() - _last_distance_received_ms > RESYNC_TIMEOUT) reset_rplidar();
                        break;
                    }
                }
#if RP_DEBUG_LEVEL > 2
                GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "READ PAYLOAD");
#endif
                payload[byte_count] = c;
                byte_count++;

                if (byte_count == _payload_length) {
#if RP_DEBUG_LEVEL > 1
                    GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "LIDAR MEASUREMENT CATCHED");
#endif
                    parse_response_data();
                    byte_count = 0;

                }
                break;

            case rp_health:
#if RP_DEBUG_LEVEL > 0
                GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "state: HEALTH");
#endif
                break;

            case rp_unknown:
#if RP_DEBUG_LEVEL > 0
                GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "state: UNKNOWN");
#endif
                if (c == RPLIDAR_PREAMBLE) {
                    _rp_state = rp_responding;
                    goto STATE;
                    break;
                }
                cnt++;
                if (cnt>10) {
                    reset_rplidar();
                    _rp_state = rp_resetted;
                    cnt=0;
                }
                break;

            default:
#if RP_DEBUG_LEVEL > 0
                GCS_MAVLINK::send_text(MAV_SEVERITY_CRITICAL, "UNKNOWN LIDAR STATE");
#endif
                break;

        }

    }

}

void AP_Proximity_RPLidarA2::parse_response_descriptor()
{//check if descriptor packet is valid
    if (_descriptor[0] == RPLIDAR_PREAMBLE && _descriptor[1] == 0x5A) {

        if (_descriptor[2] == 0x05 && _descriptor[3] == 0x00 && _descriptor[4] == 0x00 && _descriptor[5] == 0x40 && _descriptor[6] == 0x81) {
            // payload is SCAN measurement data
            _payload_length = sizeof(payload.sensor_scan);
            static_assert(sizeof(payload.sensor_scan) == 5, "Unexpected payload.sensor_scan data structure size");
            _response_type = ResponseType_SCAN;
#if RP_DEBUG_LEVEL > 1
            GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "Measurement response detected");
#endif
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state = rp_measurements;
        }
        if (_descriptor[2] == 0x03 && _descriptor[3] == 0x00 && _descriptor[4] == 0x00 && _descriptor[5] == 0x00 && _descriptor[6] == 0x06) {
            // payload is health data
            _payload_length = sizeof(payload.sensor_health);
            static_assert(sizeof(payload.sensor_health) == 3, "Unexpected payload.sensor_health data structure size");
            _response_type = ResponseType_Health;
            _last_distance_received_ms = AP_HAL::millis();
            _rp_state= rp_health;
        }
        return;
    }
#if RP_DEBUG_LEVEL > 0
    GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "Invalid response descriptor");
#endif
    _rp_state = rp_unknown;

}

void AP_Proximity_RPLidarA2::parse_response_data()
{
    switch (_response_type){
        case ResponseType_SCAN:
#if RP_DEBUG_LEVEL > 1
            GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "UART %02x %02x%02x %02x%02x", payload[0], payload[2], payload[1], payload[4], payload[3]); //show HEX values
#endif
            //check if valid SCAN packet: a valid packet starts with startbits which are complementary plus a checkbit in byte+1
            if ((payload.sensor_scan.startbit == !payload.sensor_scan.not_startbit) && payload.sensor_scan.checkbit) {
                const float angle_deg = payload.sensor_scan.angle_q6/64.0f;
                const float distance_m = (payload.sensor_scan.distance_q2/4000.0f);
                //const float quality = payload.sensor_scan.quality;
#if RP_DEBUG_LEVEL > 1
                GCS_MAVLINK::send_text(MAV_SEVERITY_INFO, "                                       D%02.2f A%03.1f Q%02d", distance_m, angle_deg, quality);
#endif

                uint8_t sector;
                if (convert_angle_to_sector(angle_deg, sector)) {
                    _angle[sector] = angle_deg;
                    _distance[sector] = distance_m;
                    _distance_valid[sector] = true;
                    _last_distance_received_ms = AP_HAL::millis();
                    // update boundary used for avoidance
                    update_boundary_for_sector(sector);
                }
            } else {
                //not valid payload packet
#if RP_DEBUG_LEVEL > 0
                GCS_MAVLINK::send_text(MAV_SEVERITY_CRITICAL, "Invalid Payload");
#endif
                _sync_error++;

            }
            break;

        case ResponseType_Health:
            //health issue if status is "3" ->HW error
            if (payload.sensor_health.status == 3) {
#if RP_DEBUG_LEVEL > 0
                GCS_MAVLINK::send_text(MAV_SEVERITY_CRITICAL, "LIDAR Error");
#endif
            }
            break;

        default:
            /*no valid payload packets recognized: return payload data=0*/
#if RP_DEBUG_LEVEL > 0
            GCS_MAVLINK::send_text(MAV_SEVERITY_CRITICAL, "Unknown LIDAR packet");
#endif
            break;
    }

}
