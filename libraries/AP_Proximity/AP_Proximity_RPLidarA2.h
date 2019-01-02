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


#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_HAL/AP_HAL.h>                   ///< for UARTDriver


class AP_Proximity_RPLidarA2 : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_RPLidarA2(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

private:
    enum rp_state {
            rp_unknown = 0,
            rp_resetted,
            rp_responding,
            rp_measurements,
            rp_health,
            rp_rate
        };

    enum ResponseType {
        ResponseType_Descriptor = 0,
        ResponseType_SCAN,
        ResponseType_EXPRESS,
        ResponseType_HEALTH,
        ResponseType_RATE
    };

    // initialise sensor (returns true if sensor is successfully initialised)
    bool initialise();
    void init_sectors();
    void set_scan_mode();

    // send request for something from sensor
    void send_request_for_health();
    void send_request_for_rate();
    void parse_response_data();
    void parse_response_descriptor();
    void get_readings();
    void reset_rplidar();

    // reply related variables
    AP_HAL::UARTDriver *_uart;
    uint8_t _descriptor[7];
    char _rp_systeminfo[63];
    bool _descriptor_data;
    bool _information_data;
    bool _resetted;
    bool _initialised;
    bool _sector_initialised;

    uint8_t _payload_length;
    uint8_t _cnt;
    uint8_t _sync_error ;
    uint16_t _byte_count;

    // request related variables
    enum ResponseType _response_type;         ///< response from the lidar
    enum rp_state _rp_state;
    uint8_t   _last_sector;                   ///< last sector requested
    uint32_t  _last_request_ms;               ///< system time of last request
    uint32_t  _last_distance_received_ms;     ///< system time of last distance measurement received from sensor
    uint32_t  _last_reset_ms;

    // sector related variables
    float _angle_deg_last;
    float _distance_m_last;

    struct PACKED _sensor_scan {
        uint8_t startbit      : 1;            ///< on the first revolution 1 else 0
        uint8_t not_startbit  : 1;            ///< complementary to startbit
        uint8_t quality       : 6;            ///< Related the reflected laser pulse strength
        uint8_t checkbit      : 1;            ///< always set to 1
        uint16_t angle_q6     : 15;           ///< Actual heading = angle_q6/64.0 Degree
        uint16_t distance_q2  : 16;           ///< Actual Distance = distance_q2/4.0 mm
    };

    struct PACKED _sensor_health {
        uint8_t status;                       ///< status definition: 0 good, 1 warning, 2 error
        uint16_t error_code;                  ///< the related error code
    };

    struct PACKED _sensor_rate {
        uint16_t t_standard;                  ///< The time used when RPLIDAR takes a single laser ranging in uS (valid for SCAN mode)
        uint16_t t_express;                   ///< The time used when RPLIDAR takes a single laser ranging in uS (valid for EXPRESS_SCAN mode)
    };

    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        _sensor_scan sensor_scan;
        _sensor_health sensor_health;
        _sensor_rate sensor_rate;
    } payload;
};
