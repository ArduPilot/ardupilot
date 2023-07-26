/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by:
 *  BlueMark Innovations BV, Roel Schiphorst
 *  Contributors: Tom Pittenger, Josh Henderson
 *  Parts of this code are based on/copied from the Open Drone ID project https://github.com/opendroneid/opendroneid-core-c
 *
 * The code has been tested with the BlueMark DroneBeacon MAVLink transponder running this command in the ArduPlane folder:
 * sim_vehicle.py --wipe-eeprom --console --map -A --serial1=uart:/dev/ttyUSB1:9600
 * (and a DroneBeacon MAVLink transponder connected to ttyUSB1)
 *
 * The Remote ID implementation expects a transponder that caches the received MAVLink messages from ArduPilot
 * and transmits them at the required intervals. So static messages are only sent once to the transponder.
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_OpenDroneID_config.h"

#ifndef AP_OPENDRONEID_ENABLED
// default to off. Enabled in hwdef.dat
#define AP_OPENDRONEID_ENABLED 0
#endif

#if AP_OPENDRONEID_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/Location.h>

#define ODID_ID_SIZE 20
#define ODID_STR_SIZE 23

#define ODID_MIN_DIR         0       // Minimum direction
#define ODID_MAX_DIR         360     // Maximum direction
#define ODID_INV_DIR         361     // Invalid direction
#define ODID_MIN_SPEED_H     0       // Minimum speed horizontal
#define ODID_MAX_SPEED_H     254.25f // Maximum speed horizontal
#define ODID_INV_SPEED_H     255     // Invalid speed horizontal
#define ODID_MIN_SPEED_V     (-62)   // Minimum speed vertical
#define ODID_MAX_SPEED_V     62      // Maximum speed vertical
#define ODID_INV_SPEED_V     63      // Invalid speed vertical
#define ODID_MIN_ALT         (-1000) // Minimum altitude
#define ODID_MAX_ALT         31767.5f// Maximum altitude
#define ODID_INV_ALT         ODID_MIN_ALT // Invalid altitude
#define ODID_MAX_TIMESTAMP   (60 * 60)
#define ODID_INV_TIMESTAMP   0xFFFF  // Invalid, No Value or Unknown Timestamp
#define ODID_MAX_AREA_RADIUS 2550
#define ODID_AREA_COUNT_MIN  1
#define ODID_AREA_COUNT_MAX  65000

class AP_DroneCAN;

class AP_OpenDroneID
{
public:
    AP_OpenDroneID();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OpenDroneID);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    void init();
    bool pre_arm_check(char* failmsg, uint8_t failmsg_len);
    void update();

    // send pending dronecan messages
    void dronecan_send(AP_DroneCAN *);

    // handle a message from the GCS
    void handle_msg(mavlink_channel_t chan, const mavlink_message_t &msg);

    bool enabled(void) const {
        return _enable != 0;
    }

    void set_arm_status(mavlink_open_drone_id_arm_status_t &status);

    void set_basic_id();

    void get_persistent_params(ExpandingString &str) const;

    void load_UAS_ID_from_persistent_memory();

    // get singleton instance
    static AP_OpenDroneID *get_singleton()
    {
        return _singleton;
    }
private:
    static AP_OpenDroneID *_singleton;
    bool _initialised;
    // parameters
    AP_Int8  _enable;
    AP_Float _baro_accuracy;    // Vertical accuracy of the barometer when installed
    AP_Int16 _options;
    AP_Int8  _mav_port;
    AP_Int8  _can_driver;

    char ua_type[3];
    char id_type[3];
    size_t id_len;
    char id_str[21];

    enum Options : int16_t {
        EnforceArming     = (1U << 0U),
        AllowNonGPSPosition = (1U << 1U),
        LockUASIDOnFirstBasicIDRx = (1U << 2U),
    };

    // check if an option is set
    bool option_enabled(const Options option) const
    {
        return (uint8_t(_options.get()) & uint8_t(option)) != 0;
    }

    mavlink_channel_t _chan; // MAVLink channel that communicates with the Remote ID Transceiver
    const mavlink_channel_t MAV_CHAN_INVALID = mavlink_channel_t(255U);
    uint32_t _last_send_location_ms;
    uint32_t _last_send_system_update_ms;
    uint32_t _last_send_static_messages_ms;
    const uint32_t _mavlink_dynamic_period_ms = 1000; //how often are mavlink dynamic messages sent in ms. E.g. 1000 = 1 Hz
    const uint32_t _mavlink_static_period_ms = 3000; //how often are mavlink static messages sent in ms

    bool     _have_height_above_takeoff;
    Location _takeoff_location;
    bool _was_armed;

    // packets ready to be sent, updated with semaphore held
    HAL_Semaphore _sem;
    mavlink_open_drone_id_location_t pkt_location;
    mavlink_open_drone_id_basic_id_t pkt_basic_id;
    mavlink_open_drone_id_system_t pkt_system;
    mavlink_open_drone_id_self_id_t pkt_self_id;
    mavlink_open_drone_id_operator_id_t pkt_operator_id;

    // last time we got a SYSTEM message
    uint32_t last_system_ms;

    // last time we got a SYSTEM_UPDATE message
    uint32_t last_system_update_ms;

    // arm status from the transmitter
    mavlink_open_drone_id_arm_status_t arm_status;
    uint32_t last_arm_status_ms;

    // last time we sent a lost transmitter message
    uint32_t last_lost_tx_ms;

    // last time we sent a lost operator location notice
    uint32_t last_lost_operator_msg_ms;
    
    // transmit functions to manually send a static MAVLink message
    void send_dynamic_out();
    void send_static_out();
    void send_basic_id_message();
    void send_system_message();
    void send_system_update_message();
    void send_self_id_message();
    void send_operator_id_message();
    void send_location_message();
    enum next_msg : uint8_t {
        NEXT_MSG_BASIC_ID = 0,
        NEXT_MSG_SYSTEM,
        NEXT_MSG_SELF_ID,
        NEXT_MSG_OPERATOR_ID,
        NEXT_MSG_ENUM_END
    } next_msg_to_send;
    uint32_t last_msg_send_ms;

    // helper functions
    MAV_ODID_HOR_ACC create_enum_horizontal_accuracy(float Accuracy) const;
    MAV_ODID_VER_ACC create_enum_vertical_accuracy(float Accuracy) const;
    MAV_ODID_SPEED_ACC create_enum_speed_accuracy(float Accuracy) const;
    MAV_ODID_TIME_ACC create_enum_timestamp_accuracy(float Accuracy) const;
    uint16_t create_direction(uint16_t direction) const;
    uint16_t create_speed_horizontal(uint16_t speed) const;
    int16_t create_speed_vertical(int16_t speed) const;
    float create_altitude(float altitude) const;
    float create_location_timestamp(float timestamp) const;

    // mask of what UAVCAN drivers need to send each packet
    const uint8_t dronecan_send_all = (1U<<HAL_MAX_CAN_PROTOCOL_DRIVERS)-1;
    uint8_t driver_mask;
    uint8_t need_send_location;
    uint8_t need_send_basic_id;
    uint8_t need_send_system;
    uint8_t need_send_self_id;
    uint8_t need_send_operator_id;

    uint8_t dronecan_done_init;
    uint8_t dronecan_init_failed;
    void dronecan_init(AP_DroneCAN *uavcan);
    void dronecan_send_location(AP_DroneCAN *uavcan);
    void dronecan_send_basic_id(AP_DroneCAN *uavcan);
    void dronecan_send_system(AP_DroneCAN *uavcan);
    void dronecan_send_self_id(AP_DroneCAN *uavcan);
    void dronecan_send_operator_id(AP_DroneCAN *uavcan);
};

namespace AP
{
AP_OpenDroneID &opendroneid();
};

#endif // AP_OPENDRONEID_ENABLED
