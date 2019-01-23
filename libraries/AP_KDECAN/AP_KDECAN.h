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
 * AP_KDECAN.h
 *
 *      Author: Francisco Ferreira
 */
 
#pragma once

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>

#include <AP_Param/AP_Param.h>

#include <atomic>

// there are 12 motor functions in SRV_Channel but CAN driver can't keep up
#define KDECAN_MAX_NUM_ESCS 8

class AP_KDECAN : public AP_HAL::CANProtocol {
public:
    AP_KDECAN();
    
    /* Do not allow copies */
    AP_KDECAN(const AP_KDECAN &other) = delete;
    AP_KDECAN &operator=(const AP_KDECAN&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // Return KDECAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_KDECAN *get_kdecan(uint8_t driver_index);

    void init(uint8_t driver_index, bool enable_filters) override;

    // called from SRV_Channels
    void update();
    
    // check that arming can happen
    bool pre_arm_check(const char* &reason);

    // send MAVLink telemetry packets
    void send_mavlink(uint8_t chan);

    // caller checks that vehicle isn't armed
    // start_stop: true to start, false to stop
    bool run_enumeration(bool start_stop);

private:
    void loop();

    bool _initialized;
    char _thread_name[9];
    uint8_t _driver_index;
    uavcan::ICanDriver* _can_driver;

    AP_Int8 _num_poles;

    // ESC detected information
    uint16_t _esc_present_bitmask;
    uint8_t _esc_max_node_id;

    // enumeration
    HAL_Semaphore _enum_sem;
    enum enumeration_state_t : uint8_t {
        ENUMERATION_STOPPED,
        ENUMERATION_START,
        ENUMERATION_STOP,
        ENUMERATION_RUNNING
    } _enumeration_state = ENUMERATION_STOPPED;

    // PWM output
    HAL_Semaphore _rc_out_sem;
    std::atomic<bool> _new_output;
    uint16_t _scaled_output[KDECAN_MAX_NUM_ESCS];

    // telemetry input
    HAL_Semaphore _telem_sem;
    struct telemetry_info_t {
        uint64_t time;
        uint16_t voltage;
        uint16_t current;
        uint16_t rpm;
        uint8_t temp;
        bool new_data;
    } _telemetry[KDECAN_MAX_NUM_ESCS];


    union frame_id_t {
        struct {
            uint8_t object_address;
            uint8_t destination_id;
            uint8_t source_id;
            uint8_t priority:5;
            uint8_t unused:3;
        };
        uint32_t value;
    };
    
    static const uint8_t AUTOPILOT_NODE_ID = 0;
    static const uint8_t BROADCAST_NODE_ID = 1;
    static const uint8_t ESC_NODE_ID_FIRST = 2;

    static const uint8_t ESC_INFO_OBJ_ADDR = 0;
    static const uint8_t SET_PWM_OBJ_ADDR = 1;
    static const uint8_t VOLTAGE_OBJ_ADDR = 2;
    static const uint8_t CURRENT_OBJ_ADDR = 3;
    static const uint8_t RPM_OBJ_ADDR = 4;
    static const uint8_t TEMPERATURE_OBJ_ADDR = 5;
    static const uint8_t GET_PWM_INPUT_OBJ_ADDR = 6;
    static const uint8_t GET_PWM_OUTPUT_OBJ_ADDR = 7;
    static const uint8_t MCU_ID_OBJ_ADDR = 8;
    static const uint8_t UPDATE_NODE_ID_OBJ_ADDR = 9;
    static const uint8_t ENUM_OBJ_ADDR = 10;
    static const uint8_t TELEMETRY_OBJ_ADDR = 11;

    static const uint16_t SET_PWM_MIN_INTERVAL_US = 2500;
    static const uint32_t TELEMETRY_INTERVAL_US = 100000;

    static const uint32_t SET_PWM_TIMEOUT_US = 2000;
    static const uint16_t TELEMETRY_TIMEOUT_US = 500;
    static const uint16_t ENUMERATION_TIMEOUT_MS = 30000;

    static const uint8_t CAN_IFACE_INDEX = 0;
};
