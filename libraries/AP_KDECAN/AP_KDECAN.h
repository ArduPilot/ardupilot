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

#include <AP_HAL/AP_HAL.h>

#ifndef AP_KDECAN_ENABLED
    #define AP_KDECAN_ENABLED (HAL_NUM_CAN_IFACES && !HAL_MINIMIZE_FEATURES) && !defined(HAL_BUILD_AP_PERIPH)
#endif

#if AP_KDECAN_ENABLED

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_HAL/utility/RingBuffer.h>

#define AP_KDECAN_USE_EVENTS (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && CH_CFG_USE_EVENTS == TRUE)

#define DEFAULT_NUM_POLES 14

#define KDECAN_MAX_NUM_ESCS 8

class AP_KDECAN : public CANSensor, public AP_ESC_Telem_Backend {
public:
    AP_KDECAN(const uint8_t driver_index);
    
    /* Do not allow copies */
    AP_KDECAN(const AP_KDECAN &other) = delete;
    AP_KDECAN &operator=(const AP_KDECAN&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // Return KDECAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_KDECAN *get_kdecan(uint8_t driver_index);

    // called from SRV_Channels
    void update();
    
    // check that arming can happen
    bool pre_arm_check(char* reason, uint8_t reason_len);

#ifdef HAL_BUILD_AP_PERIPH
    void set_param_npole(const int8_t value) { _telemetry.num_poles = value; }
#endif

    // caller checks that vehicle isn't armed
    // start_stop: true to start, false to stop
    bool run_enumeration(bool start_stop);

private:

    // enumeration for self-ordering
    enum class ENUMERATION_STATE : uint8_t {
        START,
        RUNNING,
        STOP,
        CHECK_STATUS,
        STOPPED,
    };

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    
    void handle_frame_discovery(AP_HAL::CANFrame &esc_id_frame);
    void request_telemetry();
    bool send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint16_t data);
    bool send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint8_t *data = nullptr, const uint8_t data_len = 0);

    void update_enumeration();

    void loop();

    struct {
        char thread_name[11];
        uint8_t driver_index;
    } _init;

    struct {
        // ESC detected information
        uint32_t timer_ms;
        uint16_t present_bitmask = 0;
        uint8_t max_node_id;
        uint8_t esc_num;
        ENUMERATION_STATE state = ENUMERATION_STATE::STOPPED;
    } _enumeration;

    struct {
        HAL_Semaphore sem;
        bool is_new;
        uint32_t last_new_ms;
        uint16_t pwm[KDECAN_MAX_NUM_ESCS];
#if AP_KDECAN_USE_EVENTS
        thread_t *thread_ctx;
#endif
        uint16_t present_bitmask;
        uint16_t max_node_id;
    } _output;

    struct {
        AP_Int8 num_poles;
        uint32_t timer_ms;
    } _telemetry;

    union frame_id_t {
        struct PACKED {
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


    static const uint32_t PWM_MIN_INTERVAL_MS = 3;
    static const uint32_t PWM_IS_NEW_TIMEOUT_MS = 1000;
    static const uint32_t TELEMETRY_INTERVAL_MS = 100;
    static const uint16_t ENUMERATION_TIMEOUT_MS = 30000;

};
#endif //AP_KDECAN_ENABLED
