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
 *      Author: Francisco Ferreira and Tom Pittenger
 */
 
#pragma once

#include <AP_KDECAN/AP_KDECAN_config.h>

#if AP_KDECAN_ENABLED
#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#define AP_KDECAN_USE_EVENTS (defined(CH_CFG_USE_EVENTS) && CH_CFG_USE_EVENTS == TRUE)

#if AP_KDECAN_USE_EVENTS
#include <ch.h>
#endif

#define DEFAULT_NUM_POLES 14

#define KDECAN_MAX_NUM_ESCS 8

class AP_KDECAN_Driver : public CANSensor
#if HAL_WITH_ESC_TELEM
, public AP_ESC_Telem_Backend
#endif
{
public:
    
    AP_KDECAN_Driver();

    // called from SRV_Channels
    void update(const uint8_t num_poles);

private:

    // handler for incoming frames
    void handle_frame(AP_HAL::CANFrame &frame) override;
    
    bool send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint16_t data);
    bool send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_us, const uint8_t *data = nullptr, const uint8_t data_len = 0);

    void loop();

    struct {
        uint32_t detected_bitmask;
        uint32_t detected_bitmask_ms;
    } _init;

    struct {
        HAL_Semaphore sem;
        bool is_new;
        uint32_t last_new_ms;
        uint16_t pwm[NUM_SERVO_CHANNELS];
#if AP_KDECAN_USE_EVENTS
        thread_t *thread_ctx;
#endif
    } _output;

#if HAL_WITH_ESC_TELEM
    struct {
        uint8_t num_poles;
        uint32_t timer_ms;
    } _telemetry;
#endif

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


    static const uint32_t TELEMETRY_INTERVAL_MS = 100;

};

class AP_KDECAN {
public:
    AP_KDECAN();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_KDECAN);

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    static AP_KDECAN *get_singleton() { return _singleton; }

private:
    static AP_KDECAN *_singleton;

    AP_Int8 _num_poles;
    AP_KDECAN_Driver *_driver;
};
namespace AP {
    AP_KDECAN *kdecan();
};

#endif // AP_KDECAN_ENABLED
