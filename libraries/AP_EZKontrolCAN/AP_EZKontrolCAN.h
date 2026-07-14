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

#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <AP_HAL/CANIface.h>

class AP_EZKontrolCAN : public AP_ESC_Telem_Backend {
public:
    AP_EZKontrolCAN();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_EZKontrolCAN);

    static AP_EZKontrolCAN *get_singleton();

    void init();
    void set_targets(float left_norm, float right_norm, bool armed);
    void update();

    bool enabled() const;
    bool healthy() const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    struct Telemetry {
        float bus_voltage = 0.0f;
        float bus_current = 0.0f;
        float phase_current = 0.0f;
        float speed_rpm = 0.0f;
        float temp_mos_c = 0.0f;
        float temp_motor_c = 0.0f;
        uint16_t status_bits = 0;
        uint32_t error_bits = 0;
        bool telem_1_valid = false;
        bool telem_2_valid = false;
        bool valid = false;
    };

    struct ControllerState {
        uint8_t address = 0;
        uint8_t esc_index = 0;
        bool handshake_complete = false;
        uint8_t life_counter = 0;
        uint32_t last_handshake_ms = 0;
        uint32_t last_telem_ms = 0;
        uint32_t last_handshake_warn_ms = 0;
        uint32_t last_telem_warn_ms = 0;
        uint32_t last_fault_warn_ms = 0;
        uint32_t last_error_bits = 0;
        bool debug_handshake_reported = false;
        bool debug_telem_reported = false;
        Telemetry telem {};
    };

    AP_Int8 _enable;
    AP_Int8 _can_port;
    AP_Int16 _bitrate;
    AP_Int16 _addr_left;
    AP_Int16 _addr_right;
    AP_Int8 _mode;
    AP_Int16 _max_rpm;
    AP_Float _max_curr_a;
    AP_Int8 _debug;
    AP_Int16 _timeout_ms;

    AP_HAL::CANIface *_can_iface;
    bool _can_inited;
    bool _healthy;

    float _left_target;
    float _right_target;
    bool _armed;
    uint32_t _last_target_ms;
    uint32_t _last_update_ms;
    uint32_t _last_debug_ms;
    uint32_t _last_send_ms;
    uint32_t _rx_count;
    uint32_t _last_rx_debug_count;
    uint32_t _last_rx_id;
    uint8_t _last_rx_dlc;

    ControllerState _left_state;
    ControllerState _right_state;

    static AP_EZKontrolCAN *_singleton;

    void reset_controller_state(ControllerState &state);
    void handle_rx_frame(const AP_HAL::CANFrame &frame);
    void handle_telemetry_1(ControllerState &state, const AP_HAL::CANFrame &frame);
    void handle_telemetry_2(ControllerState &state, const AP_HAL::CANFrame &frame);
    void publish_esc_telem(const ControllerState &state, uint16_t data_mask, bool publish_rpm);
    void send_handshake_ack(uint8_t dest_addr);
    void send_command(ControllerState &state, float target_norm, bool armed);
    void publish_esc_telem(const ControllerState &state) const;
    void update_health();
    ControllerState *find_state_by_address(uint8_t address);
};

namespace AP {
    AP_EZKontrolCAN *ezkontrol_can();
};
