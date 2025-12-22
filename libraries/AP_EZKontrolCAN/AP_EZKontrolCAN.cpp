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

#include "AP_EZKontrolCAN.h"
#include "AP_EZKontrolCAN_protocol.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_EZKontrolCAN *AP_EZKontrolCAN::_singleton;

const AP_Param::GroupInfo AP_EZKontrolCAN::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: EZKontrol CAN enable
    // @Description: Enable EZKontrol CAN motor backend
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, AP_EZKontrolCAN, _enable, 0),

    // @Param: CAN_PORT
    // @DisplayName: EZKontrol CAN port
    // @Description: CAN port index used for EZKontrol (1 or 2)
    // @Values: 1:CAN1,2:CAN2
    // @User: Advanced
    AP_GROUPINFO("CAN_PORT", 2, AP_EZKontrolCAN, _can_port, 1),

    // @Param: BITRATE
    // @DisplayName: EZKontrol CAN bitrate
    // @Description: CAN bitrate in kbit/s
    // @Values: 250:250k,500:500k
    // @User: Advanced
    AP_GROUPINFO("BITRATE", 3, AP_EZKontrolCAN, _bitrate, 500),

    // @Param: ADDR_L
    // @DisplayName: EZKontrol left motor address
    // @Description: Left motor node address
    // @User: Advanced
    AP_GROUPINFO("ADDR_L", 4, AP_EZKontrolCAN, _addr_left, 0xEF),

    // @Param: ADDR_R
    // @DisplayName: EZKontrol right motor address
    // @Description: Right motor node address
    // @User: Advanced
    AP_GROUPINFO("ADDR_R", 5, AP_EZKontrolCAN, _addr_right, 0xF0),

    // @Param: MODE
    // @DisplayName: EZKontrol control mode
    // @Description: Control mode for EZKontrol commands
    // @Values: 0:Speed,1:Torque
    // @User: Advanced
    AP_GROUPINFO("MODE", 6, AP_EZKontrolCAN, _mode, 0),

    // @Param: MAX_RPM
    // @DisplayName: EZKontrol maximum RPM
    // @Description: Maximum motor RPM used for scaling
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("MAX_RPM", 7, AP_EZKontrolCAN, _max_rpm, 0),

    // @Param: MAX_CURR_A
    // @DisplayName: EZKontrol maximum current
    // @Description: Maximum motor current in amps used for scaling
    // @Units: A
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("MAX_CURR_A", 8, AP_EZKontrolCAN, _max_curr_a, 0.0f),

    // @Param: DEBUG
    // @DisplayName: EZKontrol debug messages
    // @Description: Enable periodic GCS debug text
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 9, AP_EZKontrolCAN, _debug, 0),

    AP_GROUPEND
};

AP_EZKontrolCAN::AP_EZKontrolCAN() :
    _can_iface(nullptr),
    _can_inited(false),
    _healthy(false),
    _left_target(0.0f),
    _right_target(0.0f),
    _armed(false),
    _last_target_ms(0),
    _last_update_ms(0),
    _last_debug_ms(0)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

AP_EZKontrolCAN *AP_EZKontrolCAN::get_singleton()
{
    return _singleton;
}

void AP_EZKontrolCAN::init()
{
    _can_iface = nullptr;
    _can_inited = false;
    _healthy = false;

    if (!enabled()) {
        return;
    }

    const int8_t port = _can_port.get();
    if (port < 1 || port > HAL_NUM_CAN_IFACES) {
        return;
    }

    _can_iface = hal.can[port - 1];
    if (_can_iface == nullptr) {
        return;
    }

    const int16_t bitrate_kbps = _bitrate.get();
    const uint32_t bitrate_bps = (bitrate_kbps == 250) ? 250000U : 500000U;

    // TODO Stage 2: configure filters and protocol-specific options.
    _can_inited = _can_iface->init(bitrate_bps);
    _healthy = _can_inited;
}

void AP_EZKontrolCAN::set_targets(float left_norm, float right_norm, bool armed)
{
    _left_target = constrain_float(left_norm, -1.0f, 1.0f);
    _right_target = constrain_float(right_norm, -1.0f, 1.0f);
    _armed = armed;
    _last_target_ms = AP_HAL::millis();
}

void AP_EZKontrolCAN::update()
{
    _last_update_ms = AP_HAL::millis();

    if (!enabled()) {
        return;
    }

    if (_debug.get() != 0) {
        const uint32_t now_ms = _last_update_ms;
        if (now_ms - _last_debug_ms >= 1000U) {
            _last_debug_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "EZK tgt L:%.2f R:%.2f armed:%u",
                          (double)_left_target,
                          (double)_right_target,
                          _armed ? 1U : 0U);
        }
    }

    // TODO Stage 2: implement handshake, heartbeat, and command frames.
}

bool AP_EZKontrolCAN::enabled() const
{
    return _enable.get() != 0;
}

bool AP_EZKontrolCAN::healthy() const
{
    return !enabled() || _healthy;
}

AP_EZKontrolCAN *AP::ezkontrol_can()
{
    return AP_EZKontrolCAN::get_singleton();
}
