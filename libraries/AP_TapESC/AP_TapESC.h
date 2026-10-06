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
  driver for TAP UART ESCs, as used on the Intel Aero RTF

  The motor outputs for Motor1 to Motor4 are sent to the ESCs over a
  serial port configured with SERIALn_PROTOCOL set to TAP ESC.
 */
#pragma once

#include "AP_TapESC_config.h"

#if AP_TAPESC_ENABLED

#include <AP_HAL/AP_HAL.h>

class AP_TapESC {
public:
    AP_TapESC() {}

    CLASS_NO_COPY(AP_TapESC);

    // called from SRV_Channels::push() to send the motor outputs
    void update();

private:
    static const uint8_t NUM_MOTORS = 4;
    static const uint8_t MAX_MOTORS = 8;

    enum class MsgID : uint8_t {
        CONFIG_BASIC = 0,
        RUN = 2,
    };

    struct PACKED ConfigBasic {
        uint8_t max_channel_in_use;
        uint8_t channel_map[MAX_MOTORS];
        uint8_t monitor_msg_type;
        uint8_t control_mode;
        uint16_t min_channel_value;
        uint16_t max_channel_value;
    };

    struct PACKED Run {
        uint16_t value[NUM_MOTORS];
    };

    void init();
    void send_packet(MsgID msg_id, const void *data, uint8_t len);
    void send_config();
    void send_run(const uint16_t rpm[NUM_MOTORS]);
    uint16_t rpm_for_motor(uint8_t motor) const;

    AP_HAL::UARTDriver *uart;

    enum class State : uint8_t {
        UNINITIALISED,
        NO_PORT,
        WAIT_BOOT,
        UNLOCKING,
        RUNNING,
    } state;

    uint8_t unlock_count;
    uint32_t last_send_us;
    uint32_t last_led_toggle_ms;
    bool led_on;
};

#endif  // AP_TAPESC_ENABLED
