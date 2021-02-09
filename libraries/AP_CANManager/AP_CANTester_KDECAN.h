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
 * Author: Francisco Ferreira
 * Modified for CANManager by Siddharth B Purohit
 */

#include "AP_CANDriver.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 1 && !HAL_MINIMIZE_FEATURES && HAL_MAX_CAN_PROTOCOL_DRIVERS

#define NUM_ESCS 4


class AP_CANTester_KDECAN
{
public:
    AP_CANTester_KDECAN()
    {
        for (uint8_t i = 0; i < NUM_ESCS; i++) {
            _esc_info[i].mcu_id = 0xA5961824E7BD3C00 | i;
        }
    }

    bool init(AP_HAL::CANIface* can_iface);
    void loop(void);
    void print_stats(void);
    bool send_enumeration(uint8_t num);

private:
    uint8_t _driver_index = 0;
    uint8_t _interface = 0;
    AP_HAL::CANIface* _can_iface;
    uint8_t _mask_received_pwm = 0;

    struct esc_info {
        uint8_t node_id;
        uint64_t mcu_id;
        uint32_t enum_timeout_ms;

        esc_info() : node_id(1), mcu_id(0), enum_timeout_ms(0) {}
    } _esc_info[NUM_ESCS];

    uint8_t _max_node_id = 0;

    static const uint8_t BROADCAST_NODE_ID = 1;

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
    static const uint8_t START_ENUM_OBJ_ADDR = 10;
    static const uint8_t TELEMETRY_OBJ_ADDR = 11;

    struct {
        uint32_t frame_id;
        uint32_t count;
    } counters[100];

    void count_msg(uint32_t frame_id);
    HAL_EventHandle _event_handle;
};
#endif
