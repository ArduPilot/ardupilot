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
  Simulator for Volz servos

./Tools/autotest/sim_vehicle.py --gdb --debug -v Plane -A --serial5=sim:volz --speedup=1 --console -f plane-redundant

param set SERIAL5_PROTOCOL 14  // Volz
param set SERIAL5_BAUD 115200
param set SERVO_VOLZ_MASK 1803
param set SIM_VOLZ_ENA 1
param set SIM_VOLZ_MASK 1803
reboot

param fetch

#param set SIM_VOLZ_FMASK 1  # fail mask

./Tools/autotest/autotest.py --gdb --debug build.ArduCopter fly.ArduCopter.Volz

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_VOLZ_ENABLED

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"

namespace SITL {

class Volz : public SerialDevice {
public:

    Volz();

    // update state
    void update(const class Aircraft &aircraft);

    static const AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enabled.get(); }

    void update_sitl_input_pwm(struct sitl_input &input);

private:

    // parameters / configuration
    AP_Int8  _enabled;  // enable FETtec servo sim
    AP_Int32 _output_mask;  // mask of servo outputs to override
    AP_Int32 _failed_mask;  // mask of servo outputs to mark as failed

    // packet abstraction; "struct Command" is used for both reading
    // and writing to the autopilot.
    enum class CommandId : uint8_t {
        SET_EXTENDED_POSITION = 0xDC,
        EXTENDED_POSITION_RESPONSE = 0x2C,
        READ_CURRENT = 0xB0,
        CURRENT_RESPONSE = 0x30,
        READ_VOLTAGE = 0xB1,
        VOLTAGE_RESPONSE = 0x31,
        READ_TEMPERATURE = 0xC0,
        TEMPERATURE_RESPONSE = 0x10,
    };
    struct PACKED Command {
        CommandId command_id;
        uint8_t actuator_id; // actuator send to or receiving from
        uint8_t arg1; // CMD dependant argument 1
        uint8_t arg2; // CMD dependant argument 2
        uint16_t crc;

        uint16_t calculate_checksum() const {
            return crc_crc16_ibm(0xffff, (uint8_t*)this, 4);
        }
        void update_checksum();
    };

    // reading-from-autopilot support:
    union {
        Command command;
        uint8_t buffer[128];
    } u;
    uint8_t buflen;
    bool shift_command_to_front_of_buffer();
    void consume_command();
    void process_command(const Command &command);
    void update_servos(const class Aircraft &aircraft);
    void update_input();

    // class creating a servo abstraction:
    class Servo {
    public:
        uint8_t id;

        float position;  // -1 to 1
        float desired_position;
        float pcb_temperature;
        float motor_temperature;
        float primary_current;
        float secondary_current;
        float primary_voltage;
        float secondary_voltage;

        uint16_t pwm() {
            if (failed) {
                return last_good_pwm;
            }
            last_good_pwm = 1500 + desired_position*500;
            return last_good_pwm;
        }

        bool failed;
        uint16_t last_good_pwm;
    };

    // the simulated servos:
    Servo servos[16];

    // last time we updated the servo physics:
    uint32_t last_servo_update_us;


    void log_Servo(const Volz::Servo &servo, const uint32_t now_us);
};

}

#endif  // AP_SIM_VOLZ_ENABLED
