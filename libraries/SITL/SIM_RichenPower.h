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
  Simulator for the RichenPower Hybrid generators

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:richenpower --speedup=1 --console

param set SERIAL5_PROTOCOL 30
param set SERIAL5_BAUD 9600
param set SIM_RICH_ENABLE 1
param set SERVO8_FUNCTION 42
param fetch
param set SIM_RICH_CTRL 8
param set RC9_OPTION 85
param set GEN_TYPE 3

reboot

graph SERVO_OUTPUT_RAW.servo8_raw
graph RC_CHANNELS.chan9_raw
module load generator

arm throttle (denied because generator not running)

./Tools/autotest/autotest.py --gdb --debug build.ArduCopter fly.ArduCopter.RichenPower

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"
#include "SIM_GeneratorEngine.h"

namespace SITL {

class RichenPower : public SerialDevice {
public:

    RichenPower();

    // update state
    void update(const struct sitl_input &input);

    static const AP_Param::GroupInfo var_info[];

private:

    // this is what the generator supplies when running full-tilt but
    // with no load
    const float base_supply_voltage = 50.0f;
    const float max_current = 50.0f;
    const uint32_t original_seconds_until_maintenance = 20*60; // 20 minutes

// They have been... we know that the voltage drops to mid 46v
// typically if gennie stops
// So we set batt fs high 46s
// Gennie keeps batts charged to 49v + typically

    uint32_t last_sent_ms;

    void update_control_pin(const struct sitl_input &input);
    void update_send();

    enum class State {
        STOP = 21,
        IDLE = 22,
        RUN = 23,
        STOPPING = 24, // idle cool-down period
    };
    State _state = State::STOP;
    void set_run_state(State newstate);

    AP_Int8  _enabled;  // enable richenpower sim
    AP_Int8  _ctrl_pin;

    float _current_rpm;
    uint32_t _runtime_ms;
    uint32_t _last_runtime_ms;

    float _current_current;

    enum class Errors {
        MaintenanceRequired = 0,
    };

    // packet to send:
    struct PACKED RichenPacket {
        uint8_t magic1;
        uint8_t magic2;
        uint8_t version_minor;
        uint8_t version_major;
        uint8_t runtime_minutes;
        uint8_t runtime_seconds;
        uint16_t runtime_hours;
        uint32_t seconds_until_maintenance;
        uint16_t errors;
        uint16_t rpm;
        uint16_t throttle;
        uint16_t idle_throttle;
        uint16_t output_voltage;
        uint16_t output_current;
        uint16_t dynamo_current;
        uint8_t unknown1;
        uint8_t mode;
        uint8_t unknown6[38]; // "data"?!
        uint16_t checksum;
        uint8_t footermagic1;
        uint8_t footermagic2;
    };

    union RichenUnion {
        uint8_t parse_buffer[70];
        uint16_t checksum_buffer[35];
        struct RichenPacket packet;

        void update_checksum();
    };
    RichenUnion u;

    // time we were asked to stop; 
    uint32_t stop_start_ms;

    SIM_GeneratorEngine generatorengine;
};

}
