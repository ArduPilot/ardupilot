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
  Simulator for the Loweheiser EFI/generator

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:loweheiser --speedup=1 --console

param set SIM_EFI_TYPE 2
param set SERIAL5_PROTOCOL 2
param set GEN_TYPE 4
param set EFI_TYPE 4
param set BATT2_MONITOR 17  # generator (elec)
param set BATT3_MONITOR 18  # generator (fuel level)
param fetch

param set BATT3_CAPACITY 10000
param set BATT3_LOW_MAH 1000
param set BATT3_CRT_MAH 500
param set BATT3_FS_LOW_ACT 2 # RTL
param set BATT3_FS_CRT_ACT 1 # LAND
param set BATT3_LOW_VOLT 0

param set RC9_OPTION 85   # generator control

param set GEN_IDLE_TH_H 40 # NOTE without this the engine never warms up past 36 deg C
param set GEN_IDLE_TH 25

param set RC10_OPTION 212 # loweheiser manual throttle control
param set RC10_DZ 20
param set RC10_TRIM 1000
param set RC10_MIN 1000
param set RC10_MAX 2000

param set RC11_OPTION 109 # loweheiser starter channel

reboot

# for testing failsafes:
param set BATT3_CAPACITY 200
param set BATT3_LOW_MAH 100
param set BATT3_CRT_MAH 50

# stream EFI_STATUS at 10Hz:
long SET_MESSAGE_INTERVAL 225 100000

# run SITL against real generator:
DEV=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=uart:$DEV:115200 --speedup=1 --console

# run generator test script against simulator:
python ./libraries/AP_Generator/scripts/test-loweheiser.py tcp:localhost:5762

# use the generator test script to control the generator:
./libraries/AP_Generator/scripts/test-loweheiser.py $DEV

# observe RPM

# observe remaining fuel:
graph BATTERY_STATUS[2].battery_remaining
graph BATTERY_STATUS[2].current_consumed

# autotest suite:

./Tools/autotest/autotest.py --gdb --debug build.Copter test.Copter.Loweheiser

# use a usb-ttl cable to connect directly to mavlink-speaking generator:
DEV=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
mavproxy.py --master $DEV --baud 115200

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_LOWEHEISER_ENABLED

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"
#include "SIM_GeneratorEngine.h"

#include <GCS_MAVLink/GCS_MAVLink.h>

#include <stdio.h>

namespace SITL {

class Loweheiser : public SerialDevice {
public:

    Loweheiser();

    // update state
    void update();

private:

    // TODO: make these parameters:
    const uint8_t system_id = 17;
    const uint8_t component_id = 18;

    const float max_current = 50.0f;
    const float base_supply_voltage = 50.0;

    uint32_t last_sent_ms;

    void update_receive();
    void update_send();

    void maybe_send_heartbeat();
    uint32_t last_heartbeat_ms;

    void handle_message(const mavlink_message_t &msg);

    enum class EngineRunState : uint8_t {
        OFF = 0,
        ON = 1,
    };
    EngineRunState autopilot_desired_engine_run_state = EngineRunState::OFF;

    enum class GovernorState : uint8_t {
        OFF = 0,
        ON = 1,
    };
    GovernorState autopilot_desired_governor_state = GovernorState::OFF;

    float manual_throttle_pct;

    enum class StartupState : uint8_t {
        OFF = 0,
        ON = 1,
    };
    StartupState autopilot_desired_startup_state = StartupState::OFF;

    mavlink_message_t rxmsg;
    mavlink_status_t rxstatus;

    SIM_GeneratorEngine generatorengine;

    float _current_current;

    // fuel
    const float initial_fuel_level = 10;  // litres, must match battery setup
    float fuel_level = initial_fuel_level;  // litres
    float fuel_consumed = 0;  // litres
    float fuel_flow_lps = 0; // litres/second
    void update_fuel_level();

    uint32_t last_fuel_update_ms;

    mavlink_status_t mav_status;

    // parameters
    // AP_Int8 _enabled;
};

}

#endif  // AP_SIM_LOWEHEISER_ENABLED
