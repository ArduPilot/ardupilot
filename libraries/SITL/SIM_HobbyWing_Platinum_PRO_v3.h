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
  Simulator for the HobbyWing Platinum Pro v3 series ESCs

./Tools/autotest/sim_vehicle.py --gdb --debug -v plane -f plane -A --uartF=sim:hobbywing_platinum_pro_v3:3 --speedup=1 --console

param set SERIAL5_PROTOCOL 45
param set RPM1_TYPE 5
param fetch
param set RPM1_ESC_MASK 4
param set SERVO_H_V3_MASK 4
param set SIM_ESC_TELEM 0
graph RPM.rpm1

reboot

arm throttle
mode takeoff

param fetch

./Tools/autotest/autotest.py --gdb --debug build.ArduCopter fly.ArduPlane.HobbyWing_Platinum_PRO_v3

./Tools/autotest/sim_vehicle.py  --gdb --debug -v plane -f plane -A --uartF=logic_async_csv:libraries/AP_ESC_Telem/examples/samples/hobbywing-platinum-pro-v3.csv --speedup=1 -B AP_HobbyWing_Platinum_PRO_v3::update
graph RPM.rpm1

*/

#pragma once

#include "SIM_SerialDevice.h"

namespace SITL {

class HobbyWing_Platinum_PRO_v3 : public SerialDevice {
public:

    HobbyWing_Platinum_PRO_v3(uint8_t servo_channel_number);

    // update state
    void update(const class Aircraft &aircraft, const struct sitl_input &input);

private:

    uint8_t servo_channel_number;
    uint32_t counter;
    uint32_t last_send_ms;

    uint32_t commutation_time_from_rpm(int16_t rpm) const;
};

// this is a class to contain multiple instances of the ESC:
class HobbyWing_Platinum_PRO_v3s {
public:
    HobbyWing_Platinum_PRO_v3s() {}

    struct Iterator {
        Iterator(HobbyWing_Platinum_PRO_v3 **_esc) : esc{_esc} {}
        HobbyWing_Platinum_PRO_v3& operator*() const { return **esc; }
        HobbyWing_Platinum_PRO_v3* operator->() { return *esc; }

        // Prefix increment
        Iterator& operator++() { esc++; return *this; };
        // Postfix increment
        Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }

        friend bool operator== (const Iterator& a, const Iterator& b) { return a.esc == b.esc; };
        friend bool operator!= (const Iterator& a, const Iterator& b) { return a.esc != b.esc; };
    private:
        HobbyWing_Platinum_PRO_v3 **esc;
    };

    Iterator begin() { return Iterator(&escs[0]); }
    Iterator end()   { return Iterator(&escs[num_escs]); }

    SITL::HobbyWing_Platinum_PRO_v3 *create(uint8_t servo_channel_number);

private:

    HobbyWing_Platinum_PRO_v3 *escs[8];
    uint8_t num_escs;
};

}
