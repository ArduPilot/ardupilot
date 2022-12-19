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
  Simulator for the HWingOneWire ESCs

./Tools/autotest/sim_vehicle.py --gdb --debug -v plane -f plane -A --uartF=sim:hwingesc:3 --speedup=1 --console

param set SERIAL5_PROTOCOL 45
param set RPM1_TYPE 5
param set SERVO_EST_MASK 4
graph RPM.rpm1

reboot

arm throttle
mode takeoff

param fetch

./Tools/autotest/autotest.py --gdb --debug build.ArduCopter fly.ArduPlane.HWingESC

*/

#pragma once

#include "SIM_SerialDevice.h"

namespace SITL {

class HWingESC : public SerialDevice {
public:

    HWingESC(uint8_t servo_channel_number);

    // update state
    void update(const class Aircraft &aircraft, const struct sitl_input &input);

private:

    // swiped from driver
    struct PACKED Packet {
        uint8_t header; // 0x9B
        uint8_t pkt_len; // 0x16
        uint32_t counter;
        uint16_t throttle_req;
        uint16_t throttle;
        uint16_t rpm;
        uint16_t voltage;
        int16_t current;
        int16_t phase_current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
        uint16_t crc;

        void update_checksum();

    } packet {
        0x9B,  // MAGIC!
        0x16   // length
    };

    uint8_t servo_channel_number;
    uint32_t counter;
};

class HWingESCs {
public:
    HWingESCs() {}

    struct Iterator {
        Iterator(HWingESC **_esc) : esc{_esc} {}
        HWingESC& operator*() const { return **esc; }
        HWingESC* operator->() { return *esc; }

        // Prefix increment
        Iterator& operator++() { esc++; return *this; };
        // Postfix increment
        Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }

        friend bool operator== (const Iterator& a, const Iterator& b) { return a.esc == b.esc; };
        friend bool operator!= (const Iterator& a, const Iterator& b) { return a.esc != b.esc; };
    private:
        HWingESC **esc;
    };

    Iterator begin() { return Iterator(&escs[0]); }
    Iterator end()   { return Iterator(&escs[num_escs]); }

    SITL::HWingESC *create(uint8_t servo_channel_number);

private:

    HWingESC *escs[8];
    uint8_t num_escs;
};

}
