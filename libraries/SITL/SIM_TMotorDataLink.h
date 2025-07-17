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
  Simulator for the T-Motor DataLink protocol

 Note that the data being packed in is very similar to the data handled in SIM_HWing.cpp

See https://store.tmotor.com/goods.php?id=728

# the following creates a tmotordatalink connection on SERIAL5 with
#  the ESC connected to ArduPilot's servo3 output connected on the
#  "M1" port on the device and the ESC connected to ArduPilot's servo8
#  port connected on the DataLink's M3 port:

./Tools/autotest/sim_vehicle.py --gdb --debug -v plane -f plane -A --uartF=sim:tmotordatalink:0,0,3 --speedup=1 --console

param set SERIAL5_PROTOCOL 46

reboot

arm throttle
mode takeoff

param fetch

./Tools/autotest/autotest.py --gdb --debug build.Plane test.Plane.HobbyWing_DataLink

*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SIM_TMOTOR_DATALINK_ENABLED
#define AP_SIM_TMOTOR_DATALINK_ENABLED 1
#endif

#if AP_SIM_TMOTOR_DATALINK_ENABLED

#include "SIM_SerialDevice.h"

namespace SITL {

class TMotorDataLink : public SerialDevice {
public:

    TMotorDataLink();

    // set the motors which are connected to the simulated device.
    // This is a comma-separated list, where 0 can be used as a
    // placeholder for "nothing connected"
    void set_motors(const char *comma_separated_list_of_motors);

    // update state
    void update(const class Aircraft &aircraft, const struct sitl_input &input);

private:

    struct PACKED ESCInfo {
        uint8_t esc_channel_number;
        uint16_t seq;
        uint16_t throttle_req;
        uint16_t throttle;
        uint16_t rpm;
        uint16_t voltage;
        uint16_t current;
        uint16_t phase_current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
    };

    struct PACKED Packet {
        uint8_t header; // 0x9B
        uint8_t pkt_len; // 160
        uint8_t communications_protocol_version;
        uint8_t command_word;
        uint16_t seq;
        ESCInfo motor[8];
        uint16_t crc;

        void update_checksum();

    } packet {
        0x9B,  // MAGIC!
        160,   // length
        1,     // protcol version number
        0x02,  // command word
    };

    uint8_t servo_channel_numbers[8];
    const uint8_t num_servo_channels = 8;  // length of servo_channel_number

    uint32_t counter;

    uint32_t last_send_ms;

    void update_motor(uint8_t motor_number, const Aircraft &aircraft, const sitl_input &input);

};
}

#endif
