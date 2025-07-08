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

  Simulator for the LD06 proximity sensor

  Datasheet: https://storage.googleapis.com/mauser-public-images/prod_description_document/2021/315/8fcea7f5d479f4f4b71316d80b77ff45_096-6212_a.pdf

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:ld06 --speedup=1 -l 51.8752066,14.6487840,54.15,0 --map

param set SERIAL5_PROTOCOL 11
param set PRX1_TYPE 16
reboot

arm throttle
rc 3 1600

# for avoidance:
param set DISARM_DELAY 0
param set AVOID_ENABLE 2 # use proximity sensor
param set AVOID_MARGIN 2.00  # 2m
param set AVOID_BEHAVE 0 # slide
param set OA_TYPE 2
reboot

param ftp
param set OA_DB_OUTPUT 3

mode loiter
script /tmp/post-locations.scr
arm throttle
rc 3 1600
rc 3 1500
rc 2 1450

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_PS_LD06_ENABLED

#include "SIM_SerialProximitySensor.h"

#include <stdio.h>

namespace SITL {

class PS_LD06 : public SerialProximitySensor {
public:

    using SerialProximitySensor::SerialProximitySensor;

    uint32_t packet_for_location(const Location &location,
                                 uint8_t *data,
                                 uint8_t buflen) override;

    void update(const Location &location) override;

private:

    void update_input();  // in the form of PWM (uninplemented)
    void update_output(const Location &location);

    uint32_t last_scan_output_time_ms;

    float last_degrees_bf;

    uint8_t * send_buffer;
    uint16_t send_buffer_size;
};

};

#endif  // AP_SIM_PS_LD06_ENABLED
