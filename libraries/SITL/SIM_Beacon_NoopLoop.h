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
  Simulator for the NoopLoop (LinkTrack) UWB beacon system

  Driver: AP_Beacon/AP_Beacon_Nooploop.cpp

  Use:

./Tools/autotest/sim_vehicle.py -v ArduCopter -A "--serial5=sim:nooploop" --map --console

param set SERIAL5_PROTOCOL 13  # beacon
param set BCN_TYPE 3           # nooploop
param set GPS1_TYPE 0
param set AHRS_EKF_TYPE 3
param set EK3_SRC1_POSXY 4     # beacon
param set BCN_LATITUDE -35.363262
param set BCN_LONGITUDE 149.165237
param set BCN_ALT 584.0
reboot
*/

#pragma once

#include "SIM_SerialBeacon.h"

#if AP_SIM_NOOPLOOP_ENABLED

namespace SITL {

class Beacon_NoopLoop : public SerialBeacon {
public:

    using SerialBeacon::SerialBeacon;

protected:

    void send_data(const Vector3f &pos_ned) override;

private:

    static constexpr uint8_t num_anchors = 4;

    // position of anchor i, in NED metres relative to the beacon origin:
    Vector3f anchor_position(uint8_t i) const;

    // build and emit the two frames the driver consumes:
    void send_setting_frame0();                    // anchor positions
    void send_node_frame2(const Vector3f &pos_ned); // tag position + ranges

    // store value (millimetres) as a signed 24-bit little-endian integer:
    static void put_int24_le(uint8_t *buf, int32_t value_mm);
};

}

#endif  // AP_SIM_NOOPLOOP_ENABLED
