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
  Simulate SBG serial INS (sbgECom binary protocol)

  Usage:
  PARAMS:
      param set AHRS_EKF_TYPE 11
      param set EAHRS_TYPE 8
      param set SERIAL4_PROTOCOL 36
      param set SERIAL4_BAUD 230400
      param set GPS1_TYPE 21
  sim_vehicle.py -v Plane -A "--serial4=sim:SBG" --console --map -DG
*/

#pragma once

#include "SIM_config.h"
#include "SIM_SerialDevice.h"

namespace SITL
{

class SBG : public SerialDevice
{
public:

    SBG();

    // update state
    void update(void);

private:

    // answer autopilot requests (SBG_ECOM_CMD_INFO)
    void check_incoming();

    void send_frame(uint8_t msgclass, uint8_t msgid, const void *payload, uint16_t len);
    void send_device_info();
    void send_utc_time();
    void send_imu_short();
    void send_mag();
    void send_ekf_quat();
    void send_ekf_nav();
    void send_gps();
    void send_air_data();

    // get timeval using simulation time
    static void simulation_timeval(struct timeval *tv);
    // GPS time of week in ms for the given simulation time
    static uint32_t gps_tow_ms(const struct timeval &tv);

    uint32_t last_ins_pkt_ms;
    uint32_t last_gps_pkt_ms;
    bool seen_autopilot_traffic;

    // minimal parser for inbound (autopilot to device) frames. Payload and
    // CRC are not stored: only the message class/id are needed to answer
    // device info requests
    enum class ParseState : uint8_t {
        SYNC1,
        SYNC2,
        MSG,
        CLASS,
        LEN1,
        LEN2,
        DATA,
        CRC1,
        CRC2,
        ETX,
    };
    struct {
        ParseState state;
        uint8_t msgid;
        uint8_t msgclass;
        uint16_t len;
        uint16_t count;
    } rx;
};

}
