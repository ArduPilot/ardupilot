/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <iostream>
#include <string>
// #include <sstream>
//#include <vector>

#include "Socket.cpp"

class libAP_JSON
{
public:
    bool InitSockets(const char *fdm_address, const uint16_t fdm_port_in);
    bool ReceiveServoPacket(uint16_t servo_out[]);
    void SendState(double timestamp,
                               double gyro_x, double gyro_y, double gyro_z, // rad/sec
                               double accel_x, double accel_y, double accel_z, // m/s^2
                               double pos_x, double pos_y, double pos_z, // m in inertial frame
                               double phi, double theta, double psi, // attitude radians
                               double V_x, double V_y, double V_z, // m/s in inertial frame
                               double airspeed); // m/s
private:
    // \brief Keep track of controller update sim-time.
    double lastControllerUpdateTime = 0;

    // Keep track of the time the last servo packet was received.
    double lastServoPacketRecvTime = 0;

    // Socket manager
    SocketAPM sock = SocketAPM(true);

    // The address for the flight dynamics model
    // const char *libAP_JSON::fdm_address;

    // The address for the SITL flight controller - auto detected
    const char *fcu_address;

    // The port for the flight dynamics model
    // uint16_t libAP_JSON::fdm_port_in;

    // The port for the SITL flight controller - auto detected
    uint16_t fcu_port_out;

    bool arduPilotOnline;

    // Number of consecutive missed ArduPilot controller messages
    int connectionTimeoutCount;

    // Max number of consecutive missed ArduPilot controller messages before timeout
    int connectionTimeoutMaxCount;

    // Last received frame rate from the ArduPilot controller
    uint16_t fcu_frame_rate;

    // Last received frame count from the ArduPilot controller
    uint32_t fcu_frame_count = -1;
};
