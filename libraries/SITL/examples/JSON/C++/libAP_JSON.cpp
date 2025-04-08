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

/*
 * Adapted from Pierre Kancir's ArduPilot plugin for Gazebo
 */

#include "libAP_JSON.h"

#define DEBUG_ENABLED 0

// SITL JSON interface supplies 16 servo channels
#define MAX_SERVO_CHANNELS 16

// The servo packet received from ArduPilot SITL. Defined in SIM_JSON.h.
struct servo_packet {
    uint16_t magic; // 18458 expected magic value
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[16];
};

bool libAP_JSON::InitSockets(const char *fdm_address, const uint16_t fdm_port_in) {
    // configure port
    sock.set_blocking(false);
    sock.reuseaddress();

    // bind the socket
    if (!sock.bind(fdm_address, fdm_port_in))
    {
        std::cout << "[libAP_JSON] " << "failed to bind with "
                  << fdm_address << ":" << fdm_port_in << std::endl;
        return false;
    }
    std::cout << "[libAP_JSON] " << "flight dynamics model at "
              << fdm_address << ":" << fdm_port_in << std::endl;
    return true;
}

bool libAP_JSON::ReceiveServoPacket(uint16_t servo_out[]) {
    // Added detection for whether ArduPilot is online or not.
    // If ArduPilot is detected (receive of fdm packet from someone),
    // then socket receive wait time is increased from 1ms to 1 sec
    // to accommodate network jitter.
    // If ArduPilot is not detected, receive call blocks for 1ms
    // on each call.
    // Once ArduPilot presence is detected, it takes this many
    // missed receives before declaring the FCS offline.

    uint32_t waitMs;
    if (ap_online) {
        // Increase timeout for recv once we detect a packet from ArduPilot FCS.
        // If this value is too high then it will block the main Gazebo update loop
        // and adversely affect the RTF.
        waitMs = 10;
    } else {
        // Otherwise skip quickly and do not set control force.
        waitMs = 1;
    }

    servo_packet pkt;
    auto recvSize = sock.recv(&pkt, sizeof(servo_packet), waitMs);
    // libAP_JSON::fcu_address = fcu_address;
    // libAP_JSON::fcu_port_out = fcu_port_out;

    sock.last_recv_address(fcu_address, fcu_port_out);

    // drain the socket in the case we're backed up
    int counter = 0;
    while (true) {
        servo_packet last_pkt;
        auto recvSize_last = sock.recv(&last_pkt, sizeof(servo_packet), 0ul);
        if (recvSize_last == -1) {
            break;
        }
        counter++;
        pkt = last_pkt;
        recvSize = recvSize_last;
    }
    if (counter > 0) {
        std::cout << "[libAP_JSON] " << "Drained n packets: " << counter << std::endl;
    }

    // didn't receive a packet, increment timeout count if online, then return
    if (recvSize == -1) {
        if (ap_online) {
            if (++connectionTimeoutCount > connectionTimeoutMaxCount) {
                connectionTimeoutCount = 0;
                ap_online = false;
                std::cout << "[libAP_JSON] " << "Broken ArduPilot connection (no packets received)" << std::endl;
            }
        }
        return false;
    }

#if DEBUG_ENABLED
    // debug: inspect SITL packet
    std::cout << "recv " << recvSize << " bytes from " << fcu_address << ":" << fcu_port_out << "\n";
    std::cout << "magic: " << pkt.magic << "\n";
    std::cout << "frame_rate: " << pkt.frame_rate << "\n";
    std::cout << "frame_count: " << pkt.frame_count << "\n";
    std::cout << "pwm: [";
    for (auto i = 0; i < MAX_SERVO_CHANNELS - 1; ++i) {
        std::cout << pkt.pwm[i] << ", ";
    }
    std::cout << pkt.pwm[MAX_SERVO_CHANNELS - 1] << "]\n";
    std::cout << "\n";
#endif

    // check magic, return if invalid
    const uint16_t magic = 18458;
    if (magic != pkt.magic) {
        std::cout << "[libAP_JSON] Incorrect protocol magic " << pkt.magic << " should be " << magic << "\n";
        return false;
    }

    // check frame rate and frame order
    fcu_frame_rate = pkt.frame_rate;
    if (pkt.frame_count < fcu_frame_count) {
        // @TODO - implement re-initialisation
        std::cout << "[libAP_JSON] ArduPilot controller has reset\n";
    }
    else if (pkt.frame_count == fcu_frame_count) {
        // received duplicate frame, skip
        std::cout << "[libAP_JSON] Duplicate input frame count" << fcu_frame_count << "\n";
        return false;
    }
    else if (pkt.frame_count != fcu_frame_count + 1 && ap_online) {
        // missed frames, warn only
        std::cout << "[libAP_JSON] Missed " << fcu_frame_count - pkt.frame_count << " input frames\n";
    }
    fcu_frame_count = pkt.frame_count;

    // always reset the connection timeout so we don't accumulate
    connectionTimeoutCount = 0;
    if (!ap_online) {
        ap_online = true;

        std::cout << "[libAP_JSON] " << "Connected to ArduPilot controller @ "
                  << fcu_address << ":" << fcu_port_out << "\n";
    }

    for (int i = 0; i < MAX_SERVO_CHANNELS - 1; i++) {
        servo_out[i] = pkt.pwm[i];
    }

    return true;
}

void libAP_JSON::SendState(double timestamp, // seconds
                           double gyro_x, double gyro_y, double gyro_z, // rad/sec
                           double accel_x, double accel_y, double accel_z, // m/s^2
                           double pos_x, double pos_y, double pos_z, // m in inertial frame
                           double phi, double theta, double psi, // attitude radians
                           double V_x, double V_y, double V_z) // m/s (standard fixed wing frame is negative)
{
    // it is assumed that the imu orientation is NED, i.e.:
    //   x forward
    //   y right
    //   z down

    // Example ArduPilot JSON interface messages. Preceed and terminate with \n
    // {"timestamp":2500,"imu":{"gyro":[0,0,0],"accel_body":[0,0,0]},"position":[0,0,0],"attitude":[0,0,0],"velocity":[0,0,0]}

    // using namespace rapidjson;

    // build JSON string
    // all the required pieces first
    std::string s =  "\n{\"timestamp\":" + std::to_string(timestamp)
                    + ",\"imu\":{"
                        + "\"gyro\":[" + std::to_string(gyro_x) + "," + std::to_string(gyro_y) + "," + std::to_string(gyro_z) + "]"
                        + ",\"accel_body\":[" + std::to_string(accel_x) + "," + std::to_string(accel_y) + "," + std::to_string(accel_z) + "]"
                        + "}"
                    + ",\"position\":[" + std::to_string(pos_x) + "," + std::to_string(pos_y) + "," + std::to_string(pos_z) + "]"
                    + ",\"attitude\":[" + std::to_string(phi) + "," + std::to_string(theta) + "," + std::to_string(psi) + "]"
                    + ",\"velocity\":[" + std::to_string(V_x) + "," + std::to_string(V_y) + "," + std::to_string(V_z) + "]";
    
    // handle the optional JSON data
    if (set_airspeed_flag) {
        s = s + ",\"airspeed\":" + std::to_string(airspeed);
    }
    if (set_windvane_flag) {
        s = s + ",\"windvane\":{\"direction\":" + std::to_string(windvane_direction) + ",\"speed\":" + std::to_string(windvane_speed) + "}";
    }
    if (rangefinder_count > 0) {
        for (int i = 0; i < rangefinder_count; i++) {
            // rangefinder data is 1 indexed
            s = s + ",\"rng_" + std::to_string(i + 1) + "\":" + std::to_string(rangefinder[i]);
        }
    }

    s = s + "}\n"; // close the JSON
    
    // send JSON string to ArduPilot
    auto bytes_sent = sock.sendto(
        s.c_str(), s.size(),
        fcu_address,
        fcu_port_out);

#if DEBUG_ENABLED
    std::cout << s << std::endl;
#endif
}

void libAP_JSON::setAirspeed(double airspeed_in)
{
    airspeed = airspeed_in;
    set_airspeed_flag = true;
}

void libAP_JSON::setWindvane(double direction, double speed)
{
    windvane_direction = direction;
    windvane_speed = speed;
    set_windvane_flag = true;
}

void libAP_JSON::setRangefinder(double *rangefinder_in, uint8_t n)
{
    if (n <= 6) {
        rangefinder_count = n;
        for (int i = 0; i < n; i++) {
            rangefinder[i] = rangefinder_in[i];
        }
    } else {
        std::cout << "[libAP_JSON] Too many rangerfinder values!" << std::endl;
    }
}
