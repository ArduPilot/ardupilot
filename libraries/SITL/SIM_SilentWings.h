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

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  A wrapper for a simulated Silent Wings aircraft.
*/
class SilentWings : public Aircraft {
public:
    SilentWings(const char *frame_str);

    /* Updates the aircraft model by one time step */
    void update(const struct sitl_input &input) override;

    /* Static object creator */
    static Aircraft *create(const char *frame_str) {
        return new SilentWings(frame_str);
    }

private:

    /* Reply packet sent from SilentWings to ArduPlane */
    struct PACKED fdm_packet {
           unsigned int timestamp;          // Millisec  Timestamp
           double position_latitude;        // Degrees   Position latitude,
           double position_longitude;       // Degrees            longitude,
           float  altitude_msl;             // m         Altitude w.r.t. the sea level
           float  altitude_ground;          // m         Altitude w.r.t. the ground level 
           float  altitude_ground_45;       // m         Ground 45 degrees ahead (NOT IMPLEMENTED YET)
           float  altitude_ground_forward;  // m         Ground straight ahead (NOT IMPLEMENTED YET)
           float  roll;                     // Degrees
           float  pitch;                    // Degrees
           float  yaw;                      // Degrees
           float  d_roll;                   // Deg/sec   Roll speed
           float  d_pitch;                  // Deg/sec   Pitch speed
           float  d_yaw;                    // Deg/sec   Yaw speed
           float  vx;                       // m/s       Velocity vector in body-axis
           float  vy; 
           float  vz;                
           float  vx_wind;                  // m/s       Velocity vector in body-axis, relative to the wind
           float  vy_wind;
           float  vz_wind; 
           float  v_eas;                    // m/s       Equivalent (indicated) air speed.
           float  ax;                       // m/s^2     Acceleration vector in body axis
           float  ay;
           float  az;
           float  angle_of_attack;          // Degrees   Angle of attack
           float  angle_sideslip;           // Degrees   Sideslip angle
           float  vario;                    // m/s       Total energy-compensated variometer
           float  heading;                  // Degrees   Compass heading
           float  rate_of_turn;             // Deg/sec   Rate of turn
           float  airpressure;              // Pa        Local air pressure at aircraft altitude)
           float  density;                  // Air density at aircraft altitude
           float  temperature;              // Degrees Celcius   Air temperature at aircraft altitude
    } pkt;
    
    struct {
        uint32_t last_report_ms;
        uint32_t data_count;
        uint32_t frame_count;
    } report;

    bool recv_fdm(void);
    void process_packet();
    bool interim_update();
	
    /*  Create and set in/out socket for Silent Wings simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;
    
    /* Sends control inputs to the Silent Wings. */
    void send_servos(const struct sitl_input &input);

    /* Timestamp of the latest data packet received from Silent Wings. */
    uint32_t last_data_time_ms;
    
    /* Timestamp of the first data packet received from Silent Wings. */
    uint32_t first_pkt_timestamp_ms;
    
    /* Indicates whether first_pkt_timestamp_ms has been initialized (i.e., any packets have been received from Silent Wings. */ 
    bool inited_first_pkt_timestamp;
    
    /* ArduPlane's internal time when the first packet from Silent Wings is received. */
    uint64_t time_base_us;
    
    SocketAPM sock;
    const char *_sw_address = "127.0.0.1";
    int _port_in = 6060;
    int _sw_port = 6070;

    bool home_initialized;
};

} // namespace SITL
