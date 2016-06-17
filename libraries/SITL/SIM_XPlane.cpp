/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  simulator connector for XPlane
*/

#include "SIM_XPlane.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

XPlane::XPlane(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
{
    use_time_sync = false;
    const char *colon = strchr(frame_str, ':');
    if (colon) {
        xplane_ip = colon+1;
    }
    socket_in.bind("0.0.0.0", bind_port);
}

/*
  receive data from X-Plane via UDP
*/
bool XPlane::receive_data(void)
{
    uint8_t pkt[10000];
    uint8_t *p = &pkt[5];
    const uint8_t pkt_len = 36;
    uint64_t data_mask = 0;
    const uint64_t required_mask = (1U<<Times | 1U<<LatLonAlt | 1U<<PitchRollHeading | 1U<<LocVelDistTraveled | 1U<<AngularVelocities);
    Location loc {};
    Vector3f pos;
    ssize_t len = socket_in.recv(pkt, sizeof(pkt), 1);
    
    if (len < pkt_len+5 || memcmp(pkt, "DATA@", 5) != 0) {
        // not a data packet we understand
        goto failed;
    }
    len -= 5;

    while (len >= pkt_len) {
        const float *data = (const float *)p;
        uint8_t code = p[0];
        // keep a mask of what codes we have received
        if (code < 64) {
            data_mask |= (1U << code);
        }
        switch (code) {
        case Times: {
            uint64_t tus = data[3] * 1.0e6f;
            if (tus + time_base_us <= time_now_us) {
                uint64_t tdiff = time_now_us - (tus + time_base_us);
                if (tdiff > 1e6) {
                    printf("X-Plane time reset %lu\n", (unsigned long)tdiff);
                }
                time_base_us = time_now_us - tus;
            }
            uint64_t tnew = time_base_us + tus;
            //uint64_t dt = tnew - time_now_us;
            //printf("dt %u\n", (unsigned)dt);
            time_now_us = tnew;
            break;
        }
            
        case LatLonAlt: {
            loc.lat = data[1] * 1e7;
            loc.lng = data[2] * 1e7;
            loc.alt = data[3] * FEET_TO_METERS * 100.0f;
            float hagl = data[3] * FEET_TO_METERS;
            ground_level = loc.alt * 0.01f - hagl;
            break;
        }

        case Speed:
            airspeed = data[2] * KNOTS_TO_METERS_PER_SECOND;
            airspeed_pitot = airspeed;
            break;

        case AoA:
            // ignored
            break;

        case PitchRollHeading: {
            float roll, pitch, yaw;
            pitch = radians(data[1]);
            roll = radians(data[2]);
            yaw = radians(data[3]);
            dcm.from_euler(roll, pitch, yaw);
            break;
        }

        case AtmosphereWeather:
            // ignored
            break;

        case LocVelDistTraveled:
            pos.y = data[1];
            pos.z = data[2];
            pos.x = -data[3];
            velocity_ef.y = data[4];
            velocity_ef.z = -data[5];
            velocity_ef.x = -data[6];
            break;

        case AngularVelocities:
            gyro.y = data[1];
            gyro.x = data[2];
            gyro.z = data[3];
            break;

        case Gload:
            accel_body.z = -data[5] * GRAVITY_MSS;
            accel_body.x = data[6] * GRAVITY_MSS;
            accel_body.y = data[7] * GRAVITY_MSS;
            break;

        case Joystick1:
            rcin_chan_count = 4;
            rcin[0] = (data[2] + 1)*0.5f;
            rcin[1] = (data[1] + 1)*0.5f;
            rcin[3] = (data[3] + 1)*0.5f;
            break;

        case ThrottleCommand: {
            if (data[1] < 0 ||
                data[1] == throttle_sent ||
                ((uint32_t)(data[1] * 1e6)) % 1000 == throttle_magic) {
                break;
            }
            rcin[2] = data[1];
            break;
        }
            
        case Joystick2:
            break;
            
        }
        len -= pkt_len;
        p += pkt_len;
    }

    if ((data_mask & required_mask) != required_mask) {
        printf("Not receiving all required data, missing 0x%08lx\n", (unsigned long)(required_mask & ~data_mask));
        goto failed;
    }
    position = pos + position_zero;
    position.z = -loc.alt * 0.01f;
    update_position();

    accel_earth = dcm * accel_body;
    accel_earth.z += GRAVITY_MSS;
    
    // the position may slowly deviate due to float accuracy and longitude scaling
    if (get_distance(loc, location) > 4) {
        printf("X-Plane home reset dist=%f alt=%f\n", get_distance(loc, location), pos.z);
        // reset home location
        position_zero(-pos.x, -pos.y, 0);
        home.lat = loc.lat;
        home.lng = loc.lng;
        home.alt = 0;
        position.x = 0;
        position.y = 0;
        update_position();
    }

    update_mag_field_bf();

    last_data_time_ms = AP_HAL::millis();
    return true;
        
failed:
    if (AP_HAL::millis() - last_data_time_ms > 200) {
        // don't extrapolate beyond 0.2s
        return false;
    }

    // advance time by 1ms
    Vector3f rot_accel;
    frame_time_us = 1000;
    float delta_time = frame_time_us * 1e-6f;

    time_now_us += frame_time_us;

    // extrapolate sensors
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0,0,-GRAVITY_MSS));

    // new velocity and position vectors
    velocity_ef += accel_earth * delta_time;
    position += velocity_ef * delta_time;
    velocity_air_ef = velocity_ef - wind_ef;
    velocity_air_bf = dcm.transposed() * velocity_air_ef;

    update_position();
    update_mag_field_bf();
    return false;
}
    

/*
  send data to X-Plane via UDP
*/
void XPlane::send_data(const struct sitl_input &input)
{
    if (!connected) {
        uint16_t port;
        socket_in.last_recv_address(xplane_ip, port);
        socket_out.connect(xplane_ip, xplane_port);
        connected = true;
        printf("Connected to %s:%u\n", xplane_ip, (unsigned)xplane_port);
    }

    float aileron  = (input.servos[0]-1500)/500.0f;
    float elevator = (input.servos[1]-1500)/500.0f;
    float throttle = (input.servos[2]-1000)/1000.0;
    float rudder   = (input.servos[3]-1500)/500.0f;
    struct PACKED {
        uint8_t  marker[5] { 'D', 'A', 'T', 'A', '0' };
        uint32_t code;
        float    data[8];
    } d {};

    throttle += throttle_magic * 1e-6f;
    
    d.code = 11;
    d.data[0] = elevator;
    d.data[1] = aileron;
    d.data[2] = rudder;
    d.data[4] = rudder;
    socket_out.send(&d, sizeof(d));

    d.code = 25;
    d.data[0] = throttle;
    d.data[1] = throttle;
    d.data[2] = throttle;
    d.data[3] = throttle;
    d.data[4] = 0;
    socket_out.send(&d, sizeof(d));

    throttle_sent = throttle;
}
    
/*
  update the XPlane simulation by one time step
 */
void XPlane::update(const struct sitl_input &input)
{
    if (receive_data()) {
        send_data(input);
    }
    
}

} // namespace SITL
