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
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

XPlane::XPlane(const char *frame_str) :
    Aircraft(frame_str)
{
    use_time_sync = false;
    const char *colon = strchr(frame_str, ':');
    if (colon) {
        xplane_ip = colon+1;
    }

    heli_frame = (strstr(frame_str, "-heli") != nullptr);
    num_motors = 2;

    socket_in.bind("0.0.0.0", bind_port);
    printf("Waiting for XPlane data on UDP port %u and sending to port %u\n",
           (unsigned)bind_port, (unsigned)xplane_port);

    // XPlane sensor data is not good enough for EKF. Use fake EKF by default
    AP_Param::set_default_by_name("AHRS_EKF_TYPE", 10);
    AP_Param::set_default_by_name("INS_GYR_CAL", 0);
}

/*
 change what data is requested from XPlane. This saves the user from
 having to setup the data screen correctly
 */
void XPlane::select_data(uint64_t usel_mask, uint64_t sel_mask)
{
    struct PACKED {
        uint8_t  marker[5] { 'D', 'S', 'E', 'L', '0' };
        uint32_t data[8] {};
    } dsel;
    uint8_t count = 0;
    for (uint8_t i=0; i<64 && count<8; i++) {
        if ((((uint64_t)1)<<i) & sel_mask) {
            dsel.data[count++] = i;
            printf("i=%u\n", (unsigned)i);
        }
    }
    if (count != 0) {
        socket_out.send(&dsel, sizeof(dsel));
        printf("Selecting %u data types 0x%llx\n", (unsigned)count, (unsigned long long)sel_mask);
    }

    struct PACKED {
        uint8_t  marker[5] { 'U', 'S', 'E', 'L', '0' };
        uint32_t data[8] {};
    } usel;
    count = 0;

    // only de-select an output once, so we don't fight the user
    usel_mask &= ~unselected_mask;
    unselected_mask |= usel_mask;

    for (uint8_t i=0; i<64 && count<8; i++) {
        if ((((uint64_t)1)<<i) & usel_mask) {
            usel.data[count++] = i;
        }
    }
    if (count != 0) {
        socket_out.send(&usel, sizeof(usel));
        printf("De-selecting %u data types 0x%llx\n", (unsigned)count, (unsigned long long)usel_mask);
    }
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
    const uint64_t one = 1U;
    const uint64_t required_mask = (one<<Times | one<<LatLonAlt | one<<Speed | one<<PitchRollHeading |
                                    one<<LocVelDistTraveled | one<<AngularVelocities | one<<Gload |
                                    one << Joystick1 | one << ThrottleCommand | one << Trim |
                                    one << PropPitch | one << EngineRPM | one << PropRPM | one << Generator |
                                    one << Mixture);
    Location loc {};
    Vector3f pos;
    uint32_t wait_time_ms = 1;
    uint32_t now = AP_HAL::millis();

    // if we are about to get another frame from X-Plane then wait longer
    if (xplane_frame_time > wait_time_ms &&
        now+1 >= last_data_time_ms + xplane_frame_time) {
        wait_time_ms = 10;
    }
    ssize_t len = socket_in.recv(pkt, sizeof(pkt), wait_time_ms);
    
    if (len < pkt_len+5 || memcmp(pkt, "DATA", 4) != 0) {
        // not a data packet we understand
        goto failed;
    }
    len -= 5;

    if (!connected) {
        // we now know the IP X-Plane is using
        uint16_t port;
        socket_in.last_recv_address(xplane_ip, port);
        socket_out.connect(xplane_ip, xplane_port);
        connected = true;
        printf("Connected to %s:%u\n", xplane_ip, (unsigned)xplane_port);
    }
    
    while (len >= pkt_len) {
        const float *data = (const float *)p;
        uint8_t code = p[0];
        // keep a mask of what codes we have received
        if (code < 64) {
            data_mask |= (((uint64_t)1) << code);
        }
        switch (code) {
        case Times: {
            uint64_t tus = data[3] * 1.0e6f;
            if (tus + time_base_us <= time_now_us) {
                uint64_t tdiff = time_now_us - (tus + time_base_us);
                if (tdiff > 1e6f) {
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
            const float altitude_above_ground = data[4] * FEET_TO_METERS;
            ground_level = loc.alt * 0.01f - altitude_above_ground;
            break;
        }

        case Speed:
            airspeed = data[2] * KNOTS_TO_METERS_PER_SECOND;
            airspeed_pitot = airspeed;
            break;

        case AoA:
            // ignored
            break;

        case Trim:
            if (heli_frame) {
                // use flaps for collective as no direct collective data input
                rcin[2] = data[4];
            }
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
            pos.z = -data[2];
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
            if (!heli_frame) {
                /* getting joystick throttle input is very weird. The
                 * problem is that XPlane sends the ThrottleCommand packet
                 * both for joystick throttle input and for throttle that
                 * we have provided over the link. So we need some way to
                 * detect when we get our own values back. The trick used
                 * is to add throttle_magic to the values we send, then
                 * detect this offset in the data coming back. Very ugly,
                 * but I can't find a better way of allowing joystick
                 * input from XPlane10
                 */
                bool has_magic = ((uint32_t)(data[1] * throttle_magic_scale) % 1000U) == (uint32_t)(throttle_magic * throttle_magic_scale);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
                if (data[1] < 0 ||
                    data[1] == throttle_sent ||
                    has_magic) {
                    break;
                }
#pragma GCC diagnostic pop
                rcin[2] = data[1];
            }
            break;
        }

        case PropPitch: {
            break;
        }

        case EngineRPM:
            rpm[0] = data[1];
            break;

        case PropRPM:
            rpm[1] = data[1];
            break;
            
        case Joystick2:
            break;

        case Generator:
            /*
              in order to get interlock switch on helis we map the
              "generator1 on/off" function of XPlane 10 to channel 8.
             */
            rcin_chan_count = 8;
            rcin[7] = data[1];
            break;

        case Mixture:
            // map channel 6 and 7 from Mixture3 and Mixture4 for extra channels
            rcin_chan_count = MAX(7, rcin_chan_count);
            rcin[5] = data[3];
            rcin[6] = data[4];
            break;
        }
        len -= pkt_len;
        p += pkt_len;
    }

    if (data_mask != required_mask) {
        // ask XPlane to change what data it sends
        uint64_t usel = data_mask & ~required_mask;
        uint64_t sel = required_mask & ~data_mask;
        usel &= ~unselected_mask;
        if (usel || sel) {
            select_data(usel, sel);
            goto failed;
        }
    }
    position = pos + position_zero;
    update_position();
    time_advance();

    accel_earth = dcm * accel_body;
    accel_earth.z += GRAVITY_MSS;
    
    // the position may slowly deviate due to float accuracy and longitude scaling
    if (loc.get_distance(location) > 4 || abs(loc.alt - location.alt)*0.01f > 2.0f) {
        printf("X-Plane home reset dist=%f alt=%.1f/%.1f\n",
               loc.get_distance(location), loc.alt*0.01f, location.alt*0.01f);
        // reset home location
        position_zero(-pos.x, -pos.y, -pos.z);
        home.lat = loc.lat;
        home.lng = loc.lng;
        home.alt = loc.alt;
        position.x = 0;
        position.y = 0;
        position.z = 0;
        update_position();
        time_advance();
    }

    update_mag_field_bf();

    if (now > last_data_time_ms && now - last_data_time_ms < 100) {
        xplane_frame_time = now - last_data_time_ms;
    }
    last_data_time_ms = AP_HAL::millis();

    report.data_count++;
    report.frame_count++;
    return true;
        
failed:
    if (AP_HAL::millis() - last_data_time_ms > 200) {
        // don't extrapolate beyond 0.2s
        return false;
    }

    // advance time by 1ms
    frame_time_us = 1000;
    float delta_time = frame_time_us * 1e-6f;

    time_now_us += frame_time_us;

    extrapolate_sensors(delta_time);
    
    update_position();
    time_advance();
    update_mag_field_bf();
    report.frame_count++;
    return false;
}
    

/*
  send data to X-Plane via UDP
*/
void XPlane::send_data(const struct sitl_input &input)
{
    float aileron  = (input.servos[0]-1500)/500.0f;
    float elevator = (input.servos[1]-1500)/500.0f;
    float throttle = (input.servos[2]-1000)/1000.0;
    float rudder   = (input.servos[3]-1500)/500.0f;
    struct PACKED {
        uint8_t  marker[5] { 'D', 'A', 'T', 'A', '0' };
        uint32_t code;
        float    data[8];
    } d {};

    if (input.servos[0] == 0) {
        aileron = 0;
    }
    if (input.servos[1] == 0) {
        elevator = 0;
    }
    if (input.servos[2] == 0) {
        throttle = 0;
    }
    if (input.servos[3] == 0) {
        rudder = 0;
    }
    
    // we add the throttle_magic to the throttle value we send so we
    // can detect when we get it back
    throttle = ((uint32_t)(throttle * 1000)) * 1.0e-3f + throttle_magic;
    
    uint8_t flap_chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_flap, flap_chan) ||
        SRV_Channels::find_channel(SRV_Channel::k_flap_auto, flap_chan)) {
        float flap = (input.servos[flap_chan]-1000)/1000.0;
        if (!is_equal(flap, last_flap)) {
            send_dref("sim/flightmodel/controls/flaprqst", flap);
            send_dref("sim/aircraft/overflow/acf_flap_arm", flap>0?1:0);
        }
    }

    d.code = FlightCon;
    d.data[0] = elevator;
    d.data[1] = aileron;
    d.data[2] = rudder;
    d.data[4] = rudder;
    socket_out.send(&d, sizeof(d));

    if (!heli_frame) {
        d.code = ThrottleCommand;
        d.data[0] = throttle;
        d.data[1] = throttle;
        d.data[2] = throttle;
        d.data[3] = throttle;
        d.data[4] = 0;
        socket_out.send(&d, sizeof(d));
    } else {
        // send chan3 as collective pitch, on scale from -10 to +10
        float collective = 10*(input.servos[2]-1500)/500.0;

        // and send throttle from channel 8
        throttle = (input.servos[7]-1000)/1000.0;

        // allow for extra throttle outputs for special aircraft
        float throttle2 = (input.servos[5]-1000)/1000.0;
        float throttle3 = (input.servos[6]-1000)/1000.0;

        d.code = PropPitch;
        d.data[0] = collective;
        d.data[1] = -rudder*15; // reverse sense of rudder, 15 degrees pitch range
        d.data[2] = 0;
        d.data[3] = 0;
        d.data[4] = 0;
        socket_out.send(&d, sizeof(d));

        d.code = ThrottleCommand;
        d.data[0] = throttle;
        d.data[1] = throttle;
        d.data[2] = throttle2;
        d.data[3] = throttle3;
        d.data[4] = 0;
        socket_out.send(&d, sizeof(d));
    }

    throttle_sent = throttle;
}


/*
  send DREF to X-Plane via UDP
*/
void XPlane::send_dref(const char *name, float value)
{
    struct PACKED {
        uint8_t  marker[5] { 'D', 'R', 'E', 'F', '0' };
        float value;
        char name[500];
    } d {};
    d.value = value;
    strcpy(d.name, name);
    socket_out.send(&d, sizeof(d));        
}
    
/*
  update the XPlane simulation by one time step
 */
void XPlane::update(const struct sitl_input &input)
{
    if (receive_data()) {
        send_data(input);
    }

    uint32_t now = AP_HAL::millis();
    if (report.last_report_ms == 0) {
        report.last_report_ms = now;
    }
    if (now - report.last_report_ms > 5000) {
        float dt = (now - report.last_report_ms) * 1.0e-3f;
        printf("Data rate: %.1f FPS  Frame rate: %.1f FPS\n",
               report.data_count/dt, report.frame_count/dt);
        report.last_report_ms = now;
        report.data_count = 0;
        report.frame_count = 0;
    }
}

} // namespace SITL
