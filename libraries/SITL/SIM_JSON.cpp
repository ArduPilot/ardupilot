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
    Simulator Connector for JSON based interfaces
*/

#include "SIM_config.h"

#if AP_SIM_JSON_ENABLED

#include "SIM_JSON.h"

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/replace.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Filesystem/AP_Filesystem.h>

#define UDP_TIMEOUT_MS 100

extern const AP_HAL::HAL& hal;

using namespace SITL;

static const struct {
    const char *name;
    float value;
    bool save;
} sim_defaults[] = {
    { "BRD_OPTIONS", 0},
    { "INS_GYR_CAL", 0 },
    { "INS_ACC2OFFS_X",    0.001 },
    { "INS_ACC2OFFS_Y",    0.001 },
    { "INS_ACC2OFFS_Z",    0.001 },
    { "INS_ACC2SCAL_X",    1.001 },
    { "INS_ACC2SCAL_Y",    1.001 },
    { "INS_ACC2SCAL_Z",    1.001 },
    { "INS_ACCOFFS_X",     0.001 },
    { "INS_ACCOFFS_Y",     0.001 },
    { "INS_ACCOFFS_Z",     0.001 },
    { "INS_ACCSCAL_X",     1.001 },
    { "INS_ACCSCAL_Y",     1.001 },
    { "INS_ACCSCAL_Z",     1.001 },
};


JSON::JSON(const char *frame_str) :
    Aircraft(frame_str),
    sock(true)
{
    printf("Starting SITL: JSON\n");

    const char *colon = strchr(frame_str, ':');
    if (colon) {
        target_ip = colon+1;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(sim_defaults); i++) {
    AP_Param::set_default_by_name(sim_defaults[i].name, sim_defaults[i].value);
        if (sim_defaults[i].save) {
            enum ap_var_type ptype;
            AP_Param *p = AP_Param::find(sim_defaults[i].name, &ptype);
            if (!p->configured()) {
                p->save();
            }
        }
    }
}

/*
    Create & set in/out socket
*/
void JSON::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    sock.set_blocking(false);
    sock.reuseaddress();

    if (strcmp("127.0.0.1",address) != 0) {
        target_ip = address;
    }
    control_port = port_out;

    printf("JSON control interface set to %s:%u\n", target_ip, control_port);
}

/*
    Decode and send servos
*/
void JSON::output_servos(const struct sitl_input &input)
{
    size_t pkt_size = 0;
    ssize_t send_ret = -1;
    if (SRV_Channels::have_32_channels()) {
      servo_packet_32 pkt;
      pkt.frame_rate = rate_hz;
      pkt.frame_count = frame_counter;
      for (uint8_t i=0; i<32; i++) {
          pkt.pwm[i] = input.servos[i];
      }
      pkt_size = sizeof(pkt);
      send_ret = sock.sendto(&pkt, pkt_size, target_ip, control_port);
    } else {
      servo_packet_16 pkt;
      pkt.frame_rate = rate_hz;
      pkt.frame_count = frame_counter;
      for (uint8_t i=0; i<16; i++) {
          pkt.pwm[i] = input.servos[i];
      }
      pkt_size = sizeof(pkt);
      send_ret = sock.sendto(&pkt, pkt_size, target_ip, control_port);
    }

    if ((size_t)send_ret != pkt_size) {
        if (send_ret <= 0) {
            printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
                   target_ip, control_port, strerror(errno), (long)send_ret);
        } else {
            printf("Sent %ld bytes instead of %lu bytes\n", (long)send_ret, (unsigned long)pkt_size);
        }
    }
}


// rate limit reports of bad sensor packets to 1Hz
bool JSON::report_parse_error()
{
    // wall clock time, as simulated time stands still while every
    // packet is rejected
    const uint64_t now_us = get_wall_time_us();
    if (now_us - last_parse_error_us < 1000000) {
        return false;
    }
    last_parse_error_us = now_us;
    return true;
}

uint64_t JSON::parse_sensors(const uint8_t *data, size_t len)
{
#if SITL_JSON_DEBUG && AP_FILESYSTEM_FILE_WRITING_ENABLED
    // it is useful in some environments to be able to get a copy of the raw
    // JSON data
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_debug_ms >= 1000) {
        // dump received JSON at 1Hz for easy debugging
        last_debug_ms = now_ms;
        auto &fs = AP::FS();
        int fd = fs.open("json_debug.txt", O_WRONLY|O_CREAT|O_TRUNC);
        if (fd != -1) {
            fs.write(fd, data, len);
            fs.close(fd);
        }
    }
#endif

    uint64_t received_bitmask = 0;
    size_t offset = 0;
    while (offset < len) {
        size_t used;
        const uint64_t found = handle_parse_result(parser.feed(&data[offset], len - offset, used));
        if (found != 0) {
            received_bitmask = found;
        }
        offset += used;
    }
    // a datagram ends the packet in it, so senders need not end packets
    // with a newline, and a datagram that is damaged or cut short can not
    // cost a packet in the next
    const uint64_t found = handle_parse_result(parser.end_of_message());
    if (found != 0) {
        received_bitmask = found;
    }
    return received_bitmask;
}

/*
  act on a result from the parser, returning the fields of the packet
  that became the current state, or 0
 */
uint64_t JSON::handle_parse_result(AP_JSON_FieldParser::Result result)
{
    switch (result) {
    case AP_JSON_FieldParser::Result::NONE:
        return 0;

    case AP_JSON_FieldParser::Result::PACKET: {
        const uint64_t missing = REQUIRED_FIELDS & ~parser.found();
        const uint64_t non_finite = non_finite_fields(parser.found() & ~NON_FINITE_ALLOWED);
        if (missing != 0 || non_finite != 0) {
            rejected_packets++;
            if (report_parse_error()) {
                if (missing != 0) {
                    print_fields("JSON: sensor packet missing:", missing);
                } else {
                    print_fields("JSON: sensor packet with infinity or NaN in:", non_finite);
                }
            }
            state_rx = state;
            return 0;
        }
        // a complete, valid packet: it becomes the current state
        state = state_rx;
        return parser.found();
    }

    case AP_JSON_FieldParser::Result::ERROR:
        rejected_packets++;
        if (report_parse_error()) {
            printf("JSON: rejected sensor packet: %s, %u rejected\n",
                   parser.error(), unsigned(rejected_packets));
        }
        // undo whatever the bad packet had written
        state_rx = state;
        return 0;
    }
    return 0;
}

// the fields of mask holding infinity or NaN in state_rx
uint64_t JSON::non_finite_fields(uint64_t mask) const
{
    uint64_t bad = 0;
    for (uint8_t f=0; f<ARRAY_SIZE(fields); f++) {
        const uint64_t bit = 1ULL << f;
        const AP_JSON_FieldParser::Field &field = fields[f];
        if ((mask & bit) == 0 || field.type == FT::BOOL) {
            continue;
        }
        const bool is_double = field.type == FT::DOUBLE || field.type == FT::DOUBLE_ARRAY;
        for (uint8_t i=0; i<field.count; i++) {
            const double v = is_double ? static_cast<const double *>(field.ptr)[i] :
                             double(static_cast<const float *>(field.ptr)[i]);
            if (!isfinite(v)) {
                bad |= bit;
            }
        }
    }
    return bad;
}

void JSON::print_fields(const char *msg, uint64_t mask) const
{
    printf("%s", msg);
    for (uint8_t f=0; f<ARRAY_SIZE(fields); f++) {
        if (mask & (1ULL << f)) {
            printf(" %s", fields[f].path);
        }
    }
    printf(", %u rejected\n", unsigned(rejected_packets));
}

/*
    Receive new sensor data from simulator
    This is a blocking function
*/
void JSON::recv_fdm(const struct sitl_input &input)
{
    // Receive sensor packet
    ssize_t ret = sock.recv(recv_buffer, sizeof(recv_buffer), UDP_TIMEOUT_MS);
    uint32_t wait_ms = UDP_TIMEOUT_MS;

    if (state.no_lockstep && ret <= 0) {
        // in no_lockstep mode do not block waiting for data; advance time using SITL loop rate
        if (sitl != nullptr && sitl->loop_rate_hz > 0) {
            frame_time_us = (uint32_t)(1000000.0f / sitl->loop_rate_hz);
        } else {
            frame_time_us = 10000; // fallback to 10ms
        }
        time_now_us += frame_time_us;
        time_advance();
        return;
    }

    while (ret <= 0) {
        //printf("No JSON sensor message received - %s\n", strerror(errno));
        ret = sock.recv(recv_buffer, sizeof(recv_buffer), UDP_TIMEOUT_MS);
        wait_ms += UDP_TIMEOUT_MS;
        // if no sensor message is received after 10 second resend servos, this help cope with SITL and the physics getting out of sync
        if (wait_ms > 1000) {
            wait_ms = 0;
            printf("No JSON sensor message received, resending servos\n");
            output_servos(input);
        }
    }

    // a datagram filling the buffer may have been truncated, and is
    // larger than any valid packet
    if (size_t(ret) >= sizeof(recv_buffer)) {
        rejected_packets++;
        if (report_parse_error()) {
            printf("JSON: sensor datagram too large, %u rejected\n", unsigned(rejected_packets));
        }
        return;
    }

    const uint64_t received_bitmask = parse_sensors(recv_buffer, ret);
    if (received_bitmask == 0) {
        // no complete valid packet yet; parse_sensors() reports bad ones
        return;
    }

    // Must get either attitude or quaternion fields
    if ((received_bitmask & (EULER_ATT | QUAT_ATT)) == 0) {
        printf("Did not receive attitude or quaternion\n");
        return;
    }

    if (received_bitmask != last_received_bitmask) {
        // some change in the message we have received, print what we got
        printf("\nJSON received:\n");
        for (uint8_t i=0; i<ARRAY_SIZE(fields); i++) {
            if ((received_bitmask & (1ULL << i)) != 0) {
                printf("\t%s\n", fields[i].path);
            }
        }
        printf("\n");
    }
    last_received_bitmask = received_bitmask;

    accel_body = state.imu.accel_body;
    gyro = state.imu.gyro;
    velocity_ef = state.velocity;

    if ((received_bitmask & (LATITUDE | LONGITUDE | ALTITUDE)) == (LATITUDE | LONGITUDE | ALTITUDE)) {
        Location new_loc;
        new_loc.lat = state.latitude * 1.0e7;
        new_loc.lng = state.longitude * 1.0e7;
        new_loc.alt = state.altitude * 100.0;
        new_loc.relative_alt = false;
        new_loc.origin_alt = false;
        new_loc.terrain_alt = false;

        if (!home_is_set) {
            set_start_location(new_loc, 0.0f);
        }

        position = origin.get_distance_NED_double(new_loc);
    } else {
        position = state.position;
        position.xy() += origin.get_distance_NE_double(home);
    }

    if (received_bitmask & TIME_SYNC) {
        if (use_time_sync != !state.no_time_sync) {
            use_time_sync = !state.no_time_sync;
            printf("Forcing use_time_sync=%d\n", int(use_time_sync));
            if (!use_time_sync) {
                // if not using time sync then default EKF type to 10, as
                // otherwise EKF is likely to diverge
                AP_Param::set_default_by_name("AHRS_EKF_TYPE", 10);
            }
        }
    }

    // Handle lockstep setting from JSON
    if (received_bitmask & LOCKSTEP) {
        // state.no_lockstep is true when we DON'T want lockstep
        // So we need to check if it changed and act accordingly
        if (state.no_lockstep != last_no_lockstep) {
            last_no_lockstep = state.no_lockstep;
            printf("Lockstep mode changed: no_lockstep=%d (lockstep %s)\n", 
                   int(state.no_lockstep), state.no_lockstep ? "DISABLED" : "ENABLED");
        }
    }

    // deal with euler or quaternion attitude
    if ((received_bitmask & QUAT_ATT) != 0) {
        // if we have a quaternion attitude use it rather than euler
        state.quaternion.rotation_matrix(dcm);
    } else {
        dcm.from_euler(state.attitude[0], state.attitude[1], state.attitude[2]);
    }

    if ((received_bitmask & AIRSPEED)) {
        // received airspeed directly
        airspeed = state.airspeed;

        airspeed_pitot = state.airspeed;
    } else {
        
        // wind is not supported yet for JSON sim, assume zero for now        
        wind_ef.zero(); 

        // velocity relative to airmass in Earth's frame
        velocity_air_ef = velocity_ef - wind_ef;

        // velocity relative to airmass in body frame
        velocity_air_bf = dcm.transposed() * velocity_air_ef;

        // airspeed fix for eas2tas
        update_eas_airspeed();
    }

    if ((received_bitmask & WIND_VEL) != 0) {
        wind_ef = state.velocity_wind;
    }

    // Convert from a meters from origin physics to a lat long alt
    update_position();

    // update range finder distances
    for (uint8_t i=0; i<6; i++) {
        if ((received_bitmask & (RNG_1 << i)) == 0) {
            continue;
        }
        rangefinder_m[i] = state.rng[i];
    }

    // update wind vane
    if ((received_bitmask & WIND_DIR) != 0) {
        wind_vane_apparent.direction = state.wind_vane_apparent.direction;
    }
    if ((received_bitmask & WIND_SPD) != 0) {
        wind_vane_apparent.speed = state.wind_vane_apparent.speed;
    }

    // update RC input
    static_assert(ARRAY_SIZE(state.rc) <= ARRAY_SIZE(rcin), "JSON rc in size mismatch");
    uint8_t rc_chan_count = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(state.rc); i++) {
        if ((received_bitmask & (RC_1 << i)) != 0) {
            rcin[i] = (state.rc[i] - 1000.0f) / 1000.0f;
            rc_chan_count = i+1;
        }
    }
    rcin_chan_count = rc_chan_count;

    // update battery state
    if ((received_bitmask & BAT_VOLT) != 0) {
        battery_voltage = state.bat_volt;
    } else {
        battery_voltage = sitl->batt_voltage;
    }
    if ((received_bitmask & BAT_AMP) != 0) {
        battery_current = state.bat_amp;
    } else {
        battery_current = 0.0f;
    }
    // (temperature is not part of the protocol, just set it explicitly here)
    battery_temperature_degC = 0.0f;

    double deltat;
    if (state.timestamp_s < last_timestamp_s) {
        // Physics time has gone backwards, don't reset AP
        printf("Detected physics reset\n");
        deltat = 0;
        last_received_bitmask = 0;
    } else {
        deltat = state.timestamp_s - last_timestamp_s;
    }
    uint64_t new_time_us = time_now_us;
    const bool physics_continues = frame_counter != 0 && state.timestamp_s >= base_timestamp_s;
    if (physics_continues) {
        new_time_us = base_time_us +
                      uint64_t(llround((state.timestamp_s - base_timestamp_s) * 1.0e6));
    }
    if (new_time_us <= time_now_us) {
        // first frame, physics reset, or SITL ran ahead without physics
        // (no_lockstep): re-base so SITL time never goes backwards.  When
        // the physics carried on, advance by its step, otherwise
        // time_advance() adds a SIM_RATE_HZ frame and SITL runs ahead again
        new_time_us = time_now_us;
        if (physics_continues) {
            new_time_us += uint64_t(llround(deltat * 1.0e6));
        }
        base_time_us = new_time_us;
        base_timestamp_s = state.timestamp_s;
    }
    time_now_us = new_time_us;

    if (is_positive(deltat) && deltat < 0.1) {
        // Only adjust frame time if we want lockstep
        // When no_lockstep is true, skip adjust_frame_time to let SITL run free
        if (use_time_sync && !state.no_lockstep) {
            // time in us to hz
            adjust_frame_time(1.0 / deltat);
        }
        // match actual frame rate with desired speedup
        time_advance();
    }
    last_timestamp_s = state.timestamp_s;
    frame_counter++;

#if 0

    float roll, pitch, yaw;
    if ((received_bitmask & QUAT_ATT) != 0) {
        dcm.to_euler(&roll, &pitch, &yaw);
    } else {
        roll = state.attitude[0];
        pitch = state.attitude[1];
        yaw = state.attitude[2];
    }

// @LoggerMessage: JSN1
// @Description: Log data received from JSON simulator
// @Field: TimeUS: Time since system startup (us)
// @Field: TStamp: Simulation's timestamp (s)
// @Field: R: Simulation's roll (rad)
// @Field: P: Simulation's pitch (rad)
// @Field: Y: Simulation's yaw (rad)
// @Field: GX: Simulated gyroscope, X-axis (rad/sec)
// @Field: GY: Simulated gyroscope, Y-axis (rad/sec)
// @Field: GZ: Simulated gyroscope, Z-axis (rad/sec)
    AP::logger().WriteStreaming("JSN1", "TimeUS,TStamp,R,P,Y,GX,GY,GZ",
                       "ssrrrEEE",
                       "F???????",
                       "Qfffffff",
                       AP_HAL::micros64(),
                       state.timestamp_s,
                       roll,
                       pitch,
                       yaw,
                       gyro.x,
                       gyro.y,
                       gyro.z);

    Vector3f accel_ef = dcm.transposed() * accel_body;

// @LoggerMessage: JSN2
// @Description: Log data received from JSON simulator
// @Field: TimeUS: Time since system startup (us)
// @Field: VN: simulation's velocity, North-axis (m/s)
// @Field: VE: simulation's velocity, East-axis (m/s)
// @Field: VD: simulation's velocity, Down-axis (m/s)
// @Field: AX: simulation's acceleration, X-axis (m/s^2)
// @Field: AY: simulation's acceleration, Y-axis (m/s^2)
// @Field: AZ: simulation's acceleration, Z-axis (m/s^2)
// @Field: AN: simulation's acceleration, North (m/s^2)
// @Field: AE: simulation's acceleration, East (m/s^2)
// @Field: AD: simulation's acceleration, Down (m/s^2)
    AP::logger().WriteStreaming("JSN2", "TimeUS,VN,VE,VD,AX,AY,AZ,AN,AE,AD",
                       "snnnoooooo",
                       "F?????????",
                       "Qfffffffff",
                       AP_HAL::micros64(),
                       velocity_ef.x,
                       velocity_ef.y,
                       velocity_ef.z,
                       accel_body.x,
                       accel_body.y,
                       accel_body.z,
                       accel_ef.x,
                       accel_ef.y,
                       accel_ef.z);
#endif

}

/*
   update the JSON simulation by one time step
*/
void JSON::update(const struct sitl_input &input)
{
    // send to JSON model
    output_servos(input);

    // receive from JSON model
    recv_fdm(input);

    // update magnetic field
    // as the model does not provide mag feild we calculate it from position and attitude
    update_mag_field_bf();

    // allow for changes in physics step
    // Only adjust frame time if lockstep is enabled (no_lockstep is false)
    if (use_time_sync && !state.no_lockstep) {
        adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));
    }

#if 0
    // report frame rate
    if (frame_counter % 1000 == 0) {
        printf("FPS %.2f\n", rate_hz); // this is instantaneous rather than any clever average
    }
#endif
}

#endif  // AP_SIM_JSON_ENABLED
