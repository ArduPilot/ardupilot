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
  simulator connector for ardupilot version of SilentWings
*/

#include "SIM_SilentWings.h"

#if HAL_SIM_SILENTWINGS_ENABLED

#include <stdio.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

static const struct {
    const char *name;
    float value;
    bool save;
} sim_defaults[] = {
    { "AHRS_EKF_TYPE", 10 },
    { "INS_GYR_CAL", 0 },
    { "EK2_ENABLE", 0 },
    { "ARSPD_ENABLE", 1 },
    { "ARSPD_USE", 1 },
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

SilentWings::SilentWings(const char *frame_str) :
    Aircraft(frame_str),
    last_data_time_ms(0),
    first_pkt_timestamp_ms(0),
    time_base_us(0),
    sock(true),
    home_initialized(false),
    inited_first_pkt_timestamp(false)
{
    // Force ArduPlane to use sensor data from SilentWings as the actual state,
    // without using EKF, i.e., using "fake EKF (type 10)". Disable gyro calibration.
    // Set a few other parameters to specific values to keep the calibration checks happy.
    // TO DO: fix this. Setting parameters in this way doesn't appear to have any effect.
    for (uint8_t i = 0; i < ARRAY_SIZE(sim_defaults); i++) {
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
  Create and set in/out socket
*/
void SilentWings::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    // Ignore the specified port_in.
    // try to bind to a specific port so that if we restart ArduPilot
    // Gazebo keeps sending us packets. Not strictly necessary but
    // useful for debugging
    if (!sock.bind("127.0.0.1", _port_in)) {
        fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", _port_in, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    printf("Bind %s:%d for SITL in\n", "127.0.0.1", _port_in);
    sock.reuseaddress();
    sock.set_blocking(false);

    _sw_address = address;
    // Ignore the specified port_out.
    printf("Setting Silent Wings interface to %s:%d \n", _sw_address, _sw_port);
}


/*
  decode and send servos
*/
void SilentWings::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    // Turn off direct joystick input to the simulator. All joystick commands
    // should go through Mission Planner and get properly fused with ArduPlane's
    // control inputs when in automatic flight modes.
    float joystick = 0.0f;
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float throttle = filtered_servo_range(input, 2);
    float rudder   = filtered_servo_angle(input, 3);

    ssize_t buflen = asprintf(&buf,
             "JOY %f\n"
             "AIL %f\n"
             "ELE %f\n"
             "RUD %f\n"
             "THR %f\n",
             joystick, aileron, elevator, rudder, throttle) - 1;

    if (buflen < 0) {
        fprintf(stderr, "Fatal: Failed to allocate enough space for data\n"),
        exit(1);
    }

    ssize_t sent = sock.sendto(buf, buflen, _sw_address, _sw_port);
    free(buf);

    if (sent < 0) {
        fprintf(stderr, "Fatal: Failed to send on control socket\n"),
        exit(1);
    }

    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }
}


/*
  Receive an update from the FDM
  This is a blocking function
 */
bool SilentWings::recv_fdm(void)
{
    fdm_packet tmp_pkt;
    memset(&pkt, 0, sizeof(pkt));

    ssize_t nread = sock.recv(&tmp_pkt, sizeof(pkt), 0);

    // nread == -1 (255) means no data has arrived
    if (nread != sizeof(pkt)) {
        return false;
    }

    memcpy(&pkt, &tmp_pkt, sizeof(pkt));

    // data received successfully
    return true;
}


void SilentWings::process_packet()
{
    // pkt.timestamp is the time of day in SilentWings, measured in ms
    // since midnight.
    // TO DO: check what happens when a flight in SW crosses midnight
    if (inited_first_pkt_timestamp) {
        uint64_t tus = (pkt.timestamp - first_pkt_timestamp_ms) * 1.0e3f;
        time_now_us = time_base_us + tus;
    }
    else {
        first_pkt_timestamp_ms = pkt.timestamp;
        time_base_us = time_now_us;
        inited_first_pkt_timestamp = true;
    }

    dcm.from_euler(radians(pkt.roll), radians(pkt.pitch), radians(pkt.yaw));
    accel_body = Vector3f(pkt.ax * GRAVITY_MSS, pkt.ay * GRAVITY_MSS, pkt.az * GRAVITY_MSS); // This is g-load.
    gyro = Vector3f(radians(pkt.d_roll), radians(pkt.d_pitch), radians(pkt.d_yaw));
    // SilentWings provides velocity in body frame.
    velocity_ef = dcm * Vector3f(pkt.vx, pkt.vy, pkt.vz);
    // SilentWings also provides velocity in body frame w.r.t. the wind, from which we can infer the wind.
    wind_ef = dcm * (Vector3f(pkt.vx, pkt.vy, pkt.vz) - Vector3f(pkt.vx_wind, pkt.vy_wind, pkt.vz_wind));
    airspeed = pkt.v_eas;
    airspeed_pitot = pkt.v_eas;
    Location curr_location;
    curr_location.lat = pkt.position_latitude * 1.0e7;
    curr_location.lng = pkt.position_longitude * 1.0e7;
    curr_location.alt = pkt.altitude_msl * 100.0f;
    ground_level = curr_location.alt * 0.01f - pkt.altitude_ground;
    Vector3f posdelta = origin.get_distance_NED(curr_location);
    position.x = posdelta.x;
    position.y = posdelta.y;
    position.z = posdelta.z;
    update_position();

    // In case Silent Wings' reported location and our location calculated using an offset from the home location diverge, we need
    // to reset the home location.
    if (curr_location.get_distance(location) > 4 || abs(curr_location.alt - location.alt)*0.01f > 2.0f || !home_initialized) {
        printf("SilentWings home reset dist=%f alt=%.1f/%.1f\n",
               curr_location.get_distance(location), curr_location.alt*0.01f, location.alt*0.01f);
        // reset home location
        home.lat = curr_location.lat;
        home.lng = curr_location.lng;
        origin.lat = home.lat;
        origin.lng = home.lng;
        // Resetting altitude reference point in flight can throw off a bunch
        // of important calculations, so let the home altitude always be 0m MSL
        home.alt = 0;
        position.x = 0;
        position.y = 0;
        position.z = -curr_location.alt;
        home_initialized = true;
        update_position();
    }

    // Auto-adjust to Silent Wings' frame rate
    // This affects the data rate (without this adjustment, the data rate is
    // low no matter what the output_udp_rate in SW's options.dat file is).
    double deltat = (AP_HAL::millis() - last_data_time_ms) / 1000.0f;

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(1.0/deltat);
    }

    last_data_time_ms = AP_HAL::millis();

    report.data_count++;
    report.frame_count++;

    if (0) {
        printf("Delta: %f Time: %" PRIu64 "\n", deltat, time_now_us);
        printf("Accel.x %f\n", accel_body.x);
        printf("Accel.y %f\n", accel_body.y);
        printf("Accel.z %f\n", accel_body.z);
        printf("Gyro.x  %f\n", gyro.x);
        printf("Gyro.y  %f\n", gyro.y);
        printf("Gyro.z  %f\n", gyro.z);
        printf("Pos.x %f\n",   position.x);
        printf("Pos.y %f\n",   position.y);
        printf("Pos.z %f\n",   position.z);
        printf("Roll %f\n",    pkt.roll);
        printf("Pitch %f\n",   pkt.pitch);
        printf("Yaw %f\n",     pkt.yaw);
    }
}

/*
  Extrapolates sensor data if Silent Wings hasn't sent us a data packet in a while.
 */
bool SilentWings::interim_update()
{
    if (AP_HAL::millis() - last_data_time_ms > 200) {
        // don't extrapolate beyond 0.2s
        return false;
    }

    float delta_time = frame_time_us * 1e-6f;
    time_now_us += frame_time_us;
    extrapolate_sensors(delta_time);
    update_position();
    report.frame_count++;
    return true;
}


/*
  Update the Silent Wings simulation by one time step.
 */
void SilentWings::update(const struct sitl_input &input)
{
    if (recv_fdm()) {
        process_packet();
        // Time has been advanced by process_packet(.)
        send_servos(input);
    }
    else if (interim_update()) {
        // This clause is triggered only if we previously
        // received at least one data packet.
        // Time has been advanced by interim_update(.)
        send_servos(input);
    }

    // This clause is triggered if and only if we haven't received
    // any data packets yet (and therefore didn't attempt
    // extrapolating data via interim_update(.) either).
    if (!inited_first_pkt_timestamp){
        time_advance();
    }
    else {
        if (use_time_sync) {
            sync_frame_time();
        }
    }

    update_mag_field_bf();

    uint32_t now = AP_HAL::millis();

    if (report.last_report_ms == 0) {
        report.last_report_ms = now;
        printf("Resetting last report time to now\n");
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

#endif  // HAL_SIM_SILENTWINGS_ENABLED
