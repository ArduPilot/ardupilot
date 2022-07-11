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

#include "SIM_JSON.h"

#if HAL_SIM_JSON_ENABLED

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/replace.h>

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
    servo_packet pkt;
    pkt.frame_rate = rate_hz;
    pkt.frame_count = frame_counter;
    for (uint8_t i=0; i<16; i++) {
        pkt.pwm[i] = input.servos[i];
    }

    size_t send_ret = sock.sendto(&pkt, sizeof(pkt), target_ip, control_port);
    if (send_ret != sizeof(pkt)) {
        if (send_ret <= 0) {
            printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
                   target_ip, control_port, strerror(errno), (long)send_ret);
        } else {
            printf("Sent %ld bytes instead of %lu bytes\n", (long)send_ret, (unsigned long)sizeof(pkt));
        }
    }
}


/*
    very simple JSON parser for sensor data
    called with pointer to one row of sensor data, nul terminated

    This parser does not do any syntax checking, and is not at all
    general purpose
*/
uint32_t JSON::parse_sensors(const char *json)
{
    uint32_t received_bitmask = 0;

    //printf("%s\n", json);
    for (uint16_t i=0; i<ARRAY_SIZE(keytable); i++) {
        struct keytable &key = keytable[i];

        /* look for section header */
        const char *p = strstr(json, key.section);
        if (!p) {
            // we don't have this sensor
            if (key.required) {
                printf("Failed to find %s\n", key.section);
                return 0;
            }
            continue;
        }
        p += strlen(key.section)+1;

        // find key inside section
        p = strstr(p, key.key);
        if (!p) {
            if (key.required) {
                printf("Failed to find key %s/%s\n", key.section, key.key);
                return 0;
            }
            continue;
        }

        // record the keys that are found
        received_bitmask |= 1U << i;

        p += strlen(key.key)+2;
        switch (key.type) {
            case DATA_UINT64:
                *((uint64_t *)key.ptr) = strtoull(p, nullptr, 10);
                //printf("%s/%s = %lu\n", key.section, key.key, *((uint64_t *)key.ptr));
                break;

            case DATA_FLOAT:
                *((float *)key.ptr) = atof(p);
                //printf("%s/%s = %f\n", key.section, key.key, *((float *)key.ptr));
                break;

            case DATA_DOUBLE:
                *((double *)key.ptr) = atof(p);
                //printf("%s/%s = %f\n", key.section, key.key, *((double *)key.ptr));
                break;

            case DATA_VECTOR3F: {
                Vector3f *v = (Vector3f *)key.ptr;
                if (sscanf(p, "[%f, %f, %f]", &v->x, &v->y, &v->z) != 3) {
                    printf("Failed to parse Vector3f for %s/%s\n", key.section, key.key);
                    return received_bitmask;
                }
                //printf("%s/%s = %f, %f, %f\n", key.section, key.key, v->x, v->y, v->z);
                break;
            }

            case DATA_VECTOR3D: {
                Vector3d *v = (Vector3d *)key.ptr;
                if (sscanf(p, "[%lf, %lf, %lf]", &v->x, &v->y, &v->z) != 3) {
                    printf("Failed to parse Vector3f for %s/%s\n", key.section, key.key);
                    return received_bitmask;
                }
                //printf("%s/%s = %f, %f, %f\n", key.section, key.key, v->x, v->y, v->z);
                break;
            }

            case QUATERNION: {
                Quaternion *v = static_cast<Quaternion*>(key.ptr);
                if (sscanf(p, "[%f, %f, %f, %f]", &(v->q1), &(v->q2), &(v->q3), &(v->q4)) != 4) {
                    printf("Failed to parse Vector4f for %s/%s\n", key.section, key.key);
                    return received_bitmask;
                }
                break;
            }

            case BOOLEAN:
                *((bool *)key.ptr) = strtoull(p, nullptr, 10) != 0;
                //printf("%s/%s = %i\n", key.section, key.key, *((unit8_t *)key.ptr));
                break;

        }
    }

    return received_bitmask;
}

/*
    Receive new sensor data from simulator
    This is a blocking function
*/
void JSON::recv_fdm(const struct sitl_input &input)
{
    // Receive sensor packet
    ssize_t ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
    uint32_t wait_ms = UDP_TIMEOUT_MS;
    while (ret <= 0) {
        //printf("No JSON sensor message received - %s\n", strerror(errno));
        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
        wait_ms += UDP_TIMEOUT_MS;
        // if no sensor message is received after 10 second resend servos, this help cope with SITL and the physics getting out of sync
        if (wait_ms > 1000) {
            wait_ms = 0;
            printf("No JSON sensor message received, resending servos\n");
            output_servos(input);
        }
    }

    // convert '\n' into nul
    while (uint8_t *p = (uint8_t *)memchr(&sensor_buffer[sensor_buffer_len], '\n', ret)) {
        *p = 0;
    }
    sensor_buffer_len += ret;

    const uint8_t *p2 = (const uint8_t *)memrchr(sensor_buffer, 0, sensor_buffer_len);
    if (p2 == nullptr || p2 == sensor_buffer) {
        return;
    }

    const uint8_t *p1 = (const uint8_t *)memrchr(sensor_buffer, 0, p2 - sensor_buffer);
    if (p1 == nullptr) {
        return;
    }

    const uint32_t received_bitmask = parse_sensors((const char *)(p1+1));
    if (received_bitmask == 0) {
        // did not receive one of the mandatory fields
        printf("Did not contain all mandatory fields\n");
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
        for (uint16_t i=0; i<ARRAY_SIZE(keytable); i++) {
            struct keytable &key = keytable[i];
            if ((received_bitmask &  1U << i) == 0) {
                continue;
            }
            if (strcmp(key.section, "") == 0) {
                printf("\t%s\n",key.key);
            } else {
                printf("\t%s: %s\n",key.section,key.key);
            }
        }
        printf("\n");
    }
    last_received_bitmask = received_bitmask;

    memmove(sensor_buffer, p2, sensor_buffer_len - (p2 - sensor_buffer));
    sensor_buffer_len = sensor_buffer_len - (p2 - sensor_buffer);

    accel_body = state.imu.accel_body;
    gyro = state.imu.gyro;
    velocity_ef = state.velocity;
    position = state.position;
    position.xy() += origin.get_distance_NE_double(home);
    use_time_sync = !state.no_time_sync;

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
        // velocity relative to airmass in body frame
        velocity_air_bf = dcm.transposed() * velocity_ef;

        // airspeed
        airspeed = velocity_air_bf.length();

        // airspeed as seen by a fwd pitot tube (limited to 120m/s)
        airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 120.0f);
    }

    // Convert from a meters from origin physics to a lat long alt
    update_position();

    // update range finder distances
    for (uint8_t i=7; i<13; i++) {
        if ((received_bitmask &  1U << i) == 0) {
            continue;
        }
        rangefinder_m[i-7] = state.rng[i-7];
    }

    // update wind vane
    if ((received_bitmask & WIND_DIR) != 0) {
        wind_vane_apparent.direction = state.wind_vane_apparent.direction;
    }
    if ((received_bitmask & WIND_SPD) != 0) {
        wind_vane_apparent.speed = state.wind_vane_apparent.speed;
    }

    double deltat;
    if (state.timestamp_s < last_timestamp_s) {
        // Physics time has gone backwards, don't reset AP
        printf("Detected physics reset\n");
        deltat = 0;
        last_received_bitmask = 0;
    } else {
        deltat = state.timestamp_s - last_timestamp_s;
    }
    time_now_us += deltat * 1.0e6;

    if (is_positive(deltat) && deltat < 0.1) {
        // time in us to hz
        if (use_time_sync) {
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
    adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));

#if 0
    // report frame rate
    if (frame_counter % 1000 == 0) {
        printf("FPS %.2f\n", rate_hz); // this is instantaneous rather than any clever average
    }
#endif
}

#endif  // HAL_SIM_JSON_ENABLED
