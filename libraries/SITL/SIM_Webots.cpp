/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have Weboreceived a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for webots simulator
*/

#include "SIM_config.h"

#if AP_SIM_WEBOTS_ENABLED

#include "SIM_Webots.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include <AP_HAL/AP_HAL.h>
#include "pthread.h"
#include <AP_HAL/utility/replace.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

static const struct {
    const char *name;
    float value;
    bool save;
} sim_defaults[] = {
    { "AHRS_EKF_TYPE", 10 },
    { "INS_GYR_CAL", 0 },
    { "RC1_MIN", 1000, true },
    { "RC1_MAX", 2000, true },
    { "RC2_MIN", 1000, true },
    { "RC2_MAX", 2000, true },
    { "RC3_MIN", 1000, true },
    { "RC3_MAX", 2000, true },
    { "RC4_MIN", 1000, true },
    { "RC4_MAX", 2000, true },
    { "RC2_REVERSED", 1 }, // interlink has reversed rc2
    { "SERVO1_MIN", 1000 },
    { "SERVO1_MAX", 2000 },
    { "SERVO2_MIN", 1000 },
    { "SERVO2_MAX", 2000 },
    { "SERVO3_MIN", 1000 },
    { "SERVO3_MAX", 2000 },
    { "SERVO4_MIN", 1000 },
    { "SERVO4_MAX", 2000 },
    { "SERVO5_MIN", 1000 },
    { "SERVO5_MAX", 2000 },
    { "SERVO6_MIN", 1000 },
    { "SERVO6_MAX", 2000 },
    { "SERVO6_MIN", 1000 },
    { "SERVO6_MAX", 2000 },
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


Webots::Webots(const char *frame_str) :
    Aircraft(frame_str)
{
    use_time_sync = false;
    use_smoothing = false;

    char *saveptr = nullptr;
    char *s = strdup(frame_str);
    char *frame_option = strtok_r(s, ":", &saveptr);
    char *args1 = strtok_r(nullptr, ":", &saveptr);
    char *args2 = strtok_r(nullptr, ":", &saveptr);
    /*
      allow setting of IP, sensors port and control port
      format morse:IPADDRESS:SENSORS_PORT:CONTROL_PORT
     */
    if (args1) {
        webots_ip = args1;
        printf("Simulation Port %s\n",args1);
    }
    if (args2) {
        webots_sensors_port = atoi(args2);
    }
    

    if (strstr(frame_option, "-rover")) {
        output_type = OUTPUT_ROVER;
    } else if (strstr(frame_option, "-quad")) {
        output_type = OUTPUT_QUAD;
    } else if (strstr(frame_option, "-tri")) {
        output_type = OUTPUT_TRICOPTER;
    } else if (strstr(frame_option, "-pwm")) {
        output_type = OUTPUT_PWM;
    } else {
        // default to rover
        output_type = OUTPUT_ROVER;
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
    printf("Started Webots with %s:%u type %u\n",
           webots_ip, webots_sensors_port,
           (unsigned)output_type);
}

/*
  very simple JSON parser for sensor data
  called with pointer to one row of sensor data, nul terminated

  This parser only checks what it reads: each key must be a whole quoted
  word followed by ':', and each value a finite number, "[x, y, z]" vector
  or bracketed list of those.  It is not a general purpose JSON parser.

{"timestamp": 1563474924.817575, 
    "vehicle.imu": {"timestamp": 1563474924.8009083, 
        "angular_velocity": [2.319516170246061e-06, -3.5830129263558774e-07, 7.009341995711793e-09], 
        "linear_acceleration": [0.005075275432318449, 0.22471635043621063, 9.80748176574707], 
        "magnetic_field": [23088.65625, 3875.89453125, -53204.51171875]}, 
        "vehicle.gps": {"timestamp": 1563474924.8009083, "x": 5.386466364143416e-05, "y": -0.0010969983413815498, "z": 0.03717954829335213}, 
    "vehicle.velocity": {"timestamp": 1563474924.8009083, 
        "linear_velocity": [4.818238585890811e-10, 2.1333558919423012e-08, 9.310780910709582e-07], 
        "angular_velocity": [2.319516170246061e-06, -3.5830129263558774e-07, 7.009341995711793e-09], 
        "world_linear_velocity": [5.551115123125783e-17, 0.0, 9.313225746154785e-07]}, 
        "vehicle.pose": {"timestamp": 1563474924.8009083, 
            "x": 5.386466364143416e-05, "y": -0.0010969983413815498, "z": 0.03717954829335213, 
            "yaw": 7.137723878258839e-05, "pitch": -0.0005173543468117714, "roll": 0.022908739745616913}}

*/

/*
  find a JSON key as a whole quoted word, so that e.g. "rpm" does not match
  inside some other key or value that merely contains those letters
 */
const char *Webots::find_key(const char *p, const char *key)
{
    const size_t len = strlen(key);
    const char *start = p;
    while ((p = strstr(p, key)) != nullptr) {
        if (p > start && p[-1] == '"' && p[len] == '"') {
            return p;
        }
        p++;
    }
    return nullptr;
}

/*
  print a sensor-frame parse error, at most once a second of wall-clock time:
  a rejected frame is re-sent immediately, and SITL's own clock does not move
  while frames are being rejected
 */
void Webots::parse_error(const char *fmt, ...)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const double now_s = ts.tv_sec + ts.tv_nsec * 1.0e-9;
    if (now_s - last_parse_error_s < 1.0) {
        parse_errors_suppressed++;
        return;
    }
    last_parse_error_s = now_s;

    va_list ap;
    va_start(ap, fmt);
    printf("Webots: ");
    vprintf(fmt, ap);
    va_end(ap);
    if (parse_errors_suppressed > 0) {
        printf(" (%u similar suppressed)", unsigned(parse_errors_suppressed));
        parse_errors_suppressed = 0;
    }
    printf("\n");
}

static const char *skip_spaces(const char *p)
{
    while (*p == ' ') {
        p++;
    }
    return p;
}

// parse one finite number, advancing p past it
static bool parse_float(const char *&p, float &value)
{
    char *endp;
    const float v = strtof(p, &endp);
    if (endp == p || !isfinite(v)) {
        return false;
    }
    value = v;
    p = endp;
    return true;
}

static bool parse_double(const char *&p, double &value)
{
    char *endp;
    const double v = strtod(p, &endp);
    if (endp == p || !isfinite(v)) {
        return false;
    }
    value = v;
    p = endp;
    return true;
}

// parse "[x, y, z]", advancing p past the closing bracket
static bool parse_vector3f(const char *&p, Vector3f &v)
{
    p = skip_spaces(p);
    if (*p++ != '[') {
        return false;
    }
    for (uint8_t i = 0; i < 3; i++) {
        if (!parse_float(p, v[i])) {
            return false;
        }
        p = skip_spaces(p);
        if (*p++ != (i < 2 ? ',' : ']')) {
            return false;
        }
    }
    return true;
}

/*
  after one array element: step over a ',' and return true, or stop at the
  closing ']' and return false.  Anything else, including the end of the
  record, is a parse error.
 */
static bool next_element(const char *&p, bool &ok)
{
    p = skip_spaces(p);
    if (*p == ',') {
        p++;
        return true;
    }
    ok = (*p == ']');
    return false;
}

bool Webots::parse_sensors(const char *json)
{
    //printf("%s\n", json);
    // rotor speeds are optional, so a frame without them must not keep the
    // previous frame's count
    state.rpm_count = 0;

    for (uint16_t i=0; i<ARRAY_SIZE(keytable); i++) {
        struct keytable &key = keytable[i];
        // look for section header
        const char *p = strstr(json, key.section);
        if (!p) {
            if (!key.required) {
                // we don't have this sensor
                continue;
            }
            parse_error("no section %s for key %s", key.section, key.key);
            return false;
        }
        p += strlen(key.section)+1;

        // find key inside section
        p = find_key(p, key.key);
        if (!p) {
            if (!key.required) {
                // optional data this controller does not send
                continue;
            }
            parse_error("no key %s/%s", key.section, key.key);
            return false;
        }

        // step over the key's closing quote and the ':', with any spaces
        p = skip_spaces(p + strlen(key.key) + 1);
        if (*p++ != ':') {
            parse_error("no ':' after %s/%s", key.section, key.key);
            return false;
        }
        p = skip_spaces(p);

        bool ok = true;
        switch (key.type) {
        case DATA_FLOAT:
            ok = parse_float(p, *(float *)key.ptr);
            break;

        case DATA_DOUBLE:
            ok = parse_double(p, *(double *)key.ptr);
            break;

        case DATA_VECTOR3F:
            ok = parse_vector3f(p, *(Vector3f *)key.ptr);
            break;

        case DATA_VECTOR3F_ARRAY: {
            // example: [[0.0, 0.0, 0.0], [-8.97607135772705, -8.976069450378418, -8.642673492431641e-07]]
            struct vector3f_array *v = (struct vector3f_array *)key.ptr;
            uint16_t n = 0;
            ok = (*p++ == '[');
            // an empty list is fine; otherwise read elements up to the ']'
            if (!ok || *skip_spaces(p) != ']') {
                while (ok) {
                    if (n >= v->length) {
                        Vector3f *d = (Vector3f *)realloc(v->data, sizeof(Vector3f)*(n+1));
                        if (d == nullptr) {
                            return false;
                        }
                        v->data = d;
                        v->length = n+1;
                    }
                    ok = parse_vector3f(p, v->data[n]);
                    if (!ok) {
                        break;
                    }
                    n++;
                    if (!next_element(p, ok)) {
                        break;
                    }
                }
            }
            if (ok) {
                v->length = n;
            }
            break;
        }

        case DATA_FLOAT_ARRAY: {
            // example: [18.0, 12.694079399108887]
            struct float_array *v = (struct float_array *)key.ptr;
            uint16_t n = 0;
            ok = (*p++ == '[');
            // an empty list is fine; otherwise read elements up to the ']'
            if (!ok || *skip_spaces(p) != ']') {
                while (ok) {
                    if (n >= v->length) {
                        float *d = (float *)realloc(v->data, sizeof(float)*(n+1));
                        if (d == nullptr) {
                            return false;
                        }
                        v->data = d;
                        v->length = n+1;
                    }
                    ok = parse_float(p, v->data[n]);
                    if (!ok) {
                        break;
                    }
                    n++;
                    if (!next_element(p, ok)) {
                        break;
                    }
                }
            }
            if (ok) {
                v->length = n;
            }
            break;
        }

        case DATA_RPM_ARRAY: {
            // example: [5795.2, 5794.8, 5795.1, 5794.9]  (rev/min), indexed by
            // SITL servo channel; entries beyond MAX_WEBOTS_RPM are ignored
            float *v = (float *)key.ptr;
            uint8_t n = 0;
            ok = (*p++ == '[');
            // an empty list is fine; otherwise read elements up to the ']'
            if (!ok || *skip_spaces(p) != ']') {
                while (ok) {
                    float value;
                    ok = parse_float(p, value);
                    if (!ok) {
                        break;
                    }
                    if (n < MAX_WEBOTS_RPM) {
                        v[n++] = value;
                    }
                    if (!next_element(p, ok)) {
                        break;
                    }
                }
            }
            if (ok) {
                state.rpm_count = n;
            }
            break;
        }
        }

        if (!ok) {
            parse_error("bad value for %s/%s", key.section, key.key);
            return false;
        }
    }

    socket_frame_counter++;
    return true;
}

/*
  connect to the required sockets
 */
bool Webots::connect_sockets(void)
{
    if (!sim_sock) {
        sim_sock = NEW_NOTHROW SocketAPM_native(false);
        if (!sim_sock) {
            AP_HAL::panic("Out of memory for sensors socket");
        }
        if (!sim_sock->connect(webots_ip, webots_sensors_port)) {
            usleep(100000);
            if (connect_counter++ == 20) {
                printf("Waiting to connect to sensors control on %s:%u\n",
                       webots_ip, webots_sensors_port);
                connect_counter = 0;
            }
            delete sim_sock;
            sim_sock = nullptr;
            return false;
        }
        // wait for Webots packets
        sim_sock->set_blocking(true);
        sim_sock->reuseaddress();
        printf("Sensors connected\n");
    }
    return true; 
}

/*
  get any new data from the sensors socket
*/
bool Webots::sensors_receive(void)
{
    if (sensor_buffer_len >= sizeof(sensor_buffer)) {
        // a full buffer with no line terminator in it means the peer is not
        // speaking our protocol; drop it rather than wedging on a zero-length
        // recv() forever
        printf("Webots: sensor buffer overflow, discarding %u bytes\n",
               (unsigned)sensor_buffer_len);
        sensor_buffer_len = 0;
    }

    const ssize_t ret = sim_sock->recv(&sensor_buffer[sensor_buffer_len],
                                       sizeof(sensor_buffer)-sensor_buffer_len, 0);
    if (ret == 0) {
        /*
          the controller closed the connection (Webots quit, or the world was
          reset and restarted it).  Drop the socket so connect_sockets() dials
          again, and let the first frame of the new session re-base our clock
         */
        printf("Webots: sensors connection closed, reconnecting\n");
        delete sim_sock;
        sim_sock = nullptr;
        sensor_buffer_len = 0;
        last_state.timestamp = 0;
        return false;
    }
    if (ret < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN && errno != EINTR) {
            // a real socket error, not just no data yet: dial again
            printf("Webots: sensors connection error %d, reconnecting\n", errno);
            delete sim_sock;
            sim_sock = nullptr;
            sensor_buffer_len = 0;
            last_state.timestamp = 0;
        }
        return false;
    }

    // convert '\n' into nul
    while (uint8_t *p = (uint8_t *)memchr(&sensor_buffer[sensor_buffer_len], '\n', ret)) {
        *p = 0;
    }
    sensor_buffer_len += ret;

    // the newest complete record ends at the last terminator in the buffer
    const uint8_t *end = (const uint8_t *)memrchr(sensor_buffer, 0, sensor_buffer_len);
    if (end == nullptr) {
        // no complete record yet
        return false;
    }

    /*
      The record starts just after the previous terminator, or at the start of
      the buffer if this is the first one.  The original code required a second
      terminator to exist and gave up otherwise, so the very first record a
      controller sent was never parsed: SITL stayed in recv() and never replied.
      The bundled controllers only survived that because their select() timeout
      truncated to zero and they re-sent the frame in a busy loop.
     */
    const uint8_t *prev = (const uint8_t *)memrchr(sensor_buffer, 0, end - sensor_buffer);
    const uint8_t *start = (prev == nullptr) ? sensor_buffer : prev + 1;

    const bool parse_ok = parse_sensors((const char *)start);

    // discard everything up to and including the record we just consumed
    const uint32_t consumed = (uint32_t)(end + 1 - sensor_buffer);
    memmove(sensor_buffer, end + 1, sensor_buffer_len - consumed);
    sensor_buffer_len -= consumed;

    return parse_ok;
}

/*
  send a whole control frame: a stream socket's send() may write only part of
  it, and a frame cut short would run into the next one
*/
void Webots::send_frame(const char *buf, size_t len)
{
    while (len > 0) {
        const ssize_t ret = sim_sock->send(buf, len);
        if (ret <= 0) {
            if (ret < 0 && errno == EINTR) {
                continue;
            }
            // the connection is gone; the next sensors_receive() sees the
            // close or error and reconnects
            return;
        }
        buf += ret;
        len -= ret;
    }
}

/*
  output control command for a car-like rover: [steering, throttle], both
  -1..1, from SERVO1 and SERVO3
*/
void Webots::output_rover(const struct sitl_input &input)
{

    const float motor1 = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
    const float motor2 = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
    
    // construct a JSON packet for v and w
    char buf[200];
    
    const int len = snprintf(buf, sizeof(buf)-1, "{\"rover\": [%f, %f], \"wnd\": [%f, %f, %f, %f]}\n",
             motor1, motor2,
             input.wind.speed, wind_ef.x, wind_ef.y, wind_ef.z);
    
    buf[len] = 0;

    send_frame(buf, len);
}

/*
  output control command assuming a 3 channels motors and 1 channel servo
*/
void Webots::output_tricopter(const struct sitl_input &input)
{
    const float max_thrust = 1.0;
    float motors[3];
    const float servo = ((input.servos[6]-1000)/1000.0f - 0.5f);
    motors[0] = constrain_float(((input.servos[0]-1000)/1000.0f) * max_thrust, 0, max_thrust); 
    motors[1] = constrain_float(((input.servos[1]-1000)/1000.0f) * max_thrust, 0, max_thrust); 
    motors[2] = constrain_float(((input.servos[3]-1000)/1000.0f) * max_thrust, 0, max_thrust); 

    const float &m_right = motors[0]; 
    const float &m_left  = motors[1]; 
    const float &m_servo = servo ; 
    const float &m_back  = motors[2]; 

    // construct a JSON packet for motors
    char buf[200];
    const int len = snprintf(buf, sizeof(buf)-1, "{\"eng\": [%.3f, %.3f, %.3f, %.3f], \"wnd\": [%f, %3.1f, %1.1f, %2.1f]}\n",
             m_right, m_left, m_servo, m_back,
             input.wind.speed, wind_ef.x, wind_ef.y, wind_ef.z);
    //printf("\"eng\": [%.3f, %.3f, %.3f, %.3f]\n",m_right, m_left, m_servo, m_back);
    buf[len] = 0;

    send_frame(buf, len);
}



/*
  output all 16 channels as PWM values. This allows for general
  control of a robot
*/
void Webots::output_pwm(const struct sitl_input &input)
{
    char buf[2000];
    const int len = snprintf(buf, sizeof(buf)-1, "{\"pwm\": [%d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0, %d.0], \"wnd\": [%3.3f, %f, %3.3f, %3.3f]}\n",
             input.servos[0], input.servos[1], input.servos[2], input.servos[3],
             input.servos[4], input.servos[5], input.servos[6], input.servos[7],
             input.servos[8], input.servos[9], input.servos[10], input.servos[11],
             input.servos[12], input.servos[13], input.servos[14], input.servos[15],
             input.wind.speed, wind_ef.x, wind_ef.y, wind_ef.z);
    buf[len] = 0;
    send_frame(buf, len);
}


void Webots::output (const struct sitl_input &input)
{

    switch (output_type) {
        case OUTPUT_ROVER:
            output_rover(input);
            break;
        case OUTPUT_QUAD:
            output_pwm(input);
            break;
        case OUTPUT_TRICOPTER:
            output_tricopter(input);
            break;
        case OUTPUT_PWM:
            output_pwm(input);
            break;
        }
}

/*
  update the Webots simulation by one time step
 */
void Webots::update(const struct sitl_input &input)
{
    update_battery();

    if (!connect_sockets()) {
        return;
    }

    if (!sensors_receive()) {
        return;
    }

    /*
      Advance our clock by however much simulation time Webots advanced by.

      A repeated or out-of-order timestamp gives a non-positive delta.  Feeding
      that to time_advance() would stall or rewind the scheduler, so re-send the
      servo frame and wait for a fresh one instead.  The guard this replaces
      read `if ((!first) && (frame_time_us == 0))` where `first` was only ever
      cleared inside that same branch, so it could never run.
     */
    /*
      The first frame only sets the time base.  Webots' clock does not restart
      with SITL: the controllers now accept a new SITL session without the
      world being reset, so the first timestamp can be however long the world
      has already been running, and must not be taken as one giant step.
     */
    if (is_zero(last_state.timestamp)) {
        last_state = state;
        output(input);
        return;
    }

    const double frame_time_s = state.timestamp - last_state.timestamp;
    if (frame_time_s < 0) {
        // the simulator's clock went backwards (e.g. the world was reset
        // under a live connection): take this frame as the new time base
        // rather than waiting for the clock to catch up with the old one
        last_state = state;
        output(input);
        return;
    }
    if (is_zero(frame_time_s)) {
        output(input);
        return;
    }

    frame_time_us = frame_time_s * 1.0e6;
    time_now_us += frame_time_us;

    // convert from state variables to ardupilot conventions
    dcm.from_euler(state.pose.roll, state.pose.pitch, -state.pose.yaw);

    gyro = Vector3f(state.imu.angular_velocity[0],
                    state.imu.angular_velocity[1],
                    -state.imu.angular_velocity[2]);

    accel_body = Vector3f(+state.imu.linear_acceleration[0],
                        +state.imu.linear_acceleration[1],
                        -state.imu.linear_acceleration[2]);

    velocity_ef = Vector3f(+state.velocity.world_linear_velocity[0],
                        +state.velocity.world_linear_velocity[1],
                        -state.velocity.world_linear_velocity[2]);

    position = Vector3d(state.gps.x, state.gps.y, -state.gps.z);
    position.xy() += origin.get_distance_NE_double(home);

    // limit to 16G to match pixhawk1
    float a_limit = GRAVITY_MSS*16;
    accel_body.x = constrain_float(accel_body.x, -a_limit, a_limit);
    accel_body.y = constrain_float(accel_body.y, -a_limit, a_limit);
    accel_body.z = constrain_float(accel_body.z, -a_limit, a_limit);

    // fill in laser scanner results, if available
    scanner.points = state.scanner.points;
    scanner.ranges = state.scanner.ranges;

    /*
      Rotor speeds, if the controller reports them, indexed by SITL servo
      channel like rpm[].  Webots cannot read a Propeller's shaft speed, so the
      bundled controllers send an estimate that follows the rotor's
      torque-limited spin-up.  They reach AP_RPM's SITL backend (RPM1_TYPE 10),
      RPM logging and the RPM-driven harmonic notch.
     */
    for (uint8_t i = 0; i < state.rpm_count && i < ARRAY_SIZE(rpm); i++) {
        rpm[i] = state.rpm[i];
    }
    // AP_RPM_SITL walks the set bits of motor_mask to find the motor for each
    // RPM instance, so it has to know how many rotors we are reporting.
    motor_mask = (state.rpm_count >= 32) ? 0xFFFFFFFFU : ((1U << state.rpm_count) - 1U);

    update_position();

    // update magnetic field
    update_mag_field_bf();

    time_advance();

    update_wind(input);

    //report_FPS();

    output(input);

    last_state = state;
}

/*
  report frame rates
 */
void Webots::report_FPS(void)
{
    // if (frame_counter++ % 1000 == 0) {
    //     if (!is_zero(last_frame_count_s)) {
    //         uint64_t frames = socket_frame_counter - last_socket_frame_counter;
    //         last_socket_frame_counter = socket_frame_counter;
    //         double dt = state.timestamp - last_frame_count_s;
    //         printf("%.2f/%.2f FPS avg=%.2f\n",
    //                 frames / dt, 1000 / dt, 1.0/average_frame_time_s);
    //     } else {
    //         printf("Initial position %f %f %f\n", position.x, position.y, position.z);
    //     }
    //     last_frame_count_s = state.timestamp;
    // }
}

#endif  // AP_SIM_WEBOTS_ENABLED
