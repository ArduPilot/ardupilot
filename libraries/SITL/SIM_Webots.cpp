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

#include "SIM_Webots.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
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
    use_time_sync = true;
    rate_hz = 4000;
    
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

  This parser does not do any syntax checking, and is not at all
  general purpose

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

bool Webots::parse_sensors(const char *json)
{
    //printf("%s\n", json);
    ///*
     for (uint16_t i=0; i<ARRAY_SIZE(keytable); i++) {
        struct keytable &key = keytable[i];
        //printf("search   %s/%s\n", key.section, key.key);
        // look for section header 
        const char *p = strstr(json, key.section);
        if (!p) {
            // we don't have this sensor
            continue;
        }
        p += strlen(key.section)+1;

        // find key inside section
        p = strstr(p, key.key);
        if (!p) {
            printf("Failed to find key %s/%s\n", key.section, key.key);
            return false;
        }

        p += strlen(key.key)+3;
        switch (key.type) {
        case DATA_FLOAT:
            *((float *)key.ptr) = atof(p);
            //printf("GOT  %s/%s value: %f\n", key.section, key.key, *((float *)key.ptr));
            break;

        case DATA_DOUBLE:
            *((double *)key.ptr) = atof(p);
            //printf("GOT  %s/%s value: %f\n", key.section, key.key, *((double *)key.ptr));
            break;

        case DATA_VECTOR3F: {
            Vector3f *v = (Vector3f *)key.ptr;
            if (sscanf(p, "[%f, %f, %f]", &v->x, &v->y, &v->z) != 3) {
                printf("Failed to parse Vector3f for %s %s/%s\n",p,  key.section, key.key);
                //printf("Failed to parse Vector3f for  %s/%s\n", key.section, key.key);
                return false;
            }
            else
            {
                //printf("GOT  %s/%s [%f, %f, %f]\n", key.section, key.key, v->x, v->y, v->z);
            }
            
            break;
        }

        case DATA_VECTOR3F_ARRAY: {
            // example: [[0.0, 0.0, 0.0], [-8.97607135772705, -8.976069450378418, -8.642673492431641e-07]]
            if (*p++ != '[') {
                return false;
            }
            uint16_t n = 0;
            struct vector3f_array *v = (struct vector3f_array *)key.ptr;
            while (true) {
                if (n >= v->length) {
                    Vector3f *d = (Vector3f *)realloc(v->data, sizeof(Vector3f)*(n+1));
                    if (d == nullptr) {
                        return false;
                    }
                    v->data = d;
                    v->length = n+1;
                }
                if (sscanf(p, "[%f, %f, %f]", &v->data[n].x, &v->data[n].y, &v->data[n].z) != 3) {
                    //printf("Failed to parse Vector3f for %s %s/%s[%u]\n",p,  key.section, key.key, n);
                    //printf("Failed to parse Vector3f for %s/%s[%u]\n", key.section, key.key, n);
                    return false;
                }
                else
                {
                    //printf("GOT  %s/%s [%f, %f, %f]\n", key.section, key.key, v->data[n].x, v->data[n].y, v->data[n].z);
                }
                n++;
                p = strchr(p,']');
                if (!p) {
                    return false;
                }
                p++;
                if (p[0] != ',') {
                    break;
                }
                if (p[1] != ' ') {
                    return false;
                }
                p += 2;
            }
            if (p[0] != ']') {
                return false;
            }
            v->length = n;
            break;
        }

        case DATA_FLOAT_ARRAY: {
            // example: [18.0, 12.694079399108887]
            if (*p++ != '[') {
                return false;
            }
            uint16_t n = 0;
            struct float_array *v = (struct float_array *)key.ptr;
            while (true) {
                if (n >= v->length) {
                    float *d = (float *)realloc(v->data, sizeof(float)*(n+1));
                    if (d == nullptr) {
                        return false;
                    }
                    v->data = d;
                    v->length = n+1;
                }
                v->data[n] = atof(p);
                n++;
                p = strchr(p,',');
                if (!p) {
                    break;
                }
                p++;
            }
            v->length = n;
            break;
        }
        }
    }
    // */
    socket_frame_counter++;
    return true;
    
}

/*
  connect to the required sockets
 */
bool Webots::connect_sockets(void)
{
    if (!sim_sock) {
        sim_sock = new SocketAPM(false);
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
    ssize_t ret = sim_sock->recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, 0);
    if (ret <= 0) {
        no_data_counter++;
        if (no_data_counter == 1000) {
            no_data_counter = 0;
            delete sim_sock;
            sim_sock = nullptr;
        }
        return false;
    }
    no_data_counter = 0;

    // convert '\n' into nul
    while (uint8_t *p = (uint8_t *)memchr(&sensor_buffer[sensor_buffer_len], '\n', ret)) {
        *p = 0;
    }
    sensor_buffer_len += ret;

    const uint8_t *p2 = (const uint8_t *)memrchr(sensor_buffer, 0, sensor_buffer_len);
    if (p2 == nullptr || p2 == sensor_buffer) {
        return false;
    }
    const uint8_t *p1 = (const uint8_t *)memrchr(sensor_buffer, 0, p2 - sensor_buffer);
    if (p1 == nullptr) {
        return false;
    }

    bool parse_ok = parse_sensors((const char *)(p1+1));

    memmove(sensor_buffer, p2, sensor_buffer_len - (p2 - sensor_buffer));
    sensor_buffer_len = sensor_buffer_len - (p2 - sensor_buffer);

    return parse_ok;
}

/*
  output control command assuming skid-steering rover
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

    sim_sock->send(buf, len);
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

    sim_sock->send(buf, len);
}

/*
  output control command assuming a 4 channel quad
*/
void Webots::output_quad(const struct sitl_input &input)
{
    const float max_thrust = 1.0;
    float motors[4];
    for (uint8_t i=0; i<4; i++) {
        //return a filtered servo input as a value from 0 to 1
        //servo is assumed to be 1000 to 2000
        motors[i] = constrain_float(((input.servos[i]-1000)/1000.0f) * max_thrust, 0, max_thrust); 
    }
    const float &m_right = motors[0]; 
    const float &m_left  = motors[1]; 
    const float &m_front = motors[2]; 
    const float &m_back  = motors[3]; 

    // quad format in Webots is:
    // m1: front
    // m2: right
    // m3: back
    // m4: left

    // construct a JSON packet for motors
    char buf[200];
    const int len = snprintf(buf, sizeof(buf)-1, "{\"eng\": [%.3f, %.3f, %.3f, %.3f], \"wnd\": [%f, %3.1f, %1.1f, %2.1f]}\n",
             m_front, m_right, m_back, m_left,
             input.wind.speed, wind_ef.x, wind_ef.y, wind_ef.z);
    buf[len] = 0;
    sim_sock->send(buf, len);
}

/*
  output all 16 channels as PWM values. This allows for general
  control of a robot
*/
void Webots::output_pwm(const struct sitl_input &input)
{
    char buf[200];
    const int len = snprintf(buf, sizeof(buf)-1, "{\"pwm\": [%u, %uf, %u, %u, %u, %uf, %u, %u, %u, %uf, %u, %u, %u, %uf, %u, %u], \"wnd\": [%f, %f, %f, %f]}\n",
             input.servos[0], input.servos[1], input.servos[2], input.servos[3],
             input.servos[4], input.servos[5], input.servos[6], input.servos[7],
             input.servos[8], input.servos[9], input.servos[10], input.servos[11],
             input.servos[12], input.servos[13], input.servos[14], input.servos[15],
             input.wind.speed, wind_ef.x, wind_ef.y, wind_ef.z);
    buf[len ] = 0;
    sim_sock->send(buf, len);
}


void Webots::output (const struct sitl_input &input)
{
    
    switch (output_type) {
        case OUTPUT_ROVER:
            output_rover(input);
            break;
        case OUTPUT_QUAD:
            output_quad(input);
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
    
    if (!connect_sockets()) {
        return;
    }

    last_state = state;
    const bool valid = sensors_receive();
    if (valid) {
        // update average frame time used for extrapolation
        double dt = constrain_float(state.timestamp - last_state.timestamp, 0.001, 1.0/50);
        if (average_frame_time_s < 1.0e-6) {
            //if average is too small take the current dt as a good start.
            average_frame_time_s = dt;
        }
        
        // this is complementry filter for updating average.
        average_frame_time_s = average_frame_time_s * 0.98 + dt * 0.02;
        
    }
    
    // again measure dt as dt_s but with no constraints....
    double dt_s = state.timestamp - last_state.timestamp;
    
    if (dt_s < 0 || dt_s > 1) {
        // cope with restarting while connected
        initial_time_s = time_now_us * 1.0e-6f;
        return;
    }

    if (dt_s < 0.00001f) {
        float delta_time = 0.001;
        // don't go past the next expected frame
        if (delta_time + extrapolated_s > average_frame_time_s) {
            delta_time = average_frame_time_s - extrapolated_s;
        }

        // check if extrapolation_s duration is longer than average_frame_time_s
        if (delta_time <= 0) {
            // dont extrapolate anymore untill there is valid data with long-enough dt_s
            usleep(1000);
            return;
        }

        // extrapolated_s duration is safe. keep on extrapolation.
        time_now_us += delta_time * 1.0e6;
        extrapolate_sensors(delta_time);
        update_position();

        //update body magnetic field from position and rotation
        update_mag_field_bf();
        usleep(delta_time * 1.0e6);
        extrapolated_s += delta_time;

        //output(input);
        report_FPS();
        return;
    }

    if (valid)
    {
        // reset extrapolation duration.
        extrapolated_s = 0;
        
        if (initial_time_s <= 0) {
            dt_s = 0.001f;
            initial_time_s = state.timestamp - dt_s;
        }

        // convert from state variables to ardupilot conventions
        dcm.from_euler(state.pose.roll, state.pose.pitch, -state.pose.yaw);

        gyro = Vector3f(state.imu.angular_velocity[0] ,
                        state.imu.angular_velocity[1] ,
                        -state.imu.angular_velocity[2] ); 
        
        accel_body = Vector3f(+state.imu.linear_acceleration[0],
                            +state.imu.linear_acceleration[1],
                            -state.imu.linear_acceleration[2]);

        velocity_ef = Vector3f(+state.velocity.world_linear_velocity[0],
                            +state.velocity.world_linear_velocity[1],
                            -state.velocity.world_linear_velocity[2]);
        
        position = Vector3f(state.gps.x, state.gps.y, -state.gps.z);
        

        // limit to 16G to match pixhawk1
        float a_limit = GRAVITY_MSS*16;
        accel_body.x = constrain_float(accel_body.x, -a_limit, a_limit);
        accel_body.y = constrain_float(accel_body.y, -a_limit, a_limit);
        accel_body.z = constrain_float(accel_body.z, -a_limit, a_limit);

        // fill in laser scanner results, if available
        scanner.points = state.scanner.points;
        scanner.ranges = state.scanner.ranges;

        update_position();
        uint64_t new_time_us = (state.timestamp - initial_time_s)*1.0e6;
        if (new_time_us < time_now_us) {
            uint64_t dt_us = time_now_us - new_time_us;
            if (dt_us > 500000) {
                // time going backwards
                time_now_us = new_time_us;
            }
        } else {
            // update SITL time with webots time.
            time_now_us = new_time_us;
        }

        time_advance();
        
        // update magnetic field
        update_mag_field_bf();

        update_wind (input);
        output(input);

        report_FPS();
    }
}


/*
  report frame rates
 */
void Webots::report_FPS(void)
{
    if (frame_counter++ % 1000 == 0) {
        if (!is_zero(last_frame_count_s)) {
            uint64_t frames = socket_frame_counter - last_socket_frame_counter;
            last_socket_frame_counter = socket_frame_counter;
            double dt = state.timestamp - last_frame_count_s;
            printf("%.2f/%.2f FPS avg=%.2f\n",
                    frames / dt, 1000 / dt, 1.0/average_frame_time_s);
        } else {
            printf("Initial position %f %f %f\n", position.x, position.y, position.z);
        }
        last_frame_count_s = state.timestamp;
    }
}
