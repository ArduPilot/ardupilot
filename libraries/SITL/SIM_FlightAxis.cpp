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
  simulator connector for FlightAxis
*/

#include "SIM_FlightAxis.h"

#if HAL_SIM_FLIGHTAXIS_ENABLED

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

/*
  we use a thread for socket creation to reduce the impact of socket
  creation latency. These condition variables are used to synchronise
  the thread
 */
static pthread_cond_t sockcond1 = PTHREAD_COND_INITIALIZER;
static pthread_cond_t sockcond2 = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t sockmtx = PTHREAD_MUTEX_INITIALIZER;

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

static const struct {
    const char *name;
    float value;
    bool save;
} sim_defaults[] = {
    { "BRD_OPTIONS", 0},
    { "AHRS_EKF_TYPE", 10 },
    { "INS_GYR_CAL", 0 },
    { "BATT_MONITOR", 4 },
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
    { "RPM1_TYPE", 10 },
};

/*
  get system timestamp in seconds
 */
static double timestamp_sec()
{
    struct timeval tval;
    gettimeofday(&tval,NULL);
    return tval.tv_sec + (tval.tv_usec*1.0e-6);
}

FlightAxis::FlightAxis(const char *frame_str) :
    Aircraft(frame_str)
{
    use_time_sync = false;
    num_motors = 2;
    rate_hz = 250 / target_speedup;
    heli_demix = strstr(frame_str, "helidemix") != nullptr;
    rev4_servos = strstr(frame_str, "rev4") != nullptr;
    const char *colon = strchr(frame_str, ':');
    if (colon) {
        controller_ip = colon+1;
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

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&FlightAxis::socket_creator, void), "SocketCreator", 8192,
                                      AP_HAL::Scheduler::PRIORITY_BOOST, 0)) {
        printf("Failed to create socket_creator thread\n");
    }
}
    
/*
  extremely primitive SOAP parser that assumes the format used by FlightAxis
*/
void FlightAxis::parse_reply(const char *reply)
{
    const char *reply0 = reply;
    for (uint16_t i=0; i<num_keys; i++) {
        const char *p = strstr(reply, keytable[i].key);
        if (p == nullptr) {
            p = strstr(reply0, keytable[i].key);
        }
        if (p == nullptr) {
            printf("Failed to find key %s\n", keytable[i].key);
            controller_started = false;
            break;
        }
        p += strlen(keytable[i].key) + 1;
        double v;
        if (strncmp(p, "true", 4) == 0) {
            v = 1;
        } else if (strncmp(p, "false", 5) == 0) {
            v = 0;
        } else {
            v = atof(p);
        }
        keytable[i].ref = v;
        // this assumes key order and allows us to decode arrays
        p = strchr(p, '>');
        if (p != nullptr) {
            reply = p;
        }
    }
}


/*
  make a SOAP request, returning body of reply
 */
bool FlightAxis::soap_request_start(const char *action, const char *fmt, ...)
{
    va_list ap;
    char *req1;

    if (sock) {
        delete sock;
        sock = nullptr;
    }

    va_start(ap, fmt);
    vasprintf(&req1, fmt, ap);
    va_end(ap);

    pthread_mutex_lock(&sockmtx);
    while (socknext == nullptr) {
        pthread_cond_wait(&sockcond1, &sockmtx);
    }
    sock = socknext;
    socknext = nullptr;
    pthread_cond_broadcast(&sockcond2);
    pthread_mutex_unlock(&sockmtx);

    char *req;
    asprintf(&req, R"(POST / HTTP/1.1
soapaction: '%s'
content-length: %u
content-type: text/xml;charset='UTF-8'
Connection: Keep-Alive

%s)",
             action,
             (unsigned)strlen(req1), req1);
    sock->send(req, strlen(req));
    free(req1);
    free(req);
    return true;
}

char *FlightAxis::soap_request_end(uint32_t timeout_ms)
{
    if (!sock) {
        return nullptr;
    }
    if (!sock->pollin(timeout_ms)) {
        return nullptr;
    }
    sock->set_blocking(true);
    ssize_t ret = sock->recv(replybuf, sizeof(replybuf)-1, 1000);
    if (ret <= 0) {
        return nullptr;
    }
    replybuf[ret] = 0;

    char *p = strstr(replybuf, "Content-Length: ");
    if (p == nullptr) {
        delete sock;
        sock = nullptr;
        printf("No Content-Length\n");
        return nullptr;
    }

    // get the content length
    uint32_t content_length = strtoul(p+16, nullptr, 10);
    char *body = strstr(p, "\r\n\r\n");
    if (body == nullptr) {
        printf("No body\n");
        delete sock;
        sock = nullptr;
        return nullptr;
    }
    body += 4;

    // get the rest of the body
    int32_t expected_length = content_length + (body - replybuf);
    if (expected_length >= (int32_t)sizeof(replybuf)) {
        printf("Reply too large %i\n", expected_length);
        delete sock;
        sock = nullptr;
        return nullptr;
    }
    while (ret < expected_length) {
        ssize_t ret2 = sock->recv(&replybuf[ret], sizeof(replybuf)-(1+ret), 1000);
        if (ret2 <= 0) {
            delete sock;
            sock = nullptr;
            return nullptr;
        }
        // nul terminate
        replybuf[ret+ret2] = 0;
        ret += ret2;
    }
    delete sock;
    sock = nullptr;

    return strdup(replybuf);
}

void FlightAxis::exchange_data(const struct sitl_input &input)
{
    if (!sock &&
        (!controller_started ||
         is_zero(state.m_flightAxisControllerIsActive) ||
         !is_zero(state.m_resetButtonHasBeenPressed))) {
        printf("Starting controller at %s\n", controller_ip);
        // call a restore first. This allows us to connect after the aircraft is changed in RealFlight
        soap_request_start("RestoreOriginalControllerDevice", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>
</soap:Body>
</soap:Envelope>)");
        soap_request_end(1000);
        soap_request_start("InjectUAVControllerInterface", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>
</soap:Body>
</soap:Envelope>)");
        soap_request_end(1000);
        activation_frame_counter = frame_counter;
        controller_started = true;
    }

    // maximum number of servos to send is 12 with new FlightAxis
    float scaled_servos[12];
    for (uint8_t i=0; i<ARRAY_SIZE(scaled_servos); i++) {
        scaled_servos[i] = (input.servos[i] - 1000) / 1000.0f;
    }

    if (rev4_servos) {
        // swap first 4 and last 4 servos, for quadplane testing
        float saved[4];
        memcpy(saved, &scaled_servos[0], sizeof(saved));
        memcpy(&scaled_servos[0], &scaled_servos[4], sizeof(saved));
        memcpy(&scaled_servos[4], saved, sizeof(saved));
    }

    if (heli_demix) {
        // FlightAxis expects "roll/pitch/collective/yaw" input
        float swash1 = scaled_servos[0];
        float swash2 = scaled_servos[1];
        float swash3 = scaled_servos[2];

        float roll_rate = swash1 - swash2;
        float pitch_rate = -((swash1+swash2) / 2.0f - swash3);

        scaled_servos[0] = constrain_float(roll_rate + 0.5, 0, 1);
        scaled_servos[1] = constrain_float(pitch_rate + 0.5, 0, 1);
    }

    const uint16_t channels = hal.scheduler->is_system_initialized()?4095:0;
    if (!sock) {
        soap_request_start("ExchangeData", R"(<?xml version='1.0' encoding='UTF-8'?><soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<ExchangeData>
<pControlInputs>
<m-selectedChannels>%u</m-selectedChannels>
<m-channelValues-0to1>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
</m-channelValues-0to1>
</pControlInputs>
</ExchangeData>
</soap:Body>
</soap:Envelope>)",
                           channels,
                           scaled_servos[0],
                           scaled_servos[1],
                           scaled_servos[2],
                           scaled_servos[3],
                           scaled_servos[4],
                           scaled_servos[5],
                           scaled_servos[6],
                           scaled_servos[7],
                           scaled_servos[8],
                           scaled_servos[9],
                           scaled_servos[10],
                           scaled_servos[11]);
    }

    char *reply = nullptr;
    if (sock) {
        reply = soap_request_end(0);
        if (reply == nullptr) {
            sock_error_count++;
            if (sock_error_count >= 10000 && timestamp_sec() - last_recv_sec > 1) {
                printf("socket timeout\n");
                delete sock;
                sock = nullptr;
                sock_error_count = 0;
                last_recv_sec = timestamp_sec();
            }
        }
    }

    if (reply) {
        sock_error_count = 0;
        last_recv_sec = timestamp_sec();
        double lastt_s = state.m_currentPhysicsTime_SEC;
        parse_reply(reply);
        double dt = state.m_currentPhysicsTime_SEC - lastt_s;
        if (dt > 0 && dt < 0.1) {
            if (average_frame_time_s < 1.0e-6) {
                average_frame_time_s = dt;
            }
            average_frame_time_s = average_frame_time_s * 0.98 + dt * 0.02;
        }
        socket_frame_counter++;
        free(reply);
    }
}


/*
  update the FlightAxis simulation by one time step
 */
void FlightAxis::update(const struct sitl_input &input)
{
    last_input = input;
    exchange_data(input);

    double dt_seconds = state.m_currentPhysicsTime_SEC - last_time_s;
    if (dt_seconds < 0) {
        // cope with restarting RealFlight while connected
        initial_time_s = time_now_us * 1.0e-6f;
        last_time_s = state.m_currentPhysicsTime_SEC;
        position_offset.zero();
        return;
    }
    if (dt_seconds < 0.00001f) {
        float delta_time = 0.001;
        // don't go past the next expected frame
        if (delta_time + extrapolated_s > average_frame_time_s) {
            delta_time = average_frame_time_s - extrapolated_s;
        }
        if (delta_time <= 0) {
            return;
        }
        time_now_us += delta_time * 1.0e6;
        extrapolate_sensors(delta_time);
        update_position();
        update_mag_field_bf();
        extrapolated_s += delta_time;
        report_FPS();
        return;
    }

    extrapolated_s = 0;
    
    if (initial_time_s <= 0) {
        dt_seconds = 0.001f;
        initial_time_s = state.m_currentPhysicsTime_SEC - dt_seconds;
    }

    /*
      the quaternion convention in realflight seems to have Z negative
     */
    Quaternion quat(state.m_orientationQuaternion_W,
                    state.m_orientationQuaternion_Y,
                    state.m_orientationQuaternion_X,
                    -state.m_orientationQuaternion_Z);
    quat.rotation_matrix(dcm);

    gyro = Vector3f(radians(constrain_float(state.m_rollRate_DEGpSEC, -2000, 2000)),
                    radians(constrain_float(state.m_pitchRate_DEGpSEC, -2000, 2000)),
                    -radians(constrain_float(state.m_yawRate_DEGpSEC, -2000, 2000))) * target_speedup;

    velocity_ef = Vector3f(state.m_velocityWorldU_MPS,
                             state.m_velocityWorldV_MPS,
                             state.m_velocityWorldW_MPS);
    position = Vector3d(state.m_aircraftPositionY_MTR,
                        state.m_aircraftPositionX_MTR,
                        -state.m_altitudeASL_MTR - home.alt*0.01);
    position.xy() += origin.get_distance_NE_double(home);

    accel_body = {
        float(state.m_accelerationBodyAX_MPS2),
        float(state.m_accelerationBodyAY_MPS2),
        float(state.m_accelerationBodyAZ_MPS2)
    };

    // accel on the ground is nasty in realflight, and prevents helicopter disarm
    if (!is_zero(state.m_isTouchingGround)) {
        Vector3f accel_ef = (velocity_ef - last_velocity_ef) / dt_seconds;
        accel_ef.z -= GRAVITY_MSS;
        accel_body = dcm.transposed() * accel_ef;
    }

    // limit to 16G to match pixhawk
    float a_limit = GRAVITY_MSS*16;
    accel_body.x = constrain_float(accel_body.x, -a_limit, a_limit);
    accel_body.y = constrain_float(accel_body.y, -a_limit, a_limit);
    accel_body.z = constrain_float(accel_body.z, -a_limit, a_limit);

    // offset based on first position to account for offset in RF world
    if (position_offset.is_zero() || !is_zero(state.m_resetButtonHasBeenPressed)) {
        position_offset = position;
    }
    position -= position_offset;

    airspeed = state.m_airspeed_MPS;

    /* for pitot airspeed we need the airspeed along the X axis. We
       can't get that from m_airspeed_MPS, so instead we calculate it
       from wind vector and ground speed
     */
    wind_ef = Vector3f(state.m_windY_MPS,state.m_windX_MPS,state.m_windZ_MPS);
    Vector3f airspeed_3d_ef = velocity_ef - wind_ef;
    Vector3f airspeed3d = dcm.mul_transpose(airspeed_3d_ef);

    if (last_imu_rotation != ROTATION_NONE) {
        airspeed3d = sitl->ahrs_rotation_inv * airspeed3d;
    }
    airspeed_pitot = MAX(airspeed3d.x,0);

#if 0
    printf("WIND: %.1f %.1f %.1f AS3D %.1f %.1f %.1f\n",
           state.m_windX_MPS,
           state.m_windY_MPS,
           state.m_windZ_MPS,
           airspeed3d.x,
           airspeed3d.y,
           airspeed3d.z);
#endif

    battery_voltage = MAX(state.m_batteryVoltage_VOLTS, 0);
    battery_current = MAX(state.m_batteryCurrentDraw_AMPS, 0);
    rpm[0] = state.m_heliMainRotorRPM;
    rpm[1] = state.m_propRPM;

    /*
      the interlink interface supports 12 input channels
     */
    rcin_chan_count = 12;
    for (uint8_t i=0; i<rcin_chan_count; i++) {
        rcin[i] = state.rcin[i];
    }

    update_position();
    time_advance();
    uint64_t new_time_us = (state.m_currentPhysicsTime_SEC - initial_time_s)*1.0e6;
    if (new_time_us < time_now_us) {
        uint64_t dt_us = time_now_us - new_time_us;
        if (dt_us > 500000) {
            // time going backwards
            time_now_us = new_time_us;
        }
    } else {
        time_now_us = new_time_us;
    }

    last_time_s = state.m_currentPhysicsTime_SEC;

    last_velocity_ef = velocity_ef;

    // update magnetic field
    update_mag_field_bf();

    report_FPS();
}

/*
  report frame rates
 */
void FlightAxis::report_FPS(void)
{
    if (frame_counter++ % 1000 == 0) {
        if (!is_zero(last_frame_count_s)) {
            uint64_t frames = socket_frame_counter - last_socket_frame_counter;
            last_socket_frame_counter = socket_frame_counter;
            double dt = state.m_currentPhysicsTime_SEC - last_frame_count_s;
            printf("%.2f/%.2f FPS avg=%.2f\n",
                   frames / dt, 1000 / dt, 1.0/average_frame_time_s);
        } else {
            printf("Initial position %f %f %f\n", position.x, position.y, position.z);
        }
        last_frame_count_s = state.m_currentPhysicsTime_SEC;
    }
}

void FlightAxis::socket_creator(void)
{
    socket_pid = getpid();
    while (true) {
        pthread_mutex_lock(&sockmtx);
        while (socknext != nullptr) {
            pthread_cond_wait(&sockcond2, &sockmtx);
        }
        pthread_mutex_unlock(&sockmtx);
        auto *sck = new SocketAPM(false);
        if (sck == nullptr) {
            usleep(500);
            continue;
        }
        if (!sck->connect(controller_ip, controller_port)) {
            ::printf("connect failed\n");
            delete sck;
            usleep(5000);
            continue;
        }
        sck->set_blocking(false);
        socknext = sck;
        pthread_mutex_lock(&sockmtx);
        pthread_cond_broadcast(&sockcond1);
        pthread_mutex_unlock(&sockmtx);
    }
}

#endif // HAL_SIM_FLIGHTAXIS_ENABLED
