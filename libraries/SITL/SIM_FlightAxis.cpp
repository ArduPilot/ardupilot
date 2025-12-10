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

#if AP_SIM_FLIGHTAXIS_ENABLED

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

const AP_Param::GroupInfo FlightAxis::var_info[] = {
    // @Param: OPTS
    // @DisplayName: FlightAxis options
    // @Description: Bitmask of FlightAxis options
    // @Bitmask: 0: Reset position on startup
    // @Bitmask: 1: Swap first 4 and last 4 servos (for quadplane testing)
    // @Bitmask: 2: Demix heli servos and send roll/pitch/collective/yaw
    // @Bitmask: 3: Don't print frame rate stats
    // @Bitmask: 4: Don't log Dt stats
    // @User: Advanced
    AP_GROUPINFO("OPTS", 1, FlightAxis, _options, uint32_t(Option::ResetPosition)),

    // @Param: SAMPLEHZ
    // @DisplayName: FlightAxis IMU synthetic sample rate
    // @Description: FlightAxis IMU synthetic sample rate
    // @User: Advanced
    AP_GROUPINFO("SAMPLEHZ", 2, FlightAxis, _samplehz, 2000),
    AP_GROUPEND
};

/*
  we use a thread for socket creation to reduce the impact of socket
  creation latency. These condition variables are used to synchronise
  the thread
 */
static HAL_BinarySemaphore sockcond1;
static HAL_BinarySemaphore sockcond2;

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

#if defined(CYGWIN_BUILD)
#include <windows.h>

/*
  usleep on cygwin is broken for small times: https://stackoverflow.com/questions/6254703/thread-sleep-for-less-than-1-millisecond
  the solution is to define our own busy wait sleep
 */
static void us_wait(int64_t microseconds) {
    LARGE_INTEGER frequency;
    LARGE_INTEGER start_time, current_time;

    QueryPerformanceFrequency(&frequency); // Get the frequency of the performance counter
    QueryPerformanceCounter(&start_time);  // Get the current time

    // Calculate the number of ticks required for the specified microseconds
    int64_t ticks_to_wait = (int64_t)microseconds * frequency.QuadPart / 1000000;
    int64_t end_ticks = start_time.QuadPart + ticks_to_wait;

    // Busy-wait until the desired time has elapsed
    do {
        QueryPerformanceCounter(&current_time);
    } while (current_time.QuadPart < end_ticks);
}
#else
#define us_wait(x) usleep(x)
#endif

FlightAxis::FlightAxis(const char *frame_str) :
    Aircraft(frame_str)
{
    AP::sitl()->models.flightaxis_ptr = this;
    AP_Param::setup_object_defaults(this, var_info);

    use_time_sync = false;
    flightaxis_sync_imus_to_frames = true;  // tell the IMUs to advance on each frame that is processed
    rate_hz = 250 / target_speedup;
    if(strstr(frame_str, "helidemix") != nullptr) {
        _options.set(_options | uint32_t(Option::HeliDemix));
    }
    if(strstr(frame_str, "rev4") != nullptr) {
        _options.set(_options | uint32_t(Option::Rev4Servos));
    }

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

    // consumer/producer pattern
    while (sock == nullptr) {
        sock_outsem.wait_blocking();
        sock = socknext;
        socknext = nullptr;
        sock_insem.signal();
    }

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

bool FlightAxis::exchange_data(const struct sitl_input &input)
{
    if (!sock &&
        (!controller_started ||
         is_zero(next_state.m_flightAxisControllerIsActive) ||
         !is_zero(next_state.m_resetButtonHasBeenPressed))) {
        start_controller();
    }

    if (!sock) {
        send_request_message(input);
    }
    if (sock) {
        bool ret = process_reply_message();
        if (ret) {
            us_wait(250);
            send_request_message(input);
        }
        return ret;
    }
    return false;
}

void FlightAxis::start_controller()
{
    printf("Starting controller at %s\n", controller_ip);
    // call a restore first. This allows us to connect after the aircraft is changed in RealFlight
    soap_request_start("RestoreOriginalControllerDevice", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>
</soap:Body>
</soap:Envelope>)");
    soap_request_end(1000UL);
    if(option_is_set(Option::ResetPosition)) {
        soap_request_start("ResetAircraft", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<ResetAircraft><a>1</a><b>2</b></ResetAircraft>
</soap:Body>
</soap:Envelope>)");
        soap_request_end(1000UL);
    }
    soap_request_start("InjectUAVControllerInterface", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>
</soap:Body>
</soap:Envelope>)");
    soap_request_end(1000UL);
    activation_frame_counter = frame_counter;
    controller_started = true;
}

void FlightAxis::send_request_message(const struct sitl_input &input)
{ 
    // maximum number of servos to send is 12 with new FlightAxis
    float scaled_servos[12] {};
    uint16_t valid_channels = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(scaled_servos); i++) {
        if (input.servos[i] == 0) {
            continue;
        }
        scaled_servos[i] = (input.servos[i] - 1000) / 1000.0f;
        valid_channels |= 1U << i;
    }

    if (option_is_set(Option::Rev4Servos)) {
        // swap first 4 and second 4 servos, for quadplane testing
        float saved[4];
        memcpy(saved, &scaled_servos[0], sizeof(saved));
        memcpy(&scaled_servos[0], &scaled_servos[4], sizeof(saved));
        memcpy(&scaled_servos[4], saved, sizeof(saved));
        // and the validity flags:
        uint16_t new_channels = 0;
        new_channels |= (valid_channels & 0b1111111100000000);
        new_channels |= (valid_channels & 0b11110000) >> 4;
        new_channels |= (valid_channels & 0b00001111) << 4;
        valid_channels = new_channels;
    }

    if (option_is_set(Option::HeliDemix)) {
        // FlightAxis expects "roll/pitch/collective/yaw" input
        float swash1 = scaled_servos[0];
        float swash2 = scaled_servos[1];
        float swash3 = scaled_servos[2];

        float roll_rate = swash1 - swash2;
        float pitch_rate = ((swash1+swash2) / 2.0f - swash3);
        float col = (swash1 + swash2 + swash3) / 3.0;

        scaled_servos[0] = constrain_float(roll_rate + 0.5, 0, 1);
        scaled_servos[1] = constrain_float(pitch_rate + 0.5, 0, 1);
        scaled_servos[2] = constrain_float(col, 0, 1);
    }

    const uint16_t channels = hal.scheduler->is_system_initialized()?4095:0;
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

bool FlightAxis::process_reply_message()
{
    char *reply = nullptr;
    reply = soap_request_end(1);
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

    if (reply) {
        sock_error_count = 0;
        last_recv_sec = timestamp_sec();
        parse_reply(reply);
        free(reply);
        return true;
    }
    return false;
}

bool FlightAxis::wait_for_sample(const struct sitl_input &input)
{
    double dt_seconds = 0;
    const float SAMPLE_INTERVAL_S = 1.0f / _samplehz.get();
    const float SAMPLE_INTERVAL_MIN_S = (SAMPLE_INTERVAL_S/2);   // smallest interval before moving on to the next

    double lastt_s = state.m_currentPhysicsTime_SEC;

    if (is_zero(prev_state.m_currentPhysicsTime_SEC)) {
        if (exchange_data(input)) {
            state = prev_state = next_state;
            sample_interval_s = SAMPLE_INTERVAL_S;
        }
        return false;
    }

    // get a new frame if we have processed all the whole samples in the previous one
    if (state.m_currentPhysicsTime_SEC + SAMPLE_INTERVAL_MIN_S >= next_state.m_currentPhysicsTime_SEC) {
        prev_state = next_state;
        do {
            if (exchange_data(input)) {  // updates next_state
                state = prev_state; // ensure state is at the beginning of the data range

                if (is_zero(last_time_s)) {
                    last_time_s = state.m_currentPhysicsTime_SEC - SAMPLE_INTERVAL_S;
                }
                dt_seconds = next_state.m_currentPhysicsTime_SEC - last_dt_sample_s;
                last_delta_time_s = next_state.m_currentPhysicsTime_SEC - last_time_s;
                last_dt_sample_s = next_state.m_currentPhysicsTime_SEC;
            }
        } while (is_zero(dt_seconds));
        // adjust the sample interval so that the number of samples in a frame is an integer
        // this prevents very small dt's which can cause havoc with flight control
        sample_interval_s = dt_seconds / roundf(dt_seconds / SAMPLE_INTERVAL_S);
    }

    double new_time = MIN(next_state.m_currentPhysicsTime_SEC, state.m_currentPhysicsTime_SEC + sample_interval_s);
    state = interpolate_frame(next_state, prev_state, new_time);

    socket_frame_counter++;

    double dt = state.m_currentPhysicsTime_SEC - lastt_s;
    if (dt > 0 && dt < 0.1) {
        if (average_frame_time_s < 1.0e-6) {
            average_frame_time_s = dt;
        }
        average_frame_time_s = average_frame_time_s * 0.98 + dt * 0.02;
    }
#if HAL_LOGGING_ENABLED
    if (!(option_is_set(Option::NoDtLog))) {
        uint64_t time_now = uint64_t(state.m_currentPhysicsTime_SEC * 1.0e6);
// @LoggerMessage: RF
// @Description: RealFlight mode messages
// @Field: TimeUS: Time since system startup
// @Field: Dt: delta time between this frame and the previous frame
// @Field: Fps: frames-per-second implied by the current delta time
        AP::logger().WriteStreaming("RF", "TimeUS,Dt,Fps", "QdI", time_now, dt, uint32_t(roundf(1/dt)));
    }
#endif
    if (last_time_s > 0) {
        if (dt_seconds > 0 && dt_seconds < 0.1) {
            if (is_zero(average_delta_time_s)) {
                average_delta_time_s = dt_seconds;
            }
            average_delta_time_s = average_delta_time_s * 0.98 + dt_seconds * 0.02;
        }
    }

    if (is_equal(last_time_s, state.m_currentPhysicsTime_SEC)) {
        AP_HAL::panic("Time did not move");
    }

    return true;
}

/*
  update the FlightAxis simulation by one time step
 */
void FlightAxis::update(const struct sitl_input &input)
{
    if (!wait_for_sample(input)) {
        return;
    }

    double dt_seconds = state.m_currentPhysicsTime_SEC - last_time_s;
    if (dt_seconds < 0) {
        // cope with restarting RealFlight while connected
        initial_time_s = time_now_us * 1.0e-6f;
        last_time_s = state.m_currentPhysicsTime_SEC;
        position_offset.zero();
        return;
    }
    
    // initialize timer
    if (initial_time_s <= 0) {
        time_now_us = 1;    // prevent time going backwards
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
        airspeed3d = sitl->ahrs_rotation * airspeed3d;
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
    motor_mask = 3;

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
        //printf("Time going backwards by %u\n", uint32_t(time_now_us - new_time_us));
        uint64_t dt_us = time_now_us - new_time_us;
        if (dt_us > 500000) {
            // time going backwards
            time_now_us = new_time_us;
        }
    } else {
        uint64_t dt_us = new_time_us - time_now_us;
        const uint64_t glitch_threshold_us = 50000;
        const uint64_t glitch_max_us = 2000000;
        if (dt_us > glitch_threshold_us && dt_us < glitch_max_us) {
            // we've had a network glitch, compensate by advancing initial time
            float adjustment_s = (dt_us-glitch_threshold_us)*1.0e-6;
            initial_time_s += adjustment_s;
            printf("glitch %.2fs\n", adjustment_s);
            dt_us = glitch_threshold_us;
            glitch_count++;
        }
        if (dt_us) {    // adjust the frame time to match the dt which will vary over time
            adjust_frame_time(1.0e6/dt_us);
        }
        time_now_us += dt_us;
    }

    last_time_s = state.m_currentPhysicsTime_SEC;
    last_velocity_ef = velocity_ef;

    // update magnetic field
    update_mag_field_bf();

    // one rangefinder
    if (is_positive(dcm.c.z)) {
        rangefinder_m[0] = state.m_altitudeAGL_MTR / dcm.c.z;
    } else {
        rangefinder_m[0] = nanf("");
    }

    report_FPS();
}

struct FlightAxis::state FlightAxis::interpolate_frame(struct state& new_state, struct state& old_state, double new_time)
{
    struct state intermediate_state = old_state;
    double dt = new_state.m_currentPhysicsTime_SEC - old_state.m_currentPhysicsTime_SEC;
    double interval = new_time - old_state.m_currentPhysicsTime_SEC;

#define INTERPOLATE(name) (intermediate_state.name = (old_state.name + interval * (new_state.name - old_state.name) / dt))
    INTERPOLATE(m_airspeed_MPS);
    INTERPOLATE(m_altitudeAGL_MTR);
    INTERPOLATE(m_pitchRate_DEGpSEC);
    INTERPOLATE(m_rollRate_DEGpSEC);
    INTERPOLATE(m_yawRate_DEGpSEC);
    INTERPOLATE(m_aircraftPositionX_MTR);
    INTERPOLATE(m_aircraftPositionY_MTR);
    INTERPOLATE(m_velocityWorldU_MPS);
    INTERPOLATE(m_velocityWorldV_MPS);
    INTERPOLATE(m_velocityWorldW_MPS);
    INTERPOLATE(m_accelerationBodyAX_MPS2);
    INTERPOLATE(m_accelerationBodyAY_MPS2);
    INTERPOLATE(m_accelerationBodyAZ_MPS2);
    INTERPOLATE(m_windX_MPS);
    INTERPOLATE(m_windY_MPS);
    INTERPOLATE(m_windZ_MPS);
    INTERPOLATE(m_propRPM);
    INTERPOLATE(m_heliMainRotorRPM);
    INTERPOLATE(m_batteryVoltage_VOLTS);
    INTERPOLATE(m_batteryCurrentDraw_AMPS);
    INTERPOLATE(m_orientationQuaternion_X);
    INTERPOLATE(m_orientationQuaternion_Y);
    INTERPOLATE(m_orientationQuaternion_Z);
    INTERPOLATE(m_orientationQuaternion_W);
    intermediate_state.m_currentPhysicsTime_SEC = new_time;

    return intermediate_state;
}

/*
  report frame rates
 */
void FlightAxis::report_FPS(void)
{
    if (frame_counter++ % _samplehz == 0) {
        if (!is_zero(last_frame_count_s)) {
            uint64_t frames = socket_frame_counter - last_socket_frame_counter;
            last_socket_frame_counter = socket_frame_counter;
            double dt = state.m_currentPhysicsTime_SEC - last_frame_count_s;
            if(!option_is_set(Option::SilenceFPS)) {
                printf("%.2f/%.2f FPS avg=%.2f FPS net=%.2f glitches=%u\n",
                    frames / dt, _samplehz / dt, 1.0/average_frame_time_s, 1.0/average_delta_time_s, unsigned(glitch_count));
            }
        } else {
            printf("Initial position %f %f %f\n", position.x, position.y, position.z);
        }
        last_frame_count_s = state.m_currentPhysicsTime_SEC;
    }
}

#include <errno.h>

void FlightAxis::socket_creator(void)
{
    socket_pid = getpid();
    while (true) {
        while (socknext != nullptr) {
            sock_insem.wait_blocking();
        }
        auto *sck = NEW_NOTHROW SocketAPM_native(false);
        if (sck == nullptr) {
            us_wait(500);
            continue;
        }
        /*
          don't let the connection take more than 10ms (100Hz). Longer
          than this and we are better off trying for a new socket
         */
        if (!sck->connect_timeout(controller_ip, controller_port, 10)) {
            ::printf("connect failed\n");
            delete sck;
            us_wait(500);
            continue;
        }
        sck->set_blocking(false);
        socknext = sck;
        sock_outsem.signal();
    }
}

#endif // AP_SIM_FLIGHTAXIS_ENABLED
