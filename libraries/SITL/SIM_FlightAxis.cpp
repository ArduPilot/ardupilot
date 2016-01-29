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
  simulator connector for FlightAxis
*/

#include "SIM_FlightAxis.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define FLIGHTAXIS_SERVER_IP "192.168.2.48"
#define FLIGHTAXIS_SERVER_PORT 18083

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

FlightAxis::FlightAxis(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
{
    use_time_sync = false;
    rate_hz = 250 / target_speedup;
    heli_demix = strstr(frame_str, "helidemix") != NULL;
    rev4_servos = strstr(frame_str, "rev4") != NULL;
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
            break;
        }
        p += strlen(keytable[i].key) + 1;
        keytable[i].ref = atof(p);
        // this assumes key order
        reply = p;
    }
}


/*
  make a SOAP request, returning body of reply
 */
char *FlightAxis::soap_request(const char *action, const char *fmt, ...)
{
    va_list ap;
    char *req1;
    
    va_start(ap, fmt);
    vasprintf(&req1, fmt, ap);
    va_end(ap);

    //printf("%s\n", req1);
    
    // open SOAP socket to FlightAxis
    SocketAPM sock(false);
    if (!sock.connect(FLIGHTAXIS_SERVER_IP, FLIGHTAXIS_SERVER_PORT)) {
        free(req1);
        return nullptr;
    }
    sock.set_blocking(false);

    char *req;
    asprintf(&req, R"(POST / HTTP/1.1
soapaction: '%s'
content-length: %u
content-type: text/xml;charset='UTF-8'
Connection: Keep-Alive

%s)",
             action,
             (unsigned)strlen(req1), req1);
    sock.send(req, strlen(req));
    free(req1);
    free(req);
    char reply[10000];
    memset(reply, 0, sizeof(reply));
    ssize_t ret = sock.recv(reply, sizeof(reply)-1, 1000);
    if (ret <= 0) {
        printf("No data\n");
        return nullptr;
    }
    char *p = strstr(reply, "Content-Length: ");
    if (p == nullptr) {
        printf("No Content-Length\n");
        return nullptr;
    }

    // get the content length
    uint32_t content_length = strtoul(p+16, NULL, 10);
    char *body = strstr(p, "\r\n\r\n");
    if (body == nullptr) {
        printf("No body\n");
        return nullptr;
    }
    body += 4;

    // get the rest of the body
    uint32_t expected_length = content_length + (body - reply);
    if (expected_length >= sizeof(reply)) {
        printf("Reply too large %u\n", expected_length);
        return nullptr;
    }
    while (ret < expected_length) {
        ssize_t ret2 = sock.recv(&reply[ret], sizeof(reply)-(1+ret), 100);
        if (ret2 <= 0) {
            return nullptr;
        }
        ret += ret2;
    }
    return strdup(reply);
}
    


void FlightAxis::exchange_data(const struct sitl_input &input)
{
    if (!controller_started) {
        printf("Starting controller\n");
        // call a restore first. This allows us to connect after the aircraft is changed in RealFlight
        char *reply = soap_request("RestoreOriginalControllerDevice", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>
</soap:Body>
</soap:Envelope>)");
        free(reply);
        reply = soap_request("InjectUAVControllerInterface", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>
</soap:Body>
</soap:Envelope>)");
        free(reply);
        activation_frame_counter = frame_counter;
        controller_started = true;
    }

    float scaled_servos[8];
    for (uint8_t i=0; i<8; i++) {
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
    
    
    char *reply = soap_request("ExchangeData", R"(<?xml version='1.0' encoding='UTF-8'?><soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<ExchangeData>
<pControlInputs>
<m-selectedChannels>255</m-selectedChannels>
<m-channelValues-0to1>
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
                               scaled_servos[0],
                               scaled_servos[1],
                               scaled_servos[2],
                               scaled_servos[3],
                               scaled_servos[4],
                               scaled_servos[5],
                               scaled_servos[6],
                               scaled_servos[7]);

    if (reply) {
        parse_reply(reply);
        free(reply);
    }
}
    
    
/*
  update the FlightAxis simulation by one time step
 */
void FlightAxis::update(const struct sitl_input &input)
{
    Vector3f last_velocity_ef = velocity_ef;
    
    exchange_data(input);

    float dt_seconds = state.m_currentPhysicsTime_SEC - last_time_s;
    if (dt_seconds <= 0) {
        return;
    }

    if (initial_time_s <= 0) {
        dt_seconds = 0.001f;
        initial_time_s = state.m_currentPhysicsTime_SEC - dt_seconds;
    }
    
    Quaternion quat(state.m_orientationQuaternion_W,
                    state.m_orientationQuaternion_Z,
                    -state.m_orientationQuaternion_Y,
                    -state.m_orientationQuaternion_Z);
    quat.rotation_matrix(dcm);
    dcm.from_euler(radians(state.m_roll_DEG),
                   radians(state.m_inclination_DEG),
                   -radians(state.m_azimuth_DEG));
    Quaternion quat2;
    quat2.from_rotation_matrix(dcm);
    gyro = Vector3f(radians(constrain_float(state.m_rollRate_DEGpSEC, -2000, 2000)),
                    radians(constrain_float(state.m_pitchRate_DEGpSEC, -2000, 2000)),
                    -radians(constrain_float(state.m_yawRate_DEGpSEC, -2000, 2000))) * target_speedup;
    velocity_ef = Vector3f(state.m_velocityWorldU_MPS,
                             state.m_velocityWorldV_MPS,
                             state.m_velocityWorldW_MPS);
    position = Vector3f(state.m_aircraftPositionY_MTR,
                        state.m_aircraftPositionX_MTR,
                        -state.m_altitudeAGL_MTR);

    // offset based on first position to account for offset in RF world
    if (position_offset.is_zero()) {
        position_offset = position;
    }
    position -= position_offset;

    // the accel values given in the state are very strange. Calculate
    // it from delta-velocity instead, although this does introduce
    // noise
    Vector3f accel_ef = (velocity_ef - last_velocity_ef) / dt_seconds;
    accel_ef.z -= GRAVITY_MSS;
    accel_body = dcm.transposed() * accel_ef;
    accel_body.x = constrain_float(accel_body.x, -16, 16);
    accel_body.y = constrain_float(accel_body.y, -16, 16);
    accel_body.z = constrain_float(accel_body.z, -16, 16);

    airspeed = state.m_airspeed_MPS;

    battery_voltage = state.m_batteryVoltage_VOLTS;
    battery_current = state.m_batteryCurrentDraw_AMPS;
    rpm1 = state.m_propRPM;
    rpm2 = state.m_heliMainRotorRPM;
    
    update_position();
    time_now_us = (state.m_currentPhysicsTime_SEC - initial_time_s)*1.0e6;

    if (frame_counter++ % 1000 == 0) {
        if (last_frame_count_s != 0) {
            printf("%.2f FPS\n",
                   1000 / (state.m_currentPhysicsTime_SEC - last_frame_count_s));
            printf("(%.3f %.3f %.3f %.3f) (%.3f %.3f %.3f %.3f)\n",
                   quat.q1, quat.q2, quat.q3, quat.q4, 
                   quat2.q1, quat2.q2, quat2.q3, quat2.q4);
        } else {
            printf("Initial position %f %f %f\n", position.x, position.y, position.z);
        }
        last_frame_count_s = state.m_currentPhysicsTime_SEC;
    }

    last_time_s = state.m_currentPhysicsTime_SEC;
}

} // namespace SITL
