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

#include "AP_VisualOdom_NoopLoop.h"

#if HAL_VISUALODOM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#define NOOPLOOP_HEADER                     0x55    // message header
#define NOOPLOOP_NODE_FRAME2_FRAMELEN_MAX   4096    // frames should be less than 4k bytes
#define NOOPLOOP_NODE_FRAME2_SYSTIME        6       // start of 4 bytes holding system time in ms
#define NOOPLOOP_NODE_FRAME2_PRECISION_X    10      // start of 1 byte holding precision in m*100 in x axis
#define NOOPLOOP_NODE_FRAME2_PRECISION_Y    11      // start of 1 byte holding precision in m*100 in y axis
#define NOOPLOOP_NODE_FRAME2_PRECISION_Z    12      // start of 1 byte holding precision in m*100 in y axis
#define NOOPLOOP_NODE_FRAME2_POSX           13      // start of 3 bytes holding x position in m*1000
#define NOOPLOOP_NODE_FRAME2_POSY           16      // start of 3 bytes holding y position in m*1000
#define NOOPLOOP_NODE_FRAME2_POSZ           19      // start of 3 bytes holding z position in m*1000
#define NOOPLOOP_NODE_FRAME2_VELX           22      // start of 3 bytes holding x velocity in m/s*10000
#define NOOPLOOP_NODE_FRAME2_VELY           25      // start of 3 bytes holding y velocity in m/s*10000
#define NOOPLOOP_NODE_FRAME2_VELZ           28      // start of 3 bytes holding z velocity in m/s*10000
#define NOOPLOOP_NODE_FRAME2_ROLL           76      // start of 2 bytes holding roll lean angle in degrees
#define NOOPLOOP_NODE_FRAME2_PITCH          78      // start of 2 bytes holding pitch lean angle in degrees
#define NOOPLOOP_NODE_FRAME2_YAW            80      // start of 2 bytes holding yaw lean angle in degrees

extern const AP_HAL::HAL& hal;

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_VisualOdom_NoopLoop::AP_VisualOdom_NoopLoop(AP_VisualOdom &frontend) :
    AP_VisualOdom_Backend(frontend)
{
    // find uart
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_NoopLoop, 0);
}

// consume vision position estimate data and send to EKF. distances in meters
void AP_VisualOdom_NoopLoop::update()
{
    // return immediately if not serial port
    if (_uart == nullptr) {
        return;
    }

    // check uart for any incoming messages
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        const int16_t r = _uart->read();
        if ((r < 0) || (r > 0xFF)) {
            continue;
        }
        if (parse_byte((uint8_t)r)) {
            parse_msgbuf();
        }
    }
}

// process one byte received on serial port
// returns true if a message has been successfully parsed
// message is stored in _msgbuf
bool AP_VisualOdom_NoopLoop::parse_byte(uint8_t b)
{
    // process byte depending upon current state
    switch (_state) {

    case ParseState::HEADER:
        if (b == NOOPLOOP_HEADER) {
            _msgbuf[0] = b;
            _msg_len = 1;
            _crc_expected = b;
            _state = ParseState::FUNCTION_MARK;
        }
        break;

    case ParseState::FUNCTION_MARK:
        if (b == (uint8_t)FunctionMark::NODE_FRAME2) {
            _msgbuf[_msg_len] = b;
            _msg_len++;
            _crc_expected += b;
            _state = ParseState::LEN_L;
        } else {
            _state = ParseState::HEADER;
        }
        break;

    case ParseState::LEN_L:
        _msgbuf[_msg_len] = b;
        _msg_len++;
        _crc_expected += b;
        _state = ParseState::LEN_H;
        break;

    case ParseState::LEN_H:
        // extract and sanity check frame length
        _frame_len = (uint16_t)b << 8 | _msgbuf[_msg_len-1];
        if (_frame_len > NOOPLOOP_NODE_FRAME2_FRAMELEN_MAX) {
            _state = ParseState::HEADER;
            gcs().send_text(MAV_SEVERITY_CRITICAL,"invalid len:%d", (int)_frame_len);
        } else {
            _msgbuf[_msg_len] = b;
            _msg_len++;
            _crc_expected += b;
            _state = ParseState::PAYLOAD;
        }
        break;

    case ParseState::PAYLOAD:
        // add byte to buffer if there is room
        if (_msg_len < msgbuf_len_max) {
            _msgbuf[_msg_len] = b;
        }
        _msg_len++;
        if (_msg_len >= _frame_len) {
            _state = ParseState::HEADER;
            // check crc
            return (b == _crc_expected);
        } else {
            _crc_expected += b;
        }
        break;
    }

    return false;
}

// parse msgbuf and update the EKF
void AP_VisualOdom_NoopLoop::parse_msgbuf()
{
    // a message has been received
    const uint32_t now_ms = AP_HAL::millis();
    _last_update_ms = now_ms;

    // remote system time in milliseconds
    // To-Do: add jitter correction
    const uint32_t remote_systime_ms = (uint32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_SYSTIME+3] << 24 | (uint32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_SYSTIME+2] << 16 | (uint32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_SYSTIME+1] << 8 | (uint32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_SYSTIME];
    const uint64_t remote_systime_us = remote_systime_ms * 1000UL;

    // estimated precision for x,y,z position in meters
    const float precision_x = _msgbuf[NOOPLOOP_NODE_FRAME2_PRECISION_X] * 0.01;
    const float precision_y = _msgbuf[NOOPLOOP_NODE_FRAME2_PRECISION_Y] * 0.01;
    const float precision_z = _msgbuf[NOOPLOOP_NODE_FRAME2_PRECISION_Z] * 0.01;
    const float pos_err = constrain_float(cbrtf(sq(precision_x)+sq(precision_y)+sq(precision_z)), _frontend.get_pos_noise(), 100.0f);

    // x,y,z position in m*1000 in NEU frame
    const int32_t pos_x = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSX+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSX+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSX] << 8) >> 8;
    const int32_t pos_y = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSY+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSY+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSY] << 8) >> 8;
    const int32_t pos_z = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSZ+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSZ+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_POSZ] << 8) >> 8;

    // position scaled to meters in NED
    const Vector3f pos_m {pos_x * 0.001f, pos_y * 0.001f, -pos_z * 0.001f};

    // x,y,z velocity in m/s*10000 in NEU
    const int32_t vel_x = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_VELX+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_VELX+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_VELX] << 8) >> 8;
    const int32_t vel_y = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_VELY+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_VELY+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_VELY] << 8) >> 8;
    const int32_t vel_z = ((int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_VELZ+2] << 24 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_VELZ+1] << 16 | (int32_t)_msgbuf[NOOPLOOP_NODE_FRAME2_VELZ] << 8) >> 8;

    // velocity scaled to m/s in NED
    const Vector3f vel_ms {vel_x * 0.0001f, vel_y * 0.0001f, -vel_z * 0.0001f};

    // sensor does not provide accurate attitude or reset information
    const Quaternion att_none;
    const float att_err = _frontend.get_yaw_noise();
    const uint8_t reset_counter = 0;

    // write data to EKF
    AP::ahrs().writeExtNavData(pos_m, att_none, pos_err, att_err, now_ms, _frontend.get_delay_ms(), reset_counter);
    AP::ahrs().writeExtNavVelData(vel_ms, _frontend.get_vel_noise(), now_ms, _frontend.get_delay_ms());

    // log sensor data
    AP::logger().Write_VisualPosition(remote_systime_us, now_ms, pos_m.x, pos_m.y, pos_m.z, 0.0f, 0.0f, 0.0f, pos_err, att_err, reset_counter);
    AP::logger().Write_VisualVelocity(remote_systime_us, now_ms, vel_ms, _frontend.get_vel_noise(), reset_counter);

    // debug
    /*static uint8_t counter = 0;
    counter++;
    if (counter >= 10) {
        counter = 0;
        gcs().send_text(MAV_SEVERITY_CRITICAL,"t:%llu, x:%4.2f y:%4.2f z:%4.2f vx:%4.2f vy:%4.2f vz:%4.2f",
                    //(unsigned long long)remote_systime_ms,
                    (unsigned long long)remote_systime_us,
                    (double)pos_m.x,
                    (double)pos_m.y,
                    (double)pos_m.z,
                    (double)vel_ms.x,
                    (double)vel_ms.y,
                    (double)vel_ms.z);
    }
    */
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
bool AP_VisualOdom_NoopLoop::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (_uart == nullptr) {
        hal.util->snprintf(failure_msg, failure_msg_len, "No SERIALx_PROTOCOL = 29");
        return false;
    }

    // exit immediately if not healthy
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "not healthy");
        return false;
    }

    return true;
}

#endif
