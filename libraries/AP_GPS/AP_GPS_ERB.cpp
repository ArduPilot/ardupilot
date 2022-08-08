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

 //
 //  Emlid Reach Binary (ERB) GPS driver for ArduPilot.
 //  ERB protocol: http://files.emlid.com/ERB.pdf

#include "AP_GPS.h"
#include "AP_GPS_ERB.h"

#if AP_GPS_ERB_ENABLED

#define ERB_DEBUGGING 0

#define STAT_FIX_VALID 0x01

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#if ERB_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

// Process bytes available from the stream
//
// The stream is assumed to contain only messages we recognise.  If it
// contains other messages, and those messages contain the preamble
// bytes, it is possible for this code to fail to synchronise to the
// stream immediately.  Without buffering the entire message and
// re-processing it from the top, this is unavoidable. The parser
// attempts to avoid this when possible.
//
bool
AP_GPS_ERB::read(void)
{
    uint8_t data;
    int16_t numc;
    bool parsed = false;

    numc = port->available();
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        data = port->read();

        reset:
        switch(_step) {

        // Message preamble detection
        //
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            Debug("reset %u", __LINE__);
            FALLTHROUGH;
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;

        // Message header processing
        //
        case 2:
            _step++;
            _msg_id = data;
            _ck_b = _ck_a = data;                       // reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length = data;                     // payload length low byte
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length += (uint16_t)(data<<8);
            _payload_counter = 0;                       // prepare to receive payload
            break;

        // Receive message data
        //
        case 5:
            _ck_b += (_ck_a += data);                   // checksum byte
            if (_payload_counter < sizeof(_buffer)) {
                _buffer[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;

        // Checksum and message processing
        //
        case 6:
            _step++;
            if (_ck_a != data) {
                Debug("bad cka %x should be %x", data, _ck_a);
                _step = 0;
                goto reset;
            }
            break;
        case 7:
            _step = 0;
            if (_ck_b != data) {
                Debug("bad ckb %x should be %x", data, _ck_b);
                break;                                  // bad checksum
            }

            if (_parse_gps()) {
                parsed = true;
            }
            break;
        }
    }
    return parsed;
}

bool
AP_GPS_ERB::_parse_gps(void)
{
    switch (_msg_id) {
    case MSG_VER:
        Debug("Version of ERB protocol %u.%u.%u",
              _buffer.ver.ver_high,
              _buffer.ver.ver_medium,
              _buffer.ver.ver_low);
        break;
    case MSG_POS:
        Debug("Message POS");
        _last_pos_time        = _buffer.pos.time;
        state.location.lng    = (int32_t)(_buffer.pos.longitude * (double)1e7);
        state.location.lat    = (int32_t)(_buffer.pos.latitude * (double)1e7);
        state.location.alt    = gps.get_location_altitude_frame(int32_t(_buffer.pos.altitude_msl * 100.0), int32_t(_buffer.pos.altitude_ellipsoid * 100.0));

        state.height_above_WGS84 = float(_buffer.pos.altitude_ellipsoid);
        state.have_height_above_WGS84 = true;

        state.status          = next_fix;
        _new_position = true;
        state.horizontal_accuracy = _buffer.pos.horizontal_accuracy * 1.0e-3f;
        state.vertical_accuracy = _buffer.pos.vertical_accuracy * 1.0e-3f;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        break;
    case MSG_STAT:
        Debug("Message STAT fix_status=%u fix_type=%u",
              _buffer.stat.fix_status,
              _buffer.stat.fix_type);
        if (_buffer.stat.fix_status & STAT_FIX_VALID) {
            if (_buffer.stat.fix_type == AP_GPS_ERB::FIX_FIX) {
                next_fix = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
            } else if (_buffer.stat.fix_type == AP_GPS_ERB::FIX_FLOAT) {
                next_fix = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
            } else if (_buffer.stat.fix_type == AP_GPS_ERB::FIX_SINGLE) {
                next_fix = AP_GPS::GPS_OK_FIX_3D;
            } else {
                next_fix = AP_GPS::NO_FIX;
                state.status = AP_GPS::NO_FIX;
            }
        } else {
            next_fix = AP_GPS::NO_FIX;
            state.status = AP_GPS::NO_FIX;
        }
        state.num_sats = _buffer.stat.satellites;
        if (next_fix >= AP_GPS::GPS_OK_FIX_3D) {
            // use the uart receive time to make packet timestamps more accurate
            set_uart_timestamp(_payload_length + sizeof(erb_header) + 2);
            state.last_gps_time_ms = AP_HAL::millis();
            state.time_week_ms    = _buffer.stat.time;
            state.time_week       = _buffer.stat.week;
        }
        break;
    case MSG_DOPS:
        Debug("Message DOPS");
        state.hdop = _buffer.dops.hDOP;
        state.vdop = _buffer.dops.vDOP;
        break;
    case MSG_VEL:
        Debug("Message VEL");
        _last_vel_time         = _buffer.vel.time;
        state.ground_speed     = _buffer.vel.speed_2d * 0.01f;        // m/s
        // Heading 2D deg * 100000 rescaled to deg * 100
        state.ground_course = wrap_360(_buffer.vel.heading_2d * 1.0e-5f);
        state.have_vertical_velocity = true;
        state.velocity.x = _buffer.vel.vel_north * 0.01f;
        state.velocity.y = _buffer.vel.vel_east * 0.01f;
        state.velocity.z = _buffer.vel.vel_down * 0.01f;
        state.have_speed_accuracy = true;
        state.speed_accuracy = _buffer.vel.speed_accuracy * 0.01f;
        _new_speed = true;
        break;
    case MSG_RTK:
        Debug("Message RTK");
        state.rtk_baseline_coords_type = RTK_BASELINE_COORDINATE_SYSTEM_NED;
        state.rtk_num_sats      = _buffer.rtk.base_num_sats;
        if (_buffer.rtk.age_cs == 0xFFFF) {
            state.rtk_age_ms    = 0xFFFFFFFF;
        } else {
            state.rtk_age_ms    = _buffer.rtk.age_cs * 10;
        }
        state.rtk_baseline_x_mm = _buffer.rtk.baseline_N_mm;
        state.rtk_baseline_y_mm = _buffer.rtk.baseline_E_mm;
        state.rtk_baseline_z_mm = _buffer.rtk.baseline_D_mm;
        state.rtk_accuracy      = _buffer.rtk.ar_ratio;

        state.rtk_week_number   = _buffer.rtk.base_week_number;
        state.rtk_time_week_ms  = _buffer.rtk.base_time_week_ms;
        break;
    default:
        Debug("Unexpected message 0x%02x", (unsigned)_msg_id);
        return false;
    }
    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed && _last_vel_time == _last_pos_time) {
        _new_speed = _new_position = false;
        _fix_count++;
        return true;
    }
    return false;
}

/*
  detect a ERB GPS. Adds one byte, and returns true if the stream
  matches a ERB
 */
bool
AP_GPS_ERB::_detect(struct ERB_detect_state &state, uint8_t data)
{
reset:
    switch (state.step) {
        case 1:
            if (PREAMBLE2 == data) {
                state.step++;
                break;
            }
            state.step = 0;
            FALLTHROUGH;
        case 0:
            if (PREAMBLE1 == data)
                state.step++;
            break;
        case 2:
            state.step++;
            state.ck_b = state.ck_a = data;
            break;
        case 3:
            state.step++;
            state.ck_b += (state.ck_a += data);
            state.payload_length = data;
            break;
        case 4:
            state.step++;
            state.ck_b += (state.ck_a += data);
            state.payload_counter = 0;
            break;
        case 5:
            state.ck_b += (state.ck_a += data);
            if (++state.payload_counter == state.payload_length)
                state.step++;
            break;
        case 6:
            state.step++;
            if (state.ck_a != data) {
                state.step = 0;
                goto reset;
            }
            break;
        case 7:
            state.step = 0;
            if (state.ck_b == data) {
                return true;
            } else {
                goto reset;
            }
    }
    return false;
}
#endif
