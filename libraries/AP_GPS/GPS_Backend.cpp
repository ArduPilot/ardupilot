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

#include "AP_GPS.h"
#include "GPS_Backend.h"
#include <AP_Logger/AP_Logger.h>

#define GPS_BACKEND_DEBUGGING 0

#if GPS_BACKEND_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_GPS_Backend::AP_GPS_Backend(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    port(_port),
    gps(_gps),
    state(_state)
{
    state.have_speed_accuracy = false;
    state.have_horizontal_accuracy = false;
    state.have_vertical_accuracy = false;
}

int32_t AP_GPS_Backend::swap_int32(int32_t v) const
{
    const uint8_t *b = (const uint8_t *)&v;
    union {
        int32_t v;
        uint8_t b[4];
    } u;

    u.b[0] = b[3];
    u.b[1] = b[2];
    u.b[2] = b[1];
    u.b[3] = b[0];

    return u.v;
}

int16_t AP_GPS_Backend::swap_int16(int16_t v) const
{
    const uint8_t *b = (const uint8_t *)&v;
    union {
        int16_t v;
        uint8_t b[2];
    } u;

    u.b[0] = b[1];
    u.b[1] = b[0];

    return u.v;
}


/**
   fill in time_week_ms and time_week from BCD date and time components
   assumes MTK19 millisecond form of bcd_time
 */
void AP_GPS_Backend::make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds)
{
    uint8_t year, mon, day, hour, min, sec;
    uint16_t msec;

    year = bcd_date % 100;
    mon  = (bcd_date / 100) % 100;
    day  = bcd_date / 10000;

    uint32_t v = bcd_milliseconds;
    msec = v % 1000; v /= 1000;
    sec  = v % 100; v /= 100;
    min  = v % 100; v /= 100;
    hour = v % 100;

    int8_t rmon = mon - 2;
    if (0 >= rmon) {    
        rmon += 12;
        year -= 1;
    }

    // get time in seconds since unix epoch
    uint32_t ret = (year/4) - (GPS_LEAPSECONDS_MILLIS / 1000UL) + 367*rmon/12 + day;
    ret += year*365 + 10501;
    ret = ret*24 + hour;
    ret = ret*60 + min;
    ret = ret*60 + sec;

    // convert to time since GPS epoch
    ret -= 272764785UL;

    // get GPS week and time
    state.time_week = ret / AP_SEC_PER_WEEK;
    state.time_week_ms = (ret % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC;
    state.time_week_ms += msec;
}

/*
  fill in 3D velocity for a GPS that doesn't give vertical velocity numbers
 */
void AP_GPS_Backend::fill_3d_velocity(void)
{
    float gps_heading = radians(state.ground_course);

    state.velocity.x = state.ground_speed * cosf(gps_heading);
    state.velocity.y = state.ground_speed * sinf(gps_heading);
    state.velocity.z = 0;
    state.have_vertical_velocity = false;
}

void
AP_GPS_Backend::inject_data(const uint8_t *data, uint16_t len)
{
    // not all backends have valid ports
    if (port != nullptr) {
        if (port->txspace() > len) {
            port->write(data, len);
        } else {
            Debug("GPS %d: Not enough TXSPACE", state.instance + 1);
        }
    }
}

void AP_GPS_Backend::_detection_message(char *buffer, const uint8_t buflen) const
{
    const uint8_t instance = state.instance;
    const struct AP_GPS::detect_state dstate = gps.detect_state[instance];

    if (dstate.auto_detected_baud) {
        hal.util->snprintf(buffer, buflen,
                 "GPS %d: detected as %s at %d baud",
                 instance + 1,
                 name(),
                 gps._baudrates[dstate.current_baud]);
    } else {
        hal.util->snprintf(buffer, buflen,
                 "GPS %d: specified as %s",
                 instance + 1,
                 name());
    }
}


void AP_GPS_Backend::broadcast_gps_type() const
{
#ifndef HAL_NO_GCS
    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    _detection_message(buffer, sizeof(buffer));
    gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);
#endif
}

void AP_GPS_Backend::Write_AP_Logger_Log_Startup_messages() const
{
#ifndef HAL_NO_LOGGING
    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    _detection_message(buffer, sizeof(buffer));
    AP::logger().Write_Message(buffer);
#endif
}

bool AP_GPS_Backend::should_log() const
{
    return gps.should_log();
}


void AP_GPS_Backend::send_mavlink_gps_rtk(mavlink_channel_t chan)
{
#ifndef HAL_NO_GCS
    const uint8_t instance = state.instance;
    // send status
    switch (instance) {
        case 0:
            mavlink_msg_gps_rtk_send(chan,
                                 0,  // Not implemented yet
                                 0,  // Not implemented yet
                                 state.rtk_week_number,
                                 state.rtk_time_week_ms,
                                 0,  // Not implemented yet
                                 0,  // Not implemented yet
                                 state.rtk_num_sats,
                                 state.rtk_baseline_coords_type,
                                 state.rtk_baseline_x_mm,
                                 state.rtk_baseline_y_mm,
                                 state.rtk_baseline_z_mm,
                                 state.rtk_accuracy,
                                 state.rtk_iar_num_hypotheses);
            break;
        case 1:
            mavlink_msg_gps2_rtk_send(chan,
                                 0,  // Not implemented yet
                                 0,  // Not implemented yet
                                 state.rtk_week_number,
                                 state.rtk_time_week_ms,
                                 0,  // Not implemented yet
                                 0,  // Not implemented yet
                                 state.rtk_num_sats,
                                 state.rtk_baseline_coords_type,
                                 state.rtk_baseline_x_mm,
                                 state.rtk_baseline_y_mm,
                                 state.rtk_baseline_z_mm,
                                 state.rtk_accuracy,
                                 state.rtk_iar_num_hypotheses);
            break;
    }
#endif
}


/*
  set a timestamp based on arrival time on uart at current byte,
  assuming the message started nbytes ago
*/
void AP_GPS_Backend::set_uart_timestamp(uint16_t nbytes)
{
    if (port) {
        state.uart_timestamp_ms = port->receive_time_constraint_us(nbytes) / 1000U;
    }
}


void AP_GPS_Backend::check_new_itow(uint32_t itow, uint32_t msg_length)
{
    if (itow != _last_itow) {
        _last_itow = itow;

        /*
          we need to calculate a pseudo-itow, which copes with the
          iTow from the GPS changing in unexpected ways. We assume
          that timestamps from the GPS are always in multiples of
          50ms. That means we can't handle a GPS with an update rate
          of more than 20Hz. We could do more, but we'd need the GPS
          poll time to be higher
         */
        const uint32_t gps_min_period_ms = 50;

        // get the time the packet arrived on the UART
        uint64_t uart_us = port->receive_time_constraint_us(msg_length);

        uint32_t now = AP_HAL::millis();
        uint32_t dt_ms = now - _last_ms;
        _last_ms = now;

        // round to nearest 50ms period
        dt_ms = ((dt_ms + (gps_min_period_ms/2)) / gps_min_period_ms) * gps_min_period_ms;

        // work out an actual message rate. If we get 5 messages in a
        // row with a new rate we switch rate
        if (_last_rate_ms == dt_ms) {
            if (_rate_counter < 5) {
                _rate_counter++;
            } else if (_rate_ms != dt_ms) {
                _rate_ms = dt_ms;
            }
        } else {
            _rate_counter = 0;
            _last_rate_ms = dt_ms;
        }
        if (_rate_ms == 0) {
            // only allow 5Hz to 20Hz in user config
            _rate_ms = constrain_int16(gps.get_rate_ms(state.instance), 50, 200);
        }

        // round to calculated message rate
        dt_ms = ((dt_ms + (_rate_ms/2)) / _rate_ms) * _rate_ms;

        // calculate pseudo-itow
        _pseudo_itow += dt_ms * 1000U;

        // use msg arrival time, and correct for jitter
        uint64_t local_us = jitter_correction.correct_offboard_timestamp_usec(_pseudo_itow, uart_us);
        state.uart_timestamp_ms = local_us / 1000U;
    }
}
