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

#include "AP_GPS_config.h"

#if AP_GPS_ENABLED

#include "AP_GPS.h"
#include "GPS_Backend.h"
#include <AP_Logger/AP_Logger.h>
#include <time.h>
#include <AP_Common/time.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_AHRS/AP_AHRS.h>

#define GPS_BACKEND_DEBUGGING 0

#if GPS_BACKEND_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

#include <GCS_MAVLink/GCS.h>

#if AP_GPS_DEBUG_LOGGING_ENABLED
#include <AP_Filesystem/AP_Filesystem.h>
#endif

extern const AP_HAL::HAL& hal;

AP_GPS_Backend::AP_GPS_Backend(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    port(_port),
    gps(_gps),
    state(_state),
    params(_params)
{
    state.have_speed_accuracy = false;
    state.have_horizontal_accuracy = false;
    state.have_vertical_accuracy = false;
}

/**
   fill in time_week_ms and time_week from BCD date and time components
   assumes MTK19 millisecond form of bcd_time
 */
void AP_GPS_Backend::make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds)
{
    struct tm tm {};

    tm.tm_year = 100U + bcd_date % 100U;
    tm.tm_mon  = ((bcd_date / 100U) % 100U)-1;
    tm.tm_mday = bcd_date / 10000U;

    uint32_t v = bcd_milliseconds;
    uint16_t msec = v % 1000U; v /= 1000U;
    tm.tm_sec = v % 100U; v /= 100U;
    tm.tm_min = v % 100U; v /= 100U;
    tm.tm_hour = v % 100U;

    // convert from time structure to unix time
    time_t unix_time = ap_mktime(&tm);

    // convert to time since GPS epoch
    const uint32_t unix_to_GPS_secs = 315964800UL;
    const uint16_t leap_seconds_unix = GPS_LEAPSECONDS_MILLIS/1000U;
    uint32_t ret = unix_time + leap_seconds_unix - unix_to_GPS_secs;

    // get GPS week and time
    state.time_week = ret / AP_SEC_PER_WEEK;
    state.time_week_ms = (ret % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC;
    state.time_week_ms += msec;
}

/*
  get the last time of week in ms
 */
uint32_t AP_GPS_Backend::get_last_itow_ms(void) const
{
    if (!_have_itow) {
        return state.time_week_ms;
    }
    return (_pseudo_itow_delta_ms == 0)?(_last_itow_ms):((_pseudo_itow/1000ULL) + _pseudo_itow_delta_ms);
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

/*
  fill in 3D velocity for a GPS that doesn't give vertical velocity numbers
 */
void AP_GPS_Backend::velocity_to_speed_course(AP_GPS::GPS_State &s)
{
    s.ground_course = wrap_360(degrees(atan2f(s.velocity.y, s.velocity.x)));
    s.ground_speed = s.velocity.xy().length();
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
                 "GPS %d: probing for %s at %d baud",
                 instance + 1,
                 name(),
                 int(dstate.probe_baud));
    } else {
        hal.util->snprintf(buffer, buflen,
                 "GPS %d: specified as %s",
                 instance + 1,
                 name());
    }
}


void AP_GPS_Backend::broadcast_gps_type() const
{
    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    _detection_message(buffer, sizeof(buffer));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", buffer);
}

#if HAL_LOGGING_ENABLED
void AP_GPS_Backend::Write_AP_Logger_Log_Startup_messages() const
{
    char buffer[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    _detection_message(buffer, sizeof(buffer));
    AP::logger().Write_Message(buffer);
}

bool AP_GPS_Backend::should_log() const
{
    return gps.should_log();
}
#endif


#if HAL_GCS_ENABLED
void AP_GPS_Backend::send_mavlink_gps_rtk(mavlink_channel_t chan)
{
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
}
#endif


/*
  set a timestamp based on arrival time on uart at current byte,
  assuming the message started nbytes ago
*/
void AP_GPS_Backend::set_uart_timestamp(uint16_t nbytes)
{
    if (port) {
        state.last_corrected_gps_time_us = port->receive_time_constraint_us(nbytes);
        state.corrected_timestamp_updated = true;
    }
}


void AP_GPS_Backend::check_new_itow(uint32_t itow, uint32_t msg_length)
{
    if (itow != _last_itow_ms) {
        _last_itow_ms = itow;
        _have_itow = true;

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
        uint64_t uart_us;
        if (_last_pps_time_us != 0 && (state.status >= AP_GPS::GPS_OK_FIX_2D)) {
            // pps is only reliable when we have some sort of GPS fix
            uart_us = _last_pps_time_us;
            _last_pps_time_us = 0;
        } else if (port) {
            uart_us = port->receive_time_constraint_us(msg_length);
        } else {
            uart_us = AP_HAL::micros64();
        }

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
            if (_rate_ms != 0) {
                set_pps_desired_freq(1000/_rate_ms);
            }
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
        state.last_corrected_gps_time_us = local_us;
        state.corrected_timestamp_updated = true;

#ifndef HAL_BUILD_AP_PERIPH
        // look for lagged data from the GPS. This is meant to detect
        // the case that the GPS is trying to push more data into the
        // UART than can fit (eg. with GPS_RAW_DATA at 115200).
        // This is disabled on AP_Periph as it is better to catch missed packet rate at the flight
        // controller level
        float expected_lag;
        if (gps.get_lag(state.instance, expected_lag)) {
            float lag_s = (now - (state.last_corrected_gps_time_us/1000U)) * 0.001;
            if (lag_s > expected_lag+0.05) {
                // more than 50ms over expected lag, increment lag counter
                state.lagged_sample_count++;
            } else {
                state.lagged_sample_count = 0;
            }
        }
#endif // HAL_BUILD_AP_PERIPH

        if (state.status >= AP_GPS::GPS_OK_FIX_2D) {
            // we must have a decent fix to calculate difference between itow and pseudo-itow
            _pseudo_itow_delta_ms = itow - (_pseudo_itow/1000ULL);
        }
    }
}

#if GPS_MOVING_BASELINE
bool AP_GPS_Backend::calculate_moving_base_yaw(float reported_heading_deg, const float reported_distance, const float reported_D) {
    return calculate_moving_base_yaw(state, reported_heading_deg, reported_distance, reported_D);
}

bool AP_GPS_Backend::calculate_moving_base_yaw(AP_GPS::GPS_State &interim_state, const float reported_heading_deg, const float reported_distance, const float reported_D) {
    constexpr float minimum_antenna_seperation = 0.05; // meters
    constexpr float permitted_error_length_pct = 0.2;  // percentage
#if HAL_LOGGING_ENABLED || AP_AHRS_ENABLED
    float min_D = 0.0f;
    float max_D = 0.0f;
#endif
    bool selectedOffset = false;
    Vector3f offset;
    switch (MovingBase::Type(gps.params[interim_state.instance].mb_params.type)) {
        case MovingBase::Type::RelativeToAlternateInstance:
            offset = gps.params[interim_state.instance^1].antenna_offset.get() - gps.params[interim_state.instance].antenna_offset.get();
            selectedOffset = true;
            break;
        case MovingBase::Type::RelativeToCustomBase:
            offset = gps.params[interim_state.instance].mb_params.base_offset.get();
            selectedOffset = true;
            break;
    }

    if (!selectedOffset) {
        // invalid type, let's throw up a flag
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        goto bad_yaw;
    }

    {
        const float offset_dist = offset.length();
        const float min_dist = MIN(offset_dist, reported_distance);

        if (offset_dist < minimum_antenna_seperation) {
            // offsets have to be sufficiently large to get a meaningful angle off of them
            Debug("Insufficent antenna offset (%f, %f, %f)", (double)offset.x, (double)offset.y, (double)offset.z);
            goto bad_yaw;
        }

        if (reported_distance < minimum_antenna_seperation) {
            // if the reported distance is less then the minimum separation it's not sufficiently robust
            Debug("Reported baseline distance (%f) was less then the minimum antenna separation (%f)",
                  (double)reported_distance, (double)minimum_antenna_seperation);
            goto bad_yaw;
        }


        if (fabsf(offset_dist - reported_distance) > (min_dist * permitted_error_length_pct)) {
            // the magnitude of the vector is much further then we were expecting
            Debug("Offset=%.2f vs reported-distance=%.2f (max-delta=%.2f)",
                  offset_dist, reported_distance, (double)(min_dist * permitted_error_length_pct));
            goto bad_yaw;
        }

#if AP_AHRS_ENABLED
        {
            // get vehicle rotation, projected back in time using the gyro
            // this is not 100% accurate, but it is good enough for
            // this test. To do it completely accurately we'd need an
            // interface into DCM, EKF2 and EKF3 to ask for a
            // historical attitude. That is far too complex to justify
            // for this use case
            const auto &ahrs = AP::ahrs();
            const Vector3f &gyro = ahrs.get_gyro();
            Matrix3f rot_body_to_ned_min_lag = ahrs.get_rotation_body_to_ned();
            rot_body_to_ned_min_lag.rotate(gyro * -AP_GPS_MB_MIN_LAG);
            Matrix3f rot_body_to_ned_max_lag = ahrs.get_rotation_body_to_ned();
            rot_body_to_ned_max_lag.rotate(gyro * -AP_GPS_MB_MAX_LAG);

            // apply rotation to the offset to get the Z offset in NED
            const Vector3f antenna_tilt_min_lag = rot_body_to_ned_min_lag * offset;
            const Vector3f antenna_tilt_max_lag = rot_body_to_ned_max_lag * offset;
            min_D = MIN(-antenna_tilt_min_lag.z, -antenna_tilt_max_lag.z);
            max_D = MAX(-antenna_tilt_min_lag.z, -antenna_tilt_max_lag.z);
            min_D -= permitted_error_length_pct * min_dist;
            max_D += permitted_error_length_pct * min_dist;
            if (reported_D < min_D || reported_D > max_D) {
                // the vertical component is out of range, reject it
                Debug("bad alt_err %f < %f < %f", (double)min_D, (double)reported_D, (double)max_D);
                goto bad_yaw;
            }
        }
#endif // AP_AHRS_ENABLED

        {
            // at this point the offsets are looking okay, go ahead and actually calculate a useful heading
            const float rotation_offset_rad = Vector2f(-offset.x, -offset.y).angle();
            interim_state.gps_yaw = wrap_360(reported_heading_deg - degrees(rotation_offset_rad));
            interim_state.have_gps_yaw = true;
            interim_state.gps_yaw_time_ms = AP_HAL::millis();
        }
        goto good_yaw;
    }

bad_yaw:
    interim_state.have_gps_yaw = false;

good_yaw:

#if HAL_LOGGING_ENABLED
    // this log message helps diagnose GPS yaw issues
    // @LoggerMessage: GPYW
    // @Description: GPS Yaw
    // @Field: TimeUS: Time since system startup
    // @Field: Id: instance
    // @Field: RHD: reported heading,deg
    // @Field: RDist: antenna separation,m
    // @Field: RDown: vertical antenna separation,m
    // @Field: MinCDown: minimum tolerable vertical antenna separation,m
    // @Field: MaxCDown: maximum tolerable vertical antenna separation,m 
    // @Field: OK: 1 if have yaw
    AP::logger().WriteStreaming("GPYW", "TimeUS,Id,RHD,RDist,RDown,MinCDown,MaxCDown,OK",
                                "s#dmmmm-",
                                "F-------",
                                "QBfffffB",
                                AP_HAL::micros64(),
                                state.instance,
                                reported_heading_deg,
                                reported_distance,
                                reported_D,
                                min_D,
                                max_D,
                                interim_state.have_gps_yaw);
#endif

    return interim_state.have_gps_yaw;
}
#endif // GPS_MOVING_BASELINE

/*
  set altitude in location structure, honouring the driver option for
  MSL vs ellipsoid height
 */
void AP_GPS_Backend::set_alt_amsl_cm(AP_GPS::GPS_State &_state, int32_t alt_amsl_cm)
{
    if (option_set(AP_GPS::HeightEllipsoid) && _state.have_undulation) {
        // user has asked ArduPilot to use ellipsoid height in the
        // canonical height for mission and navigation
        _state.location.alt = alt_amsl_cm - _state.undulation*100;
    } else {
        _state.location.alt = alt_amsl_cm;
    }
}

#if AP_GPS_DEBUG_LOGGING_ENABLED

/*
  log some data for debugging

  the logging format matches that used by SITL with SIM_GPS_TYPE=7,
  allowing for development of GPS drivers based on logged data
*/
void AP_GPS_Backend::log_data(const uint8_t *data, uint16_t length)
{
    if (state.instance < 2) {
        logging[state.instance].buf.write(data, length);
    }
    if (!log_thread_created) {
        log_thread_created = true;
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_GPS_Backend::logging_start, void), "gps_log", 4096, AP_HAL::Scheduler::PRIORITY_IO, 0);
    }
}

AP_GPS_Backend::loginfo AP_GPS_Backend::logging[2];
bool AP_GPS_Backend::log_thread_created;

// logging loop, needs to be static to allow for re-alloc of GPS backends
void AP_GPS_Backend::logging_loop(void)
{
    while (true) {
        hal.scheduler->delay(10);
        static uint16_t lognum;
        for (uint8_t instance=0; instance<2; instance++) {
            if (logging[instance].fd == -1 && logging[instance].buf.available()) {
                char fname[] = "gpsN_XXX.log";
                fname[3] = '1' + instance;
                if (lognum == 0) {
                    for (lognum=1; lognum<1000; lognum++) {
                        struct stat st;
                        hal.util->snprintf(&fname[5], 8, "%03u.log", lognum);
                        if (AP::FS().stat(fname, &st) != 0) {
                            break;
                        }
                    }
                }
                hal.util->snprintf(&fname[5], 8, "%03u.log", lognum);
                logging[instance].fd = AP::FS().open(fname, O_WRONLY|O_CREAT|O_APPEND);
            }
            if (logging[instance].fd != -1) {
                uint32_t n = 0;
                const uint8_t *p;
                while ((p = logging[instance].buf.readptr(n)) != nullptr && n != 0) {
                    struct {
                        uint32_t magic = 0x7fe53b04U;
                        uint32_t time_ms;
                        uint32_t n;
                    } header;
                    header.n = n;
                    header.time_ms = AP_HAL::millis();
                    // short writes are unlikely and are ignored (only FS full errors)
                    AP::FS().write(logging[instance].fd, (const uint8_t *)&header, sizeof(header));
                    AP::FS().write(logging[instance].fd, p, n);
                    logging[instance].buf.advance(n);
                    AP::FS().fsync(logging[instance].fd);
                }
            }
        }
    }
}

// logging thread start, needs to be non-static for thread_create
void AP_GPS_Backend::logging_start(void)
{
    logging_loop();
}
#endif // AP_GPS_DEBUG_LOGGING_ENABLED

#endif  // AP_GPS_ENABLED
