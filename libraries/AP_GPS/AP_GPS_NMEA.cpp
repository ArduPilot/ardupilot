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
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//

/// @file	AP_GPS_NMEA.cpp
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.
///

#include <AP_Common/AP_Common.h>
#include <AP_Common/NMEA.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "AP_GPS_NMEA.h"

#if AP_GPS_NMEA_ENABLED
extern const AP_HAL::HAL& hal;

#ifndef AP_GPS_NMEA_CONFIG_PERIOD_MS
// how often we send board specific config commands
#define AP_GPS_NMEA_CONFIG_PERIOD_MS 15000U
#endif

// a quiet nan for invalid values
#define QNAN nanf("GPS")

#if AP_GPS_NMEA_TESEO_ENABLED
#define TESEO_REPLY_TIMEOUT_MS 500U
#define TESEO_SAVE_TIMEOUT_MS 1000U
#define TESEO_MAX_RETRIES 3U
#define TESEO_MAX_CONFIG_ATTEMPTS 3U

uint8_t AP_GPS_NMEA::_teseo_config_attempts[GPS_MAX_RECEIVERS];
#endif // AP_GPS_NMEA_TESEO_ENABLED

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)        (_x - '0')
#define hexdigit(x) ((x)>9?'A'+((x)-10):'0'+(x))

bool AP_GPS_NMEA::read(void)
{
    int16_t numc;
    bool parsed = false;

    send_config();

    numc = port->available();
    while (numc--) {
        char c = port->read();
#if AP_GPS_DEBUG_LOGGING_ENABLED
        log_data((const uint8_t *)&c, 1);
#endif
        if (_decode(c)) {
            parsed = true;
        }
    }
    return parsed;
}

/*
  decode one character, return true if we have successfully completed a sentence, false otherwise
 */
bool AP_GPS_NMEA::_decode(char c)
{
    _sentence_length++;
        
    switch (c) {
    case ';':
        // header separator for unicore
        if (!_is_unicore) {
            return false;
        }
        FALLTHROUGH;
    case ',': // term terminators
        _parity ^= c;
        if (_is_unicore) {
            _crc32 = crc_crc32(_crc32, (const uint8_t *)&c, 1);
        }
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*': {
        if (_sentence_done) {
            return false;
        }
        bool valid_sentence = false;
        if (_term_offset < sizeof(_term)) {
            _term[_term_offset] = 0;
            valid_sentence = _term_complete();
        }
        ++_term_number;
        _term_offset = 0;
        _is_checksum_term = c == '*';
        return valid_sentence;
    }

    case '$': // sentence begin
    case '#': // unicore message begin
        _is_unicore = (c == '#');
        _term_number = _term_offset = 0;
        _parity = 0;
        _crc32 = 0;
        _sentence_type = _GPS_SENTENCE_OTHER;
        _is_checksum_term = false;
        _sentence_length = 1;
        _sentence_done = false;
        _new_gps_yaw = QNAN;
        return false;
    }

    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
    if (!_is_checksum_term) {
        _parity ^= c;
        if (_is_unicore) {
            _crc32 = crc_crc32(_crc32, (const uint8_t *)&c, 1);
        }
    }

    return false;
}

int32_t AP_GPS_NMEA::_parse_decimal_100(const char *p)
{
    char *endptr = nullptr;
    long ret = 100 * strtol(p, &endptr, 10);
    int sign = ret < 0 ? -1 : 1;

    if (ret >= (long)INT32_MAX) {
        return INT32_MAX;
    }
    if (ret <= (long)INT32_MIN) {
        return INT32_MIN;
    }
    if (endptr == nullptr || *endptr != '.') {
        return ret;
    }

    if (isdigit(endptr[1])) {
        ret += sign * 10 * DIGIT_TO_VAL(endptr[1]);
        if (isdigit(endptr[2])) {
            ret += sign * DIGIT_TO_VAL(endptr[2]);
            if (isdigit(endptr[3])) {
                ret += sign * (DIGIT_TO_VAL(endptr[3]) >= 5);
            }
        }
    }
    return ret;
}

/*
  parse a NMEA latitude/longitude degree value. The result is in degrees*1e7
 */
uint32_t AP_GPS_NMEA::_parse_degrees()
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    float frac_min = 0;
    int32_t ret = 0;

    // scan for decimal point or end of field
    for (p = _term; *p && isdigit(*p); p++)
        ;
    q = _term;

    // convert degrees
    while ((p - q) > 2 && *q) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }

    // convert minutes
    while (p > q && *q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }

    // convert fractional minutes
    if (*p == '.') {
        q = p + 1;
        float frac_scale = 0.1f;
        while (*q && isdigit(*q)) {
            frac_min += DIGIT_TO_VAL(*q) * frac_scale;
            q++;
            frac_scale *= 0.1f;
        }
    }
    ret = (deg * (int32_t)10000000UL);
    ret += (min * (int32_t)10000000UL / 60);
    ret += (int32_t) (frac_min * (1.0e7f / 60.0f));
    return ret;
}

/*
  see if we have a new set of NMEA messages
 */
bool AP_GPS_NMEA::_have_new_message()
{
    if (_last_RMC_ms == 0 ||
        _last_GGA_ms == 0) {
        return false;
    }
    uint32_t now = AP_HAL::millis();
    if (now - _last_RMC_ms > 150 ||
        now - _last_GGA_ms > 150) {
        return false;
    }
    if (_last_VTG_ms != 0 && 
        now - _last_VTG_ms > 150) {
        return false;
    }

    /*
      if we have seen a message with 3D velocity data messages then
      wait for them again. This is important as the
      have_vertical_velocity field will be overwritten by
      fill_3d_velocity()
     */
    if (_last_vvelocity_ms != 0 &&
        now - _last_vvelocity_ms > 150 &&
        now - _last_vvelocity_ms < 1000) {
        // waiting on a message with velocity
        return false;
    }
    if (_last_vaccuracy_ms != 0 &&
        now - _last_vaccuracy_ms > 150 &&
        now - _last_vaccuracy_ms < 1000) {
        // waiting on a message with velocity accuracy
        return false;
    }

    // prevent these messages being used again
    if (_last_VTG_ms != 0) {
        _last_VTG_ms = 1;
    }

    if (now - _last_yaw_ms > 300) {
        // we have lost GPS yaw
        state.have_gps_yaw = false;
    }

    if (now - _last_KSXT_pos_ms > 500) {
        // we have lost KSXT
        _last_KSXT_pos_ms = 0;
    }

#if AP_GPS_NMEA_UNICORE_ENABLED
    if (now - _last_AGRICA_ms > 500) {
        if (_last_AGRICA_ms != 0) {
            // we have lost AGRICA
            state.have_gps_yaw = false;
#if AP_GPS_MB_YAW_OFFSET_ENABLED
            // the moving baseline offset must not outlive the yaw solution it
            // was calculated for
            state.mb_yaw_offset.zero();
#endif
            state.have_vertical_velocity = false;
            state.have_speed_accuracy = false;
            state.have_horizontal_accuracy = false;
            state.have_vertical_accuracy = false;
            state.have_undulation = false;
            _last_AGRICA_ms = 0;
        }
    }
#endif // AP_GPS_NMEA_UNICORE_ENABLED

    _last_fix_ms = now;

    _last_GGA_ms = 1;
    _last_RMC_ms = 1;
    return true;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool AP_GPS_NMEA::_term_complete()
{
    // handle the last term in a message
    if (_is_checksum_term) {
        _sentence_done = true;
        const uint32_t crc = strtoul(_term, nullptr, 16);
        const bool crc_ok = _is_unicore? (_crc32 == crc) : (_parity == crc);
        if (crc_ok) {
            uint32_t now = AP_HAL::millis();
            switch (_sentence_type) {
            case _GPS_SENTENCE_RMC:
                _last_RMC_ms = now;
                //time                        = _new_time;
                //date                        = _new_date;
                if (_last_KSXT_pos_ms == 0 && _last_AGRICA_ms == 0) {
                    state.location.lat     = _new_latitude;
                    state.location.lng     = _new_longitude;
                }
                if (_last_3D_velocity_ms == 0 ||
                    now - _last_3D_velocity_ms > 1000) {
                    state.ground_speed     = _new_speed*0.01f;
                    state.ground_course    = wrap_360(_new_course*0.01f);
                }
                if (state.status >= AP_GPS_FixType::FIX_3D) {
                    make_gps_time(_new_date, _new_time * 10);
                    if (_last_AGRICA_ms != 0) {
                        state.time_week_ms = _last_itow_ms;
                    }
                }
                set_uart_timestamp(_sentence_length);
                state.last_gps_time_ms = now;
                if (_last_vvelocity_ms == 0 ||
                    now - _last_vvelocity_ms > 1000) {
                    fill_3d_velocity();
                }
                break;
            case _GPS_SENTENCE_GGA:
                _last_GGA_ms = now;
                if (_last_KSXT_pos_ms == 0 && _last_AGRICA_ms == 0) {
                    set_alt_amsl_cm(state, _new_altitude);
                    state.location.lat  = _new_latitude;
                    state.location.lng  = _new_longitude;
                }
                state.num_sats      = _new_satellite_count;
                state.hdop          = _new_hdop;
                switch(_new_quality_indicator) {
                case 0: // Fix not available or invalid
                    state.status = AP_GPS_FixType::NONE;
                    break;
                case 1: // GPS SPS Mode, fix valid
                    state.status = AP_GPS_FixType::FIX_3D;
                    break;
                case 2: // Differential GPS, SPS Mode, fix valid
                    state.status = AP_GPS_FixType::DGPS;
                    break;
                case 3: // GPS PPS Mode, fix valid
                    state.status = AP_GPS_FixType::FIX_3D;
                    break;
                case 4: // Real Time Kinematic. System used in RTK mode with fixed integers
                    state.status = AP_GPS_FixType::RTK_FIXED;
                    break;
                case 5: // Float RTK. Satellite system used in RTK mode, floating integers
                    state.status = AP_GPS_FixType::RTK_FLOAT;
                    break;
                case 6: // Estimated (dead reckoning) Mode
                    state.status = AP_GPS_FixType::NONE;
                    break;
                default://to maintain compatibility with MAV_GPS_INPUT and others
                    state.status = AP_GPS_FixType::FIX_3D;
                    break;
                }
                break;
            case _GPS_SENTENCE_VTG:
                _last_VTG_ms = now;
                if (_last_3D_velocity_ms == 0 ||
                    now - _last_3D_velocity_ms > 1000) {
                    state.ground_speed  = _new_speed*0.01f;
                    state.ground_course = wrap_360(_new_course*0.01f);
                    if (_last_vvelocity_ms == 0 ||
                        now - _last_vvelocity_ms > 1000) {
                        fill_3d_velocity();
                    }
                }
                // VTG has no fix indicator, can't change fix status
                break;
            case _GPS_SENTENCE_HDT:
            case _GPS_SENTENCE_THS:
                if (_last_AGRICA_ms != 0 || _expect_agrica) {
                    // use AGRICA
                    break;
                }
                if (isnan(_new_gps_yaw)) {
                    // empty sentence
                    break;
                }
                _last_yaw_ms = now;
                state.gps_yaw = wrap_360(_new_gps_yaw*0.01f);
                state.have_gps_yaw = true;
                state.gps_yaw_time_ms = now;
#if AP_GPS_MB_YAW_OFFSET_ENABLED
                // this yaw is the device-reported heading, not one calculated
                // from a moving baseline offset; clear any stale offset so no
                // attitude correction is applied to it
                state.mb_yaw_offset.zero();
#endif
                // remember that we are setup to provide yaw. With
                // a NMEA GPS we can only tell if the GPS is
                // configured to provide yaw when it first sends a
                // HDT sentence.
                state.gps_yaw_configured = true;
                break;
            case _GPS_SENTENCE_PHD:
                if (_last_AGRICA_ms != 0) {
                    // prefer AGRICA
                    break;
                }
                if (_phd.msg_id == 12) {
                    state.velocity.x = _phd.fields[0] * 0.01;
                    state.velocity.y = _phd.fields[1] * 0.01;
                    state.velocity.z = _phd.fields[2] * 0.01;
                    state.have_vertical_velocity = true;
                    _last_vvelocity_ms = now;
                    // we prefer a true 3D velocity when available
                    velocity_to_speed_course(state);
                    _last_3D_velocity_ms = now;
                } else if (_phd.msg_id == 26) {
                    state.horizontal_accuracy = MAX(_phd.fields[0],_phd.fields[1]) * 0.001;
                    state.have_horizontal_accuracy = true;
                    state.vertical_accuracy = _phd.fields[2] * 0.001;
                    state.have_vertical_accuracy = true;
                    state.speed_accuracy = MAX(_phd.fields[3],_phd.fields[4]) * 0.001;
                    state.have_speed_accuracy = true;
                    _last_vaccuracy_ms = now;
                }
                break;
            case _GPS_SENTENCE_KSXT:
                if (_last_AGRICA_ms != 0 || _expect_agrica) {
                    // prefer AGRICA
                    break;
                }
                state.location.lat     = _ksxt.fields[2]*1.0e7;
                state.location.lng     = _ksxt.fields[1]*1.0e7;
                set_alt_amsl_cm(state, _ksxt.fields[3]*1.0e2);
                _last_KSXT_pos_ms = now;
                if (_ksxt.fields[9] >= 1) {
                    // we have 3D fix
                    constexpr float kmh_to_mps = 1.0 / 3.6;
                    state.velocity.y = _ksxt.fields[16] * kmh_to_mps;
                    state.velocity.x = _ksxt.fields[17] * kmh_to_mps;
                    state.velocity.z = _ksxt.fields[18] * -kmh_to_mps;
                    state.have_vertical_velocity = true;
                    _last_vvelocity_ms = now;
                    // we prefer a true 3D velocity when available
                    velocity_to_speed_course(state);
                    _last_3D_velocity_ms = now;
                }
                if (is_equal(3.0f, float(_ksxt.fields[10]))) {
                    // have good yaw (from RTK fixed moving baseline solution)
                    _last_yaw_ms = now;
                    state.gps_yaw = _ksxt.fields[4];
                    state.have_gps_yaw = true;
                    state.gps_yaw_time_ms = now;
                    state.gps_yaw_configured = true;
#if AP_GPS_MB_YAW_OFFSET_ENABLED
                    // this yaw is the receiver's own heading, not one calculated
                    // from the configured moving baseline offset; clear any stale
                    // offset so no attitude correction is applied to it
                    state.mb_yaw_offset.zero();
#endif
                }
                break;
#if AP_GPS_NMEA_UNICORE_ENABLED
            case _GPS_SENTENCE_AGRICA: {
                const auto &ag = _agrica;
                _last_AGRICA_ms = now;
                _last_vvelocity_ms = now;
                _last_vaccuracy_ms = now;
                _last_3D_velocity_ms = now;
                state.location.lat = ag.lat*1.0e7;
                state.location.lng = ag.lng*1.0e7;
                state.undulation   = -ag.undulation;
                state.have_undulation = true;
                set_alt_amsl_cm(state, ag.alt*1.0e2);
                state.velocity = ag.vel_NED;
                velocity_to_speed_course(state);
                state.speed_accuracy = ag.vel_stddev.length();
                state.horizontal_accuracy = ag.pos_stddev.xy().length();
                state.vertical_accuracy = ag.pos_stddev.z;
                state.have_vertical_velocity = true;
                state.have_speed_accuracy = true;
                state.have_horizontal_accuracy = true;
                state.have_vertical_accuracy = true;
                check_new_itow(ag.itow, _sentence_length);
                break;
            }
            case _GPS_SENTENCE_VERSIONA: {
                _have_unicore_versiona = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "NMEA %s %s %s",
                              _versiona.type,
                              _versiona.version,
                              _versiona.build_date);
                break;
            }
            case _GPS_SENTENCE_UNIHEADINGA: {
#if GPS_MOVING_BASELINE
                const auto &ag = _agrica;
                const auto &uh = _uniheadinga;
                if (now - _last_AGRICA_ms > 500 || ag.heading_status != 4) {
                    // we need heading_status from AGRICA
                    state.have_gps_yaw = false;
                    break;
                }
                const float dist = uh.baseline_length;
                const float bearing = uh.heading;
                const float alt_diff = dist*sinf(radians(-uh.pitch));
                state.relPosHeading = bearing;
                state.relPosLength = dist;
                state.relPosD = alt_diff;
                state.relposheading_ts = now;
                if (calculate_moving_base_yaw(bearing, dist, alt_diff)) {
                    state.have_gps_yaw_accuracy = true;
                    state.gps_yaw_accuracy = uh.heading_sd;
                    _last_yaw_ms = now;
                }
                state.gps_yaw_configured = true;
#endif // GPS_MOVING_BASELINE
                break;
            }
#endif // AP_GPS_NMEA_UNICORE_ENABLED
#if AP_GPS_NMEA_TESEO_ENABLED
            case _GPS_SENTENCE_PSTMKFCOV:
            case _GPS_SENTENCE_PSTMPVRAW:
            case _GPS_SENTENCE_PSTMVER:
            case _GPS_SENTENCE_PSTMSETPAROK:
            case _GPS_SENTENCE_PSTMSETPARERROR:
            case _GPS_SENTENCE_PSTMSAVEPAROK:
            case _GPS_SENTENCE_PSTMSETPAR:
            case _GPS_SENTENCE_PSTMGETPARERROR:
                teseo_sentence_complete(now);
                break;
#endif // AP_GPS_NMEA_TESEO_ENABLED
            }
            // see if we got a good message
            return _have_new_message();
        }
        // we got a bad message, ignore it
        return false;
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        /*
          special case for $PHD message
         */
        if (strcmp(_term, "PHD") == 0) {
            _sentence_type = _GPS_SENTENCE_PHD;
            return false;
        }
        if (strcmp(_term, "KSXT") == 0) {
            _sentence_type = _GPS_SENTENCE_KSXT;
            return false;
        }
#if AP_GPS_NMEA_UNICORE_ENABLED
        if (strcmp(_term, "AGRICA") == 0 && _expect_agrica) {
            _sentence_type = _GPS_SENTENCE_AGRICA;
            return false;
        }
        if (strcmp(_term, "VERSIONA") == 0) {
            _sentence_type = _GPS_SENTENCE_VERSIONA;
            return false;
        }
        if (strcmp(_term, "UNIHEADINGA") == 0 && _expect_agrica) {
            _sentence_type = _GPS_SENTENCE_UNIHEADINGA;
            return false;
        }
#endif
#if AP_GPS_NMEA_TESEO_ENABLED
        // must come before the talker ID check, which would take
        // these as talker "PS"
        if (_is_teseo && strncmp(_term, "PSTM", 4) == 0) {
            _sentence_type = teseo_sentence_type(&_term[4]);
            // empty terms aren't parsed, so start from nothing
            _pstmkfcov = {};
            _pstmpvraw = {};
            _teseo_reply = {};
            return false;
        }
#endif
        /*
          The first two letters of the NMEA term are the talker
          ID. The most common is 'GP' but there are a bunch of others
          that are valid. We accept any two characters here.
         */
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            _sentence_type = _GPS_SENTENCE_OTHER;
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "RMC") == 0) {
            _sentence_type = _GPS_SENTENCE_RMC;
        } else if (strcmp(term_type, "GGA") == 0) {
            _sentence_type = _GPS_SENTENCE_GGA;
        } else if (strcmp(term_type, "HDT") == 0) {
            _sentence_type = _GPS_SENTENCE_HDT;
        } else if (strcmp(term_type, "THS") == 0) {
            _sentence_type = _GPS_SENTENCE_THS;
        } else if (strcmp(term_type, "VTG") == 0) {
            _sentence_type = _GPS_SENTENCE_VTG;
        } else {
            _sentence_type = _GPS_SENTENCE_OTHER;
        }
        return false;
    }

    // 32 = RMC, 64 = GGA, 96 = VTG, 128 = HDT, 160 = THS
    if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
        switch (_sentence_type + _term_number) {
        // operational status
        //
        case _GPS_SENTENCE_RMC + 2: // validity (RMC)
            break;
        case _GPS_SENTENCE_GGA + 6: // Fix data (GGA)
            if (_term[0] > '0') {
                _new_quality_indicator = _term[0] - '0';
            } else {
                _new_quality_indicator = 0;
            }
            break;
        case _GPS_SENTENCE_GGA + 7: // satellite count (GGA)
            _new_satellite_count = atol(_term);
            break;
        case _GPS_SENTENCE_GGA + 8: // HDOP (GGA)
            _new_hdop = (uint16_t)_parse_decimal_100(_term);
            break;

        // time and date
        //
        case _GPS_SENTENCE_RMC + 1: // Time (RMC)
        case _GPS_SENTENCE_GGA + 1: // Time (GGA)
            _new_time = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_RMC + 9: // Date (GPRMC)
            _new_date = atol(_term);
            break;

        // location
        //
        case _GPS_SENTENCE_RMC + 3: // Latitude
        case _GPS_SENTENCE_GGA + 2:
            _new_latitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 4: // N/S
        case _GPS_SENTENCE_GGA + 3:
            if (_term[0] == 'S')
                _new_latitude = -_new_latitude;
            break;
        case _GPS_SENTENCE_RMC + 5: // Longitude
        case _GPS_SENTENCE_GGA + 4:
            _new_longitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 6: // E/W
        case _GPS_SENTENCE_GGA + 5:
            if (_term[0] == 'W')
                _new_longitude = -_new_longitude;
            break;
        case _GPS_SENTENCE_GGA + 9: // Altitude (GPGGA)
            _new_altitude = _parse_decimal_100(_term);
            break;

        // course and speed
        //
        case _GPS_SENTENCE_RMC + 7: // Speed (GPRMC)
        case _GPS_SENTENCE_VTG + 5: // Speed (VTG)
            _new_speed = (_parse_decimal_100(_term) * 514) / 1000;       // knots-> m/sec, approximates * 0.514
            break;
        case _GPS_SENTENCE_HDT + 1: // Course (HDT)
            _new_gps_yaw = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_THS + 1: // Course (THS)
            _new_gps_yaw = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_RMC + 8: // Course (GPRMC)
        case _GPS_SENTENCE_VTG + 1: // Course (VTG)
            _new_course = _parse_decimal_100(_term);
            break;

        case _GPS_SENTENCE_PHD + 1: // PHD class
            _phd.msg_class = atol(_term);
            break;
        case _GPS_SENTENCE_PHD + 2: // PHD message
            _phd.msg_id = atol(_term);
            break;
        case _GPS_SENTENCE_PHD + 5: // PHD message, itow
            _phd.itow = strtoul(_term, nullptr, 10);
            break;
        case _GPS_SENTENCE_PHD + 6 ... _GPS_SENTENCE_PHD + 11: // PHD message, fields
            if (_sentence_type == _GPS_SENTENCE_PHD) {
                _phd.fields[_term_number-6] = atol(_term);
            }
            break;
        case _GPS_SENTENCE_KSXT + 1 ... _GPS_SENTENCE_KSXT + 21: // KSXT message, fields
            if (_sentence_type == _GPS_SENTENCE_KSXT) {
                _ksxt.fields[_term_number-1] = atof(_term);
            }
            break;
#if AP_GPS_NMEA_UNICORE_ENABLED
        case _GPS_SENTENCE_AGRICA + 1 ... _GPS_SENTENCE_AGRICA + 65: // AGRICA message
            parse_agrica_field(_term_number, _term);
            break;
        case _GPS_SENTENCE_VERSIONA + 1 ... _GPS_SENTENCE_VERSIONA + 20:
            parse_versiona_field(_term_number, _term);
            break;
#if GPS_MOVING_BASELINE
        case _GPS_SENTENCE_UNIHEADINGA + 1 ... _GPS_SENTENCE_UNIHEADINGA + 28: // UNIHEADINGA message
            parse_uniheadinga_field(_term_number, _term);
            break;
#endif
#endif
#if AP_GPS_NMEA_TESEO_ENABLED
        case _GPS_SENTENCE_PSTMKFCOV + 1 ... _GPS_SENTENCE_PSTMGETPARERROR: // PSTM sentences
            parse_teseo_field(_term_number, _term);
            break;
#endif
        }
    }

    return false;
}

#if AP_GPS_NMEA_UNICORE_ENABLED
/*
  parse an AGRICA message term

  Example:
     #AGRICA,82,GPS,FINE,2237,176366400,0,0,18,15;GNSS,232,22,11,22,0,59,8,1,5,8,12,0,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,296.4656,-26.5685,0.0000,0.005,0.000,0.000,-0.005,0.044,0.032,0.038,-35.33142715815,149.13181842030,609.1494,-4471799.0368,2672944.7758,-3668288.9857,1.3923,1.5128,3.2272,2.3026,2.1633,2.1586,0.00000000000,0.00000000000,0.0000,0.00000000000,0.00000000000,0.0000,176366400,0.000,66.175285,18.972784,0.000000,0.000000,5,0,0,0*9f704dad
 */
void AP_GPS_NMEA::parse_agrica_field(uint16_t term_number, const char *term)
{
    auto &ag = _agrica;
    // subtract 8 to align term numbers with reference manual
    // look for "Unicore Reference Command Manual" to find the specification
    const uint8_t hdr_align = 8;
    if (term_number < hdr_align) {
        // discard header;
        return;
    }
    term_number -= hdr_align;
    // useful for debugging
    //::printf("AGRICA[%u]=%s\n", unsigned(term_number), term);
    switch (term_number) {
    case 10:
        ag.rtk_status = atol(term);
        break;
    case 11:
        ag.heading_status = atol(term);
        break;
    case 25 ... 26:
        ag.vel_NED[term_number-25] = strtof(term, nullptr);
        break;
    case 27:
        // AGRIC gives velocity up
        ag.vel_NED.z = -strtof(term, nullptr);
        break;
    case 28 ... 30:
        ag.vel_stddev[term_number-28] = strtof(term, nullptr);
        break;
    case 31:
        ag.lat = atof(term);
        break;
    case 32:
        ag.lng = atof(term);
        break;
    case 33:
        ag.alt = strtof(term, nullptr);
        break;
    case 49:
        ag.itow = atol(term);
        break;
    case 37 ... 39:
        ag.pos_stddev[term_number-37] = strtof(term, nullptr);
        break;
    case 52:
        ag.undulation = strtof(term, nullptr);
        break;
    }
}

#if GPS_MOVING_BASELINE
/*
  parse a UNIHEADINGA message term

  Example:
    #UNIHEADINGA,79,GPS,FINE,2242,167498200,0,0,18,22;SOL_COMPUTED,L1_INT,2.7889,296.7233,-25.7710,0.0000,0.1127,0.1812,"999",49,37,37,0,3,00,1,51*d50af0ea
 */
void AP_GPS_NMEA::parse_uniheadinga_field(uint16_t term_number, const char *term)
{
    const uint8_t hdr_align = 8;
    if (term_number < hdr_align) {
        // discard header;
        return;
    }
    term_number -= hdr_align;
    // useful for debugging
    // ::printf("UNIHEADINGA[%u]=%s\n", unsigned(term_number), term);
    auto &uh = _uniheadinga;
    switch (term_number) {
    case 4:
        uh.baseline_length = strtof(term, nullptr);
        break;
    case 5:
        uh.heading = strtof(term, nullptr);
        break;
    case 6:
        uh.pitch = strtof(term, nullptr);
        break;
    case 8:
        uh.heading_sd = strtof(term, nullptr);
        break;
    }
}
#endif // GPS_MOVING_BASELINE

// parse VERSIONA fields
void AP_GPS_NMEA::parse_versiona_field(uint16_t term_number, const char *term)
{
    // printf useful for debugging
    // ::printf("VERSIONA[%u]='%s'\n", term_number, term);
    auto &v = _versiona;
#pragma GCC diagnostic push
#if defined(__GNUC__) &&  __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif
    switch (term_number) {
    case 10:
        strncpy(v.type, _term, sizeof(v.type)-1);
        break;
    case 11:
        strncpy(v.version, _term, sizeof(v.version)-1);
        break;
    case 15:
        strncpy(v.build_date, _term, sizeof(v.build_date)-1);
        break;
    }
#pragma GCC diagnostic pop
}
#endif // AP_GPS_NMEA_UNICORE_ENABLED

#if AP_GPS_NMEA_TESEO_ENABLED
/*
  match a $PSTM sentence name, given without the PSTM prefix. Other
  $PSTM sentences, such as the $PSTMSWCONFIG lines of the boot
  banner, are ignored
 */
uint16_t AP_GPS_NMEA::teseo_sentence_type(const char *name)
{
    static const struct {
        const char *name;
        uint16_t type;
    } types[] {
        { "KFCOV", _GPS_SENTENCE_PSTMKFCOV },
        { "PVRAW", _GPS_SENTENCE_PSTMPVRAW },
        { "VER", _GPS_SENTENCE_PSTMVER },
        { "SETPAROK", _GPS_SENTENCE_PSTMSETPAROK },
        { "SETPARERROR", _GPS_SENTENCE_PSTMSETPARERROR },
        { "SAVEPAROK", _GPS_SENTENCE_PSTMSAVEPAROK },
        { "SETPAR", _GPS_SENTENCE_PSTMSETPAR },
        { "GETPARERROR", _GPS_SENTENCE_PSTMGETPARERROR },
    };
    for (const auto &t : types) {
        if (strcmp(name, t.name) == 0) {
            return t.type;
        }
    }
    return _GPS_SENTENCE_OTHER;
}

/*
  parse a $PSTM sentence term

  Examples:
    $PSTMKFCOV,8.7,50.9,25.4,150.7,0.4,0.1,0.0,0.2*49
    $PSTMPVRAW,144056.000,5131.12414,N,00005.31484,W,2,09,1.2,043.31,M,47.0,M,-0.6,0.1,0.6*58
    $PSTMSETPAR,3200,410000*<checksum>
 */
void AP_GPS_NMEA::parse_teseo_field(uint16_t term_number, const char *term)
{
    switch (_sentence_type) {
    case _GPS_SENTENCE_PSTMKFCOV:
        switch (term_number) {
        case 1:
            _pstmkfcov.pos_std = strtof(term, nullptr);
            break;
        case 2 ... 4:
            _pstmkfcov.pos_cov[term_number - 2] = strtof(term, nullptr);
            break;
        case 5:
            _pstmkfcov.vel_std = strtof(term, nullptr);
            break;
        case 6 ... 8:
            _pstmkfcov.vel_cov[term_number - 6] = strtof(term, nullptr);
            break;
        }
        if (term_number >= 1 && term_number <= 8) {
            // mark which KFCOV fields were received
            _pstmkfcov.have |= 1U << (term_number-1);
        }
        break;
    case _GPS_SENTENCE_PSTMPVRAW:
        switch (term_number) {
        case 6:
            _pstmpvraw.quality = atol(term);
            break;
        case 13 ... 15:
            _pstmpvraw.vel_NEV[term_number - 13] = strtof(term, nullptr);
            _pstmpvraw.have |= 1U << (term_number-13);
            break;
        }
        break;
    case _GPS_SENTENCE_PSTMVER:
        if (term_number == 1) {
            strncpy_noterm(_teseo_reply.version, term, sizeof(_teseo_reply.version)-1);
        }
        break;
    case _GPS_SENTENCE_PSTMSETPAROK:
    case _GPS_SENTENCE_PSTMSETPARERROR:
    case _GPS_SENTENCE_PSTMSETPAR:
        switch (term_number) {
        case 1:
            // <block><id>, e.g. 1200 is CDB 200 in the current configuration
            _teseo_reply.param = strtoul(term, nullptr, 10);
            break;
        case 2:
            if (_sentence_type == _GPS_SENTENCE_PSTMSETPAR) {
                // integers are in hex, with or without 0x
                _teseo_reply.value = strtoul(term, nullptr, 16);
                _teseo_reply.value_float = strtof(term, nullptr);
            }
            break;
        }
        break;
    }
}

/*
  use a $PSTM sentence which passed its checksum
 */
void AP_GPS_NMEA::teseo_sentence_complete(uint32_t now)
{
    auto &t = _teseo;
    switch (_sentence_type) {
    case _GPS_SENTENCE_PSTMKFCOV: {
        _have_KFCOV = true;
        // PosStd, PosVcov and VelStd, all empty without a fix
        const uint8_t needed = (1U<<0) | (1U<<3) | (1U<<4);
        if ((_pstmkfcov.have & needed) != needed) {
            break;
        }
        // PosStd is the horizontal 1-sigma: in the example it is
        // sqrt(PosNcov + PosEcov)
        state.horizontal_accuracy = _pstmkfcov.pos_std;
        state.vertical_accuracy = safe_sqrt(_pstmkfcov.pos_cov.z);
        state.speed_accuracy = _pstmkfcov.vel_std;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        state.have_speed_accuracy = true;
        _last_vaccuracy_ms = now;
        break;
    }
    case _GPS_SENTENCE_PSTMPVRAW:
        if (_pstmpvraw.quality == 0 || _pstmpvraw.have != 7) {
            break;
        }
        // unfiltered north, east and vertical velocity. ST don't
        // document the sign of Vel_V, but it follows the altitude
        // rate, so assume it is positive up
        state.velocity.x = _pstmpvraw.vel_NEV.x;
        state.velocity.y = _pstmpvraw.vel_NEV.y;
        state.velocity.z = -_pstmpvraw.vel_NEV.z;
        state.have_vertical_velocity = true;
        _last_vvelocity_ms = now;
        // we prefer a true 3D velocity when available
        velocity_to_speed_course(state);
        _last_3D_velocity_ms = now;
        break;
    case _GPS_SENTENCE_PSTMVER:
        if (strncmp(_teseo_reply.version, "BINIMG", 6) == 0 &&
            strcmp(_teseo_reply.version, t.version) != 0) {
            memcpy(t.version, _teseo_reply.version, sizeof(t.version));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %u: Teseo %s", unsigned(state.instance+1), t.version);
        }
        break;
    case _GPS_SENTENCE_PSTMSETPAROK:
        if (t.state == TeseoConfigState::SET &&
            _teseo_reply.param == 1000U + t.cmds[t.index].id) {
            t.have_reply = true;
        }
        break;
    case _GPS_SENTENCE_PSTMSETPARERROR:
        // may not name the parameter, but only one command is ever in flight
        if (t.state == TeseoConfigState::SET) {
            t.have_reply = true;
            t.reply_error = true;
        }
        break;
    case _GPS_SENTENCE_PSTMSAVEPAROK:
        if (t.state == TeseoConfigState::SAVE) {
            t.have_reply = true;
        }
        break;
    case _GPS_SENTENCE_PSTMSETPAR:
        // reply to $PSTMGETPAR of the NVM stored configuration
        if (t.state == TeseoConfigState::VERIFY &&
            _teseo_reply.param == 3000U + t.cmds[t.index].id) {
            t.have_reply = true;
            t.reply_match = teseo_reply_matches(t.cmds[t.index]);
        }
        break;
    case _GPS_SENTENCE_PSTMGETPARERROR:
        // can't read it back, so write it
        if (t.state == TeseoConfigState::VERIFY) {
            t.have_reply = true;
            t.reply_match = false;
        }
        break;
    }
}

/*
  true if the value in a $PSTMGETPAR reply already has what cmd would set
 */
bool AP_GPS_NMEA::teseo_reply_matches(const TeseoCommand &cmd) const
{
    if (cmd.is_float) {
        return fabsf(_teseo_reply.value_float - cmd.value_float) < 0.001f;
    }
    const uint32_t v = _teseo_reply.value;
    switch (cmd.mode) {
    case TeseoSetMode::VALUE:
        return v == cmd.value;
    case TeseoSetMode::OR:
        return (v & cmd.value) == cmd.value;
    case TeseoSetMode::CLEAR:
        return (v & cmd.value) == 0;
    }
    return false;
}

/*
  configuration failed: count it, and leave the module alone until
  the next detection
 */
void AP_GPS_NMEA::teseo_config_failed(void)
{
    if (state.instance < ARRAY_SIZE(_teseo_config_attempts)) {
        _teseo_config_attempts[state.instance]++;
    }
    _teseo.state = TeseoConfigState::DONE;
}

/*
  queue a $PSTMSETPAR for teseo_config_update() to send, returning
  nullptr if the queue is full
 */
AP_GPS_NMEA::TeseoCommand *AP_GPS_NMEA::teseo_add_command(uint16_t id, TeseoSetMode mode, uint32_t value)
{
    auto &t = _teseo;
    if (t.num_cmds >= ARRAY_SIZE(t.cmds)) {
        return nullptr;
    }
    TeseoCommand &cmd = t.cmds[t.num_cmds++];
    cmd = {};
    cmd.id = id;
    cmd.mode = mode;
    cmd.value = value;
    return &cmd;
}

/*
  configure a Teseo when it is detected, rather than resending config
  every period: $PSTMSETPAR only changes the RAM copy of the
  configuration, $PSTMSAVEPAR writes flash and $PSTMSRR reboots the
  module from it.

  Read back each configuration data block (CDB) we care about from
  NVM. If any differs, or GPS_SAVE_CFG=1 and this is the first attempt
  since boot, write them all, save to NVM and restart the
  module so they take effect. The module is not written to while
  armed. If the baud rate changed, AP_GPS times out this driver and
  finds the module again at the new rate, and the new driver finds it
  already configured. One command is in flight at a time, as
  $PSTMSETPARERROR may not say which parameter it is for.
 */
void AP_GPS_NMEA::teseo_config_update(void)
{
    auto &t = _teseo;
    const uint32_t now_ms = AP_HAL::millis();
    const unsigned instance = state.instance+1;

    if (t.waiting) {
        if (!t.have_reply) {
            const uint32_t timeout_ms = t.state == TeseoConfigState::SAVE ? TESEO_SAVE_TIMEOUT_MS : TESEO_REPLY_TIMEOUT_MS;
            if (now_ms - t.sent_ms < timeout_ms) {
                return;
            }
            t.waiting = false;
            if (++t.retries > TESEO_MAX_RETRIES) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %u: Teseo config failed, no reply", instance);
                teseo_config_failed();
            }
            // else resend
            return;
        }

        // act on the reply
        t.waiting = false;
        t.retries = 0;
        if (t.reply_error) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %u: Teseo rejected CDB %u", instance, t.cmds[t.index].id);
            teseo_config_failed();
            return;
        }
        switch (t.state) {
        case TeseoConfigState::VERIFY: {
            bool write = !t.reply_match;
            if (!write && ++t.index >= t.num_cmds) {
                // GPS_SAVE_CFG=1 saves once per boot even if unchanged
                write = gps._save_config == 1 && _teseo_config_attempts[state.instance] == 0;
                if (!write) {
                    t.state = TeseoConfigState::DONE;
                    return;
                }
            }
            if (write) {
                _teseo_config_attempts[state.instance]++;
                t.state = TeseoConfigState::SET;
                t.index = 0;
            }
            break;
        }
        case TeseoConfigState::SET:
            if (++t.index >= t.num_cmds) {
                t.state = TeseoConfigState::SAVE;
            }
            break;
        case TeseoConfigState::SAVE:
            t.state = TeseoConfigState::RESTART;
            break;
        default:
            break;
        }
    }

    if ((t.state == TeseoConfigState::SET ||
         t.state == TeseoConfigState::SAVE ||
         t.state == TeseoConfigState::RESTART) &&
        hal.util->get_soft_armed()) {
        // wait for disarm rather than change or restart it in flight
        return;
    }

    switch (t.state) {
    case TeseoConfigState::START: {
        if (state.instance >= ARRAY_SIZE(_teseo_config_attempts) ||
            _teseo_config_attempts[state.instance] >= TESEO_MAX_CONFIG_ATTEMPTS) {
            t.state = TeseoConfigState::DONE;
            return;
        }
        if (gps._save_config == 0) {
            // writing would only change RAM, and it takes a restart
            // from NVM to apply
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %u: Teseo not configured, GPS_SAVE_CFG=0", instance);
            t.state = TeseoConfigState::DONE;
            return;
        }
        // what we want, from the parameters
        t.num_cmds = 0;
        t.index = 0;
        t.retries = 0;
        // CPU clock from the 192f0 source, undivided
        teseo_add_command(130, TeseoSetMode::VALUE, 0x0U);
        // message list: only GGA, RMC, VTG and $PSTMKFCOV.
        teseo_add_command(201, TeseoSetMode::VALUE, 0x2U | 0x10U | 0x40U | 0x8000000U);
        // message list, high word: only $PSTMPVRAW
        teseo_add_command(228, TeseoSetMode::VALUE, 0x4000U);
        // fix period in seconds
        TeseoCommand *period = teseo_add_command(303, TeseoSetMode::VALUE, 0);
        if (period != nullptr) {
            period->is_float = true;
            period->value_float = gps.get_rate_ms(state.instance) * 0.001f;
        }
        const uint16_t gnss_mode = params.gnss_mode;
        if (gnss_mode != 0) {
            // GPS_GNSS_MODE bits for constellations.
            const uint16_t GPS = 1U<<0, SBAS = 1U<<1, GALILEO = 1U<<2, BEIDOU = 1U<<3;
            const uint16_t QZSS = 1U<<5, GLONASS = 1U<<6, NAVIC = 1U<<7;
            const struct {
                uint16_t mode_bit;
                uint16_t cdb_id;
                uint32_t bits;
            } constellations[] {
                { GPS, 200, 0x410000U },
                { GLONASS, 200, 0x220000U },
                { QZSS, 200, 0x800080U },
                { SBAS, 200, 0x4U },
                { GALILEO, 227, 0xC0U },
                { BEIDOU, 227, 0x300U },
                { NAVIC, 227, 0x18000000U },
            };

            // Set or clear each of the constellations
            const uint16_t systems = gnss_mode & (GPS | GALILEO | BEIDOU | QZSS | GLONASS | NAVIC);
            if (systems != 0) {
                const uint16_t cdb_ids[] { 200, 227 };
                for (const uint16_t id : cdb_ids) {
                    uint32_t set_bits = 0, clear_bits = 0;
                    for (const auto &c : constellations) {
                        if (c.cdb_id != id) {
                            continue;
                        }
                        if (gnss_mode & c.mode_bit) {
                            set_bits |= c.bits;
                        } else {
                            clear_bits |= c.bits;
                        }
                    }
                    if (set_bits != 0) {
                        teseo_add_command(id, TeseoSetMode::OR, set_bits);
                    }
                    if (clear_bits != 0) {
                        teseo_add_command(id, TeseoSetMode::CLEAR, clear_bits);
                    }
                }
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS %u: Teseo can't use GNSS_MODE %u", instance, unsigned(gnss_mode));
            }
        }
        // NMEA baud rate 230400
        teseo_add_command(102, TeseoSetMode::VALUE, 0xBU);
        t.state = TeseoConfigState::VERIFY;
        FALLTHROUGH;
    }

    case TeseoConfigState::VERIFY:
        // block 3 is the configuration stored in NVM
        if (!nmea_printf(port, "$PSTMGETPAR,3%03u", t.cmds[t.index].id)) {
            return;
        }
        break;

    case TeseoConfigState::SET: {
        const TeseoCommand &cmd = t.cmds[t.index];
        bool sent;
        if (cmd.is_float) {
            sent = nmea_printf(port, "$PSTMSETPAR,1%03u,%.3g", cmd.id, cmd.value_float);
        } else if (cmd.mode == TeseoSetMode::VALUE) {
            sent = nmea_printf(port, "$PSTMSETPAR,1%03u,0x%X", cmd.id, unsigned(cmd.value));
        } else {
            // mode 1 ORs the bits in, mode 2 clears them
            sent = nmea_printf(port, "$PSTMSETPAR,1%03u,0x%X,%u", cmd.id, unsigned(cmd.value), unsigned(cmd.mode));
        }
        if (!sent) {
            return;
        }
        break;
    }

    case TeseoConfigState::SAVE:
        if (!nmea_printf(port, "$PSTMSAVEPAR")) {
            return;
        }
        break;

    case TeseoConfigState::RESTART:
        if (!nmea_printf(port, "$PSTMSRR")) {
            return;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS %u: Teseo config saved, restarting", instance);
        t.state = TeseoConfigState::DONE;
        return;

    case TeseoConfigState::DONE:
        return;
    }

    t.waiting = true;
    t.have_reply = false;
    t.reply_error = false;
    t.reply_match = false;
    t.sent_ms = now_ms;
}
#endif // AP_GPS_NMEA_TESEO_ENABLED

/*
  detect a NMEA GPS. Adds one byte, and returns true if the stream
  matches a NMEA string
 */
bool
AP_GPS_NMEA::_detect(struct NMEA_detect_state &state, uint8_t data)
{
	switch (state.step) {
	case 0:
		state.ck = 0;
		if ('$' == data) {
			state.step++;
		}
		break;
	case 1:
		if ('*' == data) {
			state.step++;
		} else {
			state.ck ^= data;
		}
		break;
	case 2:
		if (hexdigit(state.ck>>4) == data) {
			state.step++;
		} else {
			state.step = 0;
		}
		break;
	case 3:
		if (hexdigit(state.ck&0xF) == data) {
            state.step = 0;
			return true;
		}
		state.step = 0;
		break;
    }
    return false;
}

/*
  send type specific config strings
 */
void AP_GPS_NMEA::send_config(void)
{
    const auto type = get_type();
    _expect_agrica = (type == AP_GPS::GPS_TYPE_UNICORE_NMEA ||
                      type == AP_GPS::GPS_TYPE_UNICORE_MOVINGBASE_NMEA);
#if AP_GPS_NMEA_TESEO_ENABLED
    _is_teseo = (type == AP_GPS::GPS_TYPE_TESEO_NMEA);
#endif
    if (gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        // not doing auto-config
        return;
    }
#if AP_GPS_NMEA_TESEO_ENABLED
    if (_is_teseo) {
        // configured once, not resent every period
        teseo_config_update();
        return;
    }
#endif
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_config_ms < AP_GPS_NMEA_CONFIG_PERIOD_MS) {
        return;
    }
    last_config_ms = now_ms;
    const uint16_t rate_ms = params.rate_ms;
#if AP_GPS_NMEA_UNICORE_ENABLED
    const float rate_s = rate_ms * 0.001;
#endif
    const uint8_t rate_hz = 1000U / rate_ms;

    switch (get_type()) {
#if AP_GPS_NMEA_UNICORE_ENABLED
    case AP_GPS::GPS_TYPE_UNICORE_MOVINGBASE_NMEA:
        port->printf("\r\nCONFIG HEADING FIXLENGTH\r\n" \
                     "CONFIG UNDULATION AUTO\r\n" \
                     "CONFIG\r\n" \
                     "UNIHEADINGA %.3f\r\n",
                     rate_s);
        state.gps_yaw_configured = true;
        FALLTHROUGH;

    case AP_GPS::GPS_TYPE_UNICORE_NMEA: {
        port->printf("\r\nAGRICA %.3f\r\n" \
                     "MODE MOVINGBASE\r\n" \
                     "GNGGA %.3f\r\n" \
                     "GNRMC %.3f\r\n",
                     rate_s, rate_s, rate_s);
        if (!_have_unicore_versiona) {
            // get version information for logging if we don't have it yet
            port->printf("VERSIONA\r\n");
            if (gps._save_config) {
                // save config changes for fast startup
                port->printf("SAVECONFIG\r\n");
            }
        }
        break;
    }
#endif // AP_GPS_NMEA_UNICORE_ENABLED

    case AP_GPS::GPS_TYPE_HEMI: {
        port->printf(
        "$JATT,NMEAHE,0\r\n" /* Prefix of GP on the HDT message */      \
        "$JASC,GPGGA,%u\r\n" /* GGA at 5Hz */                            \
        "$JASC,GPRMC,%u\r\n" /* RMC at 5Hz */                            \
        "$JASC,GPVTG,%u\r\n" /* VTG at 5Hz */                            \
        "$JASC,GPHDT,%u\r\n" /* HDT at 5Hz */                            \
        "$JMODE,SBASR,YES\r\n" /* Enable SBAS */,
        rate_hz, rate_hz, rate_hz, rate_hz);
        break;
    }

    case AP_GPS::GPS_TYPE_ALLYSTAR:
        nmea_printf(port, "$PHD,06,42,UUUUTTTT,BB,0,%u,55,0,%u,0,0,0",
                    unsigned(rate_hz), unsigned(rate_ms));
        break;

    default:
        break;
    }

#ifdef AP_GPS_NMEA_CUSTOM_CONFIG_STRING
    // allow for custom config strings, useful for peripherals
    port->printf("%s\r\n", AP_GPS_NMEA_CUSTOM_CONFIG_STRING);
#endif
}

/*
  return health status
 */
bool AP_GPS_NMEA::is_healthy(void) const
{
    switch (get_type()) {
#if AP_GPS_NMEA_UNICORE_ENABLED
    case AP_GPS::GPS_TYPE_UNICORE_MOVINGBASE_NMEA:
    case AP_GPS::GPS_TYPE_UNICORE_NMEA:
        // we should be getting AGRICA messages
        return _last_AGRICA_ms != 0;
#endif // AP_GPS_NMEA_UNICORE_ENABLED

    case AP_GPS::GPS_TYPE_HEMI:
        // we should be getting HDR for yaw
        return _last_yaw_ms != 0;

    case AP_GPS::GPS_TYPE_ALLYSTAR:
        // we should get vertical velocity and accuracy from PHD
        return _last_vvelocity_ms != 0 && _last_vaccuracy_ms != 0;

#if AP_GPS_NMEA_TESEO_ENABLED
    case AP_GPS::GPS_TYPE_TESEO_NMEA:
        // we should get accuracy from $PSTMKFCOV, which has empty
        // fields until there is a fix
        return _have_KFCOV;
#endif

    default:
        break;
    }
    return true;
}

// get the velocity lag
bool AP_GPS_NMEA::get_lag(float &lag_sec) const
{
    switch (get_type()) {
#if AP_GPS_NMEA_UNICORE_ENABLED
    case AP_GPS::GPS_TYPE_UNICORE_MOVINGBASE_NMEA:
    case AP_GPS::GPS_TYPE_UNICORE_NMEA:
        lag_sec = 0.14;
        break;
#endif // AP_GPS_NMEA_UNICORE_ENABLED

    default:
        lag_sec = 0.2;
        break;
    }
    return true;
}

#if HAL_LOGGING_ENABLED
void AP_GPS_NMEA::Write_AP_Logger_Log_Startup_messages() const
{
    AP_GPS_Backend::Write_AP_Logger_Log_Startup_messages();
#if AP_GPS_NMEA_UNICORE_ENABLED
    if (_have_unicore_versiona) {
        AP::logger().Write_MessageF("NMEA %u %s %s %s",
                                    state.instance+1,
                                    _versiona.type,
                                    _versiona.version,
                                    _versiona.build_date);
    }
#endif
#if AP_GPS_NMEA_TESEO_ENABLED
    if (_teseo.version[0] != 0) {
        AP::logger().Write_MessageF("NMEA %u Teseo %s",
                                    state.instance+1,
                                    _teseo.version);
    }
#endif
}
#endif

#endif // AP_GPS_NMEA_ENABLED
