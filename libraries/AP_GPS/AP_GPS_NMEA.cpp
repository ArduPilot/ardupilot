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
                if (state.status >= AP_GPS::GPS_OK_FIX_3D) {
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
                    state.location.alt  = _new_altitude;
                    state.location.lat  = _new_latitude;
                    state.location.lng  = _new_longitude;
                }
                state.num_sats      = _new_satellite_count;
                state.hdop          = _new_hdop;
                switch(_new_quality_indicator) {
                case 0: // Fix not available or invalid
                    state.status = AP_GPS::NO_FIX;
                    break;
                case 1: // GPS SPS Mode, fix valid
                    state.status = AP_GPS::GPS_OK_FIX_3D;
                    break;
                case 2: // Differential GPS, SPS Mode, fix valid
                    state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                    break;
                case 3: // GPS PPS Mode, fix valid
                    state.status = AP_GPS::GPS_OK_FIX_3D;
                    break;
                case 4: // Real Time Kinematic. System used in RTK mode with fixed integers
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                    break;
                case 5: // Float RTK. Satellite system used in RTK mode, floating integers
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                    break;
                case 6: // Estimated (dead reckoning) Mode
                    state.status = AP_GPS::NO_FIX;
                    break;
                default://to maintain compatibility with MAV_GPS_INPUT and others
                    state.status = AP_GPS::GPS_OK_FIX_3D;
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
                state.location.alt     = _ksxt.fields[3]*1.0e2;
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
                state.location.alt = ag.alt*1.0e2;
                state.undulation   = -ag.undulation;
                state.velocity = ag.vel_NED;
                velocity_to_speed_course(state);
                state.speed_accuracy = ag.vel_stddev.length();
                state.horizontal_accuracy = ag.pos_stddev.xy().length();
                state.vertical_accuracy = ag.pos_stddev.z;
                state.have_vertical_velocity = true;
                state.have_speed_accuracy = true;
                state.have_horizontal_accuracy = true;
                state.have_vertical_accuracy = true;
                state.have_undulation = true;
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
                const float alt_diff = dist*tanf(radians(-uh.pitch));
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
            _new_speed = (_parse_decimal_100(_term) * 514) / 1000;       // knots-> m/sec, approximiates * 0.514
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
            _phd.fields[_term_number-6] = atol(_term);
            break;
        case _GPS_SENTENCE_KSXT + 1 ... _GPS_SENTENCE_KSXT + 22: // KSXT message, fields
            _ksxt.fields[_term_number-1] = atof(_term);
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
        ag.vel_NED[term_number-25] = atof(term);
        break;
    case 27:
        // AGRIC gives velocity up
        ag.vel_NED.z = -atof(term);
        break;
    case 28 ... 30:
        ag.vel_stddev[term_number-28] = atof(term);
        break;
    case 31:
        ag.lat = atof(term);
        break;
    case 32:
        ag.lng = atof(term);
        break;
    case 33:
        ag.alt = atof(term);
        break;
    case 49:
        ag.itow = atol(term);
        break;
    case 37 ... 39:
        ag.pos_stddev[term_number-37] = atof(term);
        break;
    case 52:
        ag.undulation = atof(term);
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
        uh.baseline_length = atof(term);
        break;
    case 5:
        uh.heading = atof(term);
        break;
    case 6:
        uh.pitch = atof(term);
        break;
    case 8:
        uh.heading_sd = atof(term);
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
#if defined(__GNUC__) &&  __GNUC__ >= 10
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
    if (gps._auto_config == AP_GPS::GPS_AUTO_CONFIG_DISABLE) {
        // not doing auto-config
        return;
    }
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_config_ms < AP_GPS_NMEA_CONFIG_PERIOD_MS) {
        return;
    }
    last_config_ms = now_ms;
    const uint16_t rate_ms = gps._rate_ms[state.instance];
#if AP_GPS_NMEA_UNICORE_ENABLED
    const float rate_s = rate_ms * 0.001;
#endif
    const uint8_t rate_hz = 1000U / rate_ms;

    switch (get_type()) {
#if AP_GPS_NMEA_UNICORE_ENABLED
    case AP_GPS::GPS_TYPE_UNICORE_MOVINGBASE_NMEA:
        port->printf("\r\nMODE MOVINGBASE\r\n" \
                     "CONFIG HEADING FIXLENGTH\r\n" \
                     "CONFIG UNDULATION AUTO\r\n" \
                     "CONFIG\r\n" \
                     "UNIHEADINGA %.3f\r\n",
                     rate_s);
        state.gps_yaw_configured = true;
        FALLTHROUGH;

    case AP_GPS::GPS_TYPE_UNICORE_NMEA: {
        port->printf("\r\nAGRICA %.3f\r\n" \
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

void AP_GPS_NMEA::Write_AP_Logger_Log_Startup_messages() const
{
#if HAL_LOGGING_ENABLED
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
#endif
}

#endif // AP_GPS_NMEA_ENABLED
