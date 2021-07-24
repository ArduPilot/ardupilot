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

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "AP_GPS_NMEA.h"

extern const AP_HAL::HAL& hal;

// optionally log all NMEA data for debug purposes
// #define NMEA_LOG_PATH "nmea.log"

#ifdef NMEA_LOG_PATH
#include <stdio.h>
#endif

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)        (_x - '0')
#define hexdigit(x) ((x)>9?'A'+((x)-10):'0'+(x))

bool AP_GPS_NMEA::read(void)
{
    int16_t numc;
    bool parsed = false;

    numc = port->available();
    while (numc--) {
        char c = port->read();
#ifdef NMEA_LOG_PATH
        static FILE *logf = nullptr;
        if (logf == nullptr) {
            logf = fopen(NMEA_LOG_PATH, "wb");
        }
        if (logf != nullptr) {
            ::fwrite(&c, 1, 1, logf);
        }
#endif
        if (_decode(c)) {
            parsed = true;
        }
    }
    return parsed;
}

bool AP_GPS_NMEA::_decode(char c)
{
    bool valid_sentence = false;

    _sentence_length++;
        
    switch (c) {
    case ',': // term terminators
        _parity ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
        if (_sentence_done) {
            return false;
        }
        if (_term_offset < sizeof(_term)) {
            _term[_term_offset] = 0;
            valid_sentence = _term_complete();
        }
        ++_term_number;
        _term_offset = 0;
        _is_checksum_term = c == '*';
        return valid_sentence;

    case '$': // sentence begin
        _term_number = _term_offset = 0;
        _parity = 0;
        _sentence_type = _GPS_SENTENCE_OTHER;
        _is_checksum_term = false;
        _gps_data_good = false;
        _sentence_length = 1;
        _sentence_done = false;
        return valid_sentence;
    }

    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
    if (!_is_checksum_term)
        _parity ^= c;

    return valid_sentence;
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
      if we have seen the $PHD messages then wait for them again. This
      is important as the have_vertical_velocity field will be
      overwritten by fill_3d_velocity()
     */
    if (_last_PHD_12_ms != 0 &&
        now - _last_PHD_12_ms > 150 &&
        now - _last_PHD_12_ms < 1000) {
        // waiting on PHD_12
        return false;
    }
    if (_last_PHD_26_ms != 0 &&
        now - _last_PHD_26_ms > 150 &&
        now - _last_PHD_26_ms < 1000) {
        // waiting on PHD_26
        return false;
    }

    // prevent these messages being used again
    if (_last_VTG_ms != 0) {
        _last_VTG_ms = 1;
    }

    if (now - _last_HDT_THS_ms > 300) {
        // we have lost GPS yaw
        state.have_gps_yaw = false;
    }

    // special case for fixing low output rate of ALLYSTAR GPS modules
    const int32_t dt_ms = now - _last_fix_ms;
    if (labs(dt_ms - gps._rate_ms[state.instance]) > 50 &&
        get_type() == AP_GPS::GPS_TYPE_ALLYSTAR) {
        nmea_printf(port, "$PHD,06,42,UUUUTTTT,BB,0,%u,55,0,%u,0,0,0",
                    1000U/gps._rate_ms[state.instance],
                    gps._rate_ms[state.instance]);
    }

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
        uint8_t nibble_high = 0;
        uint8_t nibble_low  = 0;
        if (!hex_to_uint8(_term[0], nibble_high) || !hex_to_uint8(_term[1], nibble_low)) {
            return false;
        }
        const uint8_t checksum = (nibble_high << 4u) | nibble_low;
        if (checksum == _parity) {
            if (_gps_data_good) {
                uint32_t now = AP_HAL::millis();
                switch (_sentence_type) {
                case _GPS_SENTENCE_RMC:
                    _last_RMC_ms = now;
                    //time                        = _new_time;
                    //date                        = _new_date;
                    state.location.lat     = _new_latitude;
                    state.location.lng     = _new_longitude;
                    state.ground_speed     = _new_speed*0.01f;
                    state.ground_course    = wrap_360(_new_course*0.01f);
                    make_gps_time(_new_date, _new_time * 10);
                    set_uart_timestamp(_sentence_length);
                    state.last_gps_time_ms = now;
                    if (_last_PHD_12_ms == 0 ||
                        now - _last_PHD_12_ms > 1000) {
                        fill_3d_velocity();
                    }
                    break;
                case _GPS_SENTENCE_GGA:
                    _last_GGA_ms = now;
                    state.location.alt  = _new_altitude;
                    state.location.lat  = _new_latitude;
                    state.location.lng  = _new_longitude;
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
                    state.ground_speed  = _new_speed*0.01f;
                    state.ground_course = wrap_360(_new_course*0.01f);
                    if (_last_PHD_12_ms == 0 ||
                        now - _last_PHD_12_ms > 1000) {
                        fill_3d_velocity();
                    }
                    // VTG has no fix indicator, can't change fix status
                    break;
                case _GPS_SENTENCE_HDT:
                case _GPS_SENTENCE_THS:
                    _last_HDT_THS_ms = now;
                    state.gps_yaw = wrap_360(_new_gps_yaw*0.01f);
                    state.have_gps_yaw = true;
                    state.gps_yaw_time_ms = AP_HAL::millis();
                    // remember that we are setup to provide yaw. With
                    // a NMEA GPS we can only tell if the GPS is
                    // configured to provide yaw when it first sends a
                    // HDT sentence.
                    state.gps_yaw_configured = true;
                    break;
                case _GPS_SENTENCE_PHD:
                    if (_phd.msg_id == 12) {
                        state.velocity.x = _phd.fields[0] * 0.01;
                        state.velocity.y = _phd.fields[1] * 0.01;
                        state.velocity.z = _phd.fields[2] * 0.01;
                        state.have_vertical_velocity = true;
                        _last_PHD_12_ms = now;
                    } else if (_phd.msg_id == 26) {
                        state.horizontal_accuracy = MAX(_phd.fields[0],_phd.fields[1]) * 0.001;
                        state.have_horizontal_accuracy = true;
                        state.vertical_accuracy = _phd.fields[2] * 0.001;
                        state.have_vertical_accuracy = true;
                        state.speed_accuracy = MAX(_phd.fields[3],_phd.fields[4]) * 0.001;
                        state.have_speed_accuracy = true;
                        _last_PHD_26_ms = now;
                    }
                }
            } else {
                switch (_sentence_type) {
                case _GPS_SENTENCE_RMC:
                case _GPS_SENTENCE_GGA:
                    // Only these sentences give us information about
                    // fix status.
                    state.status = AP_GPS::NO_FIX;
                    break;
                case _GPS_SENTENCE_THS:
                    state.have_gps_yaw = false;
                    break;
                }
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
            // HDT doesn't have a data qualifier
            _gps_data_good = true;
        } else if (strcmp(term_type, "THS") == 0) {
            _sentence_type = _GPS_SENTENCE_THS;
        } else if (strcmp(term_type, "VTG") == 0) {
            _sentence_type = _GPS_SENTENCE_VTG;
            // VTG may not contain a data qualifier, presume the solution is good
            // unless it tells us otherwise.
            _gps_data_good = true;
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
            _gps_data_good = _term[0] == 'A';
            break;
        case _GPS_SENTENCE_GGA + 6: // Fix data (GGA)
            _gps_data_good = _term[0] > '0';
            _new_quality_indicator = _term[0] - '0';
            break;
        case _GPS_SENTENCE_THS + 2: // validity (THS)
            _gps_data_good = _term[0] == 'A';
            break;
        case _GPS_SENTENCE_VTG + 9: // validity (VTG) (we may not see this field)
            _gps_data_good = _term[0] != 'N';
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
            if (_phd.msg_class == 1 && (_phd.msg_id == 12 || _phd.msg_id == 26)) {
                // we only support $PHD messages 1/12 and 1/26
                _gps_data_good = true;
            }
            break;
        case _GPS_SENTENCE_PHD + 5: // PHD message, itow
            _phd.itow = strtoul(_term, nullptr, 10);
            break;
        case _GPS_SENTENCE_PHD + 6 ... _GPS_SENTENCE_PHD + 11: // PHD message, fields
            _phd.fields[_term_number-6] = atol(_term);
            break;
        }
    }

    return false;
}

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
