// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
//  u-blox GPS driver for ArduPilot
//	Origin code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//  Substantially rewitten for new GPS driver structure by Andrew Tridgell
//
#include <AP_GPS.h>
#include "AP_GPS_UBLOX.h"
#include <DataFlash.h>

#define UBLOX_DEBUGGING 0
#define UBLOX_FAKE_3DLOCK 0

extern const AP_HAL::HAL& hal;

#if UBLOX_DEBUGGING
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

/*
  only do detailed hardware logging on boards likely to have more log
  storage space
 */
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
#define UBLOX_HW_LOGGING 1
#else
#define UBLOX_HW_LOGGING 0
#endif

AP_GPS_UBLOX::AP_GPS_UBLOX(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _step(0),
    _msg_id(0),
    _payload_length(0),
    _payload_counter(0),
    _fix_count(0),
    _class(0),
    _new_position(0),
    _new_speed(0),
    need_rate_update(false),
    _disable_counter(0),
    next_fix(AP_GPS::NO_FIX),
    rate_update_step(0),
    _last_5hz_time(0),
    _last_hw_status(0)
{
    // stop any config strings that are pending
    gps.send_blob_start(state.instance, NULL, 0);

    // configure the GPS for the messages we want
    _configure_gps();
}

/*
  send the next step of rate updates to the GPS. This reconfigures the
  GPS on the fly to have the right message rates. It needs to be
  careful to only send a message if there is sufficient buffer space
  available on the serial port to avoid it blocking the CPU
 */
void
AP_GPS_UBLOX::send_next_rate_update(void)
{
    if (port->txspace() < (int16_t)(sizeof(struct ubx_header)+sizeof(struct ubx_cfg_nav_rate)+2)) {
        // not enough space - do it next time
        return;
    }

    //hal.console->printf_P(PSTR("next_rate: %u\n"), (unsigned)rate_update_step);

    switch (rate_update_step++) {
    case 0:
        _configure_navigation_rate(200);
        break;
    case 1:
        _configure_message_rate(CLASS_NAV, MSG_POSLLH, 1); // 28+8 bytes
        break;
    case 2:
        _configure_message_rate(CLASS_NAV, MSG_STATUS, 1); // 16+8 bytes
        break;
    case 3:
        _configure_message_rate(CLASS_NAV, MSG_SOL, 1);    // 52+8 bytes
        break;
    case 4:
        _configure_message_rate(CLASS_NAV, MSG_VELNED, 1); // 36+8 bytes
        break;
#if UBLOX_HW_LOGGING
    case 5:
        // gather MON_HW at 0.5Hz
        _configure_message_rate(CLASS_MON, MSG_MON_HW, 2); // 64+8 bytes
        break;
    case 6:
        // gather MON_HW2 at 0.5Hz
        _configure_message_rate(CLASS_MON, MSG_MON_HW2, 2); // 24+8 bytes
        break;
#endif
    default:
        need_rate_update = false;
        rate_update_step = 0;
        break;
    }
}


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
AP_GPS_UBLOX::read(void)
{
    uint8_t data;
    int16_t numc;
    bool parsed = false;

    if (need_rate_update) {
        send_next_rate_update();
    }

    numc = port->available();
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        data = port->read();

	reset:
        switch(_step) {

        // Message preamble detection
        //
        // If we fail to match any of the expected bytes, we reset
        // the state machine and re-consider the failed byte as
        // the first byte of the preamble.  This improves our
        // chances of recovering from a mismatch and makes it less
        // likely that we will be fooled by the preamble appearing
        // as data in some other message.
        //
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            Debug("reset %u", __LINE__);
        // FALLTHROUGH
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;

        // Message header processing
        //
        // We sniff the class and message ID to decide whether we
        // are going to gather the message bytes or just discard
        // them.
        //
        // We always collect the length so that we can avoid being
        // fooled by preamble bytes in messages.
        //
        case 2:
            _step++;
            _class = data;
            _ck_b = _ck_a = data;                               // reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length = data;                             // payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte

            _payload_length += (uint16_t)(data<<8);
            if (_payload_length > 512) {
                Debug("large payload %u", (unsigned)_payload_length);
                // assume very large payloads are line noise
                _payload_length = 0;
                _step = 0;
				goto reset;
            }
            _payload_counter = 0;                               // prepare to receive payload
            break;

        // Receive message data
        //
        case 6:
            _ck_b += (_ck_a += data);                   // checksum byte
            if (_payload_counter < sizeof(_buffer)) {
                _buffer.bytes[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;

        // Checksum and message processing
        //
        case 7:
            _step++;
            if (_ck_a != data) {
                Debug("bad cka %x should be %x", data, _ck_a);
                _step = 0;
				goto reset;
            }
            break;
        case 8:
            _step = 0;
            if (_ck_b != data) {
                Debug("bad ckb %x should be %x", data, _ck_b);
                break;                                                  // bad checksum
            }

            if (_parse_gps()) {
                parsed = true;
            }
        }
    }
    return parsed;
}

// Private Methods /////////////////////////////////////////////////////////////
#if UBLOX_HW_LOGGING

void AP_GPS_UBLOX::log_mon_hw(void)
{
    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
        return;
    }
    struct log_Ubx1 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_UBX1_MSG),
        timestamp  : hal.scheduler->millis(),
        instance   : state.instance,
        noisePerMS : _buffer.mon_hw_60.noisePerMS,
        jamInd     : _buffer.mon_hw_60.jamInd,
        aPower     : _buffer.mon_hw_60.aPower,
        agcCnt     : _buffer.mon_hw_60.agcCnt,
    };
    if (_payload_length == 68) {
        pkt.noisePerMS = _buffer.mon_hw_68.noisePerMS;
        pkt.jamInd     = _buffer.mon_hw_68.jamInd;
        pkt.aPower     = _buffer.mon_hw_68.aPower;
        pkt.agcCnt     = _buffer.mon_hw_68.agcCnt;
    }
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));
}

void AP_GPS_UBLOX::log_mon_hw2(void)
{
    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
        return;
    }

    struct log_Ubx2 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_UBX2_MSG),
        timestamp : hal.scheduler->millis(),
        instance  : state.instance,
        ofsI      : _buffer.mon_hw2.ofsI,
        magI      : _buffer.mon_hw2.magI,
        ofsQ      : _buffer.mon_hw2.ofsQ,
        magQ      : _buffer.mon_hw2.magQ,
    };
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));
}

void AP_GPS_UBLOX::log_accuracy(void) {
    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
        return;
    }
    struct log_Ubx3 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_UBX3_MSG),
        timestamp  : hal.scheduler->millis(),
        instance   : state.instance,
        hAcc     : state.horizontal_accuracy,
        vAcc     : state.vertical_accuracy,
        sAcc     : state.speed_accuracy
    };
    gps._DataFlash->WriteBlock(&pkt, sizeof(pkt));
}
#endif // UBLOX_HW_LOGGING

void AP_GPS_UBLOX::unexpected_message(void)
{
    Debug("Unexpected message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
    if (++_disable_counter == 0) {
        // disable future sends of this message id, but
        // only do this every 256 messages, as some
        // message types can't be disabled and we don't
        // want to get into an ack war
        Debug("Disabling message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
        _configure_message_rate(_class, _msg_id, 0);
    }
}

bool
AP_GPS_UBLOX::_parse_gps(void)
{
    if (_class == CLASS_ACK) {
        Debug("ACK %u", (unsigned)_msg_id);
        return false;
    }

    if (_class == CLASS_CFG && _msg_id == MSG_CFG_NAV_SETTINGS) {
		Debug("Got settings %u min_elev %d drLimit %u\n", 
              (unsigned)_buffer.nav_settings.dynModel,
              (int)_buffer.nav_settings.minElev,
              (unsigned)_buffer.nav_settings.drLimit);
        _buffer.nav_settings.mask = 0;
        if (gps._navfilter != AP_GPS::GPS_ENGINE_NONE &&
            _buffer.nav_settings.dynModel != gps._navfilter) {
            // we've received the current nav settings, change the engine
            // settings and send them back
            Debug("Changing engine setting from %u to %u\n",
                  (unsigned)_buffer.nav_settings.dynModel, (unsigned)gps._navfilter);
            _buffer.nav_settings.dynModel = gps._navfilter;
            _buffer.nav_settings.mask |= 1;
        }
        if (gps._min_elevation != -100 &&
            _buffer.nav_settings.minElev != gps._min_elevation) {
            Debug("Changing min elevation to %d\n", (int)gps._min_elevation);
            _buffer.nav_settings.minElev = gps._min_elevation;
            _buffer.nav_settings.mask |= 2;
        }
        if (_buffer.nav_settings.mask != 0) {
            _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS,
                          &_buffer.nav_settings,
                          sizeof(_buffer.nav_settings));
        }
        return false;
    }

    if (_class == CLASS_CFG && _msg_id == MSG_CFG_SBAS && gps._sbas_mode != 2) {
		Debug("Got SBAS settings %u %u %u 0x%x 0x%x\n", 
              (unsigned)_buffer.sbas.mode,
              (unsigned)_buffer.sbas.usage,
              (unsigned)_buffer.sbas.maxSBAS,
              (unsigned)_buffer.sbas.scanmode2,
              (unsigned)_buffer.sbas.scanmode1);
        if (_buffer.sbas.mode != gps._sbas_mode) {
            _buffer.sbas.mode = gps._sbas_mode;
            _send_message(CLASS_CFG, MSG_CFG_SBAS,
                          &_buffer.sbas,
                          sizeof(_buffer.sbas));
        }
    }

#if UBLOX_HW_LOGGING
    if (_class == CLASS_MON) {
        if (_msg_id == MSG_MON_HW) {
            if (_payload_length == 60 || _payload_length == 68) {
                log_mon_hw();
            }
        } else if (_msg_id == MSG_MON_HW2) {
            if (_payload_length == 28) {
                log_mon_hw2();  
            }
        } else {
            unexpected_message();
        }
        return false;
    }
#endif // UBLOX_HW_LOGGING

    if (_class != CLASS_NAV) {
        unexpected_message();
        return false;
    }

    switch (_msg_id) {
    case MSG_POSLLH:
        Debug("MSG_POSLLH next_fix=%u", next_fix);
        _last_pos_time        = _buffer.posllh.time;
        state.location.lng    = _buffer.posllh.longitude;
        state.location.lat    = _buffer.posllh.latitude;
        state.location.alt    = _buffer.posllh.altitude_msl / 10;
        state.status          = next_fix;
        _new_position = true;
#if UBLOX_FAKE_3DLOCK
        state.location.lng = 1491652300L;
        state.location.lat = -353632610L;
        state.location.alt = 58400;
#endif
        state.horizontal_accuracy = _buffer.posllh.horizontal_accuracy*1.0e-3f;
        state.vertical_accuracy = _buffer.posllh.vertical_accuracy*1.0e-3f;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        break;
    case MSG_STATUS:
        Debug("MSG_STATUS fix_status=%u fix_type=%u",
              _buffer.status.fix_status,
              _buffer.status.fix_type);
        if (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) {
            if( _buffer.status.fix_type == AP_GPS_UBLOX::FIX_3D) {
                next_fix = AP_GPS::GPS_OK_FIX_3D;
            }else if (_buffer.status.fix_type == AP_GPS_UBLOX::FIX_2D) {
                next_fix = AP_GPS::GPS_OK_FIX_2D;
            }else{
                next_fix = AP_GPS::NO_FIX;
                state.status = AP_GPS::NO_FIX;
            }
        }else{
            next_fix = AP_GPS::NO_FIX;
            state.status = AP_GPS::NO_FIX;
        }
#if UBLOX_FAKE_3DLOCK
        state.status = AP_GPS::GPS_OK_FIX_3D;
        next_fix = state.status;
#endif
        break;
    case MSG_SOL:
        Debug("MSG_SOL fix_status=%u fix_type=%u",
              _buffer.solution.fix_status,
              _buffer.solution.fix_type);
        if (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) {
            if( _buffer.solution.fix_type == AP_GPS_UBLOX::FIX_3D) {
                next_fix = AP_GPS::GPS_OK_FIX_3D;
            }else if (_buffer.solution.fix_type == AP_GPS_UBLOX::FIX_2D) {
                next_fix = AP_GPS::GPS_OK_FIX_2D;
            }else{
                next_fix = AP_GPS::NO_FIX;
                state.status = AP_GPS::NO_FIX;
            }
        }else{
            next_fix = AP_GPS::NO_FIX;
            state.status = AP_GPS::NO_FIX;
        }
        state.num_sats    = _buffer.solution.satellites;
        state.hdop        = _buffer.solution.position_DOP;
        if (next_fix >= AP_GPS::GPS_OK_FIX_2D) {
            state.last_gps_time_ms = hal.scheduler->millis();
            if (state.time_week == _buffer.solution.week &&
                state.time_week_ms + 200 == _buffer.solution.time) {
                // we got a 5Hz update. This relies on the way
                // that uBlox gives timestamps that are always
                // multiples of 200 for 5Hz
                _last_5hz_time = state.last_gps_time_ms;
            }
            state.time_week_ms    = _buffer.solution.time;
            state.time_week       = _buffer.solution.week;
        }
#if UBLOX_FAKE_3DLOCK
        next_fix = state.status;
        state.num_sats = 10;
        state.hdop = 200;
        state.time_week = 1721;
        state.time_week_ms = hal.scheduler->millis() + 3*60*60*1000 + 37000;
        state.last_gps_time_ms = hal.scheduler->millis();
#endif
        break;
    case MSG_VELNED:
        Debug("MSG_VELNED");
        _last_vel_time         = _buffer.velned.time;
        state.ground_speed     = _buffer.velned.speed_2d*0.01f;          // m/s
        state.ground_course_cd = _buffer.velned.heading_2d / 1000;       // Heading 2D deg * 100000 rescaled to deg * 100
        state.have_vertical_velocity = true;
        state.velocity.x = _buffer.velned.ned_north * 0.01f;
        state.velocity.y = _buffer.velned.ned_east * 0.01f;
        state.velocity.z = _buffer.velned.ned_down * 0.01f;
        state.have_speed_accuracy = true;
        state.speed_accuracy = _buffer.velned.speed_accuracy*0.01f;
        _new_speed = true;
        break;
    default:
        Debug("Unexpected NAV message 0x%02x", (unsigned)_msg_id);
        if (++_disable_counter == 0) {
            Debug("Disabling NAV message 0x%02x", (unsigned)_msg_id);
            _configure_message_rate(CLASS_NAV, _msg_id, 0);
        }
        return false;
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed && _last_vel_time == _last_pos_time) {
        _new_speed = _new_position = false;
		_fix_count++;
        if ((hal.scheduler->millis() - _last_5hz_time) > 15000U && !need_rate_update) {
            // the GPS is running slow. It possibly browned out and
            // restarted with incorrect parameters. We will slowly
            // send out new parameters to fix it
            need_rate_update = true;
            rate_update_step = 0;
            _last_5hz_time = hal.scheduler->millis();
        }

		if (_fix_count == 50 && gps._sbas_mode != 2) {
			// ask for SBAS settings every 20 seconds
			Debug("Asking for SBAS setting\n");
			_send_message(CLASS_CFG, MSG_CFG_SBAS, NULL, 0);
		}
		if (_fix_count == 100) {
			// ask for nav settings every 20 seconds
			Debug("Asking for engine setting\n");
			_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
            _fix_count = 0;
		}

#if UBLOX_HW_LOGGING
        log_accuracy();
#endif //UBLOX_HW_LOGGING

        return true;
    }
    return false;
}


// UBlox auto configuration

/*
 *  update checksum for a set of bytes
 */
void
AP_GPS_UBLOX::_update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b)
{
    while (len--) {
        ck_a += *data;
        ck_b += ck_a;
        data++;
    }
}


/*
 *  send a ublox message
 */
void
AP_GPS_UBLOX::_send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size)
{
    struct ubx_header header;
    uint8_t ck_a=0, ck_b=0;
    header.preamble1 = PREAMBLE1;
    header.preamble2 = PREAMBLE2;
    header.msg_class = msg_class;
    header.msg_id    = msg_id;
    header.length    = size;

    _update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
    _update_checksum((uint8_t *)msg, size, ck_a, ck_b);

    port->write((const uint8_t *)&header, sizeof(header));
    port->write((const uint8_t *)msg, size);
    port->write((const uint8_t *)&ck_a, 1);
    port->write((const uint8_t *)&ck_b, 1);
}


/*
 *  configure a UBlox GPS for the given message rate for a specific
 *  message class and msg_id
 */
void
AP_GPS_UBLOX::_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    struct ubx_cfg_msg_rate msg;
    msg.msg_class = msg_class;
    msg.msg_id    = msg_id;
    msg.rate          = rate;
    _send_message(CLASS_CFG, MSG_CFG_SET_RATE, &msg, sizeof(msg));
}

/*
 *  configure a UBlox GPS navigation solution rate of 200ms
 */
void
AP_GPS_UBLOX::_configure_navigation_rate(uint16_t rate_ms)
{
    struct ubx_cfg_nav_rate msg;
    msg.measure_rate_ms = rate_ms;
    msg.nav_rate        = 1;
    msg.timeref         = 0;     // UTC time
    _send_message(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}

/*
 *  configure a UBlox GPS for the given message rate
 */
void
AP_GPS_UBLOX::_configure_gps(void)
{
    port->begin(38400U);

    // start the process of updating the GPS rates
    need_rate_update = true;
    _last_5hz_time = hal.scheduler->millis();
    rate_update_step = 0;

    // ask for the current navigation settings
	Debug("Asking for engine setting\n");
    _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
}


/*
  detect a Ublox GPS. Adds one byte, and returns true if the stream
  matches a UBlox
 */
bool
AP_GPS_UBLOX::_detect(struct UBLOX_detect_state &state, uint8_t data)
{
reset:
	switch (state.step) {
        case 1:
            if (PREAMBLE2 == data) {
                state.step++;
                break;
            }
            state.step = 0;
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
            break;
        case 4:
            state.step++;
            state.ck_b += (state.ck_a += data);
            state.payload_length = data;
            break;
        case 5:
            state.step++;
            state.ck_b += (state.ck_a += data);
            state.payload_counter = 0;
            break;
        case 6:
            state.ck_b += (state.ck_a += data);
            if (++state.payload_counter == state.payload_length)
                state.step++;
            break;
        case 7:
            state.step++;
            if (state.ck_a != data) {
                state.step = 0;
				goto reset;
            }
            break;
        case 8:
            state.step = 0;
			if (state.ck_b == data) {
				// a valid UBlox packet
				return true;
			} else {
				goto reset;
			}
    }
    return false;
}
