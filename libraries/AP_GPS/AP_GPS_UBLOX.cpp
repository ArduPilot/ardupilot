// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  u-blox UBX GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//

#define UBLOX_DEBUGGING 0

#if UBLOX_DEBUGGING
#include <FastSerial.h>
# define Debug(fmt, args...)  do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args); delay(0); } while(0)
#else
# define Debug(fmt, args...)
#endif

#include "AP_GPS_UBLOX.h"
#include <stdint.h>


// Constructors ////////////////////////////////////////////////////////////////

AP_GPS_UBLOX::AP_GPS_UBLOX(Stream *s) : GPS(s)
{
}

// Public Methods //////////////////////////////////////////////////////////////

void
AP_GPS_UBLOX::init(void)
{
    // XXX it might make sense to send some CFG_MSG,CFG_NMEA messages to get the
    // right reporting configuration.

    _port->flush();

    _epoch = TIME_OF_WEEK;
    idleTimeout = 1200;
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
    uint8_t		data;
    int 		numc;
    bool		parsed = false;

    numc = _port->available();
    for (int i = 0; i < numc; i++) {	// Process bytes received

        // read the next byte
        data = _port->read();

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
            if (CLASS_NAV == data) {
                _gather = true;					// class is interesting, maybe gather
            } else {
                _gather = false;				// class is not interesting, discard
				Debug("not interesting class=0x%x", data);
            }
			_ck_b = _ck_a = data;			// reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _msg_id = data;
            if (_gather) {						// if class was interesting
                switch(data) {
                case MSG_POSLLH:				// message is interesting
                    _expect = sizeof(ubx_nav_posllh);
                    break;
                case MSG_STATUS:
                    _expect = sizeof(ubx_nav_status);
                    break;
                case MSG_SOL:
                    _expect = sizeof(ubx_nav_solution);
                    break;
                case MSG_VELNED:
                    _expect = sizeof(ubx_nav_velned);
                    break;
                default:
                    _gather = false;			// message is not interesting
                }
            } else {
				Debug("not interesting msg_id 0x%x", _msg_id);
			}
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _payload_length = data;				// payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _payload_length += (uint16_t)data;	// payload length high byte
            _payload_counter = 0;				// prepare to receive payload
            if (_payload_length != _expect && _gather) {
				Debug("bad length _payload_length=%u _expect=%u", _payload_length, _expect);
                _gather = false;
			}
            break;

            // Receive message data
            //
        case 6:
            _ck_b += (_ck_a += data);			// checksum byte
            if (_gather)						// gather data if requested
                _buffer.bytes[_payload_counter] = data;
            if (++_payload_counter == _payload_length)
                _step++;
            break;

            // Checksum and message processing
            //
        case 7:
            _step++;
            if (_ck_a != data) {
				Debug("bad cka %x should be %x", data, _ck_a);
                _step = 0;						// bad checksum
			}
            break;
        case 8:
            _step = 0;
            if (_ck_b != data) {
				Debug("bad ckb %x should be %x", data, _ck_b);
                break;							// bad checksum
			}

            if (_gather && _parse_gps()) {
				parsed = true;
            }
        }
    }
	Debug("parsed = %u", (unsigned)parsed);
    return parsed;
}

// Private Methods /////////////////////////////////////////////////////////////

bool
AP_GPS_UBLOX::_parse_gps(void)
{
    switch (_msg_id) {
    case MSG_POSLLH:
		Debug("MSG_POSLLH next_fix=%u", next_fix);
        time		= _buffer.posllh.time;
        longitude	= _buffer.posllh.longitude;
        latitude	= _buffer.posllh.latitude;
        altitude	= _buffer.posllh.altitude_msl / 10;
		fix			= next_fix;
		_new_position = true;
		break;
    case MSG_STATUS:
		Debug("MSG_STATUS fix_status=%u fix_type=%u",
					  _buffer.status.fix_status,
					  _buffer.status.fix_type);
        next_fix	= (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
		if (!next_fix) {
			fix = false;
		}
        break;
    case MSG_SOL:
		Debug("MSG_SOL fix_status=%u fix_type=%u",
					  _buffer.solution.fix_status,
					  _buffer.solution.fix_type);
        next_fix	= (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
		if (!next_fix) {
			fix = false;
		}
        num_sats	= _buffer.solution.satellites;
        hdop		= _buffer.solution.position_DOP;
        break;
    case MSG_VELNED:
		Debug("MSG_VELNED");
        speed_3d	= _buffer.velned.speed_3d;				// cm/s
        ground_speed = _buffer.velned.speed_2d;				// cm/s
        ground_course = _buffer.velned.heading_2d / 1000;	// Heading 2D deg * 100000 rescaled to deg * 100
		_new_speed = true;
        break;
    default:
        return false;
    }

	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (_new_position && _new_speed) {
		_new_speed = _new_position = false;
		return true;
	}
	return false;
}
