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
#include <stdint.h>

#include <AP_HAL.h>

#define UBLOX_DEBUGGING 0


#if UBLOX_DEBUGGING
 # define Debug(fmt, args ...)  do {Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); delay(0); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

#include "AP_GPS_UBLOX.h"

extern const AP_HAL::HAL& hal;

const prog_char AP_GPS_UBLOX::_ublox_set_binary[] PROGMEM = UBLOX_SET_BINARY;
const uint8_t AP_GPS_UBLOX::_ublox_set_binary_size = sizeof(AP_GPS_UBLOX::_ublox_set_binary);

// Public Methods //////////////////////////////////////////////////////////////

void
AP_GPS_UBLOX::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_port = s;

    // XXX it might make sense to send some CFG_MSG,CFG_NMEA messages to get the
    // right reporting configuration.

	Debug("uBlox nav_setting=%u\n", nav_setting);

    _port->flush();

    _epoch = TIME_OF_WEEK;
    idleTimeout = 1200;

    // configure the GPS for the messages we want
    _configure_gps();

    _nav_setting = nav_setting;
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

    numc = _port->available();
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        data = _port->read();

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

bool
AP_GPS_UBLOX::_parse_gps(void)
{
    if (_class == CLASS_ACK) {
        Debug("ACK %u", (unsigned)_msg_id);
        return false;
    }

    if (_class == CLASS_CFG && _msg_id == MSG_CFG_NAV_SETTINGS) {
		Debug("Got engine settings %u\n", (unsigned)_buffer.nav_settings.dynModel);
        if (_nav_setting != GPS_ENGINE_NONE &&
            _buffer.nav_settings.dynModel != _nav_setting) {
            // we've received the current nav settings, change the engine
            // settings and send them back
            Debug("Changing engine setting from %u to %u\n",
                  (unsigned)_buffer.nav_settings.dynModel, (unsigned)_nav_setting);
            _buffer.nav_settings.dynModel = _nav_setting;
            _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS,
                          &_buffer.nav_settings,
                          sizeof(_buffer.nav_settings));
        }
        return false;
    }

    if (_class != CLASS_NAV) {
        Debug("Unexpected message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
        if (++_disable_counter == 0) {
            // disable future sends of this message id, but
            // only do this every 256 messages, as some
            // message types can't be disabled and we don't
            // want to get into an ack war
            Debug("Disabling message 0x%02x 0x%02x", (unsigned)_class, (unsigned)_msg_id);
            _configure_message_rate(_class, _msg_id, 0);
        }
        return false;
    }

    switch (_msg_id) {
    case MSG_POSLLH:
        Debug("MSG_POSLLH next_fix=%u", next_fix);
        time            = _buffer.posllh.time;
        longitude       = _buffer.posllh.longitude;
        latitude        = _buffer.posllh.latitude;
        altitude        = _buffer.posllh.altitude_msl / 10;
        fix                     = next_fix;
        _new_position = true;
        break;
    case MSG_STATUS:
        Debug("MSG_STATUS fix_status=%u fix_type=%u",
              _buffer.status.fix_status,
              _buffer.status.fix_type);
        next_fix        = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
        if (!next_fix) {
            fix = false;
        }
        break;
    case MSG_SOL:
        Debug("MSG_SOL fix_status=%u fix_type=%u",
              _buffer.solution.fix_status,
              _buffer.solution.fix_type);
        next_fix        = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
        if (!next_fix) {
            fix = false;
        }
        num_sats        = _buffer.solution.satellites;
        hdop            = _buffer.solution.position_DOP;
        break;
    case MSG_VELNED:
        Debug("MSG_VELNED");
        speed_3d        = _buffer.velned.speed_3d;                              // cm/s
        ground_speed = _buffer.velned.speed_2d;                         // cm/s
        ground_course = _buffer.velned.heading_2d / 1000;       // Heading 2D deg * 100000 rescaled to deg * 100
        _have_raw_velocity = true;
        _vel_north  = _buffer.velned.ned_north;
        _vel_east   = _buffer.velned.ned_east;
        _vel_down   = _buffer.velned.ned_down;
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
    if (_new_position && _new_speed) {
        _new_speed = _new_position = false;
		_fix_count++;
		if (_fix_count == 100) {
			// ask for nav settings every 20 seconds
			Debug("Asking for engine setting\n");
			_send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
		}
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

    _port->write((const uint8_t *)&header, sizeof(header));
    _port->write((const uint8_t *)msg, size);
    _port->write((const uint8_t *)&ck_a, 1);
    _port->write((const uint8_t *)&ck_b, 1);
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
 *  configure a UBlox GPS for the given message rate
 */
void
AP_GPS_UBLOX::_configure_gps(void)
{
    struct ubx_cfg_nav_rate msg;
    const unsigned baudrates[4] = {9600U, 19200U, 38400U, 57600U};

    // the GPS may be setup for a different baud rate. This ensures
    // it gets configured correctly
    for (uint8_t i=0; i<4; i++) {
        _port->begin(baudrates[i]);
        _write_progstr_block(_port, _ublox_set_binary, _ublox_set_binary_size);
        while (_port->tx_pending()) {
          hal.scheduler->delay(1);
        }
    }
    _port->begin(38400U);

    // ask for navigation solutions every 200ms
    msg.measure_rate_ms = 200;
    msg.nav_rate        = 1;
    msg.timeref         = 0;     // UTC time
    _send_message(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));

    // ask for the messages we parse to be sent on every navigation solution
    _configure_message_rate(CLASS_NAV, MSG_POSLLH, 1);
    _configure_message_rate(CLASS_NAV, MSG_STATUS, 1);
    _configure_message_rate(CLASS_NAV, MSG_SOL, 1);
    _configure_message_rate(CLASS_NAV, MSG_VELNED, 1);

    // ask for the current navigation settings
	Debug("Asking for engine setting\n");
    _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS, NULL, 0);
}


/*
  detect a Ublox GPS. Adds one byte, and returns true if the stream
  matches a UBlox
 */
bool
AP_GPS_UBLOX::_detect(uint8_t data)
{
	static uint8_t payload_length, payload_counter;
	static uint8_t step;
	static uint8_t ck_a, ck_b;

reset:
	switch (step) {
        case 1:
            if (PREAMBLE2 == data) {
                step++;
                break;
            }
            step = 0;
        case 0:
            if (PREAMBLE1 == data)
                step++;
            break;
        case 2:
            step++;
            ck_b = ck_a = data;
            break;
        case 3:
            step++;
            ck_b += (ck_a += data);
            break;
        case 4:
            step++;
            ck_b += (ck_a += data);
            payload_length = data;
            break;
        case 5:
            step++;
            ck_b += (ck_a += data);
            payload_counter = 0;
            break;
        case 6:
            ck_b += (ck_a += data);
            if (++payload_counter == payload_length)
                step++;
            break;
        case 7:
            step++;
            if (ck_a != data) {
                step = 0;
				goto reset;
            }
            break;
        case 8:
            step = 0;
			if (ck_b == data) {
				// a valid UBlox packet
				return true;
			} else {
				goto reset;
			}
    }
    return false;
}
