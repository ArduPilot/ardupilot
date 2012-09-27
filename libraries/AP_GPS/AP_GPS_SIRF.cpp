// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  SiRF Binary GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith.
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//

#include "AP_GPS_SIRF.h"
#include <stdint.h>

// Initialisation messages
//
// Turn off all messages except for 0x29.
//
// XXX the bytes show up on the wire, but at least my test unit (EM-411) seems to ignore them.
//
static uint8_t init_messages[] = {
    0xa0, 0xa2, 0x00, 0x08, 0xa6, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa8, 0xb0, 0xb3,
    0xa0, 0xa2, 0x00, 0x08, 0xa6, 0x00, 0x29, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xb0, 0xb3
};

// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_SIRF::AP_GPS_SIRF(AP_HAL::UARTDriver *s) : GPS(s)
{
}

// Public Methods //////////////////////////////////////////////////////////////
void
AP_GPS_SIRF::init(enum GPS_Engine_Setting nav_setting)
{
    _port->flush();

    // For modules that default to something other than SiRF binary,
    // the module-specific subclass should take care of switching to binary mode
    // before calling us.

    // send SiRF binary setup messages
    _port->write(init_messages, sizeof(init_messages));
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
AP_GPS_SIRF::read(void)
{
    uint8_t data;
    int16_t numc;
    bool parsed = false;

    numc = _port->available();
    while(numc--) {

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
        // FALLTHROUGH
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;

        // Message length
        //
        // We always collect the length so that we can avoid being
        // fooled by preamble bytes in messages.
        //
        case 2:
            _step++;
            _payload_length = (uint16_t)data << 8;
            break;
        case 3:
            _step++;
            _payload_length |= data;
            _payload_counter = 0;
            _checksum = 0;
            break;

        // Message header processing
        //
        // We sniff the message ID to determine whether we are going
        // to gather the message bytes or just discard them.
        //
        case 4:
            _step++;
            _accumulate(data);
            _payload_length--;
            _gather = false;
            switch(data) {
            case MSG_GEONAV:
                if (_payload_length == sizeof(sirf_geonav)) {
                    _gather = true;
                    _msg_id = data;
                }
                break;
            }
            break;

        // Receive message data
        //
        // Note that we are effectively guaranteed by the protocol
        // that the checksum and postamble cannot be mistaken for
        // the preamble, so if we are discarding bytes in this
        // message when the payload is done we return directly
        // to the preamble detector rather than bothering with
        // the checksum logic.
        //
        case 5:
            if (_gather) {                                              // gather data if requested
                _accumulate(data);
                _buffer.bytes[_payload_counter] = data;
                if (++_payload_counter == _payload_length)
                    _step++;
            } else {
                if (++_payload_counter == _payload_length)
                    _step = 0;
            }
            break;

        // Checksum and message processing
        //
        case 6:
            _step++;
            if ((_checksum >> 8) != data) {
                _error("GPS_SIRF: checksum error\n");
                _step = 0;
            }
            break;
        case 7:
            _step = 0;
            if ((_checksum & 0xff) != data) {
                _error("GPS_SIRF: checksum error\n");
                break;
            }
            if (_gather) {
                parsed = _parse_gps();                                   // Parse the new GPS packet
            }
        }
    }
    return(parsed);
}

bool
AP_GPS_SIRF::_parse_gps(void)
{
    switch(_msg_id) {
    case MSG_GEONAV:
        time                    = _swapl(&_buffer.nav.time);
        //fix				= (0 == _buffer.nav.fix_invalid) && (FIX_3D == (_buffer.nav.fix_type & FIX_MASK));
        fix                             = (0 == _buffer.nav.fix_invalid);
        latitude                = _swapl(&_buffer.nav.latitude);
        longitude               = _swapl(&_buffer.nav.longitude);
        altitude                = _swapl(&_buffer.nav.altitude_msl);
        ground_speed    = _swapi(&_buffer.nav.ground_speed);
        // at low speeds, ground course wanders wildly; suppress changes if we are not moving
        if (ground_speed > 50)
            ground_course       = _swapi(&_buffer.nav.ground_course);
        num_sats                = _buffer.nav.satellites;

        return true;
    }
    return false;
}

void
AP_GPS_SIRF::_accumulate(uint8_t val)
{
    _checksum = (_checksum + val) & 0x7fff;
}



/*
  detect a SIRF GPS
 */
bool
AP_GPS_SIRF::_detect(uint8_t data)
{
	static uint16_t checksum;
	static uint8_t step, payload_length, payload_counter;

	switch (step) {
	case 1:
		if (PREAMBLE2 == data) {
			step++;
			break;
		}
		step = 0;
	case 0:
		payload_length = payload_counter = checksum = 0;
		if (PREAMBLE1 == data)
			step++;
		break;
	case 2:
		step++;
		if (data != 0) {
			// only look for short messages
			step = 0;
		}
		break;
	case 3:
		step++;
		payload_length = data;
		break;
	case 4:
		checksum = (checksum + data) & 0x7fff;
		if (++payload_counter == payload_length)
			step++;
		break;
	case 5:
		step++;
		if ((checksum >> 8) != data) {
			step = 0;
		}
		break;
	case 6:
		step = 0;
		if ((checksum & 0xff) == data) {
			return true;
		}
    }
    return false;
}
