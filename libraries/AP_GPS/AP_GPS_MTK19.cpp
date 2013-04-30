// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//  DIYDrones Custom Mediatek GPS driver for ArduPilot and ArduPilotMega.
//    Code by Michael Smith, Jordi Munoz and Jose Julio, Craig Elder, DIYDrones.com
//
//    This library is free software; you can redistribute it and / or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.6, v1.7, v1.8, v1.9"
//
//   Note that this driver supports both the 1.6 and 1.9 protocol varients
//

#include <AP_HAL.h>
#include "AP_GPS_MTK19.h"
#include <stdint.h>

// Public Methods //////////////////////////////////////////////////////////////
void
AP_GPS_MTK19::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_port = s;
    _port->flush();

    // initialize serial port for binary protocol use
    // XXX should assume binary, let GPS_AUTO handle dynamic config?
    _port->print(MTK_SET_BINARY);

    // set 5Hz update rate
    _port->print(MTK_OUTPUT_5HZ);

    // set SBAS on
    _port->print(SBAS_ON);

    // set WAAS on
    _port->print(WAAS_ON);

    // Set Nav Threshold to 0 m/s
    _port->print(MTK_NAVTHRES_OFF);

    // set initial epoch code
    _epoch = TIME_OF_DAY;
    _time_offset = 0;
    _offset_calculated = false;
}

// Process bytes available from the stream
//
// The stream is assumed to contain only our custom message.  If it
// contains other messages, and those messages contain the preamble bytes,
// it is possible for this code to become de-synchronised.  Without
// buffering the entire message and re-processing it from the top,
// this is unavoidable.
//
// The lack of a standard header length field makes it impossible to skip
// unrecognised messages.
//
bool
AP_GPS_MTK19::read(void)
{
    uint8_t data;
    int16_t numc;
    bool parsed = false;

    numc = _port->available();
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        data = _port->read();

restart:
        switch(_step) {

        // Message preamble, class, ID detection
        //
        // If we fail to match any of the expected bytes, we
        // reset the state machine and re-consider the failed
        // byte as the first byte of the preamble.  This
        // improves our chances of recovering from a mismatch
        // and makes it less likely that we will be fooled by
        // the preamble appearing as data in some other message.
        //
        case 0:
            if (data == PREAMBLE1_V16) {
                _mtk_revision     = MTK_GPS_REVISION_V16;
                _step++;
            } else if (data == PREAMBLE1_V19) {
                _mtk_revision     = MTK_GPS_REVISION_V19;
                _step++;
            }    
            break;
        case 1:
            if (data == PREAMBLE2) {
                _step++;
            } else {
				_step = 0;
				goto restart;
			}
			break;
        case 2:
            if (sizeof(_buffer) == data) {
                _step++;
                _ck_b = _ck_a       = data;                    // reset the checksum accumulators
                _payload_counter    = 0;
            } else {
                _step               = 0;                       // reset and wait for a message of the right class
                goto restart;
            }
            break;

        // Receive message data
        //
        case 3:
            _buffer.bytes[_payload_counter++] = data;
            _ck_b += (_ck_a += data);
            if (_payload_counter == sizeof(_buffer)) {
                _step++;
			}
            break;

        // Checksum and message processing
        //
        case 4:
            _step++;
            if (_ck_a != data) {
                _step               = 0;
				goto restart;
            }
            break;
        case 5:
            _step                   = 0;
            if (_ck_b != data) {
				goto restart;
            }

            // parse fix
            if (_buffer.msg.fix_type == FIX_3D || _buffer.msg.fix_type == FIX_3D_SBAS) {
                fix = GPS::FIX_3D;
            }else if (_buffer.msg.fix_type == FIX_2D || _buffer.msg.fix_type == FIX_2D_SBAS) {
                fix = GPS::FIX_2D;
            }else{
                fix = GPS::FIX_NONE;
            }

            if (_mtk_revision == MTK_GPS_REVISION_V16) {
                latitude            = _buffer.msg.latitude  * 10;  // V16, V17,V18 doc says *10e7 but device says otherwise
                longitude           = _buffer.msg.longitude * 10;  // V16, V17,V18 doc says *10e7 but device says otherwise
            } else {
				latitude            = _buffer.msg.latitude;
				longitude           = _buffer.msg.longitude;
			}
            altitude                = _buffer.msg.altitude;
            ground_speed            = _buffer.msg.ground_speed;
            ground_course           = _buffer.msg.ground_course;
            num_sats                = _buffer.msg.satellites;
            hdop                    = _buffer.msg.hdop;
            date                    = _buffer.msg.utc_date;

            // time from gps is UTC, but convert here to msToD
            int32_t time_utc        = _buffer.msg.utc_time;
            int32_t temp            = (time_utc/10000000);
            time_utc               -= temp*10000000;
            time                    = temp * 3600000;
            temp                    = (time_utc/100000);
            time_utc               -= temp*100000;
            time                   += temp * 60000 + time_utc;
            parsed                  = true;

#ifdef FAKE_GPS_LOCK_TIME
            if (millis() > FAKE_GPS_LOCK_TIME*1000) {
                fix                 = true;
                latitude            = -35000000UL;
                longitude           = 149000000UL;
                altitude            = 584;
            }
#endif
        }
    }
    return parsed;
}


/*
  detect a MTK16 or MTK19 GPS
 */
bool
AP_GPS_MTK19::_detect(uint8_t data)
{
    static uint8_t payload_counter;
    static uint8_t step;
    static uint8_t ck_a, ck_b;

restart:
	switch (step) {
        case 0:
            ck_b = ck_a = payload_counter = 0;
            if (data == PREAMBLE1_V16 || data == PREAMBLE1_V19) {
                step++;
            }    
            break;
        case 1:
            if (PREAMBLE2 == data) {
                step++;
            } else {
				step = 0;
				goto restart;
			}
			break;
        case 2:
            if (data == sizeof(struct diyd_mtk_msg)) {
                step++;
                ck_b = ck_a         = data;
            } else {
                step                = 0;
				goto restart;
            }
            break;
        case 3:
            ck_b += (ck_a += data);
            if (++payload_counter == sizeof(struct diyd_mtk_msg))
                step++;
            break;
        case 4:
            step++;
            if (ck_a != data) {
                step 				= 0;
				goto restart;
            }
            break;
        case 5:
            step                    = 0;
            if (ck_b != data) {
				goto restart;
			}
			return true;
	}
    return false;
}
