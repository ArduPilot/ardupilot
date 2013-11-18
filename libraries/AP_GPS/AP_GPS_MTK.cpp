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
//  DIYDrones Custom Mediatek GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
//
//	GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.1"
//

#include <stdint.h>

#include <AP_HAL.h>

#include "AP_GPS_MTK.h"

// Public Methods //////////////////////////////////////////////////////////////
void
AP_GPS_MTK::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_port = s;
    _port->flush();
	_step = 0;

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
AP_GPS_MTK::read(void)
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
            if(PREAMBLE1 == data)
                _step++;
            break;
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            goto restart;
        case 2:
            if (MESSAGE_CLASS == data) {
                _step++;
                _ck_b = _ck_a = data;                                   // reset the checksum accumulators
            } else {
                _step = 0;                                                      // reset and wait for a message of the right class
                goto restart;
            }
            break;
        case 3:
            if (MESSAGE_ID == data) {
                _step++;
                _ck_b += (_ck_a += data);
                _payload_counter = 0;
            } else {
                _step = 0;
                goto restart;
            }
            break;

        // Receive message data
        //
        case 4:
            _buffer.bytes[_payload_counter++] = data;
            _ck_b += (_ck_a += data);
            if (_payload_counter == sizeof(_buffer))
                _step++;
            break;

        // Checksum and message processing
        //
        case 5:
            _step++;
            if (_ck_a != data) {
                _error("GPS_MTK: checksum error\n");
                _step = 0;
            }
            break;
        case 6:
            _step = 0;
            if (_ck_b != data) {
                _error("GPS_MTK: checksum error\n");
                break;
            }

            // set fix type
            if (_buffer.msg.fix_type == FIX_3D) {
                fix = GPS::FIX_3D;
            }else if (_buffer.msg.fix_type == FIX_2D) {
                fix = GPS::FIX_2D;
            }else{
                fix = GPS::FIX_NONE;
            }
            latitude            = _swapl(&_buffer.msg.latitude)  * 10;
            longitude           = _swapl(&_buffer.msg.longitude) * 10;
            altitude_cm         = _swapl(&_buffer.msg.altitude);
            ground_speed_cm     = _swapl(&_buffer.msg.ground_speed);
            ground_course_cd    = _swapl(&_buffer.msg.ground_course) / 10000;
            num_sats            = _buffer.msg.satellites;

            if (fix >= GPS::FIX_2D) {
                _make_gps_time(0, _swapl(&_buffer.msg.utc_time)*10);
            }
            // we don't change _last_gps_time as we don't know the
            // full date

            parsed = true;
        }
    }
    return parsed;
}

/*
  detect a MTK GPS
 */
bool
AP_GPS_MTK::_detect(uint8_t data)
{
	static uint8_t payload_counter;
	static uint8_t step;
	static uint8_t ck_a, ck_b;

	switch(step) {
        case 1:
            if (PREAMBLE2 == data) {
                step++;
                break;
            }
            step = 0;
        case 0:
			ck_b = ck_a = payload_counter = 0;
            if(PREAMBLE1 == data)
                step++;
            break;
        case 2:
            if (MESSAGE_CLASS == data) {
                step++;
                ck_b = ck_a = data;
            } else {
                step = 0;
            }
            break;
        case 3:
            if (MESSAGE_ID == data) {
                step++;
                ck_b += (ck_a += data);
                payload_counter = 0;
            } else {
                step = 0;
            }
            break;
        case 4:
            ck_b += (ck_a += data);
            if (++payload_counter == sizeof(struct diyd_mtk_msg))
                step++;
            break;
        case 5:
            step++;
            if (ck_a != data) {
                step = 0;
            }
            break;
        case 6:
            step = 0;
            if (ck_b == data) {
				return true;
            }
	}
    return false;
}
