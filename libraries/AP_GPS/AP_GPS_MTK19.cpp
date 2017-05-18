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
//    Code by Michael Smith, Jordi Munoz and Jose Julio, Craig Elder, DIYDrones.com
//
//    GPS configuration : Custom protocol per "DIYDrones Custom Binary Sentence Specification V1.6, v1.7, v1.8, v1.9"
//
//   Note that this driver supports both the 1.6 and 1.9 protocol varients
//

#include "AP_GPS_MTK.h"
#include "AP_GPS_MTK19.h"

extern const AP_HAL::HAL& hal;

AP_GPS_MTK19::AP_GPS_MTK19(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _step(0),
    _payload_counter(0),
    _mtk_revision(0),
    _fix_counter(0)
{
    AP_GPS_MTK::send_init_blob(_state.instance, _gps);
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

    numc = port->available();
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received

        // read the next byte
        data = port->read();

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
            _buffer[_payload_counter++] = data;
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
                state.status = AP_GPS::GPS_OK_FIX_3D;
            }else if (_buffer.msg.fix_type == FIX_2D || _buffer.msg.fix_type == FIX_2D_SBAS) {
                state.status = AP_GPS::GPS_OK_FIX_2D;
            }else{
                state.status = AP_GPS::NO_FIX;
            }

            if (_mtk_revision == MTK_GPS_REVISION_V16) {
                state.location.lat  = _buffer.msg.latitude  * 10;  // V16, V17,V18 doc says *10e7 but device says otherwise
                state.location.lng  = _buffer.msg.longitude * 10;  // V16, V17,V18 doc says *10e7 but device says otherwise
            } else {
				state.location.lat  = _buffer.msg.latitude;
				state.location.lng  = _buffer.msg.longitude;
			}
            state.location.alt      = _buffer.msg.altitude;
            state.ground_speed      = _buffer.msg.ground_speed*0.01f;
            state.ground_course     = wrap_360(_buffer.msg.ground_course*0.01f);
            state.num_sats          = _buffer.msg.satellites;
            state.hdop              = _buffer.msg.hdop;
            
            if (state.status >= AP_GPS::GPS_OK_FIX_2D) {
                if (_fix_counter == 0) {
                    uint32_t bcd_time_ms;
                    bcd_time_ms = _buffer.msg.utc_time;
#if 0
                    hal.console->printf("utc_date=%lu utc_time=%lu rev=%u\n", 
                                        (unsigned long)_buffer.msg.utc_date,
                                        (unsigned long)_buffer.msg.utc_time,
                                        (unsigned)_mtk_revision);                                        
#endif
                    make_gps_time(_buffer.msg.utc_date, bcd_time_ms);
                    state.last_gps_time_ms = AP_HAL::millis();
                }
                // the _fix_counter is to reduce the cost of the GPS
                // BCD time conversion by only doing it every 10s
                // between those times we use the HAL system clock as
                // an offset from the last fix
                _fix_counter++;
                if (_fix_counter == 50) {
                    _fix_counter = 0;
                }
            }

            fill_3d_velocity();

            parsed                  = true;
        }
    }
    return parsed;
}


/*
  detect a MTK16 or MTK19 GPS
 */
bool
AP_GPS_MTK19::_detect(struct MTK19_detect_state &state, uint8_t data)
{
restart:
	switch (state.step) {
        case 0:
            state.ck_b = state.ck_a = state.payload_counter = 0;
            if (data == PREAMBLE1_V16 || data == PREAMBLE1_V19) {
                state.step++;
            }    
            break;
        case 1:
            if (PREAMBLE2 == data) {
                state.step++;
            } else {
				state.step = 0;
				goto restart;
			}
			break;
        case 2:
            if (data == sizeof(struct diyd_mtk_msg)) {
                state.step++;
                state.ck_b = state.ck_a = data;
            } else {
                state.step = 0;
				goto restart;
            }
            break;
        case 3:
            state.ck_b += (state.ck_a += data);
            if (++state.payload_counter == sizeof(struct diyd_mtk_msg))
                state.step++;
            break;
        case 4:
            state.step++;
            if (state.ck_a != data) {
                state.step = 0;
				goto restart;
            }
            break;
        case 5:
            state.step = 0;
            if (state.ck_b != data) {
				goto restart;
			}
			return true;
	}
    return false;
}
