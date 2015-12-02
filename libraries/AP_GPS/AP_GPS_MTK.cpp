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

#include "AP_GPS.h"
#include "AP_GPS_MTK.h"

// initialisation blobs to send to the GPS to try to get it into the
// right mode
const char AP_GPS_MTK::_initialisation_blob[] = MTK_OUTPUT_5HZ SBAS_ON WAAS_ON MTK_NAVTHRES_OFF;

AP_GPS_MTK::AP_GPS_MTK(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    _step(0),
    _payload_counter(0)
{
    gps.send_blob_start(state.instance, _initialisation_blob, sizeof(_initialisation_blob));
}

/*
  send an initialisation blob to configure the GPS
 */
void AP_GPS_MTK::send_init_blob(uint8_t instance, AP_GPS &gps)
{
    gps.send_blob_start(instance, _initialisation_blob, sizeof(_initialisation_blob));
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
                _step = 0;
            }
            break;
        case 6:
            _step = 0;
            if (_ck_b != data) {
                break;
            }

            // set fix type
            if (_buffer.msg.fix_type == FIX_3D) {
                state.status = AP_GPS::GPS_OK_FIX_3D;
            }else if (_buffer.msg.fix_type == FIX_2D) {
                state.status = AP_GPS::GPS_OK_FIX_2D;
            }else{
                state.status = AP_GPS::NO_FIX;
            }
            state.location.lat  = swap_int32(_buffer.msg.latitude)  * 10;
            state.location.lng  = swap_int32(_buffer.msg.longitude) * 10;
            state.location.alt  = swap_int32(_buffer.msg.altitude);
            state.ground_speed      = swap_int32(_buffer.msg.ground_speed) * 0.01f;
            state.ground_course_cd  = wrap_360_cd(swap_int32(_buffer.msg.ground_course) / 10000);
            state.num_sats          = _buffer.msg.satellites;

            if (state.status >= AP_GPS::GPS_OK_FIX_2D) {
                make_gps_time(0, swap_int32(_buffer.msg.utc_time)*10);
            }
            // we don't change _last_gps_time as we don't know the
            // full date

            fill_3d_velocity();

            parsed = true;
        }
    }
    return parsed;
}

/*
  detect a MTK GPS
 */
bool
AP_GPS_MTK::_detect(struct MTK_detect_state &state, uint8_t data)
{
	switch (state.step) {
        case 1:
            if (PREAMBLE2 == data) {
                state.step++;
                break;
            }
            state.step = 0;
        case 0:
			state.ck_b = state.ck_a = state.payload_counter = 0;
            if(PREAMBLE1 == data)
                state.step++;
            break;
        case 2:
            if (MESSAGE_CLASS == data) {
                state.step++;
                state.ck_b = state.ck_a = data;
            } else {
                state.step = 0;
            }
            break;
        case 3:
            if (MESSAGE_ID == data) {
                state.step++;
                state.ck_b += (state.ck_a += data);
                state.payload_counter = 0;
            } else {
                state.step = 0;
            }
            break;
        case 4:
            state.ck_b += (state.ck_a += data);
            if (++state.payload_counter == sizeof(struct diyd_mtk_msg))
                state.step++;
            break;
        case 5:
            state.step++;
            if (state.ck_a != data) {
                state.step = 0;
            }
            break;
        case 6:
            state.step = 0;
            if (state.ck_b == data) {
				return true;
            }
	}
    return false;
}
