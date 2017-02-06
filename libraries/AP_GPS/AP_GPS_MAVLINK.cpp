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

#include "AP_GPS_MAVLINK.h"


AP_GPS_MAVLINK::AP_GPS_MAVLINK(AP_GPS &_gps, AP_GPS::GPS_State &_state) :
    AP_GPS_NMEA(_gps, _state, NULL),
	_new_data(false)
{ }

void AP_GPS_MAVLINK::inject_data(uint8_t *data, uint8_t len)
{
    for(int i = 0; i < len; i++) {
        char c = data[i];
#ifdef NMEA_LOG_PATH
        static FILE *logf = NULL;
        if (logf == NULL) {
            logf = fopen(NMEA_LOG_PATH, "wb");
        }
        if (logf != NULL) {
            ::fwrite(&c, 1, 1, logf);
        }
#endif
        if (_decode(c)) { // We have finished parsing a valid sentence
            _new_data = true;
        }
    }
}

bool AP_GPS_MAVLINK::read(void)
{
	if(_new_data == true) {
		_new_data = false;
		return true; // We have received valid data since last read
	} else {
		return false; // We have not received any new data
	}
}
