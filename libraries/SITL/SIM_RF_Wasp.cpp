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
/*
  Simulator for the Wasp Serial rangefinder
*/

#include "SIM_RF_Wasp.h"

#include <GCS_MAVLink/GCS.h>

#include <stdio.h>
#include <string.h>

using namespace SITL;

void RF_Wasp::check_configuration()
{
    const ssize_t n = read_from_autopilot(&_buffer[_buflen], ARRAY_SIZE(_buffer) - _buflen - 1);
    if (n <= 0) {
        return;
    }
    _buflen += n;

    // ensure we have an entire line:
    const char *cr = strchr(_buffer, '\n');
    if (cr == nullptr) {
        if (_buflen == ARRAY_SIZE(_buffer) - 1) {
            // nuke it all
            memset(_buffer, '\0', ARRAY_SIZE(_buffer));
            _buflen = 0;
        }
        return;
    }
    if (!strncmp(_buffer, ">GO\n", _buflen)) {
        config.go = true;
        const char *response = "GO\n";
        write_to_autopilot(response, strlen(response));
    } else if (_buffer[0] == '>') {
        bool set = false;
        if (!set) {
            // check for string settings
            for (uint8_t i=0; i<ARRAY_SIZE(string_configs); i++) {
                if (!strncmp(&_buffer[1], string_configs[i].name, strlen(string_configs[i].name))) {
                    uint8_t offs = strlen(string_configs[i].name);
                    offs += 1; // for '>'
                    offs += 1; // for space
                    strncpy(string_configs[i].value, &_buffer[offs], MIN(ARRAY_SIZE(config.format), unsigned(cr - _buffer - offs - 1))); // -1 for the lf, -1 for the cr
//                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Wasp: config (%s) (%s)", string_configs[i].name, string_configs[i].value);
                    char response[128];
                    const size_t x = snprintf(response,
                                              ARRAY_SIZE(response),
                                              "%s %s\n",
                                              string_configs[i].name,
                                              string_configs[i].value);
                    write_to_autopilot(response, x);
                    set = true;
                    break;
                }
            }
        }
        if (!set) {
            // check for integer settings
            for (uint8_t i=0; i<ARRAY_SIZE(integer_configs); i++) {
                if (!strncmp(&_buffer[1], integer_configs[i].name, strlen(integer_configs[i].name))) {
                    uint8_t offs = strlen(integer_configs[i].name);
                    offs += 1; // for '>'
                    offs += 1; // for space
                    char tmp[32]{};
                    strncpy(tmp, &_buffer[offs], MIN(ARRAY_SIZE(config.format), unsigned(cr - _buffer - offs - 1))); // -1 for the lf, -1 for the cr
                    *(integer_configs[i].value) = atoi(tmp);
//                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Wasp: config (%s) (%d)", integer_configs[i].name, *(integer_configs[i].value));
                    char response[128];
                    const size_t x = snprintf(response,
                                              ARRAY_SIZE(response),
                                              "%s %d\n",
                                              integer_configs[i].name,
                                              *(integer_configs[i].value));
                    write_to_autopilot(response, x);
                    set = true;
                    break;
                }
            }
        }
        if (!set) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Wasp: unknown setting (%s)", &_buffer[0]);
        }
    }

    // just nuke everything in the buffer, not just what we just
    // processed.  This is until we sort out the extra-cr thing
    memset(_buffer, '\0', ARRAY_SIZE(_buffer));
    _buflen = 0;
}

void RF_Wasp::update(float range)
{
    check_configuration();
    return SerialRangeFinder::update(range);
}


uint32_t RF_Wasp::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    // the Wasp driver does not consider 0 a valid reading, so you end
    // up getting 0 samples back if you send exactly zero all the
    // time.  So munge it a little bit:
    if (alt_m < 0.01) {
        alt_m = 0.01;
    }
    return snprintf((char*)buffer, buflen, "%f\n", alt_m);
}
