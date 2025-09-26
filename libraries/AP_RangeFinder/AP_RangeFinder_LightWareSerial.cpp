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

#include "AP_RangeFinder_LightWareSerial.h"

#if AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DIST_MAX_CM           10000
#define LIGHTWARE_OUT_OF_RANGE_ADD_CM   100

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareSerial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_auto = 0;           // sum of integrated mode if (VAL > ground clearance) ? ldf : ldl
    float sum_ldf = 0;          // sum of ldf readings taken
    float sum_ldl = 0;          // sum of ldl readings taken
    uint16_t valid_count = 0;   // number of valid readings
    uint16_t valid_count_ldf = 0; // number of valid ldf readings
    uint16_t valid_count_ldl = 0; // number of valid ldl readings
    uint16_t invalid_count = 0; // number of invalid readings
    float ldf_reading_m = 0;
    float ldl_reading_m = 0;
    float auto_reading_m = 0;

    // max distance the sensor can reliably measure - read from parameters
    const int16_t distance_cm_max = max_distance_cm();
    const int16_t distance_lpf_min_cm = ground_clearance_cm();

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();

        // use legacy protocol
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::LEGACY) {
            if (c == '\r') {
                linebuf[linebuf_len] = 0;
                float dist = 0;
                int8_t lidar_reply_type = get_distance_from_lidar_reply(linebuf, dist);
                if (lidar_reply_type == 0) {
                    // invalid reading
                    invalid_count++;
                    // reset the buffer length and clear the buffer
                    linebuf_len = 0;
                    memset(linebuf, 0, sizeof(linebuf));
                    continue;
                }
                else if (!is_negative(dist) && !is_lost_signal_distance(dist * 100, distance_cm_max)) {
                    if (lidar_reply_type == 1) { 
                        sum_ldf += ldf_val_m; // summing LDF value
                        valid_count_ldf++;
                    } else if (lidar_reply_type == 2) {
                        sum_ldl += ldl_val_m; // summing LDL value
                        valid_count_ldl++;
                    }
                    // overall sum and count
                    if (lidar_reply_type == 1  && ldf_val_m < (distance_lpf_min_cm*0.01f) && ldl_val_m > 0) {
                        sum_auto += float(ldl_val_m); // use the stored ldl reading if available
                        valid_count++;
                    } else if(lidar_reply_type == 1) {
                        sum_auto += dist;
                        valid_count++;
                    }

                    // if still determining protocol update legacy valid count
                    if (protocol_state == ProtocolState::UNKNOWN) {
                        legacy_valid_count++;
                    }
                } else {
                    invalid_count++;
                }
                linebuf_len = 0;
                memset(linebuf, 0, sizeof(linebuf));
            }else if(isdigit(c) || c == '.' || c == '-' || c == 'l' || c == 'd' || c == 'f' || c == ',' || c == ':'){
                linebuf[linebuf_len++] = c;
                if (linebuf_len == sizeof(linebuf)) {
                    // too long, discard the line
                    linebuf_len = 0;
                    memset(linebuf, 0, sizeof(linebuf));
                }
            }
        }

        // use binary protocol
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::BINARY) {
            bool msb_set = BIT_IS_SET(c, 7);
            if (msb_set) {
                // received the high byte
                high_byte = c;
                high_byte_received = true;
            } else {
                // received the low byte which should be second
                if (high_byte_received) {
                    const int16_t dist = (high_byte & 0x7f) << 7 | (c & 0x7f);
                    if (dist >= 0 && !is_lost_signal_distance(dist, distance_cm_max)) {
                        sum_auto += dist * 0.01f;
                        valid_count++;
                        // if still determining protocol update binary valid count
                        if (protocol_state == ProtocolState::UNKNOWN) {
                            binary_valid_count++;
                        }
                    } else {
                        invalid_count++;
                    }
                }
                high_byte_received = false;
            }
        }
    }

    // protocol set after 10 successful reads
    if (protocol_state == ProtocolState::UNKNOWN) {
        if (binary_valid_count > 10) {
            protocol_state = ProtocolState::BINARY;
        } else if (legacy_valid_count > 10) {
            protocol_state = ProtocolState::LEGACY;
        }
    }

    uint32_t now = AP_HAL::millis();
    if (last_init_ms == 0 ||
        (now - last_init_ms > 1000 &&
         now - state.last_reading_ms > 1000)) {
        // send enough serial transitions to trigger LW20 into serial
        // mode. It starts in dual I2C/serial mode, and wants to see
        // enough transitions to switch into serial mode.
        uart->write("www\r\n");
        last_init_ms = now;
    } else {
        // Sending LDL before LDF is required
        uart->write("?LDL,2\r\n");
        uart->write("?LDF,1\r\n");
    }

    // return average of all valid readings
    if (valid_count > 0) {
        auto_reading_m = sum_auto / valid_count;
        // log the data
        if (valid_count_ldf > 0){
            ldf_reading_m = sum_ldf / valid_count_ldf;
        }
        if (valid_count_ldl > 0){
            ldl_reading_m = sum_ldl / valid_count_ldl;
        }
        // chose the reading based on LW20MODE param
        if(lw20_distance_mode() == 1)
            reading_m = ldf_reading_m;
        else if (lw20_distance_mode() == 2)
            reading_m = ldl_reading_m;
        else
            reading_m = auto_reading_m;
#if HAL_LOGGING_ENABLED
        Log_LW20_C(
            ldf_reading_m,
            ldl_reading_m,
            auto_reading_m);
#endif
        no_signal = false;
        return true;
    }

    // all readings were invalid so return out-of-range-high value
    if (invalid_count > 0) {
        reading_m = MIN(MAX(LIGHTWARE_DIST_MAX_CM, distance_cm_max + LIGHTWARE_OUT_OF_RANGE_ADD_CM), UINT16_MAX) * 0.01f;
        no_signal = true;
        return true;
    }

    // no readings so return false
    return false;
}

// check to see if distance returned by the LiDAR is a known lost-signal distance flag
bool AP_RangeFinder_LightWareSerial::is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max)
{
    if (distance_cm < distance_cm_max + LIGHTWARE_OUT_OF_RANGE_ADD_CM) {
        // in-range
        return false;
    }
    const int16_t bad_distances[] { 13000, 16000, 23000, 25000 };
    for (const auto bad_distance_cm : bad_distances) {
        if (distance_cm == bad_distance_cm) {
            return true;
        }
    }
    return false;
}

/*
This is a function to extract the distance reported by the LW20 for commands ldf and ldl
Return: 
0 if the reply is not valid
1 if the reply is for ldf 
2 if the reply is for ldl
*/
int8_t AP_RangeFinder_LightWareSerial::get_distance_from_lidar_reply(char reply[], float &distance_m)
{
    // Parse the data stream format ldl,2:0.55 or ldf,1:0.45
    int8_t channel = 0; // 1 for ldf and 2 for ldl

    
    char* token = strtok(reply, ",:");

    if (token == nullptr) {
        return channel;
    }

    token = strtok(nullptr, ",:");
    if (token == nullptr) {
        return channel;
    }
    channel = atoi(token);
    if (channel == 0) {
        return channel;
    }

    token = strtok(nullptr, ",:"); // get the part after the colon
    if (token == nullptr) {
        channel = channel;
    }

    char *distance_str = token;
    distance_m = strtof(distance_str, nullptr); // Convert to meters

    if (channel == 1) {
        ldf_val_m = distance_m; // Store the first reading in m
    } else if (channel == 2) {
        ldl_val_m = distance_m; // Store the last reading in m
    } else {
        channel = 0; // Invalid channel
    }
    return channel;
}

void AP_RangeFinder_LightWareSerial::Log_LW20_C(
    float ldf_m, float ldl_m, float integrated_m)
{
    const struct log_LW20 pkt = {
        LOG_PACKET_HEADER_INIT(LOG_LW20_MSG),
        time_us : AP_HAL::micros64(),
        dist_ldf_m : ldf_m,
        dist_ldl_m : ldl_m,
        dist_int_m : integrated_m,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

#endif
