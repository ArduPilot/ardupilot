#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include "AC_Lander.h"
extern const AP_HAL::HAL& hal;

#define LIGHTWARE_DIST_MAX_CM           10000
#define LIGHTWARE_OUT_OF_RANGE_ADD_CM   100


void AC_Lander::init_serial(uint8_t serial_instance)
{
    // uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    uart = hal.serial(serial_instance);
    if (uart != nullptr) {
        uart->begin(115200, rx_bufsize(), tx_bufsize());
    }
    
}

uint32_t AC_Lander::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Target_Lander, serial_instance);

}

// return distance to target in meters
bool AC_Lander::get_reading(float &reading_m)
{
    
    if (uart == nullptr)
    {
        return false;

    }

    float sum = 0;              // sum of all readings taken
    uint16_t valid_count = 0;   // number of valid readings
    uint16_t invalid_count = 0; // number of invalid readings

    // max distance the sensor can reliably measure - read from parameters
    const int16_t distance_cm_max = max_distance_cm;

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();

        // use legacy protocol
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::LEGACY) {
            if (c == '\r') {
                linebuf[linebuf_len] = 0;
                const float dist = strtof(linebuf, nullptr);
                if (!is_negative(dist) && !is_lost_signal_distance(dist * 100, distance_cm_max)) {
                    sum += dist;
                    valid_count++;
                    // if still determining protocol update legacy valid count
                    if (protocol_state == ProtocolState::UNKNOWN) {
                        legacy_valid_count++;
                    }
                } else {
                    invalid_count++;
                }
                linebuf_len = 0;
            } else if (isdigit(c) || c == '.' || c == '-') {
                linebuf[linebuf_len++] = c;
                if (linebuf_len == sizeof(linebuf)) {
                    // too long, discard the line
                    linebuf_len = 0;
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
                        sum += dist * 0.01f;
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
         now - last_reading_ms > 1000)) {
        // send enough serial transitions to trigger LW20 into serial
        // mode. It starts in dual I2C/serial mode, and wants to see
        // enough transitions to switch into serial mode.
        uart->write("www\r\n");
        last_init_ms = now;
    } else {
        uart->write('d');
    }

    // return average of all valid readings
    if (valid_count > 0) {
        reading_m = sum / valid_count;
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
bool AC_Lander::is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max)
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

// AC_Lander::AC_Lander(AC_Lander::Lander_State &_state):
//     state(_state)
// {

// }

void AC_Lander::init(uint8_t serial_instance)
{
    init_serial(serial_instance);
}