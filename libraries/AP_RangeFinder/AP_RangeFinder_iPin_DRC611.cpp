#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_IPIN_DRC611_ENABLED
#include "AP_RangeFinder_iPin_DRC611.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define DRC611_HEADER             0xFA
#define DRC611_FRAME_LENGTH      6

#define DRC611_CMD_START_1       0xCD
#define DRC611_CMD_START_2       0x01
#define DRC611_CMD_START_3       0x0A
#define DRC611_CMD_START_4       0x0B

#define DRC611_MIN_CM             15
#define DRC611_MAX_CM           1300

bool AP_RangeFinder_iPin_DRC611::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    if (!start_cmd_sent) {
        const uint8_t start_cmd[] {
            DRC611_CMD_START_1,
            DRC611_CMD_START_2,
            DRC611_CMD_START_3,
            DRC611_CMD_START_4,
        };
        uart->write(start_cmd, sizeof(start_cmd));
        start_cmd_sent = true;
    }

    uint32_t sum_cm = 0;
    uint16_t valid_count = 0;

    // process up to 8192 bytes and average all valid readings
    for (uint16_t byte_index = 0; byte_index < 8192; byte_index++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }

        /*
         * Find frame header
         *
         * FA 00 03 ZZ YY CHECKSUM
         *
         * We receive 5 bytes here after the header:
         *
         * FA 00 03 ZZ YY
         *
         * CHECKSUM
         *
         * Total = 6 bytes
         */

        if (frame_len == 0) {
            if (c == DRC611_HEADER) {
                frame[frame_len++] = c;
            }
            continue;
        }

        frame[frame_len++] = c;

        if (frame_len < DRC611_FRAME_LENGTH) {
            continue;
        }

        /*
         * Check:
         *
         * FA 00 03 ZZ YY
         */

        if (frame[1] != 0x00 ||
            frame[2] != 0x03) {

            frame_len = 0;

            if (c == DRC611_HEADER) {
                frame[frame_len++] = c;
            }
            continue;
        }

        uint8_t calculated = 0;

        for (uint8_t i = 1; i < DRC611_FRAME_LENGTH - 1; i++) {
            calculated += frame[i];
        }

        if (calculated != frame[DRC611_FRAME_LENGTH - 1]) {
            frame_len = 0;
            continue;
        }

        /*
         * Distance is little endian:
         *
         * ZZ = low byte
         * YY = high byte
         */

        const uint16_t distance_cm = ((uint16_t)frame[4] << 8) | frame[3];

        frame_len = 0;

        /*
         * Out of range
         */

        if (distance_cm < DRC611_MIN_CM ||
            distance_cm > DRC611_MAX_CM) {
            continue;
        }

        sum_cm += distance_cm;
        valid_count++;
    }

    if (valid_count > 0) {
        reading_m = (sum_cm * 0.01f) / valid_count;
        return true;
    }

    return false;
}

#endif
