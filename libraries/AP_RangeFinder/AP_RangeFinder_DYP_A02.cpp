/*
   Driver for Dianyingpu DYP-A02-V2.0 UART automatic output ultrasonic rangefinder
 */
#include "AP_RangeFinder_DYP_A02.h"

#if AP_RANGEFINDER_DYP_A02_ENABLED

#define DYP_A02_FRAME_HEADER 0xFF
#define DYP_A02_FRAME_LEN    4

bool AP_RangeFinder_DYP_A02::distance_outside_safe_range_cm(uint16_t distance_cm)
{
    return distance_cm < DYP_A02_SAFE_MIN_CM || distance_cm > DYP_A02_SAFE_MAX_CM;
}

bool AP_RangeFinder_DYP_A02::has_safety_fault() const
{
    if (status() != RangeFinder::Status::Good) {
        return true;
    }
    return distance_outside_safe_range_cm(distance_cm());
}

bool AP_RangeFinder_DYP_A02::find_frame_header(uint8_t start)
{
    for (uint8_t i = start; i < buffer_used; i++) {
        if (buffer[i] == DYP_A02_FRAME_HEADER) {
            if (i > 0) {
                memmove(&buffer[0], &buffer[i], buffer_used - i);
                buffer_used -= i;
            }
            return true;
        }
    }
    buffer_used = 0;
    return false;
}

bool AP_RangeFinder_DYP_A02::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint8_t num_read = uart->read(&buffer[buffer_used], ARRAY_SIZE(buffer) - buffer_used);
    buffer_used += num_read;

    if (buffer_used == 0) {
        return false;
    }

    if (buffer[0] != DYP_A02_FRAME_HEADER && !find_frame_header(1)) {
        return false;
    }

    if (buffer_used < DYP_A02_FRAME_LEN) {
        return false;
    }

    const uint8_t sum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;
    if (buffer[3] != sum) {
        find_frame_header(1);
        return false;
    }

    const uint16_t distance_mm = (buffer[1] << 8) | buffer[2];
    reading_m = distance_mm * 0.001f;
    buffer_used = 0;

    return true;
}

#endif  // AP_RANGEFINDER_DYP_A02_ENABLED
