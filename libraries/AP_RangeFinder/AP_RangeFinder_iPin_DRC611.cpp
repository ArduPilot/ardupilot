#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_IPIN_DRC611_ENABLED
#include "AP_RangeFinder_iPin_DRC611.h"
#include <AP_HAL/AP_HAL.h>

#include <cstring>

extern const AP_HAL::HAL& hal;

#define DRC611_HEADER             0xFA
#define DRC611_FRAME_LENGTH      6
#define DRC611_MIN_CM             15
#define DRC611_MAX_CM           1300

// Search the buffered candidate for another possible header.
// Preserve the remaining bytes instead of discarding them.
void AP_RangeFinder_iPin_DRC611::resync_frame()
{
    for (uint8_t i = 1; i < frame_len; i++) {
        if (frame[i] == DRC611_HEADER) {
            const uint8_t remaining = frame_len - i;
            memmove(frame, &frame[i], remaining);
            frame_len = remaining;
            return;
        }
    }

    frame_len = 0;
}

bool AP_RangeFinder_iPin_DRC611::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint32_t now = AP_HAL::millis();

    // Retry the start command at most once per second
    // if no valid measurement has arrived recently.
    const bool no_recent_reading =
        !start_cmd_attempted ||
        (now - last_valid_ms >= 1000U);

    const bool retry_due =
        !start_cmd_attempted ||
        (now - last_start_ms >= 1000U);

    if (no_recent_reading && retry_due) {
        static const uint8_t start_cmd[] = {
            0xCD, 0x01, 0x0A, 0x0B
        };

        last_start_ms = now;
        start_cmd_attempted = true;

        // Retry after one second if no valid reading arrives.
        uart->write(start_cmd, sizeof(start_cmd));
    }

    uint32_t sum_cm = 0;
    uint16_t valid_count = 0;
	uint16_t distance_cm = 0;
	uint16_t out_of_range_cm = 0;
    bool have_out_of_range = false;

    for (uint16_t byte_index = 0; byte_index < 8192; byte_index++) {
         uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        if (frame_len == 0) {
            if (c == DRC611_HEADER || c == 0x0E) {
                frame[frame_len++] = c;
            } 
            continue;
        }

        frame[frame_len++] = c;
		
		if (frame[0] == 0x0E){
	       if  ( frame_len < 2) {
               continue;
	    	}
		    if ( frame[1] == 0x83 ){
			   out_of_range_cm = 2000;
			   have_out_of_range = true;
               continue;
		    }
		    if (frame[1] == 0x89){
			   out_of_range_cm = 2000;
			   have_out_of_range = true;
               continue;
		    }
		    if ( frame[1] == 0x8E){
			   out_of_range_cm = 0;
			   have_out_of_range = true;
               continue;
		    }
        } else if (frame[0] == DRC611_HEADER){
		           if (frame_len < DRC611_FRAME_LENGTH) {
                      continue;
                    }
                    // Validate fixed header fields.
                    if (frame[1] != 0x00 || frame[2] != 0x03) {
                       resync_frame();
                       continue;
                    }
		
                    uint8_t checksum = 0;

                    for (uint8_t i = 1;
                    i < DRC611_FRAME_LENGTH - 1;
                    i++) {
                    checksum += frame[i];
                    }
 
                    if (checksum != frame[5]) {
                       resync_frame();
                    continue;
                    }
                // ZZ is the low byte; YY is the high byte.
                distance_cm = (uint16_t(frame[4]) << 8) | frame[3];
                }
				
		if (distance_cm == 0) {
           continue;
        }

        frame_len = 0;
        sum_cm += distance_cm;
        valid_count++;
    }
	
	

    if (valid_count > 0) {
        reading_m = (sum_cm * 0.01f) / valid_count;
        last_valid_ms = AP_HAL::millis();
        return true;
    }
	if (have_out_of_range) {
        reading_m = out_of_range_cm * 0.01f;
        last_valid_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

#endif // AP_RANGEFINDER_IPIN_DRC611_ENABLED
