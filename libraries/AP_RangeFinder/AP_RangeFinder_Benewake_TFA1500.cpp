#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TFA1500_ENABLED

#include "AP_RangeFinder_Benewake_TFA1500.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL &hal;

static constexpr uint8_t  TFA1500_FRAME_HEADER     = 0x5CU;
static constexpr uint8_t  TFA1500_FRAME_LENGTH     = 5U;
static constexpr uint32_t TFA1500_DIST_MAX_CM      = 130000U;

static const uint8_t TFA1500_CMD_START[] = {
    0x55U, 0xAAU, 0xCBU, 0xCCU, 0xCCU, 0xCCU, 0xCCU, 0xFBU
};

static bool parse_frame(const uint8_t *buffer, uint32_t &dist_cm)
{
    union {
        uint8_t bytes[TFA1500_FRAME_LENGTH];
        struct PACKED {
            uint8_t header;
            uint8_t dist_low;
            uint8_t dist_mid;
            uint8_t dist_high;
            uint8_t crc_sum_of_bytes;
        } packet;
    } frame;

    memcpy(frame.bytes, buffer, sizeof(frame.bytes));

    if (frame.packet.header != TFA1500_FRAME_HEADER) {
        return false;
    }

    const uint8_t expected_crc_sum_of_bytes = ~(frame.packet.dist_low +
                                        frame.packet.dist_mid +
                                        frame.packet.dist_high);

    if (expected_crc_sum_of_bytes != frame.packet.crc_sum_of_bytes) {
        return false;
    }

    dist_cm = (frame.packet.dist_high << 16) |
              (frame.packet.dist_mid << 8) |
              frame.packet.dist_low;
    return true;
}

void AP_RangeFinder_Benewake_TFA1500::init_serial(uint8_t serial_instance)
{
    AP_RangeFinder_Backend_Serial::init_serial(serial_instance);
}

bool AP_RangeFinder_Benewake_TFA1500::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    if (last_init_ms == 0 ||
        ((now - last_init_ms > 1000) && (now - state.last_reading_ms > 1000))) {
        last_init_ms = now;
        uart->write(TFA1500_CMD_START, sizeof(TFA1500_CMD_START));
    }

    float sum_cm = 0.0f;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // read limit to prevent consuming too much CPU
    for (auto j = 0U; j < 8192; j++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }

        tf_linebuf[tf_linebuf_len++] = c;

        if (tf_linebuf_len < TFA1500_FRAME_LENGTH) {
            continue;
        }

        uint32_t dist_cm = 0U;
        if (parse_frame(tf_linebuf, dist_cm)) {

            if ((dist_cm >= TFA1500_DIST_MAX_CM) || (dist_cm == (uint32_t)model_dist_max_cm())) {
                count_out_of_range++;
            } else {
                sum_cm += dist_cm;
                count++;
            }

            tf_linebuf_len = 0;

        } else {
            
            // shift buffer until a header byte is found or buffer is empty
            uint8_t i;
            for (i = 1; i < tf_linebuf_len; i++) {
                if (tf_linebuf[i] == TFA1500_FRAME_HEADER) {
                    break;
                }
            }
            if (i < tf_linebuf_len) {
                memmove(&tf_linebuf[0], &tf_linebuf[i], tf_linebuf_len - i);
                tf_linebuf_len -= i;
            } else {
                tf_linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }

    if (count_out_of_range > 0) {
        reading_m = MAX(model_dist_max_cm() * 0.01f, max_distance());
        return true;
    }

    return false;
}

#endif
