#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TFA1500_ENABLED

#include "AP_RangeFinder_Benewake_TFA1500.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL &hal;

static constexpr uint8_t TFA1500_FRAME_HEADER = 0x5CU;
static constexpr uint8_t TFA1500_FRAME_LENGTH = 5U;
static constexpr uint32_t TFA1500_DIST_MAX_CM = 130000U;

static const uint8_t TFA1500_CMD_START[] = {
    0x55U, 0xAAU, 0xCBU, 0xCCU, 0xCCU, 0xCCU, 0xCCU, 0xFBU};

bool AP_RangeFinder_Benewake_TFA1500::process_byte(uint8_t received_byte, uint32_t &dist_cm)
{
    tf_frame.bytes[tf_frame_len++] = received_byte;

    if (tf_frame_len < TFA1500_FRAME_LENGTH)
    {
        return false;
    }

    if (tf_frame.packet.header == TFA1500_FRAME_HEADER)
    {
        const uint8_t expected_crc = (uint8_t)~(tf_frame.packet.dist_low +
                                                tf_frame.packet.dist_mid +
                                                tf_frame.packet.dist_high);
        if (expected_crc == tf_frame.packet.crc_sum_of_bytes)
        {
            dist_cm = (tf_frame.packet.dist_high << 16) |
                      (tf_frame.packet.dist_mid << 8) |
                      tf_frame.packet.dist_low;
            tf_frame_len = 0;
            return true;
        }
    }

    // shift buffer until a header byte is found or buffer is empty
    uint8_t i;
    for (i = 1; i < tf_frame_len; i++)
    {
        if (tf_frame.bytes[i] == TFA1500_FRAME_HEADER)
        {
            break;
        }
    }
    if (i < tf_frame_len)
    {
        memmove(&tf_frame.bytes[0], &tf_frame.bytes[i], tf_frame_len - i);
        tf_frame_len -= i;
    }
    else
    {
        tf_frame_len = 0;
    }

    return false;
}
bool AP_RangeFinder_Benewake_TFA1500::get_reading(float &reading_m)
{
    if (uart == nullptr)
    {
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    if (last_init_ms == 0 ||
        ((now - last_init_ms > 1000) && (now - state.last_reading_ms > 1000)))
    {
        last_init_ms = now;
        uart->write(TFA1500_CMD_START, sizeof(TFA1500_CMD_START));
    }

    float sum_cm = 0.0f;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // read limit to prevent consuming too much CPU
    for (auto j = 0U; j < 8192; j++)
    {
        uint8_t received_byte;
        if (!uart->read(received_byte))
        {
            break;
        }

        uint32_t dist_cm = 0U;

        if (process_byte(received_byte, dist_cm))
        {
            if ((dist_cm >= TFA1500_DIST_MAX_CM) || (dist_cm == (uint32_t)model_dist_max_cm()))
            {
                count_out_of_range++;
            }
            else
            {
                sum_cm += dist_cm;
                count++;
            }
        }
    }

    if (count > 0)
    {
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }

    if (count_out_of_range > 0)
    {
        reading_m = MAX(model_dist_max_cm() * 0.01f, max_distance());
        return true;
    }

    return false;
}

#endif