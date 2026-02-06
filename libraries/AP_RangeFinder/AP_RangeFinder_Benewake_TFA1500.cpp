#include "AP_RangeFinder_Benewake_TFA1500.h"
#if AP_RANGEFINDER_BENEWAKE_TFA1500_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL &hal;

static constexpr uint8_t  TFA1500_FRAME_HEADER     = 0x5CU;
static constexpr uint8_t  TFA1500_FRAME_LENGTH     = 5U;
static constexpr uint32_t TFA1500_DIST_MAX_CM      = 130000U;
static constexpr uint16_t TFA1500_READ_LIMIT       = 8192U;
static constexpr uint16_t TFA1500_STARTUP_DELAY_MS = 100U;

static const uint8_t TFA1500_CMD_START[] = {
    0x55U, 0xAAU, 0xCBU, 0xCCU, 0xCCU, 0xCCU, 0xCCU, 0xFBU
};
static const uint8_t TFA1500_CMD_STOP[] = {
    0x55U, 0xAAU, 0xCCU, 0xCCU, 0xCCU, 0xCCU, 0xCCU, 0xFCU
};

static bool parse_frame(const uint8_t *frame, uint32_t &dist_cm)
{
    if (frame[0] != TFA1500_FRAME_HEADER) {
        return false;
    }

    uint8_t checksum = 0U;
    for (uint8_t i = 1U; i < (TFA1500_FRAME_LENGTH - 1U); i++)
    {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    checksum = static_cast<uint8_t>(~checksum);
    if (checksum != frame[TFA1500_FRAME_LENGTH - 1U])
    {
        return false;
    }

    dist_cm = (static_cast<uint32_t>(frame[3]) << 16) |
              (static_cast<uint32_t>(frame[2]) << 8) |
              static_cast<uint32_t>(frame[1]);
    return true;
}

void AP_RangeFinder_Benewake_TFA1500::init_serial(uint8_t serial_instance)
{
    AP_RangeFinder_Backend_Serial::init_serial(serial_instance);
    // Give the sensor time to start up.
    hal.scheduler->delay(TFA1500_STARTUP_DELAY_MS);
    if (uart == nullptr)
    {
        return;
    }
    uart->write(TFA1500_CMD_START, sizeof(TFA1500_CMD_START));
}
AP_RangeFinder_Benewake_TFA1500::~AP_RangeFinder_Benewake_TFA1500()
{
    if (uart != nullptr)
    {
        uart->write(TFA1500_CMD_STOP, sizeof(TFA1500_CMD_STOP));
    }
}

bool AP_RangeFinder_Benewake_TFA1500::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0.0f;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // 读取限制，防止占用太多 CPU
    for (uint16_t j = 0U; j < TFA1500_READ_LIMIT; j++) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }

        if (tf_linebuf_len == 0) {
            if (c == TFA1500_FRAME_HEADER) {
                tf_linebuf[tf_linebuf_len++] = c;
            }
            continue;
        }

        tf_linebuf[tf_linebuf_len++] = c;

        if (tf_linebuf_len < TFA1500_FRAME_LENGTH) {
            continue;
        }

        uint32_t dist_cm = 0U;
        if (parse_frame(tf_linebuf, dist_cm)) {

            if ((dist_cm >= TFA1500_DIST_MAX_CM) || (dist_cm == static_cast<uint32_t>(model_dist_max_cm()))) {
                count_out_of_range++;
            } else {
                sum_cm += static_cast<float>(dist_cm);
                count++;
            }

            tf_linebuf_len = 0;

        } else {
            
            for (uint8_t i = 0; i < (TFA1500_FRAME_LENGTH - 1); i++) {
                tf_linebuf[i] = tf_linebuf[i + 1];
            }

            tf_linebuf_len--; 
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