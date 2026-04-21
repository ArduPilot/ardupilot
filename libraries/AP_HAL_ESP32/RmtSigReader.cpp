#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

#ifdef HAL_ESP32_RCIN

using namespace ESP32;

void RmtSigReader::init()
{
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_RX;
    config.channel = RMT_CHANNEL_4; // On S3, Channel 0 ~ 3 (TX channel) are dedicated to sending signals. Channel 4 ~ 7 (RX channel) are dedicated to receiving signals, so this pin choice is compatible with both.
    config.clk_div = 80;   //80MHZ APB clock to the 1MHZ target frequency
    config.gpio_num = HAL_ESP32_RCIN;
    config.mem_block_num = 2; //each block could store 64 pulses
    config.flags = 0;
    config.rx_config.filter_en = true;
    config.rx_config.filter_ticks_thresh = 8;
    config.rx_config.idle_threshold = idle_threshold;

    rmt_config(&config);
    rmt_driver_install(config.channel, max_pulses * 8, 0);
    rmt_get_ringbuf_handle(config.channel, &handle);
    rmt_rx_start(config.channel, true);
}

bool RmtSigReader::add_item(uint32_t duration, bool level)
{
    bool has_more = true;
    if (duration == 0) {
        has_more = false;
        duration = idle_threshold;
    }
    if (level) {
        if (last_high == 0) {
            last_high = duration;
        }
    } else {
        if (last_high != 0) {
            ready_high = last_high;
            ready_low = duration;
            pulse_ready = true;
            last_high = 0;
        }
    }
    return has_more;
}

bool RmtSigReader::read(uint32_t &width_high, uint32_t &width_low)
{
    if (item == nullptr) {
        item = (rmt_item32_t*) xRingbufferReceive(handle, &item_size, 0);
        item_size /= 4;
        current_item = 0;
    }
    if (item == nullptr) {
        return false;
    }
    bool buffer_empty = (current_item == item_size);
    buffer_empty = buffer_empty ||
                   !add_item(item[current_item].duration0, item[current_item].level0);
    buffer_empty = buffer_empty ||
                   !add_item(item[current_item].duration1, item[current_item].level1);
    current_item++;
    if (buffer_empty) {
        vRingbufferReturnItem(handle, (void*) item);
        item = nullptr;
    }
    if (pulse_ready) {
        width_high = ready_high;
        width_low = ready_low;
        pulse_ready = false;
        return true;
    }
    return false;
}
#endif
