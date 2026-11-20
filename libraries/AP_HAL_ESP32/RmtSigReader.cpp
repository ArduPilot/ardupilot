#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

#ifdef HAL_ESP32_RCIN

using namespace ESP32;

extern const AP_HAL::HAL& hal;

void RmtSigReader::init()
{
    rx_chan = nullptr;
    item = nullptr;
    last_high = 0;
    pulse_ready = false;
    
    rmt_rx_channel_config_t config = {};
    config.clk_src = RMT_CLK_SRC_DEFAULT;
    config.resolution_hz = frequency; // 1MHZ target frequency
    config.mem_block_symbols = max_pulses; // each block stores 64 symbols on ESP32, 48 on S3 (IDF allocates needed blocks)
    config.gpio_num = (gpio_num_t)HAL_ESP32_RCIN;

    esp_err_t err = rmt_new_rx_channel(&config, &rx_chan);
    if (err != ESP_OK) {
        hal.console->printf("RmtSigReader: rmt_new_rx_channel failed: %s\n", esp_err_to_name(err));
        return;
    }

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = rx_done_cb;
    err = rmt_rx_register_event_callbacks(rx_chan, &cbs, this);
    if (err != ESP_OK) {
        hal.console->printf("RmtSigReader: rmt_register_event_callbacks failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = rmt_enable(rx_chan);
    if (err != ESP_OK) {
        hal.console->printf("RmtSigReader: rmt_enable failed: %s\n", esp_err_to_name(err));
        return;
    }

    rx_buf_active = 0;
    rx_buf_ready = -1;

    rx_config = {};
    rx_config.signal_range_min_ns = 100; // 8 ticks at 80MHz APB clock = 100ns glitch filter
    rx_config.signal_range_max_ns = idle_threshold * 1000;

    err = rmt_receive(rx_chan, rx_buf[rx_buf_active], sizeof(rx_buf[0]), &rx_config);
    if (err != ESP_OK) {
        hal.console->printf("RmtSigReader: rmt_receive failed: %s\n", esp_err_to_name(err));
        return;
    }
}

void RmtSigReader::disable()
{
    if (rx_chan != nullptr) {
        rmt_disable(rx_chan);
    }
}

bool IRAM_ATTR RmtSigReader::rx_done_cb(rmt_channel_handle_t rx_chan,
                                        const rmt_rx_done_event_data_t *edata,
                                        void *user_ctx)
{
    RmtSigReader *reader = (RmtSigReader *)user_ctx;

    if (edata->num_symbols > 0) {
        int8_t done_buf = reader->rx_buf_active;
        reader->rx_count[done_buf] = edata->num_symbols;
        reader->rx_buf_ready = done_buf;
        reader->rx_buf_active = 1 - done_buf;
    }

    rmt_receive(rx_chan, reader->rx_buf[reader->rx_buf_active], sizeof(reader->rx_buf[0]), &reader->rx_config);
    return false;
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
        int8_t ready = rx_buf_ready;
        if (ready < 0) {
            return false;
        }
        rx_buf_ready = -1;
        item = rx_buf[ready];
        item_size = rx_count[ready] * 2; // 2 half-symbols per rmt_symbol_word_t
        current_item = 0;
    }

    while (!pulse_ready && item != nullptr) {
        if (current_item < item_size) {
            const rmt_symbol_word_t &sym = item[current_item / 2];
            uint32_t duration = (current_item & 1) ? sym.duration1 : sym.duration0;
            bool level        = (current_item & 1) ? sym.level1    : sym.level0;
            current_item++;

            if (!add_item(duration, level)) {
                current_item = item_size; // Hit end marker
            }
        } else {
            add_item(0, false); // Flush trailing pulse at end of buffer
            item = nullptr;
        }
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
