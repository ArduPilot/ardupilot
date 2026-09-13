#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

#ifdef HAL_ESP32_RCIN

using namespace ESP32;

void RmtSigReader::init()
{
    last_high = 0;
    ready_high = 0;
    ready_low = 0;
    pulse_ready = false;
    active_rx_buf = 0;
    q_head = 0;
    q_tail = 0;
    is_processing = false;
    sub_item = 0;

    rmt_rx_channel_config_t rx_chan_config = {};
    rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_chan_config.resolution_hz = frequency;
    rx_chan_config.mem_block_symbols = max_pulses;
    rx_chan_config.gpio_num = HAL_ESP32_RCIN;

    if (rmt_new_rx_channel(&rx_chan_config, &rx_chan) != ESP_OK) {
        return;
    }

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = rx_done_cb;
    rmt_rx_register_event_callbacks(rx_chan, &cbs, this);

    if (rmt_enable(rx_chan) == ESP_OK) {
        is_enabled = true;
        rmt_receive_config_t rx_config = {};
        rx_config.signal_range_min_ns = 100;
        rx_config.signal_range_max_ns = idle_threshold * 1000;

        rmt_receive(rx_chan, rx_raw_buf[active_rx_buf], sizeof(rx_raw_buf[0]), &rx_config);
    }
}

void RmtSigReader::disable()
{
    rmt_disable(rx_chan);
    is_enabled = false;
}

bool IRAM_ATTR RmtSigReader::rx_done_cb(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    RmtSigReader *reader = (RmtSigReader *)user_ctx;

    if (!reader->is_enabled || edata->num_symbols == 0) {
        return pdFALSE;
    }

    // 1. Post current buffer details to the main thread queue
    uint8_t next_head = (reader->q_head + 1) % QUEUE_SIZE;
    if (next_head != reader->q_tail) {
        reader->completed_queue[reader->q_head] = { reader->active_rx_buf, (uint16_t)edata->num_symbols };
        reader->q_head = next_head;
    }

    // 2. Swap ping-pong buffer and immediately re-arm hardware
    reader->active_rx_buf = 1 - reader->active_rx_buf;

    rmt_receive_config_t rx_config = {};
    rx_config.signal_range_min_ns = 100;
    rx_config.signal_range_max_ns = idle_threshold * 1000;

    rmt_receive(rx_chan, reader->rx_raw_buf[reader->active_rx_buf], sizeof(reader->rx_raw_buf[0]), &rx_config);

    return pdFALSE;
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
    while (true) {
        // Fetch next completed buffer from queue if not currently processing one
        if (!is_processing) {
            if (q_head == q_tail) {
                return false; // Queue empty, no data to parse
            }
            BufferReady item = completed_queue[q_tail];
            q_tail = (q_tail + 1) % QUEUE_SIZE;

            processing_buf = item.buf_index;
            processing_count = item.count;
            processing_idx = 0;
            sub_item = 0;
            is_processing = true;
        }

        // Parse symbols directly from the raw ping-pong buffer
        while (processing_idx < processing_count) {
            rmt_symbol_word_t sym = rx_raw_buf[processing_buf][processing_idx];

            if (sub_item == 0) {
                sub_item = 1;
                if (!add_item(sym.duration0, sym.level0)) {
                    sub_item = 0;
                    processing_idx++;
                }
                if (pulse_ready) {
                    width_high = ready_high;
                    width_low = ready_low;
                    pulse_ready = false;
                    return true;
                }
            }

            if (sub_item == 1) {
                sub_item = 0;
                processing_idx++;
                if (!add_item(sym.duration1, sym.level1)) {
                    // Marker reached
                }
                if (pulse_ready) {
                    width_high = ready_high;
                    width_low = ready_low;
                    pulse_ready = false;
                    return true;
                }
            }
        }

        // Synthesize frame end delimiter once the buffer is fully traversed
        add_item(0, 0);
        is_processing = false;

        if (pulse_ready) {
            width_high = ready_high;
            width_low = ready_low;
            pulse_ready = false;
            return true;
        }
    }
}
#endif
