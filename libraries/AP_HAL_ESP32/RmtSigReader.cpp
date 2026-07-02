#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

#ifdef HAL_ESP32_RCIN

using namespace ESP32;

/*
  RX-done callback, invoked from the RMT ISR when a frame completes (the line has
  been idle for longer than signal_range_max_ns). Hand the received symbol batch
  to read() via the ring buffer, then immediately re-arm the next receive. This
  keeps the same producer/consumer split the legacy ring-buffer driver had, but on
  the new RMT driver (driver_ng) so it coexists with DShot's rmt_tx output.
 */
bool RmtSigReader::on_recv_done(rmt_channel_handle_t chan,
                                const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    RmtSigReader *self = (RmtSigReader *)user_ctx;
    BaseType_t hp_task_woken = pdFALSE;
    if (edata->num_symbols > 0) {
        xRingbufferSendFromISR(self->handle, edata->received_symbols,
                               edata->num_symbols * sizeof(rmt_symbol_word_t),
                               &hp_task_woken);
    }
    // re-arm for the next frame (safe to call from the done callback)
    rmt_receive(chan, self->rx_symbols, sizeof(self->rx_symbols), &self->rx_cfg);
    return hp_task_woken == pdTRUE;
}

void RmtSigReader::init()
{
    // ring buffer carries symbol batches from the ISR to read(); size it for a few frames
    handle = xRingbufferCreate(max_pulses * 8 * sizeof(rmt_symbol_word_t), RINGBUF_TYPE_NOSPLIT);

    rmt_rx_channel_config_t cfg = {};
    cfg.gpio_num = (gpio_num_t)HAL_ESP32_RCIN;
    cfg.clk_src = RMT_CLK_SRC_DEFAULT;   // 80 MHz APB
    cfg.resolution_hz = frequency;       // 1 MHz -> 1 us/tick (matches the old clk_div 80)
    cfg.mem_block_symbols = 64;          // RX uses ping-pong to stream longer frames
    rx_chan = nullptr;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&cfg, &rx_chan));

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = on_recv_done;
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, this));

    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    rx_cfg = {};
    rx_cfg.signal_range_min_ns = 1000;                  // ignore < 1 us glitches
    rx_cfg.signal_range_max_ns = idle_threshold * 1000; // 3 ms idle -> end of frame

    start_receive();
}

void RmtSigReader::start_receive()
{
    rmt_receive(rx_chan, rx_symbols, sizeof(rx_symbols), &rx_cfg);
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
        item = (rmt_symbol_word_t*) xRingbufferReceive(handle, &item_size, 0);
        item_size /= sizeof(rmt_symbol_word_t);
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
