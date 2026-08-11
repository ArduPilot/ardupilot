#include <AP_HAL/HAL.h>
#include "RmtSigReader.h"

#ifdef HAL_ESP32_RCIN

using namespace ESP32;

/*
  RX-done callback, invoked from the RMT ISR when a frame completes (the line has
  been idle for longer than signal_range_max_ns). Hand the received symbol batch
  to read() via the ring buffer. This keeps the same producer/consumer split the
  legacy ring-buffer driver had, but on the new RMT driver (driver_ng) so it
  coexists with DShot's rmt_tx output.

  ⚠️ TWO IRAM CONSTRAINTS, BOTH REQUIRED BY CONFIG_RMT_ISR_IRAM_SAFE=y
  (targets/esp32s3/esp-idf/sdkconfig.defaults:51, enabled so DShot's RMT ISR
  survives a flash-cache disable). Both were violated and both bit on hardware:

  1. This callback MUST be IRAM_ATTR. Without it rmt_rx_register_event_callbacks()
     returns ESP_ERR_INVALID_ARG ("on_recv_done callback not in IRAM") and the
     ESP_ERROR_CHECK in init() aborts -> BOOT CRASH-LOOP. OBSERVED 2026-08-11 the
     moment RC-in was first enabled on a board that also builds DShot.
  2. It MUST NOT call rmt_receive(). That function is NOT IRAM-resident in the
     pinned IDF v5.3 (no IRAM_ATTR on rmt_rx.c:361, and CONFIG_RMT_ISR_IRAM_SAFE's
     Kconfig help only promises the *interrupt handler* is IRAM-safe). Calling it
     from an IRAM-safe ISR faults whenever the cache happens to be disabled, i.e.
     an intermittent crash during flash writes -- worse than the boot abort because
     it is rare and load-dependent. The re-arm therefore moved to read(), which is
     called periodically from RCInput (RCInput.cpp:124). RCOutput.cpp's bidir-DShot
     RX callback already did exactly this, for exactly this reason.
 */
bool IRAM_ATTR RmtSigReader::on_recv_done(rmt_channel_handle_t chan,
                                          const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    RmtSigReader *self = (RmtSigReader *)user_ctx;
    BaseType_t hp_task_woken = pdFALSE;
    if (edata->num_symbols > 0) {
        // IRAM-safe: xRingbufferSendFromISR is in IRAM unless
        // CONFIG_RINGBUF_PLACE_ISR_FUNCTIONS_INTO_FLASH is set (default n).
        xRingbufferSendFromISR(self->handle, edata->received_symbols,
                               edata->num_symbols * sizeof(rmt_symbol_word_t),
                               &hp_task_woken);
    }
    // Ask read() to re-arm; do NOT call rmt_receive() here.
    self->rearm_pending = true;
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

    rearm_pending = false;
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
    // Re-arm here rather than in the ISR: rmt_receive() is not IRAM-resident and the
    // RX ISR is IRAM-safe (see the on_recv_done comment). Called every RCInput tick,
    // so the gap between a completed frame and the next arm is ~1 ms -- far shorter
    // than an RC frame period, and the protocol decoders resync anyway.
    if (rearm_pending) {
        rearm_pending = false;
        start_receive();
    }
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
