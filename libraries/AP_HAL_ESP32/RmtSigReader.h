/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "AP_HAL_ESP32.h"
#include "driver/rmt_rx.h"          // new RMT RX driver (driver_ng); coexists with DShot's rmt_tx
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

class ESP32::RmtSigReader
{
public:
    static const int frequency = 1000000;   // 1 MHz -> 1 us per RMT tick
    static const int max_pulses = 128;       // size of the per-receive symbol buffer
    static const int idle_threshold = 3000;  // us; a >= 3 ms gap marks the end of a frame
    void init();
    bool read(uint32_t &width_high, uint32_t &width_low);
private:
    bool add_item(uint32_t duration, bool level);
    void start_receive();   // (re)arm rmt_receive into rx_symbols
    // RX-done callback (runs in the RMT ISR): queues the received batch and re-arms.
    static bool on_recv_done(rmt_channel_handle_t chan,
                             const rmt_rx_done_event_data_t *edata, void *user_ctx);

    rmt_channel_handle_t rx_chan;
    rmt_receive_config_t rx_cfg;
    RingbufHandle_t handle;                    // ISR -> read() handoff of symbol batches
    rmt_symbol_word_t rx_symbols[max_pulses];  // buffer the driver fills for each frame

    // draining state for read()
    rmt_symbol_word_t *item;
    size_t item_size;
    size_t current_item;

    // pulse assembly state (high duration followed by low duration)
    uint32_t last_high;
    uint32_t ready_high;
    uint32_t ready_low;
    bool pulse_ready;
};
