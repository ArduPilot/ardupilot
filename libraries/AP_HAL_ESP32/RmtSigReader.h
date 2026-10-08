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
#include "driver/rmt_rx.h"

class ESP32::RmtSigReader
{
public:
    static const int frequency = 1000000;  //1MHZ
    static const int max_pulses = 128;
    static const int idle_threshold = 3000;  //we require at least 3ms gap between frames
    void init();
    void disable();
    bool read(uint32_t &width_high, uint32_t &width_low);
private:
    bool add_item(uint32_t duration, bool level);
    static bool IRAM_ATTR rx_done_cb(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx);

    rmt_channel_handle_t rx_chan = nullptr;
    bool is_enabled = false;

    // Ping-pong hardware buffers
    rmt_symbol_word_t rx_raw_buf[2][max_pulses];
    uint8_t active_rx_buf = 0;

    // ISR-to-Thread FIFO Queue (stores completed buffer metadata)
    struct BufferReady {
        uint8_t buf_index;
        uint16_t count;
    };
    
    static const uint8_t QUEUE_SIZE = 8;
    BufferReady completed_queue[QUEUE_SIZE];
    volatile uint8_t q_head = 0;
    volatile uint8_t q_tail = 0;

    // Decoding state tracked in main thread loop
    uint8_t processing_buf = 0;
    uint16_t processing_idx = 0;
    uint16_t processing_count = 0;
    bool is_processing = false;
    uint8_t sub_item = 0;

    uint32_t last_high = 0;
    uint32_t ready_high = 0;
    uint32_t ready_low = 0;
    bool pulse_ready = false;
};
