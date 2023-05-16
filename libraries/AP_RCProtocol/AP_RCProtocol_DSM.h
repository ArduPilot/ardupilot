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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_DSM_ENABLED

#include "AP_RCProtocol_Backend.h"

#include "SoftSerial.h"

#define AP_DSM_MAX_CHANNELS 12

class AP_RCProtocol_DSM : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_DSM(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    void start_bind(void) override;
    void update(void) override;

private:
    void _process_byte(uint32_t timestamp_ms, uint8_t byte);
    void dsm_decode();
    bool dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value);
    void dsm_guess_format(bool reset, const uint8_t dsm_frame[16], unsigned frame_channels);
    bool dsm_parse_byte(uint32_t frame_time_ms, uint8_t b, uint16_t *values,
                        uint16_t *num_values, uint16_t max_channels);
    bool dsm_decode(uint32_t frame_time_ms, const uint8_t dsm_frame[16],
                    uint16_t *values, uint16_t *num_values, uint16_t max_values);

    /**< Channel resolution, 0=unknown, 10=10 bit, 11=11 bit */
    uint8_t channel_shift;

    // format guessing state
    uint32_t	cs10;
    uint32_t	cs11;
    uint32_t samples;

    // bind state machine
    enum {
        BIND_STATE_NONE,
        BIND_STATE1,
        BIND_STATE2,
        BIND_STATE3,
        BIND_STATE4,
    } bind_state;
    uint32_t bind_last_ms;
    uint32_t bind_mode_saved;

    uint16_t last_values[AP_DSM_MAX_CHANNELS];

    struct {
        uint8_t buf[16];
        uint8_t ofs;
    } byte_input;

    enum DSM_DECODE_STATE {
        DSM_DECODE_STATE_DESYNC = 0,
        DSM_DECODE_STATE_SYNC
    } dsm_decode_state;

    uint32_t last_frame_time_ms;
    uint32_t last_rx_time_ms;
    uint16_t chan_count;

    SoftSerial ss{115200, SoftSerial::SERIAL_CONFIG_8N1};
};

#endif  // AP_RCPROTOCOL_DSM_ENABLED
