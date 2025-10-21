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
#include "Semaphores.h"
#include "RmtSigReader.h"
#include <AP_RCProtocol/AP_RCProtocol.h>


#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

class ESP32::RCInput : public AP_HAL::RCInput
{
public:
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;
    void _timer_tick(void);

    const char *protocol() const override
    {
        return last_protocol;
    }
private:
    uint16_t _rc_values[RC_INPUT_MAX_CHANNELS] = {0};
    uint64_t _last_read;
    uint8_t _num_channels;
    Semaphore rcin_mutex;
    int16_t _rssi = -1;
    uint32_t _rcin_timestamp_last_signal;
    bool _init;
    const char *last_protocol;

    ESP32::RmtSigReader sig_reader;
};
