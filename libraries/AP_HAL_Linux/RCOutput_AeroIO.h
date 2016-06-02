/// -- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil --
/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
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
 */

#pragma once

#include "AP_HAL_Linux.h"

namespace Linux {

class RCOutput_AeroIO : public AP_HAL::RCOutput {
public:
    RCOutput_AeroIO();
    ~RCOutput_AeroIO();
    void init();

    /**
     * @brief Reseal all channels to default value (0 of dutycicle)
     *
     * @return void
     */
    void reset_all_channels();

    /**
     * @brief Not implemented
     *
     * @param ch ...
     * @return void
     */
    void enable_ch(uint8_t ch);

    /**
     * @brief Disable channel (0 of dutycicle)
     *
     * @param ch channel
     * @return void
     */
    void disable_ch(uint8_t ch);

    /**
     * @brief Set all channels in the same frequency
     *
     * @param chmask Bitmask
     * @param freq_hz Frequency
     * @return void
     */
    void set_freq(uint32_t chmask, uint16_t freq_hz);

    /**
     * @brief Get frequency of channel
     *
     * @param ch channel
     * @return uint16_t
     */
    uint16_t get_freq(uint8_t ch);

    /**
     * @brief Set period in µS
     *
     * @param ch channel
     * @param period_us time in µS
     * @return void
     */
    void write(uint8_t ch, uint16_t period_us);

    void cork() override;
    void push() override;

    /**
     * @brief Return frequency
     *
     * @param ch channel
     * @return uint16_t
     */
    uint16_t read(uint8_t ch);

    /**
     * @brief Set period_us with the values in µS of each channel
     *
     * @param period_us Vector that will be filled with µS of each channel
     * @param len Size of period_us vector
     * @return void
     */
    void read(uint16_t* period_us, uint8_t len);

private:
    void reset();

    AP_HAL::OwnPtr<AP_HAL::Device> _spi;

    /**
     * @brief Low-level spi write
     *
     * @param address
     * @param data
     * @return bool, return false if fail
     */
    bool fpga_write(uint16_t address, uint16_t data);

    /**
     * @brief Low-level spi read
     *
     * @param address
     * @return uint16_t, return data
     */
    uint16_t fpga_read(uint16_t address);

    /**
     * @brief Convert from µS (1E-6 seconds) to 16bits percentage of the frequency,
     * where 0xFFFF is 1/Freq Seconds in high
     *
     * @param freq Frequency
     * @param us Time in µseconds
     * @return uint16_t, return the convertion from us in a specific frequency to 16bits percentage
     */
    uint16_t us2perc(uint16_t freq, uint16_t us);

    /**
     * @brief Convert from percentage to time in µS (1E-6 seconds)
     *
     * @param freq Frequency
     * @param perc 16b percentage
     * @return uint16_t
     */
    uint16_t perc2us(uint16_t freq, uint16_t perc);

    /**
     * @brief Frequency buffer of last written values
     *
     */
    uint16_t *_freq_buffer;

    /**
     * @brief Dutycicle buffer of last written values
     *
     */
    uint16_t *_duty_buffer;

    uint32_t _enabled_channels_mask = 0;

    /**
     * @brief If <B>true</B> the push must be called to perform the writing, otherwise will be not necessary.
     *
     */
    bool _corking = false;

    /**
     * @brief Save information about the channel that will be write in the next @RCOutput_AeroIO#push call.
     *
     * 0b...101
     *      ││└── 1º Channel (Pending operation)
     *      │└─── 2º Channel (No pending operation)
     *      └──── 3º Channel (Pending operation)
     *
     */
    uint32_t _pending_duty_write_mask = 0;
    uint32_t _pending_freq_write_mask = 0;
};

}
