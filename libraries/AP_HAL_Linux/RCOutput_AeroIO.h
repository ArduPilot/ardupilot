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

    void init() override;

    /**
     * Enable channel
     */
    void enable_ch(uint8_t ch) override;

    /**
     * Disable channel (0 of duty cycle)
     */
    void disable_ch(uint8_t ch) override;

    /**
     * Set all channels in the same frequency
     *
     * @chmask Bitmask
     * @freq_hz Frequency
     */
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;

    /**
     * Get frequency of channel
     *
     * @ch channel
     *
     * Return: frequency of this channel
     */
    uint16_t get_freq(uint8_t ch) override;

    /**
     * Set period in µs
     *
     * @ch channel
     * @period_us time in µs
     */
    void write(uint8_t ch, uint16_t period_us) override;

    void cork() override;

    void push() override;

    /**
     * Get period of the duty cycle in µs
     */
    uint16_t read(uint8_t ch) override;

    /**
     * Set @period_us with the values in µs of each channel
     *
     * @period_us vector that will be filled with duty cycle periods of
     *            each channel
     * @len size of period_us vector
     */
    void read(uint16_t *period_us, uint8_t len) override;

private:
    /**
     * Convert from µs to hw units, 16bits percentage of
     * the frequency, where 0xFFFF is 1/Freq seconds in high
     *
     * @freq Frequency
     * @usec Time in µseconds
     *
     * Return: conversion from µs in a specific frequency to 16bits
     */
    static uint16_t _usec_to_hw(uint16_t freq, uint16_t usec);

    /**
     * Convert from hw units, 16bits percentage of the frequency, to
     * time in µs
     *
     * @freq Frequency
     * @hw_val 16b percentage
     */
    static uint16_t _hw_to_usec(uint16_t freq, uint16_t hw_val);

    /**
     * Low-level spi write
     *
     * @address register address
     * @data value to be written
     *
     * Return: true on success, false otherwise
     */
    bool _hw_write(uint16_t address, uint16_t data);

    /**
     * Low-level spi read
     *
     * @address register address
     *
     * Return: value read from @address
     */
    uint16_t _hw_read(uint16_t address);

    AP_HAL::OwnPtr<AP_HAL::Device> _spi;

    /**
     * Frequency buffer of last written values
     */
    uint16_t *_freq_buffer;

    /**
     * Duty cycle buffer of last written values
     */
    uint16_t *_duty_buffer;

    /**
     * Save information about the channel that will be write in the
     * next @RCOutput_AeroIO#push call.
     *
     * 0b...101
     *      ││└── 1st channel (Pending operation)
     *      │└─── 2nd Channel (No pending operation)
     *      └──── 3rd Channel (Pending operation)
     */
    uint32_t _pending_duty_write_mask = 0;
    uint32_t _pending_freq_write_mask = 0;

    /**
     * If <B>true</B> the push must be called to perform the writing,
     * otherwise will be not necessary.
     */
    bool _corking = false;
};

}
