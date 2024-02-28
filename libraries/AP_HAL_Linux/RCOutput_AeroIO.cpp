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
#include "RCOutput_AeroIO.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

using namespace Linux;

// Device name in @SPIDeviceDriver#_device
#define DEVICE_NAME "aeroio"

// Number of channels
#define PWM_CHAN_COUNT 16

// Set all channels
#define ALL_CHAN_MASK ((1 << PWM_CHAN_COUNT) - 1)

// Default PWM frequency
#define DEFAULT_FREQ 400

// Default PWM duty cycle
#define DEFAULT_DUTY 0

// Set or Clear MSb of BYTE
#define WADDRESS(x) ((x) | 0x8000)
#define RADDRESS(x) ((x) & 0x7FFF)

// Variables to perform ongoing tests
#define READ_PREFIX 0x80
#define WRITE_PREFIX 0x40

/**
 * The data_array uses 3 elements to perform the data transaction.
 * The first element is a data byte that provides to FPGA's hardware
 * the transaction type that will be realized inside the SPI module.
 * Where:
 *
 * ╔═════════╦═════════╦══════════╦══════════╦══════════╦══════════╦══════════╦═══════════╗
 * ║   MSB   ║         ║          ║          ║          ║          ║          ║    LSB    ║
 * ╠═════════╬═════════╬══════════╬══════════╬══════════╬══════════╬══════════╬═══════════╣
 * ║ wr_addr ║ rd_addr ║ reserved ║ reserved ║ reserved ║ reserved ║ reserved ║ reserved  ║
 * ╚═════════╩═════════╩══════════╩══════════╩══════════╩══════════╩══════════╩═══════════╝
 *
 * ╔═══════════╦═════════╦═════════╗
 * ║  Register ║ wr_addr ║ rd_addr ║
 * ╠═══════════╬═════════╬═════════╣
 * ║   write   ║    0    ║    X    ║
 * ╠═══════════╬═════════╬═════════╣
 * ║   read    ║    X    ║    0    ║
 * ╠═══════════╬═════════╬═════════╣
 * ║   status  ║    1    ║    1    ║
 * ╚═══════════╩═════════╩═════════╝
 *
 * So, to perform a write transaction in the SPI module it's necessary to send. E.g:
 * 0b 01xx xxxx
 * And to a read transaction..
 * 0b 10xx xxxx
 *
 * The PWM frequency is always even and the duty cycle percentage odd. E.g:
 * pwm_01: Address 0x0000 frequency
 *       : Address 0x0001 duty cycle
 * pwm_02: Address 0x0002 frequency
 *   .
 *   .
 *   .
 *
 * Eg of allowed values:
 *     // PWM channel in 100Hz
 *     uint16_t freq = 100;
 *
 *     // duty cycle in (1823/65535) that's 2.78% of 100Hz:
 *     // the signal will hold high until 278 usec
 *     uint16_t duty = 1823;
 */

extern const AP_HAL::HAL& hal;

RCOutput_AeroIO::RCOutput_AeroIO()
    : _freq_buffer(new uint16_t[PWM_CHAN_COUNT])
    , _duty_buffer(new uint16_t[PWM_CHAN_COUNT])
{
}

RCOutput_AeroIO::~RCOutput_AeroIO()
{
    delete[] _freq_buffer;
    delete[] _duty_buffer;
}

void RCOutput_AeroIO::init()
{
    _spi = std::move(hal.spi->get_device(DEVICE_NAME));
    if (!_spi) {
        AP_HAL::panic("Could not initialize AeroIO");
    }

    // Reset all channels to default value
    cork();
    set_freq(ALL_CHAN_MASK, DEFAULT_FREQ);
    for (uint8_t i = 0; i < PWM_CHAN_COUNT; i++) {
        write(i, DEFAULT_DUTY);
    }
    push();
}

void RCOutput_AeroIO::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    _pending_freq_write_mask |= chmask;

    for (uint8_t i = 0; i < PWM_CHAN_COUNT; i++) {
        if ((chmask >> i) & 0x01) {
            _freq_buffer[i] = freq_hz;
        }
    }

    if (!_corking) {
        _corking = true;
        push();
    }
}

uint16_t RCOutput_AeroIO::get_freq(uint8_t ch)
{
    if (ch >= PWM_CHAN_COUNT) {
        return 0;
    }
    return _freq_buffer[ch];
}

void RCOutput_AeroIO::enable_ch(uint8_t ch)
{
    if (ch >= PWM_CHAN_COUNT) {
        return;
    }
    _pending_duty_write_mask |= (1U << ch);
    _corking = true;
    push();
}

void RCOutput_AeroIO::disable_ch(uint8_t ch)
{
    if (ch >= PWM_CHAN_COUNT) {
        return;
    }
    _duty_buffer[ch] = 0;
    _pending_duty_write_mask |= (1U << ch);
    _corking = true;
    push();
}

void RCOutput_AeroIO::write(uint8_t ch, uint16_t period_us)
{
    _pending_duty_write_mask |= (1U << ch);
    _duty_buffer[ch] = period_us;

    if (!_corking) {
        _corking = true;
        push();
    }
}

void RCOutput_AeroIO::cork()
{
    _corking = true;
}

void RCOutput_AeroIO::push()
{
    if (!_corking) {
        return;
    }
    _corking = false;

    for (uint8_t i = 0; i < PWM_CHAN_COUNT; i++) {
        if ((_pending_freq_write_mask >> i) & 0x01) {
            _hw_write(2 * i + 1, _freq_buffer[i]);
        }
    }

    for (uint8_t i = 0; i < PWM_CHAN_COUNT; i++) {
        if ((_pending_duty_write_mask >> i) & 0x01) {
            _hw_write(2 * i, _usec_to_hw(_freq_buffer[i], _duty_buffer[i]));
        }
    }

    _pending_freq_write_mask = _pending_duty_write_mask = 0;
}

uint16_t RCOutput_AeroIO::read(uint8_t ch)
{
    if (ch >= PWM_CHAN_COUNT) {
        return 0;
    }
#ifndef AEROIO_DEBUG
    return _duty_buffer[ch];
#else
    return _hw_to_usec(_freq_buffer[ch], _hw_read(2 * ch));
#endif
}

void RCOutput_AeroIO::read(uint16_t *period_us, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

bool RCOutput_AeroIO::_hw_write(uint16_t address, uint16_t data)
{
    struct PACKED {
        uint8_t prefix;
        be16_t addr;
        be16_t val;
    } tx;

    address = WADDRESS(address);

    tx.prefix = WRITE_PREFIX;
    tx.addr = htobe16(address);
    tx.val = htobe16(data);

    return _spi->transfer((uint8_t *)&tx, sizeof(tx), nullptr, 0);
}

uint16_t RCOutput_AeroIO::_hw_read(uint16_t address)
{
    struct PACKED {
        uint8_t prefix;
        be16_t addr;
    } tx;
    struct PACKED {
        uint8_t ignored[2];
        be16_t val;
    } rx;

    address = RADDRESS(address);

    // Write in the SPI buffer the address value
    tx.prefix = WRITE_PREFIX;
    tx.addr = htobe16(address);
    if (!_spi->transfer((uint8_t *)&tx, sizeof(tx), nullptr, 0)) {
        return 0;
    }

    /*
     * Read the SPI buffer, sending only the prefix as tx
     * The hardware will fill in 32 bits after the request
     */
    tx.prefix = READ_PREFIX;
    if (!_spi->transfer((uint8_t *)&tx, 1, (uint8_t *)&rx, sizeof(rx))) {
        return 0;
    }

    return be16toh(rx.val);
}

uint16_t RCOutput_AeroIO::_usec_to_hw(uint16_t freq, uint16_t usec)
{
    float f = freq;
    float u = usec;
    return 0xFFFF * u * f / AP_USEC_PER_SEC;
}

uint16_t RCOutput_AeroIO::_hw_to_usec(uint16_t freq, uint16_t hw_val)
{
    float p = hw_val;
    float f = freq;
    return p * AP_USEC_PER_SEC / (0xFFFF * f);
}
