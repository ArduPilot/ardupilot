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

#include "RCOutput_AeroIO.h"

#include <dirent.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <utility>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
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

// uint to bitmask
#define UINT_TO_MASK(x) (1 << x)

// Variables to perform ongoing tests
#define READ_PREFIX 0b10000000
#define WRITE_PREFIX 0b01000000
#define WRITE_TRY_MAX 200

// Set or Clear MSb of BYTE
#define SMSbB 0x8000
#define CMSbB 0x7FFF
#define WADDRESS(x) ((x) | SMSbB)
#define RADDRESS(x) ((x) & CMSbB)

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

RCOutput_AeroIO::RCOutput_AeroIO()
    : _spi(nullptr)
    , _freq_buffer(new uint16_t[PWM_CHAN_COUNT])
    , _duty_buffer(new uint16_t[PWM_CHAN_COUNT])
{
}

RCOutput_AeroIO::~RCOutput_AeroIO()
{
    delete _freq_buffer;
    delete _duty_buffer;
}

void RCOutput_AeroIO::init()
{
    _spi = std::move(hal.spi->get_device(DEVICE_NAME));

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

    for (uint8_t i = 0; (chmask >> i) != 0 && i < PWM_CHAN_COUNT; i++) {
        if ((chmask >> i) & 0b1) {
            _freq_buffer[i] = freq_hz;
        }
    }

    if (!_corking) {
        push();
    }
}

uint16_t RCOutput_AeroIO::get_freq(uint8_t ch)
{
    return fpga_read(2 * ch + 1);
}

void RCOutput_AeroIO::enable_ch(uint8_t ch)
{
    _enabled_channels_mask |= UINT_TO_MASK(ch);
    push();
}

void RCOutput_AeroIO::disable_ch(uint8_t ch)
{
    _enabled_channels_mask &= ~UINT_TO_MASK(ch);
    push();
}

void RCOutput_AeroIO::write(uint8_t ch, uint16_t period_us)
{
    _pending_duty_write_mask |= UINT_TO_MASK(ch);
    _duty_buffer[ch] = us2perc(_freq_buffer[ch], period_us);

    if (!_corking) {
        push();
    }
}

void RCOutput_AeroIO::cork()
{
    _corking = true;
}

void RCOutput_AeroIO::push()
{
    _corking = false;

    for (uint8_t i = 0; (_pending_freq_write_mask >> i) != 0 && i < PWM_CHAN_COUNT; i++) {
        if ((_pending_freq_write_mask >> i) & 0b1) {
            fpga_write(2 * i + 1, _freq_buffer[i]);
        }
    }

    for (uint8_t i = 0; (_pending_duty_write_mask >> i) != 0 && i < PWM_CHAN_COUNT; i++) {
        if ((_pending_duty_write_mask >> i) & 0b1) {
            if ((_enabled_channels_mask >> i) & 0b1) {
                fpga_write(2 * i, _duty_buffer[i]);
            } else {
                fpga_write(2 * i, 0);
            }
        }
    }

    _pending_freq_write_mask = _pending_duty_write_mask = 0;
}

uint16_t RCOutput_AeroIO::read(uint8_t ch)
{
    return fpga_read(2 * ch);
}

void RCOutput_AeroIO::read(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        period_us[i] = fpga_read(2 * i);
    }
}

bool RCOutput_AeroIO::fpga_write(uint16_t address, uint16_t data)
{
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
     * The second and third element is the MSB and LSB of the writing value.
     * This value (16bits) can be the address of memory stack, frequency in Hz
     * or duty cycle percentage.
     *
     * The PWM frequency is always even and the duty cycle percentage odd. E.g:
     * pwm_01: Address 0x0000 frequency
     *       : Address 0x0001 duty cycle
     * pwm_02: Address 0x0002 frequency
     *
     * Right now, the FPGA have only 8bits address management,
     * so, the MSb is used to write and read in the FPGA logic.
     * The address range value that is allowed is x ∈ [0 127] allowing 64 PWMs.
     *
     * Eg of allowed values.
     * uint16_t freq = 100;  // PWM channel in 100Hz
     * uint16_t duty = 1823; // duty cycle in (1823/65535) that's 2.78% of 100Hz.
     *                       // the signal will hold high until 0.010278s
     *
     */

    address = WADDRESS(address);

    uint8_t data_array[5];

    // Write in the SPI buffer the address value
    data_array[0] = WRITE_PREFIX;
    data_array[1] = ((address >> 8) & 0xFF);
    data_array[2] = (address & 0xFF);
    data_array[3] = ((data >> 8) & 0xFF);
    data_array[4] = (data & 0xFF);
    _spi->transfer(data_array, 5, nullptr, 0);
    return true;
}

uint16_t RCOutput_AeroIO::fpga_read(uint16_t address)
{
    address = RADDRESS(address);

    uint8_t data_array[3];
    uint8_t data_received[4] = { 0 };

    // Write in the SPI buffer the address value
    data_array[0] = WRITE_PREFIX;
    data_array[1] = ((address >> 8) & 0xFF);
    data_array[2] = (address & 0xFF);
    _spi->transfer(data_array, 3, nullptr, 0);

    /* Read the SPI buffer
     * The FPGA hardware will write in the SPI buffer after the request
     */
    data_array[0] = READ_PREFIX;
    _spi->transfer(data_array, 1, data_received, 4);
    return uint16_t((data_received[0] << 24) | (data_received[1] << 16) | (data_received[2] << 8) | data_received[3]);
}

uint16_t RCOutput_AeroIO::us2perc(uint16_t freq, uint16_t us)
{
    /**
     * Convert from us to 16b percentage
     *
     * \f$
     * \begin{matrix}
     * \mu_S = 10^{-6} (seconds)\\
     * F_{Hz} = frequency (Hz)\\
     * P_{16b} = \frac{\mu_{S}}{{1/F_{Hz}}} \times 65535\\
     * P_{16b} = 65535*(\mu_{S} \times F_{Hz})\\
     * \end{matrix}
     * \f$
     */

    float f = freq;
    float u = us;
    return 0xFFFF * u * f / USEC_PER_SEC;
}

uint16_t RCOutput_AeroIO::perc2us(uint16_t freq, uint16_t perc)
{
    /**
     * \begin{matrix}
     * P_{16b}= 16b_{percentage}\\
     * F_{Hz} = frequency (Hz)\\
     * \mu_{S} = (P_{16b}/65535) \times (1/F_{Hz}) \times 10^6\\
     * \mu_{S} = P_{16b}*10^6/(65535*F_{Hz})
     * \end{matrix}
     */

    float p = perc;
    float f = freq;
    return p * USEC_PER_SEC / (0xFFFF * f);
}
