/*
 * Code by Andy Piper <github@andypiper.com>
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
 *
 * Standalone SPI-based NeoPixel/WS2812 serial LED driver for Linux.
 */
#include "SerialLED_SPI.h"

#if HAL_LINUX_SERIALLED_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

extern const AP_HAL::HAL &hal;

namespace Linux {

// Out-of-class definitions required for ODR-use of static constexpr members in C++14
constexpr uint8_t SerialLED_SPI::MAX_CHANNELS;
constexpr uint8_t SerialLED_SPI::MAX_LEDS_PER_CHANNEL;

SerialLED_SPI::~SerialLED_SPI()
{
    if (_spi_fd >= 0) {
        close(_spi_fd);
    }
    free(_spi_buffer);
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        free(_channels[i].led_data);
    }
}

bool SerialLED_SPI::init(const char *spi_device, uint8_t num_channels)
{
    _spi_device = spi_device;
    _num_channels = MIN(num_channels, MAX_CHANNELS);

    if (!open_spi()) {
        return false;
    }

    _initialized = true;
    return true;
}

bool SerialLED_SPI::open_spi()
{
    _spi_fd = open(_spi_device, O_RDWR);
    if (_spi_fd < 0) {
        hal.console->printf("SerialLED_SPI: Failed to open SPI device %s\n", _spi_device);
        return false;
    }

    // Configure SPI mode
    uint8_t mode = SPI_MODE_0;
    if (ioctl(_spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        hal.console->printf("SerialLED_SPI: Failed to set SPI mode\n");
        close(_spi_fd);
        _spi_fd = -1;
        return false;
    }

    // Configure bits per word
    uint8_t bits = 8;
    if (ioctl(_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        hal.console->printf("SerialLED_SPI: Failed to set SPI bits per word\n");
        close(_spi_fd);
        _spi_fd = -1;
        return false;
    }

    // Configure SPI speed
    uint32_t speed = SPI_SPEED_HZ;
    if (ioctl(_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        hal.console->printf("SerialLED_SPI: Failed to set SPI speed\n");
        close(_spi_fd);
        _spi_fd = -1;
        return false;
    }

    return true;
}

bool SerialLED_SPI::set_num_leds(uint16_t chan, uint8_t num_leds, AP_HAL::RCOutput::output_mode mode)
{
    if (!_initialized || chan >= _num_channels || num_leds == 0) {
        return false;
    }

    // Only support NeoPixel modes
    if (mode != AP_HAL::RCOutput::MODE_NEOPIXEL && mode != AP_HAL::RCOutput::MODE_NEOPIXELRGB) {
        return false;
    }

    WITH_SEMAPHORE(_mutex);

    ChannelState &ch = _channels[chan];

    // Limit number of LEDs
    num_leds = MIN(num_leds, MAX_LEDS_PER_CHANNEL);

    // Already configured with enough LEDs
    if (ch.num_leds >= num_leds && ch.mode == mode) {
        return true;
    }

    // Check if SPI buffer needs to grow
    const uint16_t required_size = (static_cast<uint16_t>(num_leds) * BYTES_PER_LED) + RESET_BYTES;
    if (required_size > _spi_buffer_size) {
        free(_spi_buffer);
        _spi_buffer = (uint8_t *)calloc(required_size, 1);
        if (_spi_buffer == nullptr) {
            _spi_buffer_size = 0;
            return false;
        }
        _spi_buffer_size = required_size;
    }

    // Allocate or reallocate LED data array (calloc zeros memory)
    free(ch.led_data);
    ch.led_data = (SerialLed *)calloc(num_leds, sizeof(SerialLed));
    if (ch.led_data == nullptr) {
        ch.num_leds = 0;
        return false;
    }

    ch.num_leds = num_leds;
    ch.mode = mode;
    ch.pending = false;

    return true;
}

bool SerialLED_SPI::set_rgb_data(uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    if (!_initialized || chan >= _num_channels) {
        return false;
    }

    WITH_SEMAPHORE(_mutex);

    ChannelState &ch = _channels[chan];

    if (ch.num_leds == 0 || ch.led_data == nullptr) {
        return false;
    }

    // led == -1 means set all LEDs
    if (led < 0) {
        for (uint8_t i = 0; i < ch.num_leds; i++) {
            ch.led_data[i].red = red;
            ch.led_data[i].green = green;
            ch.led_data[i].blue = blue;
        }
    } else if (led < ch.num_leds) {
        ch.led_data[led].red = red;
        ch.led_data[led].green = green;
        ch.led_data[led].blue = blue;
    } else {
        return false;
    }

    ch.pending = true;
    return true;
}

bool SerialLED_SPI::send(uint16_t chan)
{
    if (!_initialized || chan >= _num_channels || _spi_fd < 0 || _spi_buffer == nullptr) {
        return false;
    }

    WITH_SEMAPHORE(_mutex);

    ChannelState &ch = _channels[chan];

    if (ch.num_leds == 0 || ch.led_data == nullptr || !ch.pending) {
        return false;
    }

    encode_led_data(chan);

    if (!send_spi(chan)) {
        return false;
    }

    ch.pending = false;
    return true;
}

/*
 * Encode LED RGB data into SPI buffer.
 *
 * At 6.4MHz SPI clock, each byte (8 bits) takes 1.25us - exactly one WS2812 bit period.
 * This makes encoding simple: each WS2812 bit becomes one SPI byte.
 *
 * WS2812 timing (T0H=0.4us, T0L=0.85us, T1H=0.8us, T1L=0.45us):
 *   - '0' bit: 0xC0 = 11000000 -> 0.31us high, 0.94us low
 *   - '1' bit: 0xF8 = 11111000 -> 0.78us high, 0.47us low
 *
 * Each LED = 24 color bits = 24 bytes.
 */
void SerialLED_SPI::encode_led_data(uint8_t channel)
{
    ChannelState &ch = _channels[channel];

    // Clear buffer (zeros serve as reset pulse at the end)
    memset(_spi_buffer, 0, _spi_buffer_size);

    uint16_t buf_idx = 0;

    for (uint8_t led = 0; led < ch.num_leds; led++) {
        // Get color bytes in the correct order
        uint8_t colors[3];
        if (ch.mode == AP_HAL::RCOutput::MODE_NEOPIXEL) {
            // GRB ordering for standard NeoPixels
            colors[0] = ch.led_data[led].green;
            colors[1] = ch.led_data[led].red;
            colors[2] = ch.led_data[led].blue;
        } else {
            // RGB ordering for MODE_NEOPIXELRGB
            colors[0] = ch.led_data[led].red;
            colors[1] = ch.led_data[led].green;
            colors[2] = ch.led_data[led].blue;
        }

        // Encode each color byte - each bit becomes one SPI byte
        for (uint8_t color = 0; color < 3; color++) {
            uint8_t value = colors[color];

            // MSB first - bit 7 down to bit 0
            for (int8_t bit = 7; bit >= 0; bit--) {
                _spi_buffer[buf_idx++] = (value & (1 << bit)) ? WS_BIT_1 : WS_BIT_0;
            }
        }
    }

    // Reset pulse: zeros already written by memset, occupying RESET_BYTES at end
}

bool SerialLED_SPI::send_spi(uint8_t channel)
{
    ChannelState &ch = _channels[channel];

    // Calculate actual buffer size needed
    uint16_t data_bytes = (static_cast<uint16_t>(ch.num_leds) * BYTES_PER_LED);
    uint16_t total_bytes = data_bytes + RESET_BYTES;

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = reinterpret_cast<unsigned long>(_spi_buffer);
    tr.rx_buf = 0;
    tr.len = total_bytes;
    tr.speed_hz = SPI_SPEED_HZ;
    tr.bits_per_word = 8;

    if (ioctl(_spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        return false;
    }

    return true;
}

}  // namespace Linux

#endif  // HAL_LINUX_SERIALLED_ENABLED
