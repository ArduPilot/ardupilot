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
 * This class can be used by any RCOutput implementation to add serial LED support.
 */
#pragma once

#include "AP_HAL_Linux.h"

#ifndef HAL_LINUX_SERIALLED_ENABLED
#define HAL_LINUX_SERIALLED_ENABLED 0
#endif

#if HAL_LINUX_SERIALLED_ENABLED

#include "Semaphores.h"
#include <AP_HAL/RCOutput.h>

namespace Linux {

/*
 * Standalone SPI-based serial LED driver for NeoPixel/WS2812 LEDs.
 *
 * Uses SPI MOSI to generate the precise timing required by WS2812 LEDs.
 * This class is designed to be composed into RCOutput implementations
 * that need serial LED support.
 */
class SerialLED_SPI {
public:
    SerialLED_SPI() {}
    ~SerialLED_SPI();

    // Initialize the SPI device with given parameters
    bool init(const char *spi_device, uint8_t num_channels);

    // Check if initialized
    bool initialized() const { return _initialized; }

    // Configure number of LEDs on a channel
    bool set_num_leds(uint16_t chan, uint8_t num_leds, AP_HAL::RCOutput::output_mode mode);

    // Set RGB data for an LED (led=-1 sets all LEDs)
    bool set_rgb_data(uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue);

    // Send LED data via SPI
    bool send(uint16_t chan);

    // Get number of channels supported
    uint8_t num_channels() const { return _num_channels; }

private:
    // Maximum supported values (must be defined before use in arrays)
    static constexpr uint8_t MAX_CHANNELS = 4;
    static constexpr uint8_t MAX_LEDS_PER_CHANNEL = 128;

    // SPI buffer sizing:
    // Each LED = 24 color bits, each bit = 8 SPI bits = 24 bytes per LED
    static constexpr uint16_t RESET_BYTES = 64;
    static constexpr uint16_t BYTES_PER_LED = 24;
    static constexpr uint16_t SPI_BUFFER_SIZE = (MAX_LEDS_PER_CHANNEL * BYTES_PER_LED) + RESET_BYTES;

    // SPI bit patterns for WS2812 encoding at 6.4MHz
    static constexpr uint8_t WS_BIT_0 = 0xC0;  // 11000000
    static constexpr uint8_t WS_BIT_1 = 0xF8;  // 11111000

    // SPI configuration
    static constexpr uint32_t SPI_SPEED_HZ = 6400000;

    // LED color data structure
    struct SerialLed {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };

    // Per-channel state with dynamically allocated LED data
    struct ChannelState {
        SerialLed *led_data;
        uint8_t num_leds;
        AP_HAL::RCOutput::output_mode mode;
        bool pending;
    };

    // Open and configure the SPI device
    bool open_spi();

    // Encode LED data to SPI buffer
    void encode_led_data(uint8_t channel);

    // Send data via SPI
    bool send_spi(uint8_t channel);

    const char *_spi_device;
    uint8_t _num_channels;
    int _spi_fd = -1;
    bool _initialized;

    ChannelState _channels[MAX_CHANNELS];
    uint8_t *_spi_buffer;
    uint16_t _spi_buffer_size;

    Semaphore _mutex;
};

}  // namespace Linux

#endif  // HAL_LINUX_SERIALLED_ENABLED
