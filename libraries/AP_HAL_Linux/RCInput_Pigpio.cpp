/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
   RCInput driver using pigpio library for edge-detection based RC input.
   This uses pigpio's alert callback mechanism to get precise edge timestamps,
   which are then fed to AP_RCProtocol's soft-serial decoder.

   Uses vendored pigpio library (public domain) from libraries/AP_HAL_Linux/pigpio/
   The pigpiod daemon must NOT be running (we use direct library access).
 */

#include "RCInput_Pigpio.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RPI

#include <AP_HAL/AP_HAL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include "pigpio/pigpio.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

// Static instance pointer for callback
static RCInput_Pigpio *_instance = nullptr;

// Edge event ring buffer
static constexpr size_t EDGE_BUFFER_SIZE = 256;
static struct {
    uint32_t tick;
    uint8_t level;
} edge_buffer[EDGE_BUFFER_SIZE];
static volatile uint16_t edge_head = 0;
static volatile uint16_t edge_tail = 0;

// pigpio alert callback - called from pigpio thread on each edge
static void _alert_callback(int gpio, int level, uint32_t tick)
{
    // Ignore watchdog timeout (level 2)
    if (level == 2) {
        return;
    }

    uint16_t next_head = (edge_head + 1) % EDGE_BUFFER_SIZE;
    if (next_head != edge_tail) {
        edge_buffer[edge_head].tick = tick;
        edge_buffer[edge_head].level = level;
        edge_head = next_head;
    }
}

RCInput_Pigpio::RCInput_Pigpio(uint32_t gpio_pin, uint32_t baudrate, bool inverted)
    : _gpio_pin(gpio_pin)
    , _baudrate(baudrate)
    , _inverted(inverted)
    , _initialized(false)
    , _pigpio_initialized(false)
{
}

RCInput_Pigpio::~RCInput_Pigpio()
{
    teardown();
}

void RCInput_Pigpio::teardown()
{
    if (_initialized) {
        gpioSetAlertFunc(_gpio_pin, nullptr);
        gpioTerminate();
        _initialized = false;
        _instance = nullptr;
    }
}

void RCInput_Pigpio::early_init()
{
    // Initialize pigpio library early, before CPU affinity is set.
    // pigpio creates internal threads for DMA handling, and these threads
    // must not be constrained to isolated CPU cores.
    _instance = this;

    // Configure pigpio before initialization
    // Disable pigpio's signal handlers to avoid conflicts with ArduPilot/systemd
    gpioCfgSetInternals(PI_CFG_NOSIGHANDLER);
    // Set 1us sample rate for accurate pulse timing
    gpioCfgClock(1, PI_CLOCK_PCM, 0);

    int ret = gpioInitialise();
    if (ret < 0) {
        printf("RCInput_Pigpio: gpioInitialise failed: %d\n", ret);
        return;
    }

    _pigpio_initialized = true;
}

void RCInput_Pigpio::init()
{
    RCInput::init();

    if (!_pigpio_initialized) {
        // early_init() failed or wasn't called
        return;
    }

    // Set GPIO as input
    int ret = gpioSetMode(_gpio_pin, PI_INPUT);
    if (ret < 0) {
        hal.console->printf("RCInput_Pigpio: gpioSetMode failed: %d\n", ret);
        gpioTerminate();
        return;
    }

    // Set up edge detection callback
    ret = gpioSetAlertFunc(_gpio_pin, _alert_callback);
    if (ret < 0) {
        hal.console->printf("RCInput_Pigpio: gpioSetAlertFunc failed: %d\n", ret);
        gpioTerminate();
        return;
    }

    // Initialize AP_RCProtocol
    AP::RC().init();

    hal.console->printf("RCInput_Pigpio: initialized on GPIO %u (edge detection mode, %s)\n",
                        _gpio_pin, _inverted ? "inverted" : "normal");
    _initialized = true;
}

void RCInput_Pigpio::_timer_tick()
{
    if (!_initialized) {
        return;
    }

    static uint32_t last_tick = 0;
    static int last_level = -1;

    // Process all pending edges
    // Note: We do NOT apply inversion here. The AP_RCProtocol SBUS backend
    // handles inverted signals via its saved_width pulse-swapping mechanism.
    // We just pass raw GPIO measurements (high_duration, low_duration).
    while (edge_tail != edge_head) {
        uint32_t tick = edge_buffer[edge_tail].tick;
        int level = edge_buffer[edge_tail].level;
        edge_tail = (edge_tail + 1) % EDGE_BUFFER_SIZE;

        if (last_level >= 0) {
            // Calculate pulse width in microseconds
            uint32_t width_us = tick - last_tick;

            // Ignore very long pulses (> 100ms)
            if (width_us < 100000) {
                // Feed raw GPIO pulse widths to AP_RCProtocol
                // For inverted SBUS: GPIO high = space (active), GPIO low = mark (idle)
                // The SBUS backend's inverted mode swaps to (mark, space) for SoftSerial
                if (last_level == 1) {
                    // GPIO was high, store duration
                    _last_high_us = width_us;
                } else {
                    // GPIO was low, we have a complete (high, low) pair
                    if (_last_high_us > 0) {
                        AP::RC().process_pulse(_last_high_us, width_us);
                    }
                    _last_high_us = 0;
                }
            }
        }

        last_tick = tick;
        last_level = level;
    }

    // Check if AP_RCProtocol decoded new input
    AP_RCProtocol &rcprot = AP::RC();
    if (rcprot.new_input()) {
        uint8_t n = rcprot.num_channels();
        if (n > LINUX_RC_INPUT_NUM_CHANNELS) {
            n = LINUX_RC_INPUT_NUM_CHANNELS;
        }
        for (uint8_t i = 0; i < n; i++) {
            _pwm_values[i] = rcprot.read(i);
        }
        _num_channels = n;
        rc_input_count++;
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RPI
