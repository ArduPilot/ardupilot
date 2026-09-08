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
 * Code by @davidbuzz and Claude
 */
#pragma once

#include <AP_HAL/GPIO.h>

/* ArduPilot pin number encoding for AP_HAL_Zephyr: the pin number packs the
 * controller index and the pin within it, so one integer addresses any GPIO. */
#define ZEPHYR_GPIO_PIN(gpio_bank_1indexed, gpio_pin) \
    ((uint8_t)(((gpio_bank_1indexed) - 1) << 5 | (gpio_pin)))

/* Maximum number of simultaneous GPIO interrupt callbacks. */
#define ZEPHYR_GPIO_MAX_IRQS 16

namespace Zephyr {

class DigitalSource : public AP_HAL::DigitalSource {
public:
    explicit DigitalSource(uint8_t pin);
    void    mode(uint8_t output) override;
    uint8_t read() override;
    void    write(uint8_t value) override;
    void    toggle() override;
private:
    uint8_t _pin;
};

class GPIO : public AP_HAL::GPIO {
public:
    GPIO();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;
    bool    valid_pin(uint8_t pin) const override;

    AP_HAL::DigitalSource* channel(uint16_t n) override;

    bool usb_connected(void) override;

    bool attach_interrupt(uint8_t pin,
                          irq_handler_fn_t fn,
                          INTERRUPT_TRIGGER_TYPE mode) override;

    bool attach_interrupt(uint8_t pin,
                          AP_HAL::Proc proc,
                          INTERRUPT_TRIGGER_TYPE mode) override;

    /* ISR-flood protection, ChibiOS parity: a chattering input must not be able to
     * starve the rest of the system. */
    void timer_tick(void) override;
    bool arming_checks(size_t buflen, char *buffer) const override;
};

}  // namespace Zephyr
