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
 * Code by Bayu Laksono
 */

#include "AP_HAL_ESP32.h"
#include "GPIO.h"

#include "hal/gpio_types.h"
#include "driver/gpio.h"

using namespace ESP32;

static gpio_num_t gpio_by_pin_num(uint8_t pin)
{
    if (pin < GPIO_NUM_MAX) {
        return (gpio_num_t)pin;
    }
    return GPIO_NUM_NC;
}

GPIO::GPIO()
{}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    gpio_num_t g = gpio_by_pin_num(pin);
    if (g != GPIO_NUM_NC) {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = output ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1ULL<<g;
        io_conf.pull_down_en = output ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
    }
}

uint8_t GPIO::read(uint8_t pin)
{
    gpio_num_t g = gpio_by_pin_num(pin);
    if (g != GPIO_NUM_NC) {
        return gpio_get_level(g);
    }
    return 0;
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    gpio_num_t g = gpio_by_pin_num(pin);
    if (g != GPIO_NUM_NC) {
        gpio_set_level(g, value);
    }
}

void GPIO::toggle(uint8_t pin)
{
    gpio_num_t g = gpio_by_pin_num(pin);
    if (g != GPIO_NUM_NC) {
        if (gpio_get_level(g)) {
            gpio_set_level(g, 0);
        }
        else {
            gpio_set_level(g, 1);
        }
    }
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t pin) {
    gpio_num_t g = gpio_by_pin_num(pin);
    if (g != GPIO_NUM_NC) {
        return NEW_NOTHROW DigitalSource(pin);
    }
    return nullptr;
}

bool GPIO::usb_connected(void)
{
    return false;
}

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{
    gpio_num_t g = gpio_by_pin_num(_pin);
    if (g != GPIO_NUM_NC) {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = output ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1ULL<<g;
        io_conf.pull_down_en = output ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
    }
}

uint8_t DigitalSource::read() {
    gpio_num_t g = gpio_by_pin_num(_pin);
    if (g != GPIO_NUM_NC) {
        return gpio_get_level(g);
    }
    return 0;
}

void DigitalSource::write(uint8_t value) {
    gpio_num_t g = gpio_by_pin_num(_pin);
    if (g != GPIO_NUM_NC) {
        gpio_set_level(g, value);
    }
}

void DigitalSource::toggle() {
    gpio_num_t g = gpio_by_pin_num(_pin);
    if (g != GPIO_NUM_NC) {
        if (gpio_get_level(g)) {
            gpio_set_level(g, 0);
        }
        else {
            gpio_set_level(g, 1);
        }
    }
}
