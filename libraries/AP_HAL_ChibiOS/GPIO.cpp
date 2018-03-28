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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include "GPIO.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <AP_BoardConfig/AP_BoardConfig.h>

using namespace ChibiOS;

// GPIO pin table from hwdef.dat
static struct gpio_entry {
    uint8_t pin_num;
    bool enabled;
    uint8_t pwm_num;
    ioline_t pal_line;
    uint16_t port;
} _gpio_tab[] = HAL_GPIO_PINS;

#define NUM_PINS ARRAY_SIZE_SIMPLE(_gpio_tab)
#define PIN_ENABLED(pin) ((pin)<NUM_PINS && _gpio_tab[pin].enabled)

/*
  map a user pin number to a GPIO table entry
 */
static struct gpio_entry *gpio_by_pin_num(uint8_t pin_num, bool check_enabled=true)
{
    for (uint8_t i=0; i<ARRAY_SIZE_SIMPLE(_gpio_tab); i++) {
        if (pin_num == _gpio_tab[i].pin_num) {
            if (check_enabled && !_gpio_tab[i].enabled) {
                return NULL;
            }
            return &_gpio_tab[i];
        }
    }
    return NULL;
}

static void ext_interrupt_cb(EXTDriver *extp, expchannel_t channel);

static EXTConfig extcfg;
static AP_HAL::Proc ext_irq[EXT_MAX_CHANNELS]; // ext int irq list

GPIO::GPIO()
{}

void GPIO::init()
{
    // auto-disable pins being used for PWM output based on BRD_PWM_COUNT parameter
    uint8_t pwm_count = AP_BoardConfig::get_pwm_count();
    for (uint8_t i=0; i<ARRAY_SIZE_SIMPLE(_gpio_tab); i++) {
        struct gpio_entry *g = &_gpio_tab[i];
        if (g->pwm_num != 0) {
            g->enabled = g->pwm_num > pwm_count;
        }
    }
}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        palSetLineMode(g->pal_line, output);
    }
}

int8_t GPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t GPIO::read(uint8_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        return palReadLine(g->pal_line);
    }
    return 0;
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        if (value == PAL_LOW) {
            palClearLine(g->pal_line);
        } else {
            palSetLine(g->pal_line);
        }
    }
}

void GPIO::toggle(uint8_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        palToggleLine(g->pal_line);
    }
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (!g) {
        return nullptr;
    }
    return new DigitalSource(g->pal_line);
}

extern const AP_HAL::HAL& hal;

/* 
   Attach an interrupt handler to a GPIO pin number. The pin number
   must be one specified with a GPIO() marker in hwdef.dat
 */
bool GPIO::attach_interrupt(uint8_t pin, AP_HAL::Proc p, uint8_t mode)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return false;
    }
    uint8_t pad = PAL_PAD(g->pal_line);
    if (p && ext_irq[pad] != nullptr && ext_irq[pad] != p) {
        // already used
        return false;
    } else if (!p && !ext_irq[pad]) {
        // nothing to remove
        return false;
    }
    uint32_t chmode = 0;
    switch(mode) {
        case HAL_GPIO_INTERRUPT_LOW:
            chmode = EXT_CH_MODE_LOW_LEVEL;
            break;
        case HAL_GPIO_INTERRUPT_FALLING:
            chmode = EXT_CH_MODE_FALLING_EDGE;
            break;
        case HAL_GPIO_INTERRUPT_RISING:
            chmode = EXT_CH_MODE_RISING_EDGE;
            break;
        case HAL_GPIO_INTERRUPT_BOTH:
            chmode = EXT_CH_MODE_BOTH_EDGES;
            break;
        default:
            if (p) {
                return false;
            }
            break;
    }
    if (_ext_started) {
        extStop(&EXTD1);
        _ext_started = false;
    }
    extcfg.channels[pad].mode = chmode;
    extcfg.channels[pad].mode |= (p?EXT_CH_MODE_AUTOSTART:0) | g->port;
    ext_irq[pad] = p;
    extcfg.channels[pad].cb = ext_interrupt_cb;
    extStart(&EXTD1, &extcfg);
    _ext_started = true;
    return true;
}

bool GPIO::usb_connected(void)
{
    return _usb_connected;
}

DigitalSource::DigitalSource(ioline_t _line) :
    line(_line)
{}

void DigitalSource::mode(uint8_t output)
{
    palSetLineMode(line, output);
}

uint8_t DigitalSource::read()
{
    return palReadLine(line);
}

void DigitalSource::write(uint8_t value)
{
    palWriteLine(line, value);
}

void DigitalSource::toggle()
{
    palToggleLine(line);
}

void ext_interrupt_cb(EXTDriver *extp, expchannel_t channel)
{
    if (ext_irq[channel] != nullptr) {
        ext_irq[channel]();
    }
}

/* 
   set the output mode for a pin. Used to restore an alternate function
   after using a pin as GPIO. Private to HAL_ChibiOS
*/
void GPIO::_set_mode(uint8_t pin, uint32_t mode)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (g) {
        palSetLineMode(g->pal_line, mode);
    }
}

#endif //HAL_BOARD_ChibiOS
