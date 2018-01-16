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
} _gpio_tab[] = HAL_GPIO_PINS;

#define NUM_PINS ARRAY_SIZE_SIMPLE(_gpio_tab)
#define PIN_ENABLED(pin) ((pin)<NUM_PINS && _gpio_tab[pin].enabled)

/*
  map a user pin number to a GPIO table entry
 */
static struct gpio_entry *gpio_by_pin_num(uint8_t pin_num)
{
    for (uint8_t i=0; i<ARRAY_SIZE_SIMPLE(_gpio_tab); i++) {
        if (pin_num == _gpio_tab[i].pin_num) {
            if (!_gpio_tab[i].enabled) {
                return NULL;
            }
            return &_gpio_tab[i];
        }
    }
    return NULL;
}

static void ext_interrupt_cb(EXTDriver *extp, expchannel_t channel);

static AP_HAL::Proc ext_irq[22]; // ext int irq list
static EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

static const uint32_t irq_port_list[] = {
    HAL_GPIO_INTERRUPT_PORT, //Chan 0
    HAL_GPIO_INTERRUPT_PORT, //Chan 1
    HAL_GPIO_INTERRUPT_PORT, //Chan 2
    HAL_GPIO_INTERRUPT_PORT, //Chan 3
    HAL_GPIO_INTERRUPT_PORT, //Chan 4
    HAL_GPIO_INTERRUPT_PORT, //Chan 5
    HAL_GPIO_INTERRUPT_PORT, //Chan 6
    HAL_GPIO_INTERRUPT_PORT, //Chan 7
    HAL_GPIO_INTERRUPT_PORT, //Chan 8
    HAL_GPIO_INTERRUPT_PORT, //Chan 9
    HAL_GPIO_INTERRUPT_PORT, //Chan 10
    HAL_GPIO_INTERRUPT_PORT, //Chan 11
    HAL_GPIO_INTERRUPT_PORT, //Chan 12
    HAL_GPIO_INTERRUPT_PORT, //Chan 13
    HAL_GPIO_INTERRUPT_PORT, //Chan 14
    HAL_GPIO_INTERRUPT_PORT  //Chan 15
};

GPIO::GPIO()
{}

void GPIO::init()
{
    extStart(&EXTD1, &extcfg);
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
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return new DigitalSource(0);
}

/* Interrupt interface: */
bool GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode) {
    extStop(&EXTD1);
    switch(mode) {
        case HAL_GPIO_INTERRUPT_LOW:
            extcfg.channels[interrupt_num].mode = EXT_CH_MODE_LOW_LEVEL;
            break;
        case HAL_GPIO_INTERRUPT_FALLING:
            extcfg.channels[interrupt_num].mode = EXT_CH_MODE_FALLING_EDGE;
            break;
        case HAL_GPIO_INTERRUPT_RISING:
            extcfg.channels[interrupt_num].mode = EXT_CH_MODE_RISING_EDGE;
            break;
        case HAL_GPIO_INTERRUPT_BOTH:
            extcfg.channels[interrupt_num].mode = EXT_CH_MODE_BOTH_EDGES;
            break;
        default: return false;
    }
    extcfg.channels[interrupt_num].mode |= EXT_CH_MODE_AUTOSTART | irq_port_list[interrupt_num];
    ext_irq[interrupt_num] = p;
    extcfg.channels[interrupt_num].cb = ext_interrupt_cb;
    extStart(&EXTD1, &extcfg);
    return true;
}

bool GPIO::usb_connected(void)
{
    return _usb_connected;
}

DigitalSource::DigitalSource(uint8_t v) :
    _v(v)
{}

void DigitalSource::mode(uint8_t output)
{}

uint8_t DigitalSource::read() {
    return _v;
}

void DigitalSource::write(uint8_t value) {
    _v = value;
}

void DigitalSource::toggle() {
    _v = !_v;
}

void ext_interrupt_cb(EXTDriver *extp, expchannel_t channel) {
    if (ext_irq[channel] != nullptr) {
        ext_irq[channel]();
    }
}
#endif //HAL_BOARD_ChibiOS
