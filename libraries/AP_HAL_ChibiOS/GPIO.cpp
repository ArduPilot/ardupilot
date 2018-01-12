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

static struct gpio_entry {
    uint8_t pin_num;
    bool enabled;
    ioline_t pal_line;
} _gpio_tab[]  = {
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
    {0, PAL_LINE(GPIOB, 7U)},
    {1, PAL_LINE(GPIOB, 6U)},
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3
    // pin numbers chosen to match px4 build
    {0,  true,  PAL_LINE(GPIOE, 12U)}, // LED
    {1,  true,  PAL_LINE(GPIOB, 0U)},
    {50, false, PAL_LINE(GPIOE, 14U)},
    {51, false, PAL_LINE(GPIOE, 13U)},
    {52, false, PAL_LINE(GPIOE, 11U)},
    {53, false, PAL_LINE(GPIOE,  9U)},
    {54, true,  PAL_LINE(GPIOD, 13U)},
    {55, true,  PAL_LINE(GPIOD, 14U)},
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_FMUV4
    {0,  true,  PAL_LINE(GPIOB, 1U)},  // green
    {1,  true,  PAL_LINE(GPIOB, 3U)},  // blue
    {2,  true,  PAL_LINE(GPIOB, 11U)}, // red
    {3,  true,  PAL_LINE(GPIOC, 3U)},  // safety
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_MINDPXV2
    {0,  true,  PAL_LINE(GPIOA, 8U)},  // run LED
#endif
};

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
    EXT_MODE_GPIOD, //Chan 0
    EXT_MODE_GPIOD, //Chan 1
    EXT_MODE_GPIOD, //Chan 2
    EXT_MODE_GPIOD, //Chan 3
    EXT_MODE_GPIOD, //Chan 4
    EXT_MODE_GPIOD, //Chan 5
    EXT_MODE_GPIOD, //Chan 6
    EXT_MODE_GPIOD, //Chan 7
    EXT_MODE_GPIOD, //Chan 8
    EXT_MODE_GPIOD, //Chan 9
    EXT_MODE_GPIOD, //Chan 10
    EXT_MODE_GPIOD, //Chan 11
    EXT_MODE_GPIOD, //Chan 12
    EXT_MODE_GPIOD, //Chan 13
    EXT_MODE_GPIOD, //Chan 14
    EXT_MODE_GPIOD //Chan 15
};

ChibiGPIO::ChibiGPIO()
{}

void ChibiGPIO::init()
{
    extStart(&EXTD1, &extcfg);
    uint8_t pwm_count = AP_BoardConfig::get_pwm_count();
    for (uint8_t i=0; i<ARRAY_SIZE_SIMPLE(_gpio_tab); i++) {
        uint8_t pin_num = _gpio_tab[i].pin_num;
        if (pin_num >= 50 && pin_num <= 55) {
            // enable GPIOs based on BRD_PWM_COUNT
            if (pin_num < 50 + pwm_count) {
                _gpio_tab[i].enabled = false;
            } else {
                _gpio_tab[i].enabled = true;
            }
        }
    }
}

void ChibiGPIO::pinMode(uint8_t pin, uint8_t output)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        palSetLineMode(g->pal_line, output);
    }
}

int8_t ChibiGPIO::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}


uint8_t ChibiGPIO::read(uint8_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        return palReadLine(g->pal_line);
    }
    return 0;
}

void ChibiGPIO::write(uint8_t pin, uint8_t value)
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

void ChibiGPIO::toggle(uint8_t pin)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        palToggleLine(g->pal_line);
    }
}

/* Alternative interface: */
AP_HAL::DigitalSource* ChibiGPIO::channel(uint16_t n) {
    return new ChibiDigitalSource(0);
}

/* Interrupt interface: */
bool ChibiGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode) {
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

bool ChibiGPIO::usb_connected(void)
{
    return _usb_connected;
}

ChibiDigitalSource::ChibiDigitalSource(uint8_t v) :
    _v(v)
{}

void ChibiDigitalSource::mode(uint8_t output)
{}

uint8_t ChibiDigitalSource::read() {
    return _v;
}

void ChibiDigitalSource::write(uint8_t value) {
    _v = value;
}

void ChibiDigitalSource::toggle() {
    _v = !_v;
}

void ext_interrupt_cb(EXTDriver *extp, expchannel_t channel) {
    if (ext_irq[channel] != nullptr) {
        ext_irq[channel]();
    }
}
#endif //HAL_BOARD_ChibiOS
