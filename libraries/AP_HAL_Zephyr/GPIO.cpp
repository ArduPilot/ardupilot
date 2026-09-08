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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#include "GPIO.h"

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#endif

#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>

using namespace Zephyr;

extern const AP_HAL::HAL& hal;

/*
  map a user pin number to a Zephyr GPIO device and pin.
  AP pin encoding: bank = pin >> 5  (0=gpio1 … 12=gpio13)
                   zpin = pin & 0x1F
*/
#ifdef __ZEPHYR__
static bool gpio_by_pin_num(uint8_t ap_pin,
                             const struct device **dev_out,
                             gpio_pin_t *zpin_out)
{
    static const char *const names[] = {
        "gpio1",  "gpio2",  "gpio3",  "gpio4",  "gpio5",
        "gpio6",  "gpio7",  "gpio8",  "gpio9",  "gpio10",
        "gpio11", "gpio12", "gpio13"
    };
    uint8_t bank = ap_pin >> 5;
    if (bank >= ARRAY_SIZE(names)) {
        return false;
    }
    const struct device *dev = device_get_binding(names[bank]);
    if (!dev || !device_is_ready(dev)) {
        return false;
    }
    *dev_out  = dev;
    *zpin_out = (gpio_pin_t)(ap_pin & 0x1Fu);
    return true;
}

/* ISR-flood protection budget: 10000 interrupts per 100ms call to
   timer_tick() = 100k/s max per pin, matching AP_HAL_ChibiOS/GPIO.cpp's
   own quota exactly. */
static constexpr uint16_t GPIO_ISR_QUOTA = 10000U;
/* isr_disabled_ticks counts 100ms timer_tick() calls since a flooded pin
   was disabled; retry after this many while disarmed (100 * 100ms = 10s,
   same as ChibiOS). */
static constexpr uint8_t GPIO_ISR_RETRY_TICKS = 100U;

struct gpio_entry {
    struct gpio_callback cb;
    AP_HAL::GPIO::irq_handler_fn_t fn;
    AP_HAL::Proc proc;
    const struct device *dev;
    gpio_pin_t zpin;
    gpio_flags_t iflags;
    uint8_t ap_pin;
    bool used;

    /* ISR-flood protection state, mirroring AP_HAL_ChibiOS::gpio_entry. */
    uint16_t isr_quota;
    uint8_t isr_disabled_ticks;
};

static struct gpio_entry _gpio_tab[ZEPHYR_GPIO_MAX_IRQS];

static void _gpio_irq_handler(const struct device *dev,
                               struct gpio_callback *cb_ptr,
                               uint32_t changed_pins)
{
    struct gpio_entry *g = CONTAINER_OF(cb_ptr, struct gpio_entry, cb);

    /* Quota enforcement first, before touching the pin or calling out. */
    if (g->isr_quota > 0) {
        g->isr_quota--;
    }
    if (g->isr_quota == 0) {
        gpio_pin_interrupt_configure(dev, g->zpin, GPIO_INT_DISABLE);
        return;
    }

    gpio_pin_t zpin = (gpio_pin_t)(find_lsb_set((int)changed_pins) - 1);
    int state = gpio_pin_get(dev, zpin);
    uint32_t now_us = AP_HAL::micros();
    if (g->fn) {
        g->fn(g->ap_pin, state > 0, now_us);
    } else if (g->proc) {
        g->proc();
    }
}

static bool _attach_interrupt(uint8_t ap_pin,
                               AP_HAL::GPIO::irq_handler_fn_t fn,
                               AP_HAL::Proc proc,
                               AP_HAL::GPIO::INTERRUPT_TRIGGER_TYPE mode)
{
    const struct device *dev;
    gpio_pin_t zpin;
    if (!gpio_by_pin_num(ap_pin, &dev, &zpin)) {
        return false;
    }

    if (mode == AP_HAL::GPIO::INTERRUPT_NONE) {
        for (int i = 0; i < ZEPHYR_GPIO_MAX_IRQS; i++) {
            if (_gpio_tab[i].used && _gpio_tab[i].ap_pin == ap_pin) {
                gpio_pin_interrupt_configure(dev, zpin, GPIO_INT_DISABLE);
                gpio_remove_callback(dev, &_gpio_tab[i].cb);
                _gpio_tab[i].used = false;
            }
        }
        return true;
    }

    gpio_flags_t iflags;
    switch (mode) {
    case AP_HAL::GPIO::INTERRUPT_FALLING: iflags = GPIO_INT_EDGE_FALLING; break;
    case AP_HAL::GPIO::INTERRUPT_RISING:  iflags = GPIO_INT_EDGE_RISING;  break;
    case AP_HAL::GPIO::INTERRUPT_BOTH:    iflags = GPIO_INT_EDGE_BOTH;    break;
    default: return false;
    }

    int slot = -1;
    for (int i = 0; i < ZEPHYR_GPIO_MAX_IRQS; i++) {
        if (!_gpio_tab[i].used) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        return false;
    }

    _gpio_tab[slot].fn     = fn;
    _gpio_tab[slot].proc   = proc;
    _gpio_tab[slot].dev    = dev;
    _gpio_tab[slot].zpin   = zpin;
    _gpio_tab[slot].iflags = iflags;
    _gpio_tab[slot].ap_pin = ap_pin;
    _gpio_tab[slot].used   = true;
    /* Fresh quota on (re)attach - covers both a brand-new source and a
       flooded pin that was explicitly detached and reattached rather than
       left to timer_tick()'s own retry. */
    _gpio_tab[slot].isr_quota          = GPIO_ISR_QUOTA;
    _gpio_tab[slot].isr_disabled_ticks = 0;

    gpio_init_callback(&_gpio_tab[slot].cb, _gpio_irq_handler, BIT(zpin));
    gpio_add_callback(dev, &_gpio_tab[slot].cb);
    gpio_pin_interrupt_configure(dev, zpin, iflags);
    return true;
}
#endif  /* __ZEPHYR__ */

GPIO::GPIO()
{}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
#ifdef __ZEPHYR__
    const struct device *dev;
    gpio_pin_t zpin;
    if (!gpio_by_pin_num(pin, &dev, &zpin)) {
        return;
    }
    gpio_flags_t flags = (output == HAL_GPIO_OUTPUT)
                         ? (GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW)
                         : (GPIO_INPUT  | GPIO_DISCONNECTED);
    gpio_pin_configure(dev, zpin, flags);
#endif
}

uint8_t GPIO::read(uint8_t pin)
{
#ifdef __ZEPHYR__
    const struct device *dev;
    gpio_pin_t zpin;
    if (!gpio_by_pin_num(pin, &dev, &zpin)) {
        return 0;
    }
    int v = gpio_pin_get(dev, zpin);
    return (v > 0) ? 1u : 0u;
#else
    return 0;
#endif
}

void GPIO::write(uint8_t pin, uint8_t value)
{
#ifdef __ZEPHYR__
    const struct device *dev;
    gpio_pin_t zpin;
    if (!gpio_by_pin_num(pin, &dev, &zpin)) {
        return;
    }
    gpio_pin_set(dev, zpin, (int)value);
#endif
}

void GPIO::toggle(uint8_t pin)
{
#ifdef __ZEPHYR__
    const struct device *dev;
    gpio_pin_t zpin;
    if (!gpio_by_pin_num(pin, &dev, &zpin)) {
        return;
    }
    gpio_pin_toggle(dev, zpin);
#endif
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t pin)
{
    return NEW_NOTHROW DigitalSource((uint8_t)(pin & 0xFFu));
}

/*
   Attach an interrupt handler to a GPIO pin number.
*/
bool GPIO::attach_interrupt(uint8_t pin,
                            irq_handler_fn_t fn,
                            INTERRUPT_TRIGGER_TYPE mode)
{
#ifdef __ZEPHYR__
    return _attach_interrupt(pin, fn, nullptr, mode);
#else
    return false;
#endif
}

bool GPIO::attach_interrupt(uint8_t pin,
                            AP_HAL::Proc proc,
                            INTERRUPT_TRIGGER_TYPE mode)
{
#ifdef __ZEPHYR__
    return _attach_interrupt(pin, nullptr, proc, mode);
#else
    return false;
#endif
}

/* Reset interrupt quotas every 100 ms and recover any pin that was cut off. */
void GPIO::timer_tick(void)
{
#ifdef __ZEPHYR__
    for (int i = 0; i < ZEPHYR_GPIO_MAX_IRQS; i++) {
        struct gpio_entry *g = &_gpio_tab[i];
        if (!g->used) {
            continue;
        }
        if (g->isr_quota != 0) {
            // still healthy - refill for the next window
            g->isr_quota = GPIO_ISR_QUOTA;
            continue;
        }

        // quota hit 0 since the last check: this pin is currently disabled
        if (g->isr_disabled_ticks == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ISR flood on pin %u", g->ap_pin);
            if (hal.util->get_soft_armed()) {
                INTERNAL_ERROR(AP_InternalError::error_t::gpio_isr);
            }
        }
        if (hal.util->get_soft_armed()) {
            // don't start counting toward a retry until disarmed
            g->isr_disabled_ticks = 1;
            continue;
        }
        if (g->isr_disabled_ticks < UINT8_MAX) {
            g->isr_disabled_ticks++;
        }

        if (g->isr_disabled_ticks > GPIO_ISR_RETRY_TICKS) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Retrying pin %u after ISR flood", g->ap_pin);
            g->isr_quota = GPIO_ISR_QUOTA;
            gpio_pin_interrupt_configure(g->dev, g->zpin, g->iflags);
            g->isr_disabled_ticks = 0;
        }
    }
#endif
}

// Check for ISR floods
bool GPIO::arming_checks(size_t buflen, char *buffer) const
{
#ifdef __ZEPHYR__
    for (int i = 0; i < ZEPHYR_GPIO_MAX_IRQS; i++) {
        if (_gpio_tab[i].used && _gpio_tab[i].isr_disabled_ticks != 0) {
            hal.util->snprintf(buffer, buflen, "Pin %u disabled (ISR flood)", _gpio_tab[i].ap_pin);
            return false;
        }
    }
#endif
    return true;
}

bool GPIO::usb_connected(void)
{
#ifdef __ZEPHYR__
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usb_cdc_acm0), okay)
    const struct device *cdc = DEVICE_DT_GET(DT_NODELABEL(usb_cdc_acm0));
    if (!device_is_ready(cdc)) {
        return false;
    }
    uint32_t dtr = 0;
    if (uart_line_ctrl_get(cdc, UART_LINE_CTRL_DTR, &dtr) == 0) {
        return dtr != 0U;
    }
    return true;
#else
    return false;
#endif
#else
    return false;
#endif
}

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{
    hal.gpio->pinMode(_pin, output);
}

uint8_t DigitalSource::read()
{
    return hal.gpio->read(_pin);
}

void DigitalSource::write(uint8_t value)
{
    hal.gpio->write(_pin, value);
}

void DigitalSource::toggle()
{
    hal.gpio->toggle(_pin);
}

// check if a pin number is valid
bool GPIO::valid_pin(uint8_t pin) const
{
    uint8_t bank = pin >> 5;
    return bank < 8;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
