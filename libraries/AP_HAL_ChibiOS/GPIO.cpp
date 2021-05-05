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

#include <AP_BoardConfig/AP_BoardConfig.h>
#include "hwdef/common/stm32_util.h"
#include <AP_InternalError/AP_InternalError.h>
#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif

using namespace ChibiOS;

// GPIO pin table from hwdef.dat
static struct gpio_entry {
    uint8_t pin_num;
    bool enabled;
    uint8_t pwm_num;
    ioline_t pal_line;
    AP_HAL::GPIO::irq_handler_fn_t fn; // callback for GPIO interface
    bool is_input;
    uint8_t mode;
    thread_reference_t thd_wait;
    uint16_t isr_quota;
} _gpio_tab[] = HAL_GPIO_PINS;

#define NUM_PINS ARRAY_SIZE(_gpio_tab)
#define PIN_ENABLED(pin) ((pin)<NUM_PINS && _gpio_tab[pin].enabled)

/*
  map a user pin number to a GPIO table entry
 */
static struct gpio_entry *gpio_by_pin_num(uint8_t pin_num, bool check_enabled=true)
{
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        if (pin_num == _gpio_tab[i].pin_num) {
            if (check_enabled && !_gpio_tab[i].enabled) {
                return NULL;
            }
            return &_gpio_tab[i];
        }
    }
    return NULL;
}

static void pal_interrupt_cb(void *arg);
static void pal_interrupt_cb_functor(void *arg);

GPIO::GPIO()
{}

void GPIO::init()
{
    // auto-disable pins being used for PWM output based on BRD_PWM_COUNT parameter
    uint8_t pwm_count = AP_BoardConfig::get_pwm_count();
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        struct gpio_entry *g = &_gpio_tab[i];
        if (g->pwm_num != 0) {
            g->enabled = g->pwm_num > pwm_count;
        }
    }
#ifdef HAL_PIN_ALT_CONFIG
    setup_alt_config();
#endif
}

#ifdef HAL_PIN_ALT_CONFIG
// chosen alternative config
uint8_t GPIO::alt_config;

/*
  alternative config table, selected using BRD_ALT_CONFIG
 */
static const struct alt_config {
    uint8_t alternate;
    uint16_t mode;
    ioline_t line;
    PERIPH_TYPE periph_type;
    uint8_t periph_instance;
} alternate_config[] HAL_PIN_ALT_CONFIG;

/*
  change pin configuration based on ALT() lines in hwdef.dat
 */
void GPIO::setup_alt_config(void)
{
    AP_BoardConfig *bc = AP::boardConfig();
    if (!bc) {
        return;
    }
    alt_config = bc->get_alt_config();
    if (alt_config == 0) {
        // use defaults
        return;
    }
    for (uint8_t i=0; i<ARRAY_SIZE(alternate_config); i++) {
        const struct alt_config &alt = alternate_config[i];
        if (alt_config == alt.alternate) {
            const iomode_t mode = alt.mode & ~PAL_STM32_HIGH;
            const uint8_t odr = (alt.mode & PAL_STM32_HIGH)?1:0;
            palSetLineMode(alt.line, mode);
            palWriteLine(alt.line, odr);
        }
    }
}
#endif // HAL_PIN_ALT_CONFIG

/*
  resolve an ioline_t to take account of alternative
  configurations. This allows drivers to get the right ioline_t for an
  alternative config. Note that this may return 0, meaning the pin is
  not mapped to this peripheral in the active config
*/
ioline_t GPIO::resolve_alt_config(ioline_t base, PERIPH_TYPE ptype, uint8_t instance)
{
#ifdef HAL_PIN_ALT_CONFIG
    if (alt_config == 0) {
        // unchanged
        return base;
    }
    for (uint8_t i=0; i<ARRAY_SIZE(alternate_config); i++) {
        const struct alt_config &alt = alternate_config[i];
        if (alt_config == alt.alternate) {
            if (ptype == alt.periph_type && instance == alt.periph_instance) {
                // we've reconfigured this peripheral with a different line
                return alt.line;
            }
        }
    }
    // now search for pins that have been configured off via BRD_ALT_CONFIG
    for (uint8_t i=0; i<ARRAY_SIZE(alternate_config); i++) {
        const struct alt_config &alt = alternate_config[i];
        if (alt_config == alt.alternate) {
            if (alt.line == base) {
                // this line is no longer available in this config
                return 0;
            }
        }
    }
#endif
    return base;
}


void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        if (!output && g->is_input &&
            (g->mode == PAL_MODE_INPUT_PULLUP ||
             g->mode == PAL_MODE_INPUT_PULLDOWN)) {
            // already set
            return;
        }
        g->mode = output?PAL_MODE_OUTPUT_PUSHPULL:PAL_MODE_INPUT;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F4) || defined(STM32G4)
        if (g->mode == PAL_MODE_OUTPUT_PUSHPULL) {
            // retain OPENDRAIN if already set
            iomode_t old_mode = palReadLineMode(g->pal_line);
            if ((old_mode & PAL_MODE_OUTPUT_OPENDRAIN) == PAL_MODE_OUTPUT_OPENDRAIN) {
                g->mode = PAL_MODE_OUTPUT_OPENDRAIN;
            }
        }
#endif
        palSetLineMode(g->pal_line, g->mode);
        g->is_input = !output;
    }
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
        if (g->is_input) {
            // control pullup/pulldown
            g->mode = value==1?PAL_MODE_INPUT_PULLUP:PAL_MODE_INPUT_PULLDOWN;
            palSetLineMode(g->pal_line, g->mode);
        } else if (value == PAL_LOW) {
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
bool GPIO::attach_interrupt(uint8_t pin,
                            irq_handler_fn_t fn,
                            INTERRUPT_TRIGGER_TYPE mode)
{
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return false;
    }
    if (!_attach_interrupt(g->pal_line,
                           palcallback_t(fn?pal_interrupt_cb_functor:nullptr),
                           g,
                           mode)) {
        return false;
    }
    g->fn = fn;
    return true;
}

/*
   Attach an interrupt handler to ioline_t
 */
bool GPIO::_attach_interrupt(ioline_t line, AP_HAL::Proc p, uint8_t mode)
{
    return _attach_interrupt(line, palcallback_t(p?pal_interrupt_cb:nullptr), (void*)p, mode);
}

bool GPIO::attach_interrupt(uint8_t pin,
                            AP_HAL::Proc proc,
                            INTERRUPT_TRIGGER_TYPE mode) {
    struct gpio_entry *g = gpio_by_pin_num(pin, false);
    if (!g) {
        return false;
    }
    return _attach_interrupt(g->pal_line, proc, mode);
}

bool GPIO::_attach_interruptI(ioline_t line, palcallback_t cb, void *p, uint8_t mode)
{
    uint32_t chmode = 0;
    switch(mode) {
        case INTERRUPT_FALLING:
            chmode = PAL_EVENT_MODE_FALLING_EDGE;
            break;
        case INTERRUPT_RISING:
            chmode = PAL_EVENT_MODE_RISING_EDGE;
            break;
        case INTERRUPT_BOTH:
            chmode = PAL_EVENT_MODE_BOTH_EDGES;
            break;
        default:
            if (p) {
                return false;
            }
            break;
    }

    palevent_t *pep = pal_lld_get_line_event(line);
    if (pep->cb && p != nullptr) {
        // the pad is already being used for a callback
        return false;
    }

    if (!p) {
        chmode = PAL_EVENT_MODE_DISABLED;
    }

    palDisableLineEventI(line);
    palSetLineCallbackI(line, cb, p);
    palEnableLineEventI(line, chmode);

    return true;
}

bool GPIO::_attach_interrupt(ioline_t line, palcallback_t cb, void *p, uint8_t mode)
{
    osalSysLock();
    bool ret = _attach_interruptI(line, cb, p, mode);
    osalSysUnlock();
    return ret;
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

static void pal_interrupt_cb(void *arg)
{
    if (arg != nullptr) {
        ((AP_HAL::Proc)arg)();
    }
}

static void pal_interrupt_cb_functor(void *arg)
{
    const uint32_t now = AP_HAL::micros();

    struct gpio_entry *g = (gpio_entry *)arg;
    if (g == nullptr) {
        // what?
        return;
    }
    if (!(g->fn)) {
        return;
    }
    if (g->isr_quota >= 1) {
        /*
          we have an interrupt quota enabled for this pin. If the
          quota remaining drops to 1 without it being refreshed in
          timer_tick then we disable the interrupt source. This is to
          prevent CPU overload due to very high GPIO interrupt counts
         */
        if (g->isr_quota == 1) {
            osalSysLockFromISR();
            palDisableLineEventI(g->pal_line);
            osalSysUnlockFromISR();
            return;
        }
        g->isr_quota--;
    }
    (g->fn)(g->pin_num, palReadLine(g->pal_line), now);
}

/*
  handle interrupt from pin change for wait_pin()
 */
static void pal_interrupt_wait(void *arg)
{
    osalSysLockFromISR();
    struct gpio_entry *g = (gpio_entry *)arg;
    if (g == nullptr || g->thd_wait == nullptr) {
        osalSysUnlockFromISR();
        return;
    }
    osalThreadResumeI(&g->thd_wait, MSG_OK);
    osalSysUnlockFromISR();
}

/*
  block waiting for a pin to change. A timeout of 0 means wait
  forever. Return true on pin change, false on timeout
*/
bool GPIO::wait_pin(uint8_t pin, INTERRUPT_TRIGGER_TYPE mode, uint32_t timeout_us)
{
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (!g) {
        return false;
    }

    osalSysLock();
    if (g->thd_wait) {
        // only allow single waiter
        osalSysUnlock();
        return false;
    }

    if (!_attach_interruptI(g->pal_line,
                           palcallback_t(pal_interrupt_wait),
                           g,
                           mode)) {
        osalSysUnlock();
        return false;
    }
        
    msg_t msg = osalThreadSuspendTimeoutS(&g->thd_wait, TIME_US2I(timeout_us));
    _attach_interruptI(g->pal_line,
                       palcallback_t(nullptr),
                       nullptr,
                       mode);
    osalSysUnlock();

    return msg == MSG_OK;
}

#ifndef IOMCU_FW
/*
  timer to setup interrupt quotas for a 100ms period from
  monitor thread
*/
void GPIO::timer_tick()
{
    // allow 100k interrupts/second max for GPIO interrupt sources, which is
    // 10k per 100ms call to timer_tick()
    const uint16_t quota = 10000U;
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        if (_gpio_tab[i].isr_quota == 1) {
            // we ran out of ISR quota for this pin since the last
            // check. This is not really an internal error, but we use
            // INTERNAL_ERROR() to get the reporting mechanism
#ifndef HAL_NO_UARTDRIVER
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"ISR flood on pin %u", _gpio_tab[i].pin_num);
#endif
            INTERNAL_ERROR(AP_InternalError::error_t::gpio_isr);
        }
        _gpio_tab[i].isr_quota = quota;
    }
}
#endif // IOMCU_FW
