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

#include <hal.h>
#include "GPIO.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include "hwdef/common/stm32_util.h"
#include <AP_InternalError/AP_InternalError.h>
#ifndef HAL_BOOTLOADER_BUILD
#include <SRV_Channel/SRV_Channel.h>
#endif
#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Math/AP_Math.h>

using namespace ChibiOS;

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

// GPIO pin table from hwdef.dat
struct gpio_entry {
    uint8_t pin_num;
    bool enabled;
    uint8_t pwm_num;
    ioline_t pal_line;
    AP_HAL::GPIO::irq_handler_fn_t fn; // callback for GPIO interface
    thread_reference_t thd_wait;
    bool is_input;
    uint8_t mode;
    uint16_t isr_quota;
    uint8_t isr_disabled_ticks;
    AP_HAL::GPIO::INTERRUPT_TRIGGER_TYPE isr_mode;
};

#ifdef HAL_GPIO_PINS
#define HAVE_GPIO_PINS 1
static struct gpio_entry _gpio_tab[] = HAL_GPIO_PINS;
#else
#define HAVE_GPIO_PINS 0
#endif


/*
  map a user pin number to a GPIO table entry
 */
static struct gpio_entry *gpio_by_pin_num(uint8_t pin_num, bool check_enabled=true)
{
#if HAVE_GPIO_PINS
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        const auto &t = _gpio_tab[i];
        if (pin_num == t.pin_num) {
            if (check_enabled && t.pwm_num != 0 && !t.enabled) {
                return NULL;
            }
            return &_gpio_tab[i];
        }
    }
#endif
    return NULL;
}

static void pal_interrupt_cb(void *arg);
static void pal_interrupt_cb_functor(void *arg);

GPIO::GPIO()
{}

void GPIO::init()
{
#if !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !defined(HAL_BOOTLOADER_BUILD)
#if HAL_WITH_IO_MCU || HAVE_GPIO_PINS
    uint8_t chan_offset = 0;
#endif
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        uint8_t GPIO_mask = 0;
        for (uint8_t i=0; i<8; i++) {
            if (SRV_Channels::is_GPIO(i)) {
                GPIO_mask |= 1U << i;
            }
        }
        iomcu.set_GPIO_mask(GPIO_mask);
        chan_offset = 8;
    }
#endif
    // auto-disable pins being used for PWM output
#if HAVE_GPIO_PINS
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        struct gpio_entry *g = &_gpio_tab[i];
        if (g->pwm_num != 0) {
            g->enabled = SRV_Channels::is_GPIO((g->pwm_num-1)+chan_offset);
        }
    }
#endif // HAVE_GPIO_PINS
#endif // HAL_BOOTLOADER_BUILD
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
            if (alt.periph_type == PERIPH_TYPE::GPIO) {
                // enable pin in GPIO table
#if HAVE_GPIO_PINS
                for (uint8_t j=0; j<ARRAY_SIZE(_gpio_tab); j++) {
                    struct gpio_entry *g = &_gpio_tab[j];
                    if (g->pal_line == alt.line) {
                        g->enabled = true;
                        break;
                    }
                }
#endif // HAVE_GPIO_PINS
                continue;
            }
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
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F4) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
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
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled() && iomcu.valid_GPIO_pin(pin)) {
        iomcu.write_GPIO(pin, value);
        return;
    }
#endif
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
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled() && iomcu.valid_GPIO_pin(pin)) {
        iomcu.toggle_GPIO(pin);
        return;
    }
#endif
    struct gpio_entry *g = gpio_by_pin_num(pin);
    if (g) {
        palToggleLine(g->pal_line);
    }
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t pin)
{
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled() && iomcu.valid_GPIO_pin(pin)) {
        return new IOMCU_DigitalSource(pin);
    }
#endif
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
    g->isr_disabled_ticks = 0;
    g->isr_quota = 0;
    if (!_attach_interrupt(g->pal_line,
                           palcallback_t(fn?pal_interrupt_cb_functor:nullptr),
                           g,
                           mode)) {
        return false;
    }
    g->fn = fn;
    g->isr_mode = mode;
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
    g->isr_disabled_ticks = 0;
    g->isr_quota = 0;
    g->isr_mode = mode;
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

#if HAL_WITH_IO_MCU
IOMCU_DigitalSource::IOMCU_DigitalSource(uint8_t _pin) :
    pin(_pin)
{}

void IOMCU_DigitalSource::write(uint8_t value)
{
    iomcu.write_GPIO(pin, value);
}

void IOMCU_DigitalSource::toggle()
{
    iomcu.toggle_GPIO(pin);
}
#endif // HAL_WITH_IO_MCU

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
  block waiting for a pin to change. Return true on pin change, false on timeout
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

    // don't allow for very long timeouts, or below the delta
    timeout_us = constrain_uint32(TIME_US2I(timeout_us), CH_CFG_ST_TIMEDELTA, TIME_US2I(30000U));

    msg_t msg = osalThreadSuspendTimeoutS(&g->thd_wait, timeout_us);
    _attach_interruptI(g->pal_line,
                       palcallback_t(nullptr),
                       nullptr,
                       mode);
    osalSysUnlock();

    return msg == MSG_OK;
}

// check if a pin number is valid
bool GPIO::valid_pin(uint8_t pin) const
{
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled() && iomcu.valid_GPIO_pin(pin)) {
        return true;
    }
#endif
    return gpio_by_pin_num(pin) != nullptr;
}

// return servo channel associated with GPIO pin.  Returns true on success and fills in servo_ch argument
// servo_ch uses zero-based indexing
bool GPIO::pin_to_servo_channel(uint8_t pin, uint8_t& servo_ch) const
{
#if HAL_WITH_IO_MCU || HAVE_GPIO_PINS
    uint8_t fmu_chan_offset = 0;
#endif
#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        // check if this is one of the main pins
        uint8_t main_servo_ch = pin;
        if (iomcu.convert_pin_number(main_servo_ch)) {
            servo_ch = main_servo_ch;
            return true;
        }
        // with IOMCU the local (FMU) channels start at 8
        fmu_chan_offset = 8;
    }
#endif

#if HAVE_GPIO_PINS
    // search _gpio_tab for matching pin
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        if (_gpio_tab[i].pin_num == pin) {
            if (_gpio_tab[i].pwm_num == 0) {
                return false;
            }
            servo_ch = _gpio_tab[i].pwm_num-1+fmu_chan_offset;
            return true;
        }
    }
#endif // HAVE_GPIO_PINS
    return false;
}

#if defined(STM32F7) || defined(STM32H7) || defined(STM32F4) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)

// allow for save and restore of pin settings
bool GPIO::get_mode(uint8_t pin, uint32_t &mode)
{
    auto *p = gpio_by_pin_num(pin);
    if (!p) {
        return false;
    }
    mode = uint32_t(palReadLineMode(p->pal_line));
    return true;
}

void GPIO::set_mode(uint8_t pin, uint32_t mode)
{
    auto *p = gpio_by_pin_num(pin);
    if (p) {
        palSetLineMode(p->pal_line, ioline_t(mode));
    }
}
#endif

#ifndef IOMCU_FW
/*
  timer to setup interrupt quotas for a 100ms period from
  monitor thread
*/
void GPIO::timer_tick()
{
    // allow 100k interrupts/second max for GPIO interrupt sources, which is
    // 10k per 100ms call to timer_tick()
#if HAVE_GPIO_PINS
    const uint16_t quota = 10000U;
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        if (_gpio_tab[i].isr_quota != 1) {
            // Reset quota for next tick
            _gpio_tab[i].isr_quota = quota;
            continue;
        }
        // we ran out of ISR quota for this pin since the last
        // check. This is not really an internal error, but we use
        // INTERNAL_ERROR() to get the reporting mechanism

        if (_gpio_tab[i].isr_disabled_ticks == 0) {
#ifndef HAL_NO_UARTDRIVER
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"ISR flood on pin %u", _gpio_tab[i].pin_num);
#endif
            // Only trigger internal error if armed
            if (hal.util->get_soft_armed()) {
                INTERNAL_ERROR(AP_InternalError::error_t::gpio_isr);
            }
        }
        if (hal.util->get_soft_armed()) {
            // Don't start counting until disarmed
            _gpio_tab[i].isr_disabled_ticks = 1;
            continue;
        }

        // Increment disabled ticks, don't wrap
        if (_gpio_tab[i].isr_disabled_ticks < UINT8_MAX) {
            _gpio_tab[i].isr_disabled_ticks++;
        }

        // 100 * 100ms = 10 seconds
        const uint8_t ISR_retry_ticks = 100U;
        if ((_gpio_tab[i].isr_disabled_ticks > ISR_retry_ticks) && (_gpio_tab[i].fn != nullptr)) {
            // Try re-enabling
#ifndef HAL_NO_UARTDRIVER
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Retrying pin %d after ISR flood", _gpio_tab[i].pin_num);
#endif
            if (attach_interrupt(_gpio_tab[i].pin_num, _gpio_tab[i].fn, _gpio_tab[i].isr_mode)) {
                // Success, reset quota
                _gpio_tab[i].isr_quota = quota;
            } else {
                // Failed, reset disabled count to try again later
                _gpio_tab[i].isr_disabled_ticks = 1;
            }
        }
    }
#endif // HAVE_GPIO_PINS
}

// Check for ISR floods
bool GPIO::arming_checks(size_t buflen, char *buffer) const
{
#if HAVE_GPIO_PINS
    for (uint8_t i=0; i<ARRAY_SIZE(_gpio_tab); i++) {
        if (_gpio_tab[i].isr_disabled_ticks != 0) {
            hal.util->snprintf(buffer, buflen, "Pin %u disabled (ISR flood)", _gpio_tab[i].pin_num);
            return false;
        }
    }
#endif // HAVE_GPIO_PINS
    return true;
}
#endif // IOMCU_FW
