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

#include "AnalogIn.h"

#ifdef __ZEPHYR__
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <GCS_MAVLink/GCS_config.h>   // defines HAL_GCS_ENABLED (no version.h)
// GCS_MAVLink.h supplies the MAV_POWER_STATUS_* enums used below.
#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS_MAVLink.h>
#endif
#endif

using namespace Zephyr;

AnalogSource::AnalogSource(AnalogIn &parent, uint8_t pin) :
    _parent(parent),
    _pin(pin)
{
}

float AnalogSource::read_average()
{
    return _parent.read_pin_raw(_pin);
}

float AnalogSource::read_latest()
{
    return _parent.read_pin_raw(_pin);
}

bool AnalogIn::valid_analog_pin(uint16_t pin) const
{
    return pin < NUM_ANALOG_PINS;
}

float AnalogSource::voltage_average()
{
    return _parent.read_pin_voltage(_pin);
}

float AnalogSource::voltage_average_ratiometric()
{
    return _parent.read_pin_voltage(_pin);
}

float AnalogSource::voltage_latest()
{
    return _parent.read_pin_voltage(_pin);
}

bool AnalogSource::set_pin(uint8_t p)
{
    if (!_parent.valid_analog_pin(p)) {
        return false;
    }
    _pin = p;
    return true;
}

AnalogIn::AnalogIn()
{
}

void AnalogIn::init()
{
#ifdef __ZEPHYR__
    _oc_init();
/* NXP names its converters lpadcN, STM32 names them adcN. Take whichever the
   board's devicetree actually has - this used to look only for lpadcN, so on
   an STM32 board _adc1 stayed null, _adc_ready stayed false and every analog
   read returned 0 regardless of what was wired. */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpadc1), okay)
    _adc1 = DEVICE_DT_GET(DT_NODELABEL(lpadc1));
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(adc1), okay)
    _adc1 = DEVICE_DT_GET(DT_NODELABEL(adc1));
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpadc2), okay)
    _adc2 = DEVICE_DT_GET(DT_NODELABEL(lpadc2));
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(adc2), okay)
    _adc2 = DEVICE_DT_GET(DT_NODELABEL(adc2));
#endif

    /* LPADC2 is optional: a board with only LPADC1 wired leaves the pointer null,
     * so every use must check it. */
    _adc_ready = (_adc1 != nullptr && device_is_ready(_adc1));

    if (!_adc_ready) {
        return;
    }

    struct adc_channel_cfg cfg = {};
    cfg.gain = ADC_GAIN_1;
    cfg.reference = ADC_REF_INTERNAL;
    cfg.acquisition_time = ADC_ACQ_TIME_DEFAULT;
    cfg.differential = 0;

    const bool adc2_ready = (_adc2 != nullptr && device_is_ready(_adc2));

    /* Configure every channel the board can ask for. On NXP LPADC the AP pin
       number is an index into two 2-channel converters, so channels 0..1 are
       all that exist. On STM32 the AP pin number IS the ADC input number -
       CubeOrange's HAL_BATT_VOLT_PIN 14 means ADC1_INP14 on PA2 - so the
       usable range runs to AP_ANALOG_MAX_CHANNEL. */
    for (uint8_t ch = 0; ch <= AP_ANALOG_MAX_CHANNEL; ch++) {
        cfg.channel_id = ch;
        (void)adc_channel_setup(_adc1, &cfg);
        if (adc2_ready) {
            (void)adc_channel_setup(_adc2, &cfg);
        }
    }
#endif
}

float AnalogIn::read_pin_raw(uint8_t pin)
{
#ifdef __ZEPHYR__
    if (!_adc_ready || !valid_analog_pin(pin)) {
        return 0.0f;
    }

    /* Pin -> (converter, channel), per SoC.
       NXP LPADC: two converters of two channels, so the AP pin number indexes
       across them - pin 0,1 are lpadc1 ch0,ch1 and pin 2,3 are lpadc2.
       STM32: one converter addressed by input number, and the AP pin number IS
       that input number. CubeOrangeZephyr's HAL_BATT_VOLT_PIN 14 / CURR_PIN 15
       are ADC1_INP14 on PA2 and INP15 on PA3, exactly as the ChibiOS board
       derives them from its "PA2 BATT_VOLTAGE_SENS ADC1" line. Forcing the
       NXP shape onto STM32 sent pin 14 to a non-existent _adc2 channel 0.

       _adc2 is legitimately absent on boards that bring out only one converter
       (mr_vmu_rt1176 is one), so the null check stays either way - without it
       a BATT_VOLT_PIN of 2 or 3 dereferenced null and hard-faulted. */
#if AP_ANALOG_PIN_IS_ADC_CHANNEL
    const struct device *adc = _adc1;
    const uint8_t channel_id = pin;
#else
    const struct device *adc = (pin < 2U) ? _adc1 : _adc2;
    const uint8_t channel_id = pin % 2U;
#endif
    if (adc == nullptr || !device_is_ready(adc)) {
        return 0.0f;
    }

    int16_t sample = 0;
    struct adc_sequence seq = {};
    seq.channels = BIT(channel_id);
    seq.buffer = &sample;
    seq.buffer_size = sizeof(sample);
    seq.resolution = 12;

    if (adc_read(adc, &seq) != 0) {
        return 0.0f;
    }

    if (sample < 0) {
        sample = 0;
    }

    return (float)sample;
#else
    (void)pin;
    return 0.0f;
#endif
}

float AnalogIn::read_pin_voltage(uint8_t pin)
{
    const float raw = read_pin_raw(pin);
    return (raw * ADC_REF_VOLTAGE) / (float)ADC_MAX_COUNTS;
}

AP_HAL::AnalogSource *AnalogIn::channel(int16_t n)
{
    if (n < 0 || n >= NUM_ANALOG_PINS) {
        return nullptr;
    }

    if (_sources[n] == nullptr) {
        _sources[n] = NEW_NOTHROW AnalogSource(*this, n);
    }

    return _sources[n];
}

float AnalogIn::board_voltage(void)
{
    return 5.0f;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR

#ifdef __ZEPHYR__
/* Over-current monitoring for the two switched 5V rails. */
void AnalogIn::_oc_init(void)
{
    _oc_gpio = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpio1));
    if (_oc_gpio == nullptr || !device_is_ready(_oc_gpio)) {
        return;
    }
    /* Inputs only. gpio1 is in the low GPIO_MUX group, whose selection
       registers already read 0 (GPIO1 rather than CM7_GPIO1) - the same
       CM7_GPIO trap that hid the SPI chip selects would apply here otherwise. */
    if (gpio_pin_configure(_oc_gpio, OC_PERIPH_PIN, GPIO_INPUT) != 0 ||
        gpio_pin_configure(_oc_gpio, OC_HIPOWER_PIN, GPIO_INPUT) != 0) {
        return;
    }
    _oc_ready = true;
}

uint16_t AnalogIn::power_status_flags(void)
{
    if (!_oc_ready) {
        return 0;
    }

    uint16_t flags = 0;

    /* Active LOW: 0 means the switch is reporting a fault. gpio_pin_get()
       returns <0 on error - treat that as "no fault" rather than inventing an
       over-current, since a read failure is not evidence of one. */
#if HAL_GCS_ENABLED
    const int periph = gpio_pin_get(_oc_gpio, OC_PERIPH_PIN);
    if (periph == 0) {
        flags |= MAV_POWER_STATUS_PERIPH_OVERCURRENT;
    }
    const int hipower = gpio_pin_get(_oc_gpio, OC_HIPOWER_PIN);
    if (hipower == 0) {
        flags |= MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
    }
#endif  // HAL_GCS_ENABLED (bootloader: no GCS, MAV_POWER_STATUS_* undefined)

    if (flags != _power_flags) {
        _power_flags = flags;
    }
    _accumulated_power_flags |= flags;

    return flags;
}

uint16_t AnalogIn::accumulated_power_status_flags(void) const
{
    return _accumulated_power_flags;
}
#else
uint16_t AnalogIn::power_status_flags(void) { return 0; }
uint16_t AnalogIn::accumulated_power_status_flags(void) const { return 0; }
#endif  // __ZEPHYR__
