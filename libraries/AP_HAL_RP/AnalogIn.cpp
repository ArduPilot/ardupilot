#include "AnalogIn.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

using namespace RP;

extern const AP_HAL::HAL& hal;

// --- AnalogSource implementation ---

AnalogSource::AnalogSource(uint8_t pin) :
    _pin(pin),
    _latest_value(0.0f),
    _sum(0.0f),
    _count(0)
{}

float AnalogSource::read_latest() {
    // Reading floats on 32-bit architectures is usually atomic,
    // so a semaphore is not critical here.
    return _latest_value;
}

float AnalogSource::read_average() {
    // We protect access with a semaphore because we modify _sum and _count
    _semaphore.take_blocking();

    float avg = 0.0f;
    if (_count > 0) {
        avg = _sum / _count;
        _sum = 0;
        _count = 0;
    } else {
        avg = _latest_value;
    }

    _semaphore.give();
    return avg;
}

bool AnalogSource::set_pin(uint8_t p) {
    _pin = p;
    return true;
}

void AnalogSource::add_sample(uint16_t sample) {
    _semaphore.take_blocking();

    _latest_value = (float)sample;
    _sum += _latest_value;
    _count++;

    _semaphore.give();
}

float AnalogSource::voltage_latest() {
    // RP2350 ADC resolution - 12 bits (4096 values)
    return read_latest() * (3.3f / 4096.0f);
}

float AnalogSource::voltage_average() {
    return read_average() * (3.3f / 4096.0f);
}

float AnalogSource::voltage_average_ratiometric() {
    return voltage_average();
}

// --- AnalogIn implementation ---

AnalogSource* AnalogIn::_channels[5] = { nullptr };

void AnalogIn::init() {
    adc_init();

    // Initialize standard ADC pins (GPIO 26-29)
    for (uint8_t i = 26; i <= 29; i++) {
        adc_gpio_init(i);
    }

    // Turn on the internal temperature sensor
    adc_set_temp_sensor_enabled(true);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AnalogIn::_update, void));
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n) {
    if (n < 0 || n > 4) return nullptr;
    if (_channels[n] == nullptr) {
        _channels[n] = new AnalogSource(n);
    }
    return _channels[n];
}

bool AnalogIn::valid_analog_pin(uint16_t pin) const {
    return (pin >= 26 && pin <= 29) || pin == 4;
}

void AnalogIn::_update() {
    for (uint8_t i = 0; i < 5; i++) {
        if (_channels[i]) {
            // Select the ADC input (0-3 external, 4 temperature sensor)
            adc_select_input(i);
            uint16_t raw = adc_read();
            _channels[i]->add_sample(raw);
        }
    }
}
