#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AnalogIn_Navio.h"

#include <cstdlib>
#include <unistd.h>
#include <cstdio>
#include <errno.h>

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

union adc_params {
    char channel[sizeof("XXXX")];
};

#define ADC_BASE_PATH "/sys/kernel/rcio/adc"
#define ADC_PATH_MAX (sizeof(ADC_BASE_PATH) + sizeof(adc_params) - 1)

void NavioAnalogSource::set_channel(uint8_t pin)
{
    char *channel_path;

    if (pin == ANALOG_INPUT_NONE) {
        return;
    }

    if (asprintf(&channel_path, "%s/ch%d", ADC_BASE_PATH, pin) == -1) {
        AP_HAL::panic("asprintf failed\n");
    }

    if (_fd >= 0) {
        ::close(_fd);
    }

    _fd = ::open(channel_path, O_RDONLY);

    if (_fd < 0) {
        hal.console->printf("%s not opened: %s\n", channel_path, strerror(errno));
    }

    free(channel_path);
}

NavioAnalogSource::NavioAnalogSource(uint8_t pin):
    _pin(pin)
{
    set_channel(pin);
}

void NavioAnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }

    set_channel(pin);

    _pin = pin;
}

float NavioAnalogSource::read_average()
{
    return read_latest();
}

float NavioAnalogSource::read_latest()
{
    return _value;
}

float NavioAnalogSource::voltage_average()
{
    char buffer[ADC_PATH_MAX];

    if (pread(_fd, buffer, sizeof(buffer), 0) <= 0) {
    /* Don't log fails since this could spam the console */
        return -1.0f;
    }

    _value = ((float) atoi(buffer)) / 1000.0f;

    return _value;
}

float NavioAnalogSource::voltage_latest()
{
    return _value;
}

float NavioAnalogSource::voltage_average_ratiometric()
{
    return _value;
}

extern const AP_HAL::HAL& hal;

NavioAnalogIn::NavioAnalogIn()
{
    _channels_number = NAVIO_ADC_MAX_CHANNELS;
}

float NavioAnalogIn::board_voltage(void)
{
    auto voltage = _board_voltage_pin->voltage_average();
    return voltage;
}
float NavioAnalogIn::servorail_voltage(void)
{
    auto voltage = _servorail_pin->voltage_average();
    return voltage;
}

AP_HAL::AnalogSource* NavioAnalogIn::channel(int16_t pin)
{
    for (uint8_t j = 0; j < _channels_number; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = new NavioAnalogSource(pin);
            return _channels[j];
        }
    }

    hal.console->println("Out of analog channels");
    return nullptr;
}

void NavioAnalogIn::init()
{
    _board_voltage_pin = channel(0);
    _servorail_pin = channel(1);

    hal.scheduler->suspend_timer_procs();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&NavioAnalogIn::_update, void));
    hal.scheduler->resume_timer_procs();
}

void NavioAnalogIn::_update()
{
    if (AP_HAL::micros() - _last_update_timestamp < 100000) {
        return;
    }

    _last_update_timestamp = AP_HAL::micros();
}

#endif
