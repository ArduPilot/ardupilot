#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AnalogIn_Navio.h"

#include <cstdlib>
#include <unistd.h>
#include <cstdio>
#include <errno.h>

extern const AP_HAL::HAL& hal;

union pwm_params {
    char channel[sizeof("XXXX")];
};

#define RCIN_BASE_PATH "/sys/kernel/rcio/adc"
#define RCIN_PATH_MAX (sizeof(RCIN_BASE_PATH) + sizeof(pwm_params) - 1)

void NavioAnalogSource::set_channel(int16_t pin)
{
    char *channel_path;

    if (pin != -1 && asprintf(&channel_path, "%s/ch%d", RCIN_BASE_PATH, pin) == -1) {
        AP_HAL::panic("asprintf\n");
    }

    if (_fd >= 0) {
        ::close(_fd);
    }

    _fd = ::open(channel_path, O_RDONLY);

    if (_fd < 0) {
        hal.console->printf("%s not opened: %s", channel_path, strerror(errno));
    }

    free(channel_path);
}

NavioAnalogSource::NavioAnalogSource(int16_t pin):
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
    char buffer[RCIN_PATH_MAX];

    int ret;

    ret = ::pread(_fd, buffer, sizeof(buffer), 0);

    if (ret < 0) {
        perror("pread");
    }

    _value = ((float) atoi(buffer)) / 1000;

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
        if (_channels[j] == NULL) {
            _channels[j] = new NavioAnalogSource(pin);
            return _channels[j];
        }
    }

    hal.console->println("Out of analog channels");
    return NULL;
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
