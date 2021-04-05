#include <cstdio>
#include <cstdlib>
#include <errno.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

#include "AnalogIn_Navio2.h"

extern const AP_HAL::HAL& hal;

#define ADC_BASE_PATH "/sys/kernel/rcio/adc"

void AnalogSource_Navio2::set_channel(uint8_t pin)
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

    _fd = ::open(channel_path, O_RDONLY|O_CLOEXEC);

    if (_fd < 0) {
        hal.console->printf("%s not opened: %s\n", channel_path, strerror(errno));
    }

    free(channel_path);
}

AnalogSource_Navio2::AnalogSource_Navio2(uint8_t pin)
    : _pin(pin)
{
    set_channel(pin);
}

void AnalogSource_Navio2::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }

    set_channel(pin);

    _pin = pin;
}

float AnalogSource_Navio2::read_average()
{
    return read_latest();
}

float AnalogSource_Navio2::read_latest()
{
    return voltage_average();
}

float AnalogSource_Navio2::voltage_average()
{
    char buffer[12];

    if (pread(_fd, buffer, sizeof(buffer) - 1, 0) <= 0) {
        /* Don't log fails since this could spam the console */
        return -1.0f;
    }

    /* Avoid overriding NULL char at the end of the string */
    buffer[sizeof(buffer) - 1] = '\0';

    _value = atoi(buffer) / 1000.0f;

    return _value;
}

float AnalogSource_Navio2::voltage_latest()
{
    read_latest();
    return _value;
}

float AnalogSource_Navio2::voltage_average_ratiometric()
{
    return voltage_average();
}

AnalogIn_Navio2::AnalogIn_Navio2()
{
}

float AnalogIn_Navio2::board_voltage(void)
{
    return _board_voltage_pin->voltage_average();
}

float AnalogIn_Navio2::servorail_voltage(void)
{
    return _servorail_pin->voltage_average();
}

AP_HAL::AnalogSource *AnalogIn_Navio2::channel(int16_t pin)
{
    WITH_SEMAPHORE(_semaphore);
    for (uint8_t j = 0; j < _channels_number; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = new AnalogSource_Navio2(pin);
            return _channels[j];
        }
    }

    hal.console->printf("Out of analog channels\n");
    return nullptr;
}

void AnalogIn_Navio2::init()
{
    _board_voltage_pin = channel(0);
    _servorail_pin = channel(1);
}
