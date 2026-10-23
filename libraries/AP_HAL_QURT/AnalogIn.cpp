#include "AnalogIn.h"
#include "RCOutput.h"

using namespace QURT;

#define ANALOG_PIN_VOLTAGE 1
#define ANALOG_PIN_CURRENT 2

#define NUM_CHANNELS 10

extern const AP_HAL::HAL &hal;

// static table of voltage sources
static AnalogSource sources[NUM_CHANNELS];

AnalogIn::AnalogIn()
{
}

float AnalogSource::read_average()
{
    return read_latest();
}

float AnalogSource::voltage_average()
{
    return read_latest();
}

float AnalogSource::voltage_latest()
{
    return read_latest();
}

float AnalogSource::read_latest()
{
    const auto *rcout = (QURT::RCOutput *)hal.rcout;
    switch (pin) {
    case ANALOG_PIN_VOLTAGE:
        return rcout->get_voltage();
    case ANALOG_PIN_CURRENT:
        return rcout->get_current();
    default:
        break;
    }
    return 0;
}

bool AnalogSource::set_pin(uint8_t p)
{
    switch (p) {
    case ANALOG_PIN_VOLTAGE:
    case ANALOG_PIN_CURRENT:
        pin = p;
        return true;
    default:
        break;
    }
    return false;
}

void AnalogIn::init()
{
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n)
{
    if (next_chan >= ARRAY_SIZE(sources)) {
        return nullptr;
    }
    return &sources[next_chan++];
}

/*
  not available, report 5v to keep GCS happy
 */
float AnalogIn::board_voltage(void)
{
    return 5.0f;
}
