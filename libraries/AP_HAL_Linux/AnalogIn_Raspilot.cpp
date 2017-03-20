#include "AnalogIn_Raspilot.h"

#include <algorithm>

#include <AP_HAL/AP_HAL.h>

#include "px4io_protocol.h"

AnalogSource_Raspilot::AnalogSource_Raspilot(int16_t pin)
    : _pin(pin)
{
}

void AnalogSource_Raspilot::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    _pin = pin;
}

float AnalogSource_Raspilot::read_average()
{
    return read_latest();
}

float AnalogSource_Raspilot::read_latest()
{
    return _value;
}

float AnalogSource_Raspilot::voltage_average()
{
    return _value;
}

float AnalogSource_Raspilot::voltage_latest()
{
    return _value;
}

float AnalogSource_Raspilot::voltage_average_ratiometric()
{
    return _value;
}

extern const AP_HAL::HAL& hal;

AnalogIn_Raspilot::AnalogIn_Raspilot()
{
    _channels_number = RASPILOT_ADC_MAX_CHANNELS;
}

float AnalogIn_Raspilot::board_voltage(void)
{
    _vcc_pin_analog_source->set_pin(4);

    return 5.0;
}

AP_HAL::AnalogSource* AnalogIn_Raspilot::channel(int16_t pin)
{
    for (uint8_t j = 0; j < _channels_number; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = new AnalogSource_Raspilot(pin);
            return _channels[j];
        }
    }

    hal.console->printf("Out of analog channels\n");
    return nullptr;
}

void AnalogIn_Raspilot::init()
{
    _vcc_pin_analog_source = channel(4);

    _dev = std::move(hal.spi->get_device("raspio"));
    if (!_dev) {
        AP_HAL::panic("Bus for AnalogIn_Raspilot not found");
        return;
    }

    _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AnalogIn_Raspilot::_update, void));
}

void AnalogIn_Raspilot::_update()
{
    struct IOPacket tx = { }, rx = { };
    uint16_t count = RASPILOT_ADC_MAX_CHANNELS;
    tx.count_code = count | PKT_CODE_READ;
    tx.page = PX4IO_PAGE_RAW_ADC_INPUT;
    tx.offset = 0;
    tx.crc = 0;
    tx.crc = crc_packet(&tx);

    /* set raspilotio to read reg4 */
    _dev->transfer((uint8_t *)&tx, sizeof(tx), (uint8_t *)&rx, sizeof(rx));

    // TODO: should not delay for such huge values: converting this to a
    // state-machine like driver would be better, adjusting the callback timer
    hal.scheduler->delay_microseconds(200);

    count = 0;
    tx.count_code = count | PKT_CODE_READ;
    tx.page = 0;
    tx.offset = 0;
    tx.crc = 0;
    tx.crc = crc_packet(&tx);

    /* get reg4 data from raspilotio */
    _dev->transfer((uint8_t *)&tx, sizeof(tx), (uint8_t *)&rx, sizeof(rx));

    for (int16_t i = 0; i < RASPILOT_ADC_MAX_CHANNELS; i++) {
        for (int16_t j=0; j < RASPILOT_ADC_MAX_CHANNELS; j++) {
            AnalogSource_Raspilot *source = _channels[j];

            if (source != nullptr && i == source->_pin) {
                source->_value = rx.regs[i] * 3.3 / 4096.0;
            }
        }
    }
}
