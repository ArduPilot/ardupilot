#include <AP_HAL/AP_HAL.h>

#include "AnalogIn_Raspilot.h"
#include "px4io_protocol.h"

AnalogSource_Raspilot::AnalogSource_Raspilot(int16_t pin):
    _pin(pin),
    _value(0.0f)
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
    //return _vcc_pin_analog_source->voltage_average() * 2.0;
}

AP_HAL::AnalogSource* AnalogIn_Raspilot::channel(int16_t pin)
{
    for (uint8_t j = 0; j < _channels_number; j++) {
        if (_channels[j] == NULL) {
            _channels[j] = new AnalogSource_Raspilot(pin);
            return _channels[j];
        }
    }

    hal.console->println("Out of analog channels");
    return NULL;
}

void AnalogIn_Raspilot::init()
{
    _vcc_pin_analog_source = channel(4);

    _spi = hal.spi->device(AP_HAL::SPIDevice_RASPIO);
    _spi_sem = _spi->get_semaphore();

    if (_spi_sem == NULL) {
        AP_HAL::panic("PANIC: RCIutput_Raspilot did not get "
                                  "valid SPI semaphore!");
        return; // never reached
    }

    hal.scheduler->suspend_timer_procs();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AnalogIn_Raspilot::_update, void));
    hal.scheduler->resume_timer_procs();
}

void AnalogIn_Raspilot::_update()
{
    if (AP_HAL::micros() - _last_update_timestamp < 100000) {
        return;
    }

    if (!_spi_sem->take_nonblocking()) {
        return;
    }

    struct IOPacket tx = { }, rx = { };
    uint16_t count = RASPILOT_ADC_MAX_CHANNELS;
    tx.count_code = count | PKT_CODE_READ;
    tx.page = PX4IO_PAGE_RAW_ADC_INPUT;
    tx.offset = 0;
    tx.crc = 0;
    tx.crc = crc_packet(&tx);
    /* set raspilotio to read reg4 */
    _spi->transaction((uint8_t *)&tx, (uint8_t *)&rx, sizeof(tx));

    hal.scheduler->delay_microseconds(200);

    count = 0;
    tx.count_code = count | PKT_CODE_READ;
    tx.page = 0;
    tx.offset = 0;
    tx.crc = 0;
    tx.crc = crc_packet(&tx);
    /* get reg4 data from raspilotio */
    _spi->transaction((uint8_t *)&tx, (uint8_t *)&rx, sizeof(tx));

    _spi_sem->give();

    for (int16_t i = 0; i < RASPILOT_ADC_MAX_CHANNELS; i++) {
        for (int16_t j=0; j < RASPILOT_ADC_MAX_CHANNELS; j++) {
            AnalogSource_Raspilot *source = _channels[j];

            if (source != NULL && i == source->_pin) {
                source->_value = rx.regs[i] * 3.3 / 4096.0;
            }
        }
    }

    _last_update_timestamp = AP_HAL::micros();
}
