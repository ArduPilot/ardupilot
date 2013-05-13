/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AnalogIn.h"
#include <drivers/drv_adc.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <nuttx/analog/adc.h>

#define ANLOGIN_DEBUGGING 0

// the airspeed port has some additional scaling applied to allow it
// to go above 3.3V. This value was found by checking the ADC output
// for a range of known inputs from 1.3V to 5.0V
#define PX4_AIRSPEED_VOLTAGE_SCALING (6.76f/4096.0f)

// pin4 in the SPI port is analog input 13, marked as analog3 on the
// PX4IO schematic v1.3, and is scaled quite strangely
#define PX4_ANALOG3_VOLTAGE_SCALING (16.88f/4096.0f)

#define PX4_VOLTAGE_SCALING (3.3f/4096.0f)

#if ANLOGIN_DEBUGGING
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

using namespace PX4;

int PX4AnalogIn::_adc_fd;
uint32_t PX4AnalogIn::_last_run;
PX4AnalogSource* PX4AnalogIn::_channels[PX4_ANALOG_MAX_CHANNELS] = {};
int PX4AnalogIn::_battery_handle = -1;
uint64_t PX4AnalogIn::_battery_timestamp;

PX4AnalogSource::PX4AnalogSource(int16_t pin, float initial_value) :
	_pin(pin),
    _value(initial_value),
    _latest_value(initial_value),
    _sum_count(0),
    _sum_value(0)
{
}

float PX4AnalogSource::read_average() 
{
    if (_sum_count == 0) {
        return _value;
    }
    hal.scheduler->suspend_timer_procs();
    _value = _sum_value / _sum_count;
    _sum_value = 0;
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();
    return _value;
}

float PX4AnalogSource::read_latest() 
{
    return _latest_value;
}

/*
  return voltage in Volts
 */
float PX4AnalogSource::voltage_average()
{
    if (_pin == PX4_ANALOG_AIRSPEED_PIN) {
        return PX4_AIRSPEED_VOLTAGE_SCALING * read_average();
    }
    if (_pin == PX4_ANALOG_ANALOG3_PIN) {
        return PX4_ANALOG3_VOLTAGE_SCALING * read_average();
    }
    return PX4_VOLTAGE_SCALING * read_average();
}

void PX4AnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    hal.scheduler->suspend_timer_procs();
    _pin = pin;
    _sum_value = 0;
    _sum_count = 0;
    _latest_value = 0;
    _value = 0;
    hal.scheduler->resume_timer_procs();
}

void PX4AnalogSource::_add_value(float v)
{
    _latest_value = v;
    _sum_value += v;
    _sum_count++;
    if (_sum_count == 254) {
        _sum_value /= 2;
        _sum_count /= 2;
    }
}


PX4AnalogIn::PX4AnalogIn()
{}

void PX4AnalogIn::init(void* machtnichts)
{
	_adc_fd = open(ADC_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
    if (_adc_fd == -1) {
        hal.scheduler->panic("Unable to open " ADC_DEVICE_PATH);
	}
    _battery_handle = orb_subscribe(ORB_ID(battery_status));
    hal.scheduler->register_timer_process(_analogin_timer);
}

/*
  called at 1kHz
 */
void PX4AnalogIn::_analogin_timer(uint32_t now)
{
    // read adc at 100Hz
    uint32_t delta_t = now - _last_run;
    if (delta_t < 10000) {
        return;
    }
    _last_run = now;

    struct adc_msg_s buf_adc[8];

    /* read all channels available */
    int ret = read(_adc_fd, &buf_adc, sizeof(buf_adc));
    if (ret > 0) {
        // match the incoming channels to the currently active pins
        for (uint8_t i=0; i<ret/sizeof(buf_adc[0]); i++) {
            Debug("chan %u value=%u\n",
                  (unsigned)buf_adc[i].am_channel,
                  (unsigned)buf_adc[i].am_data);
            for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
                PX4::PX4AnalogSource *c = _channels[j];
                if (c != NULL && buf_adc[i].am_channel == c->_pin) {
                    c->_add_value(buf_adc[i].am_data);
                }
            }
        }
    }

    // check for new battery data
    if (_battery_handle != -1) {
        struct battery_status_s battery;
        if (orb_copy(ORB_ID(battery_status), _battery_handle, &battery) == OK &&
            battery.timestamp != _battery_timestamp) {
            _battery_timestamp = battery.timestamp;
            for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
                PX4::PX4AnalogSource *c = _channels[j];
                if (c == NULL) continue;
                if (c->_pin == PX4_ANALOG_BATTERY_VOLTAGE_PIN) {
                    c->_add_value(battery.voltage_v / PX4_VOLTAGE_SCALING);
                }
                if (c->_pin == PX4_ANALOG_BATTERY_CURRENT_PIN) {
                    // scale it back to voltage, knowing that the
                    // px4io code scales by 90.0/5.0
                    c->_add_value(battery.current_a * (5.0f/90.0f) / PX4_VOLTAGE_SCALING);
                }
            }
        }
    }
}

AP_HAL::AnalogSource* PX4AnalogIn::channel(int16_t pin) 
{
    for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == NULL) {
            _channels[j] = new PX4AnalogSource(pin, 0.0);
            return _channels[j];
        }
    }
    hal.console->println("Out of analog channels");
    return NULL;
}

#endif // CONFIG_HAL_BOARD
