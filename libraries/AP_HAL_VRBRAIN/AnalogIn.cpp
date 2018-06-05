#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AnalogIn.h"
#include <drivers/drv_adc.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/config.h>
#include <arch/board/board.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/system_power.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <errno.h>
#include "GPIO.h"

#define ANLOGIN_DEBUGGING 0

// base voltage scaling for 12 bit 3.3V ADC
#define VRBRAIN_VOLTAGE_SCALING (3.3f/4096.0f)

#if ANLOGIN_DEBUGGING
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

/*
  scaling table between ADC count and actual input voltage, to account
  for voltage dividers on the board. 
 */
static const struct {
    uint8_t pin;
    float scaling;
} pin_scaling[] = {
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V52) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V52E) || defined(CONFIG_ARCH_BOARD_VRCORE_V10) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V54)
    { 10, 3.3f/4096 },
    { 11, 3.3f/4096 },
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
    { 10, 3.3f/4096 },
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)
    {  1, 3.3f/4096 },
    {  2, 3.3f/4096 },
    {  3, 3.3f/4096 },
    { 10, 3.3f/4096 },
#else
#error "Unknown board type for AnalogIn scaling"
#endif
    { 0, 0.f          },
};

using namespace VRBRAIN;

VRBRAINAnalogSource::VRBRAINAnalogSource(int16_t pin, float initial_value) :
	_pin(pin),
    _stop_pin(-1),
    _settle_time_ms(0),
    _value(initial_value),
    _value_ratiometric(initial_value),
    _latest_value(initial_value),
    _sum_count(0),
    _sum_value(0),
    _sum_ratiometric(0)
{
    _semaphore = hal.util->new_semaphore();
}

void VRBRAINAnalogSource::set_stop_pin(uint8_t p)
{ 
    _stop_pin = p; 
}

float VRBRAINAnalogSource::read_average()
{
    if (_semaphore->take(1)) {
        if (_sum_count == 0) {
            _semaphore->give();
            return _value;
        }
        _value = _sum_value / _sum_count;
        _value_ratiometric = _sum_ratiometric / _sum_count;
        _sum_value = 0;
        _sum_ratiometric = 0;
        _sum_count = 0;
        _semaphore->give();
        return _value;
    }
    return _value;
}

float VRBRAINAnalogSource::read_latest()
{
    return _latest_value;
}

/*
  return scaling from ADC count to Volts
 */
float VRBRAINAnalogSource::_pin_scaler(void)
{
    float scaling = VRBRAIN_VOLTAGE_SCALING;
    uint8_t num_scalings = ARRAY_SIZE(pin_scaling);
    for (uint8_t i=0; i<num_scalings; i++) {
        if (pin_scaling[i].pin == _pin) {
            scaling = pin_scaling[i].scaling;
            break;
        }
    }
    return scaling;
}

/*
  return voltage in Volts
 */
float VRBRAINAnalogSource::voltage_average()
{
    return _pin_scaler() * read_average();
}

/*
  return voltage in Volts, assuming a ratiometric sensor powered by
  the 5V rail
 */
float VRBRAINAnalogSource::voltage_average_ratiometric()
{
    voltage_average();
    return _pin_scaler() * _value_ratiometric;
}

/*
  return voltage in Volts
 */
float VRBRAINAnalogSource::voltage_latest()
{
    return _pin_scaler() * read_latest();
}

void VRBRAINAnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    if (_semaphore->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _pin = pin;
        _sum_value = 0;
        _sum_ratiometric = 0;
        _sum_count = 0;
        _latest_value = 0;
        _value = 0;
        _value_ratiometric = 0;
        _semaphore->give();
    }
}

/*
  apply a reading in ADC counts
 */
void VRBRAINAnalogSource::_add_value(float v, float vcc5V)
{
    if (_semaphore->take(1)) {
        _latest_value = v;
        _sum_value += v;
        if (vcc5V < 3.0f) {
            _sum_ratiometric += v;
        } else {
            // this compensates for changes in the 5V rail relative to the
            // 3.3V reference used by the ADC.
            _sum_ratiometric += v * 5.0f / vcc5V;
        }
        _sum_count++;
        if (_sum_count == 254) {
            _sum_value /= 2;
            _sum_ratiometric /= 2;
            _sum_count /= 2;
        }
        _semaphore->give();
    }
}


VRBRAINAnalogIn::VRBRAINAnalogIn() :
    _current_stop_pin_i(0),
	_board_voltage(0),
    _servorail_voltage(0),
    _power_flags(0)
{}

void VRBRAINAnalogIn::init()
{
	_adc_fd = open(ADC0_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
    if (_adc_fd == -1) {
        AP_HAL::panic("Unable to open " ADC0_DEVICE_PATH);
	}
    _battery_handle   = orb_subscribe(ORB_ID(battery_status));
    _servorail_handle = orb_subscribe(ORB_ID(servorail_status));
    _system_power_handle = orb_subscribe(ORB_ID(system_power));
}


/*
  move to the next stop pin
 */
void VRBRAINAnalogIn::next_stop_pin(void)
{
    // find the next stop pin. We start one past the current stop pin
    // and wrap completely, so we do the right thing is there is only
    // one stop pin
    for (uint8_t i=1; i <= VRBRAIN_ANALOG_MAX_CHANNELS; i++) {
        uint8_t idx = (_current_stop_pin_i + i) % VRBRAIN_ANALOG_MAX_CHANNELS;
        VRBRAIN::VRBRAINAnalogSource *c = _channels[idx];
        if (c && c->_stop_pin != -1) {
            // found another stop pin
            _stop_pin_change_time = AP_HAL::millis();
            _current_stop_pin_i = idx;

            // set that pin high
            hal.gpio->pinMode(c->_stop_pin, 1);
            hal.gpio->write(c->_stop_pin, 1);

            // set all others low
            for (uint8_t j=0; j<VRBRAIN_ANALOG_MAX_CHANNELS; j++) {
                VRBRAIN::VRBRAINAnalogSource *c2 = _channels[j];
                if (c2 && c2->_stop_pin != -1 && j != idx) {
                    hal.gpio->pinMode(c2->_stop_pin, 1);
                    hal.gpio->write(c2->_stop_pin, 0);
                }
            }
            break;
        }
    }
}

/*
  called at 1kHz
 */
void VRBRAINAnalogIn::_timer_tick(void)
{
    if (_adc_fd == -1) {
        // not initialised yet
        return;
    }

    // read adc at 100Hz
    uint32_t now = AP_HAL::micros();
    uint32_t delta_t = now - _last_run;
    if (delta_t < 10000) {
        return;
    }
    _last_run = now;

    struct adc_msg_s buf_adc[VRBRAIN_ANALOG_MAX_CHANNELS];

    // cope with initial setup of stop pin
    if (_channels[_current_stop_pin_i] == nullptr ||
        _channels[_current_stop_pin_i]->_stop_pin == -1) {
        next_stop_pin();
    }

    /* read all channels available */
    int ret = read(_adc_fd, &buf_adc, sizeof(buf_adc));
    if (ret > 0) {
        // match the incoming channels to the currently active pins
        for (uint8_t i=0; i<ret/sizeof(buf_adc[0]); i++) {
            Debug("chan %u value=%u\n",
                  (unsigned)buf_adc[i].am_channel,
                  (unsigned)buf_adc[i].am_data);
            for (uint8_t j=0; j<VRBRAIN_ANALOG_MAX_CHANNELS; j++) {
                VRBRAIN::VRBRAINAnalogSource *c = _channels[j];
                if (c != nullptr && buf_adc[i].am_channel == c->_pin) {
                    // add a value if either there is no stop pin, or
                    // the stop pin has been settling for enough time
                    if (c->_stop_pin == -1 || 
                        (_current_stop_pin_i == j &&
                         AP_HAL::millis() - _stop_pin_change_time > c->_settle_time_ms)) {
                        c->_add_value(buf_adc[i].am_data, _board_voltage);
                        if (c->_stop_pin != -1 && _current_stop_pin_i == j) {
                            next_stop_pin();
                        }
                    }
                }
            }
        }
    }

}

AP_HAL::AnalogSource* VRBRAINAnalogIn::channel(int16_t pin)
{
    for (uint8_t j=0; j<VRBRAIN_ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = new VRBRAINAnalogSource(pin, 0.0f);
            return _channels[j];
        }
    }
    hal.console->printf("Out of analog channels\n");
    return nullptr;
}

#endif // CONFIG_HAL_BOARD
