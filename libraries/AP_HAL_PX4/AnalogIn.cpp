#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
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
#define PX4_VOLTAGE_SCALING (3.3f/4096.0f)

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
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
    // PX4 has 4 FMU analog input pins
    { 10, (5.7f*3.3f)/4096 }, // FMU battery on multi-connector pin 5,
                            // 5.7:1 scaling
    { 11,  6.6f/4096  }, // analog airspeed input, 2:1 scaling
    { 12,  3.3f/4096  }, // analog2, on SPI port pin 3
    { 13, 16.8f/4096  }, // analog3, on SPI port pin 4
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V2) || defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
    { 2,   3.3f/4096  },    // 3DR Brick voltage, usually 10.1:1
                            // scaled from battery voltage
    { 3,   3.3f/4096  },    // 3DR Brick current, usually 17:1 scaled
                            // for APM_PER_VOLT
    { 4,   6.6f/4096  },    // VCC 5V rail sense
    { 10,  3.3f/4096  },    // spare ADC
    { 11,  3.3f/4096  },    // spare ADC
    { 12,  3.3f/4096  },    // spare ADC
    { 13,  3.3f/4096  },    // AUX ADC pin 4
    { 14,  3.3f/4096  },    // AUX ADC pin 3
    { 15,  6.6f/4096  },    // analog airspeed sensor, 2:1 scaling
#elif defined(CONFIG_ARCH_BOARD_AEROFC_V1)
    { 1,   3.3f/4096  },
#else
#error "Unknown board type for AnalogIn scaling"
#endif
    { 0, 0.f          },
};

using namespace PX4;

PX4AnalogSource::PX4AnalogSource(int16_t pin, float initial_value) :
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
#ifdef PX4_ANALOG_VCC_5V_PIN
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        _pin = PX4_ANALOG_VCC_5V_PIN;
    }
#endif
}

void PX4AnalogSource::set_stop_pin(uint8_t p)
{ 
    _stop_pin = p; 
}

float PX4AnalogSource::read_average() 
{
    if (_sum_count == 0) {
        return _value;
    }
    hal.scheduler->suspend_timer_procs();
    _value = _sum_value / _sum_count;
    _value_ratiometric = _sum_ratiometric / _sum_count;
    _sum_value = 0;
    _sum_ratiometric = 0;
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();
    return _value;
}

float PX4AnalogSource::read_latest() 
{
    return _latest_value;
}

/*
  return scaling from ADC count to Volts
 */
float PX4AnalogSource::_pin_scaler(void)
{
    float scaling = PX4_VOLTAGE_SCALING;
    uint8_t num_scalings = ARRAY_SIZE(pin_scaling) - 1;
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
float PX4AnalogSource::voltage_average()
{
    return _pin_scaler() * read_average();
}

/*
  return voltage in Volts, assuming a ratiometric sensor powered by
  the 5V rail
 */
float PX4AnalogSource::voltage_average_ratiometric()
{
    voltage_average();
    return _pin_scaler() * _value_ratiometric;
}

/*
  return voltage in Volts
 */
float PX4AnalogSource::voltage_latest()
{
    return _pin_scaler() * read_latest();
}

void PX4AnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    hal.scheduler->suspend_timer_procs();
    _pin = pin;
    _sum_value = 0;
    _sum_ratiometric = 0;
    _sum_count = 0;
    _latest_value = 0;
    _value = 0;
    _value_ratiometric = 0;
    hal.scheduler->resume_timer_procs();
}

/*
  apply a reading in ADC counts
 */
void PX4AnalogSource::_add_value(float v, float vcc5V)
{
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
}


PX4AnalogIn::PX4AnalogIn() :
    _current_stop_pin_i(0),
	_board_voltage(0),
    _servorail_voltage(0),
    _power_flags(0)
{}

void PX4AnalogIn::init()
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
void PX4AnalogIn::next_stop_pin(void)
{
    // find the next stop pin. We start one past the current stop pin
    // and wrap completely, so we do the right thing is there is only
    // one stop pin
    for (uint8_t i=1; i <= PX4_ANALOG_MAX_CHANNELS; i++) {
        uint8_t idx = (_current_stop_pin_i + i) % PX4_ANALOG_MAX_CHANNELS;
        PX4::PX4AnalogSource *c = _channels[idx];
        if (c && c->_stop_pin != -1) {
            // found another stop pin
            _stop_pin_change_time = AP_HAL::millis();
            _current_stop_pin_i = idx;

            // set that pin high
            hal.gpio->pinMode(c->_stop_pin, 1);
            hal.gpio->write(c->_stop_pin, 1);

            // set all others low
            for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
                PX4::PX4AnalogSource *c2 = _channels[j];
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
void PX4AnalogIn::_timer_tick(void)
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

    struct adc_msg_s buf_adc[PX4_ANALOG_MAX_CHANNELS];

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
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2) || defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
            if (buf_adc[i].am_channel == 4) {
                // record the Vcc value for later use in
                // voltage_average_ratiometric()
                _board_voltage = buf_adc[i].am_data * 6.6f / 4096;
            }
#endif
        }
        for (uint8_t i=0; i<ret/sizeof(buf_adc[0]); i++) {
            Debug("chan %u value=%u\n",
                  (unsigned)buf_adc[i].am_channel,
                  (unsigned)buf_adc[i].am_data);
            for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
                PX4::PX4AnalogSource *c = _channels[j];
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

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
    // check for new battery data on FMUv1
    if (_battery_handle != -1) {
        struct battery_status_s battery;
        bool updated = false;
        if (orb_check(_battery_handle, &updated) == 0 && updated) {
            orb_copy(ORB_ID(battery_status), _battery_handle, &battery);
            if (battery.timestamp != _battery_timestamp) {
                _battery_timestamp = battery.timestamp;
                for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
                    PX4::PX4AnalogSource *c = _channels[j];
                    if (c == nullptr) continue;
                    if (c->_pin == PX4_ANALOG_ORB_BATTERY_VOLTAGE_PIN) {
                        c->_add_value(battery.voltage_v / PX4_VOLTAGE_SCALING, 0);
                    }
                    if (c->_pin == PX4_ANALOG_ORB_BATTERY_CURRENT_PIN) {
                        // scale it back to voltage, knowing that the
                        // px4io code scales by 90.0/5.0
                        c->_add_value(battery.current_a * (5.0f/90.0f) / PX4_VOLTAGE_SCALING, 0);
                    }
                }
            }
        }
    }
#endif

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2) || defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
    // check for new servorail data on FMUv2
    if (_servorail_handle != -1) {
        struct servorail_status_s servorail;
        bool updated = false;
        if (orb_check(_servorail_handle, &updated) == 0 && updated) {
            orb_copy(ORB_ID(servorail_status), _servorail_handle, &servorail);
            if (servorail.timestamp != _servorail_timestamp) {
                _servorail_timestamp = servorail.timestamp;
                _servorail_voltage = servorail.voltage_v;
                for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
                    PX4::PX4AnalogSource *c = _channels[j];
                    if (c == nullptr) continue;
                    if (c->_pin == PX4_ANALOG_ORB_SERVO_VOLTAGE_PIN) {
                        c->_add_value(servorail.voltage_v / PX4_VOLTAGE_SCALING, 0);
                    }
                    if (c->_pin == PX4_ANALOG_ORB_SERVO_VRSSI_PIN) {
                        c->_add_value(servorail.rssi_v / PX4_VOLTAGE_SCALING, 0);
                    }
                }
            }
        }
    }
    if (_system_power_handle != -1) {
        struct system_power_s system_power;
        bool updated = false;
        if (orb_check(_system_power_handle, &updated) == 0 && updated) {
            orb_copy(ORB_ID(system_power), _system_power_handle, &system_power);
            uint16_t flags = 0;
            if (system_power.usb_connected) flags |= MAV_POWER_STATUS_USB_CONNECTED;
            if (system_power.brick_valid)   flags |= MAV_POWER_STATUS_BRICK_VALID;
            if (system_power.servo_valid)   flags |= MAV_POWER_STATUS_SERVO_VALID;
            if (system_power.periph_5V_OC)  flags |= MAV_POWER_STATUS_PERIPH_OVERCURRENT;
            if (system_power.hipower_5V_OC) flags |= MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
            if (_power_flags != 0 && 
                _power_flags != flags && 
                hal.util->get_soft_armed()) {
                // the power status has changed while armed
                flags |= MAV_POWER_STATUS_CHANGED;
            }
            _power_flags = flags;
        }
    }
#endif

}

AP_HAL::AnalogSource* PX4AnalogIn::channel(int16_t pin) 
{
    for (uint8_t j=0; j<PX4_ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = new PX4AnalogSource(pin, 0.0f);
            return _channels[j];
        }
    }
    hal.console->printf("Out of analog channels\n");
    return nullptr;
}

#endif // CONFIG_HAL_BOARD
