/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>

#include "AnalogIn.h"

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#define ANLOGIN_DEBUGGING 0

// base voltage scaling for 12 bit 3.3V ADC
#define VOLTAGE_SCALING (3.3f/4096.0f)

#if ANLOGIN_DEBUGGING
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

#define ADC_GRP1_NUM_CHANNELS   3
#define ADC_GRP1_BUF_DEPTH      8
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH] = {0};
static float avg_samples[ADC_GRP1_NUM_CHANNELS];

// special pin numbers
#define ANALOG_VCC_5V_PIN                4

/*
  scaling table between ADC count and actual input voltage, to account
  for voltage dividers on the board. 
 */
static const struct {
    uint8_t channel;
    float scaling;
} pin_scaling[] = {
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412
    { ANALOG_VCC_5V_PIN,   0.007734  },    // VCC 5V rail sense
#else
    { ANALOG_VCC_5V_PIN,   6.6f/4096  },    // VCC 5V rail sense
#endif
    { 2,   3.3f/4096  },    // 3DR Brick voltage, usually 10.1:1
                            // scaled from battery voltage
    { 3,   3.3f/4096  },    // 3DR Brick current, usually 17:1 scaled
                            // for APM_PER_VOLT
};

using namespace ChibiOS;

ChibiAnalogSource::ChibiAnalogSource(int16_t pin, float initial_value) :
    _pin(pin),
    _value(initial_value),
    _value_ratiometric(initial_value),
    _latest_value(initial_value),
    _sum_count(0),
    _sum_value(0),
    _sum_ratiometric(0)
{
/*#ifdef PX4_ANALOG_VCC_5V_PIN
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        _pin = PX4_ANALOG_VCC_5V_PIN;
    }
#endif
*/
}


float ChibiAnalogSource::read_average() 
{
    if (_sum_count == 0) {
        return _value;
    }
    _value = _sum_value / _sum_count;
    _value_ratiometric = _sum_ratiometric / _sum_count;
    _sum_value = 0;
    _sum_ratiometric = 0;
    _sum_count = 0;
    return _value;
}

float ChibiAnalogSource::read_latest() 
{
    return _latest_value;
}

/*
  return scaling from ADC count to Volts
 */
float ChibiAnalogSource::_pin_scaler(void)
{
    float scaling = VOLTAGE_SCALING;
    uint8_t num_scalings = ARRAY_SIZE(pin_scaling) - 1;
    for (uint8_t i=0; i<num_scalings; i++) {
        if (pin_scaling[i].channel == _pin) {
            scaling = pin_scaling[i].scaling;
            break;
        }
    }
    return scaling;
}

/*
  return voltage in Volts
 */
float ChibiAnalogSource::voltage_average()
{
    return _pin_scaler() * read_average();
}

/*
  return voltage in Volts, assuming a ratiometric sensor powered by
  the 5V rail
 */
float ChibiAnalogSource::voltage_average_ratiometric()
{
    voltage_average();
    return _pin_scaler() * _value_ratiometric;
}

/*
  return voltage in Volts
 */
float ChibiAnalogSource::voltage_latest()
{
    return _pin_scaler() * read_latest();
}

void ChibiAnalogSource::set_pin(uint8_t pin)
{
    if (_pin == pin) {
        return;
    }
    _pin = pin;
    _sum_value = 0;
    _sum_ratiometric = 0;
    _sum_count = 0;
    _latest_value = 0;
    _value = 0;
    _value_ratiometric = 0;
}

/*
  apply a reading in ADC counts
 */
void ChibiAnalogSource::_add_value(float v, float vcc5V)
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


ChibiAnalogIn::ChibiAnalogIn() :
    _board_voltage(0),
    _servorail_voltage(0),
    _power_flags(0)
{
    memset(samples, 0, sizeof(samples));
    for (uint8_t i = 0; i < ADC_GRP1_NUM_CHANNELS; i++) {
        avg_samples[i] = 0.0f;
    }
}

void ChibiAnalogIn::adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    if (buffer != samples) {
        return;
    }
    for (uint8_t i = 0; i < ADC_GRP1_NUM_CHANNELS; i++) {
        avg_samples[i] = 0.0f;
    }
    for (uint8_t i = 0; i < ADC_GRP1_BUF_DEPTH; i++) {
        for (uint8_t j = 0; j < ADC_GRP1_NUM_CHANNELS; j++) { 
            avg_samples[j] += samples[(i*ADC_GRP1_NUM_CHANNELS)+j];
        }
    }
    for (uint8_t i = 0; i < ADC_GRP1_NUM_CHANNELS; i++) {
        avg_samples[i] /= ADC_GRP1_BUF_DEPTH;
    }
}

void ChibiAnalogIn::init()
{
    adcStart(&ADCD1, NULL);
    memset(&adcgrpcfg, 0, sizeof(adcgrpcfg));
    adcgrpcfg.circular = true;
    adcgrpcfg.num_channels = ADC_GRP1_NUM_CHANNELS;
    adcgrpcfg.end_cb = adccallback;
    adcgrpcfg.cr2 = ADC_CR2_SWSTART;
    adcgrpcfg.smpr2 = ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_480) |
     ADC_SMPR2_SMP_AN4(ADC_SAMPLE_480);
    adcgrpcfg.sqr1 = ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
    adcgrpcfg.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4) |ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2) |
     ADC_SQR3_SQ3_N(ADC_CHANNEL_IN3);
    adcStartConversion(&ADCD1, &adcgrpcfg, &samples[0], ADC_GRP1_BUF_DEPTH); 
}

void ChibiAnalogIn::read_adc(uint32_t *val)
{
    for (uint8_t i = 0; i < ADC_GRP1_NUM_CHANNELS; i++) {
        val[i] = avg_samples[i];
    }
}
/*
  called at 1kHz
 */
void ChibiAnalogIn::_timer_tick(void)
{
    // read adc at 100Hz
    uint32_t now = AP_HAL::micros();
    uint32_t delta_t = now - _last_run;
    if (delta_t < 10000) {
        return;
    }
    _last_run = now;

    uint32_t buf_adc[ADC_GRP1_NUM_CHANNELS] = {0};

    /* read all channels available */
    read_adc(buf_adc);
    // match the incoming channels to the currently active pins
    for (uint8_t i=0; i < ADC_GRP1_NUM_CHANNELS; i++) {
        if (pin_scaling[i].channel == ANALOG_VCC_5V_PIN) {
            // record the Vcc value for later use in
            // voltage_average_ratiometric()
            _board_voltage = buf_adc[i] * pin_scaling[i].scaling;
        }
    }
    for (uint8_t i=0; i<ADC_GRP1_NUM_CHANNELS; i++) {
        Debug("chan %u value=%u\n",
              (unsigned)pin_scaling[i].channel,
              (unsigned)buf_adc[i]);
        for (uint8_t j=0; j < ADC_GRP1_NUM_CHANNELS; j++) {
            ChibiOS::ChibiAnalogSource *c = _channels[j];
            if (c != nullptr && pin_scaling[i].channel == c->_pin) {
                // add a value
                c->_add_value(buf_adc[i], _board_voltage);
            }
        }
    }

#if HAL_WITH_IO_MCU
    // now handle special inputs from IOMCU
    _servorail_voltage = iomcu.get_vservo();
#endif
}

AP_HAL::AnalogSource* ChibiAnalogIn::channel(int16_t pin) 
{
    for (uint8_t j=0; j<ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = new ChibiAnalogSource(pin, 0.0f);
            return _channels[j];
        }
    }
    hal.console->printf("Out of analog channels\n");
    return nullptr;
}

