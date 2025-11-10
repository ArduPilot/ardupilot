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
 * Code by Charles Villard, ARg and Bayu Laksono
 */
#include "AnalogIn.h"

#if AP_HAL_ANALOGIN_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_HAL_ESP32/Semaphores.h>

#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/adc_channel.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#ifndef ESP32_ADC_MAVLINK_DEBUG
// this allows the first 6 analog channels to be reported by mavlink for debugging purposes
#define ESP32_ADC_MAVLINK_DEBUG 0
#endif

#include <GCS_MAVLink/GCS_MAVLink.h>

#define ANALOGIN_DEBUGGING 0

// base voltage scaling for 12 bit 3.3V ADC
#define VOLTAGE_SCALING (3300.0f/4096.0f)

#if ANALOGIN_DEBUGGING
# define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
# define Debug(fmt, args ...)
#endif


//ADC handle
static adc_oneshot_unit_handle_t g_adc1_handle = NULL;

// we are limited to using adc1, and it supports 8 channels max, on gpio, in this order:
// ADC1_CH0=D36,ADC1_CH1=D37,ADC1_CH2=D38,ADC1_CH3=D39,ADC1_CH4=D32,ADC1_CH5=D33,ADC1_CH6=D34,ADC1_CH7=D35
// this driver will only configure the ADCs from a subset of these that the board exposes on pins.


extern const AP_HAL::HAL &hal;

using namespace ESP32;

/*
   scaling table between ADC count and actual input voltage, to account
   for voltage dividers on the board.
   */
const AnalogIn::pin_info AnalogIn::pin_config[] = {HAL_ESP32_ADC_PINS};

#define ADC_GRP1_NUM_CHANNELS   ARRAY_SIZE(AnalogIn::pin_config)


#define DEFAULT_VREF    3300         //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   256          //Multisampling

static const adc_atten_t atten = ADC_ATTEN_DB_12;

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        Debug("AnalogIn: calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        Debug("AnalogIn: calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        Debug("AnalogIn: Calibration Success for channel %d:%d", unit, channel);
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        Debug("AnalogIn: eFuse not burnt, skip software calibration");
    } else {
        Debug("AnalogIn: Invalid arg or no memory");
    }

    return calibrated;
}

void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    Debug("AnalogIn: deregister %s calibration scheme", "Curve Fitting");
    adc_cali_delete_scheme_curve_fitting(handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    Debug("AnalogIn: deregister %s calibration scheme", "Line Fitting");
    adc_cali_delete_scheme_line_fitting(handle);
#endif
}

//ardupin is the ardupilot assigned number, starting from 1-8(max)
AnalogSource::AnalogSource(int16_t ardupin, adc_channel_t adc_channel, float scaler, float initial_value) :
    _adc_channel(adc_channel),
    _ardupin(ardupin),
    _scaler(scaler),
    _value(initial_value),
    _latest_value(initial_value),
    _sum_count(0),
    _sum_value(0)
{
    Debug("AnalogIn: adding ardupin:%d-> which is adc1_channel:%d\n", _ardupin, _adc_channel);

    // for now, hard coded using adc1
    _adc_unit = ADC_UNIT_1;

    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = _adc_unit };
    if (!g_adc1_handle && ESP_OK != adc_oneshot_new_unit(&init_config, &g_adc1_handle)) {
        Debug("AnalogIn: adc_oneshot_new_unit failed for unit_id = %d\n", _adc_unit);
        return;
    }

    adc_init();
}


float AnalogSource::read_average()
{
    if ( _ardupin == ANALOG_INPUT_NONE ) {
        return 0.0f;
    }

    WITH_SEMAPHORE(_semaphore);

    if (_sum_count == 0) {
        float adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc_read();
        }
        adc_reading /= NO_OF_SAMPLES;
        return adc_reading;
    }

    _value = _sum_value / _sum_count;
    _sum_value = 0;
    _sum_count = 0;

    return _value;
}

float AnalogSource::read_latest()
{
    return _latest_value;
}

//_scaler scaling from ADC count to Volts

/*
   return voltage in Volts
   */
float AnalogSource::voltage_average()
{
    return _scaler * read_average();
}

/*
   return voltage in Volts
   */
float AnalogSource::voltage_latest()
{
    return _scaler * read_latest();
}

float AnalogSource::voltage_average_ratiometric()
{
    return _scaler * read_latest();
}

// ardupin
bool AnalogSource::set_pin(uint8_t ardupin)
{

    if (_ardupin == ardupin) {
        return true;
    }
    _ardupin = ardupin;

    int8_t pinconfig_offset = AnalogIn::find_pinconfig(ardupin);
    if (pinconfig_offset == -1 ) {
        Debug("AnalogIn: sorry set_pin() can't determine ADC1 offset from ardupin : %d \n",ardupin);
        return false;
    }

    adc_channel_t newChannel = (adc_channel_t)AnalogIn::pin_config[(uint8_t)pinconfig_offset].channel;
    float newscaler = AnalogIn::pin_config[(uint8_t)pinconfig_offset].scaling;

    Debug("AnalogIn: ardupin = %d, new channel = %d\n", ardupin, newChannel);

    if (_adc_channel == newChannel) {
        return true;
    }

    WITH_SEMAPHORE(_semaphore);

    if (_adc_cali_handle) {
        
        Debug("AnalogIn: adc_calibration_deinit(%x)\n", (uint)_adc_cali_handle);

        adc_calibration_deinit(_adc_cali_handle);
        _adc_cali_handle = 0;
    }    

    _adc_channel = newChannel;
    _scaler = newscaler;

    adc_init();

    _sum_value = 0;
    _sum_count = 0;
    _latest_value = 0;
    _value = 0;

    return true;

}

// init ADC
bool AnalogSource::adc_init()
{
    // init the pin now if possible, otherwise doo it later from set_pin
    if ( _ardupin != ANALOG_INPUT_NONE ) {

        adc_oneshot_chan_cfg_t config = {
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_12
        };
        if (ESP_OK != adc_oneshot_config_channel(g_adc1_handle, _adc_channel, &config)) {
            Debug("AnalogIn: adc_oneshot_config_channel failed for adc_channel = %d\n", _adc_channel);
            return false;
        }
        else {
            Debug("AnalogIn: adc_oneshot_config_channel for adc_channel = %d\n completed successfully", _adc_channel);
        }

        if (!adc_calibration_init(_adc_unit, _adc_channel, atten, &_adc_cali_handle)){
            Debug("AnalogIn: adc_calibration_init failed for adc_channel = %d\n", _adc_channel);
            return false;
        }
        else {
            Debug("AnalogIn: adc_calibration_init for adc_channel = %d\n completed successfully", _adc_channel);
        }
    }
    else {
        Debug("AnalogIn: adc_init(%d) skipped.\n",  _ardupin);
    }
    return true;
}

// read value from ADC
float AnalogSource::adc_read()
{
    int raw, value = 0;

    if (ESP_OK != adc_oneshot_read(g_adc1_handle, _adc_channel, &raw)) {
        Debug("AnalogIn: adc_oneshot_read failed\n");
        return 0;
    }

    if (_adc_cali_handle) {
        if(ESP_OK != adc_cali_raw_to_voltage(_adc_cali_handle, raw, &value)) {
            Debug("AnalogIn: adc_oneshot_read failed\n");
            return 0;
        }
    }
    else {
        value = raw * VOLTAGE_SCALING;
    }
    return (float)value / 1000;
}

/*
   apply a reading in ADC counts
   */
void AnalogSource::_add_value()
{
    if ( _ardupin == ANALOG_INPUT_NONE ) {
        return;
    }

    WITH_SEMAPHORE(_semaphore);

    float value = adc_read();

    _latest_value = value;
    _sum_value += value;
    _sum_count++;

    if (_sum_count == 254) {
        _sum_value /= 2;
        _sum_count /= 2;
    }
}

/*
   setup adc peripheral to capture samples with DMA into a buffer
   */
void AnalogIn::init()
{
}

/*
   called at 1kHz
*/
void AnalogIn::_timer_tick()
{
    for (uint8_t j = 0; j < ANALOG_MAX_CHANNELS; j++) {
        ESP32::AnalogSource *c = _channels[j];
        if (c != nullptr) {
            // add a value
            c->_add_value();
        }
    }

#if ESP32_ADC_MAVLINK_DEBUG
    static uint8_t count;
    if (AP_HAL::millis() > 5000 && count++ == 10) {
        count = 0;
        uint16_t adc[6] {};
        uint8_t n = ADC_GRP1_NUM_CHANNELS;
        if (n > 6) {
            n = 6;
        }
        for (uint8_t i = 0; i < n; i++) {
            adc[i] = buf_adc[i];
        }
        mavlink_msg_ap_adc_send(MAVLINK_COMM_0, adc[0], adc[1], adc[2], adc[3], adc[4],
                                adc[5]);
    }
#endif

}

//positive array index (zero is ok), or -1 on error
int8_t AnalogIn::find_pinconfig(int16_t ardupin)
{
    // from ardupin, lookup which adc gpio that is..
    for (uint8_t j = 0; j < ADC_GRP1_NUM_CHANNELS; j++) {
        if (pin_config[j].ardupin == ardupin) {
            return j;
        }
    }
    // can't find a match in definitions
    return -1;

}

//
AP_HAL::AnalogSource *AnalogIn::channel(int16_t ardupin)
{
    if (ardupin < 0) ardupin = ANALOG_INPUT_NONE;

    Debug("AnalogIn: configuring channel %d\n", ardupin);

    int8_t pinconfig_offset = find_pinconfig(ardupin);

    adc_channel_t adc_channel = (adc_channel_t)ANALOG_INPUT_NONE;
    float scaler = 0;

    if ((ardupin != ANALOG_INPUT_NONE) && (pinconfig_offset == -1 )) {
        Debug("AnalogIn: sorry channel() can't determine ADC1 offset from ardupin : %d \n",ardupin);
        ardupin = ANALOG_INPUT_NONE; // default it to this not terrible value and allow to continue
    }

    // although ANALOG_INPUT_NONE=255 is not a valid pin, we let it through here as
    //  a special case, so that it can be changed with set_pin(..) later.
    if (ardupin != ANALOG_INPUT_NONE) {
        adc_channel = (adc_channel_t)pin_config[(uint8_t)pinconfig_offset].channel;
        scaler = pin_config[(uint8_t)pinconfig_offset].scaling;
    }

    for (uint8_t j = 0; j < ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == nullptr) {

            _channels[j] = NEW_NOTHROW AnalogSource(ardupin, adc_channel, scaler, 0.0f);

            if (ardupin != ANALOG_INPUT_NONE) {
                Debug("AnalogIn: channel: %d attached to ardupin:%d at adc1_offset:%d\n",\
                                    j, ardupin, adc_channel);
            }
            else {
                Debug("AnalogIn: channel: %d created but using delayed adc and gpio pin configuration\n", j);
            }

            return _channels[j];
        }
    }
    Debug("AnalogIn: out of channels\n");
    return nullptr;
}

#endif  // AP_HAL_ANALOGIN_ENABLED
