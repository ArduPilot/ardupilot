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
 * Code by Charles Villard
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ESP32/Semaphores.h>

#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "soc/adc_channel.h"

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)

#include "AnalogIn.h"

#ifndef ESP32_ADC_MAVLINK_DEBUG
// this allows the first 6 analog channels to be reported by mavlink for debugging purposes
#define ESP32_ADC_MAVLINK_DEBUG 0
#endif

#include <GCS_MAVLink/GCS_MAVLink.h>

#define ANALOGIN_DEBUGGING 0

// base voltage scaling for 12 bit 3.3V ADC
#define VOLTAGE_SCALING (3.3f/4096.0f)

#if ANALOGIN_DEBUGGING
# define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
# define Debug(fmt, args ...)
#endif

// we are limited to using adc1, and it supports 8 channels max, on gpio, in this order:
// ADC1_CH0=D36,ADC1_CH1=D37,ADC1_CH2=D38,ADC1_CH3=D39,ADC1_CH4=D32,ADC1_CH5=D33,ADC1_CH6=D34,ADC1_CH7=D35
// this driver will only configure the ADCs from a subset of these that the board exposes on pins.


extern const AP_HAL::HAL &hal;

using namespace ESP32;

/*
   scaling table between ADC count and actual input voltage, to account
   for voltage dividers on the board.
   */
const AnalogIn::pin_info AnalogIn::pin_config[] = HAL_ESP32_ADC_PINS;

#define ADC_GRP1_NUM_CHANNELS   ARRAY_SIZE(AnalogIn::pin_config)


#define DEFAULT_VREF    1100       //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   256          //Multisampling

static const adc_atten_t atten = ADC_ATTEN_DB_11;

//ardupin is the ardupilot assigned number, starting from 1-8(max)
// 'pin' and _pin is a macro like 'ADC1_GPIO35_CHANNEL' from board config .h
AnalogSource::AnalogSource(int16_t ardupin,int16_t pin,float scaler, float initial_value, uint8_t unit) :

    _unit(unit),
    _ardupin(ardupin),
    _pin(pin),
    _scaler(scaler),
    _value(initial_value),
    _latest_value(initial_value),
    _sum_count(0),
    _sum_value(0)
{
    printf("AnalogIn: adding ardupin:%d-> which is adc1_offset:%d\n", _ardupin,_pin);

    // init the pin now if possible, otherwise doo it later from set_pin
    if ( _ardupin != ANALOG_INPUT_NONE ) {

        // dertermine actial gpio from adc offset and configure it
        gpio_num_t gpio;
        //Configure ADC
        if (unit == 1) {
            adc1_config_channel_atten((adc1_channel_t)_pin, atten);
            adc1_pad_get_io_num((adc1_channel_t)_pin, &gpio);
        } else {
            adc2_config_channel_atten((adc2_channel_t)_pin, atten);
        }

        esp_adc_cal_characteristics_t adc_chars;
        esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
        printf("AnalogIn: determined actual gpio as: %d\n", gpio);

        _gpio = gpio;// save it for later

    }
}


float AnalogSource::read_average()
{
    if ( _ardupin == ANALOG_INPUT_NONE ) {
        return 0.0f;
    }

    WITH_SEMAPHORE(_semaphore);

    if (_sum_count == 0) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (_unit == 1) {
                adc_reading += adc1_get_raw((adc1_channel_t)_pin);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)_pin, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
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

    int8_t pinconfig_offset = AnalogIn::find_pinconfig(ardupin);
    if (pinconfig_offset == -1 ) {
        DEV_PRINTF("AnalogIn: sorry set_pin() can't determine ADC1 offset from ardupin : %d \n",ardupin);
        return false;
    }

    int16_t newgpioAdcPin = AnalogIn::pin_config[(uint8_t)pinconfig_offset].channel;
    float newscaler = AnalogIn::pin_config[(uint8_t)pinconfig_offset].scaling;

    if (_pin == newgpioAdcPin) {
        return true;
    }

    WITH_SEMAPHORE(_semaphore);

    // init the target pin now if possible
    if ( ardupin != ANALOG_INPUT_NONE ) {

        gpio_num_t gpio; // new gpio
        //Configure ADC
        if (_unit == 1) {
            adc1_config_channel_atten((adc1_channel_t)newgpioAdcPin, atten);
            adc1_pad_get_io_num((adc1_channel_t)newgpioAdcPin, &gpio);
        } else {
            adc2_config_channel_atten((adc2_channel_t)newgpioAdcPin, atten);
        }

        esp_adc_cal_characteristics_t adc_chars;
        esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
        printf("AnalogIn: Adding gpio on: %d\n", gpio);

        DEV_PRINTF("AnalogIn: set_pin() FROM (ardupin:%d adc1_offset:%d gpio:%d) TO (ardupin:%d adc1_offset:%d gpio:%d)\n", \
                            _ardupin,_pin,_gpio,ardupin,newgpioAdcPin,gpio);
        _pin = newgpioAdcPin;
        _ardupin = ardupin;
        _gpio = gpio;
        _scaler = newscaler;

    }

    _sum_value = 0;
    _sum_count = 0;
    _latest_value = 0;
    _value = 0;

    return true;

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

    int value = 0;
    if (_unit == 1) {
        value = adc1_get_raw((adc1_channel_t)_pin);
    } else {
        adc2_get_raw((adc2_channel_t)_pin, ADC_WIDTH_BIT_12, &value);
    }

    _latest_value = value;
    _sum_value += value;
    _sum_count++;

    if (_sum_count == 254) {
        _sum_value /= 2;
        _sum_count /= 2;
    }
}

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("AnalogIn: eFuse Two Point: Supported\n");
    } else {
        printf("AnalogIn: eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("AnalogIn: eFuse Vref: Supported\n");
    } else {
        printf("AnalogIn: eFuse Vref: NOT supported\n");
    }
}

/*
   setup adc peripheral to capture samples with DMA into a buffer
   */
void AnalogIn::init()
{
    check_efuse();

    adc1_config_width(ADC_WIDTH_BIT_12);
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
            //c->_add_value();
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
    // can't find a match in definitons
    return -1;

}

//
AP_HAL::AnalogSource *AnalogIn::channel(int16_t ardupin)
{
    int8_t pinconfig_offset = find_pinconfig(ardupin);

    int16_t gpioAdcPin = -1;
    float scaler = -1;

    if ((ardupin != ANALOG_INPUT_NONE) && (pinconfig_offset == -1 )) {
        DEV_PRINTF("AnalogIn: sorry channel() can't determine ADC1 offset from ardupin : %d \n",ardupin);
        ardupin = ANALOG_INPUT_NONE; // default it to this not terrible value and allow to continue
    }
    // although ANALOG_INPUT_NONE=255 is not a valid pin, we let it through here as
    //  a special case, so that it can be changed with set_pin(..) later.
    if (ardupin != ANALOG_INPUT_NONE) {
        gpioAdcPin = pin_config[(uint8_t)pinconfig_offset].channel;
        scaler = pin_config[(uint8_t)pinconfig_offset].scaling;
    }

    for (uint8_t j = 0; j < ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = new AnalogSource(ardupin,gpioAdcPin, scaler,0.0f,1);

            if (ardupin != ANALOG_INPUT_NONE) {
                DEV_PRINTF("AnalogIn: channel:%d attached to ardupin:%d at adc1_offset:%d on gpio:%d\n",\
                                    j,ardupin, gpioAdcPin, _channels[j]->_gpio);
            }

            if (ardupin == ANALOG_INPUT_NONE) {
                DEV_PRINTF("AnalogIn: channel:%d created but using delayed adc and gpio pin configuration\n",j );
            }

            return _channels[j];
        }
    }
    DEV_PRINTF("AnalogIn: out of channels\n");
    return nullptr;
}

#endif // HAL_USE_ADC
