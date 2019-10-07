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

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)

#include "AnalogIn.h"

#ifndef ESP32_ADC_MAVLINK_DEBUG
// this allows the first 6 analog channels to be reported by mavlink for debugging purposes
#define ESP32_ADC_MAVLINK_DEBUG 0
#endif

#include <GCS_MAVLink/GCS_MAVLink.h>

#define ANLOGIN_DEBUGGING 0

// base voltage scaling for 12 bit 3.3V ADC
#define VOLTAGE_SCALING (3.3f/4096.0f)

#if ANLOGIN_DEBUGGING
# define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
# define Debug(fmt, args ...)
#endif


extern const AP_HAL::HAL &hal;

using namespace ESP32;

/*
   scaling table between ADC count and actual input voltage, to account
   for voltage dividers on the board.
   */
const AnalogIn::pin_info AnalogIn::pin_config[] = {
	{ADC_CHANNEL_6, 11}
};

#define ADC_GRP1_NUM_CHANNELS   ARRAY_SIZE(AnalogIn::pin_config)


#define DEFAULT_VREF    1100       //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   256          //Multisampling

static const adc_atten_t atten = ADC_ATTEN_DB_11;

AnalogSource::AnalogSource(int16_t pin, float initial_value, uint8_t unit) :
	_unit(unit),
	_pin(pin),
	_value(initial_value),
	_latest_value(initial_value),
	_sum_count(0),
	_sum_value(0)
{
	printf("Adding analogin on: %d\n", pin);
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
	printf("Adding gpio on: %d\n", gpio);
}


float AnalogSource::read_average()
{
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

/*
   return scaling from ADC count to Volts
   */
float AnalogSource::_pin_scaler()
{
	float scaling = VOLTAGE_SCALING;
	for (uint8_t i = 0; i < ADC_GRP1_NUM_CHANNELS; i++) {
		if (AnalogIn::pin_config[i].channel == _pin) {
			scaling = AnalogIn::pin_config[i].scaling;
			break;
		}
	}
	return scaling;
}

/*
   return voltage in Volts
   */
float AnalogSource::voltage_average()
{
	return _pin_scaler() * read_average();
}

/*
   return voltage in Volts
   */
float AnalogSource::voltage_latest()
{
	return _pin_scaler() * read_latest();
}

float AnalogSource::voltage_average_ratiometric()
{
	return _pin_scaler() * read_latest();
}

void AnalogSource::set_pin(uint8_t pin)
{
	if (_pin == pin) {
		return;
	}
	WITH_SEMAPHORE(_semaphore);
	_pin = pin;
	_sum_value = 0;
	_sum_count = 0;
	_latest_value = 0;
	_value = 0;
}

/*
   apply a reading in ADC counts
   */
void AnalogSource::_add_value()
{
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
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
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

AP_HAL::AnalogSource *AnalogIn::channel(int16_t pin)
{
	for (uint8_t j = 0; j < ANALOG_MAX_CHANNELS; j++) {
		if (_channels[j] == nullptr) {
			_channels[j] = new AnalogSource(pin, 0.0f);
			return _channels[j];
		}
	}
	hal.console->printf("Out of analog channels\n");
	return nullptr;
}

#endif // HAL_USE_ADC
