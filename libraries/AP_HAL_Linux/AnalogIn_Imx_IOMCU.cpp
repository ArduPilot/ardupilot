#include "AnalogIn_Imx_IOMCU.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Semaphore.h>

//AnalogSource_Imx_IOMCU
AnalogSource_Imx_IOMCU::AnalogSource_Imx_IOMCU(int16_t pin)
{
	_pin = pin;
	_value = 0;
}

float AnalogSource_Imx_IOMCU::read_average()
{
	return read_latest();
}

float AnalogSource_Imx_IOMCU::read_latest()
{
	WITH_SEMAPHORE(_semaphore);
	return _value;
}

void AnalogSource_Imx_IOMCU::set_pin(uint8_t p)
{
	WITH_SEMAPHORE(_semaphore);
	_pin = p;
}

void AnalogSource_Imx_IOMCU::set_stop_pin(uint8_t p)
{
}

void AnalogSource_Imx_IOMCU::set_settle_time(uint16_t settle_time_ms)
{
}

float AnalogSource_Imx_IOMCU::voltage_average()
{
	WITH_SEMAPHORE(_semaphore);
	return _value;
}

float AnalogSource_Imx_IOMCU::voltage_latest()
{
	WITH_SEMAPHORE(_semaphore);
	return _value;
}

float AnalogSource_Imx_IOMCU::voltage_average_ratiometric()
{
	WITH_SEMAPHORE(_semaphore);
	return _value;
}

//AnalogIn_Imx_IOMCU
AnalogIn_Imx_IOMCU::AnalogIn_Imx_IOMCU(Imx_IOMCU *m)
{
	mcu = m;
}

void AnalogIn_Imx_IOMCU::init()
{
	hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AnalogIn_Imx_IOMCU::_update, void));
}

AP_HAL::AnalogSource *AnalogIn_Imx_IOMCU::channel(int16_t n)
{
	for (uint8_t j = 0; j < 2; j++) {
		if (_channels[j] == nullptr) {
			_channels[j] = new AnalogSource_Imx_IOMCU(n);
			return _channels[j];
		}
	}

	hal.console->printf("Out of analog channels\n");
	return nullptr;
}

float AnalogIn_Imx_IOMCU::board_voltage()
{
	return 5.0f;
}

void AnalogIn_Imx_IOMCU::_update()
{
	if (AP_HAL::micros() - _last_update_timestamp < 50000) {
		return;
	}

	int adc_pc0_data;
	int adc_pc1_data;

	if (mcu->read_adc(&adc_pc0_data, &adc_pc1_data))
	{
		for (size_t i = 0; i < 2; i++) {
			AnalogSource_Imx_IOMCU *source = _channels[i];
			if (source != nullptr)
			{
				WITH_SEMAPHORE(source->_semaphore);
				if (source->_pin == 0)
				{
					source->_value = adc_pc0_data * 3.3 / 4096.0;
				}
				else if (source->_pin == 1)
				{
					source->_value = adc_pc1_data * 3.3 / 4096.0;
				}
			}
		}

		_last_update_timestamp = AP_HAL::micros();
	}
}
