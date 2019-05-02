#pragma once

#include "AP_HAL_Linux.h"
#include "Imx_IOMCU.h"

extern const AP_HAL::HAL &hal;

class AnalogSource_Imx_IOMCU : public AP_HAL::AnalogSource {
public:
	friend class AnalogIn_Imx_IOMCU;
	AnalogSource_Imx_IOMCU(int16_t pin);
	float read_average();
	float read_latest();
	void set_pin(uint8_t p);
	void set_stop_pin(uint8_t p);
	void set_settle_time(uint16_t settle_time_ms);
	float voltage_average();
	float voltage_latest();
	float voltage_average_ratiometric();
private:
	HAL_Semaphore _semaphore;
	int16_t _pin;
	float _value;
};

class AnalogIn_Imx_IOMCU : public AP_HAL::AnalogIn {
public:
	AnalogIn_Imx_IOMCU(Imx_IOMCU *m);
	void init() override;
	AP_HAL::AnalogSource *channel(int16_t n) override;

	/* Board voltage is not available */
	float board_voltage() override;

private:
	void _update();

	AnalogSource_Imx_IOMCU *_channels[2];
	uint32_t _last_update_timestamp;
	Imx_IOMCU *mcu;
};