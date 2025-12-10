#pragma once

#include "AP_HAL_Embox.h"
#include "GPIO.h"

namespace Embox
{

class DigitalSource: public AP_HAL::DigitalSource {
public:
	DigitalSource(uint8_t v);
	void mode(uint8_t output) override;
	uint8_t read() override;
	void write(uint8_t value) override;
	void toggle() override;

private:
	uint8_t _v;
};

class GPIO: public AP_HAL::GPIO {
public:
	static GPIO *from(AP_HAL::GPIO *gpio) {
		return static_cast<GPIO *>(gpio);
	}

	void init() override;

	void pinMode(uint8_t vpin, uint8_t output) override;
	uint8_t read(uint8_t vpin) override;
	void write(uint8_t vpin, uint8_t value) override;
	void toggle(uint8_t vpin) override;

	/*
     * Export pin, instantiate a new DigitalSource and return its
     * pointer.
     */
	AP_HAL::DigitalSource *channel(uint16_t vpin) override;

	bool usb_connected() override;
};

} // namespace Linux
