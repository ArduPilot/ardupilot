/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <AP_Baro.h>
#include "AP_Baro_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_baro.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

extern const AP_HAL::HAL& hal;

// Public Methods //////////////////////////////////////////////////////////////
bool AP_Baro_PX4::init(void)
{
	_baro_fd = open(BARO_DEVICE_PATH, O_RDONLY);
	if (_baro_fd < 0) {
        hal.scheduler->panic("Unable to open " BARO_DEVICE_PATH);
	}

	/* set the driver to poll at 150Hz */
	ioctl(_baro_fd, SENSORIOCSPOLLRATE, 150);

    // average over up to 10 samples
    ioctl(_baro_fd, SENSORIOCSQUEUEDEPTH, 10);

    return true;
}

// Read the sensor
uint8_t AP_Baro_PX4::read(void)
{
    uint16_t count;
    float pressure_sum, temperature_sum;
    struct baro_report baro_report;

    // read all available samples and average
    pressure_sum = 0;
    temperature_sum = 0;
    count = 0;

    while (::read(_baro_fd, &baro_report, sizeof(baro_report)) == sizeof(baro_report)) {
		pressure_sum += baro_report.pressure; // Pressure in mbar
		temperature_sum += baro_report.temperature; // degrees celcius
        count++;
    }

    if (count != 0) {
        _pressure = pressure_sum / count;
        _temperature = temperature_sum / count;
        _last_update = hal.scheduler->millis();
        _pressure_samples = count;
        healthy = true;
        return 1;
    }
    
    return 0;
}

float AP_Baro_PX4::get_pressure() {
    return _pressure;
}

float AP_Baro_PX4::get_temperature() {
    return _temperature;
}

int32_t AP_Baro_PX4::get_raw_pressure() {
    return _pressure;
}

int32_t AP_Baro_PX4::get_raw_temp() {
    return _temperature;
}

#endif // CONFIG_HAL_BOARD
