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
    int fd;

	fd = open(BARO_DEVICE_PATH, 0);
	if (fd < 0) {
        hal.scheduler->panic("Unable to open " BARO_DEVICE_PATH);
	}

	/* set the driver to poll at 150Hz */
	ioctl(fd, SENSORIOCSPOLLRATE, 150);
    close(fd);

	_baro_sub = orb_subscribe(ORB_ID(sensor_baro));

    return true;
}

// Read the sensor
uint8_t AP_Baro_PX4::read()
{
	bool baro_updated;
	orb_check(_baro_sub, &baro_updated);

	if (baro_updated) {
		struct baro_report	baro_report;

		orb_copy(ORB_ID(sensor_baro), _baro_sub, &baro_report);

		_pressure = baro_report.pressure; // Pressure in mbar
		_temperature = baro_report.temperature; // Temperature in degrees celcius
        _pressure_samples = 1;
        _last_update = hal.scheduler->millis();
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
