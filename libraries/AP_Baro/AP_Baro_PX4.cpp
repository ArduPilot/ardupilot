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
    if (_baro_fd <= 0) {
        _baro_fd = open(BARO_DEVICE_PATH, O_RDONLY);
        if (_baro_fd < 0) {
            hal.scheduler->panic("Unable to open " BARO_DEVICE_PATH);
        }

        /* set the driver to poll at 150Hz */
        ioctl(_baro_fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX);

        // average over up to 10 samples
        ioctl(_baro_fd, SENSORIOCSQUEUEDEPTH, 10);
    }

    return true;
}

// Read the sensor
uint8_t AP_Baro_PX4::read(void)
{
    accumulate();
    if (_sum_count == 0) {
        // no data available
        return 0;
    }

    _pressure    = (_pressure_sum / _sum_count) * 100.0f;
    _temperature = (_temperature_sum / _sum_count) * 10.0f;
    _pressure_samples = _sum_count;

    _pressure_sum = 0;
    _temperature_sum = 0;
    _sum_count = 0;

    healthy = true;
    return 1;
}

// accumulate sensor values
void AP_Baro_PX4::accumulate(void)
{
    struct baro_report baro_report;

    while (::read(_baro_fd, &baro_report, sizeof(baro_report)) == sizeof(baro_report)) {
		_pressure_sum += baro_report.pressure; // Pressure in mbar
		_temperature_sum += baro_report.temperature; // degrees celcius
        _sum_count++;
        _last_update = hal.scheduler->millis();
    }
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
