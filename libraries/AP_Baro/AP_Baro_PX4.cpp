/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <AP_Baro.h>
#include "AP_Baro_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>

extern const AP_HAL::HAL& hal;

/*
  constructor - opens the PX4 drivers
 */
AP_Baro_PX4::AP_Baro_PX4(AP_Baro &baro) :
    AP_Baro_Backend(baro),
    _num_instances(0)
{
    memset(instances, 0, sizeof(instances));
    instances[0].fd = open(BARO_DEVICE_PATH, O_RDONLY);
    instances[1].fd = open(BARO_DEVICE_PATH"1", O_RDONLY);

    for (uint8_t i=0; i<2; i++) {
        if (instances[i].fd != -1) {
            _num_instances = i+1;
        } else {
            break;
        }
    }

    for (uint8_t i=0; i<_num_instances; i++) {
        instances[i].instance = _frontend.register_sensor();

        /* set the driver to poll at 150Hz */
        ioctl(instances[i].fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX);

        // average over up to 20 samples
        ioctl(instances[i].fd, SENSORIOCSQUEUEDEPTH, 20);
    }
}

// Read the sensor
void AP_Baro_PX4::update(void)
{
    for (uint8_t i=0; i<_num_instances; i++) {
        struct baro_report baro_report;
        struct px4_instance &instance = instances[i];
        while (::read(instance.fd, &baro_report, sizeof(baro_report)) == sizeof(baro_report) &&
               baro_report.timestamp != instance.last_timestamp) {
            instance.pressure_sum += baro_report.pressure; // Pressure in mbar
            instance.temperature_sum += baro_report.temperature; // degrees celcius
            instance.sum_count++;
            instance.last_timestamp = baro_report.timestamp;
        }
    }

    for (uint8_t i=0; i<_num_instances; i++) {
        struct px4_instance &instance = instances[i];
        if (instance.sum_count > 0) {
            float pressure = (instance.pressure_sum / instance.sum_count) * 100;
            float temperature = instance.temperature_sum / instance.sum_count;
            instance.pressure_sum = 0;
            instance.temperature_sum = 0;
            instance.sum_count = 0;
            _copy_to_frontend(instance.instance, pressure, temperature);
        }
    }
}

#endif // CONFIG_HAL_BOARD
