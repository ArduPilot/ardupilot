#include "AP_SensorHub_IO.h"

#if HAL_SENSORHUB_ENABLED
extern const AP_HAL::HAL& hal;

AP_SensorHub_IO::AP_SensorHub_IO()
{
    _sem_write = hal.util->new_semaphore();
}

#endif