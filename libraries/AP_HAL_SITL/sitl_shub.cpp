#include <AP_HAL/AP_HAL.h>

#if HAL_SENSORHUB_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "Scheduler.h"
#include "UARTDriver.h"


#include <AP_Math/AP_Math.h>
#include <SITL/SITL.h>

#include <AP_SensorHub/AP_SensorHub.h>
#include <AP_SensorHub/AP_SensorHub_IO_FileDescriptor.h>

#include <AP_Baro/AP_Baro.h>
#include <AP_Baro/AP_Baro_SITL.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Compass/AP_Compass_SITL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_SITL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_GPS/GPS_Backend.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

using namespace HALSITL;
extern const AP_HAL::HAL& hal;

/* Simulates a SensorHub_Source vehicle type via UART */

static struct shub_state {
    int source_fd, sink_fd;
    uint32_t last_update;
} shub_state;

int SITL_State::shub_pipe(void)
{
    return shub_state.sink_fd;
}

void SITL_State::_shub_init()
{
    // We create a secondary AP_SensorHub to emulate a Source.
    _shub = new AP_SensorHub();
    _shub->setSourceMode();
    _shub_io = new AP_SensorHub_IO_FileDescriptor();
    _shub->registerIO(_shub_io);

    // Setup the Pipes for Source<->Sink communication
    int fd[2];
    pipe(fd);
    shub_state.source_fd = fd[1];
    shub_state.sink_fd = fd[0];
    HALSITL::UARTDriver::_set_nonblocking(shub_state.source_fd);
    HALSITL::UARTDriver::_set_nonblocking(shub_state.sink_fd);

    _shub_io->registerInput(shub_state.source_fd);
    _shub_io->registerOutput(shub_state.source_fd);

    // Manually configure the sensor backends.
    auto baro_backend = new AP_Baro_SITL(*_barometer);
    baro_backend->setSensorHub(_shub);
    _barometer->_add_backend(baro_backend);
    _barometer->ground_temp_init();

    auto compass_backend = new AP_Compass_SITL(*_compass);
    compass_backend->setSensorHub(_shub);
    _compass->_add_backend(compass_backend, nullptr, false);

    auto ins_backend = AP_InertialSensor_SITL::detect(*_ins);
    ins_backend->setSensorHub(_shub);
    _ins->_add_backend(ins_backend);
    ins_backend->start();
    _ins->init(1000, false);

    _serial_manager->init(); // NOTE: Initialize serial manager here as it
                             // happens before init_ardupilot. No harm in
                             // initializing twice from what I can tell.
    _gps->init(*_serial_manager, AP_SerialManager::SerialProtocol_GPS);
    _gps->update();

    // Wait for the gps driver to be initialized after the call to update().
    // This is to ensure there is a valid instance for which we can set the
    // SensorHub object.
    bool gpsInitialized = false;
    do {
        for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
            auto driver = _gps->get_driver(i);
            if (driver) {
                driver->setSensorHub(_shub);
                gpsInitialized = true;
            }
        }
        hal.scheduler->delay(1);
    } while (!gpsInitialized);

    fprintf(stdout, "SENSORHUB - Source - SITL INIT\n");
}

void SITL_State::_shub_update()
{
    // 50Hz
    if (AP_HAL::millis() - shub_state.last_update < 20) {
        return;
    }

    shub_state.last_update = AP_HAL::millis();

    // NOTE: Baro & GPS don't work like the other sensors.
    // They do not publish "raw" samples in comparison to INS and Compass.
    _barometer->update();
    _gps->update();
}
#endif