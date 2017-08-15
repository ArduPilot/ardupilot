#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AccelCal/AP_AccelCal.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Declination/AP_Declination.h>
#include <DataFlash/DataFlash.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include <AP_Vehicle/AP_Vehicle.h>

#include <AP_BoardConfig/AP_BoardConfig.h>

#include <AP_SensorHub/AP_SensorHub.h>

#include "defines.h"
#include "config.h"
#include "Parameters.h"

#if HAL_SENSORHUB_ENABLED
class SensorHub_Source : public AP_HAL::HAL::Callbacks {
public:
    friend class Parameters;

    SensorHub_Source(void);

    void setup() override;
    void loop() override;

private:
    Parameters g;
    AP_Scheduler scheduler;
    AP_GPS gps;
    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;

    AP_SerialManager serial_manager;
    AP_BoardConfig BoardConfig;

    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];
    static const AP_Scheduler::Task scheduler_tasks[];

    AP_SensorHub *shub;

public:
    void update_barometer();
    void update_gps();
    void load_parameters();
};

extern SensorHub_Source sensorhub_source;
#endif