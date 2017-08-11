#include <AP_SensorHub/AP_SensorHub.h>

#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#include <stdio.h>
#include <unistd.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if HAL_SENSORHUB_ENABLED
using namespace SensorHub;

class SensorHub_Sink : public AP_HAL::HAL::Callbacks {
public:
    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;
    void receive();

    SensorHub_Sink() :
        shub(AP_SensorHub::init_instance()),
        param_loader(var_info)
    {}

    AP_SensorHub *shub;

    AP_SerialManager serial_manager;

    AP_Baro baro;
    AP_InertialSensor ins;
    Compass compass;
    AP_GPS gps;

    AP_HAL::Stream *shub_uart;

    AP_Param param_loader;
    static const AP_Param::Info var_info[];
};

static SensorHub_Sink sink;

// TODO: parameters must be setup correctly for INS to work.
const AP_Param::Info SensorHub_Sink::var_info[] = {
    {AP_PARAM_GROUP, "INS_", 0, &sink.ins, {group_info : AP_InertialSensor::var_info}},
    AP_VAREND
};

void SensorHub_Sink::setup()
{
    fprintf(stdout, "SensorHub Sink library example\n");

    AP_BoardConfig{} .init();
    param_loader.erase_all();
    serial_manager.init();
    shub->init(serial_manager);

    fprintf(stdout, "Initializing Barometer & Compass\n");
    baro.init();
    if(!compass.init() || !compass.read()) {
        hal.console->printf("CONSOLE INIT ERROR\n");
    }
    // NOTE: Cannot use init until AP_Param is setup correctly.
    //ins.init(AP_SensorHub::UPDATE_RATE_HZ);
    gps.init(serial_manager, AP_SerialManager::SerialProtocol_SENSORHUB);

    hal.scheduler->delay(1000);
}

void SensorHub_Sink::loop()
{
    hal.scheduler->delay(1);
}

AP_HAL_MAIN_CALLBACKS(&sink);
#else
void setup() {}
void loop() {}
AP_HAL_MAIN();
#endif