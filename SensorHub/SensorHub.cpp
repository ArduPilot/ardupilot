#include "SensorHub.h"
#include <AP_GPS/GPS_Backend.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if HAL_SENSORHUB_ENABLED

SensorHub_Source sensorhub_source;

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(SensorHub_Source, &sensorhub_source, func, rate_hz, max_time_micros)
#define GSCALAR(v, name, def) { sensorhub_source.g.v.vtype, name, Parameters::k_param_ ## v, &sensorhub_source.g.v, {def_value : def} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&sensorhub_source.v, {group_info : class::var_info} }

const AP_Scheduler::Task SensorHub_Source::scheduler_tasks[] = {
    SCHED_TASK(update_barometer, 100, 100),
    SCHED_TASK(update_gps,       50, 200)
};

const AP_Param::Info SensorHub_Source::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @Values: 0:ArduPlane,4:AntennaTracker,10:Copter,20:Rover
    // @User: Advanced
    // @ReadOnly: True
    GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),


    GOBJECT(compass, "COMPASS_", Compass),
    GOBJECT(ins, "INS_", AP_InertialSensor),
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),
    GOBJECT(barometer, "GND_", AP_Baro),
    GOBJECT(gps, "GPS_", AP_GPS),
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),
    AP_VAREND
};

SensorHub_Source::SensorHub_Source(void) :
    shub(AP_SensorHub::init_instance())
{}

void SensorHub_Source::update_barometer()
{
    barometer.update();
}

void SensorHub_Source::update_gps()
{
    gps.update();
}

void SensorHub_Source::load_parameters()
{
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->printf("done.\n");
    }

    uint32_t before = AP_HAL::micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    hal.console->printf("load_all took %luus\n", (unsigned long)(AP_HAL::micros() - before));
}
void SensorHub_Source::setup()
{
    AP_Param::setup_sketch_defaults();
    serial_manager.init_console();
    serial_manager.init();
    load_parameters();
    BoardConfig.init();
    shub->init(serial_manager);

    barometer.init();
    barometer.calibrate();

    compass.init();

    gps.init(serial_manager, AP_SerialManager::SerialProtocol_GPS);
    gps.update();

    bool gpsInitialized = false;
    do {
        for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
            auto driver = gps.get_driver(i);
            if (driver) {
                driver->setSensorHub(shub);
                gpsInitialized = true;
                hal.console->printf("Initialized GPS\n");
            }
        }
        gps.update();
        hal.scheduler->delay(1);
    } while (!gpsInitialized);

    ins.init(scheduler.get_loop_rate_hz());
    shub->start();

    hal.console->printf("Initializing Scheduler...\n");
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

void SensorHub_Source::loop()
{
    ins.wait_for_sample();
    ins.update();

    auto timer = AP_HAL::micros();
    scheduler.tick();
    auto time_available = (timer + MAIN_LOOP_MICROS) - AP_HAL::micros();
    scheduler.run(time_available > MAIN_LOOP_MICROS ? 0u : time_available);
}

AP_HAL_MAIN_CALLBACKS(&sensorhub_source);
#else
void setup() {}
void loop() {}
AP_HAL_MAIN();
#endif