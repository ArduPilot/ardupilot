// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_Scheduler interface
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Baro/AP_Baro.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <Filter/Filter.h>
#include <SITL/SITL.h>
#include <AP_Buffer/AP_Buffer.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <DataFlash/DataFlash.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

class SchedTest {
public:
    void setup();
    void loop();

private:

    AP_InertialSensor ins;
    AP_Scheduler scheduler;

    uint32_t ins_counter;
    static const AP_Scheduler::Task scheduler_tasks[];

    void ins_update(void);
    void one_hz_print(void);
    void five_second_call(void);
};

static SchedTest schedtest;

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) {\
    .function = FUNCTOR_BIND(&schedtest, &SchedTest::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(func)\
    .interval_ticks = _interval_ticks,\
    .max_time_micros = _max_time_micros,\
}

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task SchedTest::scheduler_tasks[] PROGMEM = {
    SCHED_TASK(ins_update,              1,   1000),
    SCHED_TASK(one_hz_print,           50,   1000),
    SCHED_TASK(five_second_call,      250,   1800),
};


void SchedTest::setup(void)
{
    // we 
    ins.init(AP_InertialSensor::RATE_50HZ);

    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

void SchedTest::loop(void)
{
    // wait for an INS sample
    ins.wait_for_sample();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all tasks that fit in 20ms
    scheduler.run(20000);
}

/*
  update inertial sensor, reading data 
 */
void SchedTest::ins_update(void)
{
    ins_counter++;
    ins.update();
}

/*
  print something once a second
 */
void SchedTest::one_hz_print(void)
{
    hal.console->printf("one_hz: t=%lu\n", (unsigned long)hal.scheduler->millis());
}

/*
  print something every 5 seconds
 */
void SchedTest::five_second_call(void)
{
    hal.console->printf("five_seconds: t=%lu ins_counter=%u\n", (unsigned long)hal.scheduler->millis(), ins_counter);
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup(void)
{
    schedtest.setup();
}
void loop(void)
{
    schedtest.loop();
}
AP_HAL_MAIN();
