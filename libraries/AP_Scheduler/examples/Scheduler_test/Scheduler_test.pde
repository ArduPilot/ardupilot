// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_Scheduler interface
//

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_PX4.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// INS declaration
static AP_InertialSensor ins;

// loop scheduler object
static AP_Scheduler scheduler;

// counter for ins_update()
static uint32_t ins_counter;

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { ins_update,             1,   1000 },
    { one_hz_print,          50,   1000 },
    { five_second_call,     250,   1800 },
};


void setup(void)
{
    // we 
    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_50HZ);

    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

void loop(void)
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
static void ins_update(void)
{
    ins_counter++;
    ins.update();
}

/*
  print something once a second
 */
static void one_hz_print(void)
{
    hal.console->printf("one_hz: t=%lu\n", hal.scheduler->millis());
}

/*
  print something every 5 seconds
 */
static void five_second_call(void)
{
    hal.console->printf("five_seconds: t=%lu ins_counter=%u\n", hal.scheduler->millis(), ins_counter);
}

AP_HAL_MAIN();
