/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
   Lead developers: Matthew Ridley and Andrew Tridgell
 
   Please contribute your ideas! See http://dev.ardupilot.org for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Tracker.h"
#include "version.h"

#define SCHED_TASK(func) FUNCTOR_BIND(&tracker, &Tracker::func, void)

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 20ms units) and the maximum time they are expected to take (in
  microseconds)
 */
const AP_Scheduler::Task Tracker::scheduler_tasks[] PROGMEM = {
    { SCHED_TASK(update_ahrs),            1,   1000 },
    { SCHED_TASK(read_radio),             1,    200 },
    { SCHED_TASK(update_tracking),        1,   1000 },
    { SCHED_TASK(update_GPS),             5,   4000 },
    { SCHED_TASK(update_compass),         5,   1500 },
    { SCHED_TASK(update_barometer),       5,   1500 },
    { SCHED_TASK(gcs_update),             1,   1700 },
    { SCHED_TASK(gcs_data_stream_send),   1,   3000 },
    { SCHED_TASK(compass_accumulate),     1,   1500 },
    { SCHED_TASK(barometer_accumulate),   1,    900 },
    { SCHED_TASK(ten_hz_logging_loop),    5,    300 },
    { SCHED_TASK(dataflash_periodic),     1,    300 },
    { SCHED_TASK(update_notify),          1,    100 },
    { SCHED_TASK(check_usb_mux),          5,    300 },
    { SCHED_TASK(gcs_retry_deferred),     1,   1000 },
    { SCHED_TASK(one_second_loop),       50,   3900 },
    { SCHED_TASK(compass_cal_update),     1,    100 },
    { SCHED_TASK(accel_cal_update),       5,    100 }
};

/**
  setup the sketch - called once on startup
 */
void Tracker::setup() 
{
    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    // initialise notify
    notify.init(false);

    // antenna tracker does not use pre-arm checks or battery failsafe
    AP_Notify::flags.pre_arm_check = false;
    AP_Notify::flags.pre_arm_gps_check = false;
    AP_Notify::flags.failsafe_battery = false;

    init_tracker();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

/**
   loop() is called continuously 
 */
void Tracker::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    // tell the scheduler one tick has passed
    scheduler.tick();

    scheduler.run(19900UL);
}

void Tracker::dataflash_periodic(void)
{
    //DataFlash.periodic_tasks();
}

void Tracker::one_second_loop()
{
    // send a heartbeat
    gcs_send_message(MSG_HEARTBEAT);

    // make it possible to change orientation at runtime
    ahrs.set_orientation();

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // updated armed/disarmed status LEDs
    update_armed_disarmed();

    one_second_counter++;

    if (one_second_counter >= 60) {
        if(g.compass_enabled) {
            compass.save_offsets();
        }
        one_second_counter = 0;
    }
}

void Tracker::ten_hz_logging_loop()
{
    if (should_log(MASK_LOG_IMU)) {
        //DataFlash.Log_Write_IMU(ins);
    }
    if (should_log(MASK_LOG_ATTITUDE)) {
        Log_Write_Attitude();
    }
    if (should_log(MASK_LOG_RCIN)) {
        //DataFlash.Log_Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        //DataFlash.Log_Write_RCOUT();
    }
}

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

Tracker::Tracker(void)
    //: DataFlash{FIRMWARE_STRING}
{
    memset(&current_loc, 0, sizeof(current_loc));
    memset(&vehicle, 0, sizeof(vehicle));
}

Tracker tracker;

void setup(void);
void loop(void);

void setup(void)
{
    tracker.setup();
}
void loop(void)
{
    tracker.loop();
}

AP_HAL_MAIN();
