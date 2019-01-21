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

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(Tracker, &tracker, func, _interval_ticks, _max_time_micros)

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 20ms units) and the maximum time they are expected to take (in
  microseconds)
 */
const AP_Scheduler::Task Tracker::scheduler_tasks[] = {
    SCHED_TASK(update_ahrs,            50,   1000),
    SCHED_TASK(read_radio,             50,    200),
    SCHED_TASK(update_tracking,        50,   1000),
    SCHED_TASK(update_GPS,             10,   4000),
    SCHED_TASK(update_compass,         10,   1500),
    SCHED_TASK_CLASS(AP_BattMonitor,    &tracker.battery,   read,           10, 1500),
    SCHED_TASK_CLASS(AP_Baro,          &tracker.barometer,  update,         10,   1500),
    SCHED_TASK_CLASS(GCS,              (GCS*)&tracker._gcs, update_receive, 50, 1700),
    SCHED_TASK_CLASS(GCS,              (GCS*)&tracker._gcs, update_send,    50, 3000),
    SCHED_TASK_CLASS(AP_Baro,           &tracker.barometer, accumulate,     50,  900),
    SCHED_TASK(ten_hz_logging_loop,    10,    300),
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Logger,   &tracker.logger, periodic_tasks, 50,  300),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &tracker.ins,       periodic,       50,   50),
    SCHED_TASK_CLASS(AP_Notify,         &tracker.notify,    update,         50,  100),
    SCHED_TASK(one_second_loop,         1,   3900),
    SCHED_TASK(compass_cal_update,     50,    100),
    SCHED_TASK(accel_cal_update,       10,    100)
};

/**
  setup the sketch - called once on startup
 */
void Tracker::setup() 
{
    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    init_tracker();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), (uint32_t)-1);
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

void Tracker::one_second_loop()
{
    // send a heartbeat
    gcs().send_message(MSG_HEARTBEAT);

    // make it possible to change orientation at runtime
    ahrs.set_orientation();

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // updated armed/disarmed status LEDs
    update_armed_disarmed();

    one_second_counter++;

    if (one_second_counter >= 60) {
        if (g.compass_enabled) {
            compass.save_offsets();
        }
        one_second_counter = 0;
    }

    if (!ahrs.home_is_set()) {
        // set home to current location
        Location temp_loc;
        if (ahrs.get_location(temp_loc)) {
            set_home(temp_loc);
        }
    }

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    ahrs.set_likely_flying(hal.util->get_soft_armed());
}

void Tracker::ten_hz_logging_loop()
{
    if (should_log(MASK_LOG_IMU)) {
        logger.Write_IMU();
    }
    if (should_log(MASK_LOG_ATTITUDE)) {
        Log_Write_Attitude();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
}

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Tracker::Tracker(void)
    : logger(g.log_bitmask)
{
    memset(&vehicle, 0, sizeof(vehicle));
}

Tracker tracker;

AP_HAL_MAIN_CALLBACKS(&tracker);
