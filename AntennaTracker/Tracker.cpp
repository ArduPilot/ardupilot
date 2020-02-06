/*
   Lead developers: Matthew Ridley and Andrew Tridgell
 
   Please contribute your ideas! See https://dev.ardupilot.org for details

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
    SCHED_TASK(compass_save,           0.02,   200),
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
    SCHED_TASK_CLASS(Compass,          &tracker.compass,              cal_update, 50, 100),
    SCHED_TASK(stats_update,            1,    200),
    SCHED_TASK(accel_cal_update,       10,    100)
};

void Tracker::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = (uint32_t)-1;
}

void Tracker::one_second_loop()
{
    // make it possible to change orientation at runtime
    ahrs.update_orientation();

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // updated armed/disarmed status LEDs
    update_armed_disarmed();

    if (!ahrs.home_is_set()) {
        // set home to current location
        Location temp_loc;
        if (ahrs.get_location(temp_loc)) {
            if (!set_home(temp_loc)){
                // fail silently
            }
        }
    }

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    set_likely_flying(hal.util->get_soft_armed());

    AP_Notify::flags.flying = hal.util->get_soft_armed();
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

Mode *Tracker::mode_from_mode_num(const Mode::Number num)
{
    Mode *ret = nullptr;
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::STOP:
        ret = &mode_stop;
        break;
    case Mode::Number::SCAN:
        ret = &mode_scan;
        break;
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::SERVOTEST:
        ret = &mode_servotest;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::INITIALISING:
        ret = &mode_initialising;
        break;
    }
    return ret;
}

/*
  update AP_Stats
*/
void Tracker::stats_update(void)
{
    stats.set_flying(hal.util->get_soft_armed());
    stats.update();
}

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Tracker::Tracker(void)
    : logger(g.log_bitmask)
{
}

Tracker tracker;
AP_Vehicle& vehicle = tracker;

AP_HAL_MAIN_CALLBACKS(&tracker);
