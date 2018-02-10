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

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 20ms units) and the maximum time they are expected to take (in
  microseconds)
 */
AP_Task<Tracker> Tracker::scheduler_tasks[] = {
    make_task(&Tracker::update_ahrs,            50,   1000, "update_ahrs"),
    make_task(&Tracker::read_radio,             50,    200, "read_radio"),
    make_task(&Tracker::update_tracking,        50,   1000, "update_tracking"),
    make_task(&Tracker::update_GPS,             10,   4000, "update_GPS"),
    make_task(&Tracker::update_compass,         10,   1500, "update_compass"),
    make_task(&Tracker::update_battery,         10,   1500, "update_battery"),
    make_task(&Tracker::update_barometer,       10,   1500, "update_barometer"),
    make_task(&Tracker::gcs_update,             50,   1700, "gcs_update"),
    make_task(&Tracker::gcs_data_stream_send,   50,   3000, "gcs_data_stream_send"),
    make_task(&Tracker::compass_accumulate,     50,   1500, "compass_accumulate"),
    make_task(&Tracker::barometer_accumulate,   50,    900, "barometer_accumulate"),
    make_task(&Tracker::ten_hz_logging_loop,    10,    300, "ten_hz_logging_loop"),
    make_task(&Tracker::dataflash_periodic,     50,    300, "dataflash_periodic"),
    make_task(&Tracker::ins_periodic,           50,     50, "ins_periodic"),
    make_task(&Tracker::update_notify,          50,    100, "update_notify"),
    make_task(&Tracker::check_usb_mux,          10,    300, "check_usb_mux"),
    make_task(&Tracker::gcs_retry_deferred,     50,   1000, "gcs_retry_deferred"),
    make_task(&Tracker::one_second_loop,         1,   3900, "one_second_loop"),
    make_task(&Tracker::compass_cal_update,     50,    100, "compass_cal_update"),
    make_task(&Tracker::accel_cal_update,       10,    100, "accel_cal_update")
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
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

/**
   loop() is called continuously 
 */
void Tracker::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();
    
    scheduler.run(19900UL);
}

void Tracker::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

void Tracker::ins_periodic()
{
    ins.periodic();
}

void Tracker::one_second_loop()
{
    // send a heartbeat
    gcs().send_message(MSG_HEARTBEAT);

    // make it possible to change orientation at runtime
    ahrs.set_orientation();

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    // updated armed/disarmed status LEDs
    update_armed_disarmed();

    one_second_counter++;

    if (one_second_counter >= 60) {
        if (g.compass_enabled) {
            compass.save_offsets();
        }
        one_second_counter = 0;
    }
}

void Tracker::ten_hz_logging_loop()
{
    if (should_log(MASK_LOG_IMU)) {
        DataFlash.Log_Write_IMU(ins);
    }
    if (should_log(MASK_LOG_ATTITUDE)) {
        Log_Write_Attitude();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
}

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Tracker::Tracker(void)
    : DataFlash(fwver.fw_string, g.log_bitmask)
{
    memset(&current_loc, 0, sizeof(current_loc));
    memset(&vehicle, 0, sizeof(vehicle));
}

Tracker tracker;

AP_HAL_MAIN_CALLBACKS(&tracker);
