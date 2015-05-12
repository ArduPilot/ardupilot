/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
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
/* 
   main Rover class, containing all vehicle specific state
*/


/*
  scheduler table - all regular tasks should be listed here, along
  with how often they should be called (in 20ms units) and the maximum
  time they are expected to take (in microseconds)
*/
const AP_Scheduler::Task Rover::scheduler_tasks[] PROGMEM = {
	{ read_radio,             1,   1000 },
    { ahrs_update,            1,   6400 },
    { read_sonars,            1,   2000 },
    { update_current_mode,    1,   1500 },
    { set_servos,             1,   1500 },
    { update_GPS_50Hz,        1,   2500 },
    { update_GPS_10Hz,        5,   2500 },
    { update_alt,             5,   3400 },
    { navigate,               5,   1600 },
    { update_compass,         5,   2000 },
    { update_commands,        5,   1000 },
    { update_logging1,        5,   1000 },
    { update_logging2,        5,   1000 },
    { gcs_retry_deferred,     1,   1000 },
    { gcs_update,             1,   1700 },
    { gcs_data_stream_send,   1,   3000 },
    { read_control_switch,   15,   1000 },
    { read_trim_switch,       5,   1000 },
    { read_battery,           5,   1000 },
    { read_receiver_rssi,     5,   1000 },
    { update_events,          1,   1000 },
    { check_usb_mux,         15,   1000 },
    { mount_update,           1,    600 },
    { gcs_failsafe_check,     5,    600 },
    { compass_accumulate,     1,    900 },
    { update_notify,          1,    300 },
    { one_second_loop,       50,   3000 },
#if FRSKY_TELEM_ENABLED == ENABLED
    { frsky_telemetry_send,  10,    100 }
#endif
};


Rover::Rover(void) :
    param_loader(var_info)
    channel_steer(NULL),
    channel_throttle(NULL),
    channel_learn(NULL),
    in_log_download(false),
    modes(&g.mode1),
#if AP_AHRS_NAVEKF_AVAILABLE
    ahrs(ins, barometer, gps, sonar),
#else
    ahrs(ins, barometer, gps),
#endif
    L1_controller(ahrs),
    nav_controller(&L1_controller),
    steerController(ahrs),
    mission(ahrs, &start_command, &verify_command, &exit_mission),
    ServoRelayEvents(relay),
#if CAMERA == ENABLED
    AP_camera(&relay),
#endif
#if MOUNT == ENABLED
    camera_mount(ahrs, current_loc),
#endif
    control_mode(INITIALISING),
    ground_start_count(20),
    throttle(500),
    frsky_telemetry(ahrs, battery),
    home(ahrs.get_home()),
    G_Dt(0.02),
{
}
