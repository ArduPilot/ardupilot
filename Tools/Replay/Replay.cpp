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

#include "Replay.h"

#include "LogReader.h"

#include <stdio.h>
#include <AP_HAL/utility/getopt_cpp.h>

#include <AP_Vehicle/AP_Vehicle.h>

#include <GCS_MAVLink/GCS_Dummy.h>

#include <AP_HAL_Linux/Scheduler.h>

#define streq(x, y) (!strcmp(x, y))

static ReplayVehicle replayvehicle;

#define GSCALAR(v, name, def) { replayvehicle.g.v.vtype, name, Parameters::k_param_ ## v, &replayvehicle.g.v, {def_value : def} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &replayvehicle.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &replayvehicle.v, {group_info : class::var_info} }

const AP_Param::Info ReplayVehicle::var_info[] = {
    GSCALAR(dummy,         "_DUMMY", 0),

    // barometer ground calibration. The GND_ prefix is chosen for
    // compatibility with previous releases of ArduPlane
    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                    "INS_", AP_InertialSensor),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // @Group: ARSPD_
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    GOBJECT(airspeed,                               "ARSP_",   AP_Airspeed),

    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(ekf2, NavEKF2, "EK2_", NavEKF2),
    
    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass, "COMPASS_", Compass),

    // @Group: LOG
    // @Path: ../libraries/AP_Logger/AP_Logger.cpp
    GOBJECT(logger, "LOG", AP_Logger),
    
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ekf3, NavEKF3, "EK3_", NavEKF3),

    // @Group: GPS_
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),
    
    AP_VAREND
};

void ReplayVehicle::load_parameters(void)
{
    unlink("Replay.stg");
    if (!AP_Param::check_var_info()) {
        AP_HAL::panic("Bad parameter table");
    }
    // Load all auto-loaded EEPROM variables - also registers thread
    // which saves parameters, which Compass now does in its init() routine
    AP_Param::load_all();
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_AdvancedFailsafe *AP::advancedfailsafe() { return nullptr; }
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) { return false; }

// dummy method to avoid linking AP_Avoidance
// AP_Avoidance *AP::ap_avoidance() { return nullptr; }

// avoid building/linking LTM:
void AP_LTM_Telem::init() {};
// avoid building/linking Devo:
void AP_DEVO_Telem::init() {};

void ReplayVehicle::init_ardupilot(void)
{
    // we pass an empty log structure, filling the structure in with
    // either the format present in the log (if we do not emit the
    // message as a product of Replay), or the format understood in
    // the current code (if we do emit the message in the normal
    // places in the EKF, for example)
    logger.Init(log_structure, 0);
    logger.set_force_log_disarmed(true);
}

void Replay::_parse_command_line(uint8_t argc, char * const argv[])
{
    const struct GetOptLong::option options[] = {
        // name           has_arg flag   val
        {0, false, 0, 0}
    };

    GetOptLong gopt(argc, argv, "r:p:ha:g:A:n", options);

    int opt;
    while ((opt = gopt.getoption()) != -1) {
    }
       argv += gopt.optind;
       argc -= gopt.optind;

    if (argc > 0) {
        filename = argv[0];
    }
}

void Replay::setup()
{
    ::printf("Starting\n");

    uint8_t argc;
    char * const *argv;

    hal.util->commandline_arguments(argc, argv);

    _parse_command_line(argc, argv);

    _vehicle.setup();
}

void Replay::loop()
{
    // LogReader reader = LogReader(log_structure);
    if (!reader.open_log(filename)) {
        ::fprintf(stderr, "open(%s): %m\n", filename);
        exit(1);
    }
    while (reader.update()) {
    }

    // If we don't tear down the threads then they continue to access
    // global state during object destruction.
    ((Linux::Scheduler*)hal.scheduler)->teardown();

    exit(0);
}

Replay replay(replayvehicle);
AP_Vehicle& vehicle = replayvehicle;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_HAL_MAIN_CALLBACKS(&replay);
