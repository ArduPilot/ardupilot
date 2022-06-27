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
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Filesystem/posix_compat.h>
#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/Scheduler.h>
#endif

#define streq(x, y) (!strcmp(x, y))

static ReplayVehicle replayvehicle;

// list of user parameters
user_parameter *user_parameters;
bool replay_force_ekf2;
bool replay_force_ekf3;

#define GSCALAR(v, name, def) { replayvehicle.g.v.vtype, name, Parameters::k_param_ ## v, &replayvehicle.g.v, {def_value : def} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &replayvehicle.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &replayvehicle.v, {group_info : class::var_info} }

const AP_Param::Info ReplayVehicle::var_info[] = {
    GSCALAR(dummy,         "_DUMMY", 0),

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                    "INS_", AP_InertialSensor),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if AP_AIRSPEED_ENABLED
    // @Group: ARSPD_
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    GOBJECT(airspeed,                               "ARSP_",   AP_Airspeed),
#endif

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

    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),
    
    AP_VAREND
};

void ReplayVehicle::load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        AP_HAL::panic("Bad parameter table");
    }
    StorageManager::erase();
    AP_Param::erase_all();
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

#if AP_LTM_TELEM_ENABLED
// avoid building/linking LTM:
void AP_LTM_Telem::init() {};
#endif
#if AP_DEVO_TELEM_ENABLED
// avoid building/linking Devo:
void AP_DEVO_Telem::init() {};
#endif

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

void Replay::usage(void)
{
    ::printf("Options:\n");
    ::printf("\t--parm NAME=VALUE  set parameter NAME to VALUE\n");
    ::printf("\t--param-file FILENAME  load parameters from a file\n");
    ::printf("\t--force-ekf2 force enable EKF2\n");
    ::printf("\t--force-ekf3 force enable EKF3\n");
}

enum param_key : uint8_t {
    FORCE_EKF2 = 1,
    FORCE_EKF3,
};

void Replay::_parse_command_line(uint8_t argc, char * const argv[])
{
    const struct GetOptLong::option options[] = {
        // name           has_arg flag   val
        {"parm",            true,   0, 'p'},
        {"param",           true,   0, 'p'},
        {"param-file",      true,   0, 'F'},
        {"force-ekf2",      false,  0, param_key::FORCE_EKF2},
        {"force-ekf3",      false,  0, param_key::FORCE_EKF3},
        {"help",            false,  0, 'h'},
        {0, false, 0, 0}
    };

    GetOptLong gopt(argc, argv, "p:F:h", options);

    int opt;
    while ((opt = gopt.getoption()) != -1) {
		switch (opt) {
        case 'p': {
            const char *eq = strchr(gopt.optarg, '=');
            if (eq == NULL) {
                ::printf("Usage: -p NAME=VALUE\n");
                exit(1);
            }
            struct user_parameter *u = new user_parameter;
            strncpy(u->name, gopt.optarg, eq-gopt.optarg);
            u->value = atof(eq+1);
            u->next = user_parameters;
            user_parameters = u;
            break;
        }

        case 'F':
            load_param_file(gopt.optarg);
            break;

        case param_key::FORCE_EKF2:
            replay_force_ekf2 = true;
            break;

        case param_key::FORCE_EKF3:
            replay_force_ekf3 = true;
            break;

        case 'h':
        default:
            usage();
            exit(0);
        }
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

    if (argc > 0) {
        _parse_command_line(argc, argv);
    }

    _vehicle.setup();

    set_user_parameters();

    if (replay_force_ekf2) {
        reader.set_parameter("EK2_ENABLE", 1, true);
    }
    if (replay_force_ekf3) {
        reader.set_parameter("EK3_ENABLE", 1, true);
    }

    if (replay_force_ekf2 && replay_force_ekf3) {
        ::printf("Cannot force both EKF types\n");
        exit(1);
    }

    if (filename == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        // allow replay on stm32
        filename = "APM/replayin.bin";
#else
        ::printf("You must supply a log filename\n");
        exit(1);
#endif
    }
    // LogReader reader = LogReader(log_structure);
    if (!reader.open_log(filename)) {
        ::printf("open(%s): %m\n", filename);
        exit(1);
    }
}

void Replay::loop()
{
    if (!reader.update()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // If we don't tear down the threads then they continue to access
    // global state during object destruction.
        ((Linux::Scheduler*)hal.scheduler)->teardown();
#endif
        exit(0);
    }
}

/*
  setup user -p parameters
 */
void Replay::set_user_parameters(void)
{
    for (struct user_parameter *u=user_parameters; u; u=u->next) {
        if (!reader.set_parameter(u->name, u->value, true)) {
            ::printf("Failed to set parameter %s to %f\n", u->name, u->value);
            exit(1);
        }
    }
}

/*
  parse a parameter file line
 */
bool Replay::parse_param_line(char *line, char **vname, float &value)
{
    if (line[0] == '#') {
        return false;
    }
    char *saveptr = NULL;
    char *pname = strtok_r(line, ", =\t", &saveptr);
    if (pname == NULL) {
        return false;
    }
    if (strlen(pname) > AP_MAX_NAME_SIZE) {
        return false;
    }
    const char *value_s = strtok_r(NULL, ", =\t", &saveptr);
    if (value_s == NULL) {
        return false;
    }
    value = atof(value_s);
    *vname = pname;
    return true;
}


/*
  load a default set of parameters from a file
 */
void Replay::load_param_file(const char *pfilename)
{
    FILE *f = fopen(pfilename, "r");
    if (f == NULL) {
        printf("Failed to open parameter file: %s\n", pfilename);
        exit(1);
    }
    char line[100];

    while (fgets(line, sizeof(line)-1, f)) {
        char *pname;
        float value;
        if (!parse_param_line(line, &pname, value)) {
            continue;
        }
        struct user_parameter *u = new user_parameter;
        strncpy_noterm(u->name, pname, sizeof(u->name));
        u->value = value;
        u->next = user_parameters;
        user_parameters = u;
    }
    fclose(f);
}

Replay replay(replayvehicle);
AP_Vehicle& vehicle = replayvehicle;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_HAL_MAIN_CALLBACKS(&replay);
