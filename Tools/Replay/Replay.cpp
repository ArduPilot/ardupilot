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

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include "Parameters.h"
#include "VehicleType.h"
#include "MsgHandler.h"

#ifndef INT16_MIN
#define INT16_MIN -32768
#define INT16_MAX 32767
#endif

#include "LogReader.h"
#include "DataFlashFileReader.h"
#include "Replay.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

#define streq(x, y) (!strcmp(x, y))

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

ReplayVehicle replayvehicle;

struct globals globals;

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
    GOBJECT(airspeed,                               "ARSPD_",   AP_Airspeed),

    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(EKF2, NavEKF2, "EK2_", NavEKF2),
    
    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass, "COMPASS_", Compass),

    // @Group: LOG
    // @Path: ../libraries/DataFlash/DataFlash.cpp
    GOBJECT(dataflash, "LOG", DataFlash_Class),
    
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(EKF3, NavEKF3, "EK3_", NavEKF3),

    AP_VAREND
};


void ReplayVehicle::load_parameters(void)
{
    unlink("Replay.stg");
    if (!AP_Param::check_var_info()) {
        AP_HAL::panic("Bad parameter table");
    }
    AP_Param::set_default_by_name("EK2_ENABLE", 1);
    AP_Param::set_default_by_name("EK2_IMU_MASK", 1);
    AP_Param::set_default_by_name("EK3_ENABLE", 1);
    AP_Param::set_default_by_name("EK3_IMU_MASK", 1);
    AP_Param::set_default_by_name("LOG_REPLAY", 1);
    AP_Param::set_default_by_name("AHRS_EKF_TYPE", 2);
    AP_Param::set_default_by_name("LOG_FILE_BUFSIZE", 60);
}

static const struct LogStructure min_log_structure[] = {
    { LOG_FORMAT_MSG, sizeof(log_Format),
      "FMT", "BBnNZ",      "Type,Length,Name,Format,Columns" },
    { LOG_PARAMETER_MSG, sizeof(log_Parameter),
      "PARM", "QNf",        "TimeUS,Name,Value" }, 
    { LOG_MESSAGE_MSG, sizeof(log_Message),
      "MSG",  "QZ",     "TimeUS,Message"},
};

void ReplayVehicle::setup(void) 
{
    load_parameters();
    
    // we pass a minimal log structure, as we will be outputting the
    // log structures we need manually, to prevent FMT duplicates
    dataflash.Init(min_log_structure, ARRAY_SIZE(min_log_structure));
    dataflash.StartUnstartedLogging();

    ahrs.set_compass(&compass);
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);
    ahrs.set_correct_centrifugal(true);
    ahrs.set_ekf_use(true);

    EKF2.set_enable(true);
    EKF3.set_enable(true);

    printf("Starting disarmed\n");
    hal.util->set_soft_armed(false);

    barometer.init();
    barometer.setHIL(0);
    barometer.update();
    compass.init();
    ins.set_hil_mode();
}

Replay replay(replayvehicle);

void Replay::usage(void)
{
    ::printf("Options:\n");
    ::printf("\t--parm NAME=VALUE  set parameter NAME to VALUE\n");
    ::printf("\t--accel-mask MASK  set accel mask (1=accel1 only, 2=accel2 only, 3=both)\n");
    ::printf("\t--gyro-mask MASK   set gyro mask (1=gyro1 only, 2=gyro2 only, 3=both)\n");
    ::printf("\t--arm-time time    arm at time (milliseconds)\n");
    ::printf("\t--no-imt           don't use IMT data\n");
    ::printf("\t--check-generate   generate CHEK messages in output\n");
    ::printf("\t--check            check solution against CHEK messages\n");
    ::printf("\t--tolerance-euler  tolerance for euler angles in degrees\n");
    ::printf("\t--tolerance-pos    tolerance for position in meters\n");
    ::printf("\t--tolerance-vel    tolerance for velocity in meters/second\n");
    ::printf("\t--nottypes         list of msg types not to output, comma separated\n");
    ::printf("\t--downsample       downsampling rate for output\n");
    ::printf("\t--logmatch         match logging rate to source\n");
    ::printf("\t--no-params        don't use parameters from the log\n");
    ::printf("\t--no-fpe           do not generate floating point exceptions\n");
}


enum {
    OPT_CHECK = 128,
    OPT_CHECK_GENERATE,
    OPT_TOLERANCE_EULER,
    OPT_TOLERANCE_POS,
    OPT_TOLERANCE_VEL,
    OPT_NOTTYPES,
    OPT_DOWNSAMPLE,
    OPT_LOGMATCH,
    OPT_NOPARAMS,
    OPT_PARAM_FILE,
    OPT_NO_FPE,
};

void Replay::flush_dataflash(void) {
    _vehicle.dataflash.flush();
}

/*
  create a list from a comma separated string
 */
const char **Replay::parse_list_from_string(const char *str_in)
{
    uint16_t comma_count=0;
    const char *p;
    for (p=str_in; *p; p++) {
        if (*p == ',') comma_count++;
    }

    char *str = strdup(str_in);
    if (str == NULL) {
        return NULL;
    }
    const char **ret = (const char **)calloc(comma_count+2, sizeof(char *));
    if (ret == NULL) {
        free(str);
        return NULL;
    }
    char *saveptr = NULL;
    uint16_t idx = 0;
    for (p=strtok_r(str, ",", &saveptr); p; p=strtok_r(NULL, ",", &saveptr)) {
        ret[idx++] = p;
    }
    return ret;
}

void Replay::_parse_command_line(uint8_t argc, char * const argv[])
{
    const struct GetOptLong::option options[] = {
        // name           has_arg flag   val
        {"parm",            true,   0, 'p'},
        {"param",           true,   0, 'p'},
        {"param-file",      true,   0, OPT_PARAM_FILE},
        {"help",            false,  0, 'h'},
        {"accel-mask",      true,   0, 'a'},
        {"gyro-mask",       true,   0, 'g'},
        {"arm-time",        true,   0, 'A'},
        {"no-imt",          false,  0, 'n'},
        {"check-generate",  false,  0, OPT_CHECK_GENERATE},
        {"check",           false,  0, OPT_CHECK},
        {"tolerance-euler", true,   0, OPT_TOLERANCE_EULER},
        {"tolerance-pos",   true,   0, OPT_TOLERANCE_POS},
        {"tolerance-vel",   true,   0, OPT_TOLERANCE_VEL},
        {"nottypes",        true,   0, OPT_NOTTYPES},
        {"downsample",      true,   0, OPT_DOWNSAMPLE},
        {"logmatch",        false,  0, OPT_LOGMATCH},
        {"no-params",       false,  0, OPT_NOPARAMS},
        {"no-fpe",          false,  0, OPT_NO_FPE},
        {0, false, 0, 0}
    };

    GetOptLong gopt(argc, argv, "r:p:ha:g:A:", options);

    int opt;
    while ((opt = gopt.getoption()) != -1) {
		switch (opt) {
        case 'g':
            logreader.set_gyro_mask(strtol(gopt.optarg, NULL, 0));
            break;

        case 'a':
            logreader.set_accel_mask(strtol(gopt.optarg, NULL, 0));
            break;

        case 'A':
            arm_time_ms = strtol(gopt.optarg, NULL, 0);
            break;

        case 'n':
            use_imt = false;
            logreader.set_use_imt(use_imt);
            break;

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

        case OPT_CHECK_GENERATE:
            check_generate = true;
            break;

        case OPT_CHECK:
            check_solution = true;
            break;

        case OPT_TOLERANCE_EULER:
            tolerance_euler = atof(gopt.optarg);
            break;

        case OPT_TOLERANCE_POS:
            tolerance_pos = atof(gopt.optarg);
            break;

        case OPT_TOLERANCE_VEL:
            tolerance_vel = atof(gopt.optarg);
            break;

        case OPT_NOTTYPES:
            nottypes = parse_list_from_string(gopt.optarg);
            break;

        case OPT_DOWNSAMPLE:
            downsample = atoi(gopt.optarg);
            break;

        case OPT_LOGMATCH:
            logmatch = true;
            break;

        case OPT_NOPARAMS:
            globals.no_params = true;
            break;

        case OPT_PARAM_FILE:
            load_param_file(gopt.optarg);
            break;
            
        case OPT_NO_FPE:
            generate_fpe = false;
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

class IMUCounter : public DataFlashFileReader {
public:
    IMUCounter() {}
    bool handle_log_format_msg(const struct log_Format &f);
    bool handle_msg(const struct log_Format &f, uint8_t *msg);

    uint64_t last_clock_timestamp = 0;
    float last_parm_value = 0;
    char last_parm_name[17] {};
private:
    MsgHandler *handler = nullptr;
    MsgHandler *parm_handler = nullptr;
};

bool IMUCounter::handle_log_format_msg(const struct log_Format &f) {
    if (!strncmp(f.name,"IMU",4) ||
        !strncmp(f.name,"IMT",4)) {
        // an IMU or IMT message message format
        handler = new MsgHandler(f);
    }
    if (strncmp(f.name,"PARM",4) == 0) {
        // PARM message message format
        parm_handler = new MsgHandler(f);
    }

    return true;
};

bool IMUCounter::handle_msg(const struct log_Format &f, uint8_t *msg) {
    if (strncmp(f.name,"PARM",4) == 0) {
        // gather parameter values to check for SCHED_LOOP_RATE
        parm_handler->field_value(msg, "Name", last_parm_name, sizeof(last_parm_name));
        parm_handler->field_value(msg, "Value", last_parm_value);
        return true;
    }
    if (strncmp(f.name,"IMU",4) &&
        strncmp(f.name,"IMT",4)) {
        // not an IMU message
        return true;
    }

    if (handler->field_value(msg, "TimeUS", last_clock_timestamp)) {
    } else if (handler->field_value(msg, "TimeMS", last_clock_timestamp)) {
        last_clock_timestamp *= 1000;
    } else {
        ::printf("Unable to find timestamp in message");
    }
    return true;
}

/*
  find information about the log
 */
bool Replay::find_log_info(struct log_information &info) 
{
    IMUCounter reader;
    if (!reader.open_log(filename)) {
        perror(filename);
        exit(1);
    }
    char clock_source[5] = { };
    int samplecount = 0;
    uint64_t prev = 0;
    uint64_t smallest_delta = 0;
    uint64_t total_delta = 0;
    prev = 0;
    const uint16_t samples_required = 1000;
    while (samplecount < samples_required) {
        char type[5];
        if (!reader.update(type)) {
            break;
        }

        if (streq(type, "PARM") && streq(reader.last_parm_name, "SCHED_LOOP_RATE")) {
            // get rate directly from parameters
            info.update_rate = reader.last_parm_value;
        }
        if (strlen(clock_source) == 0) {
            // If you want to add a clock source, also add it to
            // handle_msg and handle_log_format_msg, above.  Note that
            // ordering is important here.  For example, when we log
            // IMT we may reduce the logging speed of IMU, so then
            // using IMU as your clock source will lead to incorrect
            // behaviour.
            if (streq(type, "IMT")) {
                strcpy(clock_source, "IMT");
            } else if (streq(type, "IMU")) {
                strcpy(clock_source, "IMU");
            } else {
                continue;
            }
            hal.console->printf("Using clock source %s\n", clock_source);
        }
        // IMT if available always overrides
        if (streq(type, "IMT") && strcmp(clock_source, "IMT") != 0) {
            strcpy(clock_source, "IMT");
            hal.console->printf("Changing clock source to %s\n", clock_source);
            samplecount = 0;
            prev = 0;
            smallest_delta = 0;
            total_delta = 0;
        }
        if (streq(type, clock_source)) {
            if (prev == 0) {
                prev = reader.last_clock_timestamp;
            } else {
                uint64_t delta = reader.last_clock_timestamp - prev;
                if (delta < 40000 && delta > 1000) {
                    if (smallest_delta == 0 || delta < smallest_delta) {
                        smallest_delta = delta;
                    }
                    samplecount++;
                    total_delta += delta;
                }
            }
            prev = reader.last_clock_timestamp;
        }

        if (streq(type, "IMU2")) {
            info.have_imu2 = true;
        }
        if (streq(type, "IMT")) {
            info.have_imt = true;
        }
        if (streq(type, "IMT2")) {
            info.have_imt2 = true;
        }
    }
    if (smallest_delta == 0) {
        ::printf("Unable to determine log rate - insufficient IMU/IMT messages? (need=%d got=%d)", samples_required, samplecount);
        return false;
    }

    float average_delta = total_delta / samplecount;
    float rate = 1.0e6f/average_delta;
    printf("average_delta=%.2f smallest_delta=%lu samplecount=%lu\n",
           average_delta, (unsigned long)smallest_delta, (unsigned long)samplecount);
    if (rate < 100) {
        info.update_rate = 50;
    } else {
        info.update_rate = 400;
    }
    return true;
}

// catch floating point exceptions
static void _replay_sig_fpe(int signum)
{
    fprintf(stderr, "ERROR: Floating point exception - flushing dataflash...\n");
    replay.flush_dataflash();
    fprintf(stderr, "ERROR: ... and aborting.\n");
    if (replay.check_solution) {
        FILE *f = fopen("replay_results.txt","a");
        fprintf(f, "%s\tFPE\tFPE\tFPE\tFPE\tFPE\n",
                replay.log_filename);
        fclose(f);
    }
    abort();
}

FILE *Replay::xfopen(const char *f, const char *mode)
{
    FILE *ret = fopen(f, mode);
    if (ret == nullptr) {
        ::fprintf(stderr, "Failed to open (%s): %m\n", f);
        abort();
    }
    return ret;
}

void Replay::setup()
{
    ::printf("Starting\n");

    uint8_t argc;
    char * const *argv;

    hal.util->commandline_arguments(argc, argv);

    _parse_command_line(argc, argv);

    if (!check_generate) {
        logreader.set_save_chek_messages(true);
    }

    set_signal_handlers();

    hal.console->printf("Processing log %s\n", filename);

    // remember filename for reporting
    log_filename = filename;

    if (!find_log_info(log_info)) {
        printf("Update to get log information\n");
        exit(1);
    }

    hal.console->printf("Using an update rate of %u Hz\n", log_info.update_rate);

    if (!logreader.open_log(filename)) {
        perror(filename);
        exit(1);
    }

    _vehicle.setup();

    inhibit_gyro_cal();

    if (log_info.update_rate == 400) {
        // assume copter for 400Hz
        _vehicle.ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);
        _vehicle.ahrs.set_fly_forward(false);
    } else if (log_info.update_rate == 50) {
        // assume copter for 400Hz
        _vehicle.ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
        _vehicle.ahrs.set_fly_forward(true);
    }
    
    set_ins_update_rate(log_info.update_rate);
}

void Replay::set_ins_update_rate(uint16_t _update_rate) {
    _vehicle.ins.init(_update_rate);
}

void Replay::inhibit_gyro_cal() {
    // swiped from LR_MsgHandler.cpp; until we see PARM messages, we
    // don't have a PARM handler available to set parameters.
    enum ap_var_type var_type;
    AP_Param *vp = AP_Param::find("INS_GYR_CAL", &var_type);
    if (vp == NULL) {
        ::fprintf(stderr, "No GYR_CAL parameter found\n");
        abort();
    }
    ((AP_Float *)vp)->set(AP_InertialSensor::GYRO_CAL_NEVER);

    // logreader.set_parameter("GYR_CAL", AP_InertialSensor::GYRO_CAL_NEVER);
}

/*
  setup user -p parameters
 */
void Replay::set_user_parameters(void)
{
    for (struct user_parameter *u=user_parameters; u; u=u->next) {
        if (!logreader.set_parameter(u->name, u->value)) {
            ::printf("Failed to set parameter %s to %f\n", u->name, u->value);
            exit(1);
        }
    }
}

void Replay::set_signal_handlers(void)
{
    struct sigaction sa;

    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if (generate_fpe) {
        // SITL_State::_parse_command_line sets up an FPE handler.  We
        // can do better:
        feenableexcept(FE_INVALID | FE_OVERFLOW);
        sa.sa_handler = _replay_sig_fpe;
    } else {
        // disable floating point exception generation:
        int exceptions = FE_OVERFLOW | FE_DIVBYZERO;
#ifndef __i386__
        // i386 with gcc doesn't work with FE_INVALID
        exceptions |= FE_INVALID;
#endif
        if (feclearexcept(exceptions)) {
            ::fprintf(stderr, "Failed to disable floating point exceptions: %s", strerror(errno));
        }
        sa.sa_handler = SIG_IGN;
    }

    if (sigaction(SIGFPE, &sa, nullptr) < 0) {
        ::fprintf(stderr, "Failed to set floating point exceptions' handler: %s", strerror(errno));
    }
}

/*
  write out EKF log messages
 */
void Replay::write_ekf_logs(void)
{
    if (!LogReader::in_list("EKF", nottypes)) {
        _vehicle.dataflash.Log_Write_EKF(_vehicle.ahrs,false);
    }
    if (!LogReader::in_list("AHRS2", nottypes)) {
        _vehicle.dataflash.Log_Write_AHRS2(_vehicle.ahrs);
    }
    if (!LogReader::in_list("POS", nottypes)) {
        _vehicle.dataflash.Log_Write_POS(_vehicle.ahrs);
    }
}

void Replay::read_sensors(const char *type)
{
    if (!done_parameters && !streq(type,"FMT") && !streq(type,"PARM")) {
        done_parameters = true;
        set_user_parameters();
    }

    if (done_parameters && streq(type, "PARM")) {
        set_user_parameters();
    }
    
    if (!done_home_init) {
        if (streq(type, "GPS") &&
            (_vehicle.gps.status() >= AP_GPS::GPS_OK_FIX_3D) && done_baro_init) {
            const Location &loc = _vehicle.gps.location();
            ::printf("GPS Lock at %.7f %.7f %.2fm time=%.1f seconds\n", 
                     loc.lat * 1.0e-7f, 
                     loc.lng * 1.0e-7f,
                     loc.alt * 0.01f,
                     AP_HAL::millis()*0.001f);
            _vehicle.ahrs.set_home(loc);
            _vehicle.compass.set_initial_location(loc.lat, loc.lng);
            done_home_init = true;
        }
    }

    if (streq(type,"GPS")) {
        _vehicle.gps.update();
        if (_vehicle.gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            _vehicle.ahrs.estimate_wind();
        }
    } else if (streq(type,"MAG")) {
        _vehicle.compass.read();
    } else if (streq(type,"ARSP")) {
        _vehicle.ahrs.set_airspeed(&_vehicle.airspeed);
    } else if (streq(type,"BARO")) {
        _vehicle.barometer.update();
        if (!done_baro_init) {
            done_baro_init = true;
            ::printf("Barometer initialised\n");
            _vehicle.barometer.update_calibration();
        }
    } 

    bool run_ahrs = false;
    if (log_info.have_imt2) {
        run_ahrs = streq(type, "IMT2");
        _vehicle.ahrs.force_ekf_start();
    } else if (log_info.have_imt) {
        run_ahrs = streq(type, "IMT");
        _vehicle.ahrs.force_ekf_start();
    } else if (log_info.have_imu2) {
        run_ahrs = streq(type, "IMU2");
    } else {
        run_ahrs = streq(type, "IMU");
    }

    /*
      always run AHRS on CHECK messages when checking the solution
     */
    if (check_solution) {
        run_ahrs = streq(type, "CHEK");
    }
    
    if (run_ahrs) {
        _vehicle.ahrs.update();
        if (_vehicle.ahrs.get_home().lat != 0) {
            _vehicle.inertial_nav.update(_vehicle.ins.get_delta_time());
        }
        if ((downsample == 0 || ++output_counter % downsample == 0) && !logmatch) {
            write_ekf_logs();
        }
        if (_vehicle.ahrs.healthy() != ahrs_healthy) {
            ahrs_healthy = _vehicle.ahrs.healthy();
            printf("AHRS health: %u at %lu\n", 
                   (unsigned)ahrs_healthy,
                   (unsigned long)AP_HAL::millis());
        }
        if (check_generate) {
            log_check_generate();
        } else if (check_solution) {
            log_check_solution();
        }
    }
    
    if (logmatch && (streq(type, "NKF1") || streq(type, "XKF1"))) {
        write_ekf_logs();
    }
}


/*
  copy current data to CHEK message
 */
void Replay::log_check_generate(void)
{
    Vector3f euler;
    Vector3f velocity;
    Location loc {};

    _vehicle.EKF2.getEulerAngles(-1,euler);
    _vehicle.EKF2.getVelNED(-1,velocity);
    _vehicle.EKF2.getLLH(loc);

    struct log_Chek packet = {
        LOG_PACKET_HEADER_INIT(LOG_CHEK_MSG),
        time_us : AP_HAL::micros64(),
        roll    : (int16_t)(100*degrees(euler.x)), // roll angle (centi-deg, displayed as deg due to format string)
        pitch   : (int16_t)(100*degrees(euler.y)), // pitch angle (centi-deg, displayed as deg due to format string)
        yaw     : (uint16_t)wrap_360_cd(100*degrees(euler.z)), // yaw angle (centi-deg, displayed as deg due to format string)
        lat     : loc.lat,
        lng     : loc.lng,
        alt     : loc.alt*0.01f,
        vnorth  : velocity.x,
        veast   : velocity.y,
        vdown   : velocity.z
    };

    _vehicle.dataflash.WriteBlock(&packet, sizeof(packet));
}


/*
  check current solution against CHEK message
 */
void Replay::log_check_solution(void)
{
    const LR_MsgHandler::CheckState &check_state = logreader.get_check_state();
    Vector3f euler;
    Vector3f velocity;
    Location loc {};

    _vehicle.EKF2.getEulerAngles(-1,euler);
    _vehicle.EKF2.getVelNED(-1,velocity);
    _vehicle.EKF2.getLLH(loc);

    float roll_error  = degrees(fabsf(euler.x - check_state.euler.x));
    float pitch_error = degrees(fabsf(euler.y - check_state.euler.y));
    float yaw_error = wrap_180_cd(100*degrees(fabsf(euler.z - check_state.euler.z)))*0.01f;
    float vel_error = (velocity - check_state.velocity).length();
    float pos_error = get_distance(check_state.pos, loc);

    check_result.max_roll_error  = MAX(check_result.max_roll_error,  roll_error);
    check_result.max_pitch_error = MAX(check_result.max_pitch_error, pitch_error);
    check_result.max_yaw_error   = MAX(check_result.max_yaw_error,   yaw_error);
    check_result.max_vel_error   = MAX(check_result.max_vel_error,   vel_error);
    check_result.max_pos_error   = MAX(check_result.max_pos_error,   pos_error);
}

void Replay::flush_and_exit()
{
    flush_dataflash();

    if (check_solution) {
        report_checks();
    }

    exit(0);
}

void Replay::loop()
{
    char type[5];

    if (arm_time_ms >= 0 && AP_HAL::millis() > (uint32_t)arm_time_ms) {
        if (!hal.util->get_soft_armed()) {
            hal.util->set_soft_armed(true);
            ::printf("Arming at %u ms\n", (unsigned)AP_HAL::millis());
        }
    }

    if (!logreader.update(type)) {
        ::printf("End of log at %.1f seconds\n", AP_HAL::millis()*0.001f);
        flush_and_exit();
    }

    if (last_timestamp != 0) {
        uint64_t gap = AP_HAL::micros64() - last_timestamp;
        if (gap > 40000) {
            ::printf("Gap in log at timestamp=%lu of length %luus\n",
                     last_timestamp, gap);
        }
    }
    last_timestamp = AP_HAL::micros64();

    read_sensors(type);

    if (!streq(type,"ATT")) {
        return;
    }

}


bool Replay::show_error(const char *text, float max_error, float tolerance)
{
    bool failed = max_error > tolerance;
    printf("%s:\t%.2f %c %.2f\n", 
           text,
           max_error,
           failed?'>':'<',
           tolerance);
    return failed;
}

/*
  report results of --check
 */
void Replay::report_checks(void)
{
    bool failed = false;
    if (tolerance_euler < 0.01f) {
        tolerance_euler = 0.01f;
    }
    FILE *f = fopen("replay_results.txt","a");
    if (f != NULL) {
        fprintf(f, "%s\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
                log_filename, 
                check_result.max_roll_error,
                check_result.max_pitch_error,
                check_result.max_yaw_error,
                check_result.max_pos_error,
                check_result.max_vel_error);
        fclose(f);
    }
    failed |= show_error("Roll error", check_result.max_roll_error, tolerance_euler);
    failed |= show_error("Pitch error", check_result.max_pitch_error, tolerance_euler);
    failed |= show_error("Yaw error", check_result.max_yaw_error, tolerance_euler);
    failed |= show_error("Position error", check_result.max_pos_error, tolerance_pos);
    failed |= show_error("Velocity error", check_result.max_vel_error, tolerance_vel);
    if (failed) {
        printf("Checks failed\n");
        exit(1);
    } else {
        printf("Checks passed\n");
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
        strncpy(u->name, pname, sizeof(u->name));
        u->value = value;
        u->next = user_parameters;
        user_parameters = u;
    }
    fclose(f);
}

/*
  see if a user parameter is set
 */
bool Replay::check_user_param(const char *name)
{
    for (struct user_parameter *u=user_parameters; u; u=u->next) {
        if (strcmp(name, u->name) == 0) {
            return true;
        }
    }
    return false;
}

const struct AP_Param::GroupInfo        GCS_MAVLINK::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN_CALLBACKS(&replay);
