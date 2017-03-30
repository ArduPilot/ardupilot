#include "Plane.h"
#include "version.h"

#if LOGGING_ENABLED == ENABLED

#if CLI_ENABLED == ENABLED
// Code to Write and Read packets from DataFlash.log memory
// Code to interact with the user to dump or erase logs

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] = {
    {"dump",        MENU_FUNC(dump_log)},
    {"erase",       MENU_FUNC(erase_logs)},
    {"enable",      MENU_FUNC(select_logs)},
    {"disable",     MENU_FUNC(select_logs)}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, FUNCTOR_BIND(&plane, &Plane::print_log_menu, bool));

bool Plane::print_log_menu(void)
{
    cliSerial->printf("logs enabled: \n");

    if (0 == g.log_bitmask) {
        cliSerial->printf("none\n");
    }else{
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
 #define PLOG(_s) if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf(" %s", # _s)
        PLOG(ATTITUDE_FAST);
        PLOG(ATTITUDE_MED);
        PLOG(GPS);
        PLOG(PM);
        PLOG(CTUN);
        PLOG(NTUN);
        PLOG(MODE);
        PLOG(IMU);
        PLOG(CMD);
        PLOG(CURRENT);
        PLOG(COMPASS);
        PLOG(TECS);
        PLOG(CAMERA);
        PLOG(RC);
        PLOG(SONAR);
 #undef PLOG
    }

    cliSerial->printf("\n");

    DataFlash.ListAvailableLogs(cliSerial);
    return(true);
}

int8_t Plane::dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log_num;
    uint16_t dump_log_start;
    uint16_t dump_log_end;

    // check that the requested log number can be read
    dump_log_num = argv[1].i;

    if (dump_log_num == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log_num <= 0) {
        cliSerial->printf("dumping all\n");
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || ((uint16_t)dump_log_num > DataFlash.get_num_logs())) {
        cliSerial->printf("bad log number\n");
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log_num, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log_num, dump_log_start, dump_log_end);
    return 0;
}

int8_t Plane::erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

int8_t Plane::select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint32_t bits;

    if (argc != 2) {
        cliSerial->printf("missing log type\n");
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp(argv[1].str, "all")) {
        bits = 0xFFFFFFFFUL;
    } else {
 #define TARG(_s)        if (!strcasecmp(argv[1].str, # _s)) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(MODE);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(COMPASS);
        TARG(TECS);
        TARG(CAMERA);
        TARG(RC);
        TARG(SONAR);
 #undef TARG
    }

    if (!strcasecmp(argv[0].str, "enable")) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }
    return(0);
}

int8_t Plane::process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}

#endif // CLI_ENABLED == ENABLED

void Plane::do_erase_logs(void)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Erasing logs");
    DataFlash.EraseAll();
    gcs_send_text(MAV_SEVERITY_INFO, "Log erase complete");
}


// Write an attitude packet
void Plane::Log_Write_Attitude(void)
{
    Vector3f targets;       // Package up the targets into a vector for commonality with Copter usage of Log_Wrote_Attitude
    targets.x = nav_roll_cd;
    targets.y = nav_pitch_cd;
    targets.z = 0;          //Plane does not have the concept of navyaw. This is a placeholder.

    if (quadplane.tailsitter_active()) {
        DataFlash.Log_Write_AttitudeView(*quadplane.ahrs_view, targets);
    } else {
        DataFlash.Log_Write_Attitude(ahrs, targets);
    }
    if (quadplane.in_vtol_mode() || quadplane.in_assisted_flight()) {
        // log quadplane PIDs separately from fixed wing PIDs
        DataFlash.Log_Write_PID(LOG_PIQR_MSG, quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIQP_MSG, quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIQY_MSG, quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIQA_MSG, quadplane.pid_accel_z.get_pid_info() );
    }

    DataFlash.Log_Write_PID(LOG_PIDR_MSG, rollController.get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDP_MSG, pitchController.get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDY_MSG, yawController.get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDS_MSG, steerController.get_pid_info());
    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        const DataFlash_Class::PID_Info *landing_info;
        landing_info = landing.get_pid_info();
        if (landing_info != nullptr) { // only log LANDING PID's while in landing
            DataFlash.Log_Write_PID(LOG_PIDL_MSG, *landing_info);
        }
    }

#if AP_AHRS_NAVEKF_AVAILABLE
 #if OPTFLOW == ENABLED
    DataFlash.Log_Write_EKF(ahrs,optflow.enabled());
 #else
    DataFlash.Log_Write_EKF(ahrs,false);
 #endif
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE(&DataFlash);
#endif
    DataFlash.Log_Write_POS(ahrs);
}

// do logging at loop rate
void Plane::Log_Write_Fast(void)
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
}


struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t num_long;
    uint16_t main_loop_count;
    uint32_t g_dt_max;
    uint32_t g_dt_min;
    uint32_t log_dropped;
    uint32_t mem_avail;
};

// Write a performance monitoring packet. Total length : 19 bytes
void Plane::Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_us         : AP_HAL::micros64(),
        num_long        : perf.num_long,
        main_loop_count : perf.mainLoop_count,
        g_dt_max        : perf.G_Dt_max,
        g_dt_min        : perf.G_Dt_min,
        log_dropped     : DataFlash.num_dropped() - perf.last_log_dropped,
        hal.util->available_memory()
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t startup_type;
    uint16_t command_total;
};

void Plane::Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        time_us         : AP_HAL::micros64(),
        startup_type    : type,
        command_total   : mission.num_commands()
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    int16_t throttle_dem;
};

// Write a control tuning packet. Total length : 22 bytes
void Plane::Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_us         : AP_HAL::micros64(),
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle),
        rudder_out      : (int16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_rudder),
        throttle_dem    : (int16_t)SpdHgt_Controller->get_throttle_demand()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float wp_distance;
    int16_t target_bearing_cd;
    int16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    float   xtrack_error;
    float   xtrack_error_i;
    float   airspeed_error;
};

// Write a navigation tuning packet
void Plane::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us             : AP_HAL::micros64(),
        wp_distance         : auto_state.wp_distance,
        target_bearing_cd   : (int16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (int16_t)nav_controller->nav_bearing_cd(),
        altitude_error_cm   : (int16_t)altitude_error_cm,
        xtrack_error        : nav_controller->crosstrack_error(),
        xtrack_error_i      : nav_controller->crosstrack_error_integrator(),
        airspeed_error      : airspeed_error
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Status {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t is_flying;
    float is_flying_probability;
    uint8_t armed;
    uint8_t safety;
    bool is_crashed;
    bool is_still;
    uint8_t stage;
    bool impact;
};

void Plane::Log_Write_Status()
{
    struct log_Status pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STATUS_MSG)
        ,time_us   : AP_HAL::micros64()
        ,is_flying   : is_flying()
        ,is_flying_probability : isFlyingProbability
        ,armed       : hal.util->get_soft_armed()
        ,safety      : static_cast<uint8_t>(hal.util->safety_switch_state())
        ,is_crashed  : crash_state.is_crashed
        ,is_still    : plane.ins.is_still()
        ,stage       : static_cast<uint8_t>(flight_stage)
        ,impact      : crash_state.impact_detected
        };

    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Sonar {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float distance;
    float voltage;
    uint8_t count;
    float correction;
};

// Write a sonar packet
void Plane::Log_Write_Sonar()
{
    uint16_t distance = 0;
    if (rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::RangeFinder_Good) {
        distance = rangefinder.distance_cm_orient(ROTATION_PITCH_270);
    }

    struct log_Sonar pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SONAR_MSG),
        time_us     : AP_HAL::micros64(),
        distance    : (float)distance*0.01f,
        voltage     : rangefinder.voltage_mv_orient(ROTATION_PITCH_270)*0.001f,
        count       : rangefinder_state.in_range_count,
        correction  : rangefinder_state.correction
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    DataFlash.Log_Write_RFND(rangefinder);
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t surface_quality;
    float flow_x;
    float flow_y;
    float body_x;
    float body_y;
};

#if OPTFLOW == ENABLED
// Write an optical flow packet
void Plane::Log_Write_Optflow()
{
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }
    const Vector2f &flowRate = optflow.flowRate();
    const Vector2f &bodyRate = optflow.bodyRate();
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_us         : AP_HAL::micros64(),
        surface_quality : optflow.quality(),
        flow_x           : flowRate.x,
        flow_y           : flowRate.y,
        body_x           : bodyRate.x,
        body_y           : bodyRate.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  arm_state;
    uint16_t arm_checks;
};

void Plane::Log_Write_Current()
{
    DataFlash.Log_Write_Current(battery);

    // also write power status
    DataFlash.Log_Write_Power();
}

void Plane::Log_Arm_Disarm() {
    struct log_Arm_Disarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_us                 : AP_HAL::micros64(),
        arm_state               : arming.is_armed(),
        arm_checks              : arming.get_enabled_checks()      
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

void Plane::Log_Write_GPS(uint8_t instance)
{
    if (!ahrs.have_ekf_logging()) {
        DataFlash.Log_Write_GPS(gps, instance);
    }
}

void Plane::Log_Write_IMU() 
{
    DataFlash.Log_Write_IMU(ins);
}

void Plane::Log_Write_RC(void)
{
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
    if (rssi.enabled()) {
        DataFlash.Log_Write_RSSI(rssi);
    }
}

void Plane::Log_Write_Baro(void)
{
    if (!ahrs.have_ekf_logging()) {
        DataFlash.Log_Write_Baro(barometer);
    }
}

// Write a AIRSPEED packet
void Plane::Log_Write_Airspeed(void)
{
    DataFlash.Log_Write_Airspeed(airspeed);
}

// log ahrs home and EKF origin to dataflash
void Plane::Log_Write_Home_And_Origin()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    // log ekf origin if set
    Location ekf_orig;
    if (ahrs.get_origin(ekf_orig)) {
        DataFlash.Log_Write_Origin(LogOriginType::ekf_origin, ekf_orig);
    }
#endif

    // log ahrs home if set
    if (home_is_set != HOME_UNSET) {
        DataFlash.Log_Write_Origin(LogOriginType::ahrs_home, ahrs.get_home());
    }
}

const struct LogStructure Plane::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "QHHIIII",  "TimeUS,NLon,NLoop,MaxT,MinT,LogDrop,Mem" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "QBH",         "TimeUS,SType,CTot" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Qcccchhh",    "TimeUS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,ThrDem" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "Qfcccfff",  "TimeUS,WpDist,TargBrg,NavBrg,AltErr,XT,XTi,ArspdErr" },
    { LOG_SONAR_MSG, sizeof(log_Sonar),             
      "SONR", "QffBf",   "TimeUS,Dist,Volt,Cnt,Corr" },
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm),
      "ARM", "QBH", "TimeUS,ArmState,ArmChecks" },
    { LOG_ATRP_MSG, sizeof(AP_AutoTune::log_ATRP),
      "ATRP", "QBBcfff",  "TimeUS,Type,State,Servo,Demanded,Achieved,P" },
    { LOG_STATUS_MSG, sizeof(log_Status),
      "STAT", "QBfBBBBBB",  "TimeUS,isFlying,isFlyProb,Armed,Safety,Crash,Still,Stage,Hit" },
    { LOG_QTUN_MSG, sizeof(QuadPlane::log_QControl_Tuning),
      "QTUN", "Qffffehhffff", "TimeUS,AngBst,ThrOut,DAlt,Alt,BarAlt,DCRt,CRt,DVx,DVy,DAx,DAy" },
#if OPTFLOW == ENABLED
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),
      "OF",   "QBffff",   "TimeUS,Qual,flowX,flowY,bodyX,bodyY" },
#endif
    { LOG_PIQR_MSG, sizeof(log_PID), \
      "PIQR", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIQP_MSG, sizeof(log_PID), \
      "PIQP", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIQY_MSG, sizeof(log_PID), \
      "PIQY", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
    { LOG_PIQA_MSG, sizeof(log_PID), \
      "PIQA", "Qffffff",  "TimeUS,Des,P,I,D,FF,AFF" }, \
};

#if CLI_ENABLED == ENABLED
// Read the DataFlash.log memory : Packet Parser
void Plane::Log_Read(uint16_t list_entry, int16_t start_page, int16_t end_page)
{
    cliSerial->printf("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n",
                        (unsigned)hal.util->available_memory());

    cliSerial->printf("%s\n", HAL_BOARD_NAME);

	DataFlash.LogReadProcess(list_entry, start_page, end_page,
                             FUNCTOR_BIND_MEMBER(&Plane::print_flight_mode, void, AP_HAL::BetterStream *, uint8_t),
                             cliSerial);
}
#endif // CLI_ENABLED

void Plane::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    DataFlash.Log_Write_Mode(control_mode);
    DataFlash.Log_Write_Rally(rally);
    Log_Write_Home_And_Origin();
}

// start a new log
void Plane::start_logging() 
{
    DataFlash.set_mission(&mission);
    DataFlash.setVehicle_Startup_Log_Writer(
        FUNCTOR_BIND(&plane, &Plane::Log_Write_Vehicle_Startup_Messages, void)
        );

    DataFlash.StartNewLog();
}

/*
  initialise logging subsystem
 */
void Plane::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));
    if (!DataFlash.CardInserted()) {
        gcs_send_text(MAV_SEVERITY_WARNING, "No dataflash card inserted");
    } else if (DataFlash.NeedPrep()) {
        gcs_send_text(MAV_SEVERITY_INFO, "Preparing log system");
        DataFlash.Prep();
        gcs_send_text(MAV_SEVERITY_INFO, "Prepared log system");
        gcs().reset_cli_timeout();
    }
}

#else // LOGGING_ENABLED

 #if CLI_ENABLED == ENABLED
bool Plane::print_log_menu(void) { return true; }
int8_t Plane::dump_log(uint8_t argc, const Menu::arg *argv) { return 0; }
int8_t Plane::erase_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
int8_t Plane::select_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
int8_t Plane::process_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
 #endif // CLI_ENABLED == ENABLED

void Plane::do_erase_logs(void) {}
void Plane::Log_Write_Attitude(void) {}
void Plane::Log_Write_Fast(void) {}
void Plane::Log_Write_Performance() {}
void Plane::Log_Write_Startup(uint8_t type) {}
void Plane::Log_Write_Control_Tuning() {}
void Plane::Log_Write_Nav_Tuning() {}
void Plane::Log_Write_Status() {}
void Plane::Log_Write_Sonar() {}

 #if OPTFLOW == ENABLED
void Plane::Log_Write_Optflow() {}
 #endif

void Plane::Log_Write_Current() {}
void Plane::Log_Arm_Disarm() {}
void Plane::Log_Write_GPS(uint8_t instance) {}
void Plane::Log_Write_IMU() {}
void Plane::Log_Write_RC(void) {}
void Plane::Log_Write_Baro(void) {}
void Plane::Log_Write_Airspeed(void) {}
void Plane::Log_Write_Home_And_Origin() {}

 #if CLI_ENABLED == ENABLED
void Plane::Log_Read(uint16_t log_num, int16_t start_page, int16_t end_page) {}
 #endif // CLI_ENABLED

void Plane::start_logging() {}
void Plane::log_init(void) {}

#endif // LOGGING_ENABLED
