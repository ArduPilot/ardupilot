#include "Rover.h"
#include "version.h"

#if LOGGING_ENABLED == ENABLED

#if CLI_ENABLED == ENABLED

// Code to interact with the user to dump or erase logs

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] = {
    {"dump",    MENU_FUNC(dump_log)},
    {"erase",   MENU_FUNC(erase_logs)},
    {"enable",  MENU_FUNC(select_logs)},
    {"disable", MENU_FUNC(select_logs)}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, FUNCTOR_BIND(&rover, &Rover::print_log_menu, bool));

bool Rover::print_log_menu(void)
{
    cliSerial->printf("logs enabled: ");

    if (0 == g.log_bitmask) {
        cliSerial->printf("none");
    } else {
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
        #define PLOG(_s)    if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf(" %s", #_s)
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
        PLOG(RANGEFINDER);
        PLOG(COMPASS);
        PLOG(CAMERA);
        PLOG(STEERING);
        #undef PLOG
    }

    cliSerial->printf("\n");

    DataFlash.ListAvailableLogs(cliSerial);
    return(true);
}

int8_t Rover::dump_log(uint8_t argc, const Menu::arg *argv)
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
    } else if ((argc != 2) || (static_cast<uint16_t>(dump_log_num) > DataFlash.get_num_logs())) {
        cliSerial->printf("bad log number\n");
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log_num, dump_log_start, dump_log_end);
    Log_Read(static_cast<uint16_t>(dump_log_num), dump_log_start, dump_log_end);
    return 0;
}


int8_t Rover::erase_logs(uint8_t argc, const Menu::arg *argv)
{
    DataFlash.EnableWrites(false);
    do_erase_logs();
    DataFlash.EnableWrites(true);
    return 0;
}

int8_t Rover::select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint16_t bits = 0;

    if (argc != 2) {
        cliSerial->printf("missing log type\n");
        return(-1);
    }

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp(argv[1].str, "all")) {
        bits = ~0;
    } else {
        #define TARG(_s)    if (!strcasecmp(argv[1].str, #_s)) bits |= MASK_LOG_ ## _s
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
        TARG(RANGEFINDER);
        TARG(COMPASS);
        TARG(CAMERA);
        TARG(STEERING);
        #undef TARG
    }

    if (!strcasecmp(argv[0].str, "enable")) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    } else {
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }
    return(0);
}

int8_t Rover::process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}

#endif  // CLI_ENABLED == ENABLED

void Rover::do_erase_logs(void)
{
    cliSerial->printf("\nErasing log...\n");
    DataFlash.EraseAll();
    cliSerial->printf("\nLog erased.\n");
}


struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t loop_time;
    uint16_t main_loop_count;
    uint32_t g_dt_max;
    int16_t  gyro_drift_x;
    int16_t  gyro_drift_y;
    int16_t  gyro_drift_z;
    uint8_t  i2c_lockup_count;
    uint16_t ins_error_count;
    uint32_t mem_avail;
};

// Write a performance monitoring packet. Total length : 19 bytes
void Rover::Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_us         : AP_HAL::micros64(),
        loop_time       : millis()- perf_mon_timer,
        main_loop_count : mainLoop_count,
        g_dt_max        : G_Dt_max,
        gyro_drift_x    : (int16_t)(ahrs.get_gyro_drift().x * 1000),
        gyro_drift_y    : (int16_t)(ahrs.get_gyro_drift().y * 1000),
        gyro_drift_z    : (int16_t)(ahrs.get_gyro_drift().z * 1000),
        i2c_lockup_count: 0,
        ins_error_count : ins.error_count(),
        hal.util->available_memory()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Steering {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float demanded_accel;
    float achieved_accel;
};

// Write a steering packet
void Rover::Log_Write_Steering()
{
    struct log_Steering pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STEERING_MSG),
        time_us        : AP_HAL::micros64(),
        demanded_accel : control_mode->lateral_acceleration,
        achieved_accel : ahrs.groundspeed() * ins.get_gyro().z,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write beacon position and distances
void Rover::Log_Write_Beacon()
{
    // exit immediately if feature is disabled
    if (!g2.beacon.enabled()) {
        return;
    }

    DataFlash.Log_Write_Beacon(g2.beacon);
}
struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t startup_type;
    uint16_t command_total;
};

void Rover::Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        time_us         : AP_HAL::micros64(),
        startup_type    : type,
        command_total   : mission.num_commands()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t steer_out;
    int16_t roll;
    int16_t pitch;
    int16_t throttle_out;
    float accel_y;
};

// Write a control tuning packet. Total length : 22 bytes
void Rover::Log_Write_Control_Tuning()
{
    const Vector3f accel = ins.get_accel();
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_us         : AP_HAL::micros64(),
        steer_out       : (int16_t)g2.motors.get_steering(),
        roll            : (int16_t)ahrs.roll_sensor,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)g2.motors.get_throttle(),
        accel_y         : accel.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t yaw;
    float    wp_distance;
    uint16_t target_bearing_cd;
    uint16_t nav_bearing_cd;
    int8_t   throttle;
    float xtrack_error;
};

// Write a navigation tuning packet
void Rover::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us             : AP_HAL::micros64(),
        yaw                 : static_cast<uint16_t>(ahrs.yaw_sensor),
        wp_distance         : wp_distance,
        target_bearing_cd   : static_cast<uint16_t>(fabsf(nav_controller->target_bearing_cd())),
        nav_bearing_cd      : static_cast<uint16_t>(fabsf(nav_controller->nav_bearing_cd())),
        throttle            : int8_t(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)),
        xtrack_error        : nav_controller->crosstrack_error()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Rover::Log_Write_Attitude()
{
    const Vector3f targets(0.0f, 0.0f, 0.0f);  // Rover does not have attitude targets, use place-holder for commonality with Dataflash Log_Write_Attitude message

    DataFlash.Log_Write_Attitude(ahrs, targets);

#if AP_AHRS_NAVEKF_AVAILABLE
  #if defined(OPTFLOW) and (OPTFLOW == ENABLED)
    DataFlash.Log_Write_EKF(ahrs, optflow.enabled());
  #else
    DataFlash.Log_Write_EKF(ahrs, false);
  #endif
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
    DataFlash.Log_Write_POS(ahrs);

    DataFlash.Log_Write_PID(LOG_PIDS_MSG, steerController.get_pid_info());

    DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pidSpeedThrottle.get_pid_info());
}

struct PACKED log_Rangefinder {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    lateral_accel;
    uint16_t rangefinder1_distance;
    uint16_t rangefinder2_distance;
    uint16_t detected_count;
    int8_t   turn_angle;
    uint16_t turn_time;
    uint16_t ground_speed;
    int8_t   throttle;
};

// Write a rangefinder packet
void Rover::Log_Write_Rangefinder()
{
    uint16_t turn_time = 0;
    if (!is_zero(obstacle.turn_angle)) {
        turn_time = AP_HAL::millis() - obstacle.detected_time_ms;
    }
    struct log_Rangefinder pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RANGEFINDER_MSG),
        time_us               : AP_HAL::micros64(),
        lateral_accel         : control_mode->lateral_acceleration,
        rangefinder1_distance : rangefinder.distance_cm(0),
        rangefinder2_distance : rangefinder.distance_cm(1),
        detected_count        : obstacle.detected_count,
        turn_angle            : static_cast<int8_t>(obstacle.turn_angle),
        turn_time             : turn_time,
        ground_speed          : static_cast<uint16_t>(fabsf(ground_speed * 100.0f)),
        throttle              : int8_t(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle))
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_Current()
{
    DataFlash.Log_Write_Current(battery);

    // also write power status
    DataFlash.Log_Write_Power();
}

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  arm_state;
    uint16_t arm_checks;
};

void Rover::Log_Arm_Disarm() {
    struct log_Arm_Disarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_us                 : AP_HAL::micros64(),
        arm_state               : arming.is_armed(),
        arm_checks              : arming.get_enabled_checks()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_RC(void)
{
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
    if (rssi.enabled()) {
        DataFlash.Log_Write_RSSI(rssi);
    }
}

struct PACKED log_Error {
  LOG_PACKET_HEADER;
  uint64_t time_us;
  uint8_t sub_system;
  uint8_t error_code;
};

// Write an error packet
void Rover::Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
  struct log_Error pkt = {
      LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
      time_us       : AP_HAL::micros64(),
      sub_system    : sub_system,
      error_code    : error_code,
  };
  DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

void Rover::Log_Write_Baro(void)
{
    DataFlash.Log_Write_Baro(barometer);
}

// log ahrs home and EKF origin to dataflash
void Rover::Log_Write_Home_And_Origin()
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

// guided mode logging
struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
};

// Write a Guided mode target
void Rover::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// wheel encoder packet
struct PACKED log_WheelEncoder {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float distance_0;
    uint8_t quality_0;
    float distance_1;
    uint8_t quality_1;
};

// log wheel encoder information
void Rover::Log_Write_WheelEncoder()
{
    // return immediately if no wheel encoders are enabled
    if (!g2.wheel_encoder.enabled(0) && !g2.wheel_encoder.enabled(1)) {
        return;
    }
    struct log_WheelEncoder pkt = {
        LOG_PACKET_HEADER_INIT(LOG_WHEELENCODER_MSG),
        time_us     : AP_HAL::micros64(),
        distance_0  : g2.wheel_encoder.get_distance(0),
        quality_0   : (uint8_t)constrain_float(g2.wheel_encoder.get_signal_quality(0),0.0f,100.0f),
        distance_1  : g2.wheel_encoder.get_distance(1),
        quality_1   : (uint8_t)constrain_float(g2.wheel_encoder.get_signal_quality(1),0.0f,100.0f)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

const LogStructure Rover::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance),
      "PM",  "QIHIhhhBHI", "TimeUS,LTime,MLC,gDt,GDx,GDy,GDz,I2CErr,INSErr,Mem" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),
      "STRT", "QBH",        "TimeUS,SType,CTot" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qhcchf",     "TimeUS,Steer,Roll,Pitch,ThrOut,AccY" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),
      "NTUN", "QHfHHbf",    "TimeUS,Yaw,WpDist,TargBrg,NavBrg,Thr,XT" },
    { LOG_RANGEFINDER_MSG, sizeof(log_Rangefinder),
      "RGFD", "QfHHHbHCb",  "TimeUS,LatAcc,R1Dist,R2Dist,DCnt,TAng,TTim,Spd,Thr" },
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm),
      "ARM", "QBH", "TimeUS,ArmState,ArmChecks" },
    { LOG_STEERING_MSG, sizeof(log_Steering),
      "STER", "Qff",   "TimeUS,Demanded,Achieved" },
    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUID",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ" },
    { LOG_ERROR_MSG, sizeof(log_Error),
      "ERR",   "QBB",         "TimeUS,Subsys,ECode" },
    { LOG_WHEELENCODER_MSG, sizeof(log_WheelEncoder),
      "WENC",  "Qfbfb",       "TimeUS,Dist0,Qual0,Dist1,Qual1" },
};

void Rover::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));

    gcs().reset_cli_timeout();

}

#if CLI_ENABLED == ENABLED
// Read the DataFlash log memory : Packet Parser
void Rover::Log_Read(uint16_t list_entry, uint16_t start_page, uint16_t end_page)
{
    cliSerial->printf("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n",
            static_cast<uint32_t>(hal.util->available_memory()));

    cliSerial->printf("%s\n", HAL_BOARD_NAME);

    DataFlash.LogReadProcess(list_entry, start_page, end_page,
                             FUNCTOR_BIND_MEMBER(&Rover::print_mode, void, AP_HAL::BetterStream *, uint8_t),
                             cliSerial);
}
#endif  // CLI_ENABLED

void Rover::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    DataFlash.Log_Write_Mode(control_mode->mode_number());
    Log_Write_Home_And_Origin();
    gps.Write_DataFlash_Log_Startup_messages();
}

#else  // LOGGING_ENABLED

// dummy functions
void Rover::Log_Write_Startup(uint8_t type) {}
void Rover::Log_Write_Current() {}
void Rover::Log_Write_Nav_Tuning() {}
void Rover::Log_Write_Performance() {}
int8_t Rover::process_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
void Rover::Log_Write_Control_Tuning() {}
void Rover::Log_Write_Rangefinder() {}
void Rover::Log_Write_Attitude() {}
void Rover::Log_Write_RC(void) {}
void Rover::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target) {}
void Rover::Log_Write_Home_And_Origin() {}
void Rover::Log_Write_Baro(void) {}
void Rover::Log_Arm_Disarm() {}
void Rover::Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
void Rover::Log_Write_Steering() {}
void Rover::Log_Write_WheelEncoder() {}

#endif  // LOGGING_ENABLED
