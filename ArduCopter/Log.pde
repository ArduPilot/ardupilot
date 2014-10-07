// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#if CLI_ENABLED == ENABLED
// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static bool     print_log_menu(void);
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
    cliSerial->printf_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->printf_P(PSTR("none"));
    }else{
        if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST) cliSerial->printf_P(PSTR(" ATTITUDE_FAST"));
        if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) cliSerial->printf_P(PSTR(" ATTITUDE_MED"));
        if (g.log_bitmask & MASK_LOG_GPS) cliSerial->printf_P(PSTR(" GPS"));
        if (g.log_bitmask & MASK_LOG_PM) cliSerial->printf_P(PSTR(" PM"));
        if (g.log_bitmask & MASK_LOG_CTUN) cliSerial->printf_P(PSTR(" CTUN"));
        if (g.log_bitmask & MASK_LOG_NTUN) cliSerial->printf_P(PSTR(" NTUN"));
        if (g.log_bitmask & MASK_LOG_RCIN) cliSerial->printf_P(PSTR(" RCIN"));
        if (g.log_bitmask & MASK_LOG_IMU) cliSerial->printf_P(PSTR(" IMU"));
        if (g.log_bitmask & MASK_LOG_CMD) cliSerial->printf_P(PSTR(" CMD"));
        if (g.log_bitmask & MASK_LOG_CURRENT) cliSerial->printf_P(PSTR(" CURRENT"));
        if (g.log_bitmask & MASK_LOG_RCOUT) cliSerial->printf_P(PSTR(" RCOUT"));
        if (g.log_bitmask & MASK_LOG_OPTFLOW) cliSerial->printf_P(PSTR(" OPTFLOW"));
        if (g.log_bitmask & MASK_LOG_COMPASS) cliSerial->printf_P(PSTR(" COMPASS"));
        if (g.log_bitmask & MASK_LOG_CAMERA) cliSerial->printf_P(PSTR(" CAMERA"));
    }

    cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);

    return(true);
}

#if CLI_ENABLED == ENABLED
static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || ((uint16_t)dump_log <= (last_log_num - DataFlash.get_num_logs())) || (static_cast<uint16_t>(dump_log) > last_log_num)) {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return (0);
}
#endif

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint16_t bits;

    if (argc != 2) {
        cliSerial->printf_P(PSTR("missing log type\n"));
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
        bits = ~0;
    } else {
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(RCIN);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(RCOUT);
        TARG(OPTFLOW);
        TARG(COMPASS);
        TARG(CAMERA);
 #undef TARG
    }

    if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }

    return(0);
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}
#endif // CLI_ENABLED

static void do_erase_logs(void)
{
	gcs_send_text_P(SEVERITY_HIGH, PSTR("Erasing logs\n"));
    DataFlash.EraseAll();
	gcs_send_text_P(SEVERITY_HIGH, PSTR("Log erase complete\n"));
}

#if AUTOTUNE_ENABLED == ENABLED
struct PACKED log_AutoTune {
    LOG_PACKET_HEADER;
    uint8_t axis;           // roll or pitch
    uint8_t tune_step;      // tuning PI or D up or down
    float   rate_min;       // maximum achieved rotation rate
    float   rate_max;       // maximum achieved rotation rate
    float   new_gain_rp;       // newly calculated gain
    float   new_gain_rd;       // newly calculated gain
    float   new_gain_sp;       // newly calculated gain
};

// Write an Autotune data packet
static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp)
{
    struct log_AutoTune pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNE_MSG),
        axis        : axis,
        tune_step   : tune_step,
        rate_min    : rate_min,
        rate_max    : rate_max,
        new_gain_rp  : new_gain_rp,
        new_gain_rd  : new_gain_rd,
        new_gain_sp  : new_gain_sp
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AutoTuneDetails {
    LOG_PACKET_HEADER;
    int16_t angle_cd;       // lean angle in centi-degrees
    float   rate_cds;       // current rotation rate in centi-degrees / second
};

// Write an Autotune data packet
static void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds)
{
    struct log_AutoTuneDetails pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNEDETAILS_MSG),
        angle_cd    : angle_cd,
        rate_cds    : rate_cds
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle_out;
    uint32_t throttle_integrator;
    int16_t  battery_voltage;
    int16_t  current_amps;
    uint16_t board_voltage;
    float    current_total;
};

// Write an Current data packet
static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        time_ms             : hal.scheduler->millis(),
        throttle_out        : g.rc_3.servo_out,
        throttle_integrator : throttle_integrator,
        battery_voltage     : (int16_t) (battery.voltage() * 100.0f),
        current_amps        : (int16_t) (battery.current_amps() * 100.0f),
        board_voltage       : (uint16_t)(hal.analogin->board_voltage()*1000),
        current_total       : battery.current_total_mah()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    // also write power status
    DataFlash.Log_Write_Power();
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    int16_t dx;
    int16_t dy;
    uint8_t surface_quality;
    int16_t x_cm;
    int16_t y_cm;
    int32_t roll;
    int32_t pitch;
};

// Write an optical flow packet
static void Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        dx              : optflow.dx,
        dy              : optflow.dy,
        surface_quality : optflow.surface_quality,
        x_cm            : (int16_t) optflow.x_cm,
        y_cm            : (int16_t) optflow.y_cm,
        roll            : of_roll,
        pitch           : of_pitch
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // OPTFLOW == ENABLED
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float    desired_pos_x;
    float    desired_pos_y;
    float    pos_x;
    float    pos_y;
    float    desired_vel_x;
    float    desired_vel_y;
    float    vel_x;
    float    vel_y;
    float    desired_accel_x;
    float    desired_accel_y;
};

// Write an Nav Tuning packet
static void Log_Write_Nav_Tuning()
{
    const Vector3f &pos_target = pos_control.get_pos_target();
    const Vector3f &vel_target = pos_control.get_vel_target();
    const Vector3f &accel_target = pos_control.get_accel_target();
    const Vector3f &position = inertial_nav.get_position();
    const Vector3f &velocity = inertial_nav.get_velocity();

    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        time_ms         : hal.scheduler->millis(),
        desired_pos_x   : pos_target.x,
        desired_pos_y   : pos_target.y,
        pos_x           : position.x,
        pos_y           : position.y,
        desired_vel_x   : vel_target.x,
        desired_vel_y   : vel_target.y,
        vel_x           : velocity.x,
        vel_y           : velocity.y,
        desired_accel_x : accel_target.x,
        desired_accel_y : accel_target.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle_in;
    int16_t  angle_boost;
    int16_t  throttle_out;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    int16_t  desired_sonar_alt;
    int16_t  sonar_alt;
    int16_t  desired_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
static void Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_ms             : hal.scheduler->millis(),
        throttle_in         : g.rc_3.control_in,
        angle_boost         : attitude_control.angle_boost(),
        throttle_out        : g.rc_3.servo_out,
        desired_alt         : pos_control.get_alt_target() / 100.0f,
        inav_alt            : current_loc.alt / 100.0f,
        baro_alt            : baro_alt,
        desired_sonar_alt   : (int16_t)target_sonar_alt,
        sonar_alt           : sonar_alt,
        desired_climb_rate  : desired_climb_rate,
        climb_rate          : climb_rate
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int16_t  offset_x;
    int16_t  offset_y;
    int16_t  offset_z;
    int16_t  motor_offset_x;
    int16_t  motor_offset_y;
    int16_t  motor_offset_z;
};

// Write a Compass packet
static void Log_Write_Compass()
{
    const Vector3f &mag_offsets = compass.get_offsets(0);
    const Vector3f &mag_motor_offsets = compass.get_motor_offsets(0);
    const Vector3f &mag = compass.get_field(0);
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        time_ms         : hal.scheduler->millis(),
        mag_x           : (int16_t)mag.x,
        mag_y           : (int16_t)mag.y,
        mag_z           : (int16_t)mag.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z,
        motor_offset_x  : (int16_t)mag_motor_offsets.x,
        motor_offset_y  : (int16_t)mag_motor_offsets.y,
        motor_offset_z  : (int16_t)mag_motor_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#if COMPASS_MAX_INSTANCES > 1
    if (compass.get_count() > 1) {
        const Vector3f &mag2_offsets = compass.get_offsets(1);
        const Vector3f &mag2_motor_offsets = compass.get_motor_offsets(1);
        const Vector3f &mag2 = compass.get_field(1);
        struct log_Compass pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS2_MSG),
            time_ms         : hal.scheduler->millis(),
            mag_x           : (int16_t)mag2.x,
            mag_y           : (int16_t)mag2.y,
            mag_z           : (int16_t)mag2.z,
            offset_x        : (int16_t)mag2_offsets.x,
            offset_y        : (int16_t)mag2_offsets.y,
            offset_z        : (int16_t)mag2_offsets.z,
            motor_offset_x  : (int16_t)mag2_motor_offsets.x,
            motor_offset_y  : (int16_t)mag2_motor_offsets.y,
            motor_offset_z  : (int16_t)mag2_motor_offsets.z
        };
        DataFlash.WriteBlock(&pkt2, sizeof(pkt2));
    }
#endif
#if COMPASS_MAX_INSTANCES > 2
    if (compass.get_count() > 2) {
        const Vector3f &mag3_offsets = compass.get_offsets(2);
        const Vector3f &mag3_motor_offsets = compass.get_motor_offsets(2);
        const Vector3f &mag3 = compass.get_field(2);
        struct log_Compass pkt3 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS3_MSG),
            time_ms         : hal.scheduler->millis(),
            mag_x           : (int16_t)mag3.x,
            mag_y           : (int16_t)mag3.y,
            mag_z           : (int16_t)mag3.z,
            offset_x        : (int16_t)mag3_offsets.x,
            offset_y        : (int16_t)mag3_offsets.y,
            offset_z        : (int16_t)mag3_offsets.z,
            motor_offset_x  : (int16_t)mag3_motor_offsets.x,
            motor_offset_y  : (int16_t)mag3_motor_offsets.y,
            motor_offset_z  : (int16_t)mag3_motor_offsets.z
        };
        DataFlash.WriteBlock(&pkt3, sizeof(pkt3));
    }
#endif
}

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    int16_t  pm_test;
    uint8_t i2c_lockup_count;
    uint16_t ins_error_count;
    uint8_t inav_error_count;
};

// Write a performance monitoring packet
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time(),
        pm_test          : pmTest1,
        i2c_lockup_count : hal.i2c->lockup_count(),
        ins_error_count  : ins.error_count(),
        inav_error_count : inertial_nav.error_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write a mission command. Total length : 36 bytes
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd)
{
    mavlink_mission_item_t mav_cmd = {};
    AP_Mission::mission_cmd_to_mavlink(cmd,mav_cmd);
    DataFlash.Log_Write_MavCmd(mission.num_commands(),mav_cmd);
}

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  control_roll;
    int16_t  roll;
    int16_t  control_pitch;
    int16_t  pitch;
    uint16_t control_yaw;
    uint16_t yaw;
};

// Write an attitude packet
static void Log_Write_Attitude()
{
    const Vector3f &targets = attitude_control.angle_ef_targets();
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_ms         : hal.scheduler->millis(),
        control_roll    : (int16_t)targets.x,
        roll            : (int16_t)ahrs.roll_sensor,
        control_pitch   : (int16_t)targets.y,
        pitch           : (int16_t)ahrs.pitch_sensor,
        control_yaw     : (uint16_t)targets.z,
        yaw             : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

#if AP_AHRS_NAVEKF_AVAILABLE
    DataFlash.Log_Write_EKF(ahrs);
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.Log_Write_SIMSTATE(DataFlash);
#endif
}

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint8_t mode;
    int16_t throttle_cruise;
};

// Write a mode packet
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        mode            : mode,
        throttle_cruise : g.throttle_cruise,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
};

// Write Startup packet
static void Log_Write_Startup()
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint8_t id;
};

// Wrote an event packet
static void Log_Write_Event(uint8_t id)
{
    if (g.log_bitmask != 0) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            id  : id
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
static void Log_Write_Data(uint8_t id, int16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
static void Log_Write_Data(uint8_t id, uint16_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
static void Log_Write_Data(uint8_t id, int32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
static void Log_Write_Data(uint8_t id, uint32_t value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

// Write a float data packet
static void Log_Write_Data(uint8_t id, float value)
{
    if (g.log_bitmask != 0) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static void Log_Write_Baro(void)
{
    DataFlash.Log_Write_Baro(barometer);
}

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
#if AUTOTUNE_ENABLED == ENABLED
    { LOG_AUTOTUNE_MSG, sizeof(log_AutoTune),
      "ATUN", "BBfffff",       "Axis,TuneStep,RateMin,RateMax,RPGain,RDGain,SPGain" },
    { LOG_AUTOTUNEDETAILS_MSG, sizeof(log_AutoTuneDetails),
      "ATDE", "cf",          "Angle,Rate" },
#endif
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "IhIhhhf",     "TimeMS,ThrOut,ThrInt,Volt,Curr,Vcc,CurrTot" },
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),       
      "OF",   "hhBccee",   "Dx,Dy,SQual,X,Y,Roll,Pitch" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),       
      "NTUN", "Iffffffffff", "TimeMS,DPosX,DPosY,PosX,PosY,DVelX,DVelY,VelX,VelY,DAccX,DAccY" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Ihhhffecchh", "TimeMS,ThrIn,AngBst,ThrOut,DAlt,Alt,BarAlt,DSAlt,SAlt,DCRt,CRt" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
#if COMPASS_MAX_INSTANCES > 1
    { LOG_COMPASS2_MSG, sizeof(log_Compass),             
      "MAG2","Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
#endif
#if COMPASS_MAX_INSTANCES > 2
    { LOG_COMPASS3_MSG, sizeof(log_Compass),             
      "MAG3","Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
#endif
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "HHIhBHB",    "NLon,NLoop,MaxT,PMT,I2CErr,INSErr,INAVErr" },
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "IccccCC",      "TimeMS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw" },
    { LOG_MODE_MSG, sizeof(log_Mode),
      "MODE", "Mh",          "Mode,ThrCrs" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "",            "" },
    { LOG_EVENT_MSG, sizeof(log_Event),         
      "EV",   "B",           "Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "Bh",         "Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "BH",         "Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "Bi",         "Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "BI",         "Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "Bf",         "Id,Value" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "BB",         "Subsys,ECode" },
};

#if CLI_ENABLED == ENABLED
// Read the DataFlash log memory
static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
    cliSerial->printf_P(PSTR("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n"
                             "\nFrame: " FRAME_CONFIG_STRING "\n"),
                        (unsigned) hal.util->available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             print_flight_mode,
                             cliSerial);
}
#endif // CLI_ENABLED

// start a new log
static void start_logging() 
{
    if (g.log_bitmask != 0) {
        if (!ap.logging_started) {
            ap.logging_started = true;
            in_mavlink_delay = true;
            DataFlash.StartNewLog();
            in_mavlink_delay = false;
            DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
            DataFlash.Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

            // write system identifier as well if available
            char sysid[40];
            if (hal.util->get_system_id(sysid)) {
                DataFlash.Log_Write_Message(sysid);
            }
            DataFlash.Log_Write_Message_P(PSTR("Frame: " FRAME_CONFIG_STRING));

            // log the flight mode
            Log_Write_Mode(control_mode);
        }
        // enable writes
        DataFlash.EnableWrites(true);
    }
}

#else // LOGGING_ENABLED

static void Log_Write_Startup() {}
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_IMU() {}
#if AUTOTUNE_ENABLED == ENABLED
static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp) {}
static void Log_Write_AutoTuneDetails(int16_t angle_cd, float rate_cds) {}
#endif
static void Log_Write_Current() {}
static void Log_Write_Compass() {}
static void Log_Write_Attitude() {}
static void Log_Write_Data(uint8_t id, int16_t value){}
static void Log_Write_Data(uint8_t id, uint16_t value){}
static void Log_Write_Data(uint8_t id, int32_t value){}
static void Log_Write_Data(uint8_t id, uint32_t value){}
static void Log_Write_Data(uint8_t id, float value){}
static void Log_Write_Event(uint8_t id){}
static void Log_Write_Optflow() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Performance() {}
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd) {}
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
static void Log_Write_Baro(void) {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED
