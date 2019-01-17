#include "Plane.h"

#if LOGGING_ENABLED == ENABLED

// Write an attitude packet
void Plane::Log_Write_Attitude(void)
{
    Vector3f targets;       // Package up the targets into a vector for commonality with Copter usage of Log_Wrote_Attitude
    targets.x = nav_roll_cd;
    targets.y = nav_pitch_cd;

    if (quadplane.in_vtol_mode() || quadplane.in_assisted_flight()) {
        // when VTOL active log the copter target yaw
        targets.z = wrap_360_cd(quadplane.attitude_control->get_att_target_euler_cd().z);
    } else {
        //Plane does not have the concept of navyaw. This is a placeholder.
        targets.z = 0;
    }
    
    if (quadplane.tailsitter_active()) {
        DataFlash.Log_Write_AttitudeView(*quadplane.ahrs_view, targets);

    } else if (quadplane.in_vtol_mode()) {
        targets = quadplane.attitude_control->get_att_target_euler_cd();
        DataFlash.Log_Write_Attitude(ahrs, targets);
    } else {
        DataFlash.Log_Write_Attitude(ahrs, targets);
    }
    if (quadplane.in_vtol_mode() || quadplane.in_assisted_flight()) {
        // log quadplane PIDs separately from fixed wing PIDs
        DataFlash.Log_Write_PID(LOG_PIQR_MSG, quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIQP_MSG, quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIQY_MSG, quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIQA_MSG, quadplane.pos_control->get_accel_z_pid().get_pid_info() );
    }

    DataFlash.Log_Write_PID(LOG_PIDR_MSG, rollController.get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDP_MSG, pitchController.get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDY_MSG, yawController.get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDS_MSG, steerController.get_pid_info());

#if AP_AHRS_NAVEKF_AVAILABLE
    DataFlash.Log_Write_EKF(ahrs);
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE();
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
    float airspeed_estimate;
};

// Write a control tuning packet. Total length : 22 bytes
void Plane::Log_Write_Control_Tuning()
{
    float est_airspeed = 0;
    ahrs.airspeed_estimate(&est_airspeed);
    
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_us         : AP_HAL::micros64(),
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle),
        rudder_out      : (int16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_rudder),
        throttle_dem    : (int16_t)SpdHgt_Controller->get_throttle_demand(),
        airspeed_estimate : est_airspeed
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
    int32_t target_lat;
    int32_t target_lng;
    int32_t target_alt;
    int32_t target_airspeed;
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
        airspeed_error      : airspeed_error,
        target_lat          : next_WP_loc.lat,
        target_lng          : next_WP_loc.lng,
        target_alt          : next_WP_loc.alt,
        target_airspeed     : target_airspeed_cm,
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
        ,is_still    : AP::ins().is_still()
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

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  arm_state;
    uint16_t arm_checks;
};

void Plane::Log_Arm_Disarm() {
    struct log_Arm_Disarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_us                 : AP_HAL::micros64(),
        arm_state               : arming.is_armed(),
        arm_checks              : arming.get_enabled_checks()      
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}


struct PACKED log_AETR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t aileron;
    int16_t elevator;
    int16_t throttle;
    int16_t rudder;
    int16_t flap;
};

void Plane::Log_Write_AETR()
{
    struct log_AETR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AETR_MSG)
        ,time_us  : AP_HAL::micros64()
        ,aileron  : SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)
        ,elevator : SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)
        ,throttle : SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)
        ,rudder   : SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)
        ,flap     : SRV_Channels::get_output_scaled(SRV_Channel::k_flap_auto)
        };

    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

void Plane::Log_Write_RC(void)
{
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
    if (rssi.enabled()) {
        DataFlash.Log_Write_RSSI(rssi);
    }
    Log_Write_AETR();
}

// type and unit information can be found in
// libraries/DataFlash/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Plane::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "QBH",         "TimeUS,SType,CTot", "s--", "F--" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Qcccchhhf",    "TimeUS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,ThrDem,Aspd", "sdddd---n", "FBBBB---0" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "QfcccfffLLii",  "TimeUS,Dist,TBrg,NavBrg,AltErr,XT,XTi,AspdE,TLat,TLng,TAlt,TAspd", "smddmmmnDUmn", "F0BBB0B0GGBB" },
    { LOG_SONAR_MSG, sizeof(log_Sonar),             
      "SONR", "QffBf",   "TimeUS,Dist,Volt,Cnt,Corr", "smv--", "FB0--" },
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm),
      "ARM", "QBH", "TimeUS,ArmState,ArmChecks", "s--", "F--" },
    { LOG_ATRP_MSG, sizeof(AP_AutoTune::log_ATRP),
      "ATRP", "QBBcfff",  "TimeUS,Type,State,Servo,Demanded,Achieved,P", "s---dd-", "F---00-" },
    { LOG_STATUS_MSG, sizeof(log_Status),
      "STAT", "QBfBBBBBB",  "TimeUS,isFlying,isFlyProb,Armed,Safety,Crash,Still,Stage,Hit", "s--------", "F--------" },
    { LOG_QTUN_MSG, sizeof(QuadPlane::log_QControl_Tuning),
      "QTUN", "Qffffffeccf", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DCRt,CRt,TMix", "s----mmmnn-", "F----00000-" },
    { LOG_AOA_SSA_MSG, sizeof(log_AOA_SSA),
      "AOA", "Qff", "TimeUS,AOA,SSA", "sdd", "F00" },
    { LOG_PIQR_MSG, sizeof(log_PID), \
      "PIQR", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },  \
    { LOG_PIQP_MSG, sizeof(log_PID), \
      "PIQP", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIQY_MSG, sizeof(log_PID), \
      "PIQY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_PIQA_MSG, sizeof(log_PID), \
      "PIQA", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS }, \
    { LOG_AETR_MSG, sizeof(log_AETR), \
      "AETR", "Qhhhhh",  "TimeUS,Ail,Elev,Thr,Rudd,Flap", "s-----", "F-----" },  \
};

void Plane::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);
    DataFlash.Log_Write_Rally(rally);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_DataFlash_Log_Startup_messages();
}

/*
  initialise logging subsystem
 */
void Plane::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Plane::Log_Write_Attitude(void) {}
void Plane::Log_Write_Fast(void) {}
void Plane::Log_Write_Performance() {}
void Plane::Log_Write_Startup(uint8_t type) {}
void Plane::Log_Write_Control_Tuning() {}
void Plane::Log_Write_Nav_Tuning() {}
void Plane::Log_Write_Status() {}
void Plane::Log_Write_Sonar() {}

void Plane::Log_Arm_Disarm() {}
void Plane::Log_Write_RC(void) {}
void Plane::Log_Write_Vehicle_Startup_Messages() {}

void Plane::log_init(void) {}

#endif // LOGGING_ENABLED
