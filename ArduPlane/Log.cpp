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

    if (quadplane.tailsitter_active() || quadplane.in_vtol_mode()) {
        // we need the attitude targets from the AC_AttitudeControl controller, as they
        // account for the acceleration limits.
        // Also, for bodyframe roll input types, _attitude_target_euler_angle is not maintained
        // since Euler angles are not used and it is a waste of cpu to compute them at the loop rate.
        // Get them from the quaternion instead:
        quadplane.attitude_control->get_attitude_target_quat().to_euler(targets.x, targets.y, targets.z);
        targets *= degrees(100.0f);
        logger.Write_AttitudeView(*quadplane.ahrs_view, targets);
    } else {
        logger.Write_Attitude(targets);
    }
    if (quadplane.in_vtol_mode() || quadplane.in_assisted_flight()) {
        // log quadplane PIDs separately from fixed wing PIDs
        logger.Write_PID(LOG_PIQR_MSG, quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIQP_MSG, quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIQY_MSG, quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIQA_MSG, quadplane.pos_control->get_accel_z_pid().get_pid_info() );
    }

    logger.Write_PID(LOG_PIDR_MSG, rollController.get_pid_info());
    logger.Write_PID(LOG_PIDP_MSG, pitchController.get_pid_info());
    logger.Write_PID(LOG_PIDY_MSG, yawController.get_pid_info());
    logger.Write_PID(LOG_PIDS_MSG, steerController.get_pid_info());

#if AP_AHRS_NAVEKF_AVAILABLE
    AP::ahrs_navekf().Log_Write();
    logger.Write_AHRS2();
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE();
#endif
    logger.Write_POS();
}

// do fast logging for plane
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
    logger.WriteCriticalBlock(&pkt, sizeof(pkt));
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
    float synthetic_airspeed;
};

// Write a control tuning packet. Total length : 22 bytes
void Plane::Log_Write_Control_Tuning()
{
    float est_airspeed = 0;
    ahrs.airspeed_estimate(est_airspeed);

    float synthetic_airspeed;
    if (!ahrs.synthetic_airspeed(synthetic_airspeed)) {
        synthetic_airspeed = logger.quiet_nan();
    }

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
        airspeed_estimate : est_airspeed,
        synthetic_airspeed : synthetic_airspeed
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_OFG_Guided {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float target_airspeed_cm;
    float target_airspeed_accel;
    float target_alt;
    float target_alt_accel;
    uint8_t target_alt_frame;
    float target_heading;
    float target_heading_limit;
};

// Write a OFG Guided packet.
void Plane::Log_Write_OFG_Guided()
{
#if OFFBOARD_GUIDED == ENABLED
    struct log_OFG_Guided pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OFG_MSG),
        time_us                : AP_HAL::micros64(),
        target_airspeed_cm     : (float)guided_state.target_airspeed_cm*(float)0.01,
        target_airspeed_accel  : guided_state.target_airspeed_accel,
        target_alt             : guided_state.target_alt,
        target_alt_accel       : guided_state.target_alt_accel,
        target_alt_frame       : guided_state.target_alt_frame,
        target_heading         : guided_state.target_heading,
        target_heading_limit   : guided_state.target_heading_accel_limit
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
#endif
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
    logger.WriteBlock(&pkt, sizeof(pkt));
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

    logger.WriteBlock(&pkt, sizeof(pkt));
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

    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Plane::Log_Write_RC(void)
{
    logger.Write_RCIN();
    logger.Write_RCOUT();
    if (rssi.enabled()) {
        logger.Write_RSSI();
    }
    Log_Write_AETR();
}

void Plane::Log_Write_Guided(void)
{
#if OFFBOARD_GUIDED == ENABLED
    if (control_mode != &mode_guided) {
        return;
    }

    if (guided_state.target_heading_time_ms != 0) {
        logger.Write_PID(LOG_PIDG_MSG, g2.guidedHeading.get_pid_info());
    }

    if ( is_positive(guided_state.target_alt) || is_positive(guided_state.target_airspeed_cm) ) {
        Log_Write_OFG_Guided();
    }
#endif // OFFBOARD_GUIDED == ENABLED
}

// incoming-to-vehicle mavlink COMMAND_INT can be logged
struct PACKED log_CMDI {
    LOG_PACKET_HEADER;
    uint64_t TimeUS;
    uint16_t CId;
    uint8_t TSys;
    uint8_t TCmp;
    uint8_t cur;
    uint8_t cont;
    float Prm1;
    float Prm2;
    float Prm3;
    float Prm4;
    int32_t Lat;
    int32_t Lng;
    float Alt;
    uint8_t F;
};

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Plane::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "QBH",         "TimeUS,SType,CTot", "s--", "F--" },

// @LoggerMessage: CTUN
// @Description: Control Tuning information
// @Field: TimeUS: Time since system startup
// @Field: NavRoll: desired roll
// @Field: Roll: achieved roll
// @Field: NavPitch: desired pitch
// @Field: Pitch: achieved pitch
// @Field: ThrOut: scaled output throttle
// @Field: RdrOut: scaled output rudder
// @Field: ThrDem: demanded speed-height-controller throttle
// @Field: Aspd: airspeed estimate (or measurement if airspeed sensor healthy and ARSPD_USE>0)
// @Field: SAs: synthetic airspeed measurement derived from non-airspeed sensors, NaN if not available

    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Qcccchhhff",    "TimeUS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,ThrDem,Aspd,SAs", "sdddd---nn", "FBBBB---00" },

// @LoggerMessage: NTUN
// @Description: Navigation Tuning information - e.g. vehicle destination
// @URL: http://ardupilot.org/rover/docs/navigation.html
// @Field: TimeUS: Time since system startup
// @Field: Dist: distance to the current navigation waypoint
// @Field: TBrg: bearing to the current navigation waypoint
// @Field: NavBrg: the vehicle's desired heading
// @Field: AltErr: difference between current vehicle height and target height
// @Field: XT: the vehicle's current distance from the current travel segment
// @Field: XTi: integration of the vehicle's crosstrack error
// @Field: AspdE: difference between vehicle's airspeed and desired airspeed
// @Field: AspdE: difference between vehicle's airspeed and desired airspeed
// @Field: TLat: target latitude
// @Field: TLng: target longitude
// @Field: TAlt: target altitude
// @Field: TAspd: target airspeed
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "QfcccfffLLii",  "TimeUS,Dist,TBrg,NavBrg,AltErr,XT,XTi,AspdE,TLat,TLng,TAlt,TAspd", "smddmmmnDUmn", "F0BBB0B0GGBB" },

// @LoggerMessage: ATRP
// @Description: Pitch/Roll AutoTune messages for Plane 
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of autotune (0 = Roll/ 1 = Pitch)
// @Field: State: AutoTune state
// @Field: Servo: Normalised control surface output (between -4500 to 4500)
// @Field: Demanded: Desired Pitch/Roll rate
// @Field: Achieved: Achieved Pitch/Roll rate
// @Field: P: Proportional part of PID
    { LOG_ATRP_MSG, sizeof(AP_AutoTune::log_ATRP),
      "ATRP", "QBBcfff",  "TimeUS,Type,State,Servo,Demanded,Achieved,P", "s---dd-", "F---00-" },

// @LoggerMessage: STAT
// @Description: Current status of the aircraft
// @Field: TimeUS: Time since system startup
// @Field: isFlying: True if aircraft is probably flying
// @Field: isFlyProb: Probabilty that the aircraft is flying
// @Field: Armed: Arm status of the aircraft
// @Field: Safety: State of the safety switch
// @Field: Crash: True if crash is detected
// @Field: Still: True when vehicle is not moving in any axis
// @Field: Stage: Current stage of the flight
// @Field: Hit: True if impact is detected
    { LOG_STATUS_MSG, sizeof(log_Status),
      "STAT", "QBfBBBBBB",  "TimeUS,isFlying,isFlyProb,Armed,Safety,Crash,Still,Stage,Hit", "s--------", "F--------" },

// @LoggerMessage: QTUN
// @Description: QuadPlane vertical tuning message
// @Field: TimeUS: Time since system startup
// @Field: ThI: throttle input
// @Field: ABst: angle boost
// @Field: ThO: throttle output
// @Field: ThH: calculated hover throttle
// @Field: DAlt: desired altitude
// @Field: Alt: achieved altitude
// @Field: BAlt: barometric altitude
// @Field: DCRt: desired climb rate
// @Field: CRt: climb rate
// @Field: TMix: transition throttle mix value
// @Field: Sscl: speed scalar for tailsitter control surfaces
    { LOG_QTUN_MSG, sizeof(QuadPlane::log_QControl_Tuning),
      "QTUN", "Qffffffeccff", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DCRt,CRt,TMix,Sscl", "s----mmmnn--", "F----00000-0" },

// @LoggerMessage: AOA
// @Description: Angle of attack and Side Slip Angle values
// @Field: TimeUS: Time since system startup
// @Field: AOA: Angle of Attack calculated from airspeed, wind vector,velocity vector 
// @Field: SSA: Side Slip Angle calculated from airspeed, wind vector,velocity vector
    { LOG_AOA_SSA_MSG, sizeof(log_AOA_SSA),
      "AOA", "Qff", "TimeUS,AOA,SSA", "sdd", "F00" },

// @LoggerMessage: PIQR,PIQP,PIQY,PIQA
// @Description: QuadPlane Proportional/Integral/Derivative gain values for Roll/Pitch/Yaw/Z
// @Field: TimeUS: Time since system startup
// @Field: Tar: desired value
// @Field: Act: achieved value
// @Field: Err: error between target and achieved
// @Field: P: proportional part of PID
// @Field: I: integral part of PID
// @Field: D: derivative part of PID
// @Field: FF: controller feed-forward portion of response
// @Field: Dmod: scaler applied to D gain to reduce limit cycling
    { LOG_PIQR_MSG, sizeof(log_PID),
      "PIQR", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },
    { LOG_PIQP_MSG, sizeof(log_PID),
      "PIQP", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },
    { LOG_PIQY_MSG, sizeof(log_PID),
      "PIQY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },
    { LOG_PIQA_MSG, sizeof(log_PID),
      "PIQA", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },

// @LoggerMessage: PIDG
// @Description: Plane Proportional/Integral/Derivative gain values for Heading when using COMMAND_INT control.
// @Field: TimeUS: Time since system startup
// @Field: Tar: desired value
// @Field: Act: achieved value
// @Field: Err: error between target and achieved
// @Field: P: proportional part of PID
// @Field: I: integral part of PID
// @Field: D: derivative part of PID
// @Field: FF: controller feed-forward portion of response
// @Field: Dmod: scaler applied to D gain to reduce limit cycling
    { LOG_PIDG_MSG, sizeof(log_PID),
      "PIDG", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS },

// @LoggerMessage: AETR
// @Description: Normalised pre-mixer control surface outputs
// @Field: TimeUS: Time since system startup
// @Field: Ail: Pre-mixer value for aileron output (between -4500 to 4500)
// @Field: Elev: Pre-mixer value for elevator output (between -4500 to 4500)
// @Field: Thr: Pre-mixer value for throttle output (between -4500 to 4500)
// @Field: Rudd: Pre-mixer value for rudder output (between -4500 to 4500)
// @Field: Flap: Pre-mixer value for flaps output (between -4500 to 4500)
    { LOG_AETR_MSG, sizeof(log_AETR),
      "AETR", "Qhhhhh",  "TimeUS,Ail,Elev,Thr,Rudd,Flap", "s-----", "F-----" },

// @LoggerMessage: OFG
// @Description: OFfboard-Guided - an advanced version of GUIDED for companion computers that includes rate/s.  
// @Field: TimeUS: Time since system startup
// @Field: Arsp:  target airspeed cm
// @Field: ArspA:  target airspeed accel
// @Field: Alt:  target alt
// @Field: AltA: target alt accel
// @Field: AltF: target alt frame
// @Field: Hdg:  target heading
// @Field: HdgA: target heading lim
    { LOG_OFG_MSG, sizeof(log_OFG_Guided),     
      "OFG", "QffffBff",    "TimeUS,Arsp,ArspA,Alt,AltA,AltF,Hdg,HdgA", "s-------", "F-------" }, 

// @LoggerMessage: CMDI
// @Description: Generic CommandInt message logger(CMDI) 
// @Field: TimeUS: Time since system startup
// @Field: CId:  command id
// @Field: TSys: target system
// @Field: TCmp: target component
// @Field: cur:  current
// @Field: cont: autocontinue
// @Field: Prm1: parameter 1
// @Field: Prm2: parameter 2
// @Field: Prm3: parameter 3
// @Field: Prm4: parameter 4
// @Field: Lat: target latitude
// @Field: Lng: target longitude
// @Field: Alt: target altitude
// @Field: F:   frame
    { LOG_CMDI_MSG, sizeof(log_CMDI),     
      "CMDI", "QHBBBBffffiifB",    "TimeUS,CId,TSys,TCmp,cur,cont,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,F", "s---------DUm-", "F---------GGB-" }, 
// these next three are same format as log_CMDI just each a different name for Heading,Speed and Alt COMMAND_INTs
    { LOG_CMDS_MSG, sizeof(log_CMDI),     
      "CMDS", "QHBBBBffffiifB",    "TimeUS,CId,TSys,TCmp,cur,cont,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,F", "s---------DUm-", "F---------GGB-" }, 
    { LOG_CMDA_MSG, sizeof(log_CMDI),     
      "CMDA", "QHBBBBffffiifB",    "TimeUS,CId,TSys,TCmp,cur,cont,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,F", "s---------DUm-", "F---------GGB-" }, 
    { LOG_CMDH_MSG, sizeof(log_CMDI),     
      "CMDH", "QHBBBBffffiifB",    "TimeUS,CId,TSys,TCmp,cur,cont,Prm1,Prm2,Prm3,Prm4,Lat,Lng,Alt,F", "s---------DUm-", "F---------GGB-" }, 

};


// Write a COMMAND INT packet.
void Plane::Log_Write_MavCmdI(const mavlink_command_int_t &mav_cmd)
{
    struct log_CMDI pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CMDI_MSG),
        TimeUS:AP_HAL::micros64(),
        CId:   mav_cmd.command,
        TSys:  mav_cmd.target_system,
        TCmp:  mav_cmd.target_component,
        cur:   mav_cmd.current,
        cont:  mav_cmd.autocontinue,
        Prm1:  mav_cmd.param1,
        Prm2:  mav_cmd.param2,
        Prm3:  mav_cmd.param3,
        Prm4:  mav_cmd.param4,
        Lat:   mav_cmd.x,
        Lng:   mav_cmd.y,
        Alt:   mav_cmd.z,
        F:     mav_cmd.frame
};

// rather than have 4 different functions for these similar outputs, we just create it as a CMDI and rename it here
#if OFFBOARD_GUIDED == ENABLED
    if (mav_cmd.command == MAV_CMD_GUIDED_CHANGE_SPEED) {
        pkt.msgid = LOG_CMDS_MSG;
    } else if (mav_cmd.command == MAV_CMD_GUIDED_CHANGE_ALTITUDE) {
        pkt.msgid = LOG_CMDA_MSG;
    } else if (mav_cmd.command == MAV_CMD_GUIDED_CHANGE_HEADING) {
        pkt.msgid = LOG_CMDH_MSG;
    }
#endif
    //normally pkt.msgid = LOG_CMDI_MSG
    logger.WriteBlock(&pkt, sizeof(pkt));

}

void Plane::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

/*
  initialise logging subsystem
 */
void Plane::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Plane::Log_Write_Attitude(void) {}
void Plane::Log_Write_Fast(void) {}
void Plane::Log_Write_Performance() {}
void Plane::Log_Write_Startup(uint8_t type) {}
void Plane::Log_Write_Control_Tuning() {}
void Plane::Log_Write_OFG_Guided() {}
void Plane::Log_Write_Nav_Tuning() {}
void Plane::Log_Write_Status() {}
void Plane::Log_Write_Guided(void) {}

void Plane::Log_Write_RC(void) {}
void Plane::Log_Write_Vehicle_Startup_Messages() {}

void Plane::log_init(void) {}

#endif // LOGGING_ENABLED
