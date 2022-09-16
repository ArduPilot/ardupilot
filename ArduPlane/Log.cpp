#include "Plane.h"

#if LOGGING_ENABLED == ENABLED

// Write an attitude packet
void Plane::Log_Write_Attitude(void)
{
    Vector3f targets;       // Package up the targets into a vector for commonality with Copter usage of Log_Wrote_Attitude
    targets.x = nav_roll_cd;
    targets.y = nav_pitch_cd;
    targets.z = 0; //Plane does not have the concept of navyaw. This is a placeholder.

#if HAL_QUADPLANE_ENABLED
    if (quadplane.show_vtol_view()) {
        // we need the attitude targets from the AC_AttitudeControl controller, as they
        // account for the acceleration limits.
        // Also, for bodyframe roll input types, _attitude_target_euler_angle is not maintained
        // since Euler angles are not used and it is a waste of cpu to compute them at the loop rate.
        // Get them from the quaternion instead:
        quadplane.attitude_control->get_attitude_target_quat().to_euler(targets.x, targets.y, targets.z);
        targets *= degrees(100.0f);
        quadplane.ahrs_view->Write_AttitudeView(targets);
    } else
#endif
            {
        ahrs.Write_Attitude(targets);
    }

#if HAL_QUADPLANE_ENABLED
    if (AP_HAL::millis() - quadplane.last_att_control_ms < 100) {
        // log quadplane PIDs separately from fixed wing PIDs
        logger.Write_PID(LOG_PIQR_MSG, quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIQP_MSG, quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIQY_MSG, quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIQA_MSG, quadplane.pos_control->get_accel_z_pid().get_pid_info() );
    }
    if (quadplane.in_vtol_mode() && quadplane.pos_control->is_active_xy()) {
        logger.Write_PID(LOG_PIDN_MSG, quadplane.pos_control->get_vel_xy_pid().get_pid_info_x());
        logger.Write_PID(LOG_PIDE_MSG, quadplane.pos_control->get_vel_xy_pid().get_pid_info_y());
    }
#endif

    logger.Write_PID(LOG_PIDR_MSG, rollController.get_pid_info());
    logger.Write_PID(LOG_PIDP_MSG, pitchController.get_pid_info());

    if (yawController.enabled()) {
        logger.Write_PID(LOG_PIDY_MSG, yawController.get_pid_info());
    }

    if (steerController.active()) {
        logger.Write_PID(LOG_PIDS_MSG, steerController.get_pid_info());
    }

    AP::ahrs().Log_Write();
}

// do fast logging for plane
void Plane::Log_Write_FullRate(void)
{
    // MASK_LOG_ATTITUDE_FULLRATE logs at 400Hz, MASK_LOG_ATTITUDE_FAST at 25Hz, MASK_LOG_ATTIUDE_MED logs at 10Hz
    // highest rate selected wins
    if (should_log(MASK_LOG_ATTITUDE_FULLRATE)) {
        Log_Write_Attitude();
    }
}


struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    float throttle_out;
    float rudder_out;
    float throttle_dem;
    float airspeed_estimate;
    float synthetic_airspeed;
    float EAS2TAS;
    int32_t groundspeed_undershoot;
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
        throttle_out    : SRV_Channels::get_output_scaled(SRV_Channel::k_throttle),
        rudder_out      : SRV_Channels::get_output_scaled(SRV_Channel::k_rudder),
        throttle_dem    : TECS_controller.get_throttle_demand(),
        airspeed_estimate : est_airspeed,
        synthetic_airspeed : synthetic_airspeed,
        EAS2TAS            : ahrs.get_EAS2TAS(),
        groundspeed_undershoot  : groundspeed_undershoot,
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
    float aileron;
    float elevator;
    float throttle;
    float rudder;
    float flap;
    float speed_scaler;
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
        ,flap     : SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto)
        ,speed_scaler : get_speed_scaler(),
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

// @LoggerMessage: CTUN
// @Description: Control Tuning information
// @Field: TimeUS: Time since system startup
// @Field: NavRoll: desired roll
// @Field: Roll: achieved roll
// @Field: NavPitch: desired pitch
// @Field: Pitch: achieved pitch
// @Field: ThO: scaled output throttle
// @Field: RdrOut: scaled output rudder
// @Field: ThD: demanded speed-height-controller throttle
// @Field: As: airspeed estimate (or measurement if airspeed sensor healthy and ARSPD_USE>0)
// @Field: SAs: synthetic airspeed measurement derived from non-airspeed sensors, NaN if not available
// @Field: E2T: equivalent to true airspeed ratio
// @Field: GU: groundspeed undershoot when flying with minimum groundspeed

    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Qccccffffffi",    "TimeUS,NavRoll,Roll,NavPitch,Pitch,ThO,RdrOut,ThD,As,SAs,E2T,GU", "sdddd---nn-n", "FBBBB---00-B" , true },

// @LoggerMessage: NTUN
// @Description: Navigation Tuning information - e.g. vehicle destination
// @Field: TimeUS: Time since system startup
// @Field: Dist: distance to the current navigation waypoint
// @Field: TBrg: bearing to the current navigation waypoint
// @Field: NavBrg: the vehicle's desired heading
// @Field: AltErr: difference between current vehicle height and target height
// @Field: XT: the vehicle's current distance from the current travel segment
// @Field: XTi: integration of the vehicle's crosstrack error
// @Field: AspdE: difference between vehicle's airspeed and desired airspeed
// @Field: TLat: target latitude
// @Field: TLng: target longitude
// @Field: TAlt: target altitude
// @Field: TAspd: target airspeed
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "QfcccfffLLii",  "TimeUS,Dist,TBrg,NavBrg,AltErr,XT,XTi,AspdE,TLat,TLng,TAlt,TAspd", "smddmmmnDUmn", "F0BBB0B0GGBB" , true },

// @LoggerMessage: ATRP
// @Description: Plane AutoTune
// @Vehicles: Plane
// @Field: TimeUS: Time since system startup
// @Field: Axis: tuning axis
// @Field: State: tuning state
// @Field: Sur: control surface deflection
// @Field: PSlew: P slew rate
// @Field: DSlew: D slew rate
// @Field: FF0: FF value single sample
// @Field: FF: FF value
// @Field: P: P value
// @Field: I: I value
// @Field: D: D value
// @Field: Action: action taken
// @Field: RMAX: Rate maximum
// @Field: TAU: time constant
    { LOG_ATRP_MSG, sizeof(AP_AutoTune::log_ATRP),
      "ATRP", "QBBffffffffBff", "TimeUS,Axis,State,Sur,PSlew,DSlew,FF0,FF,P,I,D,Action,RMAX,TAU", "s#-dkk------ks", "F--00000000-00" , true },

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
      "STAT", "QBfBBBBBB",  "TimeUS,isFlying,isFlyProb,Armed,Safety,Crash,Still,Stage,Hit", "s--------", "F--------" , true },

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
// @Field: Trn: Transistion state
// @Field: Ast: Q assist active state
#if HAL_QUADPLANE_ENABLED
    { LOG_QTUN_MSG, sizeof(QuadPlane::log_QControl_Tuning),
      "QTUN", "QffffffeccffBB", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DCRt,CRt,TMix,Sscl,Trn,Ast", "s----mmmnn----", "F----00000-0--" , true },
#endif

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
// @Field: SRate: slew rate
// @Field: Limit: 1 if I term is limited due to output saturation
    { LOG_PIQR_MSG, sizeof(log_PID),
      "PIQR", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },
    { LOG_PIQP_MSG, sizeof(log_PID),
      "PIQP", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },
    { LOG_PIQY_MSG, sizeof(log_PID),
      "PIQY", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },
    { LOG_PIQA_MSG, sizeof(log_PID),
      "PIQA", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },

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
// @Field: SRate: slew rate
// @Field: Limit: 1 if I term is limited due to output saturation
    { LOG_PIDG_MSG, sizeof(log_PID),
      "PIDG", PID_FMT,  PID_LABELS, PID_UNITS, PID_MULTS , true },

// @LoggerMessage: AETR
// @Description: Normalised pre-mixer control surface outputs
// @Field: TimeUS: Time since system startup
// @Field: Ail: Pre-mixer value for aileron output (between -4500 to 4500)
// @Field: Elev: Pre-mixer value for elevator output (between -4500 to 4500)
// @Field: Thr: Pre-mixer value for throttle output (between -4500 to 4500)
// @Field: Rudd: Pre-mixer value for rudder output (between -4500 to 4500)
// @Field: Flap: Pre-mixer value for flaps output (between -4500 to 4500)
// @Field: SS: Surface movement / airspeed scaling value
    { LOG_AETR_MSG, sizeof(log_AETR),
      "AETR", "Qffffff",  "TimeUS,Ail,Elev,Thr,Rudd,Flap,SS", "s------", "F------" , true },

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
      "OFG", "QffffBff",    "TimeUS,Arsp,ArspA,Alt,AltA,AltF,Hdg,HdgA", "s-------", "F-------" , true }, 
};

void Plane::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
#if HAL_QUADPLANE_ENABLED
    if (quadplane.initialised) {
        char frame_and_type_string[30];
        quadplane.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
        logger.Write_MessageF("QuadPlane %s", frame_and_type_string);
    }
#endif
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
void Plane::Log_Write_Control_Tuning() {}
void Plane::Log_Write_OFG_Guided() {}
void Plane::Log_Write_Nav_Tuning() {}
void Plane::Log_Write_Status() {}
void Plane::Log_Write_Guided(void) {}
void Plane::Log_Write_RC(void) {}
void Plane::Log_Write_Vehicle_Startup_Messages() {}

void Plane::log_init(void) {}

#endif // LOGGING_ENABLED
