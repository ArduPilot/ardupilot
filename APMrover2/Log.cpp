#include "Rover.h"
#include "version.h"

#include <AP_RangeFinder/RangeFinder_Backend.h>

#if LOGGING_ENABLED == ENABLED

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
        wp_distance         : control_mode->get_distance_to_destination(),
        target_bearing_cd   : static_cast<uint16_t>(abs(nav_controller->target_bearing_cd())),
        nav_bearing_cd      : static_cast<uint16_t>(abs(nav_controller->nav_bearing_cd())),
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
    DataFlash.Log_Write_EKF(ahrs);
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
    DataFlash.Log_Write_POS(ahrs);

    // log steering rate controller
    DataFlash.Log_Write_PID(LOG_PIDS_MSG, g2.attitude_control.get_steering_rate_pid().get_pid_info());
    DataFlash.Log_Write_PID(LOG_PIDA_MSG, g2.attitude_control.get_throttle_speed_pid().get_pid_info());
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
    AP_RangeFinder_Backend *s0 = rangefinder.get_backend(0);
    AP_RangeFinder_Backend *s1 = rangefinder.get_backend(1);
    struct log_Rangefinder pkt = {
        LOG_PACKET_HEADER_INIT(LOG_RANGEFINDER_MSG),
        time_us               : AP_HAL::micros64(),
        lateral_accel         : control_mode->lateral_acceleration,
        rangefinder1_distance : s0 ? s0->distance_cm() : (uint16_t)0,
        rangefinder2_distance : s1 ? s1->distance_cm() : (uint16_t)0,
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
        quality_0   : (uint8_t)constrain_float(g2.wheel_encoder.get_signal_quality(0), 0.0f, 100.0f),
        distance_1  : g2.wheel_encoder.get_distance(1),
        quality_1   : (uint8_t)constrain_float(g2.wheel_encoder.get_signal_quality(1), 0.0f, 100.0f)
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
}

void Rover::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    DataFlash.Log_Write_Mode(control_mode->mode_number(), control_mode_reason);
    Log_Write_Home_And_Origin();
    gps.Write_DataFlash_Log_Startup_messages();
}

#else  // LOGGING_ENABLED

// dummy functions
void Rover::Log_Write_Startup(uint8_t type) {}
void Rover::Log_Write_Current() {}
void Rover::Log_Write_Nav_Tuning() {}
void Rover::Log_Write_Performance() {}
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
