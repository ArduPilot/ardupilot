#pragma once

#include <AP_Logger/LogStructure.h>
#include <AP_Math/vector3.h>
#include <AP_Math/vector2.h>
#include <AP_Math/matrix3.h>
#include <AP_Math/quaternion.h>

#define LOG_IDS_FROM_DAL \
    LOG_RFRH_MSG, \
    LOG_RFRF_MSG, \
    LOG_REV2_MSG, \
    LOG_RSO2_MSG, \
    LOG_RWA2_MSG, \
    LOG_REV3_MSG, \
    LOG_RSO3_MSG, \
    LOG_RWA3_MSG, \
    LOG_REY3_MSG, \
    LOG_RFRN_MSG, \
    LOG_RISH_MSG, \
    LOG_RISI_MSG, \
    LOG_RBRH_MSG, \
    LOG_RBRI_MSG, \
    LOG_RRNH_MSG, \
    LOG_RRNI_MSG, \
    LOG_RGPH_MSG, \
    LOG_RGPI_MSG, \
    LOG_RGPJ_MSG, \
    LOG_RASH_MSG, \
    LOG_RASI_MSG, \
    LOG_RBCH_MSG, \
    LOG_RBCI_MSG, \
    LOG_RVOH_MSG, \
    LOG_RMGH_MSG, \
    LOG_RMGI_MSG, \
    LOG_ROFH_MSG, \
    LOG_REPH_MSG, \
    LOG_REVH_MSG, \
    LOG_RWOH_MSG, \
    LOG_RBOH_MSG

// Replay Data Structures
struct log_RFRH {
    uint64_t time_us;
    uint32_t time_flying_ms;
    uint8_t _end;
};

struct log_RFRF {
    uint8_t frame_types;
    uint8_t core_slow;
    uint8_t _end;
};

struct log_RFRN {
    int32_t lat;
    int32_t lng;
    int32_t alt;
    float EAS2TAS;
    uint32_t available_memory;
    Vector3f ahrs_trim;
    uint8_t vehicle_class;
    uint8_t ekf_type;
    uint8_t armed:1;
    uint8_t unused:1;  // was get_compass_is_null
    uint8_t fly_forward:1;
    uint8_t ahrs_airspeed_sensor_enabled:1;
    uint8_t opticalflow_enabled:1;
    uint8_t wheelencoder_enabled:1;
    uint8_t takeoff_expected:1;
    uint8_t touchdown_expected:1;
    uint8_t _end;
};

// Replay Data Structure - Inertial Sensor header
struct log_RISH {
    uint16_t loop_rate_hz;
    uint8_t primary_gyro;
    uint8_t primary_accel;
    float loop_delta_t;
    uint8_t accel_count;
    uint8_t gyro_count;
    uint8_t _end;
};

// Replay Data Structure - Inertial Sensor instance data
struct log_RISI {
    Vector3f delta_velocity;
    Vector3f delta_angle;
    float delta_velocity_dt;
    float delta_angle_dt;
    uint8_t use_accel:1;
    uint8_t use_gyro:1;
    uint8_t get_delta_velocity_ret:1;
    uint8_t get_delta_angle_ret:1;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: REV2
// @Description: Replay Event
struct log_REV2 {
    uint8_t event;
    uint8_t _end;
};

// @LoggerMessage: RSO2
// @Description: Replay Set Origin event
struct log_RSO2 {
    int32_t lat;
    int32_t lng;
    int32_t alt;
    uint8_t _end;
};

// @LoggerMessage: RWA2
// @Description: Replay set-default-airspeed event
struct log_RWA2 {
    float airspeed;
    float uncertainty;
    uint8_t _end;
};

// same structures for EKF3
#define log_REV3 log_REV2
#define log_RSO3 log_RSO2
#define log_RWA3 log_RWA2

// @LoggerMessage: REY3
// @Description: Replay Euler Yaw event
struct log_REY3 {
    float yawangle;
    float yawangleerr;
    uint32_t timestamp_ms;
    uint8_t type;
    uint8_t _end;
};

// @LoggerMessage: RBRH
// @Description: Replay Data Barometer Header
struct log_RBRH {
    uint8_t primary;
    uint8_t num_instances;
    uint8_t _end;
};

// @LoggerMessage: RBRI
// @Description: Replay Data Barometer Instance
struct log_RBRI {
    uint32_t last_update_ms;
    float altitude;  // from get_altitude
    bool healthy;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RRNH
// @Description: Replay Data Rangefinder Header
struct log_RRNH {
    // this is rotation-pitch-270!
    int16_t ground_clearance_cm;
    int16_t max_distance_cm;
    uint8_t num_sensors;
    uint8_t _end;
};

// @LoggerMessage: RRNI
// @Description: Replay Data Rangefinder Instance
struct log_RRNI {
    Vector3f pos_offset;
    uint16_t distance_cm;
    uint8_t orientation;
    uint8_t status;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RGPH
// @Description: Replay Data GPS Header
struct log_RGPH {
    uint8_t num_sensors;
    uint8_t primary_sensor;
    uint8_t _end;
};

// @LoggerMessage: RGPI
// @Description: Replay Data GPS Instance, infrequently changing data
struct log_RGPI {
    Vector3f antenna_offset;
    float lag_sec;
    uint8_t have_vertical_velocity:1;
    uint8_t horizontal_accuracy_returncode:1;
    uint8_t vertical_accuracy_returncode:1;
    uint8_t get_lag_returncode:1;
    uint8_t speed_accuracy_returncode:1;
    uint8_t gps_yaw_deg_returncode:1;
    uint8_t status;
    uint8_t num_sats;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RGPJ
// @Description: Replay Data GPS Instance - rapidly changing data
struct log_RGPJ {
    uint32_t last_message_time_ms;
    Vector3f velocity;
    float sacc;
    float yaw_deg;
    float yaw_accuracy_deg;
    uint32_t yaw_deg_time_ms;
    int32_t lat;
    int32_t lng;
    int32_t alt;
    float hacc;
    float vacc;
    uint16_t hdop;
    uint8_t instance;
    uint8_t _end;
};

// Replay Data Structure - Airspeed Sensor header
struct log_RASH {
    uint8_t num_sensors;
    uint8_t primary;
    uint8_t _end;
};

// Replay Data Structure - Airspeed Sensor instance
struct log_RASI {
    float airspeed;
    uint32_t last_update_ms;
    bool healthy;
    bool use;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RMGH
// @Description: Replay Data Magnetometer Header
struct log_RMGH {
    float declination;
    bool available;
    uint8_t count;
    bool auto_declination_enabled;
    uint8_t num_enabled;
    bool learn_offsets_enabled;
    bool consistent;
    uint8_t first_usable;
    uint8_t _end;
};

// @LoggerMessage: RMGI
// @Description: Replay Data Magnetometer Instance
struct log_RMGI {
    uint32_t last_update_usec;
    Vector3f offsets;
    Vector3f field;
    bool use_for_yaw;
    bool healthy;
    bool have_scale_factor;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RBCH
// @Description: Replay Data Beacon Header
struct log_RBCH {
    Vector3f vehicle_position_ned;
    float accuracy_estimate;
    int32_t origin_lat;
    int32_t origin_lng;
    int32_t origin_alt;
    uint8_t get_vehicle_position_ned_returncode:1;
    uint8_t get_origin_returncode:1;
    uint8_t enabled:1;
    uint8_t count;
    uint8_t _end;
};

// @LoggerMessage: RBCI
// @Description: Replay Data Beacon Instance
struct log_RBCI {
    uint32_t last_update_ms;
    Vector3f position;
    float distance;
    uint8_t healthy;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RVOH
// @Description: Replay Data Visual Odometry data
struct log_RVOH {
    Vector3f pos_offset;
    uint32_t delay_ms;
    uint8_t healthy;
    bool enabled;
    uint8_t _end;
};

// @LoggerMessage: ROFH
// @Description: Replay optical flow data
struct log_ROFH {
    Vector2f rawFlowRates;
    Vector2f rawGyroRates;
    uint32_t msecFlowMeas;
    Vector3f posOffset;
    float heightOverride;
    uint8_t rawFlowQuality;
    uint8_t _end;
};

// @LoggerMessage: REPH
// @Description: Replay external position data
struct log_REPH {
    Vector3f pos;
    Quaternion quat;
    float posErr;
    float angErr;
    uint32_t timeStamp_ms;
    uint32_t resetTime_ms;
    uint16_t delay_ms;
    uint8_t _end;
};

// @LoggerMessage: REVH
// @Description: Replay external position data
struct log_REVH {
    Vector3f vel;
    float err;
    uint32_t timeStamp_ms;
    uint16_t delay_ms;
    uint8_t _end;
};

// @LoggerMessage: RWOH
// @Description: Replay wheel odometry data
struct log_RWOH {
    float delAng;
    float delTime;
    uint32_t timeStamp_ms;
    Vector3f posOffset;
    float radius;
    uint8_t _end;
};

// @LoggerMessage: RBOH
// @Description: Replay body odometry data
struct log_RBOH {
    float quality;
    Vector3f delPos;
    Vector3f delAng;
    float delTime;
    uint32_t timeStamp_ms;
    Vector3f posOffset;
    uint16_t delay_ms;
    uint8_t _end;
};

#define RLOG_SIZE(sname) 3+offsetof(struct log_ ##sname,_end)

#define LOG_STRUCTURE_FROM_DAL        \
    { LOG_RFRH_MSG, RLOG_SIZE(RFRH),                          \
      "RFRH", "QI", "TimeUS,TF", "s-", "F-" }, \
    { LOG_RFRF_MSG, RLOG_SIZE(RFRF),                          \
      "RFRF", "BB", "FTypes,Slow", "--", "--" }, \
    { LOG_RFRN_MSG, RLOG_SIZE(RFRN),                            \
      "RFRN", "IIIfIfffBBB", "HLat,HLon,HAlt,E2T,AM,TX,TY,TZ,VC,EKT,Flags", "DUm????????", "GGB--------" }, \
    { LOG_REV2_MSG, RLOG_SIZE(REV2),                                   \
      "REV2", "B", "Event", "-", "-" }, \
    { LOG_RSO2_MSG, RLOG_SIZE(RSO2),                         \
      "RSO2", "III", "Lat,Lon,Alt", "DUm", "GGB" }, \
    { LOG_RWA2_MSG, RLOG_SIZE(RWA2),                         \
      "RWA2", "ff", "Airspeed,uncertainty", "nn", "00" }, \
    { LOG_REV3_MSG, RLOG_SIZE(REV3),                \
      "REV3", "B", "Event", "-", "-" }, \
    { LOG_RSO3_MSG, RLOG_SIZE(RSO3),                         \
      "RSO3", "III", "Lat,Lon,Alt", "DUm", "GGB" }, \
    { LOG_RWA3_MSG, RLOG_SIZE(RWA3),                         \
      "RWA3", "ff", "Airspeed,Uncertainty", "nn", "00" }, \
    { LOG_REY3_MSG, RLOG_SIZE(REY3),                                   \
      "REY3", "ffIB", "yawangle,yawangleerr,timestamp_ms,type", "???-", "???-" }, \
    { LOG_RISH_MSG, RLOG_SIZE(RISH),                                   \
      "RISH", "HBBfBB", "LR,PG,PA,LD,AC,GC", "------", "------" }, \
    { LOG_RISI_MSG, RLOG_SIZE(RISI),                                   \
      "RISI", "ffffffffBB", "DVX,DVY,DVZ,DAX,DAY,DAZ,DVDT,DADT,Flags,I", "---------#", "----------" }, \
    { LOG_RASH_MSG, RLOG_SIZE(RASH),                                   \
      "RASH", "BB", "Primary,NumInst", "--", "--" },  \
    { LOG_RASI_MSG, RLOG_SIZE(RASI),                                   \
      "RASI", "fIBBB", "pd,UpdateMS,H,Use,I", "----#", "-----" }, \
    { LOG_RBRH_MSG, RLOG_SIZE(RBRH),                                   \
      "RBRH", "BB", "Primary,NumInst", "--", "--" },  \
    { LOG_RBRI_MSG, RLOG_SIZE(RBRI),                                   \
      "RBRI", "IfBB", "LastUpdate,Alt,H,I", "---#", "----" }, \
    { LOG_RRNH_MSG, RLOG_SIZE(RRNH),                                   \
      "RRNH", "hhB", "GCl,MaxD,NumSensors", "???", "???" },  \
    { LOG_RRNI_MSG, RLOG_SIZE(RRNI),                                   \
      "RRNI", "fffHBBB", "PX,PY,PZ,Dist,Orient,Status,I", "------#", "-------" }, \
    { LOG_RGPH_MSG, RLOG_SIZE(RGPH),                                   \
      "RGPH", "BB", "NumInst,Primary", "--", "--" },  \
    { LOG_RGPI_MSG, RLOG_SIZE(RGPI),                                   \
      "RGPI", "ffffBBBB", "OX,OY,OZ,Lg,Flags,Stat,NSats,I", "-------#", "--------" }, \
    { LOG_RGPJ_MSG, RLOG_SIZE(RGPJ),                                   \
      "RGPJ", "IffffffIiiiffHB", "TS,VX,VY,VZ,SA,Y,YA,YT,Lat,Lon,Alt,HA,VA,HD,I", "--------------#", "---------------" }, \
    { LOG_RMGH_MSG, RLOG_SIZE(RMGH),                                   \
      "RMGH", "fBBBBBBB", "Dec,Avail,NumInst,AutoDec,NumEna,LOE,C,FUsable", "--------", "--------" },  \
    { LOG_RMGI_MSG, RLOG_SIZE(RMGI),                                   \
      "RMGI", "IffffffBBBB", "LU,OX,OY,OZ,FX,FY,FZ,UFY,H,HSF,I", "----------#", "-----------" },                                        \
    { LOG_RBCH_MSG, RLOG_SIZE(RBCH),                                   \
      "RBCH", "ffffiiiBB", "PX,PY,PZ,AE,OLat,OLng,OAlt,Flags,NumInst", "---------", "---------" },  \
    { LOG_RBCI_MSG, RLOG_SIZE(RBCI),                                   \
      "RBCI", "IffffBB", "LU,PX,PY,PZ,Dist,H,I", "smmmm-#", "?0000--" }, \
    { LOG_RVOH_MSG, RLOG_SIZE(RVOH),                                   \
      "RVOH", "fffIBB", "OX,OY,OZ,Del,H,Ena", "------", "------" }, \
    { LOG_ROFH_MSG, RLOG_SIZE(ROFH),                                   \
      "ROFH", "ffffIffffB", "FX,FY,GX,GY,Tms,PX,PY,PZ,HgtOvr,Qual", "----------", "----------" }, \
    { LOG_REPH_MSG, RLOG_SIZE(REPH),                                   \
      "REPH", "fffffffffIIH", "PX,PY,PZ,Q1,Q2,Q3,Q4,PEr,AEr,TS,RT,D", "------------", "------------" }, \
    { LOG_REVH_MSG, RLOG_SIZE(REVH),                                   \
      "REVH", "ffffIH", "VX,VY,VZ,Er,TS,D", "------", "------" }, \
    { LOG_RWOH_MSG, RLOG_SIZE(RWOH),                                   \
      "RWOH", "ffIffff", "DA,DT,TS,PX,PY,PZ,R", "-------", "-------" }, \
    { LOG_RBOH_MSG, RLOG_SIZE(RBOH),                                   \
      "RBOH", "ffffffffIfffH", "Q,DPX,DPY,DPZ,DAX,DAY,DAZ,DT,TS,OX,OY,OZ,D", "-------------", "-------------" },
