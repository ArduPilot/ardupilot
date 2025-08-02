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
    LOG_RSLL_MSG, \
    LOG_REVH_MSG, \
    LOG_RWOH_MSG, \
    LOG_RBOH_MSG, \
    LOG_RTER_MSG

// @LoggerMessage: RFRH
// @Description: Replay FRame Header
// @Field: TimeUS: Time since system startup
// @Field: TF: Time flying
struct log_RFRH {
    uint64_t time_us;
    uint32_t time_flying_ms;
    uint8_t _end;
};

// @LoggerMessage: RFRF
// @Description: Replay FRame data - Finished frame
// @Field: FTypes: accumulated method calls made during frame
// @FieldBitmaskEnum: FTypes: AP_DAL::FrameType
// @Field: Slow: true if we are not keeping up with IMU loop rate
struct log_RFRF {
    uint8_t frame_types;
    uint8_t core_slow;
    uint8_t _end;
};

// @LoggerMessage: RFRN
// @Description: Replay FRame - aNother frame header
// @Field: HLat: home latitude
// @Field: HLon: home latitude
// @Field: HAlt: home altitude AMSL
// @Field: E2T: EAS to TAS factor
// @Field: AM: available memory
// @Field: TX: AHRS trim X
// @Field: TY: AHRS trim Y
// @Field: TZ: AHRS trim Z
// @Field: VC: AHRS Vehicle Class
// @Field: EKT: configured EKF type
// @FieldValueEnum: EKT: AP_DAL::EKFType
// @Field: Flags: bitmask of boolean state
// @FieldBitmaskEnum: Flags: AP_DAL::RFRNFlags
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

// @LoggerMessage: RISH
// @Description: Replay Inertial Sensor header
// @Field: LR: INS loop rate
// @Field: PG: primary gyro index
// @Field: PA: primary accel index
// @Field: LD: INS loop-delta-t
// @Field: AC: accel count
// @Field: GC: gyro count
struct log_RISH {
    uint16_t loop_rate_hz;
    uint8_t first_usable_gyro;
    uint8_t first_usable_accel;
    float loop_delta_t;
    uint8_t accel_count;
    uint8_t gyro_count;
    uint8_t _end;
};

// @LoggerMessage: RISI
// @Description: Replay Inertial Sensor instance data
// @Field: DVX: x-axis delta-velocity
// @Field: DVY: y-axis delta-velocity
// @Field: DVZ: z-axis delta-velocity
// @Field: DAX: x-axis delta-angle
// @Field: DAY: y-axis delta-angle
// @Field: DAZ: z-axis delta-angle
// @Field: DVDT: delta-velocity-delta-time
// @Field: DADT: delta-angle-delta-time
// @Field: Flags: use-accel, use-gyro, delta-vel-valid, delta-accel-valid
// @Field: I: IMU instance
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
// @Description: Replay Event (EKF2)
// @Field: Event: external event injected into EKF
// @FieldValueEnum: Event: AP_DAL::Event
struct log_REV2 {
    uint8_t event;
    uint8_t _end;
};

// @LoggerMessage: RSO2
// @Description: Replay Set Origin event (EKF2)
// @Field: Lat: origin latitude
// @Field: Lon: origin longitude
// @Field: Alt: origin altitude
struct log_RSO2 {
    int32_t lat;
    int32_t lng;
    int32_t alt;
    uint8_t _end;
};

// @LoggerMessage: RWA2
// @Description: Replay set-default-airspeed event (EKF2)
// @Field: Airspeed: default airspeed
// @Field: uncertainty: uncertainty in default airspeed
struct log_RWA2 {
    float airspeed;
    float uncertainty;
    uint8_t _end;
};

// same structures for EKF3
// @LoggerMessage: REV3
// @Description: Replay Event (EKF3)
// @Field: Event: external event injected into EKF
// @FieldValueEnum: Event: AP_DAL::Event
#define log_REV3 log_REV2

// @LoggerMessage: RSO3
// @Description: Replay Set Origin event (EKF3)
// @Field: Lat: origin latitude
// @Field: Lon: origin longitude
// @Field: Alt: origin altitude
#define log_RSO3 log_RSO2

// @LoggerMessage: RWA3
// @Description: Replay set-default-airspeed event (EKF3)
// @Field: Airspeed: default airspeed
// @Field: Uncertainty: uncertainty in default airspeed
#define log_RWA3 log_RWA2

// @LoggerMessage: REY3
// @Description: Replay Euler Yaw event
// @Field: yawangle: externally supplied yaw angle
// @Field: yawangleerr: error in externally supplied yaw angle
// @Field: timestamp_ms: timestamp associated with yaw angle and yaw angle error
// @Field: type: number that needs documenting
struct log_REY3 {
    float yawangle;
    float yawangleerr;
    uint32_t timestamp_ms;
    uint8_t type;
    uint8_t _end;
};

// @LoggerMessage: RBRH
// @Description: Replay Data Barometer Header
// @Field: Primary: primary barometer instance number
// @Field: NumInst: number of barometer sensors
struct log_RBRH {
    uint8_t primary;
    uint8_t num_instances;
    uint8_t _end;
};

// @LoggerMessage: RBRI
// @Description: Replay Data Barometer Instance
// @Field: LastUpdate: timestamp of barometer data
// @Field: Alt: barometer altitude estimate
// @Field: H: barometer sensor health indication
// @Field: I: barometer instance number
struct log_RBRI {
    uint32_t last_update_ms;
    float altitude;  // from get_altitude
    bool healthy;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RRNH
// @Description: Replay Data Rangefinder Header
// @Field: GCl: rangefinder ground clearance for downward-facing rangefinders
// @Field: MaxD: rangefinder maximum distance for downward-facing rangefinders
// @Field: NumSensors: number of rangefinder instances
struct log_RRNH {
    // this is rotation-pitch-270!
    float ground_clearance;
    float max_distance;
    uint8_t num_sensors;
    uint8_t _end;
};

// @LoggerMessage: RRNI
// @Description: Replay Data Rangefinder Instance
// @Field: PX: rangefinder body-frame offset, X-axis
// @Field: PY: rangefinder body-frame offset, Y-axis
// @Field: PZ: rangefinder body-frame offset, Z-axis
// @Field: Dist: Measured rangefinder distance
// @Field: Orient: orientation
// @Field: Status: status
// @Field: I: rangefinder instance number
struct log_RRNI {
    Vector3f pos_offset;
    float distance;
    uint8_t orientation;
    uint8_t status;
    uint8_t instance;
    uint8_t _end;
};

// @LoggerMessage: RGPH
// @Description: Replay Data GPS Header
// @Field: NumInst: number of GPS sensors
// @Field: Primary: instance number of primary sensor
struct log_RGPH {
    uint8_t num_sensors;
    uint8_t primary_sensor;
    uint8_t _end;
};

// @LoggerMessage: RGPI
// @Description: Replay Data GPS Instance, infrequently changing data
// @Field: OX: antenna body-frame offset, X-axis
// @Field: OY: antenna body-frame offset, Y-axis
// @Field: OZ: antenna body-frame offset, Z-axis
// @Field: Lg: GPS time lag
// @Field: Flags: various GPS flags
// @FieldBits: Flags: have_vertical_velocity,horizontal_accuracy_returncode,vertical_accuracy_returncode,get_lag_returncode,speed_accuracy_returncode,gps_yaw_deg_returncode
// @Field: Stat: GPS fix status
// @Field: NSats: number of satellites GPS is using
// @Field: I: GPS sensor instance number
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
// @Field: TS: GPS data timestamp
// @Field: VX: GPS velocity, North
// @Field: VY: GPS velocity, East
// @Field: VZ: GPS velocity, Down
// @Field: SA: speed accuracy
// @Field: Y: GPS yaw
// @Field: YA: GPS yaw accuracy
// @Field: YT: timestamp of GPS yaw estimate
// @Field: Lat: latitude
// @Field: Lon: longitude
// @Field: Alt: altitude
// @Field: HA: horizontal accuracy
// @Field: VA: vertical accuracy
// @Field: HD: HDOP
// @Field: I: GPS sensor instance number
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

// @LoggerMessage: RASH
// @Description: Replay Airspeed Sensor Header
// @Field: Primary: airspeed instance number
// @Field: NumInst: number of airspeed instances
struct log_RASH {
    uint8_t num_sensors;
    uint8_t primary;
    uint8_t _end;
};

// @LoggerMessage: RASI
// @Description: Replay Airspeed Sensor Instance data
// @Field: pd: measured airspeed
// @Field: UpdateMS: timestamp of measured airspeed
// @Field: H: indicator of airspeed sensor health
// @Field: Use: true if airspeed is configured to be used
// @Field: I: airspeed instance number
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
// @Field: Dec: vehicle declination
// @Field: Avail: true if the compass library is marking itself as available
// @Field: NumInst: number of compass instances
// @Field: AutoDec: true if compass autodeclination is enabled
// @Field: NumEna: number of enabled compass instances
// @Field: LOE: true if compass learning of offsets is enabled
// @Field: C: true if compasses are consistent
// @Field: FUsable: index of first usable compass
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
// @Field: LU: last update time for magnetometer data
// @Field: OX: mag sensor offset, X-axis
// @Field: OY: mag sensor offset, Y-axis
// @Field: OZ: mag sensor offset, Z-axis
// @Field: FX: field strength, X-axis
// @Field: FY: field strength, Y-axis
// @Field: FZ: field strength, Z-axis
// @Field: UFY: true if compass is being used for yaw
// @Field: H: sensor health
// @Field: HSF: compass has scale factor
// @Field: I: magnetometer instance number
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
// @Field: PX: zero, unused
// @Field: PY: zero, unused
// @Field: PZ: zero, unused
// @Field: AE: zero, unused
// @Field: OLat: origin latitude
// @Field: OLng: origin longitude
// @Field: OAlt: origin altitude
// @Field: Flags: vehicle_position_ned_returncode,get_origin_returncode,enabled
// @Field: NumInst: number of beacons
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
// @Field: LU: last update from this beacon instance
// @Field: PX: beacon distance from origin, X-axis
// @Field: PY:  beacon distance from origin, Y-axis
// @Field: PZ:  beacon distance from origin, Z-axis
// @Field: Dist: distance to beacon
// @Field: H: beacon data health
// @Field: I: beacon instance number
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
// @Field: OX: offset, x-axis
// @Field: OY: offset, y-axis
// @Field: OZ: offset, z-axis
// @Field: Del: data delay
// @Field: H: sensor health
// @Field: Ena: sensor enabled
struct log_RVOH {
    Vector3f pos_offset;
    uint32_t delay_ms;
    uint8_t healthy;
    bool enabled;
    uint8_t _end;
};

// @LoggerMessage: ROFH
// @Description: Replay optical flow data
// @Field: FX: raw flow rate, X-axis
// @Field: FY: raw flow rate, Y-axis
// @Field: GX: gyro rate, X-axis
// @Field: GY: gyro rate, Y-axis
// @Field: Tms: measurement timestamp
// @Field: PX:gyro rate, X-axis
// @Field: PY: body-frame offset, Y-axis
// @Field: PZ: body-frame offset, Z-axis
// @Field: HgtOvr: sensor height override
// @Field: Qual: flow quality measurement
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
// @Field: PX: external position estimate, X-axis
// @Field: PY: external position estimate, Y-axis
// @Field: PZ: external position estimate, Z-axis
// @Field: Q1: external attitude quaternion
// @Field: Q2: external attitude quaternion
// @Field: Q3: external attitude quaternion
// @Field: Q4: external attitude quaternion
// @Field: PEr: external position error estimate
// @Field: AEr: external attitude error estimate
// @Field: TS: timestamp on external error estimate
// @Field: RT: timestamp of last external reset
// @Field: D: delay on external data
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

// @LoggerMessage: RSLL
// @Description: Replay Set Lat Lng event
// @Field: Lat: latitude
// @Field: Lng: longitude
// @Field: PosAccSD: position accuracy, 1-StD
// @Field: TS: timestamp of latitude/longitude
struct log_RSLL {
    int32_t lat; // WGS-84 latitude in 1E-7 degrees
    int32_t lng; // WGS-84 longitude in 1E7 degrees
    float posAccSD; // horizontal position 1 STD uncertainty (m)
    uint32_t timestamp_ms;
    uint8_t _end;
};

// @LoggerMessage: REVH
// @Description: Replay external velocity data
// @Field: VX: external velocity estimate, X-axis
// @Field: VY: external velocity estimate, Y-axis
// @Field: VZ: external velocity estimate, Z-axis
// @Field: Er: error in velocity estimate
// @Field: TS: timestamp of velocity estimate
// @Field: D: delay in external velocity data
struct log_REVH {
    Vector3f vel;
    float err;
    uint32_t timeStamp_ms;
    uint16_t delay_ms;
    uint8_t _end;
};

// @LoggerMessage: RWOH
// @Description: Replay wheel odometry data
// @Field: DA: delta-angle
// @Field: DT: delta-time
// @Field: TS: data timestamp
// @Field: PX: sensor body-frame offset, x-axis
// @Field: PY: sensor body-frame offset, y-axis
// @Field: PZ: sensor body-frame offset, z-axis
// @Field: R: wheel radius
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
// @Field: Q: data quality measure
// @Field: DPX: delta-position-X
// @Field: DPY: delta-position-Y
// @Field: DPZ: delta-position-Z
// @Field: DAX: delta-angle-X
// @Field: DAY: delta-angle-Y
// @Field: DAZ: delta-angle-Z
// @Field: DT: delta-time
// @Field: TS: data timestamp
// @Field: OX: zero, unused
// @Field: OY: zero, unused
// @Field: OZ: zero, unused
// @Field: D: zero, unused
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

// @LoggerMessage: RTER
// @Description: Replay Terrain SRTM Altitude
// @Field: Alt: altitude above origin in meters
struct log_RTER {
    float alt_m;
    uint8_t _end;
};

#define RLOG_SIZE(sname) 3+offsetof(struct log_ ##sname,_end)

#define LOG_STRUCTURE_FROM_DAL        \
    { LOG_RFRH_MSG, RLOG_SIZE(RFRH),                          \
      "RFRH", "QI", "TimeUS,TF", "s-", "F-" }, \
    { LOG_RFRF_MSG, RLOG_SIZE(RFRF),                          \
      "RFRF", "BB", "FTypes,Slow", "--", "--" }, \
    { LOG_RFRN_MSG, RLOG_SIZE(RFRN),                            \
      "RFRN", "IIIfIfffBBB", "HLat,HLon,HAlt,E2T,AM,TX,TY,TZ,VC,EKT,Flags", "DUm-bddd---", "GGB--------" }, \
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
      "RRNH", "ffB", "GCl,MaxD,NumSensors", "mm-", "00-" },  \
    { LOG_RRNI_MSG, RLOG_SIZE(RRNI),                                   \
      "RRNI", "ffffBBB", "PX,PY,PZ,Dist,Orient,Status,I", "---m--#", "---0---" }, \
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
    { LOG_RSLL_MSG, RLOG_SIZE(RSLL),                         \
      "RSLL", "IIfI", "Lat,Lng,PosAccSD,TS", "DU--", "GG--" }, \
    { LOG_REVH_MSG, RLOG_SIZE(REVH),                                   \
      "REVH", "ffffIH", "VX,VY,VZ,Er,TS,D", "------", "------" }, \
    { LOG_RWOH_MSG, RLOG_SIZE(RWOH),                                   \
      "RWOH", "ffIffff", "DA,DT,TS,PX,PY,PZ,R", "-------", "-------" }, \
    { LOG_RBOH_MSG, RLOG_SIZE(RBOH),                                   \
      "RBOH", "ffffffffIfffH", "Q,DPX,DPY,DPZ,DAX,DAY,DAZ,DT,TS,OX,OY,OZ,D", "-------------", "-------------" }, \
    { LOG_RTER_MSG, RLOG_SIZE(RTER),                                   \
      "RTER", "f", "Alt", "m", "0" },
