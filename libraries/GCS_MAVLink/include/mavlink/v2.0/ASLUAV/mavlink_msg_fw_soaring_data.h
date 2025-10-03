#pragma once
// MESSAGE FW_SOARING_DATA PACKING

#define MAVLINK_MSG_ID_FW_SOARING_DATA 8011


typedef struct __mavlink_fw_soaring_data_t {
 uint64_t timestamp; /*< [ms] Timestamp*/
 uint64_t timestampModeChanged; /*< [ms] Timestamp since last mode change*/
 float xW; /*< [m/s] Thermal core updraft strength*/
 float xR; /*< [m] Thermal radius*/
 float xLat; /*< [deg] Thermal center latitude*/
 float xLon; /*< [deg] Thermal center longitude*/
 float VarW; /*<  Variance W*/
 float VarR; /*<  Variance R*/
 float VarLat; /*<  Variance Lat*/
 float VarLon; /*<  Variance Lon */
 float LoiterRadius; /*< [m] Suggested loiter radius*/
 float LoiterDirection; /*<  Suggested loiter direction*/
 float DistToSoarPoint; /*< [m] Distance to soar point*/
 float vSinkExp; /*< [m/s] Expected sink rate at current airspeed, roll and throttle*/
 float z1_LocalUpdraftSpeed; /*< [m/s] Measurement / updraft speed at current/local airplane position*/
 float z2_DeltaRoll; /*< [deg] Measurement / roll angle tracking error*/
 float z1_exp; /*<  Expected measurement 1*/
 float z2_exp; /*<  Expected measurement 2*/
 float ThermalGSNorth; /*< [m/s] Thermal drift (from estimator prediction step only)*/
 float ThermalGSEast; /*< [m/s] Thermal drift (from estimator prediction step only)*/
 float TSE_dot; /*< [m/s]  Total specific energy change (filtered)*/
 float DebugVar1; /*<   Debug variable 1*/
 float DebugVar2; /*<   Debug variable 2*/
 uint8_t ControlMode; /*<  Control Mode [-]*/
 uint8_t valid; /*<  Data valid [-]*/
} mavlink_fw_soaring_data_t;

#define MAVLINK_MSG_ID_FW_SOARING_DATA_LEN 102
#define MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN 102
#define MAVLINK_MSG_ID_8011_LEN 102
#define MAVLINK_MSG_ID_8011_MIN_LEN 102

#define MAVLINK_MSG_ID_FW_SOARING_DATA_CRC 20
#define MAVLINK_MSG_ID_8011_CRC 20



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FW_SOARING_DATA { \
    8011, \
    "FW_SOARING_DATA", \
    25, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_fw_soaring_data_t, timestamp) }, \
         { "timestampModeChanged", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_fw_soaring_data_t, timestampModeChanged) }, \
         { "xW", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_fw_soaring_data_t, xW) }, \
         { "xR", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_fw_soaring_data_t, xR) }, \
         { "xLat", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_fw_soaring_data_t, xLat) }, \
         { "xLon", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_fw_soaring_data_t, xLon) }, \
         { "VarW", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_fw_soaring_data_t, VarW) }, \
         { "VarR", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_fw_soaring_data_t, VarR) }, \
         { "VarLat", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_fw_soaring_data_t, VarLat) }, \
         { "VarLon", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_fw_soaring_data_t, VarLon) }, \
         { "LoiterRadius", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_fw_soaring_data_t, LoiterRadius) }, \
         { "LoiterDirection", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_fw_soaring_data_t, LoiterDirection) }, \
         { "DistToSoarPoint", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_fw_soaring_data_t, DistToSoarPoint) }, \
         { "vSinkExp", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_fw_soaring_data_t, vSinkExp) }, \
         { "z1_LocalUpdraftSpeed", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_fw_soaring_data_t, z1_LocalUpdraftSpeed) }, \
         { "z2_DeltaRoll", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_fw_soaring_data_t, z2_DeltaRoll) }, \
         { "z1_exp", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_fw_soaring_data_t, z1_exp) }, \
         { "z2_exp", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_fw_soaring_data_t, z2_exp) }, \
         { "ThermalGSNorth", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_fw_soaring_data_t, ThermalGSNorth) }, \
         { "ThermalGSEast", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_fw_soaring_data_t, ThermalGSEast) }, \
         { "TSE_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_fw_soaring_data_t, TSE_dot) }, \
         { "DebugVar1", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_fw_soaring_data_t, DebugVar1) }, \
         { "DebugVar2", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_fw_soaring_data_t, DebugVar2) }, \
         { "ControlMode", NULL, MAVLINK_TYPE_UINT8_T, 0, 100, offsetof(mavlink_fw_soaring_data_t, ControlMode) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 101, offsetof(mavlink_fw_soaring_data_t, valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FW_SOARING_DATA { \
    "FW_SOARING_DATA", \
    25, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_fw_soaring_data_t, timestamp) }, \
         { "timestampModeChanged", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_fw_soaring_data_t, timestampModeChanged) }, \
         { "xW", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_fw_soaring_data_t, xW) }, \
         { "xR", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_fw_soaring_data_t, xR) }, \
         { "xLat", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_fw_soaring_data_t, xLat) }, \
         { "xLon", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_fw_soaring_data_t, xLon) }, \
         { "VarW", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_fw_soaring_data_t, VarW) }, \
         { "VarR", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_fw_soaring_data_t, VarR) }, \
         { "VarLat", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_fw_soaring_data_t, VarLat) }, \
         { "VarLon", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_fw_soaring_data_t, VarLon) }, \
         { "LoiterRadius", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_fw_soaring_data_t, LoiterRadius) }, \
         { "LoiterDirection", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_fw_soaring_data_t, LoiterDirection) }, \
         { "DistToSoarPoint", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_fw_soaring_data_t, DistToSoarPoint) }, \
         { "vSinkExp", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_fw_soaring_data_t, vSinkExp) }, \
         { "z1_LocalUpdraftSpeed", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_fw_soaring_data_t, z1_LocalUpdraftSpeed) }, \
         { "z2_DeltaRoll", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_fw_soaring_data_t, z2_DeltaRoll) }, \
         { "z1_exp", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_fw_soaring_data_t, z1_exp) }, \
         { "z2_exp", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_fw_soaring_data_t, z2_exp) }, \
         { "ThermalGSNorth", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_fw_soaring_data_t, ThermalGSNorth) }, \
         { "ThermalGSEast", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_fw_soaring_data_t, ThermalGSEast) }, \
         { "TSE_dot", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_fw_soaring_data_t, TSE_dot) }, \
         { "DebugVar1", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_fw_soaring_data_t, DebugVar1) }, \
         { "DebugVar2", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_fw_soaring_data_t, DebugVar2) }, \
         { "ControlMode", NULL, MAVLINK_TYPE_UINT8_T, 0, 100, offsetof(mavlink_fw_soaring_data_t, ControlMode) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 101, offsetof(mavlink_fw_soaring_data_t, valid) }, \
         } \
}
#endif

/**
 * @brief Pack a fw_soaring_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp
 * @param timestampModeChanged [ms] Timestamp since last mode change
 * @param xW [m/s] Thermal core updraft strength
 * @param xR [m] Thermal radius
 * @param xLat [deg] Thermal center latitude
 * @param xLon [deg] Thermal center longitude
 * @param VarW  Variance W
 * @param VarR  Variance R
 * @param VarLat  Variance Lat
 * @param VarLon  Variance Lon 
 * @param LoiterRadius [m] Suggested loiter radius
 * @param LoiterDirection  Suggested loiter direction
 * @param DistToSoarPoint [m] Distance to soar point
 * @param vSinkExp [m/s] Expected sink rate at current airspeed, roll and throttle
 * @param z1_LocalUpdraftSpeed [m/s] Measurement / updraft speed at current/local airplane position
 * @param z2_DeltaRoll [deg] Measurement / roll angle tracking error
 * @param z1_exp  Expected measurement 1
 * @param z2_exp  Expected measurement 2
 * @param ThermalGSNorth [m/s] Thermal drift (from estimator prediction step only)
 * @param ThermalGSEast [m/s] Thermal drift (from estimator prediction step only)
 * @param TSE_dot [m/s]  Total specific energy change (filtered)
 * @param DebugVar1   Debug variable 1
 * @param DebugVar2   Debug variable 2
 * @param ControlMode  Control Mode [-]
 * @param valid  Data valid [-]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fw_soaring_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t timestampModeChanged, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, float LoiterDirection, float DistToSoarPoint, float vSinkExp, float z1_LocalUpdraftSpeed, float z2_DeltaRoll, float z1_exp, float z2_exp, float ThermalGSNorth, float ThermalGSEast, float TSE_dot, float DebugVar1, float DebugVar2, uint8_t ControlMode, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FW_SOARING_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, timestampModeChanged);
    _mav_put_float(buf, 16, xW);
    _mav_put_float(buf, 20, xR);
    _mav_put_float(buf, 24, xLat);
    _mav_put_float(buf, 28, xLon);
    _mav_put_float(buf, 32, VarW);
    _mav_put_float(buf, 36, VarR);
    _mav_put_float(buf, 40, VarLat);
    _mav_put_float(buf, 44, VarLon);
    _mav_put_float(buf, 48, LoiterRadius);
    _mav_put_float(buf, 52, LoiterDirection);
    _mav_put_float(buf, 56, DistToSoarPoint);
    _mav_put_float(buf, 60, vSinkExp);
    _mav_put_float(buf, 64, z1_LocalUpdraftSpeed);
    _mav_put_float(buf, 68, z2_DeltaRoll);
    _mav_put_float(buf, 72, z1_exp);
    _mav_put_float(buf, 76, z2_exp);
    _mav_put_float(buf, 80, ThermalGSNorth);
    _mav_put_float(buf, 84, ThermalGSEast);
    _mav_put_float(buf, 88, TSE_dot);
    _mav_put_float(buf, 92, DebugVar1);
    _mav_put_float(buf, 96, DebugVar2);
    _mav_put_uint8_t(buf, 100, ControlMode);
    _mav_put_uint8_t(buf, 101, valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#else
    mavlink_fw_soaring_data_t packet;
    packet.timestamp = timestamp;
    packet.timestampModeChanged = timestampModeChanged;
    packet.xW = xW;
    packet.xR = xR;
    packet.xLat = xLat;
    packet.xLon = xLon;
    packet.VarW = VarW;
    packet.VarR = VarR;
    packet.VarLat = VarLat;
    packet.VarLon = VarLon;
    packet.LoiterRadius = LoiterRadius;
    packet.LoiterDirection = LoiterDirection;
    packet.DistToSoarPoint = DistToSoarPoint;
    packet.vSinkExp = vSinkExp;
    packet.z1_LocalUpdraftSpeed = z1_LocalUpdraftSpeed;
    packet.z2_DeltaRoll = z2_DeltaRoll;
    packet.z1_exp = z1_exp;
    packet.z2_exp = z2_exp;
    packet.ThermalGSNorth = ThermalGSNorth;
    packet.ThermalGSEast = ThermalGSEast;
    packet.TSE_dot = TSE_dot;
    packet.DebugVar1 = DebugVar1;
    packet.DebugVar2 = DebugVar2;
    packet.ControlMode = ControlMode;
    packet.valid = valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FW_SOARING_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
}

/**
 * @brief Pack a fw_soaring_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp
 * @param timestampModeChanged [ms] Timestamp since last mode change
 * @param xW [m/s] Thermal core updraft strength
 * @param xR [m] Thermal radius
 * @param xLat [deg] Thermal center latitude
 * @param xLon [deg] Thermal center longitude
 * @param VarW  Variance W
 * @param VarR  Variance R
 * @param VarLat  Variance Lat
 * @param VarLon  Variance Lon 
 * @param LoiterRadius [m] Suggested loiter radius
 * @param LoiterDirection  Suggested loiter direction
 * @param DistToSoarPoint [m] Distance to soar point
 * @param vSinkExp [m/s] Expected sink rate at current airspeed, roll and throttle
 * @param z1_LocalUpdraftSpeed [m/s] Measurement / updraft speed at current/local airplane position
 * @param z2_DeltaRoll [deg] Measurement / roll angle tracking error
 * @param z1_exp  Expected measurement 1
 * @param z2_exp  Expected measurement 2
 * @param ThermalGSNorth [m/s] Thermal drift (from estimator prediction step only)
 * @param ThermalGSEast [m/s] Thermal drift (from estimator prediction step only)
 * @param TSE_dot [m/s]  Total specific energy change (filtered)
 * @param DebugVar1   Debug variable 1
 * @param DebugVar2   Debug variable 2
 * @param ControlMode  Control Mode [-]
 * @param valid  Data valid [-]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fw_soaring_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t timestampModeChanged, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, float LoiterDirection, float DistToSoarPoint, float vSinkExp, float z1_LocalUpdraftSpeed, float z2_DeltaRoll, float z1_exp, float z2_exp, float ThermalGSNorth, float ThermalGSEast, float TSE_dot, float DebugVar1, float DebugVar2, uint8_t ControlMode, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FW_SOARING_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, timestampModeChanged);
    _mav_put_float(buf, 16, xW);
    _mav_put_float(buf, 20, xR);
    _mav_put_float(buf, 24, xLat);
    _mav_put_float(buf, 28, xLon);
    _mav_put_float(buf, 32, VarW);
    _mav_put_float(buf, 36, VarR);
    _mav_put_float(buf, 40, VarLat);
    _mav_put_float(buf, 44, VarLon);
    _mav_put_float(buf, 48, LoiterRadius);
    _mav_put_float(buf, 52, LoiterDirection);
    _mav_put_float(buf, 56, DistToSoarPoint);
    _mav_put_float(buf, 60, vSinkExp);
    _mav_put_float(buf, 64, z1_LocalUpdraftSpeed);
    _mav_put_float(buf, 68, z2_DeltaRoll);
    _mav_put_float(buf, 72, z1_exp);
    _mav_put_float(buf, 76, z2_exp);
    _mav_put_float(buf, 80, ThermalGSNorth);
    _mav_put_float(buf, 84, ThermalGSEast);
    _mav_put_float(buf, 88, TSE_dot);
    _mav_put_float(buf, 92, DebugVar1);
    _mav_put_float(buf, 96, DebugVar2);
    _mav_put_uint8_t(buf, 100, ControlMode);
    _mav_put_uint8_t(buf, 101, valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#else
    mavlink_fw_soaring_data_t packet;
    packet.timestamp = timestamp;
    packet.timestampModeChanged = timestampModeChanged;
    packet.xW = xW;
    packet.xR = xR;
    packet.xLat = xLat;
    packet.xLon = xLon;
    packet.VarW = VarW;
    packet.VarR = VarR;
    packet.VarLat = VarLat;
    packet.VarLon = VarLon;
    packet.LoiterRadius = LoiterRadius;
    packet.LoiterDirection = LoiterDirection;
    packet.DistToSoarPoint = DistToSoarPoint;
    packet.vSinkExp = vSinkExp;
    packet.z1_LocalUpdraftSpeed = z1_LocalUpdraftSpeed;
    packet.z2_DeltaRoll = z2_DeltaRoll;
    packet.z1_exp = z1_exp;
    packet.z2_exp = z2_exp;
    packet.ThermalGSNorth = ThermalGSNorth;
    packet.ThermalGSEast = ThermalGSEast;
    packet.TSE_dot = TSE_dot;
    packet.DebugVar1 = DebugVar1;
    packet.DebugVar2 = DebugVar2;
    packet.ControlMode = ControlMode;
    packet.valid = valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FW_SOARING_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif
}

/**
 * @brief Pack a fw_soaring_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] Timestamp
 * @param timestampModeChanged [ms] Timestamp since last mode change
 * @param xW [m/s] Thermal core updraft strength
 * @param xR [m] Thermal radius
 * @param xLat [deg] Thermal center latitude
 * @param xLon [deg] Thermal center longitude
 * @param VarW  Variance W
 * @param VarR  Variance R
 * @param VarLat  Variance Lat
 * @param VarLon  Variance Lon 
 * @param LoiterRadius [m] Suggested loiter radius
 * @param LoiterDirection  Suggested loiter direction
 * @param DistToSoarPoint [m] Distance to soar point
 * @param vSinkExp [m/s] Expected sink rate at current airspeed, roll and throttle
 * @param z1_LocalUpdraftSpeed [m/s] Measurement / updraft speed at current/local airplane position
 * @param z2_DeltaRoll [deg] Measurement / roll angle tracking error
 * @param z1_exp  Expected measurement 1
 * @param z2_exp  Expected measurement 2
 * @param ThermalGSNorth [m/s] Thermal drift (from estimator prediction step only)
 * @param ThermalGSEast [m/s] Thermal drift (from estimator prediction step only)
 * @param TSE_dot [m/s]  Total specific energy change (filtered)
 * @param DebugVar1   Debug variable 1
 * @param DebugVar2   Debug variable 2
 * @param ControlMode  Control Mode [-]
 * @param valid  Data valid [-]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fw_soaring_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint64_t timestampModeChanged,float xW,float xR,float xLat,float xLon,float VarW,float VarR,float VarLat,float VarLon,float LoiterRadius,float LoiterDirection,float DistToSoarPoint,float vSinkExp,float z1_LocalUpdraftSpeed,float z2_DeltaRoll,float z1_exp,float z2_exp,float ThermalGSNorth,float ThermalGSEast,float TSE_dot,float DebugVar1,float DebugVar2,uint8_t ControlMode,uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FW_SOARING_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, timestampModeChanged);
    _mav_put_float(buf, 16, xW);
    _mav_put_float(buf, 20, xR);
    _mav_put_float(buf, 24, xLat);
    _mav_put_float(buf, 28, xLon);
    _mav_put_float(buf, 32, VarW);
    _mav_put_float(buf, 36, VarR);
    _mav_put_float(buf, 40, VarLat);
    _mav_put_float(buf, 44, VarLon);
    _mav_put_float(buf, 48, LoiterRadius);
    _mav_put_float(buf, 52, LoiterDirection);
    _mav_put_float(buf, 56, DistToSoarPoint);
    _mav_put_float(buf, 60, vSinkExp);
    _mav_put_float(buf, 64, z1_LocalUpdraftSpeed);
    _mav_put_float(buf, 68, z2_DeltaRoll);
    _mav_put_float(buf, 72, z1_exp);
    _mav_put_float(buf, 76, z2_exp);
    _mav_put_float(buf, 80, ThermalGSNorth);
    _mav_put_float(buf, 84, ThermalGSEast);
    _mav_put_float(buf, 88, TSE_dot);
    _mav_put_float(buf, 92, DebugVar1);
    _mav_put_float(buf, 96, DebugVar2);
    _mav_put_uint8_t(buf, 100, ControlMode);
    _mav_put_uint8_t(buf, 101, valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#else
    mavlink_fw_soaring_data_t packet;
    packet.timestamp = timestamp;
    packet.timestampModeChanged = timestampModeChanged;
    packet.xW = xW;
    packet.xR = xR;
    packet.xLat = xLat;
    packet.xLon = xLon;
    packet.VarW = VarW;
    packet.VarR = VarR;
    packet.VarLat = VarLat;
    packet.VarLon = VarLon;
    packet.LoiterRadius = LoiterRadius;
    packet.LoiterDirection = LoiterDirection;
    packet.DistToSoarPoint = DistToSoarPoint;
    packet.vSinkExp = vSinkExp;
    packet.z1_LocalUpdraftSpeed = z1_LocalUpdraftSpeed;
    packet.z2_DeltaRoll = z2_DeltaRoll;
    packet.z1_exp = z1_exp;
    packet.z2_exp = z2_exp;
    packet.ThermalGSNorth = ThermalGSNorth;
    packet.ThermalGSEast = ThermalGSEast;
    packet.TSE_dot = TSE_dot;
    packet.DebugVar1 = DebugVar1;
    packet.DebugVar2 = DebugVar2;
    packet.ControlMode = ControlMode;
    packet.valid = valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FW_SOARING_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
}

/**
 * @brief Encode a fw_soaring_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fw_soaring_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fw_soaring_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fw_soaring_data_t* fw_soaring_data)
{
    return mavlink_msg_fw_soaring_data_pack(system_id, component_id, msg, fw_soaring_data->timestamp, fw_soaring_data->timestampModeChanged, fw_soaring_data->xW, fw_soaring_data->xR, fw_soaring_data->xLat, fw_soaring_data->xLon, fw_soaring_data->VarW, fw_soaring_data->VarR, fw_soaring_data->VarLat, fw_soaring_data->VarLon, fw_soaring_data->LoiterRadius, fw_soaring_data->LoiterDirection, fw_soaring_data->DistToSoarPoint, fw_soaring_data->vSinkExp, fw_soaring_data->z1_LocalUpdraftSpeed, fw_soaring_data->z2_DeltaRoll, fw_soaring_data->z1_exp, fw_soaring_data->z2_exp, fw_soaring_data->ThermalGSNorth, fw_soaring_data->ThermalGSEast, fw_soaring_data->TSE_dot, fw_soaring_data->DebugVar1, fw_soaring_data->DebugVar2, fw_soaring_data->ControlMode, fw_soaring_data->valid);
}

/**
 * @brief Encode a fw_soaring_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fw_soaring_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fw_soaring_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fw_soaring_data_t* fw_soaring_data)
{
    return mavlink_msg_fw_soaring_data_pack_chan(system_id, component_id, chan, msg, fw_soaring_data->timestamp, fw_soaring_data->timestampModeChanged, fw_soaring_data->xW, fw_soaring_data->xR, fw_soaring_data->xLat, fw_soaring_data->xLon, fw_soaring_data->VarW, fw_soaring_data->VarR, fw_soaring_data->VarLat, fw_soaring_data->VarLon, fw_soaring_data->LoiterRadius, fw_soaring_data->LoiterDirection, fw_soaring_data->DistToSoarPoint, fw_soaring_data->vSinkExp, fw_soaring_data->z1_LocalUpdraftSpeed, fw_soaring_data->z2_DeltaRoll, fw_soaring_data->z1_exp, fw_soaring_data->z2_exp, fw_soaring_data->ThermalGSNorth, fw_soaring_data->ThermalGSEast, fw_soaring_data->TSE_dot, fw_soaring_data->DebugVar1, fw_soaring_data->DebugVar2, fw_soaring_data->ControlMode, fw_soaring_data->valid);
}

/**
 * @brief Encode a fw_soaring_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param fw_soaring_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fw_soaring_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_fw_soaring_data_t* fw_soaring_data)
{
    return mavlink_msg_fw_soaring_data_pack_status(system_id, component_id, _status, msg,  fw_soaring_data->timestamp, fw_soaring_data->timestampModeChanged, fw_soaring_data->xW, fw_soaring_data->xR, fw_soaring_data->xLat, fw_soaring_data->xLon, fw_soaring_data->VarW, fw_soaring_data->VarR, fw_soaring_data->VarLat, fw_soaring_data->VarLon, fw_soaring_data->LoiterRadius, fw_soaring_data->LoiterDirection, fw_soaring_data->DistToSoarPoint, fw_soaring_data->vSinkExp, fw_soaring_data->z1_LocalUpdraftSpeed, fw_soaring_data->z2_DeltaRoll, fw_soaring_data->z1_exp, fw_soaring_data->z2_exp, fw_soaring_data->ThermalGSNorth, fw_soaring_data->ThermalGSEast, fw_soaring_data->TSE_dot, fw_soaring_data->DebugVar1, fw_soaring_data->DebugVar2, fw_soaring_data->ControlMode, fw_soaring_data->valid);
}

/**
 * @brief Send a fw_soaring_data message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] Timestamp
 * @param timestampModeChanged [ms] Timestamp since last mode change
 * @param xW [m/s] Thermal core updraft strength
 * @param xR [m] Thermal radius
 * @param xLat [deg] Thermal center latitude
 * @param xLon [deg] Thermal center longitude
 * @param VarW  Variance W
 * @param VarR  Variance R
 * @param VarLat  Variance Lat
 * @param VarLon  Variance Lon 
 * @param LoiterRadius [m] Suggested loiter radius
 * @param LoiterDirection  Suggested loiter direction
 * @param DistToSoarPoint [m] Distance to soar point
 * @param vSinkExp [m/s] Expected sink rate at current airspeed, roll and throttle
 * @param z1_LocalUpdraftSpeed [m/s] Measurement / updraft speed at current/local airplane position
 * @param z2_DeltaRoll [deg] Measurement / roll angle tracking error
 * @param z1_exp  Expected measurement 1
 * @param z2_exp  Expected measurement 2
 * @param ThermalGSNorth [m/s] Thermal drift (from estimator prediction step only)
 * @param ThermalGSEast [m/s] Thermal drift (from estimator prediction step only)
 * @param TSE_dot [m/s]  Total specific energy change (filtered)
 * @param DebugVar1   Debug variable 1
 * @param DebugVar2   Debug variable 2
 * @param ControlMode  Control Mode [-]
 * @param valid  Data valid [-]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fw_soaring_data_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t timestampModeChanged, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, float LoiterDirection, float DistToSoarPoint, float vSinkExp, float z1_LocalUpdraftSpeed, float z2_DeltaRoll, float z1_exp, float z2_exp, float ThermalGSNorth, float ThermalGSEast, float TSE_dot, float DebugVar1, float DebugVar2, uint8_t ControlMode, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FW_SOARING_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, timestampModeChanged);
    _mav_put_float(buf, 16, xW);
    _mav_put_float(buf, 20, xR);
    _mav_put_float(buf, 24, xLat);
    _mav_put_float(buf, 28, xLon);
    _mav_put_float(buf, 32, VarW);
    _mav_put_float(buf, 36, VarR);
    _mav_put_float(buf, 40, VarLat);
    _mav_put_float(buf, 44, VarLon);
    _mav_put_float(buf, 48, LoiterRadius);
    _mav_put_float(buf, 52, LoiterDirection);
    _mav_put_float(buf, 56, DistToSoarPoint);
    _mav_put_float(buf, 60, vSinkExp);
    _mav_put_float(buf, 64, z1_LocalUpdraftSpeed);
    _mav_put_float(buf, 68, z2_DeltaRoll);
    _mav_put_float(buf, 72, z1_exp);
    _mav_put_float(buf, 76, z2_exp);
    _mav_put_float(buf, 80, ThermalGSNorth);
    _mav_put_float(buf, 84, ThermalGSEast);
    _mav_put_float(buf, 88, TSE_dot);
    _mav_put_float(buf, 92, DebugVar1);
    _mav_put_float(buf, 96, DebugVar2);
    _mav_put_uint8_t(buf, 100, ControlMode);
    _mav_put_uint8_t(buf, 101, valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, buf, MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#else
    mavlink_fw_soaring_data_t packet;
    packet.timestamp = timestamp;
    packet.timestampModeChanged = timestampModeChanged;
    packet.xW = xW;
    packet.xR = xR;
    packet.xLat = xLat;
    packet.xLon = xLon;
    packet.VarW = VarW;
    packet.VarR = VarR;
    packet.VarLat = VarLat;
    packet.VarLon = VarLon;
    packet.LoiterRadius = LoiterRadius;
    packet.LoiterDirection = LoiterDirection;
    packet.DistToSoarPoint = DistToSoarPoint;
    packet.vSinkExp = vSinkExp;
    packet.z1_LocalUpdraftSpeed = z1_LocalUpdraftSpeed;
    packet.z2_DeltaRoll = z2_DeltaRoll;
    packet.z1_exp = z1_exp;
    packet.z2_exp = z2_exp;
    packet.ThermalGSNorth = ThermalGSNorth;
    packet.ThermalGSEast = ThermalGSEast;
    packet.TSE_dot = TSE_dot;
    packet.DebugVar1 = DebugVar1;
    packet.DebugVar2 = DebugVar2;
    packet.ControlMode = ControlMode;
    packet.valid = valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, (const char *)&packet, MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#endif
}

/**
 * @brief Send a fw_soaring_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fw_soaring_data_send_struct(mavlink_channel_t chan, const mavlink_fw_soaring_data_t* fw_soaring_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fw_soaring_data_send(chan, fw_soaring_data->timestamp, fw_soaring_data->timestampModeChanged, fw_soaring_data->xW, fw_soaring_data->xR, fw_soaring_data->xLat, fw_soaring_data->xLon, fw_soaring_data->VarW, fw_soaring_data->VarR, fw_soaring_data->VarLat, fw_soaring_data->VarLon, fw_soaring_data->LoiterRadius, fw_soaring_data->LoiterDirection, fw_soaring_data->DistToSoarPoint, fw_soaring_data->vSinkExp, fw_soaring_data->z1_LocalUpdraftSpeed, fw_soaring_data->z2_DeltaRoll, fw_soaring_data->z1_exp, fw_soaring_data->z2_exp, fw_soaring_data->ThermalGSNorth, fw_soaring_data->ThermalGSEast, fw_soaring_data->TSE_dot, fw_soaring_data->DebugVar1, fw_soaring_data->DebugVar2, fw_soaring_data->ControlMode, fw_soaring_data->valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, (const char *)fw_soaring_data, MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_FW_SOARING_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fw_soaring_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t timestampModeChanged, float xW, float xR, float xLat, float xLon, float VarW, float VarR, float VarLat, float VarLon, float LoiterRadius, float LoiterDirection, float DistToSoarPoint, float vSinkExp, float z1_LocalUpdraftSpeed, float z2_DeltaRoll, float z1_exp, float z2_exp, float ThermalGSNorth, float ThermalGSEast, float TSE_dot, float DebugVar1, float DebugVar2, uint8_t ControlMode, uint8_t valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, timestampModeChanged);
    _mav_put_float(buf, 16, xW);
    _mav_put_float(buf, 20, xR);
    _mav_put_float(buf, 24, xLat);
    _mav_put_float(buf, 28, xLon);
    _mav_put_float(buf, 32, VarW);
    _mav_put_float(buf, 36, VarR);
    _mav_put_float(buf, 40, VarLat);
    _mav_put_float(buf, 44, VarLon);
    _mav_put_float(buf, 48, LoiterRadius);
    _mav_put_float(buf, 52, LoiterDirection);
    _mav_put_float(buf, 56, DistToSoarPoint);
    _mav_put_float(buf, 60, vSinkExp);
    _mav_put_float(buf, 64, z1_LocalUpdraftSpeed);
    _mav_put_float(buf, 68, z2_DeltaRoll);
    _mav_put_float(buf, 72, z1_exp);
    _mav_put_float(buf, 76, z2_exp);
    _mav_put_float(buf, 80, ThermalGSNorth);
    _mav_put_float(buf, 84, ThermalGSEast);
    _mav_put_float(buf, 88, TSE_dot);
    _mav_put_float(buf, 92, DebugVar1);
    _mav_put_float(buf, 96, DebugVar2);
    _mav_put_uint8_t(buf, 100, ControlMode);
    _mav_put_uint8_t(buf, 101, valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, buf, MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#else
    mavlink_fw_soaring_data_t *packet = (mavlink_fw_soaring_data_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->timestampModeChanged = timestampModeChanged;
    packet->xW = xW;
    packet->xR = xR;
    packet->xLat = xLat;
    packet->xLon = xLon;
    packet->VarW = VarW;
    packet->VarR = VarR;
    packet->VarLat = VarLat;
    packet->VarLon = VarLon;
    packet->LoiterRadius = LoiterRadius;
    packet->LoiterDirection = LoiterDirection;
    packet->DistToSoarPoint = DistToSoarPoint;
    packet->vSinkExp = vSinkExp;
    packet->z1_LocalUpdraftSpeed = z1_LocalUpdraftSpeed;
    packet->z2_DeltaRoll = z2_DeltaRoll;
    packet->z1_exp = z1_exp;
    packet->z2_exp = z2_exp;
    packet->ThermalGSNorth = ThermalGSNorth;
    packet->ThermalGSEast = ThermalGSEast;
    packet->TSE_dot = TSE_dot;
    packet->DebugVar1 = DebugVar1;
    packet->DebugVar2 = DebugVar2;
    packet->ControlMode = ControlMode;
    packet->valid = valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FW_SOARING_DATA, (const char *)packet, MAVLINK_MSG_ID_FW_SOARING_DATA_MIN_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN, MAVLINK_MSG_ID_FW_SOARING_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE FW_SOARING_DATA UNPACKING


/**
 * @brief Get field timestamp from fw_soaring_data message
 *
 * @return [ms] Timestamp
 */
static inline uint64_t mavlink_msg_fw_soaring_data_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field timestampModeChanged from fw_soaring_data message
 *
 * @return [ms] Timestamp since last mode change
 */
static inline uint64_t mavlink_msg_fw_soaring_data_get_timestampModeChanged(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field xW from fw_soaring_data message
 *
 * @return [m/s] Thermal core updraft strength
 */
static inline float mavlink_msg_fw_soaring_data_get_xW(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xR from fw_soaring_data message
 *
 * @return [m] Thermal radius
 */
static inline float mavlink_msg_fw_soaring_data_get_xR(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field xLat from fw_soaring_data message
 *
 * @return [deg] Thermal center latitude
 */
static inline float mavlink_msg_fw_soaring_data_get_xLat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field xLon from fw_soaring_data message
 *
 * @return [deg] Thermal center longitude
 */
static inline float mavlink_msg_fw_soaring_data_get_xLon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field VarW from fw_soaring_data message
 *
 * @return  Variance W
 */
static inline float mavlink_msg_fw_soaring_data_get_VarW(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field VarR from fw_soaring_data message
 *
 * @return  Variance R
 */
static inline float mavlink_msg_fw_soaring_data_get_VarR(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field VarLat from fw_soaring_data message
 *
 * @return  Variance Lat
 */
static inline float mavlink_msg_fw_soaring_data_get_VarLat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field VarLon from fw_soaring_data message
 *
 * @return  Variance Lon 
 */
static inline float mavlink_msg_fw_soaring_data_get_VarLon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field LoiterRadius from fw_soaring_data message
 *
 * @return [m] Suggested loiter radius
 */
static inline float mavlink_msg_fw_soaring_data_get_LoiterRadius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field LoiterDirection from fw_soaring_data message
 *
 * @return  Suggested loiter direction
 */
static inline float mavlink_msg_fw_soaring_data_get_LoiterDirection(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field DistToSoarPoint from fw_soaring_data message
 *
 * @return [m] Distance to soar point
 */
static inline float mavlink_msg_fw_soaring_data_get_DistToSoarPoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field vSinkExp from fw_soaring_data message
 *
 * @return [m/s] Expected sink rate at current airspeed, roll and throttle
 */
static inline float mavlink_msg_fw_soaring_data_get_vSinkExp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field z1_LocalUpdraftSpeed from fw_soaring_data message
 *
 * @return [m/s] Measurement / updraft speed at current/local airplane position
 */
static inline float mavlink_msg_fw_soaring_data_get_z1_LocalUpdraftSpeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field z2_DeltaRoll from fw_soaring_data message
 *
 * @return [deg] Measurement / roll angle tracking error
 */
static inline float mavlink_msg_fw_soaring_data_get_z2_DeltaRoll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field z1_exp from fw_soaring_data message
 *
 * @return  Expected measurement 1
 */
static inline float mavlink_msg_fw_soaring_data_get_z1_exp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field z2_exp from fw_soaring_data message
 *
 * @return  Expected measurement 2
 */
static inline float mavlink_msg_fw_soaring_data_get_z2_exp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field ThermalGSNorth from fw_soaring_data message
 *
 * @return [m/s] Thermal drift (from estimator prediction step only)
 */
static inline float mavlink_msg_fw_soaring_data_get_ThermalGSNorth(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field ThermalGSEast from fw_soaring_data message
 *
 * @return [m/s] Thermal drift (from estimator prediction step only)
 */
static inline float mavlink_msg_fw_soaring_data_get_ThermalGSEast(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field TSE_dot from fw_soaring_data message
 *
 * @return [m/s]  Total specific energy change (filtered)
 */
static inline float mavlink_msg_fw_soaring_data_get_TSE_dot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field DebugVar1 from fw_soaring_data message
 *
 * @return   Debug variable 1
 */
static inline float mavlink_msg_fw_soaring_data_get_DebugVar1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field DebugVar2 from fw_soaring_data message
 *
 * @return   Debug variable 2
 */
static inline float mavlink_msg_fw_soaring_data_get_DebugVar2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field ControlMode from fw_soaring_data message
 *
 * @return  Control Mode [-]
 */
static inline uint8_t mavlink_msg_fw_soaring_data_get_ControlMode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  100);
}

/**
 * @brief Get field valid from fw_soaring_data message
 *
 * @return  Data valid [-]
 */
static inline uint8_t mavlink_msg_fw_soaring_data_get_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  101);
}

/**
 * @brief Decode a fw_soaring_data message into a struct
 *
 * @param msg The message to decode
 * @param fw_soaring_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_fw_soaring_data_decode(const mavlink_message_t* msg, mavlink_fw_soaring_data_t* fw_soaring_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fw_soaring_data->timestamp = mavlink_msg_fw_soaring_data_get_timestamp(msg);
    fw_soaring_data->timestampModeChanged = mavlink_msg_fw_soaring_data_get_timestampModeChanged(msg);
    fw_soaring_data->xW = mavlink_msg_fw_soaring_data_get_xW(msg);
    fw_soaring_data->xR = mavlink_msg_fw_soaring_data_get_xR(msg);
    fw_soaring_data->xLat = mavlink_msg_fw_soaring_data_get_xLat(msg);
    fw_soaring_data->xLon = mavlink_msg_fw_soaring_data_get_xLon(msg);
    fw_soaring_data->VarW = mavlink_msg_fw_soaring_data_get_VarW(msg);
    fw_soaring_data->VarR = mavlink_msg_fw_soaring_data_get_VarR(msg);
    fw_soaring_data->VarLat = mavlink_msg_fw_soaring_data_get_VarLat(msg);
    fw_soaring_data->VarLon = mavlink_msg_fw_soaring_data_get_VarLon(msg);
    fw_soaring_data->LoiterRadius = mavlink_msg_fw_soaring_data_get_LoiterRadius(msg);
    fw_soaring_data->LoiterDirection = mavlink_msg_fw_soaring_data_get_LoiterDirection(msg);
    fw_soaring_data->DistToSoarPoint = mavlink_msg_fw_soaring_data_get_DistToSoarPoint(msg);
    fw_soaring_data->vSinkExp = mavlink_msg_fw_soaring_data_get_vSinkExp(msg);
    fw_soaring_data->z1_LocalUpdraftSpeed = mavlink_msg_fw_soaring_data_get_z1_LocalUpdraftSpeed(msg);
    fw_soaring_data->z2_DeltaRoll = mavlink_msg_fw_soaring_data_get_z2_DeltaRoll(msg);
    fw_soaring_data->z1_exp = mavlink_msg_fw_soaring_data_get_z1_exp(msg);
    fw_soaring_data->z2_exp = mavlink_msg_fw_soaring_data_get_z2_exp(msg);
    fw_soaring_data->ThermalGSNorth = mavlink_msg_fw_soaring_data_get_ThermalGSNorth(msg);
    fw_soaring_data->ThermalGSEast = mavlink_msg_fw_soaring_data_get_ThermalGSEast(msg);
    fw_soaring_data->TSE_dot = mavlink_msg_fw_soaring_data_get_TSE_dot(msg);
    fw_soaring_data->DebugVar1 = mavlink_msg_fw_soaring_data_get_DebugVar1(msg);
    fw_soaring_data->DebugVar2 = mavlink_msg_fw_soaring_data_get_DebugVar2(msg);
    fw_soaring_data->ControlMode = mavlink_msg_fw_soaring_data_get_ControlMode(msg);
    fw_soaring_data->valid = mavlink_msg_fw_soaring_data_get_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FW_SOARING_DATA_LEN? msg->len : MAVLINK_MSG_ID_FW_SOARING_DATA_LEN;
        memset(fw_soaring_data, 0, MAVLINK_MSG_ID_FW_SOARING_DATA_LEN);
    memcpy(fw_soaring_data, _MAV_PAYLOAD(msg), len);
#endif
}
