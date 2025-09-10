#pragma once
// MESSAGE ASLCTRL_DATA PACKING

#define MAVLINK_MSG_ID_ASLCTRL_DATA 8004


typedef struct __mavlink_aslctrl_data_t {
 uint64_t timestamp; /*< [us]  Timestamp*/
 float h; /*<   See sourcecode for a description of these values... */
 float hRef; /*<   */
 float hRef_t; /*<   */
 float PitchAngle; /*< [deg] Pitch angle*/
 float PitchAngleRef; /*< [deg] Pitch angle reference*/
 float q; /*<   */
 float qRef; /*<   */
 float uElev; /*<   */
 float uThrot; /*<   */
 float uThrot2; /*<   */
 float nZ; /*<   */
 float AirspeedRef; /*< [m/s] Airspeed reference*/
 float YawAngle; /*< [deg] Yaw angle*/
 float YawAngleRef; /*< [deg] Yaw angle reference*/
 float RollAngle; /*< [deg] Roll angle*/
 float RollAngleRef; /*< [deg] Roll angle reference*/
 float p; /*<   */
 float pRef; /*<   */
 float r; /*<   */
 float rRef; /*<   */
 float uAil; /*<   */
 float uRud; /*<   */
 uint8_t aslctrl_mode; /*<   ASLCTRL control-mode (manual, stabilized, auto, etc...)*/
 uint8_t SpoilersEngaged; /*<   */
} mavlink_aslctrl_data_t;

#define MAVLINK_MSG_ID_ASLCTRL_DATA_LEN 98
#define MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN 98
#define MAVLINK_MSG_ID_8004_LEN 98
#define MAVLINK_MSG_ID_8004_MIN_LEN 98

#define MAVLINK_MSG_ID_ASLCTRL_DATA_CRC 172
#define MAVLINK_MSG_ID_8004_CRC 172



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ASLCTRL_DATA { \
    8004, \
    "ASLCTRL_DATA", \
    25, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_aslctrl_data_t, timestamp) }, \
         { "aslctrl_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 96, offsetof(mavlink_aslctrl_data_t, aslctrl_mode) }, \
         { "h", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_aslctrl_data_t, h) }, \
         { "hRef", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_aslctrl_data_t, hRef) }, \
         { "hRef_t", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_aslctrl_data_t, hRef_t) }, \
         { "PitchAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_aslctrl_data_t, PitchAngle) }, \
         { "PitchAngleRef", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_aslctrl_data_t, PitchAngleRef) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_aslctrl_data_t, q) }, \
         { "qRef", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_aslctrl_data_t, qRef) }, \
         { "uElev", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_aslctrl_data_t, uElev) }, \
         { "uThrot", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_aslctrl_data_t, uThrot) }, \
         { "uThrot2", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_aslctrl_data_t, uThrot2) }, \
         { "nZ", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_aslctrl_data_t, nZ) }, \
         { "AirspeedRef", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_aslctrl_data_t, AirspeedRef) }, \
         { "SpoilersEngaged", NULL, MAVLINK_TYPE_UINT8_T, 0, 97, offsetof(mavlink_aslctrl_data_t, SpoilersEngaged) }, \
         { "YawAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_aslctrl_data_t, YawAngle) }, \
         { "YawAngleRef", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_aslctrl_data_t, YawAngleRef) }, \
         { "RollAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_aslctrl_data_t, RollAngle) }, \
         { "RollAngleRef", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_aslctrl_data_t, RollAngleRef) }, \
         { "p", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_aslctrl_data_t, p) }, \
         { "pRef", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_aslctrl_data_t, pRef) }, \
         { "r", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_aslctrl_data_t, r) }, \
         { "rRef", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_aslctrl_data_t, rRef) }, \
         { "uAil", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_aslctrl_data_t, uAil) }, \
         { "uRud", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_aslctrl_data_t, uRud) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ASLCTRL_DATA { \
    "ASLCTRL_DATA", \
    25, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_aslctrl_data_t, timestamp) }, \
         { "aslctrl_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 96, offsetof(mavlink_aslctrl_data_t, aslctrl_mode) }, \
         { "h", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_aslctrl_data_t, h) }, \
         { "hRef", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_aslctrl_data_t, hRef) }, \
         { "hRef_t", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_aslctrl_data_t, hRef_t) }, \
         { "PitchAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_aslctrl_data_t, PitchAngle) }, \
         { "PitchAngleRef", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_aslctrl_data_t, PitchAngleRef) }, \
         { "q", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_aslctrl_data_t, q) }, \
         { "qRef", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_aslctrl_data_t, qRef) }, \
         { "uElev", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_aslctrl_data_t, uElev) }, \
         { "uThrot", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_aslctrl_data_t, uThrot) }, \
         { "uThrot2", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_aslctrl_data_t, uThrot2) }, \
         { "nZ", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_aslctrl_data_t, nZ) }, \
         { "AirspeedRef", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_aslctrl_data_t, AirspeedRef) }, \
         { "SpoilersEngaged", NULL, MAVLINK_TYPE_UINT8_T, 0, 97, offsetof(mavlink_aslctrl_data_t, SpoilersEngaged) }, \
         { "YawAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_aslctrl_data_t, YawAngle) }, \
         { "YawAngleRef", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_aslctrl_data_t, YawAngleRef) }, \
         { "RollAngle", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_aslctrl_data_t, RollAngle) }, \
         { "RollAngleRef", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_aslctrl_data_t, RollAngleRef) }, \
         { "p", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_aslctrl_data_t, p) }, \
         { "pRef", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_aslctrl_data_t, pRef) }, \
         { "r", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_aslctrl_data_t, r) }, \
         { "rRef", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_aslctrl_data_t, rRef) }, \
         { "uAil", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_aslctrl_data_t, uAil) }, \
         { "uRud", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_aslctrl_data_t, uRud) }, \
         } \
}
#endif

/**
 * @brief Pack a aslctrl_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us]  Timestamp
 * @param aslctrl_mode   ASLCTRL control-mode (manual, stabilized, auto, etc...)
 * @param h   See sourcecode for a description of these values... 
 * @param hRef   
 * @param hRef_t   
 * @param PitchAngle [deg] Pitch angle
 * @param PitchAngleRef [deg] Pitch angle reference
 * @param q   
 * @param qRef   
 * @param uElev   
 * @param uThrot   
 * @param uThrot2   
 * @param nZ   
 * @param AirspeedRef [m/s] Airspeed reference
 * @param SpoilersEngaged   
 * @param YawAngle [deg] Yaw angle
 * @param YawAngleRef [deg] Yaw angle reference
 * @param RollAngle [deg] Roll angle
 * @param RollAngleRef [deg] Roll angle reference
 * @param p   
 * @param pRef   
 * @param r   
 * @param rRef   
 * @param uAil   
 * @param uRud   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aslctrl_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t aslctrl_mode, float h, float hRef, float hRef_t, float PitchAngle, float PitchAngleRef, float q, float qRef, float uElev, float uThrot, float uThrot2, float nZ, float AirspeedRef, uint8_t SpoilersEngaged, float YawAngle, float YawAngleRef, float RollAngle, float RollAngleRef, float p, float pRef, float r, float rRef, float uAil, float uRud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLCTRL_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, h);
    _mav_put_float(buf, 12, hRef);
    _mav_put_float(buf, 16, hRef_t);
    _mav_put_float(buf, 20, PitchAngle);
    _mav_put_float(buf, 24, PitchAngleRef);
    _mav_put_float(buf, 28, q);
    _mav_put_float(buf, 32, qRef);
    _mav_put_float(buf, 36, uElev);
    _mav_put_float(buf, 40, uThrot);
    _mav_put_float(buf, 44, uThrot2);
    _mav_put_float(buf, 48, nZ);
    _mav_put_float(buf, 52, AirspeedRef);
    _mav_put_float(buf, 56, YawAngle);
    _mav_put_float(buf, 60, YawAngleRef);
    _mav_put_float(buf, 64, RollAngle);
    _mav_put_float(buf, 68, RollAngleRef);
    _mav_put_float(buf, 72, p);
    _mav_put_float(buf, 76, pRef);
    _mav_put_float(buf, 80, r);
    _mav_put_float(buf, 84, rRef);
    _mav_put_float(buf, 88, uAil);
    _mav_put_float(buf, 92, uRud);
    _mav_put_uint8_t(buf, 96, aslctrl_mode);
    _mav_put_uint8_t(buf, 97, SpoilersEngaged);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN);
#else
    mavlink_aslctrl_data_t packet;
    packet.timestamp = timestamp;
    packet.h = h;
    packet.hRef = hRef;
    packet.hRef_t = hRef_t;
    packet.PitchAngle = PitchAngle;
    packet.PitchAngleRef = PitchAngleRef;
    packet.q = q;
    packet.qRef = qRef;
    packet.uElev = uElev;
    packet.uThrot = uThrot;
    packet.uThrot2 = uThrot2;
    packet.nZ = nZ;
    packet.AirspeedRef = AirspeedRef;
    packet.YawAngle = YawAngle;
    packet.YawAngleRef = YawAngleRef;
    packet.RollAngle = RollAngle;
    packet.RollAngleRef = RollAngleRef;
    packet.p = p;
    packet.pRef = pRef;
    packet.r = r;
    packet.rRef = rRef;
    packet.uAil = uAil;
    packet.uRud = uRud;
    packet.aslctrl_mode = aslctrl_mode;
    packet.SpoilersEngaged = SpoilersEngaged;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASLCTRL_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_CRC);
}

/**
 * @brief Pack a aslctrl_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us]  Timestamp
 * @param aslctrl_mode   ASLCTRL control-mode (manual, stabilized, auto, etc...)
 * @param h   See sourcecode for a description of these values... 
 * @param hRef   
 * @param hRef_t   
 * @param PitchAngle [deg] Pitch angle
 * @param PitchAngleRef [deg] Pitch angle reference
 * @param q   
 * @param qRef   
 * @param uElev   
 * @param uThrot   
 * @param uThrot2   
 * @param nZ   
 * @param AirspeedRef [m/s] Airspeed reference
 * @param SpoilersEngaged   
 * @param YawAngle [deg] Yaw angle
 * @param YawAngleRef [deg] Yaw angle reference
 * @param RollAngle [deg] Roll angle
 * @param RollAngleRef [deg] Roll angle reference
 * @param p   
 * @param pRef   
 * @param r   
 * @param rRef   
 * @param uAil   
 * @param uRud   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aslctrl_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t aslctrl_mode, float h, float hRef, float hRef_t, float PitchAngle, float PitchAngleRef, float q, float qRef, float uElev, float uThrot, float uThrot2, float nZ, float AirspeedRef, uint8_t SpoilersEngaged, float YawAngle, float YawAngleRef, float RollAngle, float RollAngleRef, float p, float pRef, float r, float rRef, float uAil, float uRud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLCTRL_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, h);
    _mav_put_float(buf, 12, hRef);
    _mav_put_float(buf, 16, hRef_t);
    _mav_put_float(buf, 20, PitchAngle);
    _mav_put_float(buf, 24, PitchAngleRef);
    _mav_put_float(buf, 28, q);
    _mav_put_float(buf, 32, qRef);
    _mav_put_float(buf, 36, uElev);
    _mav_put_float(buf, 40, uThrot);
    _mav_put_float(buf, 44, uThrot2);
    _mav_put_float(buf, 48, nZ);
    _mav_put_float(buf, 52, AirspeedRef);
    _mav_put_float(buf, 56, YawAngle);
    _mav_put_float(buf, 60, YawAngleRef);
    _mav_put_float(buf, 64, RollAngle);
    _mav_put_float(buf, 68, RollAngleRef);
    _mav_put_float(buf, 72, p);
    _mav_put_float(buf, 76, pRef);
    _mav_put_float(buf, 80, r);
    _mav_put_float(buf, 84, rRef);
    _mav_put_float(buf, 88, uAil);
    _mav_put_float(buf, 92, uRud);
    _mav_put_uint8_t(buf, 96, aslctrl_mode);
    _mav_put_uint8_t(buf, 97, SpoilersEngaged);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN);
#else
    mavlink_aslctrl_data_t packet;
    packet.timestamp = timestamp;
    packet.h = h;
    packet.hRef = hRef;
    packet.hRef_t = hRef_t;
    packet.PitchAngle = PitchAngle;
    packet.PitchAngleRef = PitchAngleRef;
    packet.q = q;
    packet.qRef = qRef;
    packet.uElev = uElev;
    packet.uThrot = uThrot;
    packet.uThrot2 = uThrot2;
    packet.nZ = nZ;
    packet.AirspeedRef = AirspeedRef;
    packet.YawAngle = YawAngle;
    packet.YawAngleRef = YawAngleRef;
    packet.RollAngle = RollAngle;
    packet.RollAngleRef = RollAngleRef;
    packet.p = p;
    packet.pRef = pRef;
    packet.r = r;
    packet.rRef = rRef;
    packet.uAil = uAil;
    packet.uRud = uRud;
    packet.aslctrl_mode = aslctrl_mode;
    packet.SpoilersEngaged = SpoilersEngaged;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASLCTRL_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN);
#endif
}

/**
 * @brief Pack a aslctrl_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us]  Timestamp
 * @param aslctrl_mode   ASLCTRL control-mode (manual, stabilized, auto, etc...)
 * @param h   See sourcecode for a description of these values... 
 * @param hRef   
 * @param hRef_t   
 * @param PitchAngle [deg] Pitch angle
 * @param PitchAngleRef [deg] Pitch angle reference
 * @param q   
 * @param qRef   
 * @param uElev   
 * @param uThrot   
 * @param uThrot2   
 * @param nZ   
 * @param AirspeedRef [m/s] Airspeed reference
 * @param SpoilersEngaged   
 * @param YawAngle [deg] Yaw angle
 * @param YawAngleRef [deg] Yaw angle reference
 * @param RollAngle [deg] Roll angle
 * @param RollAngleRef [deg] Roll angle reference
 * @param p   
 * @param pRef   
 * @param r   
 * @param rRef   
 * @param uAil   
 * @param uRud   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aslctrl_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t aslctrl_mode,float h,float hRef,float hRef_t,float PitchAngle,float PitchAngleRef,float q,float qRef,float uElev,float uThrot,float uThrot2,float nZ,float AirspeedRef,uint8_t SpoilersEngaged,float YawAngle,float YawAngleRef,float RollAngle,float RollAngleRef,float p,float pRef,float r,float rRef,float uAil,float uRud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLCTRL_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, h);
    _mav_put_float(buf, 12, hRef);
    _mav_put_float(buf, 16, hRef_t);
    _mav_put_float(buf, 20, PitchAngle);
    _mav_put_float(buf, 24, PitchAngleRef);
    _mav_put_float(buf, 28, q);
    _mav_put_float(buf, 32, qRef);
    _mav_put_float(buf, 36, uElev);
    _mav_put_float(buf, 40, uThrot);
    _mav_put_float(buf, 44, uThrot2);
    _mav_put_float(buf, 48, nZ);
    _mav_put_float(buf, 52, AirspeedRef);
    _mav_put_float(buf, 56, YawAngle);
    _mav_put_float(buf, 60, YawAngleRef);
    _mav_put_float(buf, 64, RollAngle);
    _mav_put_float(buf, 68, RollAngleRef);
    _mav_put_float(buf, 72, p);
    _mav_put_float(buf, 76, pRef);
    _mav_put_float(buf, 80, r);
    _mav_put_float(buf, 84, rRef);
    _mav_put_float(buf, 88, uAil);
    _mav_put_float(buf, 92, uRud);
    _mav_put_uint8_t(buf, 96, aslctrl_mode);
    _mav_put_uint8_t(buf, 97, SpoilersEngaged);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN);
#else
    mavlink_aslctrl_data_t packet;
    packet.timestamp = timestamp;
    packet.h = h;
    packet.hRef = hRef;
    packet.hRef_t = hRef_t;
    packet.PitchAngle = PitchAngle;
    packet.PitchAngleRef = PitchAngleRef;
    packet.q = q;
    packet.qRef = qRef;
    packet.uElev = uElev;
    packet.uThrot = uThrot;
    packet.uThrot2 = uThrot2;
    packet.nZ = nZ;
    packet.AirspeedRef = AirspeedRef;
    packet.YawAngle = YawAngle;
    packet.YawAngleRef = YawAngleRef;
    packet.RollAngle = RollAngle;
    packet.RollAngleRef = RollAngleRef;
    packet.p = p;
    packet.pRef = pRef;
    packet.r = r;
    packet.rRef = rRef;
    packet.uAil = uAil;
    packet.uRud = uRud;
    packet.aslctrl_mode = aslctrl_mode;
    packet.SpoilersEngaged = SpoilersEngaged;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ASLCTRL_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_CRC);
}

/**
 * @brief Encode a aslctrl_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param aslctrl_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aslctrl_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_aslctrl_data_t* aslctrl_data)
{
    return mavlink_msg_aslctrl_data_pack(system_id, component_id, msg, aslctrl_data->timestamp, aslctrl_data->aslctrl_mode, aslctrl_data->h, aslctrl_data->hRef, aslctrl_data->hRef_t, aslctrl_data->PitchAngle, aslctrl_data->PitchAngleRef, aslctrl_data->q, aslctrl_data->qRef, aslctrl_data->uElev, aslctrl_data->uThrot, aslctrl_data->uThrot2, aslctrl_data->nZ, aslctrl_data->AirspeedRef, aslctrl_data->SpoilersEngaged, aslctrl_data->YawAngle, aslctrl_data->YawAngleRef, aslctrl_data->RollAngle, aslctrl_data->RollAngleRef, aslctrl_data->p, aslctrl_data->pRef, aslctrl_data->r, aslctrl_data->rRef, aslctrl_data->uAil, aslctrl_data->uRud);
}

/**
 * @brief Encode a aslctrl_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param aslctrl_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aslctrl_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_aslctrl_data_t* aslctrl_data)
{
    return mavlink_msg_aslctrl_data_pack_chan(system_id, component_id, chan, msg, aslctrl_data->timestamp, aslctrl_data->aslctrl_mode, aslctrl_data->h, aslctrl_data->hRef, aslctrl_data->hRef_t, aslctrl_data->PitchAngle, aslctrl_data->PitchAngleRef, aslctrl_data->q, aslctrl_data->qRef, aslctrl_data->uElev, aslctrl_data->uThrot, aslctrl_data->uThrot2, aslctrl_data->nZ, aslctrl_data->AirspeedRef, aslctrl_data->SpoilersEngaged, aslctrl_data->YawAngle, aslctrl_data->YawAngleRef, aslctrl_data->RollAngle, aslctrl_data->RollAngleRef, aslctrl_data->p, aslctrl_data->pRef, aslctrl_data->r, aslctrl_data->rRef, aslctrl_data->uAil, aslctrl_data->uRud);
}

/**
 * @brief Encode a aslctrl_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param aslctrl_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aslctrl_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_aslctrl_data_t* aslctrl_data)
{
    return mavlink_msg_aslctrl_data_pack_status(system_id, component_id, _status, msg,  aslctrl_data->timestamp, aslctrl_data->aslctrl_mode, aslctrl_data->h, aslctrl_data->hRef, aslctrl_data->hRef_t, aslctrl_data->PitchAngle, aslctrl_data->PitchAngleRef, aslctrl_data->q, aslctrl_data->qRef, aslctrl_data->uElev, aslctrl_data->uThrot, aslctrl_data->uThrot2, aslctrl_data->nZ, aslctrl_data->AirspeedRef, aslctrl_data->SpoilersEngaged, aslctrl_data->YawAngle, aslctrl_data->YawAngleRef, aslctrl_data->RollAngle, aslctrl_data->RollAngleRef, aslctrl_data->p, aslctrl_data->pRef, aslctrl_data->r, aslctrl_data->rRef, aslctrl_data->uAil, aslctrl_data->uRud);
}

/**
 * @brief Send a aslctrl_data message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us]  Timestamp
 * @param aslctrl_mode   ASLCTRL control-mode (manual, stabilized, auto, etc...)
 * @param h   See sourcecode for a description of these values... 
 * @param hRef   
 * @param hRef_t   
 * @param PitchAngle [deg] Pitch angle
 * @param PitchAngleRef [deg] Pitch angle reference
 * @param q   
 * @param qRef   
 * @param uElev   
 * @param uThrot   
 * @param uThrot2   
 * @param nZ   
 * @param AirspeedRef [m/s] Airspeed reference
 * @param SpoilersEngaged   
 * @param YawAngle [deg] Yaw angle
 * @param YawAngleRef [deg] Yaw angle reference
 * @param RollAngle [deg] Roll angle
 * @param RollAngleRef [deg] Roll angle reference
 * @param p   
 * @param pRef   
 * @param r   
 * @param rRef   
 * @param uAil   
 * @param uRud   
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_aslctrl_data_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t aslctrl_mode, float h, float hRef, float hRef_t, float PitchAngle, float PitchAngleRef, float q, float qRef, float uElev, float uThrot, float uThrot2, float nZ, float AirspeedRef, uint8_t SpoilersEngaged, float YawAngle, float YawAngleRef, float RollAngle, float RollAngleRef, float p, float pRef, float r, float rRef, float uAil, float uRud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ASLCTRL_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, h);
    _mav_put_float(buf, 12, hRef);
    _mav_put_float(buf, 16, hRef_t);
    _mav_put_float(buf, 20, PitchAngle);
    _mav_put_float(buf, 24, PitchAngleRef);
    _mav_put_float(buf, 28, q);
    _mav_put_float(buf, 32, qRef);
    _mav_put_float(buf, 36, uElev);
    _mav_put_float(buf, 40, uThrot);
    _mav_put_float(buf, 44, uThrot2);
    _mav_put_float(buf, 48, nZ);
    _mav_put_float(buf, 52, AirspeedRef);
    _mav_put_float(buf, 56, YawAngle);
    _mav_put_float(buf, 60, YawAngleRef);
    _mav_put_float(buf, 64, RollAngle);
    _mav_put_float(buf, 68, RollAngleRef);
    _mav_put_float(buf, 72, p);
    _mav_put_float(buf, 76, pRef);
    _mav_put_float(buf, 80, r);
    _mav_put_float(buf, 84, rRef);
    _mav_put_float(buf, 88, uAil);
    _mav_put_float(buf, 92, uRud);
    _mav_put_uint8_t(buf, 96, aslctrl_mode);
    _mav_put_uint8_t(buf, 97, SpoilersEngaged);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DATA, buf, MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_CRC);
#else
    mavlink_aslctrl_data_t packet;
    packet.timestamp = timestamp;
    packet.h = h;
    packet.hRef = hRef;
    packet.hRef_t = hRef_t;
    packet.PitchAngle = PitchAngle;
    packet.PitchAngleRef = PitchAngleRef;
    packet.q = q;
    packet.qRef = qRef;
    packet.uElev = uElev;
    packet.uThrot = uThrot;
    packet.uThrot2 = uThrot2;
    packet.nZ = nZ;
    packet.AirspeedRef = AirspeedRef;
    packet.YawAngle = YawAngle;
    packet.YawAngleRef = YawAngleRef;
    packet.RollAngle = RollAngle;
    packet.RollAngleRef = RollAngleRef;
    packet.p = p;
    packet.pRef = pRef;
    packet.r = r;
    packet.rRef = rRef;
    packet.uAil = uAil;
    packet.uRud = uRud;
    packet.aslctrl_mode = aslctrl_mode;
    packet.SpoilersEngaged = SpoilersEngaged;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DATA, (const char *)&packet, MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_CRC);
#endif
}

/**
 * @brief Send a aslctrl_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_aslctrl_data_send_struct(mavlink_channel_t chan, const mavlink_aslctrl_data_t* aslctrl_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_aslctrl_data_send(chan, aslctrl_data->timestamp, aslctrl_data->aslctrl_mode, aslctrl_data->h, aslctrl_data->hRef, aslctrl_data->hRef_t, aslctrl_data->PitchAngle, aslctrl_data->PitchAngleRef, aslctrl_data->q, aslctrl_data->qRef, aslctrl_data->uElev, aslctrl_data->uThrot, aslctrl_data->uThrot2, aslctrl_data->nZ, aslctrl_data->AirspeedRef, aslctrl_data->SpoilersEngaged, aslctrl_data->YawAngle, aslctrl_data->YawAngleRef, aslctrl_data->RollAngle, aslctrl_data->RollAngleRef, aslctrl_data->p, aslctrl_data->pRef, aslctrl_data->r, aslctrl_data->rRef, aslctrl_data->uAil, aslctrl_data->uRud);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DATA, (const char *)aslctrl_data, MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_ASLCTRL_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_aslctrl_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t aslctrl_mode, float h, float hRef, float hRef_t, float PitchAngle, float PitchAngleRef, float q, float qRef, float uElev, float uThrot, float uThrot2, float nZ, float AirspeedRef, uint8_t SpoilersEngaged, float YawAngle, float YawAngleRef, float RollAngle, float RollAngleRef, float p, float pRef, float r, float rRef, float uAil, float uRud)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, h);
    _mav_put_float(buf, 12, hRef);
    _mav_put_float(buf, 16, hRef_t);
    _mav_put_float(buf, 20, PitchAngle);
    _mav_put_float(buf, 24, PitchAngleRef);
    _mav_put_float(buf, 28, q);
    _mav_put_float(buf, 32, qRef);
    _mav_put_float(buf, 36, uElev);
    _mav_put_float(buf, 40, uThrot);
    _mav_put_float(buf, 44, uThrot2);
    _mav_put_float(buf, 48, nZ);
    _mav_put_float(buf, 52, AirspeedRef);
    _mav_put_float(buf, 56, YawAngle);
    _mav_put_float(buf, 60, YawAngleRef);
    _mav_put_float(buf, 64, RollAngle);
    _mav_put_float(buf, 68, RollAngleRef);
    _mav_put_float(buf, 72, p);
    _mav_put_float(buf, 76, pRef);
    _mav_put_float(buf, 80, r);
    _mav_put_float(buf, 84, rRef);
    _mav_put_float(buf, 88, uAil);
    _mav_put_float(buf, 92, uRud);
    _mav_put_uint8_t(buf, 96, aslctrl_mode);
    _mav_put_uint8_t(buf, 97, SpoilersEngaged);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DATA, buf, MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_CRC);
#else
    mavlink_aslctrl_data_t *packet = (mavlink_aslctrl_data_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->h = h;
    packet->hRef = hRef;
    packet->hRef_t = hRef_t;
    packet->PitchAngle = PitchAngle;
    packet->PitchAngleRef = PitchAngleRef;
    packet->q = q;
    packet->qRef = qRef;
    packet->uElev = uElev;
    packet->uThrot = uThrot;
    packet->uThrot2 = uThrot2;
    packet->nZ = nZ;
    packet->AirspeedRef = AirspeedRef;
    packet->YawAngle = YawAngle;
    packet->YawAngleRef = YawAngleRef;
    packet->RollAngle = RollAngle;
    packet->RollAngleRef = RollAngleRef;
    packet->p = p;
    packet->pRef = pRef;
    packet->r = r;
    packet->rRef = rRef;
    packet->uAil = uAil;
    packet->uRud = uRud;
    packet->aslctrl_mode = aslctrl_mode;
    packet->SpoilersEngaged = SpoilersEngaged;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ASLCTRL_DATA, (const char *)packet, MAVLINK_MSG_ID_ASLCTRL_DATA_MIN_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN, MAVLINK_MSG_ID_ASLCTRL_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE ASLCTRL_DATA UNPACKING


/**
 * @brief Get field timestamp from aslctrl_data message
 *
 * @return [us]  Timestamp
 */
static inline uint64_t mavlink_msg_aslctrl_data_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field aslctrl_mode from aslctrl_data message
 *
 * @return   ASLCTRL control-mode (manual, stabilized, auto, etc...)
 */
static inline uint8_t mavlink_msg_aslctrl_data_get_aslctrl_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  96);
}

/**
 * @brief Get field h from aslctrl_data message
 *
 * @return   See sourcecode for a description of these values... 
 */
static inline float mavlink_msg_aslctrl_data_get_h(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field hRef from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_hRef(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field hRef_t from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_hRef_t(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field PitchAngle from aslctrl_data message
 *
 * @return [deg] Pitch angle
 */
static inline float mavlink_msg_aslctrl_data_get_PitchAngle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field PitchAngleRef from aslctrl_data message
 *
 * @return [deg] Pitch angle reference
 */
static inline float mavlink_msg_aslctrl_data_get_PitchAngleRef(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field q from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_q(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field qRef from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_qRef(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field uElev from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_uElev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field uThrot from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_uThrot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field uThrot2 from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_uThrot2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field nZ from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_nZ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field AirspeedRef from aslctrl_data message
 *
 * @return [m/s] Airspeed reference
 */
static inline float mavlink_msg_aslctrl_data_get_AirspeedRef(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field SpoilersEngaged from aslctrl_data message
 *
 * @return   
 */
static inline uint8_t mavlink_msg_aslctrl_data_get_SpoilersEngaged(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  97);
}

/**
 * @brief Get field YawAngle from aslctrl_data message
 *
 * @return [deg] Yaw angle
 */
static inline float mavlink_msg_aslctrl_data_get_YawAngle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field YawAngleRef from aslctrl_data message
 *
 * @return [deg] Yaw angle reference
 */
static inline float mavlink_msg_aslctrl_data_get_YawAngleRef(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field RollAngle from aslctrl_data message
 *
 * @return [deg] Roll angle
 */
static inline float mavlink_msg_aslctrl_data_get_RollAngle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field RollAngleRef from aslctrl_data message
 *
 * @return [deg] Roll angle reference
 */
static inline float mavlink_msg_aslctrl_data_get_RollAngleRef(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field p from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_p(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field pRef from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_pRef(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field r from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_r(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field rRef from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_rRef(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field uAil from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_uAil(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field uRud from aslctrl_data message
 *
 * @return   
 */
static inline float mavlink_msg_aslctrl_data_get_uRud(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Decode a aslctrl_data message into a struct
 *
 * @param msg The message to decode
 * @param aslctrl_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_aslctrl_data_decode(const mavlink_message_t* msg, mavlink_aslctrl_data_t* aslctrl_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    aslctrl_data->timestamp = mavlink_msg_aslctrl_data_get_timestamp(msg);
    aslctrl_data->h = mavlink_msg_aslctrl_data_get_h(msg);
    aslctrl_data->hRef = mavlink_msg_aslctrl_data_get_hRef(msg);
    aslctrl_data->hRef_t = mavlink_msg_aslctrl_data_get_hRef_t(msg);
    aslctrl_data->PitchAngle = mavlink_msg_aslctrl_data_get_PitchAngle(msg);
    aslctrl_data->PitchAngleRef = mavlink_msg_aslctrl_data_get_PitchAngleRef(msg);
    aslctrl_data->q = mavlink_msg_aslctrl_data_get_q(msg);
    aslctrl_data->qRef = mavlink_msg_aslctrl_data_get_qRef(msg);
    aslctrl_data->uElev = mavlink_msg_aslctrl_data_get_uElev(msg);
    aslctrl_data->uThrot = mavlink_msg_aslctrl_data_get_uThrot(msg);
    aslctrl_data->uThrot2 = mavlink_msg_aslctrl_data_get_uThrot2(msg);
    aslctrl_data->nZ = mavlink_msg_aslctrl_data_get_nZ(msg);
    aslctrl_data->AirspeedRef = mavlink_msg_aslctrl_data_get_AirspeedRef(msg);
    aslctrl_data->YawAngle = mavlink_msg_aslctrl_data_get_YawAngle(msg);
    aslctrl_data->YawAngleRef = mavlink_msg_aslctrl_data_get_YawAngleRef(msg);
    aslctrl_data->RollAngle = mavlink_msg_aslctrl_data_get_RollAngle(msg);
    aslctrl_data->RollAngleRef = mavlink_msg_aslctrl_data_get_RollAngleRef(msg);
    aslctrl_data->p = mavlink_msg_aslctrl_data_get_p(msg);
    aslctrl_data->pRef = mavlink_msg_aslctrl_data_get_pRef(msg);
    aslctrl_data->r = mavlink_msg_aslctrl_data_get_r(msg);
    aslctrl_data->rRef = mavlink_msg_aslctrl_data_get_rRef(msg);
    aslctrl_data->uAil = mavlink_msg_aslctrl_data_get_uAil(msg);
    aslctrl_data->uRud = mavlink_msg_aslctrl_data_get_uRud(msg);
    aslctrl_data->aslctrl_mode = mavlink_msg_aslctrl_data_get_aslctrl_mode(msg);
    aslctrl_data->SpoilersEngaged = mavlink_msg_aslctrl_data_get_SpoilersEngaged(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ASLCTRL_DATA_LEN? msg->len : MAVLINK_MSG_ID_ASLCTRL_DATA_LEN;
        memset(aslctrl_data, 0, MAVLINK_MSG_ID_ASLCTRL_DATA_LEN);
    memcpy(aslctrl_data, _MAV_PAYLOAD(msg), len);
#endif
}
