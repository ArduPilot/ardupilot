#pragma once
// MESSAGE UAVIONIX_ADSB_OUT_DYNAMIC PACKING

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC 10002


typedef struct __mavlink_uavionix_adsb_out_dynamic_t {
 uint32_t utcTime; /*< [s] UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX*/
 int32_t gpsLat; /*< [degE7] Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX*/
 int32_t gpsLon; /*< [degE7] Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX*/
 int32_t gpsAlt; /*< [mm] Altitude (WGS84). UP +ve. If unknown set to INT32_MAX*/
 int32_t baroAltMSL; /*< [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX*/
 uint32_t accuracyHor; /*< [mm] Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX*/
 uint16_t accuracyVert; /*< [cm] Vertical accuracy in cm. If unknown set to UINT16_MAX*/
 uint16_t accuracyVel; /*< [mm/s] Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX*/
 int16_t velVert; /*< [cm/s] GPS vertical speed in cm/s. If unknown set to INT16_MAX*/
 int16_t velNS; /*< [cm/s] North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX*/
 int16_t VelEW; /*< [cm/s] East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX*/
 uint16_t state; /*<  ADS-B transponder dynamic input state flags*/
 uint16_t squawk; /*<  Mode A code (typically 1200 [0x04B0] for VFR)*/
 uint8_t gpsFix; /*<  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK*/
 uint8_t numSats; /*<  Number of satellites visible. If unknown set to UINT8_MAX*/
 uint8_t emergencyStatus; /*<  Emergency status*/
} mavlink_uavionix_adsb_out_dynamic_t;

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN 41
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN 41
#define MAVLINK_MSG_ID_10002_LEN 41
#define MAVLINK_MSG_ID_10002_MIN_LEN 41

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_CRC 186
#define MAVLINK_MSG_ID_10002_CRC 186



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_DYNAMIC { \
    10002, \
    "UAVIONIX_ADSB_OUT_DYNAMIC", \
    16, \
    {  { "utcTime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavionix_adsb_out_dynamic_t, utcTime) }, \
         { "gpsLat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_uavionix_adsb_out_dynamic_t, gpsLat) }, \
         { "gpsLon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_uavionix_adsb_out_dynamic_t, gpsLon) }, \
         { "gpsAlt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_uavionix_adsb_out_dynamic_t, gpsAlt) }, \
         { "gpsFix", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_uavionix_adsb_out_dynamic_t, gpsFix) }, \
         { "numSats", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_uavionix_adsb_out_dynamic_t, numSats) }, \
         { "baroAltMSL", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_uavionix_adsb_out_dynamic_t, baroAltMSL) }, \
         { "accuracyHor", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_uavionix_adsb_out_dynamic_t, accuracyHor) }, \
         { "accuracyVert", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_uavionix_adsb_out_dynamic_t, accuracyVert) }, \
         { "accuracyVel", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_uavionix_adsb_out_dynamic_t, accuracyVel) }, \
         { "velVert", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_uavionix_adsb_out_dynamic_t, velVert) }, \
         { "velNS", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_uavionix_adsb_out_dynamic_t, velNS) }, \
         { "VelEW", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_uavionix_adsb_out_dynamic_t, VelEW) }, \
         { "emergencyStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_uavionix_adsb_out_dynamic_t, emergencyStatus) }, \
         { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_uavionix_adsb_out_dynamic_t, state) }, \
         { "squawk", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_uavionix_adsb_out_dynamic_t, squawk) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_DYNAMIC { \
    "UAVIONIX_ADSB_OUT_DYNAMIC", \
    16, \
    {  { "utcTime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavionix_adsb_out_dynamic_t, utcTime) }, \
         { "gpsLat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_uavionix_adsb_out_dynamic_t, gpsLat) }, \
         { "gpsLon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_uavionix_adsb_out_dynamic_t, gpsLon) }, \
         { "gpsAlt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_uavionix_adsb_out_dynamic_t, gpsAlt) }, \
         { "gpsFix", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_uavionix_adsb_out_dynamic_t, gpsFix) }, \
         { "numSats", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_uavionix_adsb_out_dynamic_t, numSats) }, \
         { "baroAltMSL", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_uavionix_adsb_out_dynamic_t, baroAltMSL) }, \
         { "accuracyHor", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_uavionix_adsb_out_dynamic_t, accuracyHor) }, \
         { "accuracyVert", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_uavionix_adsb_out_dynamic_t, accuracyVert) }, \
         { "accuracyVel", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_uavionix_adsb_out_dynamic_t, accuracyVel) }, \
         { "velVert", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_uavionix_adsb_out_dynamic_t, velVert) }, \
         { "velNS", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_uavionix_adsb_out_dynamic_t, velNS) }, \
         { "VelEW", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_uavionix_adsb_out_dynamic_t, VelEW) }, \
         { "emergencyStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_uavionix_adsb_out_dynamic_t, emergencyStatus) }, \
         { "state", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_uavionix_adsb_out_dynamic_t, state) }, \
         { "squawk", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_uavionix_adsb_out_dynamic_t, squawk) }, \
         } \
}
#endif

/**
 * @brief Pack a uavionix_adsb_out_dynamic message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param utcTime [s] UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
 * @param gpsLat [degE7] Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
 * @param gpsLon [degE7] Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
 * @param gpsAlt [mm] Altitude (WGS84). UP +ve. If unknown set to INT32_MAX
 * @param gpsFix  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
 * @param numSats  Number of satellites visible. If unknown set to UINT8_MAX
 * @param baroAltMSL [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
 * @param accuracyHor [mm] Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
 * @param accuracyVert [cm] Vertical accuracy in cm. If unknown set to UINT16_MAX
 * @param accuracyVel [mm/s] Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
 * @param velVert [cm/s] GPS vertical speed in cm/s. If unknown set to INT16_MAX
 * @param velNS [cm/s] North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
 * @param VelEW [cm/s] East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
 * @param emergencyStatus  Emergency status
 * @param state  ADS-B transponder dynamic input state flags
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_dynamic_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN];
    _mav_put_uint32_t(buf, 0, utcTime);
    _mav_put_int32_t(buf, 4, gpsLat);
    _mav_put_int32_t(buf, 8, gpsLon);
    _mav_put_int32_t(buf, 12, gpsAlt);
    _mav_put_int32_t(buf, 16, baroAltMSL);
    _mav_put_uint32_t(buf, 20, accuracyHor);
    _mav_put_uint16_t(buf, 24, accuracyVert);
    _mav_put_uint16_t(buf, 26, accuracyVel);
    _mav_put_int16_t(buf, 28, velVert);
    _mav_put_int16_t(buf, 30, velNS);
    _mav_put_int16_t(buf, 32, VelEW);
    _mav_put_uint16_t(buf, 34, state);
    _mav_put_uint16_t(buf, 36, squawk);
    _mav_put_uint8_t(buf, 38, gpsFix);
    _mav_put_uint8_t(buf, 39, numSats);
    _mav_put_uint8_t(buf, 40, emergencyStatus);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN);
#else
    mavlink_uavionix_adsb_out_dynamic_t packet;
    packet.utcTime = utcTime;
    packet.gpsLat = gpsLat;
    packet.gpsLon = gpsLon;
    packet.gpsAlt = gpsAlt;
    packet.baroAltMSL = baroAltMSL;
    packet.accuracyHor = accuracyHor;
    packet.accuracyVert = accuracyVert;
    packet.accuracyVel = accuracyVel;
    packet.velVert = velVert;
    packet.velNS = velNS;
    packet.VelEW = VelEW;
    packet.state = state;
    packet.squawk = squawk;
    packet.gpsFix = gpsFix;
    packet.numSats = numSats;
    packet.emergencyStatus = emergencyStatus;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_CRC);
}

/**
 * @brief Pack a uavionix_adsb_out_dynamic message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param utcTime [s] UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
 * @param gpsLat [degE7] Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
 * @param gpsLon [degE7] Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
 * @param gpsAlt [mm] Altitude (WGS84). UP +ve. If unknown set to INT32_MAX
 * @param gpsFix  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
 * @param numSats  Number of satellites visible. If unknown set to UINT8_MAX
 * @param baroAltMSL [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
 * @param accuracyHor [mm] Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
 * @param accuracyVert [cm] Vertical accuracy in cm. If unknown set to UINT16_MAX
 * @param accuracyVel [mm/s] Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
 * @param velVert [cm/s] GPS vertical speed in cm/s. If unknown set to INT16_MAX
 * @param velNS [cm/s] North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
 * @param VelEW [cm/s] East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
 * @param emergencyStatus  Emergency status
 * @param state  ADS-B transponder dynamic input state flags
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_dynamic_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t utcTime,int32_t gpsLat,int32_t gpsLon,int32_t gpsAlt,uint8_t gpsFix,uint8_t numSats,int32_t baroAltMSL,uint32_t accuracyHor,uint16_t accuracyVert,uint16_t accuracyVel,int16_t velVert,int16_t velNS,int16_t VelEW,uint8_t emergencyStatus,uint16_t state,uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN];
    _mav_put_uint32_t(buf, 0, utcTime);
    _mav_put_int32_t(buf, 4, gpsLat);
    _mav_put_int32_t(buf, 8, gpsLon);
    _mav_put_int32_t(buf, 12, gpsAlt);
    _mav_put_int32_t(buf, 16, baroAltMSL);
    _mav_put_uint32_t(buf, 20, accuracyHor);
    _mav_put_uint16_t(buf, 24, accuracyVert);
    _mav_put_uint16_t(buf, 26, accuracyVel);
    _mav_put_int16_t(buf, 28, velVert);
    _mav_put_int16_t(buf, 30, velNS);
    _mav_put_int16_t(buf, 32, VelEW);
    _mav_put_uint16_t(buf, 34, state);
    _mav_put_uint16_t(buf, 36, squawk);
    _mav_put_uint8_t(buf, 38, gpsFix);
    _mav_put_uint8_t(buf, 39, numSats);
    _mav_put_uint8_t(buf, 40, emergencyStatus);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN);
#else
    mavlink_uavionix_adsb_out_dynamic_t packet;
    packet.utcTime = utcTime;
    packet.gpsLat = gpsLat;
    packet.gpsLon = gpsLon;
    packet.gpsAlt = gpsAlt;
    packet.baroAltMSL = baroAltMSL;
    packet.accuracyHor = accuracyHor;
    packet.accuracyVert = accuracyVert;
    packet.accuracyVel = accuracyVel;
    packet.velVert = velVert;
    packet.velNS = velNS;
    packet.VelEW = VelEW;
    packet.state = state;
    packet.squawk = squawk;
    packet.gpsFix = gpsFix;
    packet.numSats = numSats;
    packet.emergencyStatus = emergencyStatus;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_CRC);
}

/**
 * @brief Encode a uavionix_adsb_out_dynamic struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_dynamic C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_dynamic_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_dynamic_t* uavionix_adsb_out_dynamic)
{
    return mavlink_msg_uavionix_adsb_out_dynamic_pack(system_id, component_id, msg, uavionix_adsb_out_dynamic->utcTime, uavionix_adsb_out_dynamic->gpsLat, uavionix_adsb_out_dynamic->gpsLon, uavionix_adsb_out_dynamic->gpsAlt, uavionix_adsb_out_dynamic->gpsFix, uavionix_adsb_out_dynamic->numSats, uavionix_adsb_out_dynamic->baroAltMSL, uavionix_adsb_out_dynamic->accuracyHor, uavionix_adsb_out_dynamic->accuracyVert, uavionix_adsb_out_dynamic->accuracyVel, uavionix_adsb_out_dynamic->velVert, uavionix_adsb_out_dynamic->velNS, uavionix_adsb_out_dynamic->VelEW, uavionix_adsb_out_dynamic->emergencyStatus, uavionix_adsb_out_dynamic->state, uavionix_adsb_out_dynamic->squawk);
}

/**
 * @brief Encode a uavionix_adsb_out_dynamic struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_dynamic C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_dynamic_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_dynamic_t* uavionix_adsb_out_dynamic)
{
    return mavlink_msg_uavionix_adsb_out_dynamic_pack_chan(system_id, component_id, chan, msg, uavionix_adsb_out_dynamic->utcTime, uavionix_adsb_out_dynamic->gpsLat, uavionix_adsb_out_dynamic->gpsLon, uavionix_adsb_out_dynamic->gpsAlt, uavionix_adsb_out_dynamic->gpsFix, uavionix_adsb_out_dynamic->numSats, uavionix_adsb_out_dynamic->baroAltMSL, uavionix_adsb_out_dynamic->accuracyHor, uavionix_adsb_out_dynamic->accuracyVert, uavionix_adsb_out_dynamic->accuracyVel, uavionix_adsb_out_dynamic->velVert, uavionix_adsb_out_dynamic->velNS, uavionix_adsb_out_dynamic->VelEW, uavionix_adsb_out_dynamic->emergencyStatus, uavionix_adsb_out_dynamic->state, uavionix_adsb_out_dynamic->squawk);
}

/**
 * @brief Send a uavionix_adsb_out_dynamic message
 * @param chan MAVLink channel to send the message
 *
 * @param utcTime [s] UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
 * @param gpsLat [degE7] Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
 * @param gpsLon [degE7] Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
 * @param gpsAlt [mm] Altitude (WGS84). UP +ve. If unknown set to INT32_MAX
 * @param gpsFix  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
 * @param numSats  Number of satellites visible. If unknown set to UINT8_MAX
 * @param baroAltMSL [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
 * @param accuracyHor [mm] Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
 * @param accuracyVert [cm] Vertical accuracy in cm. If unknown set to UINT16_MAX
 * @param accuracyVel [mm/s] Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
 * @param velVert [cm/s] GPS vertical speed in cm/s. If unknown set to INT16_MAX
 * @param velNS [cm/s] North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
 * @param VelEW [cm/s] East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
 * @param emergencyStatus  Emergency status
 * @param state  ADS-B transponder dynamic input state flags
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavionix_adsb_out_dynamic_send(mavlink_channel_t chan, uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN];
    _mav_put_uint32_t(buf, 0, utcTime);
    _mav_put_int32_t(buf, 4, gpsLat);
    _mav_put_int32_t(buf, 8, gpsLon);
    _mav_put_int32_t(buf, 12, gpsAlt);
    _mav_put_int32_t(buf, 16, baroAltMSL);
    _mav_put_uint32_t(buf, 20, accuracyHor);
    _mav_put_uint16_t(buf, 24, accuracyVert);
    _mav_put_uint16_t(buf, 26, accuracyVel);
    _mav_put_int16_t(buf, 28, velVert);
    _mav_put_int16_t(buf, 30, velNS);
    _mav_put_int16_t(buf, 32, VelEW);
    _mav_put_uint16_t(buf, 34, state);
    _mav_put_uint16_t(buf, 36, squawk);
    _mav_put_uint8_t(buf, 38, gpsFix);
    _mav_put_uint8_t(buf, 39, numSats);
    _mav_put_uint8_t(buf, 40, emergencyStatus);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_CRC);
#else
    mavlink_uavionix_adsb_out_dynamic_t packet;
    packet.utcTime = utcTime;
    packet.gpsLat = gpsLat;
    packet.gpsLon = gpsLon;
    packet.gpsAlt = gpsAlt;
    packet.baroAltMSL = baroAltMSL;
    packet.accuracyHor = accuracyHor;
    packet.accuracyVert = accuracyVert;
    packet.accuracyVel = accuracyVel;
    packet.velVert = velVert;
    packet.velNS = velNS;
    packet.VelEW = VelEW;
    packet.state = state;
    packet.squawk = squawk;
    packet.gpsFix = gpsFix;
    packet.numSats = numSats;
    packet.emergencyStatus = emergencyStatus;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC, (const char *)&packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_CRC);
#endif
}

/**
 * @brief Send a uavionix_adsb_out_dynamic message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavionix_adsb_out_dynamic_send_struct(mavlink_channel_t chan, const mavlink_uavionix_adsb_out_dynamic_t* uavionix_adsb_out_dynamic)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_out_dynamic_send(chan, uavionix_adsb_out_dynamic->utcTime, uavionix_adsb_out_dynamic->gpsLat, uavionix_adsb_out_dynamic->gpsLon, uavionix_adsb_out_dynamic->gpsAlt, uavionix_adsb_out_dynamic->gpsFix, uavionix_adsb_out_dynamic->numSats, uavionix_adsb_out_dynamic->baroAltMSL, uavionix_adsb_out_dynamic->accuracyHor, uavionix_adsb_out_dynamic->accuracyVert, uavionix_adsb_out_dynamic->accuracyVel, uavionix_adsb_out_dynamic->velVert, uavionix_adsb_out_dynamic->velNS, uavionix_adsb_out_dynamic->VelEW, uavionix_adsb_out_dynamic->emergencyStatus, uavionix_adsb_out_dynamic->state, uavionix_adsb_out_dynamic->squawk);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC, (const char *)uavionix_adsb_out_dynamic, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavionix_adsb_out_dynamic_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t utcTime, int32_t gpsLat, int32_t gpsLon, int32_t gpsAlt, uint8_t gpsFix, uint8_t numSats, int32_t baroAltMSL, uint32_t accuracyHor, uint16_t accuracyVert, uint16_t accuracyVel, int16_t velVert, int16_t velNS, int16_t VelEW, uint8_t emergencyStatus, uint16_t state, uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, utcTime);
    _mav_put_int32_t(buf, 4, gpsLat);
    _mav_put_int32_t(buf, 8, gpsLon);
    _mav_put_int32_t(buf, 12, gpsAlt);
    _mav_put_int32_t(buf, 16, baroAltMSL);
    _mav_put_uint32_t(buf, 20, accuracyHor);
    _mav_put_uint16_t(buf, 24, accuracyVert);
    _mav_put_uint16_t(buf, 26, accuracyVel);
    _mav_put_int16_t(buf, 28, velVert);
    _mav_put_int16_t(buf, 30, velNS);
    _mav_put_int16_t(buf, 32, VelEW);
    _mav_put_uint16_t(buf, 34, state);
    _mav_put_uint16_t(buf, 36, squawk);
    _mav_put_uint8_t(buf, 38, gpsFix);
    _mav_put_uint8_t(buf, 39, numSats);
    _mav_put_uint8_t(buf, 40, emergencyStatus);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_CRC);
#else
    mavlink_uavionix_adsb_out_dynamic_t *packet = (mavlink_uavionix_adsb_out_dynamic_t *)msgbuf;
    packet->utcTime = utcTime;
    packet->gpsLat = gpsLat;
    packet->gpsLon = gpsLon;
    packet->gpsAlt = gpsAlt;
    packet->baroAltMSL = baroAltMSL;
    packet->accuracyHor = accuracyHor;
    packet->accuracyVert = accuracyVert;
    packet->accuracyVel = accuracyVel;
    packet->velVert = velVert;
    packet->velNS = velNS;
    packet->VelEW = VelEW;
    packet->state = state;
    packet->squawk = squawk;
    packet->gpsFix = gpsFix;
    packet->numSats = numSats;
    packet->emergencyStatus = emergencyStatus;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC, (const char *)packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVIONIX_ADSB_OUT_DYNAMIC UNPACKING


/**
 * @brief Get field utcTime from uavionix_adsb_out_dynamic message
 *
 * @return [s] UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
 */
static inline uint32_t mavlink_msg_uavionix_adsb_out_dynamic_get_utcTime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field gpsLat from uavionix_adsb_out_dynamic message
 *
 * @return [degE7] Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
 */
static inline int32_t mavlink_msg_uavionix_adsb_out_dynamic_get_gpsLat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field gpsLon from uavionix_adsb_out_dynamic message
 *
 * @return [degE7] Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
 */
static inline int32_t mavlink_msg_uavionix_adsb_out_dynamic_get_gpsLon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field gpsAlt from uavionix_adsb_out_dynamic message
 *
 * @return [mm] Altitude (WGS84). UP +ve. If unknown set to INT32_MAX
 */
static inline int32_t mavlink_msg_uavionix_adsb_out_dynamic_get_gpsAlt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field gpsFix from uavionix_adsb_out_dynamic message
 *
 * @return  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_dynamic_get_gpsFix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field numSats from uavionix_adsb_out_dynamic message
 *
 * @return  Number of satellites visible. If unknown set to UINT8_MAX
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_dynamic_get_numSats(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field baroAltMSL from uavionix_adsb_out_dynamic message
 *
 * @return [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
 */
static inline int32_t mavlink_msg_uavionix_adsb_out_dynamic_get_baroAltMSL(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field accuracyHor from uavionix_adsb_out_dynamic message
 *
 * @return [mm] Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
 */
static inline uint32_t mavlink_msg_uavionix_adsb_out_dynamic_get_accuracyHor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field accuracyVert from uavionix_adsb_out_dynamic message
 *
 * @return [cm] Vertical accuracy in cm. If unknown set to UINT16_MAX
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_dynamic_get_accuracyVert(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field accuracyVel from uavionix_adsb_out_dynamic message
 *
 * @return [mm/s] Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_dynamic_get_accuracyVel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field velVert from uavionix_adsb_out_dynamic message
 *
 * @return [cm/s] GPS vertical speed in cm/s. If unknown set to INT16_MAX
 */
static inline int16_t mavlink_msg_uavionix_adsb_out_dynamic_get_velVert(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field velNS from uavionix_adsb_out_dynamic message
 *
 * @return [cm/s] North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
 */
static inline int16_t mavlink_msg_uavionix_adsb_out_dynamic_get_velNS(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field VelEW from uavionix_adsb_out_dynamic message
 *
 * @return [cm/s] East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
 */
static inline int16_t mavlink_msg_uavionix_adsb_out_dynamic_get_VelEW(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field emergencyStatus from uavionix_adsb_out_dynamic message
 *
 * @return  Emergency status
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_dynamic_get_emergencyStatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field state from uavionix_adsb_out_dynamic message
 *
 * @return  ADS-B transponder dynamic input state flags
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_dynamic_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field squawk from uavionix_adsb_out_dynamic message
 *
 * @return  Mode A code (typically 1200 [0x04B0] for VFR)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_dynamic_get_squawk(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Decode a uavionix_adsb_out_dynamic message into a struct
 *
 * @param msg The message to decode
 * @param uavionix_adsb_out_dynamic C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavionix_adsb_out_dynamic_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_dynamic_t* uavionix_adsb_out_dynamic)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavionix_adsb_out_dynamic->utcTime = mavlink_msg_uavionix_adsb_out_dynamic_get_utcTime(msg);
    uavionix_adsb_out_dynamic->gpsLat = mavlink_msg_uavionix_adsb_out_dynamic_get_gpsLat(msg);
    uavionix_adsb_out_dynamic->gpsLon = mavlink_msg_uavionix_adsb_out_dynamic_get_gpsLon(msg);
    uavionix_adsb_out_dynamic->gpsAlt = mavlink_msg_uavionix_adsb_out_dynamic_get_gpsAlt(msg);
    uavionix_adsb_out_dynamic->baroAltMSL = mavlink_msg_uavionix_adsb_out_dynamic_get_baroAltMSL(msg);
    uavionix_adsb_out_dynamic->accuracyHor = mavlink_msg_uavionix_adsb_out_dynamic_get_accuracyHor(msg);
    uavionix_adsb_out_dynamic->accuracyVert = mavlink_msg_uavionix_adsb_out_dynamic_get_accuracyVert(msg);
    uavionix_adsb_out_dynamic->accuracyVel = mavlink_msg_uavionix_adsb_out_dynamic_get_accuracyVel(msg);
    uavionix_adsb_out_dynamic->velVert = mavlink_msg_uavionix_adsb_out_dynamic_get_velVert(msg);
    uavionix_adsb_out_dynamic->velNS = mavlink_msg_uavionix_adsb_out_dynamic_get_velNS(msg);
    uavionix_adsb_out_dynamic->VelEW = mavlink_msg_uavionix_adsb_out_dynamic_get_VelEW(msg);
    uavionix_adsb_out_dynamic->state = mavlink_msg_uavionix_adsb_out_dynamic_get_state(msg);
    uavionix_adsb_out_dynamic->squawk = mavlink_msg_uavionix_adsb_out_dynamic_get_squawk(msg);
    uavionix_adsb_out_dynamic->gpsFix = mavlink_msg_uavionix_adsb_out_dynamic_get_gpsFix(msg);
    uavionix_adsb_out_dynamic->numSats = mavlink_msg_uavionix_adsb_out_dynamic_get_numSats(msg);
    uavionix_adsb_out_dynamic->emergencyStatus = mavlink_msg_uavionix_adsb_out_dynamic_get_emergencyStatus(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN? msg->len : MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN;
        memset(uavionix_adsb_out_dynamic, 0, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_LEN);
    memcpy(uavionix_adsb_out_dynamic, _MAV_PAYLOAD(msg), len);
#endif
}
