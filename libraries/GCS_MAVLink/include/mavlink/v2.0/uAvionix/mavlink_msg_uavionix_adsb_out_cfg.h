#pragma once
// MESSAGE UAVIONIX_ADSB_OUT_CFG PACKING

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG 10001


typedef struct __mavlink_uavionix_adsb_out_cfg_t {
 uint32_t ICAO; /*<  Vehicle address (24 bit)*/
 uint16_t stallSpeed; /*< [cm/s] Aircraft stall speed in cm/s*/
 char callsign[9]; /*<  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)*/
 uint8_t emitterType; /*<  Transmitting vehicle type. See ADSB_EMITTER_TYPE enum*/
 uint8_t aircraftSize; /*<  Aircraft length and width encoding (table 2-35 of DO-282B)*/
 uint8_t gpsOffsetLat; /*<  GPS antenna lateral offset (table 2-36 of DO-282B)*/
 uint8_t gpsOffsetLon; /*<  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B)*/
 uint8_t rfSelect; /*<  ADS-B transponder reciever and transmit enable flags*/
} mavlink_uavionix_adsb_out_cfg_t;

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN 20
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN 20
#define MAVLINK_MSG_ID_10001_LEN 20
#define MAVLINK_MSG_ID_10001_MIN_LEN 20

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC 209
#define MAVLINK_MSG_ID_10001_CRC 209

#define MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN 9

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_CFG { \
    10001, \
    "UAVIONIX_ADSB_OUT_CFG", \
    8, \
    {  { "ICAO", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavionix_adsb_out_cfg_t, ICAO) }, \
         { "callsign", NULL, MAVLINK_TYPE_CHAR, 9, 6, offsetof(mavlink_uavionix_adsb_out_cfg_t, callsign) }, \
         { "emitterType", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_uavionix_adsb_out_cfg_t, emitterType) }, \
         { "aircraftSize", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_uavionix_adsb_out_cfg_t, aircraftSize) }, \
         { "gpsOffsetLat", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_uavionix_adsb_out_cfg_t, gpsOffsetLat) }, \
         { "gpsOffsetLon", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_uavionix_adsb_out_cfg_t, gpsOffsetLon) }, \
         { "stallSpeed", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_uavionix_adsb_out_cfg_t, stallSpeed) }, \
         { "rfSelect", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_uavionix_adsb_out_cfg_t, rfSelect) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_CFG { \
    "UAVIONIX_ADSB_OUT_CFG", \
    8, \
    {  { "ICAO", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_uavionix_adsb_out_cfg_t, ICAO) }, \
         { "callsign", NULL, MAVLINK_TYPE_CHAR, 9, 6, offsetof(mavlink_uavionix_adsb_out_cfg_t, callsign) }, \
         { "emitterType", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_uavionix_adsb_out_cfg_t, emitterType) }, \
         { "aircraftSize", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_uavionix_adsb_out_cfg_t, aircraftSize) }, \
         { "gpsOffsetLat", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_uavionix_adsb_out_cfg_t, gpsOffsetLat) }, \
         { "gpsOffsetLon", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_uavionix_adsb_out_cfg_t, gpsOffsetLon) }, \
         { "stallSpeed", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_uavionix_adsb_out_cfg_t, stallSpeed) }, \
         { "rfSelect", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_uavionix_adsb_out_cfg_t, rfSelect) }, \
         } \
}
#endif

/**
 * @brief Pack a uavionix_adsb_out_cfg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ICAO  Vehicle address (24 bit)
 * @param callsign  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
 * @param emitterType  Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
 * @param aircraftSize  Aircraft length and width encoding (table 2-35 of DO-282B)
 * @param gpsOffsetLat  GPS antenna lateral offset (table 2-36 of DO-282B)
 * @param gpsOffsetLon  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B)
 * @param stallSpeed [cm/s] Aircraft stall speed in cm/s
 * @param rfSelect  ADS-B transponder reciever and transmit enable flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t ICAO, const char *callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN];
    _mav_put_uint32_t(buf, 0, ICAO);
    _mav_put_uint16_t(buf, 4, stallSpeed);
    _mav_put_uint8_t(buf, 15, emitterType);
    _mav_put_uint8_t(buf, 16, aircraftSize);
    _mav_put_uint8_t(buf, 17, gpsOffsetLat);
    _mav_put_uint8_t(buf, 18, gpsOffsetLon);
    _mav_put_uint8_t(buf, 19, rfSelect);
    _mav_put_char_array(buf, 6, callsign, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN);
#else
    mavlink_uavionix_adsb_out_cfg_t packet;
    packet.ICAO = ICAO;
    packet.stallSpeed = stallSpeed;
    packet.emitterType = emitterType;
    packet.aircraftSize = aircraftSize;
    packet.gpsOffsetLat = gpsOffsetLat;
    packet.gpsOffsetLon = gpsOffsetLon;
    packet.rfSelect = rfSelect;
    mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC);
}

/**
 * @brief Pack a uavionix_adsb_out_cfg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param ICAO  Vehicle address (24 bit)
 * @param callsign  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
 * @param emitterType  Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
 * @param aircraftSize  Aircraft length and width encoding (table 2-35 of DO-282B)
 * @param gpsOffsetLat  GPS antenna lateral offset (table 2-36 of DO-282B)
 * @param gpsOffsetLon  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B)
 * @param stallSpeed [cm/s] Aircraft stall speed in cm/s
 * @param rfSelect  ADS-B transponder reciever and transmit enable flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t ICAO, const char *callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN];
    _mav_put_uint32_t(buf, 0, ICAO);
    _mav_put_uint16_t(buf, 4, stallSpeed);
    _mav_put_uint8_t(buf, 15, emitterType);
    _mav_put_uint8_t(buf, 16, aircraftSize);
    _mav_put_uint8_t(buf, 17, gpsOffsetLat);
    _mav_put_uint8_t(buf, 18, gpsOffsetLon);
    _mav_put_uint8_t(buf, 19, rfSelect);
    _mav_put_char_array(buf, 6, callsign, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN);
#else
    mavlink_uavionix_adsb_out_cfg_t packet;
    packet.ICAO = ICAO;
    packet.stallSpeed = stallSpeed;
    packet.emitterType = emitterType;
    packet.aircraftSize = aircraftSize;
    packet.gpsOffsetLat = gpsOffsetLat;
    packet.gpsOffsetLon = gpsOffsetLon;
    packet.rfSelect = rfSelect;
    mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN);
#endif
}

/**
 * @brief Pack a uavionix_adsb_out_cfg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ICAO  Vehicle address (24 bit)
 * @param callsign  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
 * @param emitterType  Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
 * @param aircraftSize  Aircraft length and width encoding (table 2-35 of DO-282B)
 * @param gpsOffsetLat  GPS antenna lateral offset (table 2-36 of DO-282B)
 * @param gpsOffsetLon  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B)
 * @param stallSpeed [cm/s] Aircraft stall speed in cm/s
 * @param rfSelect  ADS-B transponder reciever and transmit enable flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t ICAO,const char *callsign,uint8_t emitterType,uint8_t aircraftSize,uint8_t gpsOffsetLat,uint8_t gpsOffsetLon,uint16_t stallSpeed,uint8_t rfSelect)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN];
    _mav_put_uint32_t(buf, 0, ICAO);
    _mav_put_uint16_t(buf, 4, stallSpeed);
    _mav_put_uint8_t(buf, 15, emitterType);
    _mav_put_uint8_t(buf, 16, aircraftSize);
    _mav_put_uint8_t(buf, 17, gpsOffsetLat);
    _mav_put_uint8_t(buf, 18, gpsOffsetLon);
    _mav_put_uint8_t(buf, 19, rfSelect);
    _mav_put_char_array(buf, 6, callsign, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN);
#else
    mavlink_uavionix_adsb_out_cfg_t packet;
    packet.ICAO = ICAO;
    packet.stallSpeed = stallSpeed;
    packet.emitterType = emitterType;
    packet.aircraftSize = aircraftSize;
    packet.gpsOffsetLat = gpsOffsetLat;
    packet.gpsOffsetLon = gpsOffsetLon;
    packet.rfSelect = rfSelect;
    mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC);
}

/**
 * @brief Encode a uavionix_adsb_out_cfg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_cfg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_cfg_t* uavionix_adsb_out_cfg)
{
    return mavlink_msg_uavionix_adsb_out_cfg_pack(system_id, component_id, msg, uavionix_adsb_out_cfg->ICAO, uavionix_adsb_out_cfg->callsign, uavionix_adsb_out_cfg->emitterType, uavionix_adsb_out_cfg->aircraftSize, uavionix_adsb_out_cfg->gpsOffsetLat, uavionix_adsb_out_cfg->gpsOffsetLon, uavionix_adsb_out_cfg->stallSpeed, uavionix_adsb_out_cfg->rfSelect);
}

/**
 * @brief Encode a uavionix_adsb_out_cfg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_cfg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_cfg_t* uavionix_adsb_out_cfg)
{
    return mavlink_msg_uavionix_adsb_out_cfg_pack_chan(system_id, component_id, chan, msg, uavionix_adsb_out_cfg->ICAO, uavionix_adsb_out_cfg->callsign, uavionix_adsb_out_cfg->emitterType, uavionix_adsb_out_cfg->aircraftSize, uavionix_adsb_out_cfg->gpsOffsetLat, uavionix_adsb_out_cfg->gpsOffsetLon, uavionix_adsb_out_cfg->stallSpeed, uavionix_adsb_out_cfg->rfSelect);
}

/**
 * @brief Encode a uavionix_adsb_out_cfg struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_cfg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_cfg_t* uavionix_adsb_out_cfg)
{
    return mavlink_msg_uavionix_adsb_out_cfg_pack_status(system_id, component_id, _status, msg,  uavionix_adsb_out_cfg->ICAO, uavionix_adsb_out_cfg->callsign, uavionix_adsb_out_cfg->emitterType, uavionix_adsb_out_cfg->aircraftSize, uavionix_adsb_out_cfg->gpsOffsetLat, uavionix_adsb_out_cfg->gpsOffsetLon, uavionix_adsb_out_cfg->stallSpeed, uavionix_adsb_out_cfg->rfSelect);
}

/**
 * @brief Send a uavionix_adsb_out_cfg message
 * @param chan MAVLink channel to send the message
 *
 * @param ICAO  Vehicle address (24 bit)
 * @param callsign  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
 * @param emitterType  Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
 * @param aircraftSize  Aircraft length and width encoding (table 2-35 of DO-282B)
 * @param gpsOffsetLat  GPS antenna lateral offset (table 2-36 of DO-282B)
 * @param gpsOffsetLon  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B)
 * @param stallSpeed [cm/s] Aircraft stall speed in cm/s
 * @param rfSelect  ADS-B transponder reciever and transmit enable flags
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavionix_adsb_out_cfg_send(mavlink_channel_t chan, uint32_t ICAO, const char *callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN];
    _mav_put_uint32_t(buf, 0, ICAO);
    _mav_put_uint16_t(buf, 4, stallSpeed);
    _mav_put_uint8_t(buf, 15, emitterType);
    _mav_put_uint8_t(buf, 16, aircraftSize);
    _mav_put_uint8_t(buf, 17, gpsOffsetLat);
    _mav_put_uint8_t(buf, 18, gpsOffsetLon);
    _mav_put_uint8_t(buf, 19, rfSelect);
    _mav_put_char_array(buf, 6, callsign, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC);
#else
    mavlink_uavionix_adsb_out_cfg_t packet;
    packet.ICAO = ICAO;
    packet.stallSpeed = stallSpeed;
    packet.emitterType = emitterType;
    packet.aircraftSize = aircraftSize;
    packet.gpsOffsetLat = gpsOffsetLat;
    packet.gpsOffsetLon = gpsOffsetLon;
    packet.rfSelect = rfSelect;
    mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG, (const char *)&packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC);
#endif
}

/**
 * @brief Send a uavionix_adsb_out_cfg message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavionix_adsb_out_cfg_send_struct(mavlink_channel_t chan, const mavlink_uavionix_adsb_out_cfg_t* uavionix_adsb_out_cfg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_out_cfg_send(chan, uavionix_adsb_out_cfg->ICAO, uavionix_adsb_out_cfg->callsign, uavionix_adsb_out_cfg->emitterType, uavionix_adsb_out_cfg->aircraftSize, uavionix_adsb_out_cfg->gpsOffsetLat, uavionix_adsb_out_cfg->gpsOffsetLon, uavionix_adsb_out_cfg->stallSpeed, uavionix_adsb_out_cfg->rfSelect);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG, (const char *)uavionix_adsb_out_cfg, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavionix_adsb_out_cfg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t ICAO, const char *callsign, uint8_t emitterType, uint8_t aircraftSize, uint8_t gpsOffsetLat, uint8_t gpsOffsetLon, uint16_t stallSpeed, uint8_t rfSelect)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, ICAO);
    _mav_put_uint16_t(buf, 4, stallSpeed);
    _mav_put_uint8_t(buf, 15, emitterType);
    _mav_put_uint8_t(buf, 16, aircraftSize);
    _mav_put_uint8_t(buf, 17, gpsOffsetLat);
    _mav_put_uint8_t(buf, 18, gpsOffsetLon);
    _mav_put_uint8_t(buf, 19, rfSelect);
    _mav_put_char_array(buf, 6, callsign, 9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC);
#else
    mavlink_uavionix_adsb_out_cfg_t *packet = (mavlink_uavionix_adsb_out_cfg_t *)msgbuf;
    packet->ICAO = ICAO;
    packet->stallSpeed = stallSpeed;
    packet->emitterType = emitterType;
    packet->aircraftSize = aircraftSize;
    packet->gpsOffsetLat = gpsOffsetLat;
    packet->gpsOffsetLon = gpsOffsetLon;
    packet->rfSelect = rfSelect;
    mav_array_memcpy(packet->callsign, callsign, sizeof(char)*9);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG, (const char *)packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVIONIX_ADSB_OUT_CFG UNPACKING


/**
 * @brief Get field ICAO from uavionix_adsb_out_cfg message
 *
 * @return  Vehicle address (24 bit)
 */
static inline uint32_t mavlink_msg_uavionix_adsb_out_cfg_get_ICAO(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field callsign from uavionix_adsb_out_cfg message
 *
 * @return  Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_get_callsign(const mavlink_message_t* msg, char *callsign)
{
    return _MAV_RETURN_char_array(msg, callsign, 9,  6);
}

/**
 * @brief Get field emitterType from uavionix_adsb_out_cfg message
 *
 * @return  Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_cfg_get_emitterType(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field aircraftSize from uavionix_adsb_out_cfg message
 *
 * @return  Aircraft length and width encoding (table 2-35 of DO-282B)
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_cfg_get_aircraftSize(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field gpsOffsetLat from uavionix_adsb_out_cfg message
 *
 * @return  GPS antenna lateral offset (table 2-36 of DO-282B)
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_cfg_get_gpsOffsetLat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field gpsOffsetLon from uavionix_adsb_out_cfg message
 *
 * @return  GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one] (table 2-37 DO-282B)
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_cfg_get_gpsOffsetLon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field stallSpeed from uavionix_adsb_out_cfg message
 *
 * @return [cm/s] Aircraft stall speed in cm/s
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_cfg_get_stallSpeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field rfSelect from uavionix_adsb_out_cfg message
 *
 * @return  ADS-B transponder reciever and transmit enable flags
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_cfg_get_rfSelect(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Decode a uavionix_adsb_out_cfg message into a struct
 *
 * @param msg The message to decode
 * @param uavionix_adsb_out_cfg C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavionix_adsb_out_cfg_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_cfg_t* uavionix_adsb_out_cfg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavionix_adsb_out_cfg->ICAO = mavlink_msg_uavionix_adsb_out_cfg_get_ICAO(msg);
    uavionix_adsb_out_cfg->stallSpeed = mavlink_msg_uavionix_adsb_out_cfg_get_stallSpeed(msg);
    mavlink_msg_uavionix_adsb_out_cfg_get_callsign(msg, uavionix_adsb_out_cfg->callsign);
    uavionix_adsb_out_cfg->emitterType = mavlink_msg_uavionix_adsb_out_cfg_get_emitterType(msg);
    uavionix_adsb_out_cfg->aircraftSize = mavlink_msg_uavionix_adsb_out_cfg_get_aircraftSize(msg);
    uavionix_adsb_out_cfg->gpsOffsetLat = mavlink_msg_uavionix_adsb_out_cfg_get_gpsOffsetLat(msg);
    uavionix_adsb_out_cfg->gpsOffsetLon = mavlink_msg_uavionix_adsb_out_cfg_get_gpsOffsetLon(msg);
    uavionix_adsb_out_cfg->rfSelect = mavlink_msg_uavionix_adsb_out_cfg_get_rfSelect(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN? msg->len : MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN;
        memset(uavionix_adsb_out_cfg, 0, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_LEN);
    memcpy(uavionix_adsb_out_cfg, _MAV_PAYLOAD(msg), len);
#endif
}
