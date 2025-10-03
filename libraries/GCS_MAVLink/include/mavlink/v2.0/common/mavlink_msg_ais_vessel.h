#pragma once
// MESSAGE AIS_VESSEL PACKING

#define MAVLINK_MSG_ID_AIS_VESSEL 301


typedef struct __mavlink_ais_vessel_t {
 uint32_t MMSI; /*<  Mobile Marine Service Identifier, 9 decimal digits*/
 int32_t lat; /*< [degE7] Latitude*/
 int32_t lon; /*< [degE7] Longitude*/
 uint16_t COG; /*< [cdeg] Course over ground*/
 uint16_t heading; /*< [cdeg] True heading*/
 uint16_t velocity; /*< [cm/s] Speed over ground*/
 uint16_t dimension_bow; /*< [m] Distance from lat/lon location to bow*/
 uint16_t dimension_stern; /*< [m] Distance from lat/lon location to stern*/
 uint16_t tslc; /*< [s] Time since last communication in seconds*/
 uint16_t flags; /*<  Bitmask to indicate various statuses including valid data fields*/
 int8_t turn_rate; /*< [cdeg/s] Turn rate*/
 uint8_t navigational_status; /*<  Navigational status*/
 uint8_t type; /*<  Type of vessels*/
 uint8_t dimension_port; /*< [m] Distance from lat/lon location to port side*/
 uint8_t dimension_starboard; /*< [m] Distance from lat/lon location to starboard side*/
 char callsign[7]; /*<  The vessel callsign*/
 char name[20]; /*<  The vessel name*/
} mavlink_ais_vessel_t;

#define MAVLINK_MSG_ID_AIS_VESSEL_LEN 58
#define MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN 58
#define MAVLINK_MSG_ID_301_LEN 58
#define MAVLINK_MSG_ID_301_MIN_LEN 58

#define MAVLINK_MSG_ID_AIS_VESSEL_CRC 243
#define MAVLINK_MSG_ID_301_CRC 243

#define MAVLINK_MSG_AIS_VESSEL_FIELD_CALLSIGN_LEN 7
#define MAVLINK_MSG_AIS_VESSEL_FIELD_NAME_LEN 20

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIS_VESSEL { \
    301, \
    "AIS_VESSEL", \
    17, \
    {  { "MMSI", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ais_vessel_t, MMSI) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_ais_vessel_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_ais_vessel_t, lon) }, \
         { "COG", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_ais_vessel_t, COG) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_ais_vessel_t, heading) }, \
         { "velocity", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_ais_vessel_t, velocity) }, \
         { "turn_rate", NULL, MAVLINK_TYPE_INT8_T, 0, 26, offsetof(mavlink_ais_vessel_t, turn_rate) }, \
         { "navigational_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_ais_vessel_t, navigational_status) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_ais_vessel_t, type) }, \
         { "dimension_bow", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_ais_vessel_t, dimension_bow) }, \
         { "dimension_stern", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_ais_vessel_t, dimension_stern) }, \
         { "dimension_port", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_ais_vessel_t, dimension_port) }, \
         { "dimension_starboard", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_ais_vessel_t, dimension_starboard) }, \
         { "callsign", NULL, MAVLINK_TYPE_CHAR, 7, 31, offsetof(mavlink_ais_vessel_t, callsign) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 20, 38, offsetof(mavlink_ais_vessel_t, name) }, \
         { "tslc", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_ais_vessel_t, tslc) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_ais_vessel_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIS_VESSEL { \
    "AIS_VESSEL", \
    17, \
    {  { "MMSI", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ais_vessel_t, MMSI) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_ais_vessel_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_ais_vessel_t, lon) }, \
         { "COG", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_ais_vessel_t, COG) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_ais_vessel_t, heading) }, \
         { "velocity", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_ais_vessel_t, velocity) }, \
         { "turn_rate", NULL, MAVLINK_TYPE_INT8_T, 0, 26, offsetof(mavlink_ais_vessel_t, turn_rate) }, \
         { "navigational_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_ais_vessel_t, navigational_status) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_ais_vessel_t, type) }, \
         { "dimension_bow", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_ais_vessel_t, dimension_bow) }, \
         { "dimension_stern", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_ais_vessel_t, dimension_stern) }, \
         { "dimension_port", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_ais_vessel_t, dimension_port) }, \
         { "dimension_starboard", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_ais_vessel_t, dimension_starboard) }, \
         { "callsign", NULL, MAVLINK_TYPE_CHAR, 7, 31, offsetof(mavlink_ais_vessel_t, callsign) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 20, 38, offsetof(mavlink_ais_vessel_t, name) }, \
         { "tslc", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_ais_vessel_t, tslc) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_ais_vessel_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a ais_vessel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param MMSI  Mobile Marine Service Identifier, 9 decimal digits
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param COG [cdeg] Course over ground
 * @param heading [cdeg] True heading
 * @param velocity [cm/s] Speed over ground
 * @param turn_rate [cdeg/s] Turn rate
 * @param navigational_status  Navigational status
 * @param type  Type of vessels
 * @param dimension_bow [m] Distance from lat/lon location to bow
 * @param dimension_stern [m] Distance from lat/lon location to stern
 * @param dimension_port [m] Distance from lat/lon location to port side
 * @param dimension_starboard [m] Distance from lat/lon location to starboard side
 * @param callsign  The vessel callsign
 * @param name  The vessel name
 * @param tslc [s] Time since last communication in seconds
 * @param flags  Bitmask to indicate various statuses including valid data fields
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ais_vessel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char *callsign, const char *name, uint16_t tslc, uint16_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIS_VESSEL_LEN];
    _mav_put_uint32_t(buf, 0, MMSI);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_uint16_t(buf, 12, COG);
    _mav_put_uint16_t(buf, 14, heading);
    _mav_put_uint16_t(buf, 16, velocity);
    _mav_put_uint16_t(buf, 18, dimension_bow);
    _mav_put_uint16_t(buf, 20, dimension_stern);
    _mav_put_uint16_t(buf, 22, tslc);
    _mav_put_uint16_t(buf, 24, flags);
    _mav_put_int8_t(buf, 26, turn_rate);
    _mav_put_uint8_t(buf, 27, navigational_status);
    _mav_put_uint8_t(buf, 28, type);
    _mav_put_uint8_t(buf, 29, dimension_port);
    _mav_put_uint8_t(buf, 30, dimension_starboard);
    _mav_put_char_array(buf, 31, callsign, 7);
    _mav_put_char_array(buf, 38, name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIS_VESSEL_LEN);
#else
    mavlink_ais_vessel_t packet;
    packet.MMSI = MMSI;
    packet.lat = lat;
    packet.lon = lon;
    packet.COG = COG;
    packet.heading = heading;
    packet.velocity = velocity;
    packet.dimension_bow = dimension_bow;
    packet.dimension_stern = dimension_stern;
    packet.tslc = tslc;
    packet.flags = flags;
    packet.turn_rate = turn_rate;
    packet.navigational_status = navigational_status;
    packet.type = type;
    packet.dimension_port = dimension_port;
    packet.dimension_starboard = dimension_starboard;
    mav_array_memcpy(packet.callsign, callsign, sizeof(char)*7);
    mav_array_memcpy(packet.name, name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIS_VESSEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIS_VESSEL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN, MAVLINK_MSG_ID_AIS_VESSEL_LEN, MAVLINK_MSG_ID_AIS_VESSEL_CRC);
}

/**
 * @brief Pack a ais_vessel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param MMSI  Mobile Marine Service Identifier, 9 decimal digits
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param COG [cdeg] Course over ground
 * @param heading [cdeg] True heading
 * @param velocity [cm/s] Speed over ground
 * @param turn_rate [cdeg/s] Turn rate
 * @param navigational_status  Navigational status
 * @param type  Type of vessels
 * @param dimension_bow [m] Distance from lat/lon location to bow
 * @param dimension_stern [m] Distance from lat/lon location to stern
 * @param dimension_port [m] Distance from lat/lon location to port side
 * @param dimension_starboard [m] Distance from lat/lon location to starboard side
 * @param callsign  The vessel callsign
 * @param name  The vessel name
 * @param tslc [s] Time since last communication in seconds
 * @param flags  Bitmask to indicate various statuses including valid data fields
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ais_vessel_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char *callsign, const char *name, uint16_t tslc, uint16_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIS_VESSEL_LEN];
    _mav_put_uint32_t(buf, 0, MMSI);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_uint16_t(buf, 12, COG);
    _mav_put_uint16_t(buf, 14, heading);
    _mav_put_uint16_t(buf, 16, velocity);
    _mav_put_uint16_t(buf, 18, dimension_bow);
    _mav_put_uint16_t(buf, 20, dimension_stern);
    _mav_put_uint16_t(buf, 22, tslc);
    _mav_put_uint16_t(buf, 24, flags);
    _mav_put_int8_t(buf, 26, turn_rate);
    _mav_put_uint8_t(buf, 27, navigational_status);
    _mav_put_uint8_t(buf, 28, type);
    _mav_put_uint8_t(buf, 29, dimension_port);
    _mav_put_uint8_t(buf, 30, dimension_starboard);
    _mav_put_char_array(buf, 31, callsign, 7);
    _mav_put_char_array(buf, 38, name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIS_VESSEL_LEN);
#else
    mavlink_ais_vessel_t packet;
    packet.MMSI = MMSI;
    packet.lat = lat;
    packet.lon = lon;
    packet.COG = COG;
    packet.heading = heading;
    packet.velocity = velocity;
    packet.dimension_bow = dimension_bow;
    packet.dimension_stern = dimension_stern;
    packet.tslc = tslc;
    packet.flags = flags;
    packet.turn_rate = turn_rate;
    packet.navigational_status = navigational_status;
    packet.type = type;
    packet.dimension_port = dimension_port;
    packet.dimension_starboard = dimension_starboard;
    mav_array_memcpy(packet.callsign, callsign, sizeof(char)*7);
    mav_array_memcpy(packet.name, name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIS_VESSEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIS_VESSEL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN, MAVLINK_MSG_ID_AIS_VESSEL_LEN, MAVLINK_MSG_ID_AIS_VESSEL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN, MAVLINK_MSG_ID_AIS_VESSEL_LEN);
#endif
}

/**
 * @brief Pack a ais_vessel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param MMSI  Mobile Marine Service Identifier, 9 decimal digits
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param COG [cdeg] Course over ground
 * @param heading [cdeg] True heading
 * @param velocity [cm/s] Speed over ground
 * @param turn_rate [cdeg/s] Turn rate
 * @param navigational_status  Navigational status
 * @param type  Type of vessels
 * @param dimension_bow [m] Distance from lat/lon location to bow
 * @param dimension_stern [m] Distance from lat/lon location to stern
 * @param dimension_port [m] Distance from lat/lon location to port side
 * @param dimension_starboard [m] Distance from lat/lon location to starboard side
 * @param callsign  The vessel callsign
 * @param name  The vessel name
 * @param tslc [s] Time since last communication in seconds
 * @param flags  Bitmask to indicate various statuses including valid data fields
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ais_vessel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t MMSI,int32_t lat,int32_t lon,uint16_t COG,uint16_t heading,uint16_t velocity,int8_t turn_rate,uint8_t navigational_status,uint8_t type,uint16_t dimension_bow,uint16_t dimension_stern,uint8_t dimension_port,uint8_t dimension_starboard,const char *callsign,const char *name,uint16_t tslc,uint16_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIS_VESSEL_LEN];
    _mav_put_uint32_t(buf, 0, MMSI);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_uint16_t(buf, 12, COG);
    _mav_put_uint16_t(buf, 14, heading);
    _mav_put_uint16_t(buf, 16, velocity);
    _mav_put_uint16_t(buf, 18, dimension_bow);
    _mav_put_uint16_t(buf, 20, dimension_stern);
    _mav_put_uint16_t(buf, 22, tslc);
    _mav_put_uint16_t(buf, 24, flags);
    _mav_put_int8_t(buf, 26, turn_rate);
    _mav_put_uint8_t(buf, 27, navigational_status);
    _mav_put_uint8_t(buf, 28, type);
    _mav_put_uint8_t(buf, 29, dimension_port);
    _mav_put_uint8_t(buf, 30, dimension_starboard);
    _mav_put_char_array(buf, 31, callsign, 7);
    _mav_put_char_array(buf, 38, name, 20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIS_VESSEL_LEN);
#else
    mavlink_ais_vessel_t packet;
    packet.MMSI = MMSI;
    packet.lat = lat;
    packet.lon = lon;
    packet.COG = COG;
    packet.heading = heading;
    packet.velocity = velocity;
    packet.dimension_bow = dimension_bow;
    packet.dimension_stern = dimension_stern;
    packet.tslc = tslc;
    packet.flags = flags;
    packet.turn_rate = turn_rate;
    packet.navigational_status = navigational_status;
    packet.type = type;
    packet.dimension_port = dimension_port;
    packet.dimension_starboard = dimension_starboard;
    mav_array_memcpy(packet.callsign, callsign, sizeof(char)*7);
    mav_array_memcpy(packet.name, name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIS_VESSEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIS_VESSEL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN, MAVLINK_MSG_ID_AIS_VESSEL_LEN, MAVLINK_MSG_ID_AIS_VESSEL_CRC);
}

/**
 * @brief Encode a ais_vessel struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ais_vessel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ais_vessel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ais_vessel_t* ais_vessel)
{
    return mavlink_msg_ais_vessel_pack(system_id, component_id, msg, ais_vessel->MMSI, ais_vessel->lat, ais_vessel->lon, ais_vessel->COG, ais_vessel->heading, ais_vessel->velocity, ais_vessel->turn_rate, ais_vessel->navigational_status, ais_vessel->type, ais_vessel->dimension_bow, ais_vessel->dimension_stern, ais_vessel->dimension_port, ais_vessel->dimension_starboard, ais_vessel->callsign, ais_vessel->name, ais_vessel->tslc, ais_vessel->flags);
}

/**
 * @brief Encode a ais_vessel struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ais_vessel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ais_vessel_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ais_vessel_t* ais_vessel)
{
    return mavlink_msg_ais_vessel_pack_chan(system_id, component_id, chan, msg, ais_vessel->MMSI, ais_vessel->lat, ais_vessel->lon, ais_vessel->COG, ais_vessel->heading, ais_vessel->velocity, ais_vessel->turn_rate, ais_vessel->navigational_status, ais_vessel->type, ais_vessel->dimension_bow, ais_vessel->dimension_stern, ais_vessel->dimension_port, ais_vessel->dimension_starboard, ais_vessel->callsign, ais_vessel->name, ais_vessel->tslc, ais_vessel->flags);
}

/**
 * @brief Encode a ais_vessel struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ais_vessel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ais_vessel_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ais_vessel_t* ais_vessel)
{
    return mavlink_msg_ais_vessel_pack_status(system_id, component_id, _status, msg,  ais_vessel->MMSI, ais_vessel->lat, ais_vessel->lon, ais_vessel->COG, ais_vessel->heading, ais_vessel->velocity, ais_vessel->turn_rate, ais_vessel->navigational_status, ais_vessel->type, ais_vessel->dimension_bow, ais_vessel->dimension_stern, ais_vessel->dimension_port, ais_vessel->dimension_starboard, ais_vessel->callsign, ais_vessel->name, ais_vessel->tslc, ais_vessel->flags);
}

/**
 * @brief Send a ais_vessel message
 * @param chan MAVLink channel to send the message
 *
 * @param MMSI  Mobile Marine Service Identifier, 9 decimal digits
 * @param lat [degE7] Latitude
 * @param lon [degE7] Longitude
 * @param COG [cdeg] Course over ground
 * @param heading [cdeg] True heading
 * @param velocity [cm/s] Speed over ground
 * @param turn_rate [cdeg/s] Turn rate
 * @param navigational_status  Navigational status
 * @param type  Type of vessels
 * @param dimension_bow [m] Distance from lat/lon location to bow
 * @param dimension_stern [m] Distance from lat/lon location to stern
 * @param dimension_port [m] Distance from lat/lon location to port side
 * @param dimension_starboard [m] Distance from lat/lon location to starboard side
 * @param callsign  The vessel callsign
 * @param name  The vessel name
 * @param tslc [s] Time since last communication in seconds
 * @param flags  Bitmask to indicate various statuses including valid data fields
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ais_vessel_send(mavlink_channel_t chan, uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char *callsign, const char *name, uint16_t tslc, uint16_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIS_VESSEL_LEN];
    _mav_put_uint32_t(buf, 0, MMSI);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_uint16_t(buf, 12, COG);
    _mav_put_uint16_t(buf, 14, heading);
    _mav_put_uint16_t(buf, 16, velocity);
    _mav_put_uint16_t(buf, 18, dimension_bow);
    _mav_put_uint16_t(buf, 20, dimension_stern);
    _mav_put_uint16_t(buf, 22, tslc);
    _mav_put_uint16_t(buf, 24, flags);
    _mav_put_int8_t(buf, 26, turn_rate);
    _mav_put_uint8_t(buf, 27, navigational_status);
    _mav_put_uint8_t(buf, 28, type);
    _mav_put_uint8_t(buf, 29, dimension_port);
    _mav_put_uint8_t(buf, 30, dimension_starboard);
    _mav_put_char_array(buf, 31, callsign, 7);
    _mav_put_char_array(buf, 38, name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIS_VESSEL, buf, MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN, MAVLINK_MSG_ID_AIS_VESSEL_LEN, MAVLINK_MSG_ID_AIS_VESSEL_CRC);
#else
    mavlink_ais_vessel_t packet;
    packet.MMSI = MMSI;
    packet.lat = lat;
    packet.lon = lon;
    packet.COG = COG;
    packet.heading = heading;
    packet.velocity = velocity;
    packet.dimension_bow = dimension_bow;
    packet.dimension_stern = dimension_stern;
    packet.tslc = tslc;
    packet.flags = flags;
    packet.turn_rate = turn_rate;
    packet.navigational_status = navigational_status;
    packet.type = type;
    packet.dimension_port = dimension_port;
    packet.dimension_starboard = dimension_starboard;
    mav_array_memcpy(packet.callsign, callsign, sizeof(char)*7);
    mav_array_memcpy(packet.name, name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIS_VESSEL, (const char *)&packet, MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN, MAVLINK_MSG_ID_AIS_VESSEL_LEN, MAVLINK_MSG_ID_AIS_VESSEL_CRC);
#endif
}

/**
 * @brief Send a ais_vessel message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ais_vessel_send_struct(mavlink_channel_t chan, const mavlink_ais_vessel_t* ais_vessel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ais_vessel_send(chan, ais_vessel->MMSI, ais_vessel->lat, ais_vessel->lon, ais_vessel->COG, ais_vessel->heading, ais_vessel->velocity, ais_vessel->turn_rate, ais_vessel->navigational_status, ais_vessel->type, ais_vessel->dimension_bow, ais_vessel->dimension_stern, ais_vessel->dimension_port, ais_vessel->dimension_starboard, ais_vessel->callsign, ais_vessel->name, ais_vessel->tslc, ais_vessel->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIS_VESSEL, (const char *)ais_vessel, MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN, MAVLINK_MSG_ID_AIS_VESSEL_LEN, MAVLINK_MSG_ID_AIS_VESSEL_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIS_VESSEL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ais_vessel_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t MMSI, int32_t lat, int32_t lon, uint16_t COG, uint16_t heading, uint16_t velocity, int8_t turn_rate, uint8_t navigational_status, uint8_t type, uint16_t dimension_bow, uint16_t dimension_stern, uint8_t dimension_port, uint8_t dimension_starboard, const char *callsign, const char *name, uint16_t tslc, uint16_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, MMSI);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_uint16_t(buf, 12, COG);
    _mav_put_uint16_t(buf, 14, heading);
    _mav_put_uint16_t(buf, 16, velocity);
    _mav_put_uint16_t(buf, 18, dimension_bow);
    _mav_put_uint16_t(buf, 20, dimension_stern);
    _mav_put_uint16_t(buf, 22, tslc);
    _mav_put_uint16_t(buf, 24, flags);
    _mav_put_int8_t(buf, 26, turn_rate);
    _mav_put_uint8_t(buf, 27, navigational_status);
    _mav_put_uint8_t(buf, 28, type);
    _mav_put_uint8_t(buf, 29, dimension_port);
    _mav_put_uint8_t(buf, 30, dimension_starboard);
    _mav_put_char_array(buf, 31, callsign, 7);
    _mav_put_char_array(buf, 38, name, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIS_VESSEL, buf, MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN, MAVLINK_MSG_ID_AIS_VESSEL_LEN, MAVLINK_MSG_ID_AIS_VESSEL_CRC);
#else
    mavlink_ais_vessel_t *packet = (mavlink_ais_vessel_t *)msgbuf;
    packet->MMSI = MMSI;
    packet->lat = lat;
    packet->lon = lon;
    packet->COG = COG;
    packet->heading = heading;
    packet->velocity = velocity;
    packet->dimension_bow = dimension_bow;
    packet->dimension_stern = dimension_stern;
    packet->tslc = tslc;
    packet->flags = flags;
    packet->turn_rate = turn_rate;
    packet->navigational_status = navigational_status;
    packet->type = type;
    packet->dimension_port = dimension_port;
    packet->dimension_starboard = dimension_starboard;
    mav_array_memcpy(packet->callsign, callsign, sizeof(char)*7);
    mav_array_memcpy(packet->name, name, sizeof(char)*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIS_VESSEL, (const char *)packet, MAVLINK_MSG_ID_AIS_VESSEL_MIN_LEN, MAVLINK_MSG_ID_AIS_VESSEL_LEN, MAVLINK_MSG_ID_AIS_VESSEL_CRC);
#endif
}
#endif

#endif

// MESSAGE AIS_VESSEL UNPACKING


/**
 * @brief Get field MMSI from ais_vessel message
 *
 * @return  Mobile Marine Service Identifier, 9 decimal digits
 */
static inline uint32_t mavlink_msg_ais_vessel_get_MMSI(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from ais_vessel message
 *
 * @return [degE7] Latitude
 */
static inline int32_t mavlink_msg_ais_vessel_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from ais_vessel message
 *
 * @return [degE7] Longitude
 */
static inline int32_t mavlink_msg_ais_vessel_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field COG from ais_vessel message
 *
 * @return [cdeg] Course over ground
 */
static inline uint16_t mavlink_msg_ais_vessel_get_COG(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field heading from ais_vessel message
 *
 * @return [cdeg] True heading
 */
static inline uint16_t mavlink_msg_ais_vessel_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field velocity from ais_vessel message
 *
 * @return [cm/s] Speed over ground
 */
static inline uint16_t mavlink_msg_ais_vessel_get_velocity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field turn_rate from ais_vessel message
 *
 * @return [cdeg/s] Turn rate
 */
static inline int8_t mavlink_msg_ais_vessel_get_turn_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  26);
}

/**
 * @brief Get field navigational_status from ais_vessel message
 *
 * @return  Navigational status
 */
static inline uint8_t mavlink_msg_ais_vessel_get_navigational_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field type from ais_vessel message
 *
 * @return  Type of vessels
 */
static inline uint8_t mavlink_msg_ais_vessel_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field dimension_bow from ais_vessel message
 *
 * @return [m] Distance from lat/lon location to bow
 */
static inline uint16_t mavlink_msg_ais_vessel_get_dimension_bow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field dimension_stern from ais_vessel message
 *
 * @return [m] Distance from lat/lon location to stern
 */
static inline uint16_t mavlink_msg_ais_vessel_get_dimension_stern(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field dimension_port from ais_vessel message
 *
 * @return [m] Distance from lat/lon location to port side
 */
static inline uint8_t mavlink_msg_ais_vessel_get_dimension_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field dimension_starboard from ais_vessel message
 *
 * @return [m] Distance from lat/lon location to starboard side
 */
static inline uint8_t mavlink_msg_ais_vessel_get_dimension_starboard(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field callsign from ais_vessel message
 *
 * @return  The vessel callsign
 */
static inline uint16_t mavlink_msg_ais_vessel_get_callsign(const mavlink_message_t* msg, char *callsign)
{
    return _MAV_RETURN_char_array(msg, callsign, 7,  31);
}

/**
 * @brief Get field name from ais_vessel message
 *
 * @return  The vessel name
 */
static inline uint16_t mavlink_msg_ais_vessel_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 20,  38);
}

/**
 * @brief Get field tslc from ais_vessel message
 *
 * @return [s] Time since last communication in seconds
 */
static inline uint16_t mavlink_msg_ais_vessel_get_tslc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field flags from ais_vessel message
 *
 * @return  Bitmask to indicate various statuses including valid data fields
 */
static inline uint16_t mavlink_msg_ais_vessel_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Decode a ais_vessel message into a struct
 *
 * @param msg The message to decode
 * @param ais_vessel C-struct to decode the message contents into
 */
static inline void mavlink_msg_ais_vessel_decode(const mavlink_message_t* msg, mavlink_ais_vessel_t* ais_vessel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ais_vessel->MMSI = mavlink_msg_ais_vessel_get_MMSI(msg);
    ais_vessel->lat = mavlink_msg_ais_vessel_get_lat(msg);
    ais_vessel->lon = mavlink_msg_ais_vessel_get_lon(msg);
    ais_vessel->COG = mavlink_msg_ais_vessel_get_COG(msg);
    ais_vessel->heading = mavlink_msg_ais_vessel_get_heading(msg);
    ais_vessel->velocity = mavlink_msg_ais_vessel_get_velocity(msg);
    ais_vessel->dimension_bow = mavlink_msg_ais_vessel_get_dimension_bow(msg);
    ais_vessel->dimension_stern = mavlink_msg_ais_vessel_get_dimension_stern(msg);
    ais_vessel->tslc = mavlink_msg_ais_vessel_get_tslc(msg);
    ais_vessel->flags = mavlink_msg_ais_vessel_get_flags(msg);
    ais_vessel->turn_rate = mavlink_msg_ais_vessel_get_turn_rate(msg);
    ais_vessel->navigational_status = mavlink_msg_ais_vessel_get_navigational_status(msg);
    ais_vessel->type = mavlink_msg_ais_vessel_get_type(msg);
    ais_vessel->dimension_port = mavlink_msg_ais_vessel_get_dimension_port(msg);
    ais_vessel->dimension_starboard = mavlink_msg_ais_vessel_get_dimension_starboard(msg);
    mavlink_msg_ais_vessel_get_callsign(msg, ais_vessel->callsign);
    mavlink_msg_ais_vessel_get_name(msg, ais_vessel->name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIS_VESSEL_LEN? msg->len : MAVLINK_MSG_ID_AIS_VESSEL_LEN;
        memset(ais_vessel, 0, MAVLINK_MSG_ID_AIS_VESSEL_LEN);
    memcpy(ais_vessel, _MAV_PAYLOAD(msg), len);
#endif
}
