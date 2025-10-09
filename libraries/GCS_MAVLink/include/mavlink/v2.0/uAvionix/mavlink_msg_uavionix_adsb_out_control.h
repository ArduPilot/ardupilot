#pragma once
// MESSAGE UAVIONIX_ADSB_OUT_CONTROL PACKING

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL 10007


typedef struct __mavlink_uavionix_adsb_out_control_t {
 int32_t baroAltMSL; /*< [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX*/
 uint16_t squawk; /*<  Mode A code (typically 1200 [0x04B0] for VFR)*/
 uint8_t state; /*<  ADS-B transponder control state flags*/
 uint8_t emergencyStatus; /*<  Emergency status*/
 char flight_id[8]; /*<  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.*/
 uint8_t x_bit; /*<  X-Bit enable (military transponders only)*/
} mavlink_uavionix_adsb_out_control_t;

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN 17
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN 17
#define MAVLINK_MSG_ID_10007_LEN 17
#define MAVLINK_MSG_ID_10007_MIN_LEN 17

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC 71
#define MAVLINK_MSG_ID_10007_CRC 71

#define MAVLINK_MSG_UAVIONIX_ADSB_OUT_CONTROL_FIELD_FLIGHT_ID_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_CONTROL { \
    10007, \
    "UAVIONIX_ADSB_OUT_CONTROL", \
    6, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_uavionix_adsb_out_control_t, state) }, \
         { "baroAltMSL", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_uavionix_adsb_out_control_t, baroAltMSL) }, \
         { "squawk", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_uavionix_adsb_out_control_t, squawk) }, \
         { "emergencyStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_uavionix_adsb_out_control_t, emergencyStatus) }, \
         { "flight_id", NULL, MAVLINK_TYPE_CHAR, 8, 8, offsetof(mavlink_uavionix_adsb_out_control_t, flight_id) }, \
         { "x_bit", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_uavionix_adsb_out_control_t, x_bit) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_CONTROL { \
    "UAVIONIX_ADSB_OUT_CONTROL", \
    6, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_uavionix_adsb_out_control_t, state) }, \
         { "baroAltMSL", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_uavionix_adsb_out_control_t, baroAltMSL) }, \
         { "squawk", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_uavionix_adsb_out_control_t, squawk) }, \
         { "emergencyStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_uavionix_adsb_out_control_t, emergencyStatus) }, \
         { "flight_id", NULL, MAVLINK_TYPE_CHAR, 8, 8, offsetof(mavlink_uavionix_adsb_out_control_t, flight_id) }, \
         { "x_bit", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_uavionix_adsb_out_control_t, x_bit) }, \
         } \
}
#endif

/**
 * @brief Pack a uavionix_adsb_out_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  ADS-B transponder control state flags
 * @param baroAltMSL [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @param emergencyStatus  Emergency status
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 * @param x_bit  X-Bit enable (military transponders only)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t state, int32_t baroAltMSL, uint16_t squawk, uint8_t emergencyStatus, const char *flight_id, uint8_t x_bit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN];
    _mav_put_int32_t(buf, 0, baroAltMSL);
    _mav_put_uint16_t(buf, 4, squawk);
    _mav_put_uint8_t(buf, 6, state);
    _mav_put_uint8_t(buf, 7, emergencyStatus);
    _mav_put_uint8_t(buf, 16, x_bit);
    _mav_put_char_array(buf, 8, flight_id, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN);
#else
    mavlink_uavionix_adsb_out_control_t packet;
    packet.baroAltMSL = baroAltMSL;
    packet.squawk = squawk;
    packet.state = state;
    packet.emergencyStatus = emergencyStatus;
    packet.x_bit = x_bit;
    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC);
}

/**
 * @brief Pack a uavionix_adsb_out_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  ADS-B transponder control state flags
 * @param baroAltMSL [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @param emergencyStatus  Emergency status
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 * @param x_bit  X-Bit enable (military transponders only)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t state, int32_t baroAltMSL, uint16_t squawk, uint8_t emergencyStatus, const char *flight_id, uint8_t x_bit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN];
    _mav_put_int32_t(buf, 0, baroAltMSL);
    _mav_put_uint16_t(buf, 4, squawk);
    _mav_put_uint8_t(buf, 6, state);
    _mav_put_uint8_t(buf, 7, emergencyStatus);
    _mav_put_uint8_t(buf, 16, x_bit);
    _mav_put_char_array(buf, 8, flight_id, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN);
#else
    mavlink_uavionix_adsb_out_control_t packet;
    packet.baroAltMSL = baroAltMSL;
    packet.squawk = squawk;
    packet.state = state;
    packet.emergencyStatus = emergencyStatus;
    packet.x_bit = x_bit;
    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a uavionix_adsb_out_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state  ADS-B transponder control state flags
 * @param baroAltMSL [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @param emergencyStatus  Emergency status
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 * @param x_bit  X-Bit enable (military transponders only)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t state,int32_t baroAltMSL,uint16_t squawk,uint8_t emergencyStatus,const char *flight_id,uint8_t x_bit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN];
    _mav_put_int32_t(buf, 0, baroAltMSL);
    _mav_put_uint16_t(buf, 4, squawk);
    _mav_put_uint8_t(buf, 6, state);
    _mav_put_uint8_t(buf, 7, emergencyStatus);
    _mav_put_uint8_t(buf, 16, x_bit);
    _mav_put_char_array(buf, 8, flight_id, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN);
#else
    mavlink_uavionix_adsb_out_control_t packet;
    packet.baroAltMSL = baroAltMSL;
    packet.squawk = squawk;
    packet.state = state;
    packet.emergencyStatus = emergencyStatus;
    packet.x_bit = x_bit;
    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC);
}

/**
 * @brief Encode a uavionix_adsb_out_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_control_t* uavionix_adsb_out_control)
{
    return mavlink_msg_uavionix_adsb_out_control_pack(system_id, component_id, msg, uavionix_adsb_out_control->state, uavionix_adsb_out_control->baroAltMSL, uavionix_adsb_out_control->squawk, uavionix_adsb_out_control->emergencyStatus, uavionix_adsb_out_control->flight_id, uavionix_adsb_out_control->x_bit);
}

/**
 * @brief Encode a uavionix_adsb_out_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_control_t* uavionix_adsb_out_control)
{
    return mavlink_msg_uavionix_adsb_out_control_pack_chan(system_id, component_id, chan, msg, uavionix_adsb_out_control->state, uavionix_adsb_out_control->baroAltMSL, uavionix_adsb_out_control->squawk, uavionix_adsb_out_control->emergencyStatus, uavionix_adsb_out_control->flight_id, uavionix_adsb_out_control->x_bit);
}

/**
 * @brief Encode a uavionix_adsb_out_control struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_control_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_control_t* uavionix_adsb_out_control)
{
    return mavlink_msg_uavionix_adsb_out_control_pack_status(system_id, component_id, _status, msg,  uavionix_adsb_out_control->state, uavionix_adsb_out_control->baroAltMSL, uavionix_adsb_out_control->squawk, uavionix_adsb_out_control->emergencyStatus, uavionix_adsb_out_control->flight_id, uavionix_adsb_out_control->x_bit);
}

/**
 * @brief Send a uavionix_adsb_out_control message
 * @param chan MAVLink channel to send the message
 *
 * @param state  ADS-B transponder control state flags
 * @param baroAltMSL [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @param emergencyStatus  Emergency status
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 * @param x_bit  X-Bit enable (military transponders only)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavionix_adsb_out_control_send(mavlink_channel_t chan, uint8_t state, int32_t baroAltMSL, uint16_t squawk, uint8_t emergencyStatus, const char *flight_id, uint8_t x_bit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN];
    _mav_put_int32_t(buf, 0, baroAltMSL);
    _mav_put_uint16_t(buf, 4, squawk);
    _mav_put_uint8_t(buf, 6, state);
    _mav_put_uint8_t(buf, 7, emergencyStatus);
    _mav_put_uint8_t(buf, 16, x_bit);
    _mav_put_char_array(buf, 8, flight_id, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC);
#else
    mavlink_uavionix_adsb_out_control_t packet;
    packet.baroAltMSL = baroAltMSL;
    packet.squawk = squawk;
    packet.state = state;
    packet.emergencyStatus = emergencyStatus;
    packet.x_bit = x_bit;
    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC);
#endif
}

/**
 * @brief Send a uavionix_adsb_out_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavionix_adsb_out_control_send_struct(mavlink_channel_t chan, const mavlink_uavionix_adsb_out_control_t* uavionix_adsb_out_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_out_control_send(chan, uavionix_adsb_out_control->state, uavionix_adsb_out_control->baroAltMSL, uavionix_adsb_out_control->squawk, uavionix_adsb_out_control->emergencyStatus, uavionix_adsb_out_control->flight_id, uavionix_adsb_out_control->x_bit);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL, (const char *)uavionix_adsb_out_control, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavionix_adsb_out_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t state, int32_t baroAltMSL, uint16_t squawk, uint8_t emergencyStatus, const char *flight_id, uint8_t x_bit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, baroAltMSL);
    _mav_put_uint16_t(buf, 4, squawk);
    _mav_put_uint8_t(buf, 6, state);
    _mav_put_uint8_t(buf, 7, emergencyStatus);
    _mav_put_uint8_t(buf, 16, x_bit);
    _mav_put_char_array(buf, 8, flight_id, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC);
#else
    mavlink_uavionix_adsb_out_control_t *packet = (mavlink_uavionix_adsb_out_control_t *)msgbuf;
    packet->baroAltMSL = baroAltMSL;
    packet->squawk = squawk;
    packet->state = state;
    packet->emergencyStatus = emergencyStatus;
    packet->x_bit = x_bit;
    mav_array_memcpy(packet->flight_id, flight_id, sizeof(char)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL, (const char *)packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVIONIX_ADSB_OUT_CONTROL UNPACKING


/**
 * @brief Get field state from uavionix_adsb_out_control message
 *
 * @return  ADS-B transponder control state flags
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_control_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field baroAltMSL from uavionix_adsb_out_control message
 *
 * @return [mbar] Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
 */
static inline int32_t mavlink_msg_uavionix_adsb_out_control_get_baroAltMSL(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field squawk from uavionix_adsb_out_control message
 *
 * @return  Mode A code (typically 1200 [0x04B0] for VFR)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_control_get_squawk(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field emergencyStatus from uavionix_adsb_out_control message
 *
 * @return  Emergency status
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_control_get_emergencyStatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field flight_id from uavionix_adsb_out_control message
 *
 * @return  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_control_get_flight_id(const mavlink_message_t* msg, char *flight_id)
{
    return _MAV_RETURN_char_array(msg, flight_id, 8,  8);
}

/**
 * @brief Get field x_bit from uavionix_adsb_out_control message
 *
 * @return  X-Bit enable (military transponders only)
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_control_get_x_bit(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a uavionix_adsb_out_control message into a struct
 *
 * @param msg The message to decode
 * @param uavionix_adsb_out_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavionix_adsb_out_control_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_control_t* uavionix_adsb_out_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavionix_adsb_out_control->baroAltMSL = mavlink_msg_uavionix_adsb_out_control_get_baroAltMSL(msg);
    uavionix_adsb_out_control->squawk = mavlink_msg_uavionix_adsb_out_control_get_squawk(msg);
    uavionix_adsb_out_control->state = mavlink_msg_uavionix_adsb_out_control_get_state(msg);
    uavionix_adsb_out_control->emergencyStatus = mavlink_msg_uavionix_adsb_out_control_get_emergencyStatus(msg);
    mavlink_msg_uavionix_adsb_out_control_get_flight_id(msg, uavionix_adsb_out_control->flight_id);
    uavionix_adsb_out_control->x_bit = mavlink_msg_uavionix_adsb_out_control_get_x_bit(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN;
        memset(uavionix_adsb_out_control, 0, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL_LEN);
    memcpy(uavionix_adsb_out_control, _MAV_PAYLOAD(msg), len);
#endif
}
