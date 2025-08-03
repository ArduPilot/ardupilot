#pragma once
// MESSAGE UAVIONIX_ADSB_OUT_STATUS PACKING

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS 10008


typedef struct __mavlink_uavionix_adsb_out_status_t {
 uint16_t squawk; /*<  Mode A code (typically 1200 [0x04B0] for VFR)*/
 uint8_t state; /*<  ADS-B transponder status state flags*/
 uint8_t NIC_NACp; /*<  Integrity and Accuracy of traffic reported as a 4-bit value for each field (NACp 7:4, NIC 3:0) and encoded by Containment Radius (HPL) and Estimated Position Uncertainty (HFOM), respectively*/
 uint8_t boardTemp; /*<  Board temperature in C*/
 uint8_t fault; /*<  ADS-B transponder fault flags*/
 char flight_id[8]; /*<  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.*/
} mavlink_uavionix_adsb_out_status_t;

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN 14
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN 14
#define MAVLINK_MSG_ID_10008_LEN 14
#define MAVLINK_MSG_ID_10008_MIN_LEN 14

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_CRC 240
#define MAVLINK_MSG_ID_10008_CRC 240

#define MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_FIELD_FLIGHT_ID_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_STATUS { \
    10008, \
    "UAVIONIX_ADSB_OUT_STATUS", \
    6, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_uavionix_adsb_out_status_t, state) }, \
         { "squawk", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_uavionix_adsb_out_status_t, squawk) }, \
         { "NIC_NACp", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_uavionix_adsb_out_status_t, NIC_NACp) }, \
         { "boardTemp", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_uavionix_adsb_out_status_t, boardTemp) }, \
         { "fault", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_uavionix_adsb_out_status_t, fault) }, \
         { "flight_id", NULL, MAVLINK_TYPE_CHAR, 8, 6, offsetof(mavlink_uavionix_adsb_out_status_t, flight_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_OUT_STATUS { \
    "UAVIONIX_ADSB_OUT_STATUS", \
    6, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_uavionix_adsb_out_status_t, state) }, \
         { "squawk", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_uavionix_adsb_out_status_t, squawk) }, \
         { "NIC_NACp", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_uavionix_adsb_out_status_t, NIC_NACp) }, \
         { "boardTemp", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_uavionix_adsb_out_status_t, boardTemp) }, \
         { "fault", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_uavionix_adsb_out_status_t, fault) }, \
         { "flight_id", NULL, MAVLINK_TYPE_CHAR, 8, 6, offsetof(mavlink_uavionix_adsb_out_status_t, flight_id) }, \
         } \
}
#endif

/**
 * @brief Pack a uavionix_adsb_out_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  ADS-B transponder status state flags
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @param NIC_NACp  Integrity and Accuracy of traffic reported as a 4-bit value for each field (NACp 7:4, NIC 3:0) and encoded by Containment Radius (HPL) and Estimated Position Uncertainty (HFOM), respectively
 * @param boardTemp  Board temperature in C
 * @param fault  ADS-B transponder fault flags
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t state, uint16_t squawk, uint8_t NIC_NACp, uint8_t boardTemp, uint8_t fault, const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, squawk);
    _mav_put_uint8_t(buf, 2, state);
    _mav_put_uint8_t(buf, 3, NIC_NACp);
    _mav_put_uint8_t(buf, 4, boardTemp);
    _mav_put_uint8_t(buf, 5, fault);
    _mav_put_char_array(buf, 6, flight_id, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN);
#else
    mavlink_uavionix_adsb_out_status_t packet;
    packet.squawk = squawk;
    packet.state = state;
    packet.NIC_NACp = NIC_NACp;
    packet.boardTemp = boardTemp;
    packet.fault = fault;
    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_CRC);
}

/**
 * @brief Pack a uavionix_adsb_out_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  ADS-B transponder status state flags
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @param NIC_NACp  Integrity and Accuracy of traffic reported as a 4-bit value for each field (NACp 7:4, NIC 3:0) and encoded by Containment Radius (HPL) and Estimated Position Uncertainty (HFOM), respectively
 * @param boardTemp  Board temperature in C
 * @param fault  ADS-B transponder fault flags
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t state, uint16_t squawk, uint8_t NIC_NACp, uint8_t boardTemp, uint8_t fault, const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, squawk);
    _mav_put_uint8_t(buf, 2, state);
    _mav_put_uint8_t(buf, 3, NIC_NACp);
    _mav_put_uint8_t(buf, 4, boardTemp);
    _mav_put_uint8_t(buf, 5, fault);
    _mav_put_char_array(buf, 6, flight_id, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN);
#else
    mavlink_uavionix_adsb_out_status_t packet;
    packet.squawk = squawk;
    packet.state = state;
    packet.NIC_NACp = NIC_NACp;
    packet.boardTemp = boardTemp;
    packet.fault = fault;
    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a uavionix_adsb_out_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state  ADS-B transponder status state flags
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @param NIC_NACp  Integrity and Accuracy of traffic reported as a 4-bit value for each field (NACp 7:4, NIC 3:0) and encoded by Containment Radius (HPL) and Estimated Position Uncertainty (HFOM), respectively
 * @param boardTemp  Board temperature in C
 * @param fault  ADS-B transponder fault flags
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t state,uint16_t squawk,uint8_t NIC_NACp,uint8_t boardTemp,uint8_t fault,const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, squawk);
    _mav_put_uint8_t(buf, 2, state);
    _mav_put_uint8_t(buf, 3, NIC_NACp);
    _mav_put_uint8_t(buf, 4, boardTemp);
    _mav_put_uint8_t(buf, 5, fault);
    _mav_put_char_array(buf, 6, flight_id, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN);
#else
    mavlink_uavionix_adsb_out_status_t packet;
    packet.squawk = squawk;
    packet.state = state;
    packet.NIC_NACp = NIC_NACp;
    packet.boardTemp = boardTemp;
    packet.fault = fault;
    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_CRC);
}

/**
 * @brief Encode a uavionix_adsb_out_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_status_t* uavionix_adsb_out_status)
{
    return mavlink_msg_uavionix_adsb_out_status_pack(system_id, component_id, msg, uavionix_adsb_out_status->state, uavionix_adsb_out_status->squawk, uavionix_adsb_out_status->NIC_NACp, uavionix_adsb_out_status->boardTemp, uavionix_adsb_out_status->fault, uavionix_adsb_out_status->flight_id);
}

/**
 * @brief Encode a uavionix_adsb_out_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_status_t* uavionix_adsb_out_status)
{
    return mavlink_msg_uavionix_adsb_out_status_pack_chan(system_id, component_id, chan, msg, uavionix_adsb_out_status->state, uavionix_adsb_out_status->squawk, uavionix_adsb_out_status->NIC_NACp, uavionix_adsb_out_status->boardTemp, uavionix_adsb_out_status->fault, uavionix_adsb_out_status->flight_id);
}

/**
 * @brief Encode a uavionix_adsb_out_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_out_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_uavionix_adsb_out_status_t* uavionix_adsb_out_status)
{
    return mavlink_msg_uavionix_adsb_out_status_pack_status(system_id, component_id, _status, msg,  uavionix_adsb_out_status->state, uavionix_adsb_out_status->squawk, uavionix_adsb_out_status->NIC_NACp, uavionix_adsb_out_status->boardTemp, uavionix_adsb_out_status->fault, uavionix_adsb_out_status->flight_id);
}

/**
 * @brief Send a uavionix_adsb_out_status message
 * @param chan MAVLink channel to send the message
 *
 * @param state  ADS-B transponder status state flags
 * @param squawk  Mode A code (typically 1200 [0x04B0] for VFR)
 * @param NIC_NACp  Integrity and Accuracy of traffic reported as a 4-bit value for each field (NACp 7:4, NIC 3:0) and encoded by Containment Radius (HPL) and Estimated Position Uncertainty (HFOM), respectively
 * @param boardTemp  Board temperature in C
 * @param fault  ADS-B transponder fault flags
 * @param flight_id  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavionix_adsb_out_status_send(mavlink_channel_t chan, uint8_t state, uint16_t squawk, uint8_t NIC_NACp, uint8_t boardTemp, uint8_t fault, const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, squawk);
    _mav_put_uint8_t(buf, 2, state);
    _mav_put_uint8_t(buf, 3, NIC_NACp);
    _mav_put_uint8_t(buf, 4, boardTemp);
    _mav_put_uint8_t(buf, 5, fault);
    _mav_put_char_array(buf, 6, flight_id, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_CRC);
#else
    mavlink_uavionix_adsb_out_status_t packet;
    packet.squawk = squawk;
    packet.state = state;
    packet.NIC_NACp = NIC_NACp;
    packet.boardTemp = boardTemp;
    packet.fault = fault;
    mav_array_memcpy(packet.flight_id, flight_id, sizeof(char)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_CRC);
#endif
}

/**
 * @brief Send a uavionix_adsb_out_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavionix_adsb_out_status_send_struct(mavlink_channel_t chan, const mavlink_uavionix_adsb_out_status_t* uavionix_adsb_out_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_out_status_send(chan, uavionix_adsb_out_status->state, uavionix_adsb_out_status->squawk, uavionix_adsb_out_status->NIC_NACp, uavionix_adsb_out_status->boardTemp, uavionix_adsb_out_status->fault, uavionix_adsb_out_status->flight_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS, (const char *)uavionix_adsb_out_status, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavionix_adsb_out_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t state, uint16_t squawk, uint8_t NIC_NACp, uint8_t boardTemp, uint8_t fault, const char *flight_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, squawk);
    _mav_put_uint8_t(buf, 2, state);
    _mav_put_uint8_t(buf, 3, NIC_NACp);
    _mav_put_uint8_t(buf, 4, boardTemp);
    _mav_put_uint8_t(buf, 5, fault);
    _mav_put_char_array(buf, 6, flight_id, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_CRC);
#else
    mavlink_uavionix_adsb_out_status_t *packet = (mavlink_uavionix_adsb_out_status_t *)msgbuf;
    packet->squawk = squawk;
    packet->state = state;
    packet->NIC_NACp = NIC_NACp;
    packet->boardTemp = boardTemp;
    packet->fault = fault;
    mav_array_memcpy(packet->flight_id, flight_id, sizeof(char)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS, (const char *)packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVIONIX_ADSB_OUT_STATUS UNPACKING


/**
 * @brief Get field state from uavionix_adsb_out_status message
 *
 * @return  ADS-B transponder status state flags
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_status_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field squawk from uavionix_adsb_out_status message
 *
 * @return  Mode A code (typically 1200 [0x04B0] for VFR)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_status_get_squawk(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field NIC_NACp from uavionix_adsb_out_status message
 *
 * @return  Integrity and Accuracy of traffic reported as a 4-bit value for each field (NACp 7:4, NIC 3:0) and encoded by Containment Radius (HPL) and Estimated Position Uncertainty (HFOM), respectively
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_status_get_NIC_NACp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field boardTemp from uavionix_adsb_out_status message
 *
 * @return  Board temperature in C
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_status_get_boardTemp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field fault from uavionix_adsb_out_status message
 *
 * @return  ADS-B transponder fault flags
 */
static inline uint8_t mavlink_msg_uavionix_adsb_out_status_get_fault(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field flight_id from uavionix_adsb_out_status message
 *
 * @return  Flight Identification: 8 ASCII characters, '0' through '9', 'A' through 'Z' or space. Spaces (0x20) used as a trailing pad character, or when call sign is unavailable.
 */
static inline uint16_t mavlink_msg_uavionix_adsb_out_status_get_flight_id(const mavlink_message_t* msg, char *flight_id)
{
    return _MAV_RETURN_char_array(msg, flight_id, 8,  6);
}

/**
 * @brief Decode a uavionix_adsb_out_status message into a struct
 *
 * @param msg The message to decode
 * @param uavionix_adsb_out_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavionix_adsb_out_status_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_out_status_t* uavionix_adsb_out_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavionix_adsb_out_status->squawk = mavlink_msg_uavionix_adsb_out_status_get_squawk(msg);
    uavionix_adsb_out_status->state = mavlink_msg_uavionix_adsb_out_status_get_state(msg);
    uavionix_adsb_out_status->NIC_NACp = mavlink_msg_uavionix_adsb_out_status_get_NIC_NACp(msg);
    uavionix_adsb_out_status->boardTemp = mavlink_msg_uavionix_adsb_out_status_get_boardTemp(msg);
    uavionix_adsb_out_status->fault = mavlink_msg_uavionix_adsb_out_status_get_fault(msg);
    mavlink_msg_uavionix_adsb_out_status_get_flight_id(msg, uavionix_adsb_out_status->flight_id);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN;
        memset(uavionix_adsb_out_status, 0, MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_STATUS_LEN);
    memcpy(uavionix_adsb_out_status, _MAV_PAYLOAD(msg), len);
#endif
}
