#pragma once
// MESSAGE MISSION_CURRENT PACKING

#define MAVLINK_MSG_ID_MISSION_CURRENT 42


typedef struct __mavlink_mission_current_t {
 uint16_t seq; /*<  Sequence*/
 uint16_t total; /*<  Total number of mission items on vehicle (on last item, sequence == total). If the autopilot stores its home location as part of the mission this will be excluded from the total. 0: Not supported, UINT16_MAX if no mission is present on the vehicle.*/
 uint8_t mission_state; /*<  Mission state machine state. MISSION_STATE_UNKNOWN if state reporting not supported.*/
 uint8_t mission_mode; /*<  Vehicle is in a mode that can execute mission items or suspended. 0: Unknown, 1: In mission mode, 2: Suspended (not in mission mode).*/
} mavlink_mission_current_t;

#define MAVLINK_MSG_ID_MISSION_CURRENT_LEN 6
#define MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN 2
#define MAVLINK_MSG_ID_42_LEN 6
#define MAVLINK_MSG_ID_42_MIN_LEN 2

#define MAVLINK_MSG_ID_MISSION_CURRENT_CRC 28
#define MAVLINK_MSG_ID_42_CRC 28



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_CURRENT { \
    42, \
    "MISSION_CURRENT", \
    4, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mission_current_t, seq) }, \
         { "total", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_mission_current_t, total) }, \
         { "mission_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mission_current_t, mission_state) }, \
         { "mission_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mission_current_t, mission_mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_CURRENT { \
    "MISSION_CURRENT", \
    4, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mission_current_t, seq) }, \
         { "total", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_mission_current_t, total) }, \
         { "mission_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mission_current_t, mission_state) }, \
         { "mission_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mission_current_t, mission_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  Sequence
 * @param total  Total number of mission items on vehicle (on last item, sequence == total). If the autopilot stores its home location as part of the mission this will be excluded from the total. 0: Not supported, UINT16_MAX if no mission is present on the vehicle.
 * @param mission_state  Mission state machine state. MISSION_STATE_UNKNOWN if state reporting not supported.
 * @param mission_mode  Vehicle is in a mode that can execute mission items or suspended. 0: Unknown, 1: In mission mode, 2: Suspended (not in mission mode).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_current_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t seq, uint16_t total, uint8_t mission_state, uint8_t mission_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint16_t(buf, 2, total);
    _mav_put_uint8_t(buf, 4, mission_state);
    _mav_put_uint8_t(buf, 5, mission_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#else
    mavlink_mission_current_t packet;
    packet.seq = seq;
    packet.total = total;
    packet.mission_state = mission_state;
    packet.mission_mode = mission_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CURRENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
}

/**
 * @brief Pack a mission_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  Sequence
 * @param total  Total number of mission items on vehicle (on last item, sequence == total). If the autopilot stores its home location as part of the mission this will be excluded from the total. 0: Not supported, UINT16_MAX if no mission is present on the vehicle.
 * @param mission_state  Mission state machine state. MISSION_STATE_UNKNOWN if state reporting not supported.
 * @param mission_mode  Vehicle is in a mode that can execute mission items or suspended. 0: Unknown, 1: In mission mode, 2: Suspended (not in mission mode).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_current_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t seq, uint16_t total, uint8_t mission_state, uint8_t mission_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint16_t(buf, 2, total);
    _mav_put_uint8_t(buf, 4, mission_state);
    _mav_put_uint8_t(buf, 5, mission_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#else
    mavlink_mission_current_t packet;
    packet.seq = seq;
    packet.total = total;
    packet.mission_state = mission_state;
    packet.mission_mode = mission_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CURRENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif
}

/**
 * @brief Pack a mission_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq  Sequence
 * @param total  Total number of mission items on vehicle (on last item, sequence == total). If the autopilot stores its home location as part of the mission this will be excluded from the total. 0: Not supported, UINT16_MAX if no mission is present on the vehicle.
 * @param mission_state  Mission state machine state. MISSION_STATE_UNKNOWN if state reporting not supported.
 * @param mission_mode  Vehicle is in a mode that can execute mission items or suspended. 0: Unknown, 1: In mission mode, 2: Suspended (not in mission mode).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t seq,uint16_t total,uint8_t mission_state,uint8_t mission_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint16_t(buf, 2, total);
    _mav_put_uint8_t(buf, 4, mission_state);
    _mav_put_uint8_t(buf, 5, mission_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#else
    mavlink_mission_current_t packet;
    packet.seq = seq;
    packet.total = total;
    packet.mission_state = mission_state;
    packet.mission_mode = mission_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CURRENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
}

/**
 * @brief Encode a mission_current struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current)
{
    return mavlink_msg_mission_current_pack(system_id, component_id, msg, mission_current->seq, mission_current->total, mission_current->mission_state, mission_current->mission_mode);
}

/**
 * @brief Encode a mission_current struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_current_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current)
{
    return mavlink_msg_mission_current_pack_chan(system_id, component_id, chan, msg, mission_current->seq, mission_current->total, mission_current->mission_state, mission_current->mission_mode);
}

/**
 * @brief Encode a mission_current struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mission_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_current_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current)
{
    return mavlink_msg_mission_current_pack_status(system_id, component_id, _status, msg,  mission_current->seq, mission_current->total, mission_current->mission_state, mission_current->mission_mode);
}

/**
 * @brief Send a mission_current message
 * @param chan MAVLink channel to send the message
 *
 * @param seq  Sequence
 * @param total  Total number of mission items on vehicle (on last item, sequence == total). If the autopilot stores its home location as part of the mission this will be excluded from the total. 0: Not supported, UINT16_MAX if no mission is present on the vehicle.
 * @param mission_state  Mission state machine state. MISSION_STATE_UNKNOWN if state reporting not supported.
 * @param mission_mode  Vehicle is in a mode that can execute mission items or suspended. 0: Unknown, 1: In mission mode, 2: Suspended (not in mission mode).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_current_send(mavlink_channel_t chan, uint16_t seq, uint16_t total, uint8_t mission_state, uint8_t mission_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint16_t(buf, 2, total);
    _mav_put_uint8_t(buf, 4, mission_state);
    _mav_put_uint8_t(buf, 5, mission_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, buf, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    mavlink_mission_current_t packet;
    packet.seq = seq;
    packet.total = total;
    packet.mission_state = mission_state;
    packet.mission_mode = mission_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#endif
}

/**
 * @brief Send a mission_current message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_current_send_struct(mavlink_channel_t chan, const mavlink_mission_current_t* mission_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_current_send(chan, mission_current->seq, mission_current->total, mission_current->mission_state, mission_current->mission_mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)mission_current, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_CURRENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_current_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq, uint16_t total, uint8_t mission_state, uint8_t mission_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint16_t(buf, 2, total);
    _mav_put_uint8_t(buf, 4, mission_state);
    _mav_put_uint8_t(buf, 5, mission_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, buf, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    mavlink_mission_current_t *packet = (mavlink_mission_current_t *)msgbuf;
    packet->seq = seq;
    packet->total = total;
    packet->mission_state = mission_state;
    packet->mission_mode = mission_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)packet, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_CURRENT UNPACKING


/**
 * @brief Get field seq from mission_current message
 *
 * @return  Sequence
 */
static inline uint16_t mavlink_msg_mission_current_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field total from mission_current message
 *
 * @return  Total number of mission items on vehicle (on last item, sequence == total). If the autopilot stores its home location as part of the mission this will be excluded from the total. 0: Not supported, UINT16_MAX if no mission is present on the vehicle.
 */
static inline uint16_t mavlink_msg_mission_current_get_total(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field mission_state from mission_current message
 *
 * @return  Mission state machine state. MISSION_STATE_UNKNOWN if state reporting not supported.
 */
static inline uint8_t mavlink_msg_mission_current_get_mission_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field mission_mode from mission_current message
 *
 * @return  Vehicle is in a mode that can execute mission items or suspended. 0: Unknown, 1: In mission mode, 2: Suspended (not in mission mode).
 */
static inline uint8_t mavlink_msg_mission_current_get_mission_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a mission_current message into a struct
 *
 * @param msg The message to decode
 * @param mission_current C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_current_decode(const mavlink_message_t* msg, mavlink_mission_current_t* mission_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_current->seq = mavlink_msg_mission_current_get_seq(msg);
    mission_current->total = mavlink_msg_mission_current_get_total(msg);
    mission_current->mission_state = mavlink_msg_mission_current_get_mission_state(msg);
    mission_current->mission_mode = mavlink_msg_mission_current_get_mission_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_CURRENT_LEN? msg->len : MAVLINK_MSG_ID_MISSION_CURRENT_LEN;
        memset(mission_current, 0, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
    memcpy(mission_current, _MAV_PAYLOAD(msg), len);
#endif
}
