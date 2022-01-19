#pragma once
// MESSAGE UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT PACKING

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT 10003


typedef struct __mavlink_uavionix_adsb_transceiver_health_report_t {
 uint8_t rfHealth; /*<  ADS-B transponder messages*/
} mavlink_uavionix_adsb_transceiver_health_report_t;

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN 1
#define MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN 1
#define MAVLINK_MSG_ID_10003_LEN 1
#define MAVLINK_MSG_ID_10003_MIN_LEN 1

#define MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRC 4
#define MAVLINK_MSG_ID_10003_CRC 4



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT { \
    10003, \
    "UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT", \
    1, \
    {  { "rfHealth", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_uavionix_adsb_transceiver_health_report_t, rfHealth) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT { \
    "UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT", \
    1, \
    {  { "rfHealth", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_uavionix_adsb_transceiver_health_report_t, rfHealth) }, \
         } \
}
#endif

/**
 * @brief Pack a uavionix_adsb_transceiver_health_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rfHealth  ADS-B transponder messages
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_transceiver_health_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t rfHealth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN];
    _mav_put_uint8_t(buf, 0, rfHealth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN);
#else
    mavlink_uavionix_adsb_transceiver_health_report_t packet;
    packet.rfHealth = rfHealth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRC);
}

/**
 * @brief Pack a uavionix_adsb_transceiver_health_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rfHealth  ADS-B transponder messages
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uavionix_adsb_transceiver_health_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t rfHealth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN];
    _mav_put_uint8_t(buf, 0, rfHealth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN);
#else
    mavlink_uavionix_adsb_transceiver_health_report_t packet;
    packet.rfHealth = rfHealth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRC);
}

/**
 * @brief Encode a uavionix_adsb_transceiver_health_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_transceiver_health_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_transceiver_health_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uavionix_adsb_transceiver_health_report_t* uavionix_adsb_transceiver_health_report)
{
    return mavlink_msg_uavionix_adsb_transceiver_health_report_pack(system_id, component_id, msg, uavionix_adsb_transceiver_health_report->rfHealth);
}

/**
 * @brief Encode a uavionix_adsb_transceiver_health_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uavionix_adsb_transceiver_health_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uavionix_adsb_transceiver_health_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uavionix_adsb_transceiver_health_report_t* uavionix_adsb_transceiver_health_report)
{
    return mavlink_msg_uavionix_adsb_transceiver_health_report_pack_chan(system_id, component_id, chan, msg, uavionix_adsb_transceiver_health_report->rfHealth);
}

/**
 * @brief Send a uavionix_adsb_transceiver_health_report message
 * @param chan MAVLink channel to send the message
 *
 * @param rfHealth  ADS-B transponder messages
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uavionix_adsb_transceiver_health_report_send(mavlink_channel_t chan, uint8_t rfHealth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN];
    _mav_put_uint8_t(buf, 0, rfHealth);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRC);
#else
    mavlink_uavionix_adsb_transceiver_health_report_t packet;
    packet.rfHealth = rfHealth;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT, (const char *)&packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRC);
#endif
}

/**
 * @brief Send a uavionix_adsb_transceiver_health_report message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_uavionix_adsb_transceiver_health_report_send_struct(mavlink_channel_t chan, const mavlink_uavionix_adsb_transceiver_health_report_t* uavionix_adsb_transceiver_health_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_uavionix_adsb_transceiver_health_report_send(chan, uavionix_adsb_transceiver_health_report->rfHealth);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT, (const char *)uavionix_adsb_transceiver_health_report, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRC);
#endif
}

#if MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_uavionix_adsb_transceiver_health_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t rfHealth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, rfHealth);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT, buf, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRC);
#else
    mavlink_uavionix_adsb_transceiver_health_report_t *packet = (mavlink_uavionix_adsb_transceiver_health_report_t *)msgbuf;
    packet->rfHealth = rfHealth;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT, (const char *)packet, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_CRC);
#endif
}
#endif

#endif

// MESSAGE UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT UNPACKING


/**
 * @brief Get field rfHealth from uavionix_adsb_transceiver_health_report message
 *
 * @return  ADS-B transponder messages
 */
static inline uint8_t mavlink_msg_uavionix_adsb_transceiver_health_report_get_rfHealth(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a uavionix_adsb_transceiver_health_report message into a struct
 *
 * @param msg The message to decode
 * @param uavionix_adsb_transceiver_health_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_uavionix_adsb_transceiver_health_report_decode(const mavlink_message_t* msg, mavlink_uavionix_adsb_transceiver_health_report_t* uavionix_adsb_transceiver_health_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    uavionix_adsb_transceiver_health_report->rfHealth = mavlink_msg_uavionix_adsb_transceiver_health_report_get_rfHealth(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN? msg->len : MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN;
        memset(uavionix_adsb_transceiver_health_report, 0, MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_LEN);
    memcpy(uavionix_adsb_transceiver_health_report, _MAV_PAYLOAD(msg), len);
#endif
}
